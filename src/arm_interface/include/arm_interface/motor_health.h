#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "arm_interface/motor_status.h"

namespace arm_interface {

/** One reading of a motor's health registers (Dynamixel Protocol 2.0, MX/X series). */
struct MotorSample {
  bool valid = false;            ///< false = the read failed or has not happened yet
  uint8_t hardware_error = 0;    ///< Hardware Error Status register (address 70)
  bool alert = false;            ///< Hardware Alert bit (0x80) of the status packet ERR byte
  int16_t load_raw = 0;          ///< Present Load, signed, 0.1 % of max torque (MX) per count
  uint16_t voltage_raw = 0;      ///< Present Input Voltage, 0.1 V per count
  uint8_t temperature_c = 0;     ///< Present Temperature [C]
};

/**
 * Warning thresholds. All are heuristic starting values, not derived from the
 * motor datasheets; tune from measured data.
 */
struct MotorHealthConfig {
  double temp_warn_margin_c = 3.0;     ///< warn when temperature >= motor's own limit - this
  double load_full_scale_raw = 1000.0; ///< raw load counts that mean 100 % (MX series)
  double load_warn_fraction = 0.8;     ///< warn when |load| >= this fraction of full scale ...
  double load_sustain_s = 5.0;         ///< ... continuously for this long
  double voltage_min_v = 10.0;         ///< warn below this input voltage
  double voltage_max_v = 14.5;         ///< warn above this input voltage
};

enum class HealthLevel { kOk = 0, kWarn = 1, kFault = 2 };

struct MotorHealth {
  HealthLevel level = HealthLevel::kOk;
  std::string text = "ok";
  uint8_t hardware_error = 0;  ///< copy of the sample's register, for gating resets
};

/**
 * Evaluates MotorSamples into a HealthLevel, tracking how long each motor has
 * been above the load threshold. Pure logic: time is passed in, nothing here
 * touches ROS or the bus.
 *
 * Levels: kFault = a hardware error is latched (the firmware has already cut or
 * is about to cut torque; only a reboot clears it) or the packet's alert bit is
 * set; kWarn = temperature near the motor's own limit, sustained high load, or
 * input voltage out of range; kOk otherwise (including "no valid sample yet").
 */
class MotorHealthMonitor {
 public:
  MotorHealthMonitor(size_t num_motors, const MotorHealthConfig& config)
      : config_(config), high_load_since_(num_motors, -1.0) {}

  MotorHealth Evaluate(size_t index, const MotorSample& s, double temperature_limit_c, double now_s) {
    MotorHealth h;
    if (!s.valid) {
      high_load_since_[index] = -1.0;
      h.text = "no data";
      return h;
    }
    h.hardware_error = s.hardware_error;
    if (s.hardware_error != 0) {
      h.level = HealthLevel::kFault;
      h.text = "hardware error 0x" + Hex(s.hardware_error) + " (" +
               JoinNames(DecodeHardwareError(s.hardware_error)) + "); reboot the motor to clear";
      return h;
    }
    if (s.alert) {
      h.level = HealthLevel::kFault;
      h.text = "hardware alert bit set but the error register reads 0; re-read pending";
      return h;
    }

    std::vector<std::string> warnings;
    if (s.temperature_c + config_.temp_warn_margin_c >= temperature_limit_c) {
      warnings.push_back("temperature " + std::to_string(s.temperature_c) + " C near limit " +
                         std::to_string(static_cast<int>(temperature_limit_c)) + " C");
    }
    const double load_fraction = std::abs(static_cast<double>(s.load_raw)) / config_.load_full_scale_raw;
    if (load_fraction >= config_.load_warn_fraction) {
      if (high_load_since_[index] < 0.0) high_load_since_[index] = now_s;
      if (now_s - high_load_since_[index] >= config_.load_sustain_s) {
        warnings.push_back("load " + std::to_string(static_cast<int>(100.0 * load_fraction)) + " % for " +
                           std::to_string(static_cast<int>(now_s - high_load_since_[index])) + " s");
      }
    } else {
      high_load_since_[index] = -1.0;
    }
    const double volts = 0.1 * s.voltage_raw;
    if (volts < config_.voltage_min_v || volts > config_.voltage_max_v) {
      warnings.push_back("input voltage " + std::to_string(volts).substr(0, 4) + " V out of range");
    }
    if (!warnings.empty()) {
      h.level = HealthLevel::kWarn;
      h.text = warnings[0];
      for (size_t i = 1; i < warnings.size(); ++i) h.text += "; " + warnings[i];
    }
    return h;
  }

 private:
  static std::string Hex(uint8_t v) {
    static const char* d = "0123456789ABCDEF";
    return std::string() + d[v >> 4] + d[v & 0xF];
  }

  MotorHealthConfig config_;
  std::vector<double> high_load_since_;  ///< time the load first exceeded the threshold, -1 = not above
};

/** Worst level in a list (kOk for an empty list). */
inline HealthLevel WorstLevel(const std::vector<MotorHealth>& healths) {
  HealthLevel worst = HealthLevel::kOk;
  for (const auto& h : healths) {
    if (static_cast<int>(h.level) > static_cast<int>(worst)) worst = h.level;
  }
  return worst;
}

/**
 * True when the software e-stop should be triggered: some motor is in kFault and
 * the operator left escalation enabled. Warnings never trigger it.
 */
inline bool ShouldTriggerEStop(const std::vector<MotorHealth>& healths, bool escalate_enabled) {
  return escalate_enabled && WorstLevel(healths) == HealthLevel::kFault;
}

/**
 * Bitmask of motors (bit i = motor index i) that must be excluded from torque
 * writes: those with a latched hardware error. Torque enable on such a motor
 * does nothing useful and a failed write elsewhere in the node ends the process.
 */
inline uint32_t FaultedMask(const std::vector<MotorHealth>& healths) {
  uint32_t mask = 0;
  for (size_t i = 0; i < healths.size() && i < 32; ++i) {
    if (healths[i].level == HealthLevel::kFault) mask |= (1u << i);
  }
  return mask;
}

/**
 * A software e-stop reset is allowed only when no motor still has a latched
 * hardware error: resetting would release the hold on a machine that cannot
 * follow commands. Empty string = allowed, otherwise the reason.
 */
inline std::string ResetRefusalReason(const std::vector<MotorHealth>& healths, const std::vector<int>& ids) {
  for (size_t i = 0; i < healths.size(); ++i) {
    if (healths[i].level == HealthLevel::kFault) {
      return "motor ID " + std::to_string(i < ids.size() ? ids[i] : static_cast<int>(i)) + " (joint_" +
             std::to_string(i + 1) + "): " + healths[i].text;
    }
  }
  return "";
}

/**
 * Whether the status packet answering a REBOOT instruction means the reboot was refused.
 *
 * The motor answers BEFORE it restarts, so the ERROR byte still carries the Hardware Alert flag (bit 7) of the
 * fault being cleared: that alone is not a failure (it made every reset of a faulted motor look failed while the
 * motor in fact rebooted). Only the error number (bits 0-6: result fail, instruction error, CRC, access, ...) is.
 */
inline bool RebootPacketRejected(uint8_t error_byte) { return (error_byte & 0x7F) != 0; }

}  // namespace arm_interface
