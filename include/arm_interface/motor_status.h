#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace arm_interface {

/**
 * Decodes the Dynamixel X-series "Hardware Error Status" register (address 70)
 * into the names of the set bits. Empty vector = no error. Pure function.
 *
 *   bit 0 input voltage error   bit 3 motor encoder error
 *   bit 2 overheating error     bit 4 electrical shock error
 *   bit 5 overload error        (bits 1, 6, 7 are unused)
 */
inline std::vector<std::string> DecodeHardwareError(uint8_t status) {
  static const char* const kNames[8] = {"input voltage", "reserved bit 1", "overheating", "motor encoder",
                                         "electrical shock", "overload", "reserved bit 6", "reserved bit 7"};
  std::vector<std::string> out;
  for (int bit = 0; bit < 8; ++bit) {
    if (status & (1u << bit)) {
      out.push_back(kNames[bit]);
    }
  }
  return out;
}

/**
 * Names of the error types in the Shutdown register (address 63) that make the
 * firmware switch torque OFF by itself when they occur. Same bit layout as
 * DecodeHardwareError. Empty = the motor never shuts down on its own.
 */
inline std::vector<std::string> DecodeShutdownMask(uint8_t mask) { return DecodeHardwareError(mask); }

/** Operating mode register (address 11) as text. */
inline std::string DecodeOperatingMode(uint8_t mode) {
  switch (mode) {
    case 0: return "current";
    case 1: return "velocity";
    case 3: return "position";
    case 4: return "extended position";
    case 5: return "current-based position";
    case 16: return "PWM";
    default: return "unknown (" + std::to_string(mode) + ")";
  }
}

/**
 * Joins names with ", " for printing; "none" for an empty list.
 */
inline std::string JoinNames(const std::vector<std::string>& names) {
  if (names.empty()) {
    return "none";
  }
  std::string out = names[0];
  for (size_t i = 1; i < names.size(); ++i) {
    out += ", " + names[i];
  }
  return out;
}

}  // namespace arm_interface
