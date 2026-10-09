#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

/// @file
/// Qt- and ROS-free decisions of the arm control panel: health roll-up, status line, motor report rows,
/// which buttons are usable. Kept apart from the widgets so they can be unit-tested without a display.

namespace arm_gui {

enum class Health { kUnknown, kOk, kWarn, kError };

/// DiagnosticStatus level (0 OK, 1 WARN, 2+ ERROR/STALE) to Health.
inline Health HealthFromLevel(int level) {
  return level <= 0 ? Health::kOk : (level == 1 ? Health::kWarn : Health::kError);
}

/// Worst health of a set of diagnostic levels; kUnknown when there are none.
inline Health WorstHealth(const std::vector<int>& levels) {
  if (levels.empty()) return Health::kUnknown;
  return HealthFromLevel(*std::max_element(levels.begin(), levels.end()));
}

inline const char* HealthName(Health h) {
  switch (h) {
    case Health::kOk: return "OK";
    case Health::kWarn: return "WARN";
    case Health::kError: return "ERROR";
    default: return "n/a";
  }
}

enum class Torque { kUnknown, kOn, kOff };

/// Torque state from /arm_interface/drive_state (the driver has no separate torque topic).
inline Torque TorqueFromDrive(const std::string& drive) {
  if (drive.empty()) return Torque::kUnknown;
  return drive == "TORQUE_OFF" ? Torque::kOff : Torque::kOn;
}

/// E-stop as the panel knows it. An unknown state is treated as engaged (fail closed, like the senders).
enum class EStop { kUnknown, kReleased, kEngaged };

inline bool EStopBlocks(EStop e) { return e != EStop::kReleased; }

/// One line of the motor report, from a DiagnosticStatus (name, level, message, key/value strings).
struct MotorRow {
  std::string name;
  int level = 0;
  std::string message;
  bool has_readings = false;
  double temperature_c = 0.0, temperature_limit_c = 0.0, load_percent = 0.0, voltage_v = 0.0;
};

inline MotorRow MakeMotorRow(const std::string& full_name, int level, const std::string& message,
                             const std::map<std::string, std::string>& values) {
  MotorRow row;
  const auto slash = full_name.find_last_of('/');
  row.name = slash == std::string::npos ? full_name : full_name.substr(slash + 1);
  row.level = level;
  row.message = message == "ok" ? "" : message;
  auto get = [&values](const char* key, double* out) {
    auto it = values.find(key);
    if (it == values.end() || it->second.empty()) return false;
    char* end = nullptr;
    const double v = std::strtod(it->second.c_str(), &end);
    if (end == it->second.c_str()) return false;
    *out = v;
    return true;
  };
  row.has_readings = get("temperature_c", &row.temperature_c) & get("temperature_limit_c", &row.temperature_limit_c) &
                     get("load_percent", &row.load_percent) & get("input_voltage_v", &row.voltage_v);
  return row;
}

/// Name and message of the first motor in ERROR, "" when none is.
inline std::string FirstProblem(const std::vector<MotorRow>& rows) {
  for (const auto& r : rows)
    if (r.level >= 2) return r.name + ": " + r.message.substr(0, 50);
  return "";
}

/// Status bar text, e.g. "e-stop: released | drive: HOLDING | motors: OK".
inline std::string StatusText(EStop estop, const std::string& drive, Health motors, const std::string& problem = "") {
  const char* head = estop == EStop::kEngaged ? "e-stop: ENGAGED"
                     : estop == EStop::kReleased ? "e-stop: released"
                                                 : "e-stop: UNKNOWN (treated as engaged)";
  std::string text = std::string(head) + " | drive: " + (drive.empty() ? "n/a" : drive) + " | motors: " + HealthName(motors);
  if (!problem.empty()) text += " | " + problem;
  return text;
}

/// What the panel currently knows, as input to the button rules.
struct PanelState {
  EStop estop = EStop::kUnknown;
  Torque torque = Torque::kUnknown;
  Health motors = Health::kUnknown;
  bool backend_present = false;  ///< the preset service exists
  bool move_active = false;
  bool gripper_backend_present = false;  ///< the gripper_control services exist
  bool gripper_busy = false;             ///< an open/close call is in progress
};

struct ButtonEnables {
  bool go, save, cancel, reset, torque, reenable;
  bool gripper_move, gripper_stop;
};

/// Which buttons are usable. E-STOP itself is always enabled and is not listed.
/// go: only with torque on and the e-stop released (the executor refuses otherwise; refusing here says why).
inline ButtonEnables ComputeEnables(const PanelState& s) {
  ButtonEnables e;
  e.go = s.backend_present && !s.move_active && s.estop == EStop::kReleased && s.torque == Torque::kOn;
  e.save = s.backend_present;
  e.cancel = s.move_active;
  e.reset = EStopBlocks(s.estop);
  e.torque = s.estop == EStop::kReleased && s.torque != Torque::kUnknown && !s.move_active;
  e.reenable = s.motors == Health::kError;
  // Gripper: open/close need the e-stop released and torque on (the driver would refuse otherwise); stop is always allowed while a
  // call is running, even under an e-stop, like the E-STOP button itself.
  e.gripper_move = s.gripper_backend_present && !s.gripper_busy && s.estop == EStop::kReleased && s.torque == Torque::kOn;
  e.gripper_stop = s.gripper_backend_present && s.gripper_busy;
  return e;
}

/// One line for the gripper box: position (mm), load (% of max torque), and the gripper_control phase/message.
inline std::string GripperText(bool backend_present, bool has_position, double position_m, bool has_load, double load_percent,
                               const std::string& phase, const std::string& message) {
  if (!backend_present) return "gripper_control is not running (roslaunch arm_grasp gripper_control.launch)";
  char buf[64];
  std::string text;
  if (has_position) {
    std::snprintf(buf, sizeof(buf), "joint_6 %.1f mm", 1000.0 * position_m);
    text = buf;
  } else {
    text = "joint_6 n/a";
  }
  if (has_load) {
    std::snprintf(buf, sizeof(buf), " | load %.1f %%", load_percent);
    text += buf;
  } else {
    text += " | load n/a";
  }
  if (!phase.empty()) text += " | " + phase + (message.empty() ? "" : ": " + message);
  return text;
}

/// True when no motor_status has arrived for longer than max_age_s (or never): the table must not be trusted.
inline bool MotorDataStale(double age_s, double max_age_s) { return age_s < 0.0 || age_s > max_age_s; }

inline std::string HtmlEscape(const std::string& in) {
  std::string out;
  for (char c : in) {
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else out += c;
  }
  return out;
}

/// Background colour (opaque, hex) of a motor row by diagnostic level.
inline const char* LevelHex(int level) { return level <= 0 ? "#b8e0be" : (level == 1 ? "#f8de80" : "#ec8080"); }

/// The motor report as an HTML table (shown in a QLabel): Motor, Temp C (value / limit), Load %, Volt, Status.
inline std::string MotorTableHtml(const std::vector<MotorRow>& rows) {
  std::string h = "<table width='100%' cellspacing='0' cellpadding='3'><tr><th align='left'>Motor</th>"
                  "<th>Temp C</th><th>Load %</th><th>Volt</th><th align='left'>Status</th></tr>";
  for (const auto& r : rows) {
    char t[48], l[24], v[24];
    std::snprintf(t, sizeof(t), "%.0f / %.0f", r.temperature_c, r.temperature_limit_c);
    std::snprintf(l, sizeof(l), "%.1f", r.load_percent);
    std::snprintf(v, sizeof(v), "%.1f", r.voltage_v);
    const std::string status = r.message.empty() ? HealthName(HealthFromLevel(r.level)) : r.message;
    h += std::string("<tr bgcolor='") + LevelHex(r.level) + "'><td>" + HtmlEscape(r.name) + "</td>";
    h += r.has_readings ? std::string("<td align='center'>") + t + "</td><td align='center'>" + l + "</td><td align='center'>" + v + "</td>"
                        : "<td align='center'>-</td><td align='center'>-</td><td align='center'>-</td>";
    h += "<td>" + HtmlEscape(status) + "</td></tr>";
  }
  return h + "</table>";
}

/// Largest absolute joint difference [rad] between two equal-length poses.
inline double LargestJointMove(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0.0;
  for (size_t i = 0; i < a.size() && i < b.size(); ++i) m = std::max(m, std::fabs(a[i] - b[i]));
  return m;
}

/// One-line text of a motor row for the table's tooltip and the log.
inline std::string FormatMotorRow(const MotorRow& r) {
  char buf[160];
  if (r.has_readings)
    std::snprintf(buf, sizeof(buf), "%s  %3.0f/%.0f C  load %5.1f%%  %4.1f V", r.name.c_str(), r.temperature_c,
                  r.temperature_limit_c, r.load_percent, r.voltage_v);
  else
    std::snprintf(buf, sizeof(buf), "%s  (no readings)", r.name.c_str());
  std::string s = buf;
  if (!r.message.empty()) s += "  " + r.message;
  return s;
}

}  // namespace arm_gui
