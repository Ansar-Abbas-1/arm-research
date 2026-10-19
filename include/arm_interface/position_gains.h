#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace arm_interface {

// Validation of a request to write the Dynamixel position-loop gains (Protocol 2.0 X/MX table: Position D gain 80,
// I gain 82, P gain 84, 2 bytes each, RAM: cleared at a power cycle, never the EEPROM). Pure: no ROS, no bus.
//
// A request holds per joint a value for P, I and D; kGainUnchanged (-1) leaves that gain alone and a vector left empty
// leaves the whole gain unchanged. Rules (heuristic safety limits, not from the datasheet):
//  - the gripper joint (the last one) is never written: its hold load is regulated by gripper_control;
//  - I may be raised up to limits.max_i (default 1000, far below the register's 16383) so a typo cannot saturate the loop;
//  - P and D are refused unless limits.allow_pd, since the test this exists for only needs the integral term;
//  - every value is -1 or 0..limits.max_register.
constexpr int kGainUnchanged = -1;

struct GainRequest {
  std::vector<int> p, i, d;
};

struct GainLimits {
  int max_register = 16383;   ///< the 2-byte register's documented range
  int max_i = 1000;           ///< operator cap on the integral gain (heuristic)
  bool allow_pd = false;      ///< P and D writes need an explicit opt-in
};

inline int GainFor(const std::vector<int>& v, std::size_t joint) {
  return joint < v.size() ? v[joint] : kGainUnchanged;
}

/// Empty string if the request is acceptable, otherwise the reason it is refused. `num_joints` includes the gripper (the last joint).
inline std::string ValidateGains(const GainRequest& r, std::size_t num_joints, const GainLimits& limits) {
  const struct { const char* name; const std::vector<int>* v; int cap; bool allowed; } gains[] = {
      {"P", &r.p, limits.max_register, limits.allow_pd},
      {"I", &r.i, limits.max_i, true},
      {"D", &r.d, limits.max_register, limits.allow_pd}};
  for (const auto& g : gains) {
    if (!g.v->empty() && g.v->size() != num_joints)
      return std::string(g.name) + " gain: need " + std::to_string(num_joints) + " values (or none), got " + std::to_string(g.v->size());
    for (std::size_t j = 0; j < g.v->size(); ++j) {
      const int value = (*g.v)[j];
      if (value == kGainUnchanged) continue;
      if (!g.allowed) return std::string(g.name) + " gain writes are disabled (allow_pd_gain_write is false)";
      if (j + 1 == num_joints) return std::string(g.name) + " gain of the gripper (joint " + std::to_string(num_joints) + ") is never written";
      if (value < 0 || value > limits.max_register)
        return std::string(g.name) + " gain of joint " + std::to_string(j + 1) + ": " + std::to_string(value) + " is outside 0.." + std::to_string(limits.max_register);
      if (value > g.cap)
        return std::string(g.name) + " gain of joint " + std::to_string(j + 1) + ": " + std::to_string(value) + " is above the cap " + std::to_string(g.cap);
    }
  }
  return "";
}

/// True if the request changes at least one gain.
inline bool AnyGainChange(const GainRequest& r) {
  for (const auto* v : {&r.p, &r.i, &r.d})
    for (int x : *v)
      if (x != kGainUnchanged) return true;
  return false;
}

}  // namespace arm_interface
