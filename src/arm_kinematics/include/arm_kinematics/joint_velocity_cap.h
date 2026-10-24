#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace arm_kinematics {

/**
 * Lowers per-joint velocity limits to user caps. Pure function (no ROS I/O).
 *
 * @param limits  base limits [rad/s], normally the URDF limits.
 * @param caps    empty = no cap; one entry = applied to every joint; otherwise
 *                one entry per joint. A scalar <= 0 means "unset". Per-joint
 *                entries must be > 0.
 * @param warn    optional out: caps above the base limit are clamped to it
 *                (a cap can only lower a limit); their joint indices are
 *                appended here so the caller can log them.
 * @throws std::invalid_argument on a wrong-sized or non-positive list.
 */
inline std::vector<double> CapJointVelocityLimits(const std::vector<double>& limits,
                                                  const std::vector<double>& caps,
                                                  std::vector<size_t>* warn = nullptr) {
  if (caps.empty() || (caps.size() == 1 && caps[0] <= 0.0)) {
    return limits;
  }
  if (caps.size() != 1 && caps.size() != limits.size()) {
    throw std::invalid_argument("max_joint_velocity has " + std::to_string(caps.size()) +
                                " entries, expected 1 or " + std::to_string(limits.size()));
  }
  std::vector<double> out = limits;
  for (size_t i = 0; i < limits.size(); ++i) {
    const double cap = caps.size() == 1 ? caps[0] : caps[i];
    if (!(cap > 0.0)) {
      throw std::invalid_argument("max_joint_velocity[" + std::to_string(i) +
                                  "] must be > 0, got " + std::to_string(cap));
    }
    if (cap > limits[i] && warn != nullptr) {
      warn->push_back(i);
    }
    out[i] = std::min(limits[i], cap);
  }
  return out;
}

/**
 * Reads `~max_joint_velocity` (scalar or list, rad/s; absent = no cap) and
 * applies CapJointVelocityLimits to `limits`. Throws on a malformed value, since
 * this is a safety knob and silently ignoring it would be worse.
 */
inline std::vector<double> ApplyMaxJointVelocityParam(ros::NodeHandle& pnh,
                                                      const std::vector<double>& limits) {
  XmlRpc::XmlRpcValue value;
  if (!pnh.getParam("max_joint_velocity", value)) {
    return limits;
  }
  auto as_double = [](XmlRpc::XmlRpcValue& v) {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<double>(static_cast<int>(v));
    throw std::invalid_argument("max_joint_velocity entries must be numbers");
  };
  std::vector<double> caps;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < value.size(); ++i) caps.push_back(as_double(value[i]));
  } else {
    caps.push_back(as_double(value));
  }
  std::vector<size_t> clamped;
  std::vector<double> out = CapJointVelocityLimits(limits, caps, &clamped);
  for (size_t i : clamped) {
    ROS_WARN_STREAM("max_joint_velocity[" << i << "] exceeds the URDF limit " << limits[i]
                     << " rad/s; using the URDF limit");
  }
  return out;
}

/**
 * Reads a stand-alone per-joint limit parameter (e.g. `~max_joint_acceleration`,
 * rad/s^2): absent, or a scalar <= 0, returns empty (= no limit); a scalar > 0
 * is broadcast to `n` joints; a list must have `n` entries, all > 0. Throws
 * std::invalid_argument otherwise.
 */
inline std::vector<double> ReadJointLimitParam(ros::NodeHandle& pnh, const std::string& name,
                                               size_t n) {
  XmlRpc::XmlRpcValue value;
  if (!pnh.getParam(name, value)) {
    return {};
  }
  auto as_double = [&name](XmlRpc::XmlRpcValue& v) {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(v);
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<double>(static_cast<int>(v));
    throw std::invalid_argument(name + " entries must be numbers");
  };
  std::vector<double> out;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < value.size(); ++i) out.push_back(as_double(value[i]));
  } else {
    double scalar = as_double(value);
    if (scalar <= 0.0) return {};
    out.assign(n, scalar);
  }
  if (out.size() != n) {
    throw std::invalid_argument(name + " has " + std::to_string(out.size()) + " entries, expected " +
                                std::to_string(n));
  }
  for (size_t i = 0; i < n; ++i) {
    if (!(out[i] > 0.0)) {
      throw std::invalid_argument(name + "[" + std::to_string(i) + "] must be > 0, got " +
                                  std::to_string(out[i]));
    }
  }
  return out;
}

}  // namespace arm_kinematics
