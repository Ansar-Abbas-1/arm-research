#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <sensor_msgs/JointState.h>

namespace arm_kinematics {

// Finds `joint_name` in msg, writing its position to *out. Returns false
// (leaving *out untouched) if msg doesn't name it.
inline bool FindJointPosition(const sensor_msgs::JointState& msg, const std::string& joint_name,
                               double* out) {
  auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
  if (it == msg.name.end()) {
    return false;
  }
  *out = msg.position[std::distance(msg.name.begin(), it)];
  return true;
}

// Looks up every name in joint_names, in order. Returns false (leaving
// *out untouched) if msg doesn't name all of them -- the shared "wait for
// a complete /joint_states message" pattern every node in this workspace
// that tracks live joint state uses. `missing_joint_name`, if non-null, is
// set to the first name not found, for callers that want to report it.
inline bool ExtractJointPositions(const sensor_msgs::JointState& msg,
                                   const std::vector<std::string>& joint_names,
                                   std::vector<double>* out,
                                   std::string* missing_joint_name = nullptr) {
  std::vector<double> positions(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (!FindJointPosition(msg, joint_names[i], &positions[i])) {
      if (missing_joint_name != nullptr) {
        *missing_joint_name = joint_names[i];
      }
      return false;
    }
  }
  *out = std::move(positions);
  return true;
}

}  // namespace arm_kinematics
