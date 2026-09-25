#pragma once

#include <string>
#include <vector>

#include <trajectory_msgs/JointTrajectory.h>

namespace arm_trajectory {

// Quintic-time-scaled joint-space trajectory from `q_start` to `q_goal`,
// synchronized so every joint arrives together at the same duration T --
// the max over joints of each one's own minimum feasible duration given
// its velocity limit (see docs/trajectory_generation.md /
// MinDurationForVelocityLimit). Sampled at `sample_rate_hz` into a
// trajectory_msgs::JointTrajectory (positions, velocities, and
// accelerations populated at every point; the first point is `q_start`
// at time 0, the last is `q_goal` at time T).
//
// Throws std::invalid_argument if q_start/q_goal/velocity_limits/
// joint_names sizes don't all match, or if sample_rate_hz <= 0.
trajectory_msgs::JointTrajectory GenerateJointSpaceTrajectory(
    const std::vector<double>& q_start, const std::vector<double>& q_goal,
    const std::vector<double>& velocity_limits, const std::vector<std::string>& joint_names,
    double sample_rate_hz);

}  // namespace arm_trajectory
