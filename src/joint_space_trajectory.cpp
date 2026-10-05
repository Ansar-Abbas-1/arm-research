#include "arm_trajectory/joint_space_trajectory.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <ros/duration.h>

#include "arm_trajectory/quintic_time_scaling.h"

namespace arm_trajectory {

trajectory_msgs::JointTrajectory GenerateJointSpaceTrajectory(
    const std::vector<double>& q_start, const std::vector<double>& q_goal,
    const std::vector<double>& velocity_limits, const std::vector<std::string>& joint_names,
    double sample_rate_hz, const std::vector<double>& acceleration_limits) {
  const size_t n = q_start.size();
  if (q_goal.size() != n || velocity_limits.size() != n || joint_names.size() != n) {
    throw std::invalid_argument(
        "GenerateJointSpaceTrajectory: q_start/q_goal/velocity_limits/joint_names size mismatch");
  }
  if (!acceleration_limits.empty() && acceleration_limits.size() != n) {
    throw std::invalid_argument("GenerateJointSpaceTrajectory: acceleration_limits must be empty or one per joint");
  }
  if (sample_rate_hz <= 0.0) {
    throw std::invalid_argument("GenerateJointSpaceTrajectory: sample_rate_hz must be > 0");
  }

  std::vector<double> delta(n);
  double duration = 0.0;
  for (size_t i = 0; i < n; ++i) {
    delta[i] = q_goal[i] - q_start[i];
    duration = std::max(duration, MinDurationForVelocityLimit(delta[i], velocity_limits[i]));
    if (!acceleration_limits.empty()) {
      duration = std::max(duration, MinDurationForAccelerationLimit(delta[i], acceleration_limits[i]));
    }
  }

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names;

  if (duration == 0.0) {
    // Already at goal (or no joint needs to move): single point, no motion.
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = q_start;
    point.velocities.assign(n, 0.0);
    point.accelerations.assign(n, 0.0);
    point.time_from_start = ros::Duration(0.0);
    traj.points.push_back(point);
    return traj;
  }

  int num_samples = std::max(2, static_cast<int>(std::round(duration * sample_rate_hz)) + 1);
  traj.points.reserve(num_samples);

  for (int k = 0; k < num_samples; ++k) {
    double t = duration * static_cast<double>(k) / static_cast<double>(num_samples - 1);
    double tau = t / duration;

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(n);
    point.velocities.resize(n);
    point.accelerations.resize(n);
    for (size_t i = 0; i < n; ++i) {
      point.positions[i] = q_start[i] + delta[i] * QuinticPosition(tau);
      point.velocities[i] = delta[i] * QuinticVelocity(tau) / duration;
      point.accelerations[i] = delta[i] * QuinticAcceleration(tau) / (duration * duration);
    }
    point.time_from_start = ros::Duration(t);
    traj.points.push_back(point);
  }
  // Snap the endpoints to the exact requested values -- floating-point
  // evaluation of the quintic at tau=0/1 should already give exactly
  // q_start/q_goal, but this removes any doubt for callers/tests that
  // compare against the exact requested goal.
  traj.points.front().positions = q_start;
  traj.points.back().positions = q_goal;

  return traj;
}

}  // namespace arm_trajectory
