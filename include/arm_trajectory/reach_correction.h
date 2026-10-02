#pragma once

#include <string>

#include <Eigen/Dense>

namespace arm_trajectory {

// Integral-style steady-state correction for reach_target_pose_node.
//
// In Position Control Mode a loaded joint stops short of its goal and
// nothing in the motor's loop removes the offset (measured on hardware
// 2026-10-02: ~21 mm end-effector error on a 3 cm reach, constant over
// 0.3 s, plan itself accurate to 1 mm). The planner cannot see that, so
// after a reach the node measures where the arm really ended up and, if
// enabled, re-aims at the original target PLUS an accumulated
// compensation. Each step ADDS the remaining error to the running
// compensation (integral action) rather than re-aiming relative to the
// original target each time, so repeated steps converge instead of
// oscillating around a fixed overshoot.
struct ReachCorrectionOptions {
  int max_steps = 0;          // 0 = correction off
  double trigger = 0.002;     // m: error at or below this needs no correction
  double max_total = 0.03;    // m: cap on the accumulated compensation magnitude
  double gain = 1.0;          // fraction of the remaining error added per step, (0, 1]
};

struct CorrectionDecision {
  bool apply = false;
  Eigen::Vector3d compensation = Eigen::Vector3d::Zero();  // new TOTAL compensation if apply
  std::string reason;                                       // why not, when !apply
};

// target / achieved: end-effector positions (m). compensation: the total
// currently applied on top of the original target. Pure function, no ROS.
inline CorrectionDecision NextCorrection(const Eigen::Vector3d& target, const Eigen::Vector3d& achieved,
                                         const Eigen::Vector3d& compensation, const ReachCorrectionOptions& options,
                                         int steps_done) {
  CorrectionDecision d;
  d.compensation = compensation;
  if (options.max_steps <= 0 || options.gain <= 0.0 || options.max_total <= 0.0) {
    d.reason = "correction disabled";
    return d;
  }
  if (steps_done >= options.max_steps) {
    d.reason = "correction step budget used";
    return d;
  }
  Eigen::Vector3d error = target - achieved;
  if (error.norm() <= options.trigger) {
    d.reason = "error within trigger";
    return d;
  }
  Eigen::Vector3d candidate = compensation + options.gain * error;
  if (candidate.norm() > options.max_total) {
    candidate *= options.max_total / candidate.norm();
  }
  if ((candidate - compensation).norm() < 1e-6) {
    d.reason = "compensation saturated at max_total";
    return d;
  }
  d.apply = true;
  d.compensation = candidate;
  return d;
}

}  // namespace arm_trajectory
