#pragma once

#include <cmath>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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
  // trigger/max_total set from the 2026-10-02 hardware A/B (see
  // docs/trajectory_generation.md Finding 8): the arm's repeatability under
  // load is ~5-6 mm, so correcting below ~6 mm chases noise, and initial
  // errors reached 39 mm so a 30 mm cap bound in 3 of 4 runs.
  double trigger = 0.006;     // m: error at or below this needs no correction
  double max_total = 0.06;    // m: cap on the accumulated compensation magnitude
  double gain = 1.0;          // fraction of the remaining error added per step, (0, 1]
  // Pitch compensation (the same integral scheme on the one orientation
  // component this arm controls; roll and yaw are free, never corrected).
  // Shares max_steps and gain with the position correction.
  bool pitch = true;
  double pitch_trigger = 0.01;    // rad: |pitch error| at or below this needs no correction
  double pitch_max_total = 0.2;   // rad: cap on the accumulated pitch compensation
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

struct PitchDecision {
  bool apply = false;
  double compensation = 0.0;  // new TOTAL pitch compensation (rad) if apply
  std::string reason;
};

// Signed pitch error: the rotation (rad) about the ACHIEVED pose's own local
// Y axis still needed to reach the target orientation. Same quantity (and
// sign) PositionPitchIkSolver enforces as e_pitch.
inline double PitchError(const Eigen::Matrix3d& target_rotation, const Eigen::Matrix3d& achieved_rotation) {
  Eigen::AngleAxisd diff(target_rotation * achieved_rotation.transpose());
  Eigen::Vector3d rotation_error = diff.angle() * diff.axis();
  return (achieved_rotation * Eigen::Vector3d::UnitY()).dot(rotation_error);
}

// Integral pitch step, same shape as NextCorrection. `pitch_error` is the
// signed PitchError of the arm's real final pose.
inline PitchDecision NextPitchCorrection(double pitch_error, double compensation,
                                         const ReachCorrectionOptions& options, int steps_done) {
  PitchDecision d;
  d.compensation = compensation;
  if (!options.pitch || options.max_steps <= 0 || options.gain <= 0.0 || options.pitch_max_total <= 0.0) {
    d.reason = "pitch correction disabled";
    return d;
  }
  if (steps_done >= options.max_steps) {
    d.reason = "correction step budget used";
    return d;
  }
  if (std::fabs(pitch_error) <= options.pitch_trigger) {
    d.reason = "pitch error within trigger";
    return d;
  }
  double candidate = compensation + options.gain * pitch_error;
  if (std::fabs(candidate) > options.pitch_max_total) {
    candidate = std::copysign(options.pitch_max_total, candidate);
  }
  if (std::fabs(candidate - compensation) < 1e-6) {
    d.reason = "pitch compensation saturated at pitch_max_total";
    return d;
  }
  d.apply = true;
  d.compensation = candidate;
  return d;
}

// Where to aim: the original target shifted by the position compensation
// and rotated by the pitch compensation about the target's own local Y.
inline Eigen::Isometry3d AimPose(const Eigen::Isometry3d& target, const Eigen::Vector3d& position_compensation,
                                 double pitch_compensation) {
  Eigen::Isometry3d aim = target;
  aim.translation() += position_compensation;
  aim.linear() = target.rotation() * Eigen::AngleAxisd(pitch_compensation, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return aim;
}

}  // namespace arm_trajectory
