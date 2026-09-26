#pragma once

#include <vector>

#include <arm_kinematics/forward_kinematics.h>

namespace arm_teleop {

struct ResolvedRateResult {
  std::vector<double> joint_velocities;  // rad/s, already clamped to velocity_limits
  // ||achieved task velocity|| / ||commanded task velocity||, after the
  // velocity_limits clamp above -- 1.0 if the command was ~zero (nothing
  // to saturate). A caller compares this against a threshold to warn when
  // a limit or near-singular Jacobian meaningfully reduced the commanded
  // motion, without ever treating that as a failure (unlike position-IK,
  // there is no convergence check here to fail).
  double achieved_to_commanded_ratio = 1.0;
};

// Damped-least-squares resolved-rate control. Solves for joint velocities
// realizing the task-space command [vx, vy, vz, pitch_rate] as closely as
// possible: vx/vy/vz in solver's root_link frame, pitch_rate about the
// end-effector's OWN current local Y axis (not a root-frame axis), so
// "pitch" means the same physical wrist motion regardless of the arm's
// current orientation. Roll/yaw are intentionally not commanded at all --
// left free to whatever the minimum-norm joint velocity solution implies,
// not actively held.
//
// Derivation (see docs/teleop.md): row-project the standard 6xN geometric
// Jacobian's angular block onto the pitch axis to get a 4xN task Jacobian
// J_task = [J_v ; e_pitch^T J_w]. Task rows (4) < joints (N=5) here --
// OPPOSITE of arm_kinematics::InverseKinematicsSolver's full-6D-pose case
// (6 > 5, left/normal-equations pseudo-inverse) -- so this is the
// redundant-manipulator (right) damped pseudo-inverse form:
//   qdot = J_task^T (J_task J_task^T + damping^2 I)^-1 xdot_cmd
// `damping` tapers qdot smoothly as J_task approaches rank deficiency
// (a singularity, or a direction the task-space projection can't reach)
// instead of blowing up -- same role as IkOptions::initial_damping, just
// fixed rather than adaptive (there's no iteration to adapt over: this is
// a single per-tick velocity solve, not an iterative position solve).
//
// The result is then clamped componentwise to `velocity_limits` (rad/s,
// GetActuatedJointVelocityLimits()) -- a defensive hardware-safety clamp
// independent of the damping above.
ResolvedRateResult ComputeResolvedRateVelocity(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const std::vector<double>& joint_positions, double vx, double vy, double vz,
    double pitch_rate, const std::vector<double>& velocity_limits, double damping);

}  // namespace arm_teleop
