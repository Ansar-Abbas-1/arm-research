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
// `damping` (lambda) is now VARIABLE, not fixed: it is used as a
// singularity threshold on sigma_min (J_task's smallest singular value),
// only switching on smoothly as sigma_min drops below it --
//   lambda^2(sigma_min) = 0                                        if sigma_min >= damping
//                       = damping^2 * (1 - (sigma_min/damping)^2)  otherwise
// away from a singularity (sigma_min >= damping) this recovers the exact,
// undamped minimum-norm pseudo-inverse -- a FIXED lambda regularizes every
// tick by the same amount regardless of conditioning, which biases the
// achieved velocity's DIRECTION (not just its magnitude) away from
// xdot_cmd whenever J_task's singular values are spread out, even in a
// clearly feasible pose (confirmed live and in test_resolved_rate.cpp's
// kNegligibleDamping-vs-kDamping comment). Near an actual singularity
// (sigma_min -> 0), lambda^2 still tapers up to the same damping^2 ceiling
// as before, so the safety property (never blowing up) is unchanged.
//
// The result is then clamped componentwise to `velocity_limits` (rad/s,
// GetActuatedJointVelocityLimits()) -- a defensive hardware-safety clamp
// independent of the damping above.
//
// Optional null-space secondary task: this chain is redundant for the 4D
// task above (5 joints, 4 rows), so infinitely many qdot achieve the same
// task velocity -- the formula above picks the minimum-norm one, with no
// preference among the others. If `secondary_link_solver` is non-null and
// `nullspace_gain` is nonzero, a secondary objective is projected into the
// task's null space (Liegeois gradient projection: qdot = qdot_primary +
// (I - J_task+ J_task) qdot_0), biasing `secondary_link_solver`'s tip link
// toward vertical (its own local +Z axis aligned with world +Z) WITHOUT
// changing the achieved task velocity at all -- N qdot_0 is, by
// construction, exactly in J_task's null space. `secondary_link_solver`
// must share `fk_solver`'s root_link and actuated-joint ordering, and its
// actuated-joint count must be <= `joint_positions.size()` (only the
// joints upstream of its own tip link can affect that link's orientation,
// e.g. link_4's own joint and everything downstream of it don't). See
// docs/teleop.md for the derivation. nullspace_gain = 0 (or a null
// secondary_link_solver) disables this entirely, reducing to the
// primary-task-only law above -- as does a zero commanded task velocity
// (vx=vy=vz=pitch_rate=0), even with nullspace_gain nonzero: the bias has
// no target pose to settle into, only a gradient to follow, so leaving it
// active while the operator holds the deadman with a centered stick makes
// the arm visibly drift with nothing commanding it (confirmed the hard
// way in live testing) -- gating it on real input fixes that, and is
// exact rather than a heuristic, since the caller's deadzone already
// makes xdot_cmd exactly zero whenever the stick is centered.
ResolvedRateResult ComputeResolvedRateVelocity(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const std::vector<double>& joint_positions, double vx, double vy, double vz,
    double pitch_rate, const std::vector<double>& velocity_limits, double damping,
    const arm_kinematics::ForwardKinematicsSolver* secondary_link_solver = nullptr,
    double nullspace_gain = 0.0);

}  // namespace arm_teleop
