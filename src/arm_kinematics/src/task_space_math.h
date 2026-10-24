#pragma once

#include <Eigen/Dense>

namespace arm_kinematics {
namespace detail {

// Variable (Chiaverini) damped right-pseudo-inverse solve for a 4xN task
// Jacobian, shared by ComputeResolvedRateVelocity (rate-level: `rhs` is a
// commanded task velocity, the result a joint velocity) and
// PositionPitchIkSolver (position-level: `rhs` is a task-space error, the
// result a Newton step) -- identical linear algebra either way, see
// docs/resolved_rate.md's damping derivation. Not a public header: this is
// an internal implementation detail shared between this library's own
// .cpp files, not part of arm_kinematics' public API.
//
//   lambda^2(sigma_min) = 0                                        if sigma_min >= damping_threshold
//                       = damping_threshold^2 * (1 - (sigma_min/damping_threshold)^2)  otherwise
//   result = J_task^T (J_task J_task^T + lambda^2 I)^-1 rhs
//
// If `nullspace_qdot0` is non-null, it's projected into J_task's null
// space (Liegeois: (I - J_task+ J_task) * qdot0) and added to the primary
// result -- the caller computes qdot0 itself (it needs the secondary
// task's own Jacobian columns/link pose, which this shared helper has no
// reason to know about).
Eigen::VectorXd SolveDampedRightPseudoInverse(
    const Eigen::Matrix<double, 4, Eigen::Dynamic>& J_task, const Eigen::Vector4d& rhs,
    double damping_threshold, const Eigen::VectorXd* nullspace_qdot0 = nullptr);

}  // namespace detail
}  // namespace arm_kinematics
