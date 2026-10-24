#pragma once

#include <vector>

#include <Eigen/Dense>

#include "arm_kinematics/forward_kinematics.h"

namespace arm_kinematics {

// Geometric (analytical) Jacobian of solver's chain at joint_positions:
// column i = [z_i x (p_e - p_i) ; z_i], where p_i/z_i are joint i's axis
// origin/direction (ForwardKinematicsSolver::JointFrame) and p_e is the
// end-effector (tip_link) position, all in root_link. See
// docs/inverse_kinematics.md for the derivation.
//
// Rows 0-2: linear velocity part. Rows 3-5: angular velocity part.
// Throws std::invalid_argument (via ComputePose) on a joint_positions size
// mismatch.
Eigen::Matrix<double, 6, Eigen::Dynamic> ComputeGeometricJacobian(
    const ForwardKinematicsSolver& solver, const std::vector<double>& joint_positions);

}  // namespace arm_kinematics
