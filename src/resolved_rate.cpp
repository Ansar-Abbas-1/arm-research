#include "arm_teleop/resolved_rate.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

#include "arm_kinematics/jacobian.h"

namespace arm_teleop {

ResolvedRateResult ComputeResolvedRateVelocity(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const std::vector<double>& joint_positions, double vx, double vy, double vz,
    double pitch_rate, const std::vector<double>& velocity_limits, double damping) {
  const size_t n = joint_positions.size();

  Eigen::Isometry3d pose = fk_solver.ComputePose(joint_positions);
  Eigen::Vector3d pitch_axis_root = pose.rotation() * Eigen::Vector3d::UnitY();

  Eigen::Matrix<double, 6, Eigen::Dynamic> J = arm_kinematics::ComputeGeometricJacobian(
      fk_solver, joint_positions);

  Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
  J_task.topRows<3>() = J.topRows<3>();
  J_task.row(3) = pitch_axis_root.transpose() * J.bottomRows<3>();

  Eigen::Vector4d xdot_cmd(vx, vy, vz, pitch_rate);

  Eigen::Matrix4d gram = J_task * J_task.transpose() + (damping * damping) * Eigen::Matrix4d::Identity();
  Eigen::Vector4d y = gram.ldlt().solve(xdot_cmd);
  Eigen::VectorXd qdot = J_task.transpose() * y;

  ResolvedRateResult result;
  result.joint_velocities.resize(n);
  for (size_t i = 0; i < n; ++i) {
    result.joint_velocities[i] = std::min(velocity_limits[i], std::max(-velocity_limits[i], qdot(i)));
  }

  Eigen::VectorXd qdot_clamped =
      Eigen::Map<const Eigen::VectorXd>(result.joint_velocities.data(), n);
  Eigen::Vector4d xdot_achieved = J_task * qdot_clamped;
  double cmd_norm = xdot_cmd.norm();
  result.achieved_to_commanded_ratio = cmd_norm > 1e-9 ? (xdot_achieved.norm() / cmd_norm) : 1.0;

  return result;
}

}  // namespace arm_teleop
