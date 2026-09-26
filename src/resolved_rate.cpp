#include "arm_teleop/resolved_rate.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "arm_kinematics/jacobian.h"

namespace arm_teleop {

ResolvedRateResult ComputeResolvedRateVelocity(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const std::vector<double>& joint_positions, double vx, double vy, double vz,
    double pitch_rate, const std::vector<double>& velocity_limits, double damping,
    const arm_kinematics::ForwardKinematicsSolver* secondary_link_solver, double nullspace_gain) {
  const size_t n = joint_positions.size();

  Eigen::Isometry3d pose = fk_solver.ComputePose(joint_positions);
  Eigen::Vector3d pitch_axis_root = pose.rotation() * Eigen::Vector3d::UnitY();

  Eigen::Matrix<double, 6, Eigen::Dynamic> J = arm_kinematics::ComputeGeometricJacobian(
      fk_solver, joint_positions);

  Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
  J_task.topRows<3>() = J.topRows<3>();
  J_task.row(3) = pitch_axis_root.transpose() * J.bottomRows<3>();

  Eigen::Vector4d xdot_cmd(vx, vy, vz, pitch_rate);

  // Variable (Chiaverini-style) damping instead of a fixed lambda: a fixed
  // lambda regularizes every configuration by the same amount regardless of
  // how well-conditioned J_task actually is, which biases the DIRECTION of
  // the achieved velocity away from xdot_cmd whenever J_task's singular
  // values are spread out (not just its magnitude) -- confirmed in
  // test_resolved_rate.cpp's own kNegligibleDamping-vs-kDamping comment
  // ("~2-4% relative error at kDamping=0.05 ... consistent with a
  // damping-induced bias") and reported live as the jog target
  // systematically deviating from the commanded pose even in clearly
  // feasible configurations. J_task J_task^T = U Sigma^2 U^T (SVD), so its
  // eigenvalues are the squared singular values of J_task; sigma_min is the
  // smallest one. Damping now only switches on as sigma_min approaches the
  // threshold `damping` itself:
  //   lambda^2(sigma_min) = 0                                  if sigma_min >= damping
  //                       = damping^2 * (1 - (sigma_min/damping)^2)  otherwise
  // so a well-conditioned tick (sigma_min >= damping) recovers the exact,
  // undamped minimum-norm pseudo-inverse (zero direction bias), while a
  // tick approaching a true singularity (sigma_min -> 0) still tapers
  // smoothly to the same lambda^2 = damping^2 ceiling as before -- same
  // singularity-robustness, continuous at sigma_min = damping.
  Eigen::Matrix4d gram0 = J_task * J_task.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(gram0, Eigen::EigenvaluesOnly);
  double sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));
  double lambda_sq = 0.0;
  if (sigma_min < damping) {
    double ratio = sigma_min / damping;
    lambda_sq = damping * damping * (1.0 - ratio * ratio);
  }
  Eigen::Matrix4d gram = gram0 + lambda_sq * Eigen::Matrix4d::Identity();
  auto gram_ldlt = gram.ldlt();
  Eigen::Vector4d y = gram_ldlt.solve(xdot_cmd);
  Eigen::VectorXd qdot = J_task.transpose() * y;

  // Gated on an actual commanded task velocity, not just deadman held:
  // found the hard way in live testing that an always-on secondary task
  // makes the arm visibly drift/crawl while the operator is just holding
  // the deadman with the stick centered, which looks like a runaway bug
  // (there is nothing to "reach" -- the bias has no target, just a
  // gradient, so it never settles to exactly zero on its own). Real input
  // (xdot_cmd) is exactly zero here whenever the caller's deadzone
  // suppressed the stick, so this check is exact, not a heuristic.
  if (secondary_link_solver != nullptr && nullspace_gain != 0.0 && xdot_cmd.squaredNorm() > 1e-12) {
    const size_t m = secondary_link_solver->GetActuatedJointNames().size();
    if (m > n) {
      throw std::invalid_argument(
          "ComputeResolvedRateVelocity: secondary_link_solver has more actuated joints than "
          "joint_positions -- it must be an upstream sub-chain of fk_solver's");
    }
    std::vector<double> q_prefix(joint_positions.begin(), joint_positions.begin() + m);
    Eigen::Vector3d link_axis =
        secondary_link_solver->ComputePose(q_prefix).rotation() * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();

    // d(link_axis)/dq_i = z_i x link_axis, for each upstream joint i -- z_i
    // (the joint's own world-frame rotation axis) is identical whether
    // read off this Jacobian or one computed for secondary_link_solver
    // directly: it depends only on the shared root->joint_i sub-chain, not
    // on which tip the solver was built for. Reusing J's columns here
    // avoids a second Jacobian evaluation entirely.
    Eigen::VectorXd qdot0 = Eigen::VectorXd::Zero(n);
    for (size_t i = 0; i < m; ++i) {
      Eigen::Vector3d z_i = J.block<3, 1>(3, i);
      qdot0(i) = nullspace_gain * world_up.dot(z_i.cross(link_axis));
    }

    Eigen::MatrixXd J_pinv = J_task.transpose() * gram_ldlt.solve(Eigen::Matrix4d::Identity());
    Eigen::MatrixXd null_projector = Eigen::MatrixXd::Identity(n, n) - J_pinv * J_task;
    qdot += null_projector * qdot0;
  }

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
