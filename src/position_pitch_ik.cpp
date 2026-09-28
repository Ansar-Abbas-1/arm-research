#include "arm_kinematics/position_pitch_ik.h"

#include <algorithm>
#include <stdexcept>

#include "arm_kinematics/jacobian.h"
#include "task_space_math.h"

namespace arm_kinematics {

namespace {

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

}  // namespace

PositionPitchIkSolver::PositionPitchIkSolver(const ForwardKinematicsSolver& solver,
                                              PositionPitchIkOptions options)
    : solver_(solver), options_(options), rng_(options.rng_seed) {}

PositionPitchIkResult PositionPitchIkSolver::Solve(const Eigen::Isometry3d& target_pose,
                                                     const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  if (seed.size() != limits.size()) {
    throw std::invalid_argument("PositionPitchIkSolver::Solve: seed has " +
                                 std::to_string(seed.size()) + " values, expected " +
                                 std::to_string(limits.size()));
  }

  PositionPitchIkResult best = SolveFromSeed(target_pose, seed);

  std::vector<std::uniform_real_distribution<double>> dists;
  dists.reserve(limits.size());
  for (const auto& lim : limits) {
    dists.emplace_back(lim.first, lim.second);
  }

  // Same restart bookkeeping as InverseKinematicsSolver::Solve: count every
  // attempt made, not just whichever one produced the best result.
  int restarts_attempted = 0;
  for (int r = 0; r < options_.max_restarts && !best.converged; ++r) {
    std::vector<double> restart_seed(limits.size());
    for (size_t i = 0; i < limits.size(); ++i) {
      restart_seed[i] = dists[i](rng_);
    }
    PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
    ++restarts_attempted;
    if (candidate.converged || candidate.residual.head<3>().norm() < best.residual.head<3>().norm()) {
      best = candidate;
    }
  }
  best.restarts_used = restarts_attempted;
  return best;
}

PositionPitchIkResult PositionPitchIkSolver::SolveFromSeed(const Eigen::Isometry3d& target_pose,
                                                             const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  const size_t n = seed.size();
  std::vector<double> q = seed;

  size_t m = 0;
  if (options_.nullspace_secondary_link_solver != nullptr && options_.nullspace_gain != 0.0) {
    m = options_.nullspace_secondary_link_solver->GetActuatedJointNames().size();
    if (m > n) {
      throw std::invalid_argument(
          "PositionPitchIkSolver: nullspace_secondary_link_solver has more actuated joints than "
          "the chain -- it must be an upstream sub-chain of solver's");
    }
  }

  Eigen::Matrix<double, 6, 1> residual = Eigen::Matrix<double, 6, 1>::Zero();
  int iterations = 0;
  bool converged = false;

  for (; iterations < options_.max_iterations; ++iterations) {
    Eigen::Isometry3d pose = solver_.ComputePose(q);
    Eigen::Vector3d pitch_axis = pose.rotation() * Eigen::Vector3d::UnitY();

    Eigen::Vector3d position_error = target_pose.translation() - pose.translation();
    Eigen::Vector3d rotation_error = RotationVector(target_pose.rotation() * pose.rotation().transpose());
    double pitch_error = pitch_axis.dot(rotation_error);

    residual.head<3>() = position_error;
    residual.tail<3>() = rotation_error;  // full 3-component rotation error, informational (see header)

    if (position_error.norm() < options_.position_tolerance &&
        std::abs(pitch_error) < options_.pitch_tolerance) {
      converged = true;
      break;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> J = ComputeGeometricJacobian(solver_, q);
    Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
    J_task.topRows<3>() = J.topRows<3>();
    J_task.row(3) = pitch_axis.transpose() * J.bottomRows<3>();

    Eigen::Vector4d e_task(position_error.x(), position_error.y(), position_error.z(), pitch_error);

    Eigen::VectorXd qdot0;
    bool have_nullspace_bias = false;
    if (m > 0) {
      std::vector<double> q_prefix(q.begin(), q.begin() + m);
      Eigen::Vector3d link_axis =
          options_.nullspace_secondary_link_solver->ComputePose(q_prefix).rotation() *
          Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
      qdot0 = Eigen::VectorXd::Zero(n);
      for (size_t i = 0; i < m; ++i) {
        Eigen::Vector3d z_i = J.block<3, 1>(3, i);
        qdot0(i) = options_.nullspace_gain * world_up.dot(z_i.cross(link_axis));
      }
      have_nullspace_bias = true;
    }

    Eigen::VectorXd dq = detail::SolveDampedRightPseudoInverse(
        J_task, e_task, options_.damping, have_nullspace_bias ? &qdot0 : nullptr);

    double step_norm = dq.lpNorm<Eigen::Infinity>();
    if (step_norm > options_.max_step) {
      dq *= (options_.max_step / step_norm);
    }

    for (size_t i = 0; i < n; ++i) {
      q[i] = std::min(std::max(q[i] + dq(i), limits[i].first), limits[i].second);
    }
  }

  if (!converged) {
    // `residual` above is computed at the START of each iteration (before
    // that iteration's step), so if the loop ran out of iterations rather
    // than breaking on convergence, it's stale by one step relative to the
    // `q` just updated in the loop's final pass -- recompute once more so
    // the returned residual always matches the returned joint_positions,
    // not the position one step behind them.
    Eigen::Isometry3d pose = solver_.ComputePose(q);
    residual.head<3>() = target_pose.translation() - pose.translation();
    residual.tail<3>() = RotationVector(target_pose.rotation() * pose.rotation().transpose());
  }

  PositionPitchIkResult result;
  result.joint_positions = q;
  result.residual = residual;
  result.iterations = iterations;
  result.converged = converged;
  result.restarts_used = 0;  // caller (Solve) fills this in
  return result;
}

}  // namespace arm_kinematics
