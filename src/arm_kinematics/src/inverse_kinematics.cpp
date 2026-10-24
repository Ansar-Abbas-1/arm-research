#include "arm_kinematics/inverse_kinematics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "arm_kinematics/jacobian.h"
#include "arm_kinematics/joint_limit_sampling.h"

namespace arm_kinematics {

namespace {

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

Eigen::Matrix<double, 6, 1> PoseError(const Eigen::Isometry3d& target,
                                       const Eigen::Isometry3d& current) {
  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = target.translation() - current.translation();
  e.tail<3>() = RotationVector(target.rotation() * current.rotation().transpose());
  return e;
}

bool IsConverged(const Eigen::Matrix<double, 6, 1>& e, const IkOptions& options) {
  return e.head<3>().norm() < options.position_tolerance &&
         e.tail<3>().norm() < options.orientation_tolerance;
}

double WeightedCost(const Eigen::Matrix<double, 6, 1>& e, const IkOptions& options) {
  return e.dot(options.weights.asDiagonal() * e);
}

}  // namespace

InverseKinematicsSolver::InverseKinematicsSolver(const ForwardKinematicsSolver& solver,
                                                   IkOptions options)
    : solver_(solver), options_(options), rng_(options.rng_seed) {}

IkResult InverseKinematicsSolver::Solve(const Eigen::Isometry3d& target_pose,
                                         const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  if (seed.size() != limits.size()) {
    throw std::invalid_argument("InverseKinematicsSolver::Solve: seed has " +
                                 std::to_string(seed.size()) + " values, expected " +
                                 std::to_string(limits.size()));
  }

  IkResult best = SolveFromSeed(target_pose, seed);

  std::vector<std::uniform_real_distribution<double>> dists = DistributionsFromJointLimits(limits);

  // restarts_used tracks how many restarts were actually *attempted*, not
  // which one happened to produce the best result -- those can differ: a
  // later restart can fail to improve on an earlier one's result while
  // still having been tried, and the caller cares about total effort
  // spent, not just the winning attempt's index.
  int restarts_attempted = 0;
  for (int r = 0; r < options_.max_restarts && !best.converged; ++r) {
    std::vector<double> restart_seed(limits.size());
    for (size_t i = 0; i < limits.size(); ++i) {
      restart_seed[i] = dists[i](rng_);
    }
    IkResult candidate = SolveFromSeed(target_pose, restart_seed);
    ++restarts_attempted;
    if (candidate.converged || candidate.residual.norm() < best.residual.norm()) {
      best = candidate;
    }
  }
  best.restarts_used = restarts_attempted;
  return best;
}

IkResult InverseKinematicsSolver::SolveFromSeed(const Eigen::Isometry3d& target_pose,
                                                  const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  std::vector<double> q = seed;
  const size_t n = q.size();
  Eigen::Matrix<double, 6, 1> e = PoseError(target_pose, solver_.ComputePose(q));
  double cost = WeightedCost(e, options_);
  double lambda = options_.initial_damping;
  int iterations = 0;

  for (; iterations < options_.max_iterations; ++iterations) {
    if (IsConverged(e, options_)) {
      break;
    }

    bool step_accepted = false;
    for (int inner = 0; inner < options_.max_inner_retries; ++inner) {
      Eigen::Matrix<double, 6, Eigen::Dynamic> J = ComputeGeometricJacobian(solver_, q);
      // Left pseudo-inverse (normal-equations) damped least squares:
      // Dq = (J^T W J + lambda^2 I)^-1 . J^T W . e -- see class comment /
      // docs/inverse_kinematics.md for why this form (not the redundant-
      // manipulator right-pseudo-inverse one) applies here.
      Eigen::MatrixXd JtW = J.transpose() * options_.weights.asDiagonal();
      Eigen::MatrixXd H = JtW * J + (lambda * lambda) * Eigen::MatrixXd::Identity(n, n);
      Eigen::VectorXd dq = H.ldlt().solve(JtW * e);

      double step_norm = dq.lpNorm<Eigen::Infinity>();
      if (step_norm > options_.max_step) {
        dq *= (options_.max_step / step_norm);
      }

      std::vector<double> q_candidate = q;
      for (size_t i = 0; i < n; ++i) {
        q_candidate[i] = std::min(std::max(q[i] + dq(i), limits[i].first), limits[i].second);
      }
      Eigen::Matrix<double, 6, 1> e_candidate =
          PoseError(target_pose, solver_.ComputePose(q_candidate));
      double cost_candidate = WeightedCost(e_candidate, options_);

      if (cost_candidate < cost) {
        q = q_candidate;
        e = e_candidate;
        cost = cost_candidate;
        lambda = std::max(lambda * options_.damping_decrease_factor, options_.damping_min);
        step_accepted = true;
        break;
      }
      lambda = std::min(lambda * options_.damping_increase_factor, options_.damping_max);
    }

    if (!step_accepted) {
      // No improving step found even at max damping within the inner
      // retry budget: a local minimum (or the retry budget was too
      // small), not something more outer iterations alone would fix.
      // Stop rather than burn the remaining budget doing nothing.
      break;
    }
  }

  IkResult result;
  result.joint_positions = q;
  result.residual = e;
  result.iterations = iterations;
  result.converged = IsConverged(e, options_);
  result.restarts_used = 0;  // caller (Solve) fills this in
  return result;
}

}  // namespace arm_kinematics
