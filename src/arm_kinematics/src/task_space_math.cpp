#include "task_space_math.h"

#include <algorithm>
#include <cmath>

namespace arm_kinematics {
namespace detail {

Eigen::VectorXd SolveDampedRightPseudoInverse(const Eigen::Matrix<double, 4, Eigen::Dynamic>& J_task,
                                               const Eigen::Vector4d& rhs, double damping_threshold,
                                               const Eigen::VectorXd* nullspace_qdot0) {
  const Eigen::Index n = J_task.cols();

  Eigen::Matrix4d gram0 = J_task * J_task.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(gram0, Eigen::EigenvaluesOnly);
  double sigma_min = std::sqrt(std::max(0.0, eigensolver.eigenvalues()(0)));
  double lambda_sq = 0.0;
  if (sigma_min < damping_threshold) {
    double ratio = sigma_min / damping_threshold;
    lambda_sq = damping_threshold * damping_threshold * (1.0 - ratio * ratio);
  }
  Eigen::Matrix4d gram = gram0 + lambda_sq * Eigen::Matrix4d::Identity();
  auto gram_ldlt = gram.ldlt();

  Eigen::VectorXd result = J_task.transpose() * gram_ldlt.solve(rhs);

  if (nullspace_qdot0 != nullptr) {
    Eigen::MatrixXd J_pinv = J_task.transpose() * gram_ldlt.solve(Eigen::Matrix4d::Identity());
    Eigen::MatrixXd null_projector = Eigen::MatrixXd::Identity(n, n) - J_pinv * J_task;
    result += null_projector * (*nullspace_qdot0);
  }

  return result;
}

}  // namespace detail
}  // namespace arm_kinematics
