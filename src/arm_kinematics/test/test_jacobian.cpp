// Cross-checks ComputeGeometricJacobian (the analytical formula this
// package ships) against a numerical (central finite-difference)
// Jacobian of ForwardKinematicsSolver::ComputePose -- a second,
// independent computation of the same quantity, not a re-check of the
// same code path. Catches sign/axis-order derivation mistakes that a
// self-consistency test against the same formula never could.
//
// Finite-difference derivation for the angular block (see
// docs/inverse_kinematics.md): perturbing only joint i by +-h and
// linearizing, R(q+h e_i) ~= Exp(h z_i) R(q) and R(q-h e_i) ~= Exp(-h z_i)
// R(q), so R(q+h e_i)*R(q-h e_i)^T ~= Exp(2h z_i), and its rotation vector
// divided by 2h recovers z_i directly -- independent of the analytical
// formula, using only ComputePose and the standard axis-angle log map.

#include <gtest/gtest.h>

#include <memory>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/jacobian.h"

namespace {

constexpr double kFdStep = 1e-6;
constexpr double kTol = 1e-5;  // finite-difference truncation error dominates

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

Eigen::Matrix<double, 6, Eigen::Dynamic> NumericalJacobian(
    const arm_kinematics::ForwardKinematicsSolver& solver, std::vector<double> q) {
  const size_t n = q.size();
  Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n);
  for (size_t i = 0; i < n; ++i) {
    std::vector<double> q_plus = q, q_minus = q;
    q_plus[i] += kFdStep;
    q_minus[i] -= kFdStep;

    Eigen::Isometry3d pose_plus = solver.ComputePose(q_plus);
    Eigen::Isometry3d pose_minus = solver.ComputePose(q_minus);

    J.block<3, 1>(0, i) = (pose_plus.translation() - pose_minus.translation()) / (2.0 * kFdStep);
    J.block<3, 1>(3, i) =
        RotationVector(pose_plus.rotation() * pose_minus.rotation().transpose()) / (2.0 * kFdStep);
  }
  return J;
}

}  // namespace

class JacobianTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                         "link_5");
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
};

TEST_F(JacobianTest, MatchesFiniteDifferenceAtZeroConfig) {
  std::vector<double> q = {0.0, 0.0, 0.0, 0.0, 0.0};
  auto J_analytical = arm_kinematics::ComputeGeometricJacobian(*solver_, q);
  auto J_numerical = NumericalJacobian(*solver_, q);
  EXPECT_LT((J_analytical - J_numerical).norm(), kTol);
}

TEST_F(JacobianTest, MatchesFiniteDifferenceAtSeveralConfigs) {
  const std::vector<std::vector<double>> configs = {
      {0.3, 0.5, -0.2, 0.4, -0.3},
      {1.0, 1.5, -1.0, 2.0, -2.5},
      {-2.0, 3.0, -2.0, 4.0, -3.0},
      {0.0, 3.14, -2.220, 4.505, -3.62},  // near joint limits
  };
  for (const auto& q : configs) {
    auto J_analytical = arm_kinematics::ComputeGeometricJacobian(*solver_, q);
    auto J_numerical = NumericalJacobian(*solver_, q);
    EXPECT_LT((J_analytical - J_numerical).norm(), kTol) << "config: q0=" << q[0];
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
