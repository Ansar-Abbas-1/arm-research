// Unit tests for ForwardKinematicsSolver against independently-computed
// reference values (see docs/forward_kinematics.md for the derivation and
// the numpy script used to cross-check them -- a second, independent
// implementation of the same math, not a reuse of this package's code).
//
// Loads the real arm_interface/urdf/arm.urdf (plain URDF, no xacro) via
// ros::package::getPath, so this test validates against the actual robot
// this package targets rather than a hand-copied fixture that could drift
// from it.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"

namespace {

constexpr double kTightTol = 1e-9;  // exact analytic comparison

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  if (arm_interface_path.empty()) {
    ADD_FAILURE() << "arm_interface package not found on ROS_PACKAGE_PATH";
    return model;
  }
  bool ok = model.initFile(arm_interface_path + "/urdf/arm.urdf");
  EXPECT_TRUE(ok) << "failed to parse arm_interface/urdf/arm.urdf";
  return model;
}

void ExpectPoseNear(const Eigen::Isometry3d& actual, const Eigen::Vector3d& expected_pos,
                     const Eigen::Matrix3d& expected_rot, double tol) {
  EXPECT_NEAR((actual.translation() - expected_pos).norm(), 0.0, tol);
  EXPECT_NEAR((actual.linear() - expected_rot).norm(), 0.0, tol);
}

}  // namespace

class ForwardKinematicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                         "link_5");
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
};

TEST_F(ForwardKinematicsTest, ActuatedJointNamesExcludeFixedIntermediate) {
  // The chain is base_link -> ... -> link_intermediate -> [fixed
  // joint_intermediate] -> link_3 -> ... -> link_5: 5 actuated joints,
  // joint_intermediate must NOT appear (it's the easy-to-drop case this
  // milestone's plan flagged).
  const auto& names = solver_->GetActuatedJointNames();
  ASSERT_EQ(names.size(), 5u);
  EXPECT_EQ(names[0], "joint_1");
  EXPECT_EQ(names[1], "joint_2");
  EXPECT_EQ(names[2], "joint_3");
  EXPECT_EQ(names[3], "joint_4");
  EXPECT_EQ(names[4], "joint_5");
}

TEST_F(ForwardKinematicsTest, ZeroConfigMatchesIndependentReference) {
  // Reference computed independently in Python/numpy (fk_reference.py,
  // same equations as docs/forward_kinematics.md), not derived from this
  // C++ implementation.
  Eigen::Vector3d expected_pos(-0.065804339, 0.0, 0.444317584);
  Eigen::Matrix3d expected_rot;
  expected_rot << 0.068642308, 0.0, 0.997641335, 0.0, 1.0, 0.0, -0.997641335, 0.0, 0.068642308;

  auto pose = solver_->ComputePose({0.0, 0.0, 0.0, 0.0, 0.0});
  ExpectPoseNear(pose, expected_pos, expected_rot, 1e-6);
}

TEST_F(ForwardKinematicsTest, MultiJointConfigMatchesIndependentReference) {
  Eigen::Vector3d expected_pos(0.189062287, 0.058483819, 0.48518007);
  Eigen::Matrix3d expected_rot;
  expected_rot << -0.310748111, -0.295520207, 0.903384425, -0.096125655, 0.955336489, 0.27944955,
      -0.945619094, 0.0, -0.325276083;

  auto pose = solver_->ComputePose({0.3, 0.5, -0.2, 0.4, -0.3});
  ExpectPoseNear(pose, expected_pos, expected_rot, 1e-6);
}

TEST_F(ForwardKinematicsTest, Joint1RotatesWholeChainAboutBaseZ) {
  // joint_1's axis is z and it is the first transform in the chain, so
  // rotating it by theta must be equivalent to left-multiplying the
  // theta=0 pose by Rz(theta), for ANY values of joints 2-5. This is a
  // hand-provable invariant of the chain structure, independent of the
  // exact numeric geometry -- if the implementation applied joint_1 out of
  // order, or on the wrong side of the product, this fails.
  const std::vector<double> other_joints = {0.4, -0.6, 0.7, -0.1};
  for (double theta : {0.0, 0.3, -1.2, M_PI / 2, 2.0}) {
    std::vector<double> q_theta = {theta};
    q_theta.insert(q_theta.end(), other_joints.begin(), other_joints.end());
    std::vector<double> q_zero = {0.0};
    q_zero.insert(q_zero.end(), other_joints.begin(), other_joints.end());

    auto pose_theta = solver_->ComputePose(q_theta);
    auto pose_zero = solver_->ComputePose(q_zero);
    Eigen::Isometry3d expected = Eigen::Isometry3d(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ())) * pose_zero;

    EXPECT_NEAR((pose_theta.translation() - expected.translation()).norm(), 0.0, kTightTol)
        << "theta=" << theta;
    EXPECT_NEAR((pose_theta.linear() - expected.linear()).norm(), 0.0, kTightTol)
        << "theta=" << theta;
  }
}

TEST_F(ForwardKinematicsTest, WrongInputSizeThrows) {
  EXPECT_THROW(solver_->ComputePose({0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(solver_->ComputePose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

TEST(ForwardKinematicsConstruction, UnreachableTipLinkThrows) {
  urdf::Model model = LoadArmModel();
  EXPECT_THROW(arm_kinematics::ForwardKinematicsSolver(model, "base_link", "not_a_real_link"),
               std::runtime_error);
}

TEST(ForwardKinematicsConstruction, EndEffectorChainAddsFixedOffsetPastLink5) {
  // joint_6 (link_5 -> end_effector) is FIXED (translation-only, per the
  // URDF fix made when this milestone's IK target turned out to need
  // end_effector, not link_5 -- see arm.urdf's comment on joint_6). This
  // wasn't previously exercised: every other test's tip_link ended right
  // after an ACTUATED joint. Confirms a chain ending after a FIXED joint
  // still (a) excludes it from the actuated joint list, and (b) applies
  // its constant offset correctly on top of the link_5 pose.
  urdf::Model model = LoadArmModel();
  arm_kinematics::ForwardKinematicsSolver link5_solver(model, "base_link", "link_5");
  arm_kinematics::ForwardKinematicsSolver ee_solver(model, "base_link", "end_effector");

  EXPECT_EQ(ee_solver.GetActuatedJointNames(), link5_solver.GetActuatedJointNames());

  const std::vector<std::vector<double>> configs = {
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.3, 0.5, -0.2, 0.4, -0.3},
      {1.0, 1.5, -1.0, 2.0, -2.5},
  };
  for (const auto& q : configs) {
    Eigen::Isometry3d link5_pose = link5_solver.ComputePose(q);
    Eigen::Isometry3d ee_pose = ee_solver.ComputePose(q);

    // Position offset is exactly (0,0,0.105) along link_5's own z-axis --
    // no rotation, since joint_6 is fixed now (not revolute).
    Eigen::Vector3d expected_ee_pos = link5_pose.translation() + link5_pose.rotation() * Eigen::Vector3d(0, 0, 0.105);
    EXPECT_NEAR((ee_pose.translation() - expected_ee_pos).norm(), 0.0, kTightTol);
    // Orientation is unaffected by the fixed offset.
    EXPECT_NEAR((ee_pose.linear() - link5_pose.linear()).norm(), 0.0, kTightTol);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
