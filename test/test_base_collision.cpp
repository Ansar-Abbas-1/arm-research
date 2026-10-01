#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/base_collision.h"
#include "arm_kinematics/forward_kinematics.h"

using arm_kinematics::ArmIntersectsBase;
using arm_kinematics::BoxObstacle;
using arm_kinematics::CapsuleIntersectsBox;
using arm_kinematics::ForwardKinematicsSolver;

namespace {

// Plain box, half-extents (1,1,1), centered at the origin -- easy hand
// arithmetic for every case below.
BoxObstacle UnitCube() {
  BoxObstacle box;
  box.half_extents = Eigen::Vector3d(1.0, 1.0, 1.0);
  return box;
}

}  // namespace

TEST(CapsuleIntersectsBoxTest, SegmentFarOutsideDoesNotIntersect) {
  BoxObstacle box = UnitCube();
  EXPECT_FALSE(CapsuleIntersectsBox(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(6, 6, 6), 0.1, box));
}

TEST(CapsuleIntersectsBoxTest, SegmentThroughCenterIntersects) {
  BoxObstacle box = UnitCube();
  EXPECT_TRUE(CapsuleIntersectsBox(Eigen::Vector3d(-2, 0, 0), Eigen::Vector3d(2, 0, 0), 0.1, box));
}

TEST(CapsuleIntersectsBoxTest, EndpointInsideBoxIntersects) {
  BoxObstacle box = UnitCube();
  EXPECT_TRUE(CapsuleIntersectsBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(5, 5, 5), 0.1, box));
}

// Segment at y=1.5 (0.5 above the box's y=1 face), x in [0,5], z=0. For
// x in [0,1] the closest box point is (x,1,0) -- pure y-gap of 0.5,
// independent of x there -- so the segment's true minimum distance to the
// box is exactly 0.5, hand-derived, not guessed.
TEST(CapsuleIntersectsBoxTest, ClosestApproachExactlyAtRadiusIsBoundaryTrue) {
  BoxObstacle box = UnitCube();
  Eigen::Vector3d p0(0, 1.5, 0), p1(5, 1.5, 0);
  EXPECT_TRUE(CapsuleIntersectsBox(p0, p1, 0.5, box));   // distance == radius -> boundary, true
  EXPECT_FALSE(CapsuleIntersectsBox(p0, p1, 0.4, box));  // distance (0.5) > radius -> false
}

// Segment at x=2, y=2 (both axes overshooting simultaneously -- a corner
// case, not a single-face case), z in [0,5]. For z in [0,1] (overlapping
// the box's own z in [-1,1]) the closest box point is the corner (1,1,z):
// distance^2 = (2-1)^2 + (2-1)^2 = 2, independent of z there. Exercises
// the 2-simultaneously-active-axes branch of the breakpoint logic.
TEST(CapsuleIntersectsBoxTest, CornerCaseWithTwoActiveAxes) {
  BoxObstacle box = UnitCube();
  Eigen::Vector3d p0(2, 2, 0), p1(2, 2, 5);
  EXPECT_TRUE(CapsuleIntersectsBox(p0, p1, 1.5, box));   // 1.5^2=2.25 > 2 -> intersects
  EXPECT_FALSE(CapsuleIntersectsBox(p0, p1, 1.0, box));  // 1.0^2=1 < 2 -> does not
}

TEST(CapsuleIntersectsBoxTest, ZeroLengthSegmentInsideIntersects) {
  BoxObstacle box = UnitCube();
  EXPECT_TRUE(CapsuleIntersectsBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), 0.1, box));
}

TEST(CapsuleIntersectsBoxTest, ZeroLengthSegmentOutsideDoesNotIntersect) {
  BoxObstacle box = UnitCube();
  EXPECT_FALSE(CapsuleIntersectsBox(Eigen::Vector3d(5, 5, 5), Eigen::Vector3d(5, 5, 5), 0.1, box));
}

namespace {

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

// Box as actually deployed (see trajectory_service_node.cpp): top face at
// base_link's own origin, extending downward.
BoxObstacle DeployedBaseBox(double half_extent = 0.20) {
  BoxObstacle box;
  box.half_extents = Eigen::Vector3d::Constant(half_extent);
  box.box_to_root = Eigen::Translation3d(0, 0, -half_extent) * Eigen::Isometry3d::Identity();
  return box;
}

}  // namespace

class ArmIntersectsBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<ForwardKinematicsSolver>(model_, "base_link", "end_effector");
  }

  urdf::Model model_;
  std::unique_ptr<ForwardKinematicsSolver> solver_;
};

// Regression guard: frame[0] (joint_1's own origin, (0,0,0.03)) and
// frame[1] (joint_2's origin, (0,0,0.095)) are both invariant to every
// joint angle (see base_collision.h's own comment) and, with the deployed
// box's top face at z=0, both sit just ABOVE it -- never inside. This is
// exactly what lets first_checked_segment_index default to 1. A
// near-zero-angle configuration should NOT be flagged.
TEST_F(ArmIntersectsBaseTest, NearZeroConfigurationDoesNotCollideWithDeployedBox) {
  BoxObstacle box = DeployedBaseBox();
  std::vector<double> q = {0.01, 0.5, -0.5, 0.5, -0.5};  // avoid the exact {0,0,0,0,0} singularity
  EXPECT_FALSE(ArmIntersectsBase(*solver_, q, box, 0.02));
}

TEST_F(ArmIntersectsBaseTest, NormalOutstretchedConfigurationDoesNotCollide) {
  BoxObstacle box = DeployedBaseBox();
  std::vector<double> q = {0.0, 1.5708, -0.5, 1.5708, -1.5708};
  EXPECT_FALSE(ArmIntersectsBase(*solver_, q, box, 0.02));
}

// q2 swung to its lower limit folds link_2 straight down: frame[2] lands at
// (0.0097, 0, -0.2449) -- inside the box's x/y half-extents and well within
// its z in [-0.40, 0] -- so segment 1 (frame1->frame2) demonstrably crosses
// the top face. Verified via a standalone probe against the built library
// before being hand-asserted here, not guessed.
TEST_F(ArmIntersectsBaseTest, ElbowFoldedIntoBaseCollides) {
  BoxObstacle box = DeployedBaseBox();
  std::vector<double> q = {0.0, -1.6, 0.0, 0.0, 0.0};
  EXPECT_TRUE(ArmIntersectsBase(*solver_, q, box, 0.02));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
