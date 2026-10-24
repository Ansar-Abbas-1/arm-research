#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "arm_kinematics/joint_velocity_cap.h"

using arm_kinematics::CapJointVelocityLimits;

namespace {
const std::vector<double> kLimits = {2.0, 1.0, 1.5};
}

TEST(JointVelocityCap, EmptyOrNonPositiveScalarLeavesLimitsUnchanged) {
  EXPECT_EQ(CapJointVelocityLimits(kLimits, {}), kLimits);
  EXPECT_EQ(CapJointVelocityLimits(kLimits, {0.0}), kLimits);
  EXPECT_EQ(CapJointVelocityLimits(kLimits, {-1.0}), kLimits);
}

TEST(JointVelocityCap, ScalarAppliesToEveryJointAsMinimum) {
  std::vector<double> out = CapJointVelocityLimits(kLimits, {1.2});
  EXPECT_DOUBLE_EQ(out[0], 1.2);
  EXPECT_DOUBLE_EQ(out[1], 1.0);  // already lower than the cap
  EXPECT_DOUBLE_EQ(out[2], 1.2);
}

TEST(JointVelocityCap, PerJointListIsAppliedPerJoint) {
  std::vector<double> out = CapJointVelocityLimits(kLimits, {0.5, 0.25, 1.0});
  EXPECT_DOUBLE_EQ(out[0], 0.5);
  EXPECT_DOUBLE_EQ(out[1], 0.25);
  EXPECT_DOUBLE_EQ(out[2], 1.0);
}

TEST(JointVelocityCap, CapAboveLimitIsClampedAndReported) {
  std::vector<size_t> warn;
  std::vector<double> out = CapJointVelocityLimits(kLimits, {5.0, 5.0, 1.0}, &warn);
  EXPECT_EQ(out, (std::vector<double>{2.0, 1.0, 1.0}));
  EXPECT_EQ(warn, (std::vector<size_t>{0, 1}));
}

TEST(JointVelocityCap, RejectsWrongSizeAndNonPositiveEntries) {
  EXPECT_THROW(CapJointVelocityLimits(kLimits, {1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(CapJointVelocityLimits(kLimits, {1.0, 0.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(CapJointVelocityLimits(kLimits, {1.0, -0.5, 1.0}), std::invalid_argument);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
