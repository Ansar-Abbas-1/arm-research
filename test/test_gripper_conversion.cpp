#include <gtest/gtest.h>

#include "arm_interface/gripper_conversion.h"

using arm_interface::GripperConversion;

namespace {

// Real joint_6 numbers: rotational range derived from
// config/ArmInterface.yaml's bias/sign/encoder_min/max (index 5),
// linear range from urdf/arm.urdf's joint_6 <limit>.
GripperConversion RealJoint6Conversion() {
  return GripperConversion{-1.2916, 1.2824, 0.0, 0.025};
}

}  // namespace

TEST(GripperConversionTest, EndpointsMapToEndpoints) {
  GripperConversion c = RealJoint6Conversion();
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_min), c.linear_min, 1e-9);
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_max), c.linear_max, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_min), c.rotational_min, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_max), c.rotational_max, 1e-9);
}

TEST(GripperConversionTest, MidpointMapsToMidpoint) {
  GripperConversion c = RealJoint6Conversion();
  double rot_mid = 0.5 * (c.rotational_min + c.rotational_max);
  double lin_mid = 0.5 * (c.linear_min + c.linear_max);
  EXPECT_NEAR(c.RotationalToLinear(rot_mid), lin_mid, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(lin_mid), rot_mid, 1e-9);
}

TEST(GripperConversionTest, RoundTripIsIdentity) {
  GripperConversion c = RealJoint6Conversion();
  for (double rot : {-1.0, -0.3, 0.0, 0.5, 1.1}) {
    EXPECT_NEAR(c.LinearToRotational(c.RotationalToLinear(rot)), rot, 1e-9);
  }
}

TEST(GripperConversionTest, OutOfRangeInputsClampRatherThanExtrapolate) {
  GripperConversion c = RealJoint6Conversion();
  // A motor reading transiently outside its own configured encoder bounds
  // (startup, slop, a glitchy read) must not produce a linear value
  // outside the kinematic joint's own valid [linear_min, linear_max].
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_min - 1.0), c.linear_min, 1e-9);
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_max + 1.0), c.linear_max, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_min - 1.0), c.rotational_min, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_max + 1.0), c.rotational_max, 1e-9);
}

TEST(GripperConversionTest, VelocityConversionIsConstantSlope) {
  GripperConversion c = RealJoint6Conversion();
  double slope = (c.rotational_max - c.rotational_min) / (c.linear_max - c.linear_min);
  EXPECT_NEAR(c.LinearToRotationalVelocity(1.0), slope, 1e-9);
  EXPECT_NEAR(c.LinearToRotationalVelocity(0.0), 0.0, 1e-9);
  EXPECT_NEAR(c.RotationalToLinearVelocity(slope), 1.0, 1e-9);
}

// Confirmed live (2026-10-01): the real gripper's open/close sense was
// opposite arm_sim_validator_node's with invert=false -- config/
// ArmInterface.yaml now sets gripper_invert: true to match. These cases
// guard that fix: endpoints swap which linear value they map to, but
// round-tripping and clamping behavior stay correct either way.
TEST(GripperConversionTest, InvertSwapsWhichEndpointMapsToWhichLinearValue) {
  GripperConversion c = RealJoint6Conversion();
  c.invert = true;
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_min), c.linear_max, 1e-9);
  EXPECT_NEAR(c.RotationalToLinear(c.rotational_max), c.linear_min, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_min), c.rotational_max, 1e-9);
  EXPECT_NEAR(c.LinearToRotational(c.linear_max), c.rotational_min, 1e-9);
}

TEST(GripperConversionTest, InvertedRoundTripIsStillIdentity) {
  GripperConversion c = RealJoint6Conversion();
  c.invert = true;
  for (double rot : {-1.0, -0.3, 0.0, 0.5, 1.1}) {
    EXPECT_NEAR(c.LinearToRotational(c.RotationalToLinear(rot)), rot, 1e-9);
  }
}

TEST(GripperConversionTest, InvertedVelocityConversionFlipsSign) {
  GripperConversion c = RealJoint6Conversion();
  c.invert = true;
  double slope = (c.rotational_max - c.rotational_min) / (c.linear_max - c.linear_min);
  EXPECT_NEAR(c.LinearToRotationalVelocity(1.0), -slope, 1e-9);
  EXPECT_NEAR(c.RotationalToLinearVelocity(-slope), 1.0, 1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
