#include <gtest/gtest.h>

#include "arm_interface/motor_status.h"

using arm_interface::DecodeHardwareError;
using arm_interface::DecodeOperatingMode;
using arm_interface::JoinNames;

TEST(MotorStatus, NoErrorDecodesToEmpty) {
  EXPECT_TRUE(DecodeHardwareError(0).empty());
  EXPECT_EQ(JoinNames(DecodeHardwareError(0)), "none");
}

TEST(MotorStatus, EachBitMapsToItsName) {
  EXPECT_EQ(DecodeHardwareError(0x01), (std::vector<std::string>{"input voltage"}));
  EXPECT_EQ(DecodeHardwareError(0x04), (std::vector<std::string>{"overheating"}));
  EXPECT_EQ(DecodeHardwareError(0x08), (std::vector<std::string>{"motor encoder"}));
  EXPECT_EQ(DecodeHardwareError(0x10), (std::vector<std::string>{"electrical shock"}));
  EXPECT_EQ(DecodeHardwareError(0x20), (std::vector<std::string>{"overload"}));
}

TEST(MotorStatus, MultipleBitsAreAllReported) {
  // 0x24 = overheating + overload (bits 2 and 5)
  EXPECT_EQ(JoinNames(DecodeHardwareError(0x24)), "overheating, overload");
}

TEST(MotorStatus, OperatingModes) {
  EXPECT_EQ(DecodeOperatingMode(3), "position");
  EXPECT_EQ(DecodeOperatingMode(1), "velocity");
  EXPECT_EQ(DecodeOperatingMode(4), "extended position");
  EXPECT_EQ(DecodeOperatingMode(9), "unknown (9)");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
