#include <gtest/gtest.h>

#include "arm_trajectory/estop_monitor.h"

using arm_trajectory::EStopStatus;
using arm_trajectory::EStopView;

TEST(EStopView, StartsUnknown) {
  EStopView v;
  EXPECT_EQ(v.status(), EStopStatus::kUnknown);
}

TEST(EStopView, UnknownFailsClosedUnlessStateIsNotRequired) {
  EStopView v;
  EXPECT_FALSE(v.AllowsMotion(true));
  EXPECT_NE(v.Reason(true).find("unknown"), std::string::npos);
  EXPECT_TRUE(v.AllowsMotion(false));
  EXPECT_TRUE(v.Reason(false).empty());
}

TEST(EStopView, ReleasedAllowsMotion) {
  EStopView v;
  v.OnState(false);
  EXPECT_EQ(v.status(), EStopStatus::kReleased);
  EXPECT_TRUE(v.AllowsMotion(true));
  EXPECT_TRUE(v.AllowsMotion(false));
  EXPECT_TRUE(v.Reason(true).empty());
}

TEST(EStopView, EngagedBlocksMotionRegardlessOfTheRequireFlag) {
  EStopView v;
  v.OnState(true);
  EXPECT_FALSE(v.AllowsMotion(true));
  EXPECT_FALSE(v.AllowsMotion(false));
  EXPECT_NE(v.Reason(false).find("engaged"), std::string::npos);
}

TEST(EStopView, ResetReturnsToReleasedAndNeverBackToUnknown) {
  EStopView v;
  v.OnState(true);
  v.OnState(false);
  EXPECT_EQ(v.status(), EStopStatus::kReleased);
  EXPECT_TRUE(v.AllowsMotion(true));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
