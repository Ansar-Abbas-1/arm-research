#include <gtest/gtest.h>

#include "arm_interface/estop.h"

using arm_interface::EStopLatch;

TEST(EStop, StartsReleasedAndAllowsEverything) {
  EStopLatch e;
  EXPECT_FALSE(e.latched());
  EXPECT_TRUE(e.CommandsAllowed());
  EXPECT_TRUE(e.TorqueDisableAllowed());
  EXPECT_FALSE(e.HoldPending());
}

TEST(EStop, TriggerLatchesBlocksCommandsAndTorqueDisable) {
  EStopLatch e;
  EXPECT_TRUE(e.Trigger());
  EXPECT_TRUE(e.latched());
  EXPECT_FALSE(e.CommandsAllowed());
  EXPECT_FALSE(e.TorqueDisableAllowed());  // a stray set_torque(false) must not drop the lock
}

TEST(EStop, HoldIsPendingUntilActuallyApplied) {
  EStopLatch e;
  e.Trigger();
  EXPECT_TRUE(e.HoldPending());
  EXPECT_FALSE(e.hold_applied());
  e.MarkHoldApplied(false);  // write failed: the loop must retry
  EXPECT_TRUE(e.HoldPending());
  e.MarkHoldApplied(true);
  EXPECT_FALSE(e.HoldPending());
  EXPECT_TRUE(e.hold_applied());
}

TEST(EStop, TriggerIsIdempotentAndDoesNotDiscardAnAppliedHold) {
  EStopLatch e;
  EXPECT_TRUE(e.Trigger());
  e.MarkHoldApplied(true);
  EXPECT_FALSE(e.Trigger());  // second trigger is a no-op
  EXPECT_TRUE(e.hold_applied());
}

TEST(EStop, ResetReleasesAndRequiresAFreshHoldNextTime) {
  EStopLatch e;
  e.Trigger();
  e.MarkHoldApplied(true);
  EXPECT_TRUE(e.Reset());
  EXPECT_FALSE(e.latched());
  EXPECT_TRUE(e.CommandsAllowed());
  EXPECT_FALSE(e.hold_applied());
  EXPECT_TRUE(e.Trigger());
  EXPECT_TRUE(e.HoldPending());  // the earlier hold does not carry over
}

TEST(EStop, ResetWhenNotLatchedIsReportedAsSuch) {
  EStopLatch e;
  EXPECT_FALSE(e.Reset());
}

TEST(EStop, MarkHoldAppliedIsIgnoredWhenNotLatched) {
  EStopLatch e;
  e.MarkHoldApplied(true);
  EXPECT_FALSE(e.hold_applied());
  e.Trigger();
  EXPECT_TRUE(e.HoldPending());  // a stale "applied" must not satisfy a new trigger
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
