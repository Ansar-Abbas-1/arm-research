#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "arm_interface/drive_state.h"
#include "arm_interface/goal_slew.h"

using namespace arm_interface;

namespace {
const std::vector<DriveState> kStates = {DriveState::kTorqueOff, DriveState::kHolding, DriveState::kTracking,
                                          DriveState::kEStopped};
const std::vector<DriveEvent> kEvents = {DriveEvent::kCommand,          DriveEvent::kCommandTimeout,
                                          DriveEvent::kTorqueOnRequest,  DriveEvent::kTorqueOffRequest,
                                          DriveEvent::kEStopTrigger,     DriveEvent::kEStopReset,
                                          DriveEvent::kFault};
}  // namespace

// The operator's core requirement: the end of a command stream (trajectory
// finished, or the sender died) must never touch torque.
TEST(DriveState, CommandTimeoutNeverTouchesTorqueInAnyState) {
  for (DriveState s : kStates) {
    DriveTransition t = NextDriveState(s, DriveEvent::kCommandTimeout);
    EXPECT_EQ(t.action, DriveAction::kNone) << ToString(s);
    EXPECT_FALSE(t.refused);
  }
  EXPECT_EQ(NextDriveState(DriveState::kTracking, DriveEvent::kCommandTimeout).next, DriveState::kHolding);
}

TEST(DriveState, NormalOperationNeverToggleTorque) {
  // startup hold -> command -> tracking -> stream ends -> hold -> command ...
  DriveState s = DriveState::kHolding;
  for (int cycle = 0; cycle < 3; ++cycle) {
    DriveTransition t = NextDriveState(s, DriveEvent::kCommand);
    EXPECT_EQ(t.next, DriveState::kTracking);
    EXPECT_EQ(t.action, DriveAction::kNone);  // no seed/enable: torque was never dropped
    s = t.next;
    t = NextDriveState(s, DriveEvent::kCommandTimeout);
    EXPECT_EQ(t.next, DriveState::kHolding);
    EXPECT_EQ(t.action, DriveAction::kNone);
    s = t.next;
  }
}

TEST(DriveState, OnlyTorqueOffNeedsSeedAndEnableToResume) {
  DriveTransition t = NextDriveState(DriveState::kTorqueOff, DriveEvent::kCommand);
  EXPECT_EQ(t.next, DriveState::kTracking);
  EXPECT_EQ(t.action, DriveAction::kSeedAndEnable);
  t = NextDriveState(DriveState::kTorqueOff, DriveEvent::kTorqueOnRequest);
  EXPECT_EQ(t.next, DriveState::kHolding);
  EXPECT_EQ(t.action, DriveAction::kSeedAndEnable);
}

TEST(DriveState, ExplicitTorqueOffDisablesFromHoldingAndTracking) {
  for (DriveState s : {DriveState::kHolding, DriveState::kTracking}) {
    DriveTransition t = NextDriveState(s, DriveEvent::kTorqueOffRequest);
    EXPECT_EQ(t.next, DriveState::kTorqueOff);
    EXPECT_EQ(t.action, DriveAction::kDisable);
  }
  EXPECT_EQ(NextDriveState(DriveState::kTorqueOff, DriveEvent::kTorqueOffRequest).action, DriveAction::kNone);
}

TEST(DriveState, EStopLatchesFromAnyStateAndHolds) {
  for (DriveState s : {DriveState::kTorqueOff, DriveState::kHolding, DriveState::kTracking}) {
    DriveTransition t = NextDriveState(s, DriveEvent::kEStopTrigger);
    EXPECT_EQ(t.next, DriveState::kEStopped) << ToString(s);
    EXPECT_EQ(t.action, DriveAction::kEStopHold) << ToString(s);
  }
  EXPECT_EQ(NextDriveState(DriveState::kEStopped, DriveEvent::kEStopTrigger).action, DriveAction::kNone);
}

TEST(DriveState, EStopRefusesCommandsAndTorqueRequests) {
  for (DriveEvent e : {DriveEvent::kCommand, DriveEvent::kTorqueOnRequest, DriveEvent::kTorqueOffRequest}) {
    DriveTransition t = NextDriveState(DriveState::kEStopped, e);
    EXPECT_TRUE(t.refused);
    EXPECT_EQ(t.next, DriveState::kEStopped);
    EXPECT_EQ(t.action, DriveAction::kNone);
  }
}

TEST(DriveState, EStopResetGoesToHoldingWithoutTouchingTorque) {
  DriveTransition t = NextDriveState(DriveState::kEStopped, DriveEvent::kEStopReset);
  EXPECT_EQ(t.next, DriveState::kHolding);
  EXPECT_EQ(t.action, DriveAction::kNone);
  // a reset when not latched changes nothing
  for (DriveState s : {DriveState::kTorqueOff, DriveState::kHolding, DriveState::kTracking}) {
    EXPECT_EQ(NextDriveState(s, DriveEvent::kEStopReset).next, s);
  }
}

TEST(DriveState, FaultDisablesFromEverywhereIncludingEStop) {
  for (DriveState s : {DriveState::kHolding, DriveState::kTracking, DriveState::kEStopped}) {
    DriveTransition t = NextDriveState(s, DriveEvent::kFault);
    EXPECT_EQ(t.next, DriveState::kTorqueOff);
    EXPECT_EQ(t.action, DriveAction::kDisable);
  }
  EXPECT_EQ(NextDriveState(DriveState::kTorqueOff, DriveEvent::kFault).action, DriveAction::kNone);
}

TEST(DriveState, EStoppedOnlyLeavesViaResetOrFault) {
  for (DriveEvent e : kEvents) {
    DriveTransition t = NextDriveState(DriveState::kEStopped, e);
    if (e == DriveEvent::kEStopReset || e == DriveEvent::kFault) {
      EXPECT_NE(t.next, DriveState::kEStopped);
    } else {
      EXPECT_EQ(t.next, DriveState::kEStopped);
    }
  }
}

TEST(DriveState, TableIsTotalAndOnlyDisableOrEnableActionsChangeTorque) {
  // Every (state, event) pair is defined, and a Disable only ever comes from
  // an explicit request or a fault.
  for (DriveState s : kStates) {
    for (DriveEvent e : kEvents) {
      DriveTransition t = NextDriveState(s, e);
      if (t.action == DriveAction::kDisable) {
        EXPECT_TRUE(e == DriveEvent::kTorqueOffRequest || e == DriveEvent::kFault) << ToString(s);
      }
    }
  }
}

TEST(GoalSlew, ReachesTargetWithinOneStepWhenClose) {
  EXPECT_DOUBLE_EQ(SlewToward(0.0, 0.01, 0.05), 0.01);
  EXPECT_DOUBLE_EQ(SlewToward(0.0, -0.05, 0.05), -0.05);
}

TEST(GoalSlew, LimitsStepBothDirectionsAndNeverOvershoots) {
  EXPECT_DOUBLE_EQ(SlewToward(0.0, 1.0, 0.05), 0.05);
  EXPECT_DOUBLE_EQ(SlewToward(0.0, -1.0, 0.05), -0.05);
  double x = 0.0;
  int ticks = 0;
  while (x != 1.0 && ticks < 1000) {
    double next = SlewToward(x, 1.0, 0.03);
    EXPECT_LE(next, 1.0);
    EXPECT_LE(next - x, 0.03 + 1e-12);
    x = next;
    ++ticks;
  }
  EXPECT_EQ(ticks, 34);  // ceil(1.0 / 0.03)
}

TEST(GoalSlew, ZeroNegativeOrNonFiniteInputsHoldTheCurrentGoal) {
  EXPECT_DOUBLE_EQ(SlewToward(0.3, 1.0, 0.0), 0.3);
  EXPECT_DOUBLE_EQ(SlewToward(0.3, 1.0, -0.1), 0.3);
  EXPECT_DOUBLE_EQ(SlewToward(0.3, std::numeric_limits<double>::quiet_NaN(), 0.1), 0.3);
  EXPECT_DOUBLE_EQ(SlewToward(0.3, 1.0, std::numeric_limits<double>::quiet_NaN()), 0.3);
  EXPECT_DOUBLE_EQ(SlewToward(0.3, std::numeric_limits<double>::infinity(), 0.1), 0.3);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
