// Unit tests for the pure joystick-to-target-delta mapping -- no ROS node,
// no running master needed (sensor_msgs::Joy is a plain message struct).

#include <gtest/gtest.h>

#include "arm_teleop/joy_target_mapper.h"

using arm_teleop::ComputeJoyTargetUpdate;
using arm_teleop::JoyMapperParams;
using arm_teleop::JoyTargetUpdate;

namespace {

sensor_msgs::Joy MakeJoy(size_t num_axes, size_t num_buttons) {
  sensor_msgs::Joy joy;
  joy.axes.assign(num_axes, 0.0f);
  joy.buttons.assign(num_buttons, 0);
  return joy;
}

constexpr double kTol = 1e-9;

}  // namespace

TEST(JoyTargetMapper, DeadmanNotHeldSuppressesEverything) {
  JoyMapperParams params;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.axes[params.axis_x] = 1.0f;
  joy.buttons[params.gripper_open_button] = 1;
  // deadman_button left unpressed

  JoyTargetUpdate update = ComputeJoyTargetUpdate(joy, params, 0.1);
  EXPECT_FALSE(update.moving);
  EXPECT_FALSE(update.gripper_changed);
  EXPECT_NEAR(update.dx, 0.0, kTol);
  EXPECT_NEAR(update.gripper_delta, 0.0, kTol);
}

TEST(JoyTargetMapper, FullStickDeflectionScalesWithGainAndDt) {
  JoyMapperParams params;
  params.linear_gain = 0.2;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.axes[params.axis_x] = 1.0f;
  joy.axes[params.axis_y] = -1.0f;

  JoyTargetUpdate update = ComputeJoyTargetUpdate(joy, params, 0.5);
  EXPECT_TRUE(update.moving);
  EXPECT_NEAR(update.dx, 0.2 * 1.0 * 0.5, kTol);
  EXPECT_NEAR(update.dy, 0.2 * -1.0 * 0.5, kTol);
  EXPECT_NEAR(update.dz, 0.0, kTol);
}

TEST(JoyTargetMapper, DeadzoneSuppressesSmallNoise) {
  JoyMapperParams params;
  params.axis_deadzone = 0.1;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.axes[params.axis_x] = 0.05f;  // below deadzone

  JoyTargetUpdate update = ComputeJoyTargetUpdate(joy, params, 0.1);
  EXPECT_FALSE(update.moving);
  EXPECT_NEAR(update.dx, 0.0, kTol);
}

TEST(JoyTargetMapper, GripperOpenAndCloseAreOpposite) {
  JoyMapperParams params;
  params.gripper_rate = 0.4;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.gripper_open_button] = 1;

  JoyTargetUpdate open_update = ComputeJoyTargetUpdate(joy, params, 0.25);
  EXPECT_TRUE(open_update.gripper_changed);
  EXPECT_NEAR(open_update.gripper_delta, 0.4 * 0.25, kTol);

  joy.buttons[params.gripper_open_button] = 0;
  joy.buttons[params.gripper_close_button] = 1;
  JoyTargetUpdate close_update = ComputeJoyTargetUpdate(joy, params, 0.25);
  EXPECT_TRUE(close_update.gripper_changed);
  EXPECT_NEAR(close_update.gripper_delta, -0.4 * 0.25, kTol);
}

TEST(JoyTargetMapper, BothGripperButtonsHeldIsTreatedAsNoChange) {
  JoyMapperParams params;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.gripper_open_button] = 1;
  joy.buttons[params.gripper_close_button] = 1;

  JoyTargetUpdate update = ComputeJoyTargetUpdate(joy, params, 0.1);
  EXPECT_FALSE(update.gripper_changed);
  EXPECT_NEAR(update.gripper_delta, 0.0, kTol);
}

TEST(JoyTargetMapper, ShortJoyMessageTreatedAsInactiveNotOutOfBounds) {
  JoyMapperParams params;  // default indices reference axes/buttons beyond this short message
  sensor_msgs::Joy joy = MakeJoy(1, 1);

  JoyTargetUpdate update = ComputeJoyTargetUpdate(joy, params, 0.1);
  EXPECT_FALSE(update.moving);
  EXPECT_FALSE(update.gripper_changed);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
