// Unit tests for the pure joystick-to-command mapping -- no ROS node, no
// running master needed (sensor_msgs::Joy is a plain message struct).

#include <gtest/gtest.h>

#include "arm_teleop/joy_target_mapper.h"

using arm_teleop::ComputeJoyCommand;
using arm_teleop::JoyCommand;
using arm_teleop::JoyMapperParams;

namespace {

sensor_msgs::Joy MakeJoy(size_t num_axes, size_t num_buttons) {
  sensor_msgs::Joy joy;
  joy.axes.assign(num_axes, 0.0f);
  joy.buttons.assign(num_buttons, 0);
  return joy;
}

constexpr double kTol = 1e-9;

}  // namespace

TEST(JoyTargetMapper, DeadmanNotHeldSuppressesEverythingIncludingEdges) {
  JoyMapperParams params;
  sensor_msgs::Joy prev = MakeJoy(8, 8);
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.axes[params.axis_x] = 1.0f;
  joy.buttons[params.gripper_open_button] = 1;
  joy.buttons[params.mode_toggle_button] = 1;  // a rising edge, but deadman is unpressed
  // deadman_button left unpressed

  JoyCommand cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_FALSE(cmd.deadman_held);
  EXPECT_NEAR(cmd.vx, 0.0, kTol);
  EXPECT_FALSE(cmd.gripper_active);
  EXPECT_FALSE(cmd.mode_toggled);
  EXPECT_FALSE(cmd.commit_pressed);
}

TEST(JoyTargetMapper, FullStickDeflectionScalesWithGainOnly) {
  JoyMapperParams params;
  params.linear_gain = 0.2;
  params.pitch_gain = 0.4;
  sensor_msgs::Joy prev = MakeJoy(8, 8);
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.axes[params.axis_x] = 1.0f;
  joy.axes[params.axis_y] = -1.0f;
  joy.axes[params.axis_pitch] = 0.5f;

  JoyCommand cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_TRUE(cmd.deadman_held);
  EXPECT_NEAR(cmd.vx, 0.2 * 1.0, kTol);
  EXPECT_NEAR(cmd.vy, 0.2 * -1.0, kTol);
  EXPECT_NEAR(cmd.vz, 0.0, kTol);
  EXPECT_NEAR(cmd.pitch_rate, 0.4 * 0.5, kTol);
}

TEST(JoyTargetMapper, DeadzoneSuppressesSmallNoise) {
  JoyMapperParams params;
  params.axis_deadzone = 0.1;
  sensor_msgs::Joy prev = MakeJoy(8, 8);
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.axes[params.axis_x] = 0.05f;  // below deadzone

  JoyCommand cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_NEAR(cmd.vx, 0.0, kTol);
}

TEST(JoyTargetMapper, GripperOpenAndCloseAreOpposite) {
  JoyMapperParams params;
  params.gripper_rate = 0.4;
  sensor_msgs::Joy prev = MakeJoy(8, 8);
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.gripper_open_button] = 1;

  JoyCommand open_cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_TRUE(open_cmd.gripper_active);
  EXPECT_NEAR(open_cmd.gripper_rate, 0.4, kTol);

  joy.buttons[params.gripper_open_button] = 0;
  joy.buttons[params.gripper_close_button] = 1;
  JoyCommand close_cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_TRUE(close_cmd.gripper_active);
  EXPECT_NEAR(close_cmd.gripper_rate, -0.4, kTol);
}

TEST(JoyTargetMapper, BothGripperButtonsHeldIsTreatedAsNoChange) {
  JoyMapperParams params;
  sensor_msgs::Joy prev = MakeJoy(8, 8);
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.gripper_open_button] = 1;
  joy.buttons[params.gripper_close_button] = 1;

  JoyCommand cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_FALSE(cmd.gripper_active);
  EXPECT_NEAR(cmd.gripper_rate, 0.0, kTol);
}

TEST(JoyTargetMapper, ModeToggleFiresOnceOnRisingEdgeNotWhileHeld) {
  JoyMapperParams params;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.mode_toggle_button] = 1;

  sensor_msgs::Joy prev_unpressed = MakeJoy(8, 8);
  prev_unpressed.buttons[params.deadman_button] = 1;
  EXPECT_TRUE(ComputeJoyCommand(joy, prev_unpressed, params).mode_toggled);

  // Held for a second tick: previous tick's message already shows it
  // pressed, so this is no longer a rising edge.
  EXPECT_FALSE(ComputeJoyCommand(joy, joy, params).mode_toggled);
}

TEST(JoyTargetMapper, CommitFiresOnceOnRisingEdge) {
  JoyMapperParams params;
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.commit_button] = 1;

  sensor_msgs::Joy prev_unpressed = MakeJoy(8, 8);
  prev_unpressed.buttons[params.deadman_button] = 1;
  EXPECT_TRUE(ComputeJoyCommand(joy, prev_unpressed, params).commit_pressed);
  EXPECT_FALSE(ComputeJoyCommand(joy, joy, params).commit_pressed);
}

TEST(JoyTargetMapper, ShortJoyMessageTreatedAsInactiveNotOutOfBounds) {
  JoyMapperParams params;  // default indices reference axes/buttons beyond this short message
  sensor_msgs::Joy prev = MakeJoy(1, 1);
  sensor_msgs::Joy joy = MakeJoy(1, 1);

  JoyCommand cmd = ComputeJoyCommand(joy, prev, params);
  EXPECT_FALSE(cmd.deadman_held);
  EXPECT_FALSE(cmd.mode_toggled);
}

TEST(JoyTargetMapper, EmptyPreviousMessageTreatedAsNoEdgeYet) {
  JoyMapperParams params;
  sensor_msgs::Joy empty_prev;  // no axes/buttons at all -- e.g. before the first /joy message
  sensor_msgs::Joy joy = MakeJoy(8, 8);
  joy.buttons[params.deadman_button] = 1;
  joy.buttons[params.mode_toggle_button] = 1;

  JoyCommand cmd = ComputeJoyCommand(joy, empty_prev, params);
  EXPECT_TRUE(cmd.mode_toggled);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
