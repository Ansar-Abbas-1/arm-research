#include "arm_teleop/joy_target_mapper.h"

#include <cstdlib>

namespace arm_teleop {

namespace {

// Bounds-checked accessors: a Joy message from a controller with fewer
// axes/buttons than the configured index (or an empty "previous message"
// placeholder before the first one ever arrives) shouldn't crash the
// mapper, just treat the missing input as inactive.
double Axis(const sensor_msgs::Joy& joy, int index) {
  if (index < 0 || static_cast<size_t>(index) >= joy.axes.size()) {
    return 0.0;
  }
  return joy.axes[index];
}

bool Button(const sensor_msgs::Joy& joy, int index) {
  if (index < 0 || static_cast<size_t>(index) >= joy.buttons.size()) {
    return false;
  }
  return joy.buttons[index] != 0;
}

double ApplyDeadzone(double value, double deadzone) {
  return std::abs(value) < deadzone ? 0.0 : value;
}

}  // namespace

JoyCommand ComputeJoyCommand(const sensor_msgs::Joy& joy, const sensor_msgs::Joy& prev_joy,
                              const JoyMapperParams& params) {
  JoyCommand cmd;

  cmd.deadman_held = Button(joy, params.deadman_button);
  if (!cmd.deadman_held) {
    return cmd;  // deadman not held: no motion, no gripper change, no mode/commit edge
  }

  cmd.vx = ApplyDeadzone(Axis(joy, params.axis_x), params.axis_deadzone) * params.linear_gain;
  cmd.vy = ApplyDeadzone(Axis(joy, params.axis_y), params.axis_deadzone) * params.linear_gain;
  cmd.vz = ApplyDeadzone(Axis(joy, params.axis_z), params.axis_deadzone) * params.linear_gain;
  cmd.pitch_rate =
      ApplyDeadzone(Axis(joy, params.axis_pitch), params.axis_deadzone) * params.pitch_gain;

  bool open_held = Button(joy, params.gripper_open_button);
  bool close_held = Button(joy, params.gripper_close_button);
  if (open_held != close_held) {  // exactly one: both-held is treated as no change
    cmd.gripper_active = true;
    cmd.gripper_rate = (open_held ? 1.0 : -1.0) * params.gripper_rate;
  }

  bool mode_toggle_now = Button(joy, params.mode_toggle_button);
  bool mode_toggle_prev = Button(prev_joy, params.mode_toggle_button);
  cmd.mode_toggled = mode_toggle_now && !mode_toggle_prev;

  bool commit_now = Button(joy, params.commit_button);
  bool commit_prev = Button(prev_joy, params.commit_button);
  cmd.commit_pressed = commit_now && !commit_prev;

  return cmd;
}

}  // namespace arm_teleop
