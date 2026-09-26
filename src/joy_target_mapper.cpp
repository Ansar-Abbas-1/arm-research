#include "arm_teleop/joy_target_mapper.h"

#include <cstdlib>

namespace arm_teleop {

namespace {

// Bounds-checked accessors: a Joy message from a controller with fewer
// axes/buttons than the configured index shouldn't crash the mapper, just
// treat the missing input as inactive.
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

JoyTargetUpdate ComputeJoyTargetUpdate(const sensor_msgs::Joy& joy, const JoyMapperParams& params,
                                       double dt) {
  JoyTargetUpdate update;

  if (!Button(joy, params.deadman_button)) {
    return update;  // deadman not held: no motion, no gripper change
  }

  double ax = ApplyDeadzone(Axis(joy, params.axis_x), params.axis_deadzone);
  double ay = ApplyDeadzone(Axis(joy, params.axis_y), params.axis_deadzone);
  double az = ApplyDeadzone(Axis(joy, params.axis_z), params.axis_deadzone);

  if (ax != 0.0 || ay != 0.0 || az != 0.0) {
    update.moving = true;
    update.dx = ax * params.linear_gain * dt;
    update.dy = ay * params.linear_gain * dt;
    update.dz = az * params.linear_gain * dt;
  }

  bool open_held = Button(joy, params.gripper_open_button);
  bool close_held = Button(joy, params.gripper_close_button);
  if (open_held != close_held) {  // exactly one: both-held is treated as no change
    update.gripper_changed = true;
    update.gripper_delta = (open_held ? 1.0 : -1.0) * params.gripper_rate * dt;
  }

  return update;
}

}  // namespace arm_teleop
