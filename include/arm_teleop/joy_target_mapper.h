#pragma once

#include <sensor_msgs/Joy.h>

namespace arm_teleop {

// Axis/button indices and gains are ROS-param-configurable (see
// teleop_node.cpp) -- these are defaults for a common Xbox-style layout
// (axes 0/1 = left stick, axis 4 = right stick Y; buttons 0/1 = A/B,
// button 4 = left bumper), not a hardcoded assumption about the user's
// actual controller. Adjust via ~axis_*/~button_* params to match.
struct JoyMapperParams {
  int deadman_button = 4;         // held to enable any motion (teleop_twist_joy convention)
  int axis_x = 1;                 // left stick vertical  -> end-effector +X
  int axis_y = 0;                 // left stick horizontal -> end-effector +Y
  int axis_z = 4;                 // right stick vertical -> end-effector +Z
  double axis_deadzone = 0.05;    // |axis| below this treated as 0 -- heuristic, covers stick rest noise
  double linear_gain = 0.1;       // m/s at full stick deflection -- heuristic default, tune to taste

  int gripper_open_button = 1;    // B
  int gripper_close_button = 0;   // A
  double gripper_rate = 0.5;      // rad/s while held -- heuristic; joint_6 is a fixed URDF joint
                                   // (see arm_interface/urdf/arm.urdf), so there's no real URDF
                                   // <limit> to derive this or gripper_min/max from
  double gripper_min = -1.0;
  double gripper_max = 1.0;
};

struct JoyTargetUpdate {
  bool moving = false;             // deadman held AND at least one axis past its deadzone
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;

  bool gripper_changed = false;    // deadman held AND exactly one gripper button held
                                    // (both held simultaneously: treated as no change, not a race)
  double gripper_delta = 0.0;      // unclamped -- caller accumulates and clamps to
                                    // [gripper_min, gripper_max] against its own running target
};

// Pure function: Joy message + params + elapsed time -> position/gripper
// deltas for this tick. No ROS I/O, no internal state -- unit-testable
// without a running node. All motion is gated on `deadman_button`: with it
// unheld, both `moving` and `gripper_changed` are false regardless of any
// other input, per standard joystick-teleop safety convention.
JoyTargetUpdate ComputeJoyTargetUpdate(const sensor_msgs::Joy& joy, const JoyMapperParams& params,
                                       double dt);

}  // namespace arm_teleop
