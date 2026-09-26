#pragma once

#include <sensor_msgs/Joy.h>

namespace arm_teleop {

// Axis/button indices and gains are ROS-param-configurable (see
// teleop_node.cpp) -- these are defaults for a common Xbox-style layout
// (axes 0/1 = left stick, axis 3 = a second stick axis for pitch, axis 4 =
// right stick Y; buttons 0/1 = A/B, button 4 = left bumper, buttons 6/7 =
// back/start), not a hardcoded assumption about the user's actual
// controller. Adjust via ~axis_*/~button_* params to match.
struct JoyMapperParams {
  int deadman_button = 4;          // held to enable ANY motion or state change below
  int axis_x = 1;                  // left stick vertical   -> end-effector +X velocity
  int axis_y = 0;                  // left stick horizontal -> end-effector +Y velocity
  int axis_z = 4;                  // right stick vertical  -> end-effector +Z velocity
  int axis_pitch = 3;              // right stick horizontal -> wrist pitch rate
  double axis_deadzone = 0.05;     // |axis| below this treated as 0 -- heuristic, covers stick rest noise
  double linear_gain = 0.1;        // m/s at full stick deflection -- heuristic default, tune to taste
  double pitch_gain = 0.5;         // rad/s at full stick deflection -- heuristic default

  int gripper_open_button = 1;     // B
  int gripper_close_button = 0;    // A
  double gripper_rate = 0.5;       // rad/s while held -- heuristic; joint_6 is a fixed URDF joint
                                    // (see arm_interface/urdf/arm.urdf), so there's no real URDF
                                    // <limit> to derive this or gripper_min/max from
  double gripper_min = -1.0;
  double gripper_max = 1.0;

  int mode_toggle_button = 6;      // back/select: DIRECT_JOG <-> SET_REFERENCE, edge-triggered
  int commit_button = 7;           // start: generate+execute a trajectory to the reference pose,
                                    // edge-triggered, only meaningful in SET_REFERENCE mode
};

// Everything here is a RATE (velocity), not a pre-integrated delta --
// integration against dt is the caller's job (teleop_node), since the
// resolved-rate control law and the SET_REFERENCE pose accumulator both
// need the raw commanded rate, not a rate baked in at one specific dt.
struct JoyCommand {
  bool deadman_held = false;

  double vx = 0.0;          // m/s
  double vy = 0.0;          // m/s
  double vz = 0.0;          // m/s
  double pitch_rate = 0.0;  // rad/s, about the end-effector's own local Y axis

  bool gripper_active = false;  // exactly one gripper button held (both held: no-op, not a race)
  double gripper_rate = 0.0;    // rad/s, signed; 0 unless gripper_active

  bool mode_toggled = false;    // true for exactly one tick per press (rising edge), not per tick held
  bool commit_pressed = false;  // true for exactly one tick per press (rising edge)
};

// Pure function: current + previous Joy message (for edge detection on the
// mode-toggle/commit buttons) + params -> a JoyCommand. No ROS I/O, no
// internal state -- unit-testable without a running node. Every field is
// gated on `deadman_button`: with it unheld, everything (including
// mode_toggled/commit_pressed) is false/zero regardless of any other
// input or edge, per standard joystick-teleop safety convention.
JoyCommand ComputeJoyCommand(const sensor_msgs::Joy& joy, const sensor_msgs::Joy& prev_joy,
                              const JoyMapperParams& params);

}  // namespace arm_teleop
