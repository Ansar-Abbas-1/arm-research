#pragma once

#include <algorithm>

namespace arm_interface {

// The gripper (joint_6) is kinematically/visually prismatic -- the fingers
// genuinely translate, not rotate, and `urdf/arm.urdf` models it that way
// -- but it is ACTUATED by a rotational Dynamixel motor, same as every
// other joint; a mechanical linkage inside the gripper (lead-screw,
// rack-and-pinion, or similar) converts the motor's rotation into that
// linear travel. This type is the boundary where that conversion happens:
// every other joint's hardware-facing code treats "the motor's own
// rotation" and "the kinematic joint value" as the same number, but for
// the gripper they are two different domains related by this mechanism.
//
// UNCALIBRATED PLACEHOLDER: assumes a simple affine (linear-in-angle)
// relationship between motor angle and finger travel -- exact for an
// ideal lead-screw/rack-and-pinion, approximately true for many small
// gripper mechanisms, but this repo has no real measurement or spec of
// the actual mechanism (same caveat `urdf/arm.urdf`'s own joint_6 comment
// already flags for the linear range itself). Revisit once the real
// mechanism is characterized.
struct GripperConversion {
  double rotational_min;  // radians -- the motor's own real range, see
  double rotational_max;  // config/ArmInterface.yaml's bias/sign/encoder_min/max
                           // for the gripper's index (joint_6)
  double linear_min;      // meters -- matching urdf/arm.urdf's joint_6 <limit>
  double linear_max;

  // Clamped to [0,1] before mapping: a live motor reading can transiently
  // sit outside its own configured encoder bounds (startup, slop, a
  // momentary read glitch) -- clamping keeps the reported linear position
  // within the kinematic joint's own valid range instead of extrapolating
  // nonsense past it.
  double RotationalToLinear(double rotational) const {
    double t = (rotational - rotational_min) / (rotational_max - rotational_min);
    t = std::max(0.0, std::min(1.0, t));
    return linear_min + t * (linear_max - linear_min);
  }

  double LinearToRotational(double linear) const {
    double t = (linear - linear_min) / (linear_max - linear_min);
    t = std::max(0.0, std::min(1.0, t));
    return rotational_min + t * (rotational_max - rotational_min);
  }

  // Derivative of the above (constant, since the map is affine) -- for
  // converting a commanded LINEAR velocity (e.g. from a future gripper
  // velocity command) into the equivalent ROTATIONAL velocity the motor
  // must be driven at, and back.
  double LinearToRotationalVelocity(double linear_velocity) const {
    return linear_velocity * (rotational_max - rotational_min) / (linear_max - linear_min);
  }

  double RotationalToLinearVelocity(double rotational_velocity) const {
    return rotational_velocity * (linear_max - linear_min) / (rotational_max - rotational_min);
  }
};

}  // namespace arm_interface
