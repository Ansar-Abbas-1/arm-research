#pragma once

#include <algorithm>
#include <cmath>

namespace arm_interface {

// Rate limit on the goal written to a position-mode motor.
//
// A Dynamixel with no Profile Velocity configured moves to a new goal at
// its own full speed, so ANY step in the written goal looks like a jump:
// resuming after a hold (the arm has sagged a little below its goal),
// enabling torque from off, the first sample of a trajectory that does not
// start exactly at the held goal. Instead of chasing each cause, the goal
// the motor sees is always the commanded one moved toward by at most
// `max_step` per tick from the goal LAST WRITTEN, so it is continuous by
// construction, whatever the cause (docs/sim_to_real.md section 16).
//
// max_step <= 0 or a non-finite input holds the current goal (fail-safe:
// never an unbounded step).
inline double SlewToward(double current, double target, double max_step) {
  if (!std::isfinite(current) || !std::isfinite(target) || !(max_step > 0.0)) {
    return current;
  }
  double delta = target - current;
  if (std::fabs(delta) <= max_step) {
    return target;
  }
  return current + std::copysign(max_step, delta);
}

}  // namespace arm_interface
