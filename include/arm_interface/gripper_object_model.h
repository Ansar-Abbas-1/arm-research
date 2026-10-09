#pragma once
#include <algorithm>
#include <cmath>

namespace arm_interface {

/**
 * Simulation-only contact model of the gripper, for testing grasp logic without hardware.
 *
 * `contact_pos` is the joint_6 position at which closing is blocked (NaN = closes freely). The gripper joint is
 * "closed at the lower value" (joint_6: 0 = closed). A command below `contact_pos` leaves the measured position at
 * `contact_pos`, and the motor load then grows with how far the command is past it, as for a position-controlled servo
 * pushing against a block (a linear model, NOT identified from hardware).
 */

/// Measured gripper position: the command, but never closing past the contact.
inline double GripperMeasuredWithContact(double commanded, double contact_pos) {
  if (!std::isfinite(contact_pos)) return commanded;
  return std::max(commanded, contact_pos);
}

/// Motor load in % of max torque: baseline plus load_per_mm per millimetre of squeeze, saturated at 100 %.
inline double GripperLoadWithContact(double commanded, double contact_pos, double load_per_mm, double baseline) {
  if (!std::isfinite(contact_pos)) return baseline;
  const double squeeze_mm = std::max(0.0, contact_pos - commanded) * 1000.0;
  return std::min(100.0, baseline + load_per_mm * squeeze_mm);
}

}  // namespace arm_interface
