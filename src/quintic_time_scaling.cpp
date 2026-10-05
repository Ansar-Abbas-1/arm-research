#include "arm_trajectory/quintic_time_scaling.h"

#include <cmath>
#include <stdexcept>

namespace arm_trajectory {

double QuinticPosition(double tau) {
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  double tau5 = tau4 * tau;
  return 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
}

double QuinticVelocity(double tau) {
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  return 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;
}

double QuinticAcceleration(double tau) {
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  return 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;
}

double MinDurationForVelocityLimit(double delta, double velocity_limit) {
  if (velocity_limit <= 0.0) {
    throw std::invalid_argument("MinDurationForVelocityLimit: velocity_limit must be > 0");
  }
  if (delta == 0.0) {
    return 0.0;
  }
  return kQuinticPeakVelocityFraction * std::fabs(delta) / velocity_limit;
}

double MinDurationForAccelerationLimit(double delta, double accel_limit) {
  if (accel_limit <= 0.0) {
    throw std::invalid_argument("MinDurationForAccelerationLimit: accel_limit must be > 0");
  }
  if (delta == 0.0) {
    return 0.0;
  }
  return std::sqrt(kQuinticPeakAccelerationFraction * std::fabs(delta) / accel_limit);
}

}  // namespace arm_trajectory
