#pragma once

namespace arm_trajectory {

// Quintic time-scaling: the standard smooth point-to-point profile with
// zero velocity AND acceleration at both endpoints (avoids the velocity
// discontinuities of a naive trapezoidal profile). See
// docs/trajectory_generation.md for the derivation.
//
// s(tau)  = 10*tau^3 - 15*tau^4 + 6*tau^5      s(0)=0, s(1)=1
// s'(tau) = 30*tau^2 - 60*tau^3 + 30*tau^4      s'(0)=s'(1)=0
//
// `tau` is normalized time in [0, 1]; callers pass t/T.

// Position fraction s(tau), tau expected in [0, 1] (not checked/clamped --
// callers control tau via t/T, which is always in range by construction).
double QuinticPosition(double tau);

// Velocity fraction s'(tau) (i.e. d/d_tau of QuinticPosition, NOT yet
// divided by the real duration T -- actual joint velocity is
// delta_q * QuinticVelocity(t/T) / T).
double QuinticVelocity(double tau);

// Acceleration fraction s''(tau) (NOT yet divided by T^2 -- actual joint
// acceleration is delta_q * QuinticAcceleration(t/T) / (T*T)).
double QuinticAcceleration(double tau);

// s'(tau) is maximized at tau=0.5, where it equals exactly 15/8 (see
// docs/trajectory_generation.md). Exposed as a named constant since
// MinDurationForVelocityLimit's derivation depends on this exact value,
// not an approximation.
constexpr double kQuinticPeakVelocityFraction = 15.0 / 8.0;

// Minimum duration T (seconds) such that a quintic-scaled motion from 0 to
// `delta` (radians) never exceeds `velocity_limit` (rad/s, must be > 0):
// T_min = kQuinticPeakVelocityFraction * |delta| / velocity_limit.
// Returns 0.0 if delta == 0 (no motion needed). Throws
// std::invalid_argument if velocity_limit <= 0.
double MinDurationForVelocityLimit(double delta, double velocity_limit);

// |s''(tau)| is maximized where s'''(tau) = 60 - 360*tau + 360*tau^2 = 0,
// i.e. tau = (1 -/+ 1/sqrt(3))/2, where it equals exactly 10/sqrt(3)
// (about 5.7735). Actual peak joint acceleration of a quintic-scaled
// motion is kQuinticPeakAccelerationFraction * |delta| / T^2.
constexpr double kQuinticPeakAccelerationFraction = 5.773502691896258;

// Minimum duration T (seconds) such that a quintic-scaled motion from 0 to
// `delta` (radians) never exceeds `accel_limit` (rad/s^2, must be > 0):
// T_min = sqrt(kQuinticPeakAccelerationFraction * |delta| / accel_limit).
// Returns 0.0 if delta == 0. Throws std::invalid_argument if accel_limit <= 0.
// Note it scales as sqrt(delta), unlike the velocity bound (linear in delta):
// small moves are acceleration-limited, large ones velocity-limited.
double MinDurationForAccelerationLimit(double delta, double accel_limit);

}  // namespace arm_trajectory
