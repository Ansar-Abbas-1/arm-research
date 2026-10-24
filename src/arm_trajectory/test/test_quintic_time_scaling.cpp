// Cross-checks the closed-form quintic time-scaling formulas against a
// numerical (central finite-difference) derivative of QuinticPosition --
// an independent computation, not a re-check of the same formula.

#include <gtest/gtest.h>

#include <cmath>\n#include <algorithm>\n#include <stdexcept>

#include "arm_trajectory/quintic_time_scaling.h"

using arm_trajectory::MinDurationForAccelerationLimit;
using arm_trajectory::MinDurationForVelocityLimit;
using arm_trajectory::kQuinticPeakAccelerationFraction;
using arm_trajectory::QuinticAcceleration;
using arm_trajectory::QuinticPosition;
using arm_trajectory::QuinticVelocity;
using arm_trajectory::kQuinticPeakVelocityFraction;

namespace {
constexpr double kFdStep = 1e-6;
constexpr double kTol = 1e-6;
}  // namespace

TEST(QuinticTimeScaling, BoundaryConditions) {
  EXPECT_NEAR(QuinticPosition(0.0), 0.0, kTol);
  EXPECT_NEAR(QuinticPosition(1.0), 1.0, kTol);
  EXPECT_NEAR(QuinticVelocity(0.0), 0.0, kTol);
  EXPECT_NEAR(QuinticVelocity(1.0), 0.0, kTol);
  EXPECT_NEAR(QuinticAcceleration(0.0), 0.0, kTol);
  EXPECT_NEAR(QuinticAcceleration(1.0), 0.0, kTol);
}

TEST(QuinticTimeScaling, MonotonicPosition) {
  double prev = QuinticPosition(0.0);
  for (double tau = 0.05; tau <= 1.0; tau += 0.05) {
    double cur = QuinticPosition(tau);
    EXPECT_GE(cur, prev);
    prev = cur;
  }
}

TEST(QuinticTimeScaling, VelocityMatchesFiniteDifferenceOfPosition) {
  for (double tau = 0.1; tau <= 0.9; tau += 0.1) {
    double numerical = (QuinticPosition(tau + kFdStep) - QuinticPosition(tau - kFdStep)) / (2 * kFdStep);
    EXPECT_NEAR(QuinticVelocity(tau), numerical, 1e-4) << "tau=" << tau;
  }
}

TEST(QuinticTimeScaling, AccelerationMatchesFiniteDifferenceOfVelocity) {
  for (double tau = 0.1; tau <= 0.9; tau += 0.1) {
    double numerical = (QuinticVelocity(tau + kFdStep) - QuinticVelocity(tau - kFdStep)) / (2 * kFdStep);
    EXPECT_NEAR(QuinticAcceleration(tau), numerical, 1e-4) << "tau=" << tau;
  }
}

TEST(QuinticTimeScaling, PeakVelocityIsAtHalfAndEquals15Over8) {
  // Independent check: scan velocity at many tau and confirm the maximum
  // occurs at tau=0.5 and equals kQuinticPeakVelocityFraction, rather than
  // trusting the constant's derivation alone.
  double max_v = -1.0;
  double argmax = -1.0;
  for (double tau = 0.0; tau <= 1.0; tau += 0.001) {
    double v = QuinticVelocity(tau);
    if (v > max_v) {
      max_v = v;
      argmax = tau;
    }
  }
  EXPECT_NEAR(argmax, 0.5, 0.002);
  EXPECT_NEAR(max_v, kQuinticPeakVelocityFraction, 1e-4);
  EXPECT_NEAR(kQuinticPeakVelocityFraction, 15.0 / 8.0, 1e-12);
}

TEST(MinDurationForVelocityLimitTest, MatchesDerivedFormula) {
  double delta = 2.0;
  double v_max = 1.5;
  double expected = kQuinticPeakVelocityFraction * std::fabs(delta) / v_max;
  EXPECT_NEAR(MinDurationForVelocityLimit(delta, v_max), expected, 1e-12);
}

TEST(MinDurationForVelocityLimitTest, NegativeDeltaUsesAbsoluteValue) {
  EXPECT_NEAR(MinDurationForVelocityLimit(-2.0, 1.5), MinDurationForVelocityLimit(2.0, 1.5), 1e-12);
}

TEST(MinDurationForVelocityLimitTest, ZeroDeltaNeedsNoTime) {
  EXPECT_EQ(MinDurationForVelocityLimit(0.0, 1.0), 0.0);
}

TEST(MinDurationForVelocityLimitTest, NonPositiveVelocityLimitThrows) {
  EXPECT_THROW(MinDurationForVelocityLimit(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(MinDurationForVelocityLimit(1.0, -1.0), std::invalid_argument);
}

TEST(MinDurationForVelocityLimitTest, GeneratedMotionRespectsLimitExactly) {
  // If we actually run a quintic profile of magnitude `delta` over
  // T = MinDurationForVelocityLimit(delta, v_max), its peak velocity
  // should be exactly v_max (not just <= it) -- confirms the formula
  // isn't just conservative, it's tight.
  double delta = 3.0;
  double v_max = 2.0;
  double T = MinDurationForVelocityLimit(delta, v_max);
  double peak = -1.0;
  for (double tau = 0.0; tau <= 1.0; tau += 0.0005) {
    peak = std::max(peak, delta * QuinticVelocity(tau) / T);
  }
  EXPECT_NEAR(peak, v_max, 1e-3);
}

TEST(QuinticTimeScaling, PeakAccelerationFractionMatchesNumericMaximum) {
  double peak = 0.0;
  for (double tau = 0.0; tau <= 1.0; tau += 1e-5) {
    peak = std::max(peak, std::fabs(QuinticAcceleration(tau)));
  }
  EXPECT_NEAR(peak, kQuinticPeakAccelerationFraction, 1e-4);
  EXPECT_NEAR(kQuinticPeakAccelerationFraction, 10.0 / std::sqrt(3.0), 1e-12);
}

TEST(MinDurationForAccelerationLimitTest, GeneratedMotionPeaksExactlyAtLimit) {
  double delta = 0.3;
  double a_max = 2.0;
  double T = MinDurationForAccelerationLimit(delta, a_max);
  double peak = 0.0;
  for (double tau = 0.0; tau <= 1.0; tau += 1e-5) {
    peak = std::max(peak, std::fabs(delta * QuinticAcceleration(tau) / (T * T)));
  }
  EXPECT_NEAR(peak, a_max, 1e-3);
}

TEST(MinDurationForAccelerationLimitTest, ScalesWithSquareRootOfDelta) {
  EXPECT_NEAR(MinDurationForAccelerationLimit(4.0, 1.0) / MinDurationForAccelerationLimit(1.0, 1.0), 2.0, 1e-12);
  EXPECT_DOUBLE_EQ(MinDurationForAccelerationLimit(-0.5, 1.0), MinDurationForAccelerationLimit(0.5, 1.0));
  EXPECT_DOUBLE_EQ(MinDurationForAccelerationLimit(0.0, 1.0), 0.0);
  EXPECT_THROW(MinDurationForAccelerationLimit(1.0, 0.0), std::invalid_argument);
  EXPECT_THROW(MinDurationForAccelerationLimit(1.0, -1.0), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
