#include <gtest/gtest.h>

#include "arm_trajectory/reach_correction.h"

using arm_trajectory::NextCorrection;
using arm_trajectory::ReachCorrectionOptions;
using Vec = Eigen::Vector3d;

namespace {
ReachCorrectionOptions On() {
  ReachCorrectionOptions o;
  o.max_steps = 3;
  return o;
}
}  // namespace

TEST(ReachCorrection, OffByDefault) {
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0.02, 0, 0), Vec::Zero(), ReachCorrectionOptions{}, 0);
  EXPECT_FALSE(d.apply);
}

TEST(ReachCorrection, WithinTriggerNeedsNoCorrection) {
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0.001, 0, 0), Vec::Zero(), On(), 0);
  EXPECT_FALSE(d.apply);
}

TEST(ReachCorrection, FirstStepAddsTheMeasuredError) {
  // Arm ended 12 mm too far in +x and 16 mm too low: re-aim the opposite way.
  auto d = NextCorrection(Vec(0, 0, 0.5), Vec(0.012, 0, 0.484), Vec::Zero(), On(), 0);
  ASSERT_TRUE(d.apply);
  EXPECT_NEAR(d.compensation.x(), -0.012, 1e-12);
  EXPECT_NEAR(d.compensation.z(), 0.016, 1e-12);
}

TEST(ReachCorrection, StepsAccumulateRatherThanRestart) {
  // Integral action: the second step adds the remaining 4 mm to the 16 mm
  // already applied (20 mm total), it does not re-aim 4 mm from scratch.
  auto d = NextCorrection(Vec(0, 0, 0.5), Vec(0, 0, 0.496), Vec(0, 0, 0.016), On(), 1);
  ASSERT_TRUE(d.apply);
  EXPECT_NEAR(d.compensation.z(), 0.020, 1e-12);
}

TEST(ReachCorrection, GainScalesTheStep) {
  ReachCorrectionOptions o = On();
  o.gain = 0.5;
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.02), Vec::Zero(), o, 0);
  ASSERT_TRUE(d.apply);
  EXPECT_NEAR(d.compensation.z(), 0.010, 1e-12);
}

TEST(ReachCorrection, TotalCompensationIsCapped) {
  ReachCorrectionOptions o = On();
  o.max_total = 0.03;
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.1), Vec::Zero(), o, 0);
  ASSERT_TRUE(d.apply);
  EXPECT_NEAR(d.compensation.norm(), 0.03, 1e-12);
  EXPECT_NEAR(d.compensation.z(), 0.03, 1e-12);
}

TEST(ReachCorrection, SaturatedCompensationStopsInsteadOfSpinning) {
  ReachCorrectionOptions o = On();
  o.max_total = 0.03;
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.05), Vec(0, 0, 0.03), o, 1);
  EXPECT_FALSE(d.apply);
  EXPECT_NE(d.reason.find("saturated"), std::string::npos);
}

TEST(ReachCorrection, StepBudgetIsRespected) {
  auto d = NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.02), Vec::Zero(), On(), 3);
  EXPECT_FALSE(d.apply);
  EXPECT_NE(d.reason.find("budget"), std::string::npos);
}

TEST(ReachCorrection, NonPositiveGainDisables) {
  ReachCorrectionOptions o = On();
  o.gain = 0.0;
  EXPECT_FALSE(NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.02), Vec::Zero(), o, 0).apply);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
