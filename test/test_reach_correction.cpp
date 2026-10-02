#include <gtest/gtest.h>

#include "arm_trajectory/reach_correction.h"

using arm_trajectory::NextCorrection;
using arm_trajectory::ReachCorrectionOptions;
using Vec = Eigen::Vector3d;

namespace {
// Explicit values (not the defaults) so these tests do not move when the
// tuned defaults do; the defaults are pinned separately below.
ReachCorrectionOptions On() {
  ReachCorrectionOptions o;
  o.max_steps = 3;
  o.trigger = 0.002;
  o.max_total = 0.03;
  return o;
}
}  // namespace

TEST(ReachCorrection, TunedDefaultsFromHardwareAB) {
  ReachCorrectionOptions o;
  EXPECT_EQ(o.max_steps, 0);          // feature stays off unless enabled
  EXPECT_DOUBLE_EQ(o.trigger, 0.006);
  EXPECT_DOUBLE_EQ(o.max_total, 0.06);
  // A 4 mm error is inside the noise floor: no correction at the defaults.
  o.max_steps = 2;
  EXPECT_FALSE(NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.004), Vec::Zero(), o, 0).apply);
  EXPECT_TRUE(NextCorrection(Vec(0, 0, 0), Vec(0, 0, -0.012), Vec::Zero(), o, 0).apply);
}

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

// ---- pitch compensation ----

using arm_trajectory::AimPose;
using arm_trajectory::NextPitchCorrection;
using arm_trajectory::PitchError;

namespace {
Eigen::Matrix3d Ry(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d SomeTarget() {
  return (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}
}  // namespace

TEST(PitchCorrection, PitchErrorIsSignedRotationStillNeededAboutPitchAxis) {
  Eigen::Matrix3d Rt = SomeTarget();
  // Arm fell 0.07 rad short about the pitch axis: it still needs +0.07.
  EXPECT_NEAR(PitchError(Rt, Rt * Ry(-0.07)), 0.07, 1e-9);
  EXPECT_NEAR(PitchError(Rt, Rt * Ry(+0.05)), -0.05, 1e-9);
  EXPECT_NEAR(PitchError(Rt, Rt), 0.0, 1e-12);
}

TEST(PitchCorrection, AimingByTheErrorCancelsAMotorUndershoot) {
  // Closed loop on the sign convention: if the motors always land 0.07 rad
  // short about the pitch axis, aiming at target + measured error lands
  // exactly on target.
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = SomeTarget();
  double shortfall = 0.07;
  Eigen::Matrix3d first = target.rotation() * Ry(-shortfall);
  double e = PitchError(target.rotation(), first);
  Eigen::Isometry3d aim = AimPose(target, Vec::Zero(), e);
  Eigen::Matrix3d second = aim.rotation() * Ry(-shortfall);
  EXPECT_NEAR(PitchError(target.rotation(), second), 0.0, 1e-9);
}

TEST(PitchCorrection, AimPoseRotatesOnlyAboutTheTargetsLocalY) {
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = SomeTarget();
  target.translation() = Vec(0.1, 0.2, 0.3);
  Eigen::Isometry3d aim = AimPose(target, Vec(0.01, 0, -0.02), 0.1);
  EXPECT_LT((aim.rotation() * Vec::UnitY() - target.rotation() * Vec::UnitY()).norm(), 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(aim.rotation() * target.rotation().transpose()).angle(), 0.1, 1e-9);
  EXPECT_NEAR(aim.translation().x(), 0.11, 1e-12);
  EXPECT_NEAR(aim.translation().z(), 0.28, 1e-12);
}

TEST(PitchCorrection, ZeroCompensationAimsAtTheTarget) {
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.linear() = SomeTarget();
  EXPECT_LT((AimPose(target, Vec::Zero(), 0.0).matrix() - target.matrix()).norm(), 1e-12);
}

TEST(PitchCorrection, OffByDefaultAndWhenSwitchedOff) {
  EXPECT_FALSE(NextPitchCorrection(0.08, 0.0, ReachCorrectionOptions{}, 0).apply);
  ReachCorrectionOptions o = On();
  o.pitch = false;
  EXPECT_FALSE(NextPitchCorrection(0.08, 0.0, o, 0).apply);
}

TEST(PitchCorrection, WithinTriggerNeedsNoCorrection) {
  EXPECT_FALSE(NextPitchCorrection(0.005, 0.0, On(), 0).apply);
  EXPECT_FALSE(NextPitchCorrection(-0.005, 0.0, On(), 0).apply);
}

TEST(PitchCorrection, StepsAccumulateWithSign) {
  auto d1 = NextPitchCorrection(0.07, 0.0, On(), 0);
  ASSERT_TRUE(d1.apply);
  EXPECT_NEAR(d1.compensation, 0.07, 1e-12);
  auto d2 = NextPitchCorrection(-0.02, d1.compensation, On(), 1);  // overshot by 0.02
  ASSERT_TRUE(d2.apply);
  EXPECT_NEAR(d2.compensation, 0.05, 1e-12);
}

TEST(PitchCorrection, CapAndSaturationAndBudget) {
  ReachCorrectionOptions o = On();
  o.pitch_max_total = 0.2;
  auto capped = NextPitchCorrection(-0.5, 0.0, o, 0);
  ASSERT_TRUE(capped.apply);
  EXPECT_NEAR(capped.compensation, -0.2, 1e-12);
  auto saturated = NextPitchCorrection(-0.5, -0.2, o, 1);
  EXPECT_FALSE(saturated.apply);
  EXPECT_NE(saturated.reason.find("saturated"), std::string::npos);
  EXPECT_FALSE(NextPitchCorrection(0.07, 0.0, On(), 3).apply);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
