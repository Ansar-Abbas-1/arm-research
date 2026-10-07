#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "arm_servo/servo_law.h"

using namespace arm_servo;

namespace {
ServoParams Params() {
  ServoParams p;
  p.lambda = 1.0;
  p.v_max = 0.04;
  return p;
}
Eigen::Matrix3d Ry(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
}  // namespace

// ---- the law -------------------------------------------------------------------------------------------------

TEST(ServoLaw, ZeroErrorGivesZeroCommand) {
  const auto p = Params();
  const TaskCommand c = ComputeTaskCommand(p.p_star, 0.0, p);
  EXPECT_NEAR(c.v_ee.norm(), 0.0, 1e-12);
  EXPECT_NEAR(c.pitch_rate, 0.0, 1e-12);
  EXPECT_NEAR(c.error_norm, 0.0, 1e-12);
}

TEST(ServoLaw, MovesTowardTheMarkerProportionallyBelowTheSpeedCap) {
  const auto p = Params();
  const Eigen::Vector3d marker = p.p_star + Eigen::Vector3d(0.0, 0.0, 0.02);   // marker 2 cm too far ahead
  const TaskCommand c = ComputeTaskCommand(marker, 0.0, p);
  EXPECT_NEAR(c.v_ee.z(), p.lambda * 0.02, 1e-12);                             // move forward, toward it
  EXPECT_NEAR(c.v_ee.x(), 0.0, 1e-12);
}

TEST(ServoLaw, TheSpeedCapKeepsTheDirection) {
  const auto p = Params();
  const Eigen::Vector3d marker = p.p_star + Eigen::Vector3d(0.3, 0.0, 0.4);
  const TaskCommand c = ComputeTaskCommand(marker, 0.0, p);
  EXPECT_NEAR(c.v_ee.norm(), p.v_max, 1e-12);
  EXPECT_NEAR(c.v_ee.x() / c.v_ee.z(), 0.3 / 0.4, 1e-9);
}

TEST(ServoLaw, PitchRateIsProportionalAndCapped) {
  const auto p = Params();
  EXPECT_NEAR(ComputeTaskCommand(p.p_star, 0.05, p).pitch_rate, 0.05 * p.lambda_pitch, 1e-12);
  EXPECT_NEAR(ComputeTaskCommand(p.p_star, 1.0, p).pitch_rate, p.omega_max, 1e-12);
  EXPECT_NEAR(ComputeTaskCommand(p.p_star, -1.0, p).pitch_rate, -p.omega_max, 1e-12);
}

TEST(ServoLaw, MarkerInEndEffectorAppliesTheMount) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  T.translation() = Eigen::Vector3d(-0.025, 0.0, -0.06);
  const Eigen::Vector3d p_e = MarkerInEndEffector(T, Eigen::Vector3d(0.0, 0.0, 0.16));   // 16 cm ahead of the camera
  EXPECT_NEAR(p_e.x(), -0.025, 1e-12);
  EXPECT_NEAR(p_e.z(), 0.10, 1e-12);       // 16 cm from the camera = 10 cm from the end effector, 6 cm behind it
}

TEST(PitchError, ToolAxisStraightDownIsZeroAndTheSignReducesTheError) {
  const Eigen::Matrix3d R_down = Ry(M_PI);                       // E z column = (0, 0, -1): the tool axis points down
  EXPECT_NEAR(R_down.col(2).z(), -1.0, 1e-12);
  EXPECT_NEAR(PitchErrorDown(R_down), 0.0, 1e-12);
  for (double a : {0.3, -0.3}) {
    const Eigen::Matrix3d R = R_down * Ry(a);                    // tilted by a about the end effector's OWN y axis
    const double phi = PitchErrorDown(R);
    EXPECT_NEAR(phi, -a, 1e-12);                                 // the rotation about its own y that undoes it
    EXPECT_NEAR(PitchErrorDown(R * Ry(phi)), 0.0, 1e-12);        // applying the commanded correction removes the error
  }
}

// ---- the supervisor ------------------------------------------------------------------------------------------

namespace {
SupervisorInputs Good(double now) {
  SupervisorInputs in;
  in.now = now;
  in.estop_allows_motion = true;
  in.measurement_age = 0.1;
  in.camera_distance = 0.4;
  in.ee_distance = 0.35;
  in.error_norm = 0.2;
  in.pitch_error = 0.2;
  return in;
}
}  // namespace

TEST(Supervisor, RunsUntilConvergedForTheRequiredTimeThenIsDone) {
  ServoSupervisor s(Params());
  s.BeginServoing(0.0);
  EXPECT_TRUE(s.Update(Good(0.1)).motion_allowed);
  auto in = Good(1.0);
  in.error_norm = 0.003;
  in.pitch_error = 0.01;
  EXPECT_EQ(s.Update(in).state, ServoState::kServoing);          // inside the tolerance, but not for long enough yet
  in.now = 1.5;
  EXPECT_EQ(s.Update(in).state, ServoState::kServoing);
  in.now = 2.01;
  const auto out = s.Update(in);
  EXPECT_EQ(out.state, ServoState::kDone);
  EXPECT_FALSE(out.motion_allowed);                              // done: send nothing, the driver holds
}

TEST(Supervisor, LeavingTheToleranceRestartsTheConvergenceTimer) {
  ServoSupervisor s(Params());
  s.BeginServoing(0.0);
  auto in = Good(1.0);
  in.error_norm = 0.003; in.pitch_error = 0.01;
  s.Update(in);
  in.now = 1.8; in.error_norm = 0.02;                            // out again
  s.Update(in);
  in.now = 2.4; in.error_norm = 0.003;
  EXPECT_EQ(s.Update(in).state, ServoState::kServoing);
}

TEST(Supervisor, AnEStopOrUnknownEStopAbortsAtOnce) {
  ServoSupervisor s(Params());
  s.BeginServoing(0.0);
  auto in = Good(0.5);
  in.estop_allows_motion = false;
  const auto out = s.Update(in);
  EXPECT_EQ(out.state, ServoState::kAborted);
  EXPECT_EQ(out.reason, AbortReason::kEStop);
  EXPECT_FALSE(out.motion_allowed);
}

TEST(Supervisor, AStaleMeasurementHoldsStillAndALostOneAborts) {
  ServoSupervisor s(Params());
  s.BeginServoing(0.0);
  auto in = Good(0.5);
  in.measurement_age = 0.5;                                      // older than 0.3 s, younger than 1 s
  auto out = s.Update(in);
  EXPECT_EQ(out.state, ServoState::kServoing);
  EXPECT_TRUE(out.zero_velocity);
  in.now = 1.5;
  in.measurement_age = 1.2;
  out = s.Update(in);
  EXPECT_EQ(out.state, ServoState::kAborted);
  EXPECT_EQ(out.reason, AbortReason::kMarkerLost);
}

TEST(Supervisor, LosingTheMarkerNearTheGoalEndsDoneAndFarFromItAborts) {
  {
    ServoSupervisor s(Params());
    s.BeginServoing(0.0);
    auto in = Good(1.0);
    in.error_norm = 0.012; in.pitch_error = 0.02;                 // within 2 cm of the goal, fresh
    s.Update(in);
    in.now = 2.5; in.measurement_age = 1.2;                       // then the marker leaves the view
    const auto out = s.Update(in);
    EXPECT_EQ(out.state, ServoState::kDone);
    EXPECT_TRUE(out.left_view);
    EXPECT_FALSE(out.motion_allowed);
  }
  {
    ServoSupervisor s(Params());
    s.BeginServoing(0.0);
    auto in = Good(1.0);
    in.error_norm = 0.05;                                         // 5 cm away when last seen
    s.Update(in);
    in.now = 2.5; in.measurement_age = 1.2;
    const auto out = s.Update(in);
    EXPECT_EQ(out.state, ServoState::kAborted);
    EXPECT_EQ(out.reason, AbortReason::kMarkerLost);
    EXPECT_FALSE(out.left_view);
  }
  {
    ServoSupervisor s(Params());                                  // never seen a fresh measurement: no free pass
    s.BeginServoing(0.0);
    auto in = Good(2.5); in.measurement_age = 1.2; in.error_norm = 0.0;
    EXPECT_EQ(s.Update(in).state, ServoState::kAborted);
  }
}

TEST(Supervisor, TooCloseToTheMarkerAborts) {
  for (int which = 0; which < 2; ++which) {
    ServoSupervisor s(Params());
    s.BeginServoing(0.0);
    auto in = Good(0.5);
    (which == 0 ? in.camera_distance : in.ee_distance) = 0.07;   // below the 8 cm minimum
    const auto out = s.Update(in);
    EXPECT_EQ(out.state, ServoState::kAborted) << which;
    EXPECT_EQ(out.reason, which == 0 ? AbortReason::kTooCloseCamera : AbortReason::kTooCloseEndEffector);
  }
}

TEST(Supervisor, CollisionGroundAndRunTimeAbort) {
  {
    ServoSupervisor s(Params()); s.BeginServoing(0.0);
    auto in = Good(0.5); in.collision = true;
    EXPECT_EQ(s.Update(in).reason, AbortReason::kCollision);
  }
  {
    ServoSupervisor s(Params()); s.BeginServoing(0.0);
    auto in = Good(0.5); in.below_ground = true;
    EXPECT_EQ(s.Update(in).reason, AbortReason::kBelowGround);
  }
  {
    ServoSupervisor s(Params()); s.BeginServoing(0.0);
    EXPECT_EQ(s.Update(Good(61.0)).reason, AbortReason::kRunTimeout);
  }
}

TEST(Supervisor, TheFirstAbortReasonStaysAndResetClearsIt) {
  ServoSupervisor s(Params());
  s.BeginServoing(0.0);
  s.Abort(AbortReason::kUserStop);
  s.Abort(AbortReason::kCollision);
  EXPECT_EQ(s.reason(), AbortReason::kUserStop);
  s.Reset();
  EXPECT_EQ(s.state(), ServoState::kIdle);
}

TEST(Supervisor, NothingMovesInTheStatesBeforeServoing) {
  ServoSupervisor s(Params());
  EXPECT_FALSE(s.Update(Good(0.0)).motion_allowed);              // idle
  s.BeginSelecting(0.0);
  EXPECT_FALSE(s.Update(Good(0.1)).motion_allowed);              // selecting a target
  auto in = Good(0.2); in.estop_allows_motion = false;
  EXPECT_EQ(s.Update(in).state, ServoState::kAborted);           // an e-stop also ends the selection
}

// ---- target selection ----------------------------------------------------------------------------------------

TEST(TargetSelector, PicksTheClosestMarkerSeenConsistently) {
  TargetSelector sel(5, 0.6);
  for (int i = 0; i < 5; ++i) sel.AddFrame({{94, 0.47 + 0.001 * i}, {118, 0.52}, {137, 0.60}});
  ASSERT_TRUE(sel.Ready());
  EXPECT_EQ(sel.Choice(), 94);
}

TEST(TargetSelector, AMarkerSeenInTooFewFramesIsIgnoredEvenIfClosest) {
  TargetSelector sel(5, 0.6);
  sel.AddFrame({{102, 0.30}, {118, 0.52}});                      // 102 flickers in once, much closer
  for (int i = 0; i < 4; ++i) sel.AddFrame({{118, 0.52}});
  EXPECT_EQ(sel.Choice(), 118);
}

TEST(TargetSelector, NothingSeenMeansNoTarget) {
  TargetSelector sel(3, 0.6);
  for (int i = 0; i < 3; ++i) sel.AddFrame({});
  ASSERT_TRUE(sel.Ready());
  EXPECT_EQ(sel.Choice(), -1);
}

// ---- the discrete stability bound of the derivation ------------------------------------------------------------

namespace {
// e[k+1] = e[k] - g e[k-d], g = lambda T; returns |e| after n steps from e = 1
double Simulate(double g, int d, int n) {
  std::vector<double> e(n + d + 2, 1.0);
  for (int k = d; k < n + d; ++k) e[k + 1] = e[k] - g * e[k - d];
  return std::fabs(e[n + d]);
}
}  // namespace

TEST(DelayBound, ScalarLoopIsStableBelowAndUnstableAboveTwoSinPiOver2Times2dPlus1) {
  for (int d = 0; d <= 4; ++d) {
    const double bound = 2.0 * std::sin(M_PI / (2.0 * (2.0 * d + 1.0)));
    EXPECT_LT(Simulate(0.9 * bound, d, 600), 1e-2) << "d = " << d << " bound " << bound;    // decays
    EXPECT_GT(Simulate(1.1 * bound, d, 600), 1e2) << "d = " << d << " bound " << bound;     // grows
  }
  EXPECT_NEAR(2.0 * std::sin(M_PI / 10.0), 0.618, 1e-3);          // d = 2, the case used in the design
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
