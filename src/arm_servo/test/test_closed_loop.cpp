// Closed-loop simulation of the visual servo with the REAL arm kinematics: the marker is fixed in the world, the
// camera measurement comes at 10 Hz with a delay, the arm follows its setpoint with a first-order lag, and the
// controller (servo_law + the resolved-rate solver the reach pipeline uses, with the null-space bias that protects
// motor 4) never sees the true state. Checks the properties derived in servo_law.h.
#include <gtest/gtest.h>

#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_servo/servo_law.h"
#include "arm_servo/servo_sim.h"

using namespace arm_servo;

namespace {

urdf::Model LoadArm() {
  urdf::Model model;
  EXPECT_TRUE(model.initFile(ros::package::getPath("arm_interface") + "/urdf/arm.urdf"));
  return model;
}

SimScenario Nominal() {
  SimScenario sc;
  sc.T_true = Mount(M_PI / 2, -0.025, 0.0, -0.06);
  sc.T_assumed = sc.T_true;
  return sc;
}

class ClosedLoop : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArm();
    sim_ = std::make_unique<ServoSimulator>(model_);
  }
  urdf::Model model_;
  std::unique_ptr<ServoSimulator> sim_;
};

}  // namespace

TEST_F(ClosedLoop, ConvergesToTheStandOffWithinTheSpeedCapsAndWithoutHittingTheBase) {
  SimScenario sc = Nominal();
  const SimResult r = sim_->Run(sc);
  EXPECT_LT(r.final_error, 0.003) << "final |e| " << r.final_error;
  EXPECT_LT(r.final_pitch_error, 0.05);
  EXPECT_TRUE(r.done) << "the supervisor never declared convergence";
  EXPECT_FALSE(r.aborted) << r.abort_reason;
  EXPECT_LE(r.max_v, sc.params.v_max + 1e-9);
  EXPECT_LE(r.max_joint_speed, sc.max_joint_velocity + 1e-9);
  EXPECT_FALSE(r.collided);
}

TEST_F(ClosedLoop, AMountErrorMovesTheFixedPointByAtMostItsSizeAndNeverDestabilises) {
  SimScenario sc = Nominal();
  // the controller believes a mount that is 5 deg and 5 mm off the real one
  sc.T_assumed = Mount(M_PI / 2 + 5.0 * M_PI / 180.0, -0.025 + 0.005, 0.0, -0.06);
  const SimResult r = sim_->Run(sc);
  EXPECT_LT(r.final_error, 0.003);                               // it converges on what it believes ...
  // |t error| + 2 sin(theta/2) |p* - t_assumed|: the CAMERA-to-marker distance at the goal (see servo_law.h)
  const double cam_to_marker = (sc.params.p_star - sc.T_assumed.translation()).norm();
  const double bound = 0.005 + 2.0 * std::sin(0.5 * 5.0 * M_PI / 180.0) * cam_to_marker + 0.002;
  EXPECT_LT(r.final_true_error, bound) << "true error " << r.final_true_error << " bound " << bound;   // ... which is near
  EXPECT_GT(r.final_true_error, 0.001);                          // and it IS off by something (the test has teeth)
}

TEST_F(ClosedLoop, StableWithTheDesignGainAndADelayOfTwoVisionPeriods) {
  SimScenario sc = Nominal();
  sc.delay = 0.2;                                                // d = 2 samples at 10 Hz: bound lambda T < 0.618
  sc.params.lambda = 1.0;                                        // lambda T = 0.1
  EXPECT_LT(sim_->Run(sc).final_error, 0.003);
}

TEST_F(ClosedLoop, ADelayedLoopWithAGainFarAboveTheBoundDoesNotSettle) {
  SimScenario sc = Nominal();
  sc.delay = 0.3;                                                // d = 3: bound lambda T < 0.445, lambda < 4.45 /s
  sc.params.lambda = 30.0;
  sc.params.v_max = 0.04;
  EXPECT_GT(sim_->Run(sc).worst_error_last_10s, 0.005);                // keeps hunting (the speed cap turns the divergence into a limit cycle)
}

TEST_F(ClosedLoop, StillConvergesWithMeasurementNoiseDroppedFramesAndDelayJitter) {
  SimScenario sc = Nominal();
  sc.noise_sigma = 0.001;                                        // 1 mm per axis (3 mm in depth)
  sc.dropout = 0.2;
  sc.delay_jitter = 0.05;
  const SimResult r = sim_->Run(sc);
  EXPECT_FALSE(r.aborted) << r.abort_reason;
  EXPECT_LT(r.worst_error_last_10s, 0.008);
  EXPECT_GT(r.frames_dropped, 20);                               // the dropout actually happened
}

TEST_F(ClosedLoop, StartingOffToTheSideStillConverges) {
  SimScenario sc = Nominal();
  sc.p_e0 = Eigen::Vector3d(0.06, 0.04, 0.28);
  EXPECT_LT(sim_->Run(sc).final_error, 0.004);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
