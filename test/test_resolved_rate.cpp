// Cross-checks ComputeResolvedRateVelocity against independent
// finite-difference/geometric reasoning, not just internal
// self-consistency: for a small dt, integrating the returned joint
// velocities and re-computing FK should move the end-effector by
// approximately the commanded task velocity times dt (first-order
// consistency of the control law), using ComputePose -- a different code
// path than the Jacobian used to derive the velocities in the first place.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_teleop/resolved_rate.h"

using arm_teleop::ComputeResolvedRateVelocity;

namespace {

constexpr double kDt = 1e-4;    // small enough that first-order integration error is negligible
constexpr double kDamping = 0.05;
// The FD cross-check tests use a near-zero damping instead of kDamping:
// a damped pseudo-inverse trades *exact* tracking for singularity-
// robustness by design (that's the entire point), so J_task*qdot is only
// approximately xdot_cmd at kDamping's realistic magnitude -- confirmed
// empirically (~2-4% relative error at kDamping=0.05, consistent with a
// damping-induced bias, not a logic error). Using near-zero damping here
// isolates "is the underlying (undamped) pseudo-inverse math correct" from
// "does damping trade off accuracy as expected" -- the latter is exercised
// qualitatively by VelocityLimitsAreRespected below instead.
constexpr double kNegligibleDamping = 1e-6;
constexpr double kTol = 1e-4;

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

}  // namespace

class ResolvedRateTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                          "end_effector");
    velocity_limits_ = solver_->GetActuatedJointVelocityLimits();
    // A well-conditioned, non-singular seed away from the all-zero
    // configuration (arm_trajectory's Finding 1: {0,0,0,0,0} is a genuine
    // kinematic singularity -- not a fair test point for a generic
    // velocity-tracking check).
    q_ = {0.3, 0.5, -0.2, 0.4, -0.3};
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
  std::vector<double> velocity_limits_;
  std::vector<double> q_;
};

class ResolvedRateNullspaceTest : public ResolvedRateTest {
 protected:
  void SetUp() override {
    ResolvedRateTest::SetUp();
    link4_solver_ =
        std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link", "link_4");
    // Deliberately not vertical (joint_4=0.4 rad away from whatever
    // configuration would make it so), so there's real bias for the
    // secondary task to act on.
    link_axis_before_ = link4_solver_->ComputePose({q_[0], q_[1], q_[2], q_[3]}).rotation() *
                         Eigen::Vector3d::UnitZ();
    ASSERT_LT(std::abs(link_axis_before_.dot(Eigen::Vector3d::UnitZ())), 0.999)
        << "test setup should start visibly non-vertical";
  }

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> link4_solver_;
  Eigen::Vector3d link_axis_before_;
};

TEST_F(ResolvedRateTest, ZeroCommandGivesZeroVelocityAndFullRatio) {
  auto result = ComputeResolvedRateVelocity(*solver_, q_, 0.0, 0.0, 0.0, 0.0, velocity_limits_,
                                             kDamping);
  for (double v : result.joint_velocities) {
    EXPECT_NEAR(v, 0.0, 1e-9);
  }
  EXPECT_NEAR(result.achieved_to_commanded_ratio, 1.0, 1e-9);
}

TEST_F(ResolvedRateTest, LinearVelocityMatchesFiniteDifferenceOfPose) {
  const double vx = 0.05, vy = -0.03, vz = 0.02;
  auto result =
      ComputeResolvedRateVelocity(*solver_, q_, vx, vy, vz, 0.0, velocity_limits_, kNegligibleDamping);
  EXPECT_GT(result.achieved_to_commanded_ratio, 0.95)
      << "away from a singularity, a small linear command should be nearly fully achievable";

  std::vector<double> q_next(q_.size());
  for (size_t i = 0; i < q_.size(); ++i) {
    q_next[i] = q_[i] + result.joint_velocities[i] * kDt;
  }

  Eigen::Isometry3d pose_before = solver_->ComputePose(q_);
  Eigen::Isometry3d pose_after = solver_->ComputePose(q_next);
  Eigen::Vector3d actual_translation = (pose_after.translation() - pose_before.translation()) / kDt;

  EXPECT_NEAR(actual_translation.x(), vx, kTol);
  EXPECT_NEAR(actual_translation.y(), vy, kTol);
  EXPECT_NEAR(actual_translation.z(), vz, kTol);
}

TEST_F(ResolvedRateTest, PitchCommandRotatesAboutEndEffectorsOwnLocalYAxis) {
  const double pitch_rate = 0.2;
  auto result = ComputeResolvedRateVelocity(*solver_, q_, 0.0, 0.0, 0.0, pitch_rate, velocity_limits_,
                                             kNegligibleDamping);

  std::vector<double> q_next(q_.size());
  for (size_t i = 0; i < q_.size(); ++i) {
    q_next[i] = q_[i] + result.joint_velocities[i] * kDt;
  }

  Eigen::Isometry3d pose_before = solver_->ComputePose(q_);
  Eigen::Isometry3d pose_after = solver_->ComputePose(q_next);
  Eigen::Vector3d expected_axis = pose_before.rotation() * Eigen::Vector3d::UnitY();

  Eigen::Vector3d actual_rotation_vector =
      RotationVector(pose_after.rotation() * pose_before.rotation().transpose()) / kDt;

  EXPECT_NEAR(actual_rotation_vector.norm(), pitch_rate, kTol);
  EXPECT_GT(actual_rotation_vector.normalized().dot(expected_axis), 0.999)
      << "rotation should be about the end-effector's own local Y axis, not a root-frame axis";
}

TEST_F(ResolvedRateTest, VelocityLimitsAreRespected) {
  std::vector<double> tiny_limits(velocity_limits_.size(), 0.01);  // artificially tight, forces clamping
  auto result = ComputeResolvedRateVelocity(*solver_, q_, 1.0, 1.0, 1.0, 1.0, tiny_limits, kDamping);
  for (size_t i = 0; i < result.joint_velocities.size(); ++i) {
    EXPECT_LE(std::abs(result.joint_velocities[i]), tiny_limits[i] + 1e-9);
  }
  EXPECT_LT(result.achieved_to_commanded_ratio, 1.0)
      << "an artificially tight velocity limit should measurably reduce the achieved ratio";
}

TEST_F(ResolvedRateNullspaceTest, ZeroGainMatchesNoSecondaryTaskAtAll) {
  auto with_solver_zero_gain = ComputeResolvedRateVelocity(
      *solver_, q_, 0.1, 0.0, 0.0, 0.0, velocity_limits_, kDamping, link4_solver_.get(), 0.0);
  auto without_solver = ComputeResolvedRateVelocity(*solver_, q_, 0.1, 0.0, 0.0, 0.0, velocity_limits_,
                                                     kDamping, nullptr, 0.0);
  for (size_t i = 0; i < q_.size(); ++i) {
    EXPECT_NEAR(with_solver_zero_gain.joint_velocities[i], without_solver.joint_velocities[i], 1e-12)
        << "gain=0 must disable the secondary task exactly, regardless of a solver being passed";
  }
}

TEST_F(ResolvedRateNullspaceTest, NoBiasWithZeroPrimaryCommand) {
  // Gated off deliberately: with nothing commanded, there's no target for
  // the bias to settle into, only a gradient to keep following forever --
  // confirmed the hard way in live testing that leaving this active while
  // idling (deadman held, stick centered) makes the arm visibly drift with
  // no input at all, which looks exactly like a runaway bug.
  const double gain = 0.4;
  auto result = ComputeResolvedRateVelocity(*solver_, q_, 0.0, 0.0, 0.0, 0.0, velocity_limits_,
                                             kNegligibleDamping, link4_solver_.get(), gain);
  for (double v : result.joint_velocities) {
    EXPECT_NEAR(v, 0.0, 1e-9) << "no primary command should mean no motion at all, bias included";
  }
}

TEST_F(ResolvedRateNullspaceTest, BiasesTowardVerticalAlongsideANonzeroPrimaryCommand) {
  // N (the null-space projector) doesn't depend on the primary command,
  // but the primary term itself can incidentally nudge link_4's alignment
  // too -- in either direction, depending on the specific command and
  // configuration -- so comparing raw before/after alignment with the
  // bias on conflates two effects. Isolate the bias's own marginal
  // contribution instead: same primary command, with vs. without the
  // bias, and check alignment specifically improves with it.
  const double vx = 0.02;
  auto without_bias = ComputeResolvedRateVelocity(*solver_, q_, vx, 0.0, 0.0, 0.0, velocity_limits_,
                                                    kNegligibleDamping, link4_solver_.get(), 0.0);
  auto with_bias = ComputeResolvedRateVelocity(*solver_, q_, vx, 0.0, 0.0, 0.0, velocity_limits_,
                                                kNegligibleDamping, link4_solver_.get(), 0.4);

  auto AlignmentAfter = [&](const std::vector<double>& qdot) {
    std::vector<double> q_next(q_.size());
    for (size_t i = 0; i < q_.size(); ++i) {
      q_next[i] = q_[i] + qdot[i] * kDt;
    }
    Eigen::Vector3d axis = link4_solver_->ComputePose({q_next[0], q_next[1], q_next[2], q_next[3]})
                               .rotation() *
                           Eigen::Vector3d::UnitZ();
    return axis.dot(Eigen::Vector3d::UnitZ());
  };

  EXPECT_GT(AlignmentAfter(with_bias.joint_velocities), AlignmentAfter(without_bias.joint_velocities))
      << "adding the null-space bias should improve vertical alignment relative to the same "
         "primary command without it";
}

TEST_F(ResolvedRateNullspaceTest, DoesNotDisturbThePrimaryTask) {
  const double vx = 0.05, vy = -0.03, vz = 0.02;
  auto result = ComputeResolvedRateVelocity(*solver_, q_, vx, vy, vz, 0.0, velocity_limits_,
                                             kNegligibleDamping, link4_solver_.get(), 0.4);
  EXPECT_GT(result.achieved_to_commanded_ratio, 0.95)
      << "a null-space-only bias must not measurably eat into the commanded task velocity";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
