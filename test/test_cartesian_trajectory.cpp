#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <ros/package.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/position_pitch_ik.h>

#include "arm_trajectory/cartesian_trajectory.h"

using arm_trajectory::GenerateCartesianTrajectory;

namespace {

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

double MaxConsecutiveJointStep(const trajectory_msgs::JointTrajectory& traj) {
  double worst = 0.0;
  for (size_t k = 1; k < traj.points.size(); ++k) {
    double step = 0.0;
    for (size_t i = 0; i < traj.points[k].positions.size(); ++i) {
      double d = traj.points[k].positions[i] - traj.points[k - 1].positions[i];
      step += d * d;
    }
    worst = std::max(worst, std::sqrt(step));
  }
  return worst;
}

}  // namespace

class CartesianTrajectoryTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    fk_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                     "end_effector");
    nullspace_fk_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                               "link_4");
    // Default options: no null-space bias (nullspace_gain=0.0) -- most of
    // this suite is about path generation/duration-scaling behavior, not
    // the secondary task (which, once active, deliberately keeps
    // regulating even from an already-primary-converged sample -- see
    // NullspaceOptionsThreadedThroughWholeTrajectory below, which turns it
    // on explicitly and in isolation).
    ik_options_ = arm_kinematics::PositionPitchIkOptions();
    joint_names_ = fk_->GetActuatedJointNames();
    velocity_limits_ = fk_->GetActuatedJointVelocityLimits();
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> nullspace_fk_;
  arm_kinematics::PositionPitchIkOptions ik_options_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
};

TEST_F(CartesianTrajectoryTest, ReachesGuaranteedReachableGoal) {
  // Not {0,0,0,0,0}: that configuration is a genuine kinematic singularity
  // of this arm (all four parallel-axis "elbow" joints sit simultaneously
  // at a limit boundary there -- confirmed via Jacobian SVD, smallest
  // singular value ~1e-33 -- the classic fully-extended-arm singularity).
  // IK correctly can't make progress from exactly on top of a singularity,
  // no matter how tiny the required correction; that's not a solver bug
  // to work around here, see docs/trajectory_generation.md.
  std::vector<double> q_start = {0.1, 0.5, -0.5, 0.5, -0.5};
  std::vector<double> q_goal_config = {0.4, 0.7, -0.3, 0.6, -0.5};
  Eigen::Isometry3d goal_pose = fk_->ComputePose(q_goal_config);

  auto result = GenerateCartesianTrajectory(*fk_, ik_options_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GE(result.trajectory.points.size(), 2u);
  EXPECT_EQ(result.trajectory.points.front().positions, q_start);

  Eigen::Isometry3d achieved = fk_->ComputePose(result.trajectory.points.back().positions);
  EXPECT_LT((achieved.translation() - goal_pose.translation()).norm(), 1e-3);
}

TEST_F(CartesianTrajectoryTest, VelocityLimitsRespectedAfterScaling) {
  std::vector<double> q_start = {-1.0, 0.5, -0.5, 1.0, -1.0};
  std::vector<double> q_goal_config = {1.0, 2.0, -1.5, 3.0, -2.5};
  Eigen::Isometry3d goal_pose = fk_->ComputePose(q_goal_config);

  auto result = GenerateCartesianTrajectory(*fk_, ik_options_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  ASSERT_TRUE(result.success) << result.message;

  const auto& points = result.trajectory.points;
  for (size_t k = 1; k < points.size(); ++k) {
    double dt = (points[k].time_from_start - points[k - 1].time_from_start).toSec();
    ASSERT_GT(dt, 0.0);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      double v = std::fabs(points[k].positions[i] - points[k - 1].positions[i]) / dt;
      EXPECT_LE(v, velocity_limits_[i] * 1.02) << "sample " << k << " joint " << i;  // 2% numeric margin
    }
  }
}

TEST_F(CartesianTrajectoryTest, WarmStartKeepsConsecutiveSamplesClose) {
  // Not a proof warm-starting is engaged (that's an implementation
  // detail), but a real property it should produce: consecutive IK
  // solutions along a smooth Cartesian path shouldn't jump by anything
  // close to the scale of switching elbow configurations (order-1 rad
  // per joint). A sizeable multi-joint move is used specifically so a
  // basin-jump, if warm-starting weren't working, would be plausible --
  // but not the largest possible one: generation time grows sharply with
  // move size (iterative velocity-limit duration-scaling over many
  // samples, each needing its own IK solve -- a known, documented
  // performance limitation for large Cartesian moves, see
  // docs/trajectory_generation.md), and this is a unit test, not a
  // benchmark of that limitation.
  std::vector<double> q_start = {-1.0, 0.2, -0.3, 0.5, -0.5};
  std::vector<double> q_goal_config = {1.0, 1.7, -1.2, 2.0, -2.0};
  Eigen::Isometry3d goal_pose = fk_->ComputePose(q_goal_config);

  auto result = GenerateCartesianTrajectory(*fk_, ik_options_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(MaxConsecutiveJointStep(result.trajectory), 0.3);
}

TEST_F(CartesianTrajectoryTest, UnreachableGoalFailsWithMessageNotGarbage) {
  std::vector<double> q_start = {0.1, 0.5, -0.5, 0.5, -0.5};  // avoid the {0,0,0,0,0} singularity
  Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
  goal_pose.translation() = Eigen::Vector3d(10.0, 10.0, 10.0);  // far outside max reach

  auto result = GenerateCartesianTrajectory(*fk_, ik_options_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST_F(CartesianTrajectoryTest, LargeMoveNeedsMultipleFullResolutionCorrectionPasses) {
  // Regression test for the coarse-to-fine duration search (M3 Finding 4
  // fix): this specific move's coarse (25-sample) search converges on its
  // FIRST duration guess -- the true worst-velocity window is narrow
  // enough that a coarse scan steps right over it -- so nearly all the
  // real convergence work lands in the fine (full-resolution) correction
  // loop, which needs 4 passes here (confirmed via temporary debug
  // instrumentation while developing this). An earlier version hardcoded
  // that loop's cap to 2 regardless of max_duration_iterations and failed
  // this exact move (worst ratio only 1.023 -- one more pass would have
  // converged it) even though the original single-phase algorithm would
  // have succeeded well within its own default budget. The fix ties the
  // fine-phase cap to max_duration_iterations itself, so it can never be
  // tighter than the original algorithm's own convergence guarantee.
  //
  // Slower than this file's other tests (real IK solves across several
  // full-resolution passes, ~300+ samples each) -- deliberately not
  // reduced further: every smaller/cheaper move tried during development
  // converged in a single fine-phase pass and didn't exercise this path
  // at all. Still well under the "not multi-minute" bar
  // docs/trajectory_generation.md sets for this suite.
  std::vector<double> q_start = {-2.5, 0.2, -2.0, 0.5, -3.4};
  std::vector<double> q_goal_config = {2.5, 3.0, -0.1, 4.3, -0.1};
  Eigen::Isometry3d goal_pose = fk_->ComputePose(q_goal_config);

  auto result = GenerateCartesianTrajectory(*fk_, ik_options_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  ASSERT_TRUE(result.success) << result.message;
}

TEST_F(CartesianTrajectoryTest, NullspaceOptionsThreadedThroughWholeTrajectory) {
  // Not a re-test of the bias math itself (arm_kinematics' own
  // PositionPitchIkTest/ResolvedRateNullspaceTest already cover that
  // exhaustively) -- this is a regression test for the PLUMBING added
  // here: that PositionPitchIkOptions' nullspace_secondary_link_solver/
  // nullspace_gain, when a caller sets them (as trajectory_service_node
  // does by default), actually reach both the goal solve AND every
  // interior-sample solve inside GenerateCartesianTrajectory (via
  // PathTrackingIkOptions), not just get silently dropped somewhere in the
  // refactor. Both configs below use fixed rng_seed=1 (deterministic
  // restarts) to isolate the bias's own effect from restart randomness --
  // same methodology arm_kinematics' own nullspace tests use.

  // Below-base, downward-pointing start pose (same one resolved_rate_demo_node
  // uses): link_4 sits essentially exactly horizontal there, maximal room
  // to show straightening.
  std::vector<double> q_start = {-1.5747, 2.7009, -0.4995, 2.8188, -3.5092};
  std::vector<double> q_goal_config = {-1.2, 2.4, -0.3, 2.5, -3.0};
  Eigen::Isometry3d goal_pose = fk_->ComputePose(q_goal_config);

  arm_kinematics::PositionPitchIkOptions no_bias_options = ik_options_;
  no_bias_options.rng_seed = 1;

  auto without_bias = GenerateCartesianTrajectory(*fk_, no_bias_options, q_start, goal_pose,
                                                   velocity_limits_, joint_names_, 50.0);
  ASSERT_TRUE(without_bias.success) << without_bias.message;

  arm_kinematics::PositionPitchIkOptions biased_options = ik_options_;
  biased_options.rng_seed = 1;
  biased_options.nullspace_secondary_link_solver = nullspace_fk_.get();
  biased_options.nullspace_gain = 0.05;  // confirmed-safe default, see position_pitch_ik.h

  auto with_bias = GenerateCartesianTrajectory(*fk_, biased_options, q_start, goal_pose,
                                                velocity_limits_, joint_names_, 50.0);
  ASSERT_TRUE(with_bias.success) << with_bias.message;

  // nullspace_fk_ (base_link -> link_4) has fewer actuated joints than the
  // full chain -- same convention as PositionPitchIkSolver itself (see
  // position_pitch_ik.cpp), take only the leading joints upstream of it.
  const size_t m = nullspace_fk_->GetActuatedJointNames().size();
  auto VerticalAlignment = [&](const std::vector<double>& q) {
    std::vector<double> q_prefix(q.begin(), q.begin() + m);
    Eigen::Vector3d z = nullspace_fk_->ComputePose(q_prefix).rotation() * Eigen::Vector3d::UnitZ();
    return Eigen::Vector3d::UnitZ().dot(z);
  };

  double alignment_without = VerticalAlignment(without_bias.trajectory.points.back().positions);
  double alignment_with = VerticalAlignment(with_bias.trajectory.points.back().positions);
  EXPECT_GT(alignment_with, alignment_without)
      << "nullspace_gain configured via GenerateCartesianTrajectory's ik_options should measurably "
         "improve link_4's vertical alignment at the trajectory's end relative to no bias";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
