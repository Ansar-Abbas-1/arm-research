// Tests for PositionPitchIkSolver. See docs/position_pitch_ik.md for the
// derivation this cross-checks against.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/inverse_kinematics.h"
#include "arm_kinematics/position_pitch_ik.h"

using arm_kinematics::PositionPitchIkOptions;
using arm_kinematics::PositionPitchIkSolver;

namespace {

urdf::Model LoadArmModel() {
  urdf::Model model;
  std::string arm_interface_path = ros::package::getPath("arm_interface");
  EXPECT_FALSE(arm_interface_path.empty()) << "arm_interface package not found";
  EXPECT_TRUE(model.initFile(arm_interface_path + "/urdf/arm.urdf"));
  return model;
}

double PitchComponent(const Eigen::Isometry3d& pose_a, const Eigen::Isometry3d& pose_b) {
  // Same convention as the solver itself: project the angle-axis rotation
  // error onto pose_a's own local Y axis.
  Eigen::Vector3d pitch_axis = pose_a.rotation() * Eigen::Vector3d::UnitY();
  Eigen::AngleAxisd aa(pose_a.rotation() * pose_b.rotation().transpose());
  return pitch_axis.dot(aa.angle() * aa.axis());
}

}  // namespace

class PositionPitchIkTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_ = LoadArmModel();
    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link",
                                                                          "end_effector");
    link4_solver_ =
        std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model_, "base_link", "link_4");
    // Not {0,0,0,0,0}: a genuine kinematic singularity (M3 Finding 1),
    // not a fair seed for a generic convergence check.
    seed_ = {0.3, 0.5, -0.2, 0.4, -0.3};
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> link4_solver_;
  std::vector<double> seed_;
};

TEST_F(PositionPitchIkTest, ReachesKnownReachableTargetsPositionAndPitch) {
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d target = solver_->ComputePose(q_known);

  PositionPitchIkSolver ik(*solver_);
  auto result = ik.Solve(target, seed_);
  ASSERT_TRUE(result.converged) << "residual: " << result.residual.transpose();

  Eigen::Isometry3d achieved = solver_->ComputePose(result.joint_positions);
  EXPECT_LT((achieved.translation() - target.translation()).norm(), 1e-3);
  EXPECT_LT(std::abs(PitchComponent(achieved, target)), 1e-2);
}

TEST_F(PositionPitchIkTest, ConvergesOnASmallRandomTargetBenchmark) {
  // Smaller N than inverse_kinematics.md's own 20-target benchmark, to
  // keep this suite fast -- position+pitch is a strictly easier target
  // than a full 6D pose on this chain (M2: only position+one angle is
  // independently reachable at all), so a smaller sample is still a
  // meaningful convergence check, not a coverage compromise.
  std::mt19937 rng(42);
  const auto& limits = solver_->GetActuatedJointLimits();
  std::vector<std::uniform_real_distribution<double>> dists;
  for (const auto& lim : limits) dists.emplace_back(lim.first, lim.second);

  PositionPitchIkSolver ik(*solver_, PositionPitchIkOptions{});
  int converged_count = 0;
  const int kNumTargets = 10;
  for (int t = 0; t < kNumTargets; ++t) {
    std::vector<double> q_target(limits.size());
    for (size_t i = 0; i < limits.size(); ++i) q_target[i] = dists[i](rng);
    Eigen::Isometry3d target = solver_->ComputePose(q_target);

    auto result = ik.Solve(target, seed_);
    if (result.converged) ++converged_count;
  }
  EXPECT_EQ(converged_count, kNumTargets)
      << "position+pitch should converge reliably given restarts, same as full-pose IK's own "
         "benchmark in inverse_kinematics.md";
}

TEST_F(PositionPitchIkTest, NullspaceBiasImprovesVerticalAlignmentAtConvergence) {
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d target = solver_->ComputePose(q_known);

  PositionPitchIkOptions without_bias_options;
  PositionPitchIkSolver without_bias(*solver_, without_bias_options);
  auto without_bias_result = without_bias.Solve(target, seed_);
  ASSERT_TRUE(without_bias_result.converged);

  PositionPitchIkOptions with_bias_options;
  with_bias_options.nullspace_secondary_link_solver = link4_solver_.get();
  with_bias_options.nullspace_gain = 0.05;
  PositionPitchIkSolver with_bias(*solver_, with_bias_options);
  auto with_bias_result = with_bias.Solve(target, seed_);
  ASSERT_TRUE(with_bias_result.converged);

  auto VerticalAlignment = [&](const std::vector<double>& q) {
    std::vector<double> q4(q.begin(), q.begin() + 4);
    Eigen::Vector3d axis = link4_solver_->ComputePose(q4).rotation() * Eigen::Vector3d::UnitZ();
    return axis.dot(Eigen::Vector3d::UnitZ());
  };

  EXPECT_GT(VerticalAlignment(with_bias_result.joint_positions),
            VerticalAlignment(without_bias_result.joint_positions))
      << "the same target, solved with the null-space bias enabled, should converge to a more "
         "vertically-aligned link_4 than without it";
}

TEST_F(PositionPitchIkTest, ZeroGainMatchesNoSecondaryTaskAtAll) {
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d target = solver_->ComputePose(q_known);

  PositionPitchIkOptions with_solver_zero_gain_options;
  with_solver_zero_gain_options.nullspace_secondary_link_solver = link4_solver_.get();
  with_solver_zero_gain_options.nullspace_gain = 0.0;
  PositionPitchIkSolver with_solver_zero_gain(*solver_, with_solver_zero_gain_options);

  PositionPitchIkSolver without_solver(*solver_, PositionPitchIkOptions{});

  auto a = with_solver_zero_gain.Solve(target, seed_);
  auto b = without_solver.Solve(target, seed_);
  ASSERT_TRUE(a.converged);
  ASSERT_TRUE(b.converged);
  for (size_t i = 0; i < a.joint_positions.size(); ++i) {
    EXPECT_NEAR(a.joint_positions[i], b.joint_positions[i], 1e-9)
        << "gain=0 must disable the secondary task exactly, regardless of a solver being passed";
  }
}

TEST_F(PositionPitchIkTest, ConvergesOnATargetTheFullPoseIkCannotReach) {
  // A target whose orientation deliberately isn't on this arm's reachable
  // 5D submanifold (M2 finding): take a genuinely reachable pose and bolt
  // on an extra large roll rotation that has nothing to do with pitch. A
  // large, generic perturbation like this lands off the (measure-zero-ish)
  // reachable submanifold with overwhelming probability -- confirmed
  // empirically here (not assumed) via the full IkResult below.
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d reachable = solver_->ComputePose(q_known);
  Eigen::Isometry3d unreachable_orientation = reachable;
  unreachable_orientation.linear() = reachable.rotation() * Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitX());

  arm_kinematics::InverseKinematicsSolver full_ik(*solver_);
  auto full_result = full_ik.Solve(unreachable_orientation, seed_);
  ASSERT_FALSE(full_result.converged)
      << "test setup assumption broken: the perturbed target turned out to be fully reachable "
         "after all, this test needs a different perturbation";

  PositionPitchIkSolver reduced_ik(*solver_);
  auto reduced_result = reduced_ik.Solve(unreachable_orientation, seed_);
  EXPECT_TRUE(reduced_result.converged)
      << "position+pitch of this target should still be reachable even though the full 6D pose "
         "isn't -- residual: "
      << reduced_result.residual.transpose();
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
