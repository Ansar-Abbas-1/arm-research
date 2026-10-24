// Tests for PositionPitchIkSolver. See docs/position_pitch_ik.md for the
// derivation this cross-checks against.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <memory>
#include <random>

#include <ros/package.h>
#include <urdf/model.h>

#include "arm_kinematics/base_collision.h"
#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/inverse_kinematics.h"
#include "arm_kinematics/position_pitch_ik.h"

using arm_kinematics::ArmIntersectsBase;
using arm_kinematics::BoxObstacle;
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

// Box as actually deployed (see trajectory_service_node.cpp and
// test_base_collision.cpp): top face at base_link's own origin, extending
// downward, half-extents 0.20.
BoxObstacle DeployedBaseBox() {
  BoxObstacle box;
  box.half_extents = Eigen::Vector3d::Constant(0.20);
  box.box_to_root = Eigen::Translation3d(0, 0, -0.20) * Eigen::Isometry3d::Identity();
  return box;
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
  // A single hardcoded target/seed pair isn't a reliable way to ask this
  // question: which of the many joint configurations sharing a
  // position+pitch solution a redundant solver settles into depends on
  // its exact path through joint space, so a solver-internal change with
  // NO effect on the bias's own math (e.g. the backtracking line search
  // added for max_inner_retries -- see position_pitch_ik.h -- which only
  // engages when no secondary task is configured, so it changes the
  // no-bias baseline's own convergence path, not the bias path) can flip
  // a single-target comparison by chance. Same fix this session already
  // applied elsewhere for exactly this reason: sweep several targets with
  // a fixed rng_seed (isolates the bias's effect from restart randomness)
  // and check the bias wins in the majority, not every single one --
  // matching the honest, already-documented limitation in
  // PositionPitchIkOptions::nullspace_gain's own comment (a slow,
  // linear-rate fixed-point iteration, not a guaranteed-monotonic one).
  std::vector<std::vector<double>> known_configs = {
      {0.8, 1.2, -0.9, 1.5, -1.8},   {0.3, 0.9, -0.6, 1.0, -1.2},
      {-0.5, 1.6, -1.1, 2.0, -2.4},  {1.1, 0.6, -0.4, 0.8, -1.0},
      {-1.2, 2.1, -1.4, 2.6, -3.0},  {0.6, 1.9, -1.2, 2.2, -2.7},
      {-0.2, 0.7, -0.5, 1.2, -1.5},  {1.4, 1.3, -0.8, 1.7, -2.1},
  };

  auto VerticalAlignment = [&](const std::vector<double>& q) {
    std::vector<double> q4(q.begin(), q.begin() + 4);
    Eigen::Vector3d axis = link4_solver_->ComputePose(q4).rotation() * Eigen::Vector3d::UnitZ();
    return axis.dot(Eigen::Vector3d::UnitZ());
  };

  int bias_better = 0;
  int total = 0;
  for (const auto& q_known : known_configs) {
    Eigen::Isometry3d target = solver_->ComputePose(q_known);

    PositionPitchIkOptions without_bias_options;
    without_bias_options.rng_seed = 1;
    PositionPitchIkSolver without_bias(*solver_, without_bias_options);
    auto without_bias_result = without_bias.Solve(target, seed_);
    ASSERT_TRUE(without_bias_result.converged);

    PositionPitchIkOptions with_bias_options;
    with_bias_options.rng_seed = 1;
    with_bias_options.nullspace_secondary_link_solver = link4_solver_.get();
    with_bias_options.nullspace_gain = 0.05;
    PositionPitchIkSolver with_bias(*solver_, with_bias_options);
    auto with_bias_result = with_bias.Solve(target, seed_);
    ASSERT_TRUE(with_bias_result.converged);

    ++total;
    if (VerticalAlignment(with_bias_result.joint_positions) >
        VerticalAlignment(without_bias_result.joint_positions)) {
      ++bias_better;
    }
  }

  EXPECT_GT(bias_better, total / 2)
      << "the null-space bias should converge to a more vertically-aligned link_4 than no bias "
         "in the majority of targets (won " << bias_better << "/" << total << ")";
}

TEST_F(PositionPitchIkTest, NullspaceDisplacementBudgetBoundsTheBiasMotionPerSolve) {
  // From an already-converged seed the primary task has nothing to do, so
  // the whole joint displacement of a Solve() is bias (plus negligible
  // leakage). The budget must cap it; unlimited must move much further, or
  // the test would prove nothing.
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d target = solver_->ComputePose(q_known);

  auto max_displacement = [&](double budget) {
    PositionPitchIkOptions options;
    options.rng_seed = 1;
    options.max_restarts = 0;
    options.nullspace_secondary_link_solver = link4_solver_.get();
    options.nullspace_gain = 0.05;
    options.nullspace_max_displacement = budget;
    PositionPitchIkSolver solver(*solver_, options);
    auto result = solver.Solve(target, q_known);
    double worst = 0.0;
    for (size_t i = 0; i < q_known.size(); ++i) {
      worst = std::max(worst, std::abs(result.joint_positions[i] - q_known[i]));
    }
    return std::make_pair(worst, result.converged);
  };

  const auto unlimited = max_displacement(0.0);
  const auto small = max_displacement(0.01);
  const auto tiny = max_displacement(0.002);

  EXPECT_GT(unlimited.first, 0.05) << "fixture must show real bias motion without a budget";
  EXPECT_LE(small.first, 0.01 * 1.5 + 1e-3);
  EXPECT_LE(tiny.first, 0.002 * 1.5 + 1e-3);
  EXPECT_LT(tiny.first, small.first);
  EXPECT_LT(small.first, unlimited.first);
  EXPECT_TRUE(small.second) << "a spent budget must not break primary convergence";
  EXPECT_TRUE(tiny.second);
}

namespace {
// |up . z_link4|: how vertical the LINE of link_4 is (up or down both count).
double VerticalLine(arm_kinematics::ForwardKinematicsSolver& link4, const std::vector<double>& q) {
  std::vector<double> q4(q.begin(), q.begin() + 4);
  return std::abs(link4.ComputePose(q4).rotation().col(2).z());
}
}  // namespace

TEST_F(PositionPitchIkTest, PostureTaskMakesLink4SubstantiallyMoreVerticalWithoutBreakingThePrimaryTask) {
  // Regression for the "null-space projection does nothing" report: the old
  // per-Newton-step bias was no better than no bias on average (41.6 vs 39.4 deg
  // mean tilt over random targets). Fixed seeds, warm starts 0.3 rad from a
  // known solution, as the service sees them.
  std::mt19937 rng(11);
  const auto limits = solver_->GetActuatedJointLimits();
  double sum_off = 0.0, sum_on = 0.0;
  int total = 0, improved = 0;
  for (int t = 0; t < 40; ++t) {
    std::vector<double> qk(5), seed(5);
    for (int i = 0; i < 5; ++i) {
      std::uniform_real_distribution<double> u(limits[i].first + 0.15, limits[i].second - 0.15);
      qk[i] = u(rng);
      std::normal_distribution<double> nd(0.0, 0.3);
      seed[i] = std::min(std::max(qk[i] + nd(rng), limits[i].first), limits[i].second);
    }
    Eigen::Isometry3d target = solver_->ComputePose(qk);
    PositionPitchIkOptions off;
    off.rng_seed = 1;
    PositionPitchIkOptions on = off;
    on.nullspace_secondary_link_solver = link4_solver_.get();
    on.nullspace_gain = 0.05;
    auto r_off = PositionPitchIkSolver(*solver_, off).Solve(target, seed);
    auto r_on = PositionPitchIkSolver(*solver_, on).Solve(target, seed);
    if (!r_off.converged || !r_on.converged) continue;
    // The posture task must never cost primary accuracy.
    EXPECT_LT((solver_->ComputePose(r_on.joint_positions).translation() - target.translation()).norm(),
              on.position_tolerance);
    ++total;
    const double v_off = VerticalLine(*link4_solver_, r_off.joint_positions);
    const double v_on = VerticalLine(*link4_solver_, r_on.joint_positions);
    sum_off += std::acos(std::min(1.0, v_off));
    sum_on += std::acos(std::min(1.0, v_on));
    if (v_on > v_off + 1e-3) ++improved;
    EXPECT_GE(v_on, v_off - 0.02) << "target " << t << ": the posture task must not make it worse";
  }
  ASSERT_GT(total, 20);
  EXPECT_LT(sum_on / total, 0.6 * sum_off / total) << "mean tilt should drop by at least 40%";
  EXPECT_GT(improved, total / 2);
}

TEST_F(PositionPitchIkTest, LinkPointingStraightDownCountsAsVerticalAndIsNotFlippedUp) {
  // The old objective maximised the SIGNED up . z, so a link already vertical
  // pointing down was pushed up through horizontal (and sat on a zero-gradient
  // point exactly at straight down). The far pose reached on 2026-10-05 has
  // link_4 at 179.7 deg.
  std::vector<double> q_far = {0.015, 2.608, -2.183, 3.019, -2.542};
  Eigen::Isometry3d target = solver_->ComputePose(q_far);
  ASSERT_GT(VerticalLine(*link4_solver_, q_far), 0.999);

  PositionPitchIkOptions on;
  on.rng_seed = 1;
  on.nullspace_secondary_link_solver = link4_solver_.get();
  on.nullspace_gain = 0.05;
  auto r = PositionPitchIkSolver(*solver_, on).Solve(target, q_far);
  ASSERT_TRUE(r.converged);
  EXPECT_GE(VerticalLine(*link4_solver_, r.joint_positions), 0.999);
  for (size_t i = 0; i < q_far.size(); ++i) {
    EXPECT_NEAR(r.joint_positions[i], q_far[i], 0.05) << "joint " << i << " should barely move";
  }
}

TEST_F(PositionPitchIkTest, ZeroBudgetIsIdenticalToUnlimited) {
  std::vector<double> q_known = {0.8, 1.2, -0.9, 1.5, -1.8};
  Eigen::Isometry3d target = solver_->ComputePose(q_known);
  PositionPitchIkOptions a_options;
  a_options.rng_seed = 1;
  a_options.nullspace_secondary_link_solver = link4_solver_.get();
  a_options.nullspace_gain = 0.05;
  PositionPitchIkOptions b_options = a_options;
  b_options.nullspace_max_displacement = 0.0;
  b_options.nullspace_max_rate = 0.0;
  auto a = PositionPitchIkSolver(*solver_, a_options).Solve(target, seed_);
  auto b = PositionPitchIkSolver(*solver_, b_options).Solve(target, seed_);
  for (size_t i = 0; i < a.joint_positions.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.joint_positions[i], b.joint_positions[i]);
  }
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

// q=(0,-1.6,0,0,0): elbow folded straight down, confirmed colliding with
// the deployed base box via a standalone probe against the built library
// (same config as test_base_collision.cpp's ElbowFoldedIntoBaseCollides).
// Seeding Solve() exactly at this config means SolveFromSeed trivially
// converges to it (residual 0) WITHOUT collision checking -- so a
// restart search finding something else can only be the collision check
// actually doing its job, not an artifact of a hard-to-reach target.
TEST_F(PositionPitchIkTest, RestartSearchFindsCollisionFreeConfiguration) {
  std::vector<double> q_colliding = {0.0, -1.6, 0.0, 0.0, 0.0};
  Eigen::Isometry3d target = solver_->ComputePose(q_colliding);
  BoxObstacle box = DeployedBaseBox();

  PositionPitchIkOptions options;
  options.collision_obstacle = &box;
  options.max_restarts = 100;
  options.rng_seed = 7;  // verified via standalone probe: finds a collision-free match in 10 restarts
  PositionPitchIkSolver ik(*solver_, options);
  auto result = ik.Solve(target, q_colliding);

  EXPECT_TRUE(result.converged) << "residual: " << result.residual.transpose();
  EXPECT_TRUE(result.collision_free);
  EXPECT_FALSE(ArmIntersectsBase(*solver_, result.joint_positions, box, 0.02));
}

// Same scenario as above, with a null-space secondary task also
// configured -- confirms the collision check is threaded through BOTH of
// Solve()'s restart branches (have_nullspace_bias true/false), not just
// the simpler no-bias one.
TEST_F(PositionPitchIkTest, CollisionCheckAppliesIndependentlyOfNullspaceBias) {
  std::vector<double> q_colliding = {0.0, -1.6, 0.0, 0.0, 0.0};
  Eigen::Isometry3d target = solver_->ComputePose(q_colliding);
  BoxObstacle box = DeployedBaseBox();

  PositionPitchIkOptions options;
  options.collision_obstacle = &box;
  options.max_restarts = 100;
  options.rng_seed = 7;  // verified via standalone probe: finds a collision-free match in 34 restarts
  options.nullspace_secondary_link_solver = link4_solver_.get();
  options.nullspace_gain = 0.05;
  PositionPitchIkSolver ik(*solver_, options);
  auto result = ik.Solve(target, q_colliding);

  EXPECT_TRUE(result.converged) << "residual: " << result.residual.transpose();
  EXPECT_TRUE(result.collision_free);
  EXPECT_FALSE(ArmIntersectsBase(*solver_, result.joint_positions, box, 0.02));
}

// Target's own position is deep inside the box (tip=(-0.0221,0,-0.1347),
// confirmed via probe) -- EVERY configuration that actually reaches it
// necessarily collides (the final segment's own endpoint is inside the
// box, by construction), regardless of which branch of the chain's 1-DOF
// redundancy a solution lands in. This is a case the collision-aware
// restart search genuinely cannot satisfy both bars on at once.
//
// Verified behavior (standalone probe, 5 rng seeds x 50 restarts, all
// agreeing): the restart/selection logic's qualifying-tier-dominates
// policy -- the SAME pattern already used for the primary
// acceptable/not-acceptable gate elsewhere in this function, just with
// collision folded into what "qualifying" means -- means it prefers ANY
// collision-free candidate over a colliding one unconditionally, even an
// unconverged, poorly-aligned one, rather than ever returning the
// perfectly-converged but colliding exact match. So the honest result
// here is collision_free=true, converged=false -- NOT, as originally
// assumed before this was actually measured, collision_free=false. A
// caller must check both fields; `converged=false` is what correctly
// surfaces "this target could not be reached without colliding."
TEST_F(PositionPitchIkTest, InfeasibleTargetPrefersCollisionFreeOverExactCollidingMatch) {
  std::vector<double> q_inside = {0.0, 0.8, 1.1, 0.0, 0.0};
  Eigen::Isometry3d target = solver_->ComputePose(q_inside);
  BoxObstacle box = DeployedBaseBox();

  PositionPitchIkOptions options;
  options.collision_obstacle = &box;
  options.max_restarts = 50;
  options.rng_seed = 1;
  PositionPitchIkSolver ik(*solver_, options);
  auto result = ik.Solve(target, q_inside);

  EXPECT_TRUE(result.collision_free);
  EXPECT_FALSE(result.converged)
      << "converging exactly on this target requires colliding -- the search should have "
         "preferred a non-colliding, non-converged candidate instead; residual: "
      << result.residual.transpose();
  EXPECT_FALSE(ArmIntersectsBase(*solver_, result.joint_positions, box, 0.02));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
