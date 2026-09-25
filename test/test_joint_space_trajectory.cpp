#include <gtest/gtest.h>

#include <cmath>

#include "arm_trajectory/joint_space_trajectory.h"

using arm_trajectory::GenerateJointSpaceTrajectory;

namespace {
const std::vector<std::string> kJointNames = {"j1", "j2", "j3"};
}  // namespace

TEST(JointSpaceTrajectory, StartsAndEndsAtRequestedPositions) {
  std::vector<double> q_start = {0.0, 0.5, -0.3};
  std::vector<double> q_goal = {1.0, -0.5, 0.2};
  std::vector<double> v_limits = {1.0, 1.0, 1.0};

  auto traj = GenerateJointSpaceTrajectory(q_start, q_goal, v_limits, kJointNames, 50.0);
  ASSERT_GE(traj.points.size(), 2u);
  EXPECT_EQ(traj.points.front().positions, q_start);
  EXPECT_EQ(traj.points.back().positions, q_goal);
  EXPECT_NEAR(traj.points.front().time_from_start.toSec(), 0.0, 1e-9);
}

TEST(JointSpaceTrajectory, SynchronizedArrivalAllJointsSameDuration) {
  // joint 0 needs much more time than joints 1/2 given its velocity limit;
  // all three must still arrive at the SAME final time_from_start.
  std::vector<double> q_start = {0.0, 0.0, 0.0};
  std::vector<double> q_goal = {10.0, 0.1, 0.1};
  std::vector<double> v_limits = {1.0, 100.0, 100.0};

  auto traj = GenerateJointSpaceTrajectory(q_start, q_goal, v_limits, kJointNames, 50.0);
  double T = traj.points.back().time_from_start.toSec();
  EXPECT_GT(T, 10.0);  // dominated by joint 0's slow limit
  for (const auto& point : traj.points) {
    EXPECT_LE(point.time_from_start.toSec(), T + 1e-9);
  }
}

TEST(JointSpaceTrajectory, VelocityNeverExceedsLimitAtAnySample) {
  std::vector<double> q_start = {-1.0, 2.0, 0.0};
  std::vector<double> q_goal = {1.0, -2.0, 3.0};
  std::vector<double> v_limits = {0.5, 1.5, 2.0};

  auto traj = GenerateJointSpaceTrajectory(q_start, q_goal, v_limits, kJointNames, 100.0);
  for (const auto& point : traj.points) {
    ASSERT_EQ(point.velocities.size(), v_limits.size());
    for (size_t i = 0; i < v_limits.size(); ++i) {
      EXPECT_LE(std::fabs(point.velocities[i]), v_limits[i] + 1e-6) << "joint " << i;
    }
  }
}

TEST(JointSpaceTrajectory, ConsecutiveSampleDeltaMatchesFiniteDifferencedVelocity) {
  // Independent cross-check: the analytic velocities populated in the
  // message should match what you'd compute by finite-differencing the
  // positions across consecutive samples.
  std::vector<double> q_start = {0.0, 0.0, 0.0};
  std::vector<double> q_goal = {2.0, -1.0, 0.5};
  std::vector<double> v_limits = {1.0, 1.0, 1.0};

  auto traj = GenerateJointSpaceTrajectory(q_start, q_goal, v_limits, kJointNames, 200.0);
  for (size_t k = 1; k + 1 < traj.points.size(); ++k) {
    double dt = traj.points[k + 1].time_from_start.toSec() - traj.points[k - 1].time_from_start.toSec();
    for (size_t i = 0; i < kJointNames.size(); ++i) {
      double fd = (traj.points[k + 1].positions[i] - traj.points[k - 1].positions[i]) / dt;
      EXPECT_NEAR(traj.points[k].velocities[i], fd, 1e-2) << "sample " << k << " joint " << i;
    }
  }
}

TEST(JointSpaceTrajectory, AlreadyAtGoalReturnsSinglePoint) {
  std::vector<double> q = {0.1, 0.2, 0.3};
  std::vector<double> v_limits = {1.0, 1.0, 1.0};
  auto traj = GenerateJointSpaceTrajectory(q, q, v_limits, kJointNames, 50.0);
  ASSERT_EQ(traj.points.size(), 1u);
  EXPECT_EQ(traj.points[0].positions, q);
  EXPECT_NEAR(traj.points[0].time_from_start.toSec(), 0.0, 1e-9);
}

TEST(JointSpaceTrajectory, SizeMismatchThrows) {
  std::vector<double> q3 = {0.0, 0.0, 0.0};
  std::vector<double> q2 = {0.0, 0.0};
  std::vector<double> v3 = {1.0, 1.0, 1.0};
  EXPECT_THROW(GenerateJointSpaceTrajectory(q3, q2, v3, kJointNames, 50.0), std::invalid_argument);
  EXPECT_THROW(GenerateJointSpaceTrajectory(q3, q3, v3, kJointNames, 0.0), std::invalid_argument);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
