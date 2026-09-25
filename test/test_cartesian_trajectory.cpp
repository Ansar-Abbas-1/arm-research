#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <ros/package.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/inverse_kinematics.h>

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
    ik_ = std::make_unique<arm_kinematics::InverseKinematicsSolver>(*fk_);
    joint_names_ = fk_->GetActuatedJointNames();
    velocity_limits_ = fk_->GetActuatedJointVelocityLimits();
  }

  urdf::Model model_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_;
  std::unique_ptr<arm_kinematics::InverseKinematicsSolver> ik_;
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

  auto result = GenerateCartesianTrajectory(*fk_, *ik_, q_start, goal_pose, velocity_limits_,
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

  auto result = GenerateCartesianTrajectory(*fk_, *ik_, q_start, goal_pose, velocity_limits_,
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

  auto result = GenerateCartesianTrajectory(*fk_, *ik_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(MaxConsecutiveJointStep(result.trajectory), 0.3);
}

TEST_F(CartesianTrajectoryTest, UnreachableGoalFailsWithMessageNotGarbage) {
  std::vector<double> q_start = {0.1, 0.5, -0.5, 0.5, -0.5};  // avoid the {0,0,0,0,0} singularity
  Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
  goal_pose.translation() = Eigen::Vector3d(10.0, 10.0, 10.0);  // far outside max reach

  auto result = GenerateCartesianTrajectory(*fk_, *ik_, q_start, goal_pose, velocity_limits_,
                                             joint_names_, 50.0);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(result.trajectory.points.empty());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
