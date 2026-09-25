#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/inverse_kinematics.h>

#include "arm_trajectory/GenerateTrajectory.h"
#include "arm_trajectory/cartesian_trajectory.h"
#include "arm_trajectory/joint_space_trajectory.h"

// Advertises /generate_trajectory: computes (but does not execute) a
// point-to-point trajectory -- joint-space, or Cartesian if a goal_pose
// is given with no goal_joint_positions. Fast/synchronous, hence a plain
// service rather than an action (execution, which has a real duration and
// needs to be preemptible, is trajectory_executor_node's job).
class TrajectoryServiceNode {
 public:
  TrajectoryServiceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    pnh.param<double>("default_sample_rate", default_sample_rate_, 50.0);

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("trajectory_service_node: failed to load URDF from 'robot_description'");
    }

    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    ik_solver_ = std::make_unique<arm_kinematics::InverseKinematicsSolver>(*fk_solver_);
    joint_names_ = fk_solver_->GetActuatedJointNames();
    velocity_limits_ = fk_solver_->GetActuatedJointVelocityLimits();
    latest_joint_positions_.assign(joint_names_.size(), 0.0);
    have_joint_state_ = false;

    joint_sub_ = nh.subscribe("/joint_states_in", 10, &TrajectoryServiceNode::JointStatesCb, this);
    generate_srv_ =
        nh.advertiseService("/generate_trajectory", &TrajectoryServiceNode::GenerateCb, this);
    ROS_INFO_STREAM("trajectory_service_node: ready, " << root_link << " -> " << tip_link << " over "
                                                        << joint_names_.size() << " actuated joints");
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        return;
      }
      positions[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  std::vector<double> ResolveStart(const std::vector<double>& requested) {
    if (!requested.empty()) {
      return requested;
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!have_joint_state_) {
      ROS_WARN("trajectory_service_node: no /joint_states_in received yet, starting from all-zero");
    }
    return latest_joint_positions_;
  }

  bool GenerateCb(arm_trajectory::GenerateTrajectory::Request& req,
                  arm_trajectory::GenerateTrajectory::Response& res) {
    std::vector<double> start;
    try {
      start = ResolveStart(std::vector<double>(req.start_joint_positions.begin(),
                                                req.start_joint_positions.end()));
    } catch (const std::exception& e) {
      ROS_ERROR("trajectory_service_node: %s", e.what());
      return false;
    }
    double sample_rate = req.sample_rate > 0.0 ? req.sample_rate : default_sample_rate_;

    if (!req.goal_joint_positions.empty()) {
      std::vector<double> goal(req.goal_joint_positions.begin(), req.goal_joint_positions.end());
      if (goal.size() != joint_names_.size() || start.size() != joint_names_.size()) {
        res.success = false;
        res.message = "goal_joint_positions/start size mismatch with actuated joint count (" +
                      std::to_string(joint_names_.size()) + ")";
        return true;
      }
      try {
        res.trajectory = arm_trajectory::GenerateJointSpaceTrajectory(start, goal, velocity_limits_,
                                                                       joint_names_, sample_rate);
        res.success = true;
      } catch (const std::exception& e) {
        res.success = false;
        res.message = e.what();
      }
      return true;
    }

    // Cartesian goal.
    Eigen::Isometry3d goal_pose;
    tf2::fromMsg(req.goal_pose, goal_pose);

    if (!req.cartesian_path) {
      // Endpoint only matters: solve IK once, then a plain joint-space
      // trajectory to that solution -- no path-tracking approximation
      // involved, so no peak_*_deviation to report.
      arm_kinematics::IkResult ik = ik_solver_->Solve(goal_pose, start);
      if (!ik.converged) {
        res.success = false;
        res.message = "goal pose did not converge: position error " +
                      std::to_string(ik.residual.head<3>().norm()) + " m, orientation error " +
                      std::to_string(ik.residual.tail<3>().norm()) + " rad, after " +
                      std::to_string(ik.restarts_used) + " restarts";
        return true;
      }
      try {
        res.trajectory = arm_trajectory::GenerateJointSpaceTrajectory(
            start, ik.joint_positions, velocity_limits_, joint_names_, sample_rate);
        res.success = true;
      } catch (const std::exception& e) {
        res.success = false;
        res.message = e.what();
      }
      return true;
    }

    arm_trajectory::CartesianTrajectoryResult result;
    try {
      result = arm_trajectory::GenerateCartesianTrajectory(*fk_solver_, *ik_solver_, start, goal_pose,
                                                            velocity_limits_, joint_names_,
                                                            sample_rate);
    } catch (const std::exception& e) {
      res.success = false;
      res.message = e.what();
      return true;
    }
    res.success = result.success;
    res.message = result.message;
    res.peak_position_deviation = result.peak_position_deviation;
    res.peak_orientation_deviation = result.peak_orientation_deviation;
    if (result.success) {
      res.trajectory = result.trajectory;
    }
    return true;
  }

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<arm_kinematics::InverseKinematicsSolver> ik_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  double default_sample_rate_;

  ros::Subscriber joint_sub_;
  ros::ServiceServer generate_srv_;

  std::mutex joint_state_mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_service_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    TrajectoryServiceNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("trajectory_service_node: %s", e.what());
    return 1;
  }
  return 0;
}
