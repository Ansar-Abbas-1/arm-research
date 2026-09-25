#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include "arm_kinematics/SolveIk.h"
#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/inverse_kinematics.h"

// ROS wrapper around InverseKinematicsSolver: advertises /solve_ik, and
// tracks /joint_states_in as the default seed when a request's
// seed_joint_positions is empty (same joint-state topic/convention as
// fk_node and arm_interface).
class InverseKinematicsNode {
 public:
  InverseKinematicsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "link_5");

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("ik_node: failed to load URDF from 'robot_description' param");
    }

    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link,
                                                                            tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();
    latest_joint_positions_.assign(joint_names_.size(), 0.0);
    have_joint_state_ = false;

    ik_solver_ = std::make_unique<arm_kinematics::InverseKinematicsSolver>(*fk_solver_);

    joint_sub_ = nh.subscribe("/joint_states_in", 10, &InverseKinematicsNode::JointStatesCb, this);
    solve_srv_ = nh.advertiseService("/solve_ik", &InverseKinematicsNode::SolveCb, this);
    ROS_INFO_STREAM("ik_node: ready, solving " << root_link << " -> " << tip_link << " over "
                                                << joint_names_.size() << " actuated joints");
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        return;  // incomplete message; keep the previous cached state
      }
      positions[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  bool SolveCb(arm_kinematics::SolveIk::Request& req, arm_kinematics::SolveIk::Response& res) {
    std::vector<double> seed;
    if (!req.seed_joint_positions.empty()) {
      seed.assign(req.seed_joint_positions.begin(), req.seed_joint_positions.end());
    } else {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_joint_state_) {
        ROS_WARN("ik_node: no /joint_states_in received yet, seeding from all-zero");
      }
      seed = latest_joint_positions_;
    }

    Eigen::Isometry3d target;
    tf2::fromMsg(req.target_pose, target);

    arm_kinematics::IkResult result;
    try {
      result = ik_solver_->Solve(target, seed);
    } catch (const std::exception& e) {
      ROS_ERROR("ik_node: Solve failed: %s", e.what());
      return false;  // malformed request (e.g. bad seed size) -> service call error
    }

    res.converged = result.converged;
    res.joint_positions.assign(result.joint_positions.begin(), result.joint_positions.end());
    res.residual.assign(result.residual.data(), result.residual.data() + result.residual.size());
    res.iterations = result.iterations;
    res.restarts_used = result.restarts_used;
    return true;
  }

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<arm_kinematics::InverseKinematicsSolver> ik_solver_;
  std::vector<std::string> joint_names_;

  ros::Subscriber joint_sub_;
  ros::ServiceServer solve_srv_;

  std::mutex joint_state_mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ik_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    InverseKinematicsNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("ik_node: %s", e.what());
    return 1;
  }
  return 0;
}
