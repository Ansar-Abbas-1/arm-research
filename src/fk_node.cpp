#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/joint_state_utils.h"

// Thin ROS wrapper: subscribes to the joint-state topic (same convention
// arm_interface/arm_sim_validator_node publish to, /joint_states_in) and
// publishes the computed tip-link pose to a plain topic -- deliberately
// not into TF, since robot_state_publisher already broadcasts the same
// frame from the same URDF; a second broadcaster would conflict with it,
// and a separate topic is what makes comparing the two meaningful.
class ForwardKinematicsNode {
 public:
  ForwardKinematicsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("fk_node: failed to load URDF from 'robot_description' param");
    }

    solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    joint_names_ = solver_->GetActuatedJointNames();
    ROS_INFO_STREAM("fk_node: solving " << root_link << " -> " << tip_link << " over "
                                         << joint_names_.size() << " actuated joints");

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/computed_end_effector_pose", 10);
    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &ForwardKinematicsNode::JointStatesCb, this);
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> joint_positions;
    std::string missing_joint;
    if (!arm_kinematics::ExtractJointPositions(*msg, joint_names_, &joint_positions, &missing_joint)) {
      ROS_WARN_THROTTLE(2.0, "fk_node: /joint_states_in is missing required joint '%s'",
                         missing_joint.c_str());
      return;
    }

    Eigen::Isometry3d pose;
    try {
      pose = solver_->ComputePose(joint_positions);
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "fk_node: ComputePose failed: %s", e.what());
      return;
    }

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    pose_msg.header.frame_id = solver_->root_link();
    pose_msg.pose = tf2::toMsg(pose);
    pose_pub_.publish(pose_msg);
  }

  ros::NodeHandle& nh_;
  ros::Publisher pose_pub_;
  ros::Subscriber joint_sub_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> solver_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fk_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    ForwardKinematicsNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("fk_node: %s", e.what());
    return 1;
  }
  return 0;
}
