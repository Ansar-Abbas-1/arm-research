#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/resolved_rate.h"

// Minimal, standalone demo of ComputeResolvedRateVelocity and its
// null-space secondary task -- neither has a ROS wrapper anywhere yet
// (both are pure library code, gtest-only so far), and this is deliberately
// NOT part of arm_teleop: a fixed, continuous task-space command (default:
// a slow +X jog), streamed at ~rate to /joint_states_out, with the
// null-space bias active throughout so ~nullspace_tip_link visibly
// straightens toward vertical in rviz while the arm moves -- exactly the
// scenario the bias is gated on (see resolved_rate.h: no primary command,
// no bias, by design), so a real command has to be running to see it at all.
class ResolvedRateDemoNode {
 public:
  ResolvedRateDemoNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
    std::string root_link, tip_link, nullspace_tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    pnh.param<std::string>("nullspace_tip_link", nullspace_tip_link, "link_4");
    pnh.param("rate", rate_hz_, 50.0);
    pnh.param("velocity_damping", velocity_damping_, 0.05);
    pnh.param("nullspace_gain", nullspace_gain_, 0.4);
    pnh.param("vx", vx_, 0.05);
    pnh.param("vy", vy_, 0.0);
    pnh.param("vz", vz_, 0.0);
    pnh.param("pitch_rate", pitch_rate_, 0.0);

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("resolved_rate_demo_node: failed to load URDF from 'robot_description'");
    }
    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    nullspace_solver_ =
        std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, nullspace_tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();
    velocity_limits_ = fk_solver_->GetActuatedJointVelocityLimits();
    joint_limits_ = fk_solver_->GetActuatedJointLimits();

    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &ResolvedRateDemoNode::JointStatesCb, this);
    joint_out_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_out", 10);

    ROS_INFO("resolved_rate_demo_node: waiting for /joint_states_in ...");
    ros::Rate wait_rate(10.0);
    while (ros::ok() && !HaveJointState()) {
      ros::spinOnce();
      wait_rate.sleep();
    }
    ROS_INFO("resolved_rate_demo_node: ready -- streaming vx=%.3f vy=%.3f vz=%.3f pitch_rate=%.3f, "
             "nullspace_gain=%.2f biasing %s toward vertical",
             vx_, vy_, vz_, pitch_rate_, nullspace_gain_, nullspace_tip_link.c_str());

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_), &ResolvedRateDemoNode::TimerCb, this);
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        return;  // wait for a message that names every actuated joint
      }
      positions[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  bool HaveJointState() {
    std::lock_guard<std::mutex> lock(mutex_);
    return have_joint_state_;
  }

  std::vector<double> LatestJointPositions() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_joint_positions_;
  }

  void TimerCb(const ros::TimerEvent&) {
    std::vector<double> q = LatestJointPositions();
    const double dt = 1.0 / rate_hz_;

    arm_kinematics::ResolvedRateResult result = arm_kinematics::ComputeResolvedRateVelocity(
        *fk_solver_, q, vx_, vy_, vz_, pitch_rate_, velocity_limits_, velocity_damping_,
        nullspace_solver_.get(), nullspace_gain_);

    sensor_msgs::JointState cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.name = joint_names_;
    cmd.position.resize(q.size());
    for (size_t i = 0; i < q.size(); ++i) {
      double q_next = q[i] + result.joint_velocities[i] * dt;
      cmd.position[i] = std::min(joint_limits_[i].second, std::max(joint_limits_[i].first, q_next));
    }
    joint_out_pub_.publish(cmd);
  }

  ros::NodeHandle& nh_;
  ros::Subscriber joint_sub_;
  ros::Publisher joint_out_pub_;
  ros::Timer timer_;

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> nullspace_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  std::vector<std::pair<double, double>> joint_limits_;

  double rate_hz_;
  double velocity_damping_;
  double nullspace_gain_;
  double vx_, vy_, vz_, pitch_rate_;

  std::mutex mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "resolved_rate_demo_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    ResolvedRateDemoNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("resolved_rate_demo_node: %s", e.what());
    return 1;
  }
  return 0;
}
