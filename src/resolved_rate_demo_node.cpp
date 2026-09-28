#include <algorithm>
#include <cmath>
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
// NOT part of arm_teleop: a continuous task-space command, streamed at
// ~rate to /joint_states_out, with the null-space bias active throughout
// so ~nullspace_tip_link visibly straightens toward vertical in rviz
// while the arm moves -- exactly the scenario the bias is gated on (see
// resolved_rate.h: no primary command, no bias, by design), so a real
// command has to be running to see it at all.
//
// The commanded direction reverses every ~oscillation_period seconds
// (default vx=0.05 m/s, flipping every 4s) rather than jogging one way
// forever -- confirmed the hard way running this live: a one-directional
// jog runs the arm into a joint limit within a few seconds (this arm's
// workspace is a ~1.2m-radius envelope, see
// arm_kinematics/docs/workspace_identification.md) and then just sits
// there pinned once every joint clamps, with nothing left to watch.
// Oscillating keeps the null-space bias something to act against
// indefinitely.
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
    pnh.param("oscillation_period", oscillation_period_, 4.0);
    // Default: end_effector below base_link and pointing down (tool axis
    // ~(0,-0.13,-0.99)), with ~nullspace_tip_link (link_4) essentially
    // exactly HORIZONTAL to start (dot with world Z ~ -1e-4) -- found by
    // scanning random in-limit configurations for the most tilted-from-
    // vertical one satisfying "below base, pointing down", not
    // hand-derived (this chain's per-joint URDF rpy offsets make that
    // impractical to do reliably by hand). Maximum room for the
    // null-space bias to visibly straighten it toward vertical.
    std::vector<double> default_start = {-1.5747, 2.7009, -0.4995, 2.8188, -3.5092};
    pnh.param("start_joint_positions", start_joint_positions_, default_start);
    pnh.param("setup_move_duration", setup_move_duration_, 3.0);
    if (start_joint_positions_.size() != default_start.size()) {
      throw std::runtime_error("resolved_rate_demo_node: ~start_joint_positions must have " +
                                std::to_string(default_start.size()) + " values");
    }

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

    MoveToStartConfiguration();

    ROS_INFO("resolved_rate_demo_node: ready -- streaming vx=%.3f vy=%.3f vz=%.3f pitch_rate=%.3f, "
             "nullspace_gain=%.2f biasing %s toward vertical",
             vx_, vy_, vz_, pitch_rate_, nullspace_gain_, nullspace_tip_link.c_str());

    start_time_ = ros::Time::now();
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_), &ResolvedRateDemoNode::TimerCb, this);
  }

 private:
  // Straight-line joint-space interpolation from wherever the sim actually
  // is to start_joint_positions_, published directly (bypassing
  // ComputeResolvedRateVelocity entirely -- this is just getting into
  // position for the demo, not something the resolved-rate/null-space
  // math needs to be involved in).
  void MoveToStartConfiguration() {
    std::vector<double> q_from = LatestJointPositions();
    const int num_steps = std::max(1, static_cast<int>(setup_move_duration_ * rate_hz_));
    ros::Rate move_rate(rate_hz_);
    for (int step = 1; step <= num_steps; ++step) {
      double frac = static_cast<double>(step) / num_steps;
      sensor_msgs::JointState cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.name = joint_names_;
      cmd.position.resize(q_from.size());
      for (size_t i = 0; i < q_from.size(); ++i) {
        cmd.position[i] = q_from[i] + frac * (start_joint_positions_[i] - q_from[i]);
      }
      joint_out_pub_.publish(cmd);
      ros::spinOnce();
      move_rate.sleep();
    }
    ROS_INFO("resolved_rate_demo_node: moved to demo start configuration");
  }

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

    double elapsed = (ros::Time::now() - start_time_).toSec();
    double sign = std::fmod(elapsed, 2.0 * oscillation_period_) < oscillation_period_ ? 1.0 : -1.0;

    arm_kinematics::ResolvedRateResult result = arm_kinematics::ComputeResolvedRateVelocity(
        *fk_solver_, q, sign * vx_, sign * vy_, sign * vz_, sign * pitch_rate_, velocity_limits_,
        velocity_damping_, nullspace_solver_.get(), nullspace_gain_);

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
  double oscillation_period_;
  std::vector<double> start_joint_positions_;
  double setup_move_duration_;
  ros::Time start_time_;

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
