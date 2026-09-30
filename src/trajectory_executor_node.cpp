#include <mutex>
#include <string>
#include <vector>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <arm_kinematics/joint_state_utils.h>

// Plays a trajectory_msgs::JointTrajectory back to /joint_states_out at
// each point's time_from_start, via the standard
// control_msgs/FollowJointTrajectoryAction interface -- chosen over a
// custom blocking service specifically for the preemption/cancellation it
// gives for free: M4 (joystick teleop, a new target arriving mid-motion)
// and M5 (hardware e-stop) both need to be able to interrupt an
// in-progress trajectory, which a blocking call can't do. See
// docs/trajectory_generation.md.
//
// Deliberately minimal: no trajectory blending/queuing, one goal at a
// time (SimpleActionServer's default behavior -- a new goal preempts
// whatever's running). Publishes basic feedback (desired vs. latest
// /joint_states_in actual) but doesn't compute per-joint error summaries
// beyond what the message already carries raw.
class TrajectoryExecutorNode {
 public:
  using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;

  TrajectoryExecutorNode(ros::NodeHandle& nh)
      : nh_(nh),
        server_(nh, "follow_joint_trajectory",
                boost::bind(&TrajectoryExecutorNode::Execute, this, _1), false) {
    joint_out_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_out", 10);
    joint_in_sub_ = nh_.subscribe("/joint_states_in", 10, &TrajectoryExecutorNode::JointStatesCb, this);
    server_.start();
    ROS_INFO("trajectory_executor_node: ready, action server on ~follow_joint_trajectory");
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    latest_joint_state_ = *msg;
  }

  sensor_msgs::JointState LatestJointState() {
    std::lock_guard<std::mutex> lock(latest_state_mutex_);
    return latest_joint_state_;
  }

  void Preempt() {
    control_msgs::FollowJointTrajectoryResult result;
    server_.setPreempted(result, "preempted mid-trajectory");
  }

  void Execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
    const auto& traj = goal->trajectory;
    if (traj.points.empty()) {
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      server_.setAborted(result, "empty trajectory");
      return;
    }

    ros::Time start_time = ros::Time::now();
    for (size_t i = 0; i < traj.points.size(); ++i) {
      const auto& point = traj.points[i];

      // Wait until this point's scheduled time, checking for preemption
      // (or node shutdown) throughout rather than only between points --
      // a long final segment shouldn't delay responding to a cancel.
      ros::Time point_time = start_time + point.time_from_start;
      ros::Rate wait_rate(50.0);
      while (ros::ok() && ros::Time::now() < point_time) {
        if (server_.isPreemptRequested() || !ros::ok()) {
          Preempt();
          return;
        }
        wait_rate.sleep();
      }
      if (server_.isPreemptRequested()) {
        Preempt();
        return;
      }

      sensor_msgs::JointState cmd;
      cmd.header.stamp = ros::Time::now();
      cmd.name = traj.joint_names;
      cmd.position = point.positions;
      joint_out_pub_.publish(cmd);

      control_msgs::FollowJointTrajectoryFeedback feedback;
      feedback.header.stamp = ros::Time::now();
      feedback.joint_names = traj.joint_names;
      feedback.desired = point;
      sensor_msgs::JointState actual_state = LatestJointState();
      feedback.actual.positions.resize(traj.joint_names.size(), 0.0);
      for (size_t j = 0; j < traj.joint_names.size(); ++j) {
        arm_kinematics::FindJointPosition(actual_state, traj.joint_names[j], &feedback.actual.positions[j]);
      }
      server_.publishFeedback(feedback);
    }

    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    server_.setSucceeded(result, "trajectory complete");
  }

  ros::NodeHandle& nh_;
  Server server_;
  ros::Publisher joint_out_pub_;
  ros::Subscriber joint_in_sub_;

  std::mutex latest_state_mutex_;
  sensor_msgs::JointState latest_joint_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_executor_node");
  ros::NodeHandle nh;
  // SimpleActionServer's execute-callback constructor already runs
  // Execute() in its own background thread, so a plain spin() here
  // (handling the goal/cancel/feedback plumbing and our two topics) is
  // standard and sufficient -- no separate spinner needed.
  TrajectoryExecutorNode node(nh);
  ros::spin();
  return 0;
}
