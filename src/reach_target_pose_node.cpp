#include <algorithm>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>

#include "arm_trajectory/GenerateTrajectory.h"
#include "arm_trajectory/ReachTargetPose.h"

// Non-interactive "go to this pose" entry point -- the M4-teleop-free
// replacement for this branch's milestone. Deliberately thin: no new
// planning/IK/generation logic, just orchestrates the two pieces M3
// already built and tested (/generate_trajectory, follow_joint_trajectory)
// behind a single blocking call, then verifies via FK where the arm
// actually ended up rather than trusting "trajectory generation
// succeeded" as the whole story. See docs/trajectory_generation.md.
class ReachTargetPoseNode {
 public:
  using ActionClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  ReachTargetPoseNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), action_client_("follow_joint_trajectory", true) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    // Matches InverseKinematicsSolver's own default tolerances (see
    // arm_kinematics/inverse_kinematics.h): the goal IK solve inside
    // /generate_trajectory was already required to hit this precision, so
    // holding the EXECUTED result to the same bar is the natural check,
    // not a new arbitrary number.
    pnh.param("position_tolerance", position_tolerance_, 1e-4);
    pnh.param("orientation_tolerance", orientation_tolerance_, 1e-3);
    // Defensive cap on how long a single execution is allowed to run
    // before this call gives up waiting -- trajectory_executor_node
    // always terminates the action on its own (success/abort/preempt), so
    // this is a backstop against something being badly wrong elsewhere in
    // the graph, not expected to fire in normal operation.
    pnh.param("execution_timeout_margin", execution_timeout_margin_, 10.0);

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("reach_target_pose_node: failed to load URDF from 'robot_description'");
    }
    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();

    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &ReachTargetPoseNode::JointStatesCb, this);
    generate_client_ = nh_.serviceClient<arm_trajectory::GenerateTrajectory>("/generate_trajectory");
    reach_srv_ = nh_.advertiseService("/reach_target_pose", &ReachTargetPoseNode::ReachCb, this);

    ROS_INFO("reach_target_pose_node: waiting for /generate_trajectory and follow_joint_trajectory ...");
    generate_client_.waitForExistence();
    action_client_.waitForServer();
    ROS_INFO_STREAM("reach_target_pose_node: ready, " << root_link << " -> " << tip_link);
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
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  bool LatestJointPositions(std::vector<double>* out) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!have_joint_state_) {
      return false;
    }
    *out = latest_joint_positions_;
    return true;
  }

  bool ReachCb(arm_trajectory::ReachTargetPose::Request& req,
               arm_trajectory::ReachTargetPose::Response& res) {
    arm_trajectory::GenerateTrajectory generate;
    generate.request.goal_pose = req.target_pose;
    generate.request.cartesian_path = req.cartesian_path;
    generate.request.sample_rate = req.sample_rate;

    if (!generate_client_.call(generate)) {
      res.success = false;
      res.message = "reach_target_pose: /generate_trajectory call failed (service unavailable?)";
      return true;
    }
    if (!generate.response.success) {
      res.success = false;
      res.message = "reach_target_pose: trajectory generation failed: " + generate.response.message;
      return true;
    }
    if (generate.response.trajectory.points.empty()) {
      res.success = false;
      res.message = "reach_target_pose: generated trajectory has no points";
      return true;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = generate.response.trajectory;
    goal.trajectory.header.stamp = ros::Time::now();
    action_client_.sendGoal(goal);

    double planned_duration = goal.trajectory.points.back().time_from_start.toSec();
    bool finished = action_client_.waitForResult(
        ros::Duration(planned_duration + execution_timeout_margin_));
    if (!finished) {
      action_client_.cancelGoal();
      res.success = false;
      res.message = "reach_target_pose: execution did not finish within " +
                     std::to_string(planned_duration + execution_timeout_margin_) + "s, cancelled";
      return true;
    }
    if (action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      res.success = false;
      res.message = "reach_target_pose: execution did not succeed: " +
                     action_client_.getState().toString() + " (" +
                     action_client_.getState().getText() + ")";
      return true;
    }

    std::vector<double> final_q;
    if (!LatestJointPositions(&final_q)) {
      res.success = false;
      res.message = "reach_target_pose: execution finished but no /joint_states_in received yet, "
                     "cannot verify final pose";
      return true;
    }

    Eigen::Isometry3d final_pose = fk_solver_->ComputePose(final_q);
    Eigen::Isometry3d target_pose;
    tf2::fromMsg(req.target_pose, target_pose);

    double position_error = (final_pose.translation() - target_pose.translation()).norm();
    Eigen::AngleAxisd orientation_diff(final_pose.rotation().transpose() * target_pose.rotation());
    double orientation_error = std::fabs(orientation_diff.angle());

    res.final_position_error = position_error;
    res.final_orientation_error = orientation_error;

    if (position_error > position_tolerance_ || orientation_error > orientation_tolerance_) {
      res.success = false;
      res.message = "reach_target_pose: execution succeeded but final pose is out of tolerance "
                     "(position error " +
                     std::to_string(position_error) + " m, orientation error " +
                     std::to_string(orientation_error) + " rad)";
      return true;
    }

    res.success = true;
    res.message = "reached target pose";
    return true;
  }

  ros::NodeHandle& nh_;
  ros::Subscriber joint_sub_;
  ros::ServiceClient generate_client_;
  ros::ServiceServer reach_srv_;
  ActionClient action_client_;

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::vector<std::string> joint_names_;
  double position_tolerance_;
  double orientation_tolerance_;
  double execution_timeout_margin_;

  std::mutex joint_state_mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "reach_target_pose_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // The action client's own feedback/result callbacks and the service
  // callback both need to make progress while ReachCb blocks inside
  // waitForResult() -- an AsyncSpinner (not plain ros::spin(), which is
  // single-threaded and would deadlock here) handles both concurrently.
  ros::AsyncSpinner spinner(2);
  spinner.start();
  try {
    ReachTargetPoseNode node(nh, pnh);
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("reach_target_pose_node: %s", e.what());
    return 1;
  }
  return 0;
}
