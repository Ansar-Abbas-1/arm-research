#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>

#include "arm_trajectory/ReachTargetPose.h"

// RViz-interactive front end for /reach_target_pose (see
// docs/trajectory_generation.md): a 6-DOF marker the operator drags to
// the desired end-effector pose, committed via a right-click menu entry
// rather than continuous jogging -- a one-shot "set target, then go"
// interaction, not a control loop, so it can't accumulate the class of
// drift/sync bugs a streamed joystick command stream can. Deliberately no
// new planning logic: this is purely a pose source feeding the same
// service the CLI (reach_pose_cli.py) and any future caller (M6/M7
// perception) would use.
class InteractivePoseMarkerNode {
 public:
  InteractivePoseMarkerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), server_("interactive_pose_marker") {
    pnh.param<std::string>("root_link", root_link_, "base_link");
    std::string tip_link;
    pnh.param<std::string>("tip_link", tip_link, "end_effector");

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error(
          "interactive_pose_marker_node: failed to load URDF from 'robot_description'");
    }
    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link_, tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();

    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &InteractivePoseMarkerNode::JointStatesCb, this);
    reach_client_ = nh_.serviceClient<arm_trajectory::ReachTargetPose>("/reach_target_pose");

    ROS_INFO("interactive_pose_marker_node: waiting for /joint_states_in and /reach_target_pose ...");
    ros::Rate wait_rate(10.0);
    while (ros::ok() && !HaveJointState()) {
      wait_rate.sleep();  // AsyncSpinner (started in main) services JointStatesCb concurrently
    }
    reach_client_.waitForExistence();

    CreateMarkerAtCurrentPose();
    ROS_INFO(
        "interactive_pose_marker_node: ready. In rviz, add an InteractiveMarkers display on "
        "/interactive_pose_marker/update, drag the marker (frame '%s'), then right-click -> "
        "'Send to /reach_target_pose' to commit.",
        root_link_.c_str());
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

  Eigen::Isometry3d CurrentPose() {
    std::lock_guard<std::mutex> lock(mutex_);
    return fk_solver_->ComputePose(latest_joint_positions_);
  }

  static void NormalizeQuaternion(geometry_msgs::Quaternion& q) {
    double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n > 1e-9) {
      q.w /= n;
      q.x /= n;
      q.y /= n;
      q.z /= n;
    }
  }

  // A control's move/rotate axis is its own local X axis, so orienting
  // the control's quaternion to point local X along world X/Y/Z (in turn)
  // is the standard way to get one move+rotate control pair per world
  // axis -- see interactive_markers' own basic_controls tutorial for the
  // same construction.
  void AddAxisControls(visualization_msgs::InteractiveMarker& int_marker, double qx, double qy,
                        double qz, const std::string& axis_name) {
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = qx;
    control.orientation.y = qy;
    control.orientation.z = qz;
    NormalizeQuaternion(control.orientation);

    control.name = "move_" + axis_name;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "rotate_" + axis_name;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }

  void CreateMarkerAtCurrentPose() {
    Eigen::Isometry3d pose = CurrentPose();

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = root_link_;
    int_marker.header.stamp = ros::Time::now();
    int_marker.pose = tf2::toMsg(pose);
    int_marker.scale = 0.2;
    int_marker.name = "target_pose";
    int_marker.description = "";

    // Always-visible orange arrow so the pending (uncommitted) target is
    // legible even with no control actively engaged -- same visual
    // convention arm_teleop's own reference/jog-target markers used.
    visualization_msgs::Marker visual;
    visual.type = visualization_msgs::Marker::ARROW;
    visual.scale.x = 0.10;
    visual.scale.y = 0.02;
    visual.scale.z = 0.02;
    visual.color.r = 1.0f;
    visual.color.g = 0.5f;
    visual.color.b = 0.0f;
    visual.color.a = 0.9f;
    visualization_msgs::InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;
    visual_control.markers.push_back(visual);
    int_marker.controls.push_back(visual_control);

    AddAxisControls(int_marker, 1, 0, 0, "x");
    AddAxisControls(int_marker, 0, 1, 0, "y");
    AddAxisControls(int_marker, 0, 0, 1, "z");

    // Small green sphere, always visible, dedicated to the right-click
    // menu -- separate from the drag controls above so a menu click can't
    // be mistaken for (or interfere with) a drag.
    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.always_visible = true;
    visualization_msgs::Marker menu_marker;
    menu_marker.type = visualization_msgs::Marker::SPHERE;
    menu_marker.scale.x = menu_marker.scale.y = menu_marker.scale.z = 0.03;
    menu_marker.color.r = 0.0f;
    menu_marker.color.g = 1.0f;
    menu_marker.color.b = 0.3f;
    menu_marker.color.a = 0.9f;
    menu_control.markers.push_back(menu_marker);
    int_marker.controls.push_back(menu_control);

    server_.insert(int_marker, boost::bind(&InteractivePoseMarkerNode::FeedbackCb, this, _1));
    menu_handler_.insert("Send to /reach_target_pose",
                          boost::bind(&InteractivePoseMarkerNode::SendCb, this, _1));
    menu_handler_.apply(server_, int_marker.name);
    server_.applyChanges();
  }

  // POSE_UPDATE feedback needs no handling: the server already tracks the
  // marker's live pose on its own; SendCb below reads it fresh at commit
  // time via the feedback message's own `pose` field.
  void FeedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&) {}

  void SendCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    arm_trajectory::ReachTargetPose srv;
    srv.request.target_pose = feedback->pose;
    srv.request.cartesian_path = false;

    ROS_INFO("interactive_pose_marker_node: sending target pose to /reach_target_pose ...");
    if (!reach_client_.call(srv)) {
      ROS_WARN("interactive_pose_marker_node: /reach_target_pose call failed (service unavailable?)");
      return;
    }
    if (!srv.response.success) {
      ROS_WARN_STREAM("interactive_pose_marker_node: reach failed: " << srv.response.message);
      return;
    }
    ROS_INFO_STREAM("interactive_pose_marker_node: reached target (position error "
                     << srv.response.final_position_error << " m, orientation error "
                     << srv.response.final_orientation_error << " rad)");

    // Snap the marker back to the arm's new actual pose so the next drag
    // starts from reality, not wherever it was dropped -- the same
    // "don't let a stale target keep looking live" lesson arm_teleop's
    // jog-target reset-on-release fix was for.
    Eigen::Isometry3d actual = CurrentPose();
    server_.setPose(feedback->marker_name, tf2::toMsg(actual));
    server_.applyChanges();
  }

  ros::NodeHandle& nh_;
  interactive_markers::InteractiveMarkerServer server_;
  interactive_markers::MenuHandler menu_handler_;
  ros::Subscriber joint_sub_;
  ros::ServiceClient reach_client_;

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::vector<std::string> joint_names_;
  std::string root_link_;

  std::mutex mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "interactive_pose_marker_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // SendCb blocks on /reach_target_pose for the whole motion -- same
  // reasoning as reach_target_pose_node itself: a single-threaded spin()
  // would starve JointStatesCb for that entire window, so the
  // post-commit marker-reset would read a stale, pre-motion pose.
  ros::AsyncSpinner spinner(2);
  spinner.start();
  try {
    InteractivePoseMarkerNode node(nh, pnh);
    ros::waitForShutdown();
  } catch (const std::exception& e) {
    ROS_FATAL("interactive_pose_marker_node: %s", e.what());
    return 1;
  }
  return 0;
}
