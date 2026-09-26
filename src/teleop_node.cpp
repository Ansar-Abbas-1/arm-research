#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>
#include <visualization_msgs/Marker.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/inverse_kinematics.h>
#include <arm_trajectory/GenerateTrajectory.h>

#include "arm_teleop/joy_target_mapper.h"
#include "arm_teleop/resolved_rate.h"

namespace arm_teleop {

// Two explicit, mutually exclusive modes (toggled by ~mode_toggle_button):
//
//  - DIRECT_JOG (default): resolved-rate velocity control (X/Y/Z + wrist
//    pitch, see resolved_rate.h) streamed directly to /joint_states_out at
//    the control rate. Bypasses arm_trajectory/follow_joint_trajectory
//    entirely -- an action-based executor built for discrete planned
//    trajectories is the wrong abstraction for a continuous ~50Hz position
//    stream, and (the actual motivation) resolved-rate control has no
//    convergence check to fail, unlike the position-IK-per-tick approach
//    this replaced: a saturated/near-singular direction just contributes
//    less achieved velocity (see ~saturation_warn_ratio below), never
//    freezes. See docs/teleop.md for why the original per-tick-IK design
//    (still in this file's git history) got stuck. This chain is
//    redundant for the 4D task (5 joints, 4 rows), so an optional
//    null-space secondary task (~nullspace_gain, ~nullspace_tip_link)
//    biases that leftover freedom toward keeping ~nullspace_tip_link's
//    own local Z axis vertical, without disturbing the commanded task
//    velocity -- see resolved_rate.h and docs/teleop.md for the
//    derivation. ~nullspace_gain=0 disables it entirely.
//  - SET_REFERENCE: the same stick moves an internal reference pose only
//    (position free-running, orientation evolves by pitch alone -- roll/
//    yaw stay exactly as they were when this mode was entered, since every
//    applied rotation is a pure rotation about the pose's own evolving
//    pitch axis). The real arm does not move. ~commit_button sends the
//    reference pose through arm_trajectory's existing, already-tested
//    /generate_trajectory (cartesian_path=false) + follow_joint_trajectory
//    -- a ONE-SHOT planned move, not a per-tick stream, so the service's
//    own default (restart-enabled) IK is fine here: there's no rapid
//    re-preemption to break continuity.
//
// SET_REFERENCE also previews the reference as an arrow marker
// (/teleop_reference_marker) plus a "ghost" arm (/ghost_joint_states, a
// full best-effort IK solution warm-started tick to tick -- default
// IkOptions, restarts enabled: this is a non-critical visual preview, not
// a controller loop, so occasional restart-induced elbow-configuration
// jumps or a slower tick are cosmetic, not the correctness/safety problem
// they were for the per-tick position-IK design this replaced; on a
// non-converged tick the ghost just keeps its last valid configuration
// rather than showing something wrong). In DIRECT_JOG the ghost instead
// mirrors the real arm exactly, so a second RobotModel display pointed at
// it (see docs/teleop.md) is visually indistinguishable from the real one
// whenever it isn't actively being used to preview something.
//
// Switching modes always cancels any in-flight follow_joint_trajectory
// goal first, so trajectory_executor_node's publisher and this node's own
// direct-publish path can never write /joint_states_out at the same time.
//
// Gripper (joint_6): published directly to /joint_states_out, independent
// of both modes above -- confirmed safe against arm_sim_validator_node's
// commandedCb, which updates only the joint indices named in each incoming
// message and leaves the rest untouched. joint_6 is a fixed URDF joint
// (see arm_interface/urdf/arm.urdf) wired to physical motor 6 (the
// gripper) outside the pose kinematic chain, so it has no real URDF
// <limit> to derive gripper_min/max/rate from -- heuristic defaults (see
// joy_target_mapper.h), flagged for recalibration once real gripper
// hardware/limits are known (M5).
class TeleopNode {
 public:
  using ActionClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
  enum class Mode { kDirectJog, kSetReference };

  TeleopNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), action_client_("follow_joint_trajectory", true) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    pnh.param("rate", rate_hz_, 50.0);
    pnh.param("joy_timeout", joy_timeout_, 0.5);
    pnh.param("sample_rate", sample_rate_, 50.0);
    pnh.param("velocity_damping", velocity_damping_, 0.05);
    pnh.param("saturation_warn_ratio", saturation_warn_ratio_, 0.5);

    pnh.param("deadman_button", params_.deadman_button, params_.deadman_button);
    pnh.param("axis_x", params_.axis_x, params_.axis_x);
    pnh.param("axis_y", params_.axis_y, params_.axis_y);
    pnh.param("axis_z", params_.axis_z, params_.axis_z);
    pnh.param("axis_pitch", params_.axis_pitch, params_.axis_pitch);
    pnh.param("axis_deadzone", params_.axis_deadzone, params_.axis_deadzone);
    pnh.param("linear_gain", params_.linear_gain, params_.linear_gain);
    pnh.param("pitch_gain", params_.pitch_gain, params_.pitch_gain);
    pnh.param("gripper_open_button", params_.gripper_open_button, params_.gripper_open_button);
    pnh.param("gripper_close_button", params_.gripper_close_button, params_.gripper_close_button);
    pnh.param("gripper_rate", params_.gripper_rate, params_.gripper_rate);
    pnh.param("gripper_min", params_.gripper_min, params_.gripper_min);
    pnh.param("gripper_max", params_.gripper_max, params_.gripper_max);
    pnh.param("mode_toggle_button", params_.mode_toggle_button, params_.mode_toggle_button);
    pnh.param("commit_button", params_.commit_button, params_.commit_button);

    std::string nullspace_tip_link;
    pnh.param<std::string>("nullspace_tip_link", nullspace_tip_link, "link_4");
    pnh.param("nullspace_gain", nullspace_gain_, 0.4);
    // 0.4 rad/s (per unit misalignment) is heuristic, not derived -- a
    // gentle bias, not strong enough to noticeably fight a deliberate jog
    // command. Set ~nullspace_gain:=0.0 to disable the secondary task
    // entirely (ComputeResolvedRateVelocity treats that as "off").

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("teleop_node: failed to load URDF from 'robot_description' param");
    }
    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    ik_solver_ = std::make_unique<arm_kinematics::InverseKinematicsSolver>(*fk_solver_);
    nullspace_solver_ =
        std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, nullspace_tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();
    velocity_limits_ = fk_solver_->GetActuatedJointVelocityLimits();
    joint_limits_ = fk_solver_->GetActuatedJointLimits();

    joy_sub_ = nh_.subscribe("/joy", 10, &TeleopNode::JoyCb, this);
    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &TeleopNode::JointStatesCb, this);
    joint_out_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_out", 10);
    ghost_pub_ = nh_.advertise<sensor_msgs::JointState>("/ghost_joint_states", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/teleop_reference_marker", 1);
    // Latched: a fresh subscriber (rqt, rostopic echo, a person debugging)
    // sees the current mode immediately, without needing to wait for the
    // next toggle -- there was previously no direct way to observe this
    // from outside the node at all, which made a lengthy live-hardware
    // debugging session (see git history / hand-off notes) far more
    // confusing than it needed to be: the mode-toggle logic was correct
    // the whole time, but repeated toggles across many manual test rounds
    // were easy to lose count of without this.
    mode_pub_ = nh_.advertise<std_msgs::String>("/teleop_mode", 1, /*latch=*/true);
    generate_client_ = nh_.serviceClient<arm_trajectory::GenerateTrajectory>("/generate_trajectory");

    ROS_INFO("teleop_node: waiting for /generate_trajectory and follow_joint_trajectory ...");
    generate_client_.waitForExistence();
    action_client_.waitForServer();
    ROS_INFO("teleop_node: ready at %.1f Hz, mode=DIRECT_JOG (deadman=button %d, mode toggle=button %d)",
              rate_hz_, params_.deadman_button, params_.mode_toggle_button);
    PublishMode();

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_), &TeleopNode::TimerCb, this);
  }

 private:
  void JoyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    last_joy_ = *msg;
    last_joy_stamp_ = ros::Time::now();
    have_joy_ = true;
  }

  // Continuous mirror of the real robot's joint state -- deliberately NOT
  // a one-time seed: DIRECT_JOG's integration is closed-loop off this
  // (measured actual state), not open-loop off its own accumulated
  // command, to avoid drifting away from where the arm really is.
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
    actual_joint_positions_ = positions;
    have_actual_state_ = true;
  }

  std::vector<double> ActualJointPositions() {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return actual_joint_positions_;
  }

  void PublishMode() {
    std_msgs::String msg;
    msg.data = (mode_ == Mode::kDirectJog) ? "DIRECT_JOG" : "SET_REFERENCE";
    mode_pub_.publish(msg);
  }

  void PublishGripper() {
    sensor_msgs::JointState cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.name = {"joint_6"};
    cmd.position = {gripper_position_};
    joint_out_pub_.publish(cmd);
  }

  void PublishGhost() {
    sensor_msgs::JointState ghost;
    ghost.header.stamp = ros::Time::now();
    ghost.name = joint_names_;
    ghost.name.push_back("joint_6");
    ghost.position = ghost_joint_positions_;
    ghost.position.push_back(gripper_position_);
    ghost_pub_.publish(ghost);
  }

  void PublishArrowMarker(const std::string& ns, const Eigen::Isometry3d& pose, int action,
                          float r, float g, float b) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = fk_solver_->root_link();
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = action;
    if (action == visualization_msgs::Marker::ADD) {
      marker.pose = tf2::toMsg(pose);
      marker.scale.x = 0.10;   // arrow length
      marker.scale.y = 0.015;  // head diameter
      marker.scale.z = 0.015;  // head length
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.9;
    }
    marker_pub_.publish(marker);
  }

  void PublishReferenceMarker(int action) {
    // Orange while the current reference is reachable (matches the ghost
    // preview / what commit would do right now), red when it isn't --
    // purely informational, never blocks the target from continuing to
    // move (see UpdateGhostPreview).
    if (reference_reachable_) {
      PublishArrowMarker("teleop_reference", reference_pose_, action, 1.0f, 0.5f, 0.0f);
    } else {
      PublishArrowMarker("teleop_reference", reference_pose_, action, 1.0f, 0.0f, 0.0f);
    }
  }

  void PublishJogTargetMarker(int action) {
    PublishArrowMarker("teleop_jog_target", jog_target_pose_, action, 0.0f, 1.0f, 0.3f);  // green
  }

  static void ApplyPitchRotation(Eigen::Isometry3d& pose, double pitch_rate, double dt) {
    if (pitch_rate == 0.0) {
      return;
    }
    Eigen::Vector3d pitch_axis = pose.rotation() * Eigen::Vector3d::UnitY();
    Eigen::AngleAxisd delta(pitch_rate * dt, pitch_axis);
    pose.linear() = (delta.toRotationMatrix() * pose.rotation()).eval();
  }

  // The reference pose is a free-flying virtual target, not real arm
  // motion, so nothing here ever stops it from moving -- reported and
  // confirmed as an over-correction in an earlier version, which rejected
  // (froze) the ENTIRE jog increment the instant a single tick's IK didn't
  // converge, with no way for the user to recover except reversing
  // direction and hoping. Only two things actually need reachability:
  // this best-effort ghost preview (M3 Finding 2's "best-effort interior
  // samples, only the goal needs to be exactly reachable" pattern, applied
  // here to "only the eventual commit target needs to be exactly
  // reachable, not every point the reference passes through") and, for
  // real, CommitReference() itself -- which already has its own check and
  // already warns+no-ops on failure, unaffected by any of this.
  //
  // On a non-converged tick, ghost_joint_positions_ simply keeps its last
  // valid configuration (never shows a wrong one) and reference_reachable_
  // flips false so the marker can say so -- both purely informational.
  // Warm-started from the last valid ghost solution, restarts enabled
  // (default IkOptions) -- see the class comment for why that's fine for
  // this cosmetic-preview, one-shot-commit use case.
  void UpdateGhostPreview() {
    arm_kinematics::IkResult ik = ik_solver_->Solve(reference_pose_, ghost_joint_positions_);
    reference_reachable_ = ik.converged;
    if (ik.converged) {
      ghost_joint_positions_ = ik.joint_positions;
    } else {
      ROS_WARN_THROTTLE(1.0,
                         "teleop_node: reference pose currently unreachable (residual %.4f m / "
                         "%.4f rad) -- ghost preview frozen, still tracking your jog input though",
                         ik.residual.head<3>().norm(), ik.residual.tail<3>().norm());
    }
  }

  void CommitReference() {
    arm_trajectory::GenerateTrajectory srv;
    srv.request.goal_pose = tf2::toMsg(reference_pose_);
    srv.request.cartesian_path = false;
    srv.request.sample_rate = sample_rate_;

    if (!generate_client_.call(srv)) {
      ROS_WARN("teleop_node: commit failed -- /generate_trajectory call failed (service unavailable?)");
      return;
    }
    if (!srv.response.success) {
      ROS_WARN("teleop_node: commit failed -- reference pose unreachable: %s",
                srv.response.message.c_str());
      return;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = srv.response.trajectory;
    goal.trajectory.header.stamp = ros::Time::now();
    action_client_.sendGoal(goal);
    pending_action_goal_ = true;
    ROS_INFO("teleop_node: reference committed, executing (%zu points)",
              srv.response.trajectory.points.size());
  }

  void TimerCb(const ros::TimerEvent&) {
    if (!have_actual_state_) {
      ROS_WARN_THROTTLE(2.0, "teleop_node: no /joint_states_in received yet, idle");
      return;
    }
    std::vector<double> q = ActualJointPositions();
    if (ghost_joint_positions_.empty()) {
      ghost_joint_positions_ = q;  // seed before the first-ever PublishGhost() call
    }
    if (!have_jog_target_) {
      jog_target_pose_ = fk_solver_->ComputePose(q);
      have_jog_target_ = true;
    }

    if (!have_joy_ || (ros::Time::now() - last_joy_stamp_).toSec() > joy_timeout_) {
      return;  // no joystick input, or it went stale (disconnected/topic died): stay put
    }

    JoyCommand cmd = ComputeJoyCommand(last_joy_, prev_tick_joy_, params_);
    prev_tick_joy_ = last_joy_;  // snapshot for the NEXT tick's edge detection

    if (cmd.mode_toggled) {
      if (pending_action_goal_) {
        action_client_.cancelGoal();
        pending_action_goal_ = false;
      }
      if (mode_ == Mode::kDirectJog) {
        mode_ = Mode::kSetReference;
        reference_pose_ = fk_solver_->ComputePose(q);
        ghost_joint_positions_ = q;
        reference_reachable_ = true;
        ROS_INFO("teleop_node: mode -> SET_REFERENCE (seeded from current pose)");
      } else {
        mode_ = Mode::kDirectJog;
        jog_target_pose_ = fk_solver_->ComputePose(q);
        PublishReferenceMarker(visualization_msgs::Marker::DELETE);
        ROS_INFO("teleop_node: mode -> DIRECT_JOG");
      }
      PublishMode();
    }

    if (cmd.gripper_active) {
      gripper_position_ = std::min(params_.gripper_max,
                                    std::max(params_.gripper_min,
                                              gripper_position_ + cmd.gripper_rate * (1.0 / rate_hz_)));
      PublishGripper();
    }

    if (!cmd.deadman_held) {
      // No motion without the deadman, in either mode. In DIRECT_JOG, also
      // reset the jog target back to wherever the arm actually is right
      // now and hide its marker: otherwise a target that had drifted
      // while the deadman was held would still be sitting out there,
      // ready to resume being chased the instant the deadman is pressed
      // again with the stick still centered -- exactly the "moves with no
      // input" symptom this reset exists to prevent. The SET_REFERENCE
      // preview, in contrast, stays visible and unchanged: it's meant to
      // persist across a released deadman (see docs/teleop.md).
      if (mode_ == Mode::kDirectJog) {
        jog_target_pose_ = fk_solver_->ComputePose(q);
        PublishJogTargetMarker(visualization_msgs::Marker::DELETE);
      } else {
        PublishReferenceMarker(visualization_msgs::Marker::ADD);
      }
      PublishGhost();
      return;
    }

    const double dt = 1.0 / rate_hz_;

    if (mode_ == Mode::kDirectJog) {
      // Null-space secondary task (vertical-bias) temporarily disabled --
      // reported as "absolutely not working" live and not worth debugging
      // right now; commenting out the two arguments below re-enables it
      // once there's time to look at it properly.
      ResolvedRateResult result = ComputeResolvedRateVelocity(
          *fk_solver_, q, cmd.vx, cmd.vy, cmd.vz, cmd.pitch_rate, velocity_limits_, velocity_damping_
          /* , nullspace_solver_.get(), nullspace_gain_ */);
      if (result.achieved_to_commanded_ratio < saturation_warn_ratio_) {
        ROS_WARN_THROTTLE(1.0,
                           "teleop_node: commanded velocity saturated to %.0f%% (near a joint limit "
                           "or an unreachable task-space direction)",
                           result.achieved_to_commanded_ratio * 100.0);
      }
      sensor_msgs::JointState arm_cmd;
      arm_cmd.header.stamp = ros::Time::now();
      arm_cmd.name = joint_names_;
      arm_cmd.position.resize(q.size());
      for (size_t i = 0; i < q.size(); ++i) {
        double q_next = q[i] + result.joint_velocities[i] * dt;
        arm_cmd.position[i] =
            std::min(joint_limits_[i].second, std::max(joint_limits_[i].first, q_next));
      }
      joint_out_pub_.publish(arm_cmd);
      ghost_joint_positions_ = arm_cmd.position;  // ghost mirrors the real arm: invisible overlap
      PublishGhost();

      // Jog target marker: tracks the same commanded rate the arm is
      // actually being driven by, purely for visibility -- it does NOT
      // feed back into the control law above. Idle (no commanded rate):
      // snap it to the current actual pose rather than letting it sit
      // wherever it last was, so it never quietly represents "somewhere
      // else" while nothing is happening.
      if (cmd.vx != 0.0 || cmd.vy != 0.0 || cmd.vz != 0.0 || cmd.pitch_rate != 0.0) {
        jog_target_pose_.translation() += Eigen::Vector3d(cmd.vx, cmd.vy, cmd.vz) * dt;
        ApplyPitchRotation(jog_target_pose_, cmd.pitch_rate, dt);
      } else {
        jog_target_pose_ = fk_solver_->ComputePose(q);
      }
      PublishJogTargetMarker(visualization_msgs::Marker::ADD);
    } else {
      if (cmd.vx != 0.0 || cmd.vy != 0.0 || cmd.vz != 0.0 || cmd.pitch_rate != 0.0) {
        reference_pose_.translation() += Eigen::Vector3d(cmd.vx, cmd.vy, cmd.vz) * dt;
        ApplyPitchRotation(reference_pose_, cmd.pitch_rate, dt);
        UpdateGhostPreview();  // best-effort; never blocks the reference itself from moving
      }
      PublishGhost();
      PublishReferenceMarker(visualization_msgs::Marker::ADD);
      if (cmd.commit_pressed) {
        CommitReference();
      }
    }
  }

  ros::NodeHandle& nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_sub_;
  ros::Publisher joint_out_pub_;
  ros::Publisher ghost_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher mode_pub_;
  ros::ServiceClient generate_client_;
  ActionClient action_client_;
  ros::Timer timer_;

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<arm_kinematics::InverseKinematicsSolver> ik_solver_;
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> nullspace_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  std::vector<std::pair<double, double>> joint_limits_;

  JoyMapperParams params_;
  double rate_hz_;
  double joy_timeout_;
  double sample_rate_;
  double velocity_damping_;
  double saturation_warn_ratio_;
  double nullspace_gain_;

  sensor_msgs::Joy last_joy_;
  sensor_msgs::Joy prev_tick_joy_;  // empty until the first tick: Button() treats that as "unpressed"
  ros::Time last_joy_stamp_;
  bool have_joy_ = false;

  std::mutex joint_state_mutex_;
  std::vector<double> actual_joint_positions_;
  bool have_actual_state_ = false;

  Mode mode_ = Mode::kDirectJog;
  Eigen::Isometry3d reference_pose_ = Eigen::Isometry3d::Identity();
  bool reference_reachable_ = true;  // informational only -- see UpdateGhostPreview
  std::vector<double> ghost_joint_positions_;
  Eigen::Isometry3d jog_target_pose_ = Eigen::Isometry3d::Identity();
  bool have_jog_target_ = false;
  double gripper_position_ = 0.0;
  bool pending_action_goal_ = false;  // avoids a benign-but-noisy cancelGoal()-with-nothing-running log
};

}  // namespace arm_teleop

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  try {
    arm_teleop::TeleopNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("teleop_node: %s", e.what());
    return 1;
  }
  return 0;
}
