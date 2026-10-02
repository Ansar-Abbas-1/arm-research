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
#include <arm_kinematics/joint_state_utils.h>

#include "arm_trajectory/GenerateTrajectory.h"
#include "arm_trajectory/ReachTargetPose.h"
#include "arm_trajectory/estop_monitor.h"
#include "arm_trajectory/reach_correction.h"

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
    // Originally matched InverseKinematicsSolver's own default
    // tolerances (1e-4 m / 1e-3 rad) exactly, on the theory that the goal
    // IK solve inside /generate_trajectory was already required to hit
    // that precision. Loosened after actually using this interactively
    // (rviz marker drags): sitting right at IK's own convergence
    // threshold meant plenty of genuinely-fine drags -- landing well
    // within a millimeter -- registered as "out of tolerance" anyway,
    // since a marker-dragged orientation is rarely exactly on the
    // reachable manifold and IK converges to a small residual rather than
    // exactly zero. 1mm/0.01rad is loose enough to stop flagging those as
    // failures while still catching a genuinely wrong final pose.
    //
    // orientation_tolerance_ is now checked against PITCH only, not full
    // 3D orientation angle -- /generate_trajectory's IK (PositionPitchIkSolver)
    // never enforces roll/yaw (mechanically irrelevant on this arm; see
    // arm_kinematics/docs/position_pitch_ik.md), so holding the executed
    // result to a full-orientation bar it was never asked to hit would
    // flag genuinely-successful reaches as failures.
    pnh.param("position_tolerance", position_tolerance_, 1e-3);
    pnh.param("orientation_tolerance", orientation_tolerance_, 1e-2);
    // Defensive cap on how long a single execution is allowed to run
    // before this call gives up waiting -- trajectory_executor_node
    // always terminates the action on its own (success/abort/preempt), so
    // this is a backstop against something being badly wrong elsewhere in
    // the graph, not expected to fire in normal operation.
    pnh.param("execution_timeout_margin", execution_timeout_margin_, 10.0);

    // Optional steady-state correction, OFF by default (correction_steps
    // = 0): see reach_correction.h for why it exists and how it behaves.
    pnh.param("correction_steps", correction_.max_steps, 0);
    pnh.param("correction_trigger", correction_.trigger, correction_.trigger);
    pnh.param("correction_max_total", correction_.max_total, correction_.max_total);
    pnh.param("correction_gain", correction_.gain, correction_.gain);
    // Pitch compensation (the position scheme applied to the one orientation
    // component the arm controls): active whenever correction is on, unless
    // switched off here.
    pnh.param("correction_pitch", correction_.pitch, correction_.pitch);
    pnh.param("correction_pitch_trigger", correction_.pitch_trigger, correction_.pitch_trigger);
    pnh.param("correction_pitch_max_total", correction_.pitch_max_total, correction_.pitch_max_total);
    // Wait before measuring when correcting, so the reading is the settled
    // pose. Kept short: robot_interface's watchdog drops torque 0.5 s after
    // the last command, and a correction should start before that.
    pnh.param("correction_settle", correction_settle_, 0.05);

    // The executor is the real choke point; this gives a clear early answer
    // and stops the correction loop. Fail-closed like the executor.
    bool require_estop_state = true;
    pnh.param("require_estop_state", require_estop_state, true);
    estop_ = std::make_unique<arm_trajectory::EStopMonitor>(nh_, require_estop_state);

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
    ROS_INFO_STREAM("reach_target_pose_node: ready, " << root_link << " -> " << tip_link
                    << (correction_.max_steps > 0
                            ? ", steady-state correction ON (" + std::to_string(correction_.max_steps) + " steps)"
                            : ""));
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> positions;
    if (!arm_kinematics::ExtractJointPositions(*msg, joint_names_, &positions)) {
      return;  // wait for a message that names every actuated joint
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

  // Generates a trajectory to `pose` from the LIVE joint state and runs it
  // to completion. On any failure fills *error and returns false.
  bool GenerateAndExecute(const geometry_msgs::Pose& pose, bool cartesian_path, double sample_rate,
                          std::string* error) {
    arm_trajectory::GenerateTrajectory generate;
    generate.request.goal_pose = pose;
    generate.request.cartesian_path = cartesian_path;
    generate.request.sample_rate = sample_rate;

    if (!generate_client_.call(generate)) {
      *error = "reach_target_pose: /generate_trajectory call failed (service unavailable?)";
      return false;
    }
    if (!generate.response.success) {
      *error = "reach_target_pose: trajectory generation failed: " + generate.response.message;
      return false;
    }
    if (generate.response.trajectory.points.empty()) {
      *error = "reach_target_pose: generated trajectory has no points";
      return false;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = generate.response.trajectory;
    goal.trajectory.header.stamp = ros::Time::now();
    if (!last_exec_end_.isZero() && correction_.max_steps > 0) {
      // robot_interface drops torque 0.5 s after the last command; a
      // correction that starts later begins from a sagged pose.
      ROS_INFO("reach_target_pose: %.3f s since the previous trajectory ended (torque watchdog 0.5 s)",
               (ros::Time::now() - last_exec_end_).toSec());
    }
    action_client_.sendGoal(goal);

    double planned_duration = goal.trajectory.points.back().time_from_start.toSec();
    bool finished = action_client_.waitForResult(
        ros::Duration(planned_duration + execution_timeout_margin_));
    if (!finished) {
      action_client_.cancelGoal();
      *error = "reach_target_pose: execution did not finish within " +
               std::to_string(planned_duration + execution_timeout_margin_) + "s, cancelled";
      return false;
    }
    last_exec_end_ = ros::Time::now();
    if (action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
      *error = "reach_target_pose: execution did not succeed: " + action_client_.getState().toString() + " (" +
               action_client_.getState().getText() + ")";
      return false;
    }
    return true;
  }

  struct Measured {
    Eigen::Isometry3d pose;
    double position_error;
    double orientation_error;  // |pitch error|
    double pitch_error;        // signed, see arm_trajectory::PitchError
  };

  // FK of the latest joint state vs `target_pose`. Orientation error is the
  // PITCH-axis-projected rotation error only, same convention as
  // PositionPitchIkSolver's own e_pitch (roll/yaw are free, never checked).
  bool Measure(const Eigen::Isometry3d& target_pose, Measured* out) {
    std::vector<double> final_q;
    if (!LatestJointPositions(&final_q)) {
      return false;
    }
    out->pose = fk_solver_->ComputePose(final_q);
    out->position_error = (out->pose.translation() - target_pose.translation()).norm();
    out->pitch_error = arm_trajectory::PitchError(target_pose.rotation(), out->pose.rotation());
    out->orientation_error = std::fabs(out->pitch_error);
    return true;
  }

  bool ReachCb(arm_trajectory::ReachTargetPose::Request& req,
               arm_trajectory::ReachTargetPose::Response& res) {
    estop_->WaitForState(1.0);
    if (!estop_->AllowsMotion()) {
      res.success = false;
      res.message = "reach_target_pose: not moving: " + estop_->Reason();
      return true;
    }

    std::string error;
    if (!GenerateAndExecute(req.target_pose, req.cartesian_path, req.sample_rate, &error)) {
      res.success = false;
      res.message = error;
      return true;
    }

    Eigen::Isometry3d target_pose;
    tf2::fromMsg(req.target_pose, target_pose);

    if (correction_.max_steps > 0) {
      ros::Duration(correction_settle_).sleep();
    }
    Measured measured;
    if (!Measure(target_pose, &measured)) {
      res.success = false;
      res.message = "reach_target_pose: execution finished but no /joint_states_in received yet, "
                     "cannot verify final pose";
      return true;
    }

    // Optional steady-state correction (off unless ~correction_steps > 0).
    // Always verified against the ORIGINAL target, never the compensated
    // aim point. Any failure just stops correcting; what is reported is
    // always the arm's real, re-measured final state.
    int steps_done = 0;
    Eigen::Vector3d compensation = Eigen::Vector3d::Zero();
    double pitch_compensation = 0.0;
    std::string correction_note;
    while (true) {
      if (estop_->engaged()) {
        correction_note = " (correction stopped: e-stop engaged)";
        break;
      }
      arm_trajectory::CorrectionDecision d = arm_trajectory::NextCorrection(
          target_pose.translation(), measured.pose.translation(), compensation, correction_, steps_done);
      arm_trajectory::PitchDecision dp = arm_trajectory::NextPitchCorrection(
          measured.pitch_error, pitch_compensation, correction_, steps_done);
      if (!d.apply && !dp.apply) {
        if (correction_.max_steps > 0) {
          correction_note = " (correction stopped: " + d.reason + "; " + dp.reason + ")";
        }
        break;
      }
      // Each component that does not need a step keeps its current compensation.
      const Eigen::Vector3d next_compensation = d.apply ? d.compensation : compensation;
      const double next_pitch_compensation = dp.apply ? dp.compensation : pitch_compensation;
      geometry_msgs::Pose aim =
          tf2::toMsg(arm_trajectory::AimPose(target_pose, next_compensation, next_pitch_compensation));
      ROS_INFO("reach_target_pose: correction %d/%d, error %.4f m / pitch %+.4f rad, total compensation "
               "(%.4f, %.4f, %.4f) m / pitch %+.4f rad",
               steps_done + 1, correction_.max_steps, measured.position_error, measured.pitch_error,
               next_compensation.x(), next_compensation.y(), next_compensation.z(), next_pitch_compensation);
      bool executed = GenerateAndExecute(aim, req.cartesian_path, req.sample_rate, &error);
      ros::Duration(correction_settle_).sleep();
      if (executed) {
        ++steps_done;
        compensation = next_compensation;
        pitch_compensation = next_pitch_compensation;
      } else {
        correction_note = " (correction step failed and stopped: " + error + ")";
      }
      Measured again;
      if (!Measure(target_pose, &again)) {
        break;
      }
      measured = again;
      if (!executed) {
        break;
      }
    }
    if (steps_done > 0) {
      correction_note = " after " + std::to_string(steps_done) + " correction step(s)" + correction_note;
    }

    res.final_position_error = measured.position_error;
    res.final_orientation_error = measured.orientation_error;

    const bool correcting = correction_.max_steps > 0;
    const double position_tolerance =
        arm_trajectory::ReportTolerance(position_tolerance_, correction_.trigger, correcting);
    const double orientation_tolerance = arm_trajectory::ReportTolerance(
        orientation_tolerance_, correction_.pitch_trigger, correcting && correction_.pitch);
    if (measured.position_error > position_tolerance || measured.orientation_error > orientation_tolerance) {
      res.success = false;
      res.message = "reach_target_pose: execution succeeded but final pose is out of tolerance "
                     "(position error " +
                     std::to_string(measured.position_error) + " m, orientation error " +
                     std::to_string(measured.orientation_error) + " rad)" + correction_note;
      return true;
    }

    res.success = true;
    res.message = "reached target pose" + correction_note;
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
  std::unique_ptr<arm_trajectory::EStopMonitor> estop_;
  arm_trajectory::ReachCorrectionOptions correction_;
  double correction_settle_ = 0.05;
  ros::Time last_exec_end_;  // when the previous trajectory finished (zero = none yet)

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
