#include <algorithm>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/inverse_kinematics.h>
#include <arm_trajectory/joint_space_trajectory.h>

#include "arm_teleop/joy_target_mapper.h"

namespace arm_teleop {

// Restarts DISABLED at the InverseKinematicsSolver level (max_restarts=0):
// its restart mechanism draws a replacement seed *uniformly across the
// entire joint range* (arm_kinematics/inverse_kinematics.cpp), which is
// exactly wrong for a per-tick, continuity-dependent jog -- found
// empirically during self-test, not assumed. Two things were tried and
// rejected before this:
//
//   - The service's own default options (restarts enabled, ~20 budget):
//     a tiny ~1cm jog step could trigger a restart landing in a
//     *different elbow configuration*. Confirmed via debug logging: every
//     tick produced a 150-200 point trajectory (~3-4s) for what should be
//     a near-instant nudge, constantly preempted by the next tick's own
//     barely-progressed trajectory -- the arm never really moved despite
//     the internal target advancing correctly.
//   - A small fixed restart budget (3): still uses the same
//     entire-joint-range random draw, just less often. Confirmed via the
//     rostest below: one restart landed in a configuration whose
//     intermediate joint-space path swung the *measured* Cartesian
//     position in the wrong direction mid-flight (Finding 2/3-style: a
//     joint-space interpolation between very different configurations
//     doesn't trace a straight Cartesian line) -- an operator jogging
//     with a joystick would see the arm briefly move opposite to command.
//     Not acceptable at any restart frequency, however rare.
//
// SolveWithLocalRetries() below replaces Solve()'s global random restart
// with a small, BOUNDED-DISTANCE local perturbation retry -- every
// attempted seed stays within kLocalRetryNudge of the current continuous
// configuration, so a recovered tick still can't produce a large
// joint-space jump, by construction.
arm_kinematics::IkOptions JogIkOptions() {
  arm_kinematics::IkOptions opts;
  opts.max_restarts = 0;
  // Loosened from the 1e-4 m default: still well below what a joystick jog
  // needs to feel precise, and a single fresh Solve() call re-adapting its
  // damping from scratch each tick lands consistently in the 1-2mm range
  // even when it's genuinely converging, not stuck.
  opts.position_tolerance = 1e-3;
  opts.max_iterations = 300;
  opts.max_inner_retries = 30;
  return opts;
}

// Local, bounded-distance recovery for a tick whose primary warm-started
// IK attempt didn't converge -- e.g. arm_trajectory's Finding 1
// ({0,0,0,0,0} is a genuine kinematic singularity, the arm's home
// configuration) or another locally ill-conditioned point reached mid-jog.
// Unlike InverseKinematicsSolver's own restarts, every retry seed here is
// `seed` perturbed by at most kLocalRetryNudge per joint (clamped to
// limits) -- never a fresh draw across the whole joint range -- so a
// recovered tick still can't jump to a distant elbow configuration.
// Nudge magnitude and attempt count are heuristic, not derived; if every
// attempt still fails to converge, the caller drops the tick (state stays
// exactly as it was, retried next tick with fresh joystick input).
constexpr double kLocalRetryNudge = 0.1;  // rad
constexpr int kLocalRetryAttempts = 5;

std::vector<double> ClampToLimits(std::vector<double> q,
                                   const std::vector<std::pair<double, double>>& limits) {
  for (size_t i = 0; i < q.size(); ++i) {
    q[i] = std::min(limits[i].second, std::max(limits[i].first, q[i]));
  }
  return q;
}

// arm_trajectory's Finding 1 (docs/trajectory_generation.md): {0,0,0,0,0}
// is a genuine kinematic singularity (Jacobian SVD smallest singular value
// ~1e-33) -- the arm's default/home configuration. Threshold and nudge are
// heuristic, not derived; see the JointStatesCb call site for why a
// one-time startup nudge is needed in addition to SolveWithLocalRetries.
constexpr double kSingularityZeroEpsilon = 0.01;  // rad
constexpr double kSingularityEscapeNudge = 0.05;  // rad

bool NearZeroConfiguration(const std::vector<double>& q) {
  for (double v : q) {
    if (std::abs(v) > kSingularityZeroEpsilon) {
      return false;
    }
  }
  return true;
}

arm_kinematics::IkResult SolveWithLocalRetries(
    const arm_kinematics::InverseKinematicsSolver& ik_solver, const Eigen::Isometry3d& target,
    const std::vector<double>& seed, const std::vector<std::pair<double, double>>& limits,
    std::mt19937& rng) {
  arm_kinematics::IkResult best = ik_solver.Solve(target, seed);
  if (best.converged) {
    return best;
  }

  // Directed attempt first: per Finding 1, the known singularity has
  // multiple joints sitting exactly at one of their own limit boundaries
  // at once, so a *random* jitter only has good odds of nudging every
  // affected joint the correct (away-from-boundary) direction by chance.
  // Move each joint toward the middle of its own range instead --
  // guaranteed away from whichever boundary it's actually at, if any.
  std::vector<double> directed = seed;
  for (size_t i = 0; i < directed.size(); ++i) {
    double mid = 0.5 * (limits[i].first + limits[i].second);
    double step = (seed[i] < mid) ? kLocalRetryNudge : -kLocalRetryNudge;
    directed[i] = seed[i] + step;
  }
  arm_kinematics::IkResult directed_result = ik_solver.Solve(target, ClampToLimits(directed, limits));
  if (directed_result.converged) {
    return directed_result;
  }
  if (directed_result.residual.norm() < best.residual.norm()) {
    best = directed_result;
  }

  // Fall back to random local jitter for anything the directed nudge alone
  // doesn't resolve (e.g. a merely-hard-to-converge point, not a boundary
  // singularity).
  std::uniform_real_distribution<double> jitter(-kLocalRetryNudge, kLocalRetryNudge);
  for (int attempt = 0; attempt < kLocalRetryAttempts; ++attempt) {
    std::vector<double> nudged = seed;
    for (size_t i = 0; i < nudged.size(); ++i) {
      nudged[i] += jitter(rng);
    }
    arm_kinematics::IkResult candidate = ik_solver.Solve(target, ClampToLimits(nudged, limits));
    if (candidate.converged) {
      return candidate;
    }
    if (candidate.residual.norm() < best.residual.norm()) {
      best = candidate;
    }
  }
  return best;
}

// Incremental Cartesian jogging from a joystick. Does its own FK/IK
// in-process (like arm_trajectory's cartesian_trajectory.cpp), rather than
// going through /generate_trajectory, specifically to keep control over
// IK's warm-start/restart behavior above -- the service's own default
// options are wrong for this per-tick, continuity-dependent use case.
//
// Each timer tick maps the latest /joy message to a position delta
// (ComputeJoyTargetUpdate, pure/unit-tested separately), adds it to an
// internally-owned target pose held at a FIXED orientation (whatever the
// arm's orientation was at startup -- see docs/teleop.md for why full
// 6-DOF jogging isn't attempted here), solves IK warm-started from the
// previous tick's joint solution, and -- only if that converges -- turns
// the result into a quintic-timed joint-space trajectory
// (GenerateJointSpaceTrajectory, the same math arm_trajectory uses) sent
// to follow_joint_trajectory. A new goal there preempts whatever's still
// executing (standard SimpleActionServer/SimpleActionClient behavior),
// which is the actual point of jogging responsively.
//
// Gripper (joint_6): published directly to /joint_states_out, independent
// of the pose-jogging path -- confirmed safe against
// arm_sim_validator_node's commandedCb, which updates only the joint
// indices named in each incoming message and leaves the rest untouched.
// joint_6 is a fixed URDF joint (see arm_interface/urdf/arm.urdf) wired to
// physical motor 6 (the gripper) outside the pose kinematic chain, so it
// has no real URDF <limit> to derive gripper_min/max/rate from -- those
// are heuristic defaults (see joy_target_mapper.h), flagged for
// recalibration once real gripper hardware/limits are known (M5).
class TeleopNode {
 public:
  using ActionClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  TeleopNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), action_client_("follow_joint_trajectory", true) {
    std::string root_link, tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    pnh.param("rate", rate_hz_, 10.0);
    pnh.param("joy_timeout", joy_timeout_, 0.5);
    pnh.param("sample_rate", sample_rate_, 50.0);

    pnh.param("deadman_button", params_.deadman_button, params_.deadman_button);
    pnh.param("axis_x", params_.axis_x, params_.axis_x);
    pnh.param("axis_y", params_.axis_y, params_.axis_y);
    pnh.param("axis_z", params_.axis_z, params_.axis_z);
    pnh.param("axis_deadzone", params_.axis_deadzone, params_.axis_deadzone);
    pnh.param("linear_gain", params_.linear_gain, params_.linear_gain);
    pnh.param("gripper_open_button", params_.gripper_open_button, params_.gripper_open_button);
    pnh.param("gripper_close_button", params_.gripper_close_button, params_.gripper_close_button);
    pnh.param("gripper_rate", params_.gripper_rate, params_.gripper_rate);
    pnh.param("gripper_min", params_.gripper_min, params_.gripper_min);
    pnh.param("gripper_max", params_.gripper_max, params_.gripper_max);

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("teleop_node: failed to load URDF from 'robot_description' param");
    }
    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    ik_solver_ = std::make_unique<arm_kinematics::InverseKinematicsSolver>(*fk_solver_, JogIkOptions());
    joint_names_ = fk_solver_->GetActuatedJointNames();
    velocity_limits_ = fk_solver_->GetActuatedJointVelocityLimits();
    joint_limits_ = fk_solver_->GetActuatedJointLimits();

    joy_sub_ = nh_.subscribe("/joy", 10, &TeleopNode::JoyCb, this);
    joint_sub_ = nh_.subscribe("/joint_states_in", 10, &TeleopNode::JointStatesCb, this);
    gripper_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_out", 10);

    ROS_INFO("teleop_node: waiting for follow_joint_trajectory ...");
    action_client_.waitForServer();
    ROS_INFO("teleop_node: ready at %.1f Hz (deadman=button %d)", rate_hz_, params_.deadman_button);

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz_), &TeleopNode::TimerCb, this);
  }

 private:
  void JoyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    last_joy_ = *msg;
    last_joy_stamp_ = ros::Time::now();
    have_joy_ = true;
  }

  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    if (have_seed_) {
      return;  // joint state is owned by jogging from here on, not re-synced
    }
    std::vector<double> positions(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        return;  // wait for a message that names every actuated joint
      }
      positions[i] = msg->position[std::distance(msg->name.begin(), it)];
    }

    if (NearZeroConfiguration(positions)) {
      // Finding 1's singularity is a hard kinematic restriction, not a
      // numerical-conditioning wobble -- SolveWithLocalRetries's per-tick
      // recovery (below) confirmed empirically insufficient to escape it
      // (every directed/random local retry plateaued at the same ~1-2mm
      // residual, iterations pinned at the max_iterations budget every
      // time). A one-time, larger, fixed nudge before jogging even starts
      // is what actually clears it.
      for (auto& v : positions) {
        v += kSingularityEscapeNudge;
      }
      positions = ClampToLimits(positions, joint_limits_);
      ROS_WARN(
          "teleop_node: initial joint state is at/near the all-zero kinematic singularity "
          "(arm_trajectory Finding 1) -- nudging internal seed by %.3f rad/joint before jogging",
          kSingularityEscapeNudge);
    }

    current_joint_positions_ = positions;
    try {
      target_pose_ = fk_solver_->ComputePose(current_joint_positions_);
    } catch (const std::exception& e) {
      ROS_ERROR("teleop_node: initial ComputePose failed: %s", e.what());
      return;
    }
    have_seed_ = true;
    ROS_INFO("teleop_node: initial joint seed + target pose set from /joint_states_in");
  }

  void TimerCb(const ros::TimerEvent&) {
    if (!have_seed_) {
      ROS_WARN_THROTTLE(2.0, "teleop_node: no /joint_states_in received yet, idle");
      return;
    }
    if (!have_joy_ || (ros::Time::now() - last_joy_stamp_).toSec() > joy_timeout_) {
      return;  // no joystick input, or it went stale (disconnected/topic died): stay put
    }

    JoyTargetUpdate update = ComputeJoyTargetUpdate(last_joy_, params_, 1.0 / rate_hz_);

    if (update.gripper_changed) {
      gripper_position_ = std::min(params_.gripper_max,
                                    std::max(params_.gripper_min, gripper_position_ + update.gripper_delta));
      sensor_msgs::JointState gripper_cmd;
      gripper_cmd.header.stamp = ros::Time::now();
      gripper_cmd.name = {"joint_6"};
      gripper_cmd.position = {gripper_position_};
      gripper_pub_.publish(gripper_cmd);
    }

    if (!update.moving) {
      return;
    }

    Eigen::Isometry3d candidate = target_pose_;
    candidate.translation() += Eigen::Vector3d(update.dx, update.dy, update.dz);

    arm_kinematics::IkResult ik =
        SolveWithLocalRetries(*ik_solver_, candidate, current_joint_positions_, joint_limits_, rng_);
    if (!ik.converged) {
      // Candidate outside reachable space (or too hard to converge even
      // with local retries): drop this increment rather than committing
      // to an unreachable target and getting stuck repeatedly failing
      // every subsequent tick.
      ROS_WARN_THROTTLE(1.0,
                         "teleop_node: jog target unreachable from current seed (residual %.4f m / "
                         "%.4f rad), ignoring",
                         ik.residual.head<3>().norm(), ik.residual.tail<3>().norm());
      return;
    }

    trajectory_msgs::JointTrajectory traj;
    try {
      traj = arm_trajectory::GenerateJointSpaceTrajectory(current_joint_positions_, ik.joint_positions,
                                                           velocity_limits_, joint_names_, sample_rate_);
    } catch (const std::exception& e) {
      ROS_ERROR_THROTTLE(1.0, "teleop_node: GenerateJointSpaceTrajectory failed: %s", e.what());
      return;
    }

    current_joint_positions_ = ik.joint_positions;  // commit warm-start seed
    target_pose_ = candidate;                       // commit Cartesian target
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    goal.trajectory.header.stamp = ros::Time::now();
    action_client_.sendGoal(goal);  // preempts whatever's still executing
  }

  ros::NodeHandle& nh_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_sub_;
  ros::Publisher gripper_pub_;
  ActionClient action_client_;
  ros::Timer timer_;

  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> fk_solver_;
  std::unique_ptr<arm_kinematics::InverseKinematicsSolver> ik_solver_;
  std::vector<std::string> joint_names_;
  std::vector<double> velocity_limits_;
  std::vector<std::pair<double, double>> joint_limits_;
  std::mt19937 rng_{std::random_device{}()};

  JoyMapperParams params_;
  double rate_hz_;
  double joy_timeout_;
  double sample_rate_;

  sensor_msgs::Joy last_joy_;
  ros::Time last_joy_stamp_;
  bool have_joy_ = false;

  bool have_seed_ = false;
  std::vector<double> current_joint_positions_;
  Eigen::Isometry3d target_pose_;
  double gripper_position_ = 0.0;
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
