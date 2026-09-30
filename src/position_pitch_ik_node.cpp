#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_eigen/tf2_eigen.h>
#include <urdf/model.h>

#include "arm_kinematics/SolvePositionPitchIk.h"
#include "arm_kinematics/forward_kinematics.h"
#include "arm_kinematics/joint_state_utils.h"
#include "arm_kinematics/position_pitch_ik.h"

// ROS wrapper around PositionPitchIkSolver -- same shape as ik_node (which
// wraps InverseKinematicsSolver): advertises /solve_position_pitch_ik, and
// tracks /joint_states_in as the default seed when a request's
// seed_joint_positions is empty. Every PositionPitchIkOptions field is a
// ~param, so all of it (including the null-space secondary task) is
// reconfigurable via a launch file <param>/<rosparam> block or a plain
// YAML loaded with <rosparam command="load">, the same as any other node
// in this workspace -- see config/position_pitch_ik.yaml for a worked
// example and docs/position_pitch_ik.md for what each option means.
class PositionPitchIkNode {
 public:
  PositionPitchIkNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string root_link, tip_link, nullspace_tip_link;
    pnh.param<std::string>("root_link", root_link, "base_link");
    pnh.param<std::string>("tip_link", tip_link, "end_effector");
    pnh.param<std::string>("nullspace_tip_link", nullspace_tip_link, "link_4");

    arm_kinematics::PositionPitchIkOptions options;
    pnh.param("position_tolerance", options.position_tolerance, options.position_tolerance);
    pnh.param("pitch_tolerance", options.pitch_tolerance, options.pitch_tolerance);
    pnh.param("max_iterations", options.max_iterations, options.max_iterations);
    pnh.param("damping", options.damping, options.damping);
    pnh.param("max_step", options.max_step, options.max_step);
    pnh.param("max_inner_retries", options.max_inner_retries, options.max_inner_retries);
    pnh.param("max_restarts", options.max_restarts, options.max_restarts);
    pnh.param("nullspace_gain", options.nullspace_gain, options.nullspace_gain);
    pnh.param("nullspace_tolerance", options.nullspace_tolerance, options.nullspace_tolerance);
    // rng_seed deliberately NOT exposed as a param -- matches ik_node,
    // which doesn't expose InverseKinematicsSolver's either: leave
    // restarts genuinely randomized for normal use, construct the solver
    // directly in a test for reproducibility instead.

    urdf::Model model;
    if (!model.initParam("robot_description")) {
      throw std::runtime_error("position_pitch_ik_node: failed to load URDF from 'robot_description'");
    }

    fk_solver_ = std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, tip_link);
    joint_names_ = fk_solver_->GetActuatedJointNames();
    latest_joint_positions_.assign(joint_names_.size(), 0.0);
    have_joint_state_ = false;

    // Always constructed, regardless of ~nullspace_gain: the library
    // itself already treats gain=0 (the default) as "secondary task off"
    // exactly, whether or not a solver was passed (see
    // PositionPitchIkTest.ZeroGainMatchesNoSecondaryTaskAtAll) -- no need
    // to duplicate that on/off logic here too.
    nullspace_solver_ =
        std::make_unique<arm_kinematics::ForwardKinematicsSolver>(model, root_link, nullspace_tip_link);
    options.nullspace_secondary_link_solver = nullspace_solver_.get();

    ik_solver_ = std::make_unique<arm_kinematics::PositionPitchIkSolver>(*fk_solver_, options);

    joint_sub_ = nh.subscribe("/joint_states_in", 10, &PositionPitchIkNode::JointStatesCb, this);
    solve_srv_ =
        nh.advertiseService("/solve_position_pitch_ik", &PositionPitchIkNode::SolveCb, this);
    ROS_INFO_STREAM("position_pitch_ik_node: ready, solving " << root_link << " -> " << tip_link
                     << " (position+pitch only) over " << joint_names_.size() << " actuated joints"
                     << (options.nullspace_gain != 0.0
                             ? ", null-space bias toward vertical " + nullspace_tip_link
                             : ""));
  }

 private:
  void JointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    std::vector<double> positions;
    if (!arm_kinematics::ExtractJointPositions(*msg, joint_names_, &positions)) {
      return;  // incomplete message; keep the previous cached state
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_positions_ = positions;
    have_joint_state_ = true;
  }

  bool SolveCb(arm_kinematics::SolvePositionPitchIk::Request& req,
               arm_kinematics::SolvePositionPitchIk::Response& res) {
    std::vector<double> seed;
    if (!req.seed_joint_positions.empty()) {
      seed.assign(req.seed_joint_positions.begin(), req.seed_joint_positions.end());
    } else {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_joint_state_) {
        ROS_WARN("position_pitch_ik_node: no /joint_states_in received yet, seeding from all-zero");
      }
      seed = latest_joint_positions_;
    }

    Eigen::Isometry3d target;
    tf2::fromMsg(req.target_pose, target);

    arm_kinematics::PositionPitchIkResult result;
    try {
      result = ik_solver_->Solve(target, seed);
    } catch (const std::exception& e) {
      ROS_ERROR("position_pitch_ik_node: Solve failed: %s", e.what());
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
  std::unique_ptr<arm_kinematics::ForwardKinematicsSolver> nullspace_solver_;
  std::unique_ptr<arm_kinematics::PositionPitchIkSolver> ik_solver_;
  std::vector<std::string> joint_names_;

  ros::Subscriber joint_sub_;
  ros::ServiceServer solve_srv_;

  std::mutex joint_state_mutex_;
  std::vector<double> latest_joint_positions_;
  bool have_joint_state_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "position_pitch_ik_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    PositionPitchIkNode node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("position_pitch_ik_node: %s", e.what());
    return 1;
  }
  return 0;
}
