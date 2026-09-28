#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <trajectory_msgs/JointTrajectory.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/inverse_kinematics.h>

namespace arm_trajectory {

struct CartesianTrajectoryResult {
  bool success;
  std::string message;  // failure detail if !success (only possible failure: the goal itself
                         // isn't reachable -- interior path samples never fail, see below)
  trajectory_msgs::JointTrajectory trajectory;
  // Worst-case interior-sample tracking residual (best-effort path
  // tracking -- see GenerateCartesianTrajectory). 0 if success is false,
  // or if the path required no interior samples (already at goal).
  double peak_position_deviation = 0.0;     // meters
  double peak_orientation_deviation = 0.0;  // radians
};

// Generates a joint-space-sampled trajectory tracing the interpolated
// Cartesian path (position lerp + orientation SLERP, quintic-timed) from
// FK(q_start) to `pose_goal`. Each sample is warm-started from the
// *previous* sample's joint solution -- not optional on this redundant
// chain, see docs/trajectory_generation.md -- to avoid jumping between
// elbow configurations sample-to-sample.
//
// `ik_solver` (caller-provided, default/strict tolerances) validates only
// the GOAL: the actual target the caller asked for must be genuinely
// reachable, and this is the only way this function fails (`message`
// explains why if so). INTERIOR samples are **best-effort**: this chain
// has 5 DOF over a 6D task space (M2 finding), so linear+SLERP
// interpolation between two exactly-reachable poses does not generally
// stay on the reachable 5D submanifold, and the deviation grows (with no
// fixed bound) the further into the path you go -- demanding exact
// tracking there isn't something this arm can always deliver, so an
// interior sample takes IK's closest achievable solution whatever its
// residual, rather than failing the whole trajectory over it. The worst
// such residual is reported in `peak_position_deviation`/
// `peak_orientation_deviation` so a caller can judge whether the path was
// actually a good fit. See docs/trajectory_generation.md for how this was
// found (every restart converging to the same, path-position-dependent
// residual regardless of seed -- a genuine best-fit minimum on a path
// that leaves the reachable manifold, not a stuck solver) and
// PathTrackingIkOptions() in the .cpp for the tolerance used to decide
// when a sample is "good enough" to stop restarting on.
//
// Duration is seeded from the joint-space endpoint difference (a
// heuristic -- the actual induced joint-space path isn't linear, see
// docs/trajectory_generation.md), then searched via coarse-to-fine
// duration-rescaling (up to `max_duration_iterations` cheap, fixed-
// sample-count search passes, followed by a small bounded number of
// full-sample_rate_hz-resolution correction passes) until every sample's
// finite-differenced joint velocity is within its limit -- see the .cpp
// for why this replaced re-sampling the whole path at full resolution on
// every iteration (M3 Finding 4: that was the dominant cost for a large
// Cartesian move).
//
// Unlike GenerateJointSpaceTrajectory, per-joint velocities in the
// returned trajectory are finite-differenced approximations (there's no
// closed form for them along a Cartesian-interpolated path), and
// accelerations are left empty -- not meaningful to approximate the same
// way, and not needed by this milestone's executor.
CartesianTrajectoryResult GenerateCartesianTrajectory(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const arm_kinematics::InverseKinematicsSolver& ik_solver, const std::vector<double>& q_start,
    const Eigen::Isometry3d& pose_goal, const std::vector<double>& velocity_limits,
    const std::vector<std::string>& joint_names, double sample_rate_hz,
    int max_duration_iterations = 8);

}  // namespace arm_trajectory
