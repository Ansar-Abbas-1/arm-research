#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <trajectory_msgs/JointTrajectory.h>

#include <arm_kinematics/forward_kinematics.h>
#include <arm_kinematics/position_pitch_ik.h>

namespace arm_trajectory {

struct CartesianTrajectoryResult {
  bool success;
  std::string message;  // failure detail if !success -- the goal not being reachable, OR (if
                         // ik_options.collision_obstacle is configured) the goal or any interior
                         // path sample colliding with it, see below
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
// `ik_options` (caller-provided) configures the GOAL solve: position+pitch
// tolerances, damping, and -- since PositionPitchIkOptions carries it --
// the optional null-space secondary task (nullspace_secondary_link_solver/
// nullspace_gain) and the optional base-collision check
// (collision_obstacle/collision_capsule_radius/
// collision_first_checked_segment_index, see base_collision.h), which this
// function's own interior-sample solver also inherits (see
// PathTrackingIkOptions() in the .cpp), so both apply smoothly across the
// whole trajectory, not just the endpoint. UNLIKE the null-space bias,
// though, collision is NOT best-effort at interior samples: a colliding
// goal or interior sample is a hard failure of trajectory generation (see
// `message`), distinct from best-effort tracking deviation -- interior
// samples are solved with max_restarts=0 (see PathTrackingIkOptions() for
// why that's load-bearing), so there is no restart search available there
// to route around an obstacle the way the goal's own Solve() call can;
// retiming the duration doesn't fix a geometrically-colliding interpolated
// path either, so a colliding interior sample fails generation immediately
// rather than continuing to retry at a different duration. The
// actual target the caller asked for must be genuinely reachable at
// ik_options' own tolerances, and this is the only way this function fails
// (`message` explains why if so). INTERIOR samples are **best-effort**:
// this chain has 5 DOF over the 4D position+pitch task (still redundant,
// M2/position_pitch_ik.md finding), so linear+SLERP interpolation between
// two exactly-reachable poses does not generally stay on the reachable
// submanifold, and the deviation grows (with no fixed bound) the further
// into the path you go -- demanding exact tracking there isn't something
// this arm can always deliver, so an interior sample takes IK's closest
// achievable solution whatever its residual, rather than failing the whole
// trajectory over it. The worst such residual is reported in
// `peak_position_deviation`/`peak_orientation_deviation` so a caller can
// judge whether the path was actually a good fit. See
// docs/trajectory_generation.md for how this was found (every restart
// converging to the same, path-position-dependent residual regardless of
// seed -- a genuine best-fit minimum on a path that leaves the reachable
// manifold, not a stuck solver) and PathTrackingIkOptions() in the .cpp for
// the tolerance used to decide when a sample is "good enough" to stop
// restarting on.
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
// `acceleration_limits` [rad/s^2], optional (empty = no bound): the
// duration is also kept at or above MinDurationForAccelerationLimit applied
// to each joint's TRAVEL along the sampled path (sum of |dq| between samples;
// equals |q_goal - q_start| for a monotone joint path). This is a
// conservative approximation of the quintic profile's peak acceleration, used
// instead of second-differencing the IK samples, which amplifies their noise
// by 1/dt^2. It ignores the curvature term of a bent joint path.
//
// Unlike GenerateJointSpaceTrajectory, per-joint velocities in the
// returned trajectory are finite-differenced approximations (there's no
// closed form for them along a Cartesian-interpolated path), and
// accelerations are left empty -- not meaningful to approximate the same
// way, and not needed by this milestone's executor.
CartesianTrajectoryResult GenerateCartesianTrajectory(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const arm_kinematics::PositionPitchIkOptions& ik_options, const std::vector<double>& q_start,
    const Eigen::Isometry3d& pose_goal, const std::vector<double>& velocity_limits,
    const std::vector<std::string>& joint_names, double sample_rate_hz,
    int max_duration_iterations = 8, const std::vector<double>& acceleration_limits = {});

}  // namespace arm_trajectory
