#include "arm_trajectory/cartesian_trajectory.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <ros/duration.h>

#include "arm_trajectory/quintic_time_scaling.h"

namespace arm_trajectory {

namespace {

Eigen::Isometry3d InterpolatePose(const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal,
                                   double fraction) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = start.translation() + fraction * (goal.translation() - start.translation());
  Eigen::Quaterniond q_start(start.rotation());
  Eigen::Quaterniond q_goal(goal.rotation());
  pose.linear() = q_start.slerp(fraction, q_goal).toRotationMatrix();
  return pose;
}

// IK options for INTERIOR path samples: looser than the goal-check
// tolerance, and (unlike the goal check) never treated as a hard failure
// -- see the "best-effort tracking" note on GenerateCartesianTrajectory.
// The looser tolerance just avoids grinding on a residual that isn't
// going to shrink further: this chain has 5 DOF over the 4D position+pitch
// task (still redundant), so linear-position+SLERP-orientation
// interpolation between two exactly-reachable poses does NOT generally
// stay on the reachable submanifold. Confirmed empirically: every restart
// from every seed converged to the *same* small residual for a struggling
// interior sample, and the residual grows with distance into the
// interpolation -- the signature of a genuine best-fit minimum on a path
// that leaves the reachable manifold, not a stuck solver (a bug would vary
// by seed/restart and wouldn't grow smoothly with path position). See
// docs/trajectory_generation.md.
//
// Copies the null-space secondary-task fields (nullspace_secondary_link_solver
// /nullspace_gain/nullspace_tolerance/damping/max_step) from `base` unchanged
// -- so a configured bias applies at every interior sample too, riding
// along "for free" the same way it does within a single Solve() call (the
// primary task's own contribution is near-zero once warm-started close to
// each sample's target) -- while overriding the primary-task convergence
// knobs for interior-sample use:
//
// max_restarts = 0 is deliberate and load-bearing, not just "skip the
// extra work": Solve()'s restart mechanism explicitly abandons the
// warm-started attempt and tries seeds from anywhere in joint space when
// that attempt doesn't hit the target tolerance -- exactly what we do NOT
// want here, since a random restart can land in a different elbow
// configuration with marginally lower residual and get accepted as
// "best", destroying warm-start continuity between samples. That
// discontinuity isn't just an aesthetic problem: it produces a large
// finite-differenced joint velocity between consecutive samples, which
// then feeds the velocity-based duration-scaling loop in
// GenerateCartesianTrajectory and can blow up the requested duration (and
// hence sample count) without bound. Found by hanging a test for 15+
// minutes generating an exploding number of samples -- restarts on
// interior points is what did it, not the loosened tolerance.
arm_kinematics::PositionPitchIkOptions PathTrackingIkOptions(
    const arm_kinematics::PositionPitchIkOptions& base) {
  arm_kinematics::PositionPitchIkOptions options = base;
  options.position_tolerance = 1e-3;  // 10x looser than PositionPitchIkOptions' own default (1e-4)
  options.pitch_tolerance = 5e-3;     // 5x looser than the default (1e-3)
  options.max_restarts = 0;           // see above -- load-bearing, not an optimization
  // Best-effort means we accept whatever residual a sample settles at
  // regardless -- burning a large iteration budget on a sample that
  // structurally can't converge further wastes work for no benefit, and
  // across hundreds of samples in a long/hard path that adds up to real
  // wall-clock time (found empirically: one test case took 10s, another
  // didn't finish in 60s, before this). A much smaller budget still lets a
  // genuinely-reachable sample converge quickly; it just stops sooner on
  // one that can't. (No max_inner_retries here, unlike the old
  // InverseKinematicsSolver-based version: PositionPitchIkSolver has no
  // accept/reject LM logic to retry -- see position_pitch_ik.h.)
  options.max_iterations = 20;
  return options;
}

struct SampleAttempt {
  std::vector<std::vector<double>> q_samples;
  std::vector<double> t_samples;
  double peak_position_deviation;     // meters, worst interior-sample residual (best-effort tracking)
  double peak_orientation_deviation;  // radians
};

SampleAttempt GenerateSamplesAtDuration(const arm_kinematics::ForwardKinematicsSolver& fk_solver,
                                         const arm_kinematics::PositionPitchIkOptions& ik_options,
                                         const std::vector<double>& q_start,
                                         const Eigen::Isometry3d& pose_start,
                                         const Eigen::Isometry3d& pose_goal, double duration,
                                         int num_samples) {
  // Own solver, looser tolerances -- see PathTrackingIkOptions().
  arm_kinematics::PositionPitchIkSolver path_ik_solver(fk_solver, PathTrackingIkOptions(ik_options));

  SampleAttempt attempt;
  attempt.q_samples.reserve(num_samples);
  attempt.t_samples.reserve(num_samples);
  attempt.peak_position_deviation = 0.0;
  attempt.peak_orientation_deviation = 0.0;

  std::vector<double> q_prev = q_start;
  for (int k = 0; k < num_samples; ++k) {
    double t = duration * static_cast<double>(k) / static_cast<double>(num_samples - 1);
    double tau = t / duration;
    double frac = QuinticPosition(tau);

    Eigen::Isometry3d target = (k == num_samples - 1) ? pose_goal
                               : (k == 0)              ? pose_start
                                                        : InterpolatePose(pose_start, pose_goal, frac);

    // Best-effort: take whatever IK finds, converged or not -- an
    // interior waypoint not being exactly reachable is expected on this
    // chain (see above), not a failure condition. Only the goal (checked
    // by the caller before this function runs) must be exact. residual's
    // rotation component (peak_orientation_deviation below) is
    // informational only, same convention as PositionPitchIkResult
    // elsewhere -- only its pitch-axis projection is actually enforced.
    arm_kinematics::PositionPitchIkResult ik = path_ik_solver.Solve(target, q_prev);
    attempt.peak_position_deviation =
        std::max(attempt.peak_position_deviation, ik.residual.head<3>().norm());
    attempt.peak_orientation_deviation =
        std::max(attempt.peak_orientation_deviation, ik.residual.tail<3>().norm());

    attempt.q_samples.push_back(ik.joint_positions);
    attempt.t_samples.push_back(t);
    q_prev = ik.joint_positions;
  }

  return attempt;
}

int ProjectedSampleCount(double duration, double sample_rate_hz) {
  return std::max(2, static_cast<int>(std::round(duration * sample_rate_hz)) + 1);
}

// Returns the largest (|finite-difference velocity| / limit) ratio found
// across all joints and consecutive sample pairs; <= 1.0 means every
// joint stayed within its velocity limit throughout.
double WorstVelocityRatio(const std::vector<std::vector<double>>& q_samples,
                           const std::vector<double>& t_samples,
                           const std::vector<double>& velocity_limits) {
  double worst = 0.0;
  for (size_t k = 1; k < q_samples.size(); ++k) {
    double dt = t_samples[k] - t_samples[k - 1];
    for (size_t i = 0; i < velocity_limits.size(); ++i) {
      double v = std::fabs(q_samples[k][i] - q_samples[k - 1][i]) / dt;
      worst = std::max(worst, v / velocity_limits[i]);
    }
  }
  return worst;
}

}  // namespace

CartesianTrajectoryResult GenerateCartesianTrajectory(
    const arm_kinematics::ForwardKinematicsSolver& fk_solver,
    const arm_kinematics::PositionPitchIkOptions& ik_options, const std::vector<double>& q_start,
    const Eigen::Isometry3d& pose_goal, const std::vector<double>& velocity_limits,
    const std::vector<std::string>& joint_names, double sample_rate_hz,
    int max_duration_iterations) {
  CartesianTrajectoryResult result;
  Eigen::Isometry3d pose_start = fk_solver.ComputePose(q_start);

  // Seed the joint-space delta (and hence duration) with a single IK
  // solve at the goal -- also validates the goal is reachable at all
  // before any sampling work. Own solver at ik_options' own (strict)
  // tolerances -- interior samples get their own, looser one, see
  // GenerateSamplesAtDuration/PathTrackingIkOptions().
  arm_kinematics::PositionPitchIkSolver goal_ik_solver(fk_solver, ik_options);
  arm_kinematics::PositionPitchIkResult goal_ik = goal_ik_solver.Solve(pose_goal, q_start);
  if (!goal_ik.converged) {
    result.success = false;
    result.message = "goal pose did not converge: position error " +
                      std::to_string(goal_ik.residual.head<3>().norm()) +
                      " m, rotation residual (informational; only its pitch component is enforced) " +
                      std::to_string(goal_ik.residual.tail<3>().norm()) + " rad, after " +
                      std::to_string(goal_ik.restarts_used) + " restarts";
    return result;
  }

  double duration = 0.0;
  for (size_t i = 0; i < q_start.size(); ++i) {
    duration = std::max(
        duration, MinDurationForVelocityLimit(goal_ik.joint_positions[i] - q_start[i], velocity_limits[i]));
  }

  if (duration == 0.0) {
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = q_start;
    point.velocities.assign(q_start.size(), 0.0);
    point.time_from_start = ros::Duration(0.0);
    traj.points.push_back(point);
    result.success = true;
    result.trajectory = traj;
    return result;
  }

  // Defensive sanity cap, independent of PathTrackingIkOptions'
  // max_restarts=0 fix for the root cause (a discontinuous warm-start):
  // duration-scaling multiplies by whatever worst_ratio comes back, so
  // one anomalous sample could in principle still blow it up. This turns
  // "generate an absurd number of samples" into a clean failure instead
  // -- found the hard way by a test that hung for 15+ minutes before this
  // existed.
  constexpr int kMaxReasonableSamples = 5000;

  // Coarse-to-fine duration search (M3 Finding 4 fix): the original
  // version re-sampled the WHOLE path at full sample_rate_hz resolution
  // on every rescaling iteration, so cost scaled with
  // max_duration_iterations x (duration x sample_rate_hz) x per-sample IK
  // cost -- the dominant cost for a large Cartesian move. The velocity
  // limit check only needs approximately the right duration to converge,
  // not a full-resolution trajectory at every guess: search cheaply first
  // at a small, FIXED sample count (independent of duration/sample_rate_hz,
  // so it stays cheap even for a large move), then run the real
  // sample_rate_hz resolution only once the duration estimate has
  // converged. A coarser search sample spacing means the finite-differenced
  // velocity there is an average over a wider interval and can UNDERESTIMATE
  // a true peak within it -- never overestimate in a way that would make
  // the final check unnecessarily strict -- so a bounded number of
  // full-resolution correction passes still follows the coarse search,
  // and the full-resolution attempt actually returned is always the one
  // that passed the real check. Same success/failure criteria and
  // trajectory as before; only the search cost changes.
  constexpr int kCoarseSearchSamples = 25;
  // Same ceiling as the coarse search (and as the original single-phase
  // loop before this change): in practice the coarse phase gets duration
  // very close, so this almost always takes 0-1 passes -- but it must not
  // be tighter than the original algorithm's own convergence guarantee,
  // or a move that genuinely needed several rescaling steps can fail here
  // where the original would have succeeded. Confirmed the hard way: a
  // hardcoded cap of 2 failed a real move whose worst ratio was only
  // 1.023 (one more pass would have converged it).
  const int max_full_resolution_corrections = max_duration_iterations;

  for (int iter = 0; iter < max_duration_iterations; ++iter) {
    SampleAttempt coarse = GenerateSamplesAtDuration(fk_solver, ik_options, q_start, pose_start,
                                                      pose_goal, duration, kCoarseSearchSamples);
    double worst_ratio = WorstVelocityRatio(coarse.q_samples, coarse.t_samples, velocity_limits);
    if (worst_ratio <= 1.0) {
      break;
    }
    if (iter == max_duration_iterations - 1) {
      result.success = false;
      result.message = "could not satisfy velocity limits within " +
                        std::to_string(max_duration_iterations) +
                        " coarse duration-scaling iterations (worst ratio " +
                        std::to_string(worst_ratio) + ")";
      return result;
    }
    duration *= worst_ratio * 1.01;  // small safety margin past the exact scaling factor
  }

  SampleAttempt attempt;
  for (int correction = 0; correction <= max_full_resolution_corrections; ++correction) {
    int projected_samples = ProjectedSampleCount(duration, sample_rate_hz);
    if (projected_samples > kMaxReasonableSamples) {
      result.success = false;
      result.message = "projected sample count (" + std::to_string(projected_samples) +
                        ") exceeds sanity limit (" + std::to_string(kMaxReasonableSamples) +
                        ") at duration=" + std::to_string(duration) + "s -- likely runaway duration scaling";
      return result;
    }

    attempt = GenerateSamplesAtDuration(fk_solver, ik_options, q_start, pose_start, pose_goal,
                                        duration, projected_samples);

    double worst_ratio = WorstVelocityRatio(attempt.q_samples, attempt.t_samples, velocity_limits);
    if (worst_ratio <= 1.0) {
      break;
    }
    if (correction == max_full_resolution_corrections) {
      result.success = false;
      result.message = "could not satisfy velocity limits within " +
                        std::to_string(max_full_resolution_corrections) +
                        " full-resolution correction passes after coarse search (worst ratio " +
                        std::to_string(worst_ratio) + ")";
      return result;
    }
    duration *= worst_ratio * 1.01;
  }

  trajectory_msgs::JointTrajectory traj;
  traj.joint_names = joint_names;
  traj.points.reserve(attempt.q_samples.size());
  for (size_t k = 0; k < attempt.q_samples.size(); ++k) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = attempt.q_samples[k];
    point.velocities.assign(joint_names.size(), 0.0);
    if (k > 0) {
      double dt = attempt.t_samples[k] - attempt.t_samples[k - 1];
      for (size_t i = 0; i < joint_names.size(); ++i) {
        point.velocities[i] = (attempt.q_samples[k][i] - attempt.q_samples[k - 1][i]) / dt;
      }
    }
    point.time_from_start = ros::Duration(attempt.t_samples[k]);
    traj.points.push_back(point);
  }

  result.success = true;
  result.trajectory = traj;
  result.peak_position_deviation = attempt.peak_position_deviation;
  result.peak_orientation_deviation = attempt.peak_orientation_deviation;
  return result;
}

}  // namespace arm_trajectory
