#include "arm_kinematics/position_pitch_ik.h"

#include <algorithm>
#include <stdexcept>

#include "arm_kinematics/jacobian.h"
#include "task_space_math.h"

namespace arm_kinematics {

namespace {

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

}  // namespace

PositionPitchIkSolver::PositionPitchIkSolver(const ForwardKinematicsSolver& solver,
                                              PositionPitchIkOptions options)
    : solver_(solver), options_(options), rng_(options.rng_seed) {}

PositionPitchIkResult PositionPitchIkSolver::Solve(const Eigen::Isometry3d& target_pose,
                                                     const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  if (seed.size() != limits.size()) {
    throw std::invalid_argument("PositionPitchIkSolver::Solve: seed has " +
                                 std::to_string(seed.size()) + " values, expected " +
                                 std::to_string(limits.size()));
  }

  PositionPitchIkResult best = SolveFromSeed(target_pose, seed);

  std::vector<std::uniform_real_distribution<double>> dists;
  dists.reserve(limits.size());
  for (const auto& lim : limits) {
    dists.emplace_back(lim.first, lim.second);
  }

  // Same restart bookkeeping as InverseKinematicsSolver::Solve: count every
  // attempt made, not just whichever one produced the best result.
  int restarts_attempted = 0;
  for (int r = 0; r < options_.max_restarts && !best.converged; ++r) {
    std::vector<double> restart_seed(limits.size());
    for (size_t i = 0; i < limits.size(); ++i) {
      restart_seed[i] = dists[i](rng_);
    }
    PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
    ++restarts_attempted;
    if (candidate.converged || candidate.residual.head<3>().norm() < best.residual.head<3>().norm()) {
      best = candidate;
    }
  }
  best.restarts_used = restarts_attempted;
  return best;
}

PositionPitchIkResult PositionPitchIkSolver::SolveFromSeed(const Eigen::Isometry3d& target_pose,
                                                             const std::vector<double>& seed) const {
  const auto& limits = solver_.GetActuatedJointLimits();
  const size_t n = seed.size();
  std::vector<double> q = seed;

  size_t m = 0;
  if (options_.nullspace_secondary_link_solver != nullptr && options_.nullspace_gain != 0.0) {
    m = options_.nullspace_secondary_link_solver->GetActuatedJointNames().size();
    if (m > n) {
      throw std::invalid_argument(
          "PositionPitchIkSolver: nullspace_secondary_link_solver has more actuated joints than "
          "the chain -- it must be an upstream sub-chain of solver's");
    }
  }

  Eigen::Matrix<double, 6, 1> residual = Eigen::Matrix<double, 6, 1>::Zero();
  int iterations = 0;
  bool converged = false;

  for (; iterations < options_.max_iterations; ++iterations) {
    Eigen::Isometry3d pose = solver_.ComputePose(q);
    Eigen::Vector3d pitch_axis = pose.rotation() * Eigen::Vector3d::UnitY();

    Eigen::Vector3d position_error = target_pose.translation() - pose.translation();
    Eigen::Vector3d rotation_error = RotationVector(target_pose.rotation() * pose.rotation().transpose());
    double pitch_error = pitch_axis.dot(rotation_error);

    residual.head<3>() = position_error;
    residual.tail<3>() = rotation_error;  // full 3-component rotation error, informational (see header)

    Eigen::Matrix<double, 6, Eigen::Dynamic> J = ComputeGeometricJacobian(solver_, q);
    Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
    J_task.topRows<3>() = J.topRows<3>();
    J_task.row(3) = pitch_axis.transpose() * J.bottomRows<3>();

    Eigen::Vector4d e_task(position_error.x(), position_error.y(), position_error.z(), pitch_error);

    // Computed BEFORE the convergence check (not just before the step)
    // deliberately: convergence below requires this to have settled too,
    // not just the primary task, and its own magnitude is exactly the
    // right thing to check that against -- it has a genuine fixed point
    // at zero (see resolved_rate.h/.md's scalar-triple-product identity),
    // so "small qdot0" really does mean "close to the secondary
    // objective," not just "few iterations happened to run."
    Eigen::VectorXd qdot0;
    bool have_nullspace_bias = false;
    if (m > 0) {
      std::vector<double> q_prefix(q.begin(), q.begin() + m);
      Eigen::Vector3d link_axis =
          options_.nullspace_secondary_link_solver->ComputePose(q_prefix).rotation() *
          Eigen::Vector3d::UnitZ();
      const Eigen::Vector3d world_up = Eigen::Vector3d::UnitZ();
      qdot0 = Eigen::VectorXd::Zero(n);
      for (size_t i = 0; i < m; ++i) {
        Eigen::Vector3d z_i = J.block<3, 1>(3, i);
        qdot0(i) = options_.nullspace_gain * world_up.dot(z_i.cross(link_axis));
      }
      have_nullspace_bias = true;
    }

    // Real bug, confirmed empirically before this fix: breaking as soon
    // as position+pitch converged gave the null-space bias exactly as
    // many iterations as the PRIMARY task happened to need and not one
    // more -- typically far too few for the bias's own, much slower,
    // per-step-limited convergence to get anywhere close to its fixed
    // point. A 100-target sweep isolating restart randomness (fixed rng,
    // only seeds that converged directly compared) showed the bias
    // always improves alignment relative to no bias at all (so its
    // direction/sign was never wrong), but still landed worse than the
    // *seed's own starting alignment* in the majority of cases simply
    // because the loop stopped too early to let it work. Once the
    // primary task is satisfied, e_task is already ~0, so continuing to
    // iterate costs nothing extra there -- the step becomes almost pure
    // null-space work "for free," so requiring the bias to *also* have
    // settled (or the iteration budget to run out) rather than stopping
    // the instant the primary does is the actual fix, not a change to
    // the bias math itself (which was already correct).
    bool primary_converged = position_error.norm() < options_.position_tolerance &&
                              std::abs(pitch_error) < options_.pitch_tolerance;
    bool secondary_converged = !have_nullspace_bias || qdot0.norm() < options_.nullspace_tolerance;
    if (primary_converged && secondary_converged) {
      break;  // nothing left to gain from further iteration
    }

    Eigen::VectorXd dq = detail::SolveDampedRightPseudoInverse(
        J_task, e_task, options_.damping, have_nullspace_bias ? &qdot0 : nullptr);

    double step_norm = dq.lpNorm<Eigen::Infinity>();
    if (step_norm > options_.max_step) {
      dq *= (options_.max_step / step_norm);
    }

    for (size_t i = 0; i < n; ++i) {
      q[i] = std::min(std::max(q[i] + dq(i), limits[i].first), limits[i].second);
    }
  }

  // `residual`/`converged` are both determined from the TRUE final state
  // here, not from whatever they were mid-loop -- `residual` above is
  // computed at the START of each iteration (before that iteration's
  // step), so unless the loop exited via the both-converged `break`
  // (where nothing changed after it was last computed), it's stale by
  // one step relative to the `q` a subsequent step, or max_iterations
  // exhaustion, just left behind. And `converged` genuinely needs
  // recomputing here rather than latching true the moment primary first
  // looked satisfied mid-loop: a real bug caught live -- once the loop
  // was fixed (above) to keep running after primary convergence so the
  // null-space bias gets real iterations, the bias's own damped (not
  // exact) null-space projection can leak a small amount back into the
  // primary task over MANY subsequent steps, occasionally drifting
  // position error back out of tolerance by the time the budget runs
  // out, even though it was satisfied earlier. `converged` must mean
  // "the returned joint_positions/residual actually satisfy the
  // tolerance," not "did at some earlier point" -- recomputing
  // unconditionally is also simpler than tracking exactly which exit
  // path needs it, at the cost of one extra FK call.
  Eigen::Vector3d final_position_error;
  double final_pitch_error;
  {
    Eigen::Isometry3d pose = solver_.ComputePose(q);
    Eigen::Vector3d pitch_axis = pose.rotation() * Eigen::Vector3d::UnitY();
    final_position_error = target_pose.translation() - pose.translation();
    Eigen::Vector3d rotation_error =
        RotationVector(target_pose.rotation() * pose.rotation().transpose());
    final_pitch_error = pitch_axis.dot(rotation_error);
    residual.head<3>() = final_position_error;
    residual.tail<3>() = rotation_error;
  }
  converged = final_position_error.norm() < options_.position_tolerance &&
              std::abs(final_pitch_error) < options_.pitch_tolerance;

  PositionPitchIkResult result;
  result.joint_positions = q;
  result.residual = residual;
  result.iterations = iterations;
  result.converged = converged;
  result.restarts_used = 0;  // caller (Solve) fills this in
  return result;
}

}  // namespace arm_kinematics
