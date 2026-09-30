#include "arm_kinematics/position_pitch_ik.h"

#include <algorithm>
#include <stdexcept>

#include "arm_kinematics/jacobian.h"
#include "arm_kinematics/joint_limit_sampling.h"
#include "task_space_math.h"

namespace arm_kinematics {

namespace {

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

struct TaskError {
  Eigen::Vector3d position_error;
  Eigen::Vector3d rotation_error;  // full 3-component, informational (see header)
  double pitch_error;              // rotation_error projected onto the CURRENT iterate's own
                                    // local Y axis -- the only rotation component enforced
  Eigen::Vector3d pitch_axis;      // that same axis, in world frame -- exposed since
                                    // SolveFromSeed's main loop also needs it to build J_task,
                                    // and re-deriving it would mean a second ComputePose call
};

// Position/rotation/pitch error of `q` against `target_pose`, same
// convention everywhere it's used in this file (SolveFromSeed's per-
// iteration error, the drift-cap and backtracking candidate checks, and
// the final residual recompute) -- pure evaluation, no side effects.
TaskError ComputeTaskError(const ForwardKinematicsSolver& solver, const Eigen::Isometry3d& target_pose,
                            const std::vector<double>& q) {
  Eigen::Isometry3d pose = solver.ComputePose(q);
  TaskError e;
  e.pitch_axis = pose.rotation() * Eigen::Vector3d::UnitY();
  e.position_error = target_pose.translation() - pose.translation();
  e.rotation_error = RotationVector(target_pose.rotation() * pose.rotation().transpose());
  e.pitch_error = e.pitch_axis.dot(e.rotation_error);
  return e;
}

// Shared between SolveFromSeed's drift cap (bounding how far a
// secondary-task-only step may leak the primary task back out of
// tolerance) and Solve()'s restart/selection logic (what counts as
// "acceptable" for alignment-based comparison) -- both need the SAME
// margin, or a seed SolveFromSeed correctly kept within bound could still
// get rejected by a stricter selection check, defeating the whole point of
// capping the drift in the first place. See SolveFromSeed's own comment
// (measured 64x, on an uncapped run) for why 10x, not something smaller,
// is actually needed in practice.
constexpr double kDriftCapMultiplier = 10.0;

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

  std::vector<std::uniform_real_distribution<double>> dists = DistributionsFromJointLimits(limits);

  bool have_nullspace_bias =
      options_.nullspace_secondary_link_solver != nullptr && options_.nullspace_gain != 0.0;

  // Pitch error isn't stored directly in PositionPitchIkResult (only the
  // full, mostly-informational rotation residual is) -- recompute it the
  // same way SolveFromSeed does, from the result's own final pose.
  auto PitchError = [this](const PositionPitchIkResult& r) {
    Eigen::Vector3d pitch_axis = solver_.ComputePose(r.joint_positions).rotation() * Eigen::Vector3d::UnitY();
    return pitch_axis.dot(r.residual.tail<3>());
  };
  // Same bound as SolveFromSeed's own drift cap (kDriftCapMultiplier) --
  // "acceptable" here means "as good as a capped secondary-task drift is
  // ever allowed to leave the primary task," not full strict convergence.
  auto IsAcceptable = [this, &PitchError](const PositionPitchIkResult& r) {
    return r.residual.head<3>().norm() < kDriftCapMultiplier * options_.position_tolerance &&
           std::abs(PitchError(r)) < kDriftCapMultiplier * options_.pitch_tolerance;
  };
  auto Alignment = [this](const std::vector<double>& q) {
    size_t m = options_.nullspace_secondary_link_solver->GetActuatedJointNames().size();
    std::vector<double> q_prefix(q.begin(), q.begin() + m);
    Eigen::Vector3d axis =
        options_.nullspace_secondary_link_solver->ComputePose(q_prefix).rotation() *
        Eigen::Vector3d::UnitZ();
    return Eigen::Vector3d::UnitZ().dot(axis);
  };

  // Real bug, found live: with a null-space task configured, a seed
  // already sitting on excellent alignment (e.g. the arm's own current
  // pose, warm-started from itself) can leak just past the tight primary
  // tolerance purely from the capped secondary-task drift above -- not
  // "genuinely failed to converge." The ORIGINAL restart policy below
  // (strict `converged`, "keep whichever has lower position residual")
  // would then reseed from a totally unrelated RANDOM joint configuration
  // and keep it purely for edging out on position residual, discarding
  // whatever alignment the original near-converged seed had already
  // achieved -- alignment never entered the comparison at all. When a
  // null-space task is configured, "acceptable" (within the same drift
  // cap) substitutes for strict convergence as the bar, and among
  // acceptable candidates the better-aligned one wins.
  int restarts_attempted = 0;
  if (have_nullspace_bias) {
    for (int r = 0; r < options_.max_restarts && !IsAcceptable(best); ++r) {
      std::vector<double> restart_seed(limits.size());
      for (size_t i = 0; i < limits.size(); ++i) {
        restart_seed[i] = dists[i](rng_);
      }
      PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
      ++restarts_attempted;
      bool best_acceptable = IsAcceptable(best);
      bool candidate_acceptable = IsAcceptable(candidate);
      if (candidate_acceptable && (!best_acceptable ||
                                    Alignment(candidate.joint_positions) > Alignment(best.joint_positions))) {
        best = candidate;
      } else if (!candidate_acceptable && !best_acceptable &&
                 candidate.residual.head<3>().norm() < best.residual.head<3>().norm()) {
        best = candidate;  // neither is acceptable yet -- fall back to the old policy
      }
    }
  } else {
    // Same restart bookkeeping as InverseKinematicsSolver::Solve: count
    // every attempt made, not just whichever one produced the best result.
    for (int r = 0; r < options_.max_restarts && !best.converged; ++r) {
      std::vector<double> restart_seed(limits.size());
      for (size_t i = 0; i < limits.size(); ++i) {
        restart_seed[i] = dists[i](rng_);
      }
      PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
      ++restarts_attempted;
      if (candidate.converged ||
          candidate.residual.head<3>().norm() < best.residual.head<3>().norm()) {
        best = candidate;
      }
    }
  }

  if (have_nullspace_bias) {
    // `converged`'s meaning shifts here: SolveFromSeed's own field is
    // always strict (position_tolerance/pitch_tolerance exactly), but
    // when a null-space task is configured, `best` was deliberately
    // selected against the SAME drift-capped "acceptable" bar the
    // restart loop above uses -- reporting the strict, pre-selection
    // value here would tell every caller (trajectory_service_node,
    // GenerateCartesianTrajectory's goal check, ...) that a result this
    // function chose on purpose, specifically because it was good enough
    // and well-aligned, had "failed," which is exactly the false failure
    // this whole fix exists to avoid. See docs/position_pitch_ik.md.
    best.converged = IsAcceptable(best);
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
  // Tracks whether the primary task has EVER been satisfied so far in this
  // call -- once true, subsequent iterations are pure secondary-task work
  // and their drift back into the primary task gets bounded below (see
  // kDriftCapMultiplier), rather than left free to accumulate for the rest
  // of the iteration budget.
  bool primary_ever_converged = false;

  for (; iterations < options_.max_iterations; ++iterations) {
    TaskError e = ComputeTaskError(solver_, target_pose, q);

    residual.head<3>() = e.position_error;
    residual.tail<3>() = e.rotation_error;  // full 3-component rotation error, informational (see header)

    Eigen::Matrix<double, 6, Eigen::Dynamic> J = ComputeGeometricJacobian(solver_, q);
    Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
    J_task.topRows<3>() = J.topRows<3>();
    J_task.row(3) = e.pitch_axis.transpose() * J.bottomRows<3>();

    Eigen::Vector4d e_task(e.position_error.x(), e.position_error.y(), e.position_error.z(),
                            e.pitch_error);

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
    bool primary_converged = e.position_error.norm() < options_.position_tolerance &&
                              std::abs(e.pitch_error) < options_.pitch_tolerance;
    bool secondary_converged = !have_nullspace_bias || qdot0.norm() < options_.nullspace_tolerance;
    if (primary_converged && secondary_converged) {
      break;  // nothing left to gain from further iteration
    }
    primary_ever_converged = primary_ever_converged || primary_converged;

    Eigen::VectorXd dq = detail::SolveDampedRightPseudoInverse(
        J_task, e_task, options_.damping, have_nullspace_bias ? &qdot0 : nullptr);

    double step_norm = dq.lpNorm<Eigen::Infinity>();
    if (step_norm > options_.max_step) {
      dq *= (options_.max_step / step_norm);
    }

    if (have_nullspace_bias && !primary_ever_converged) {
      // A configured secondary task mixes null-space work into every dq
      // from the very first iteration, not just after primary convergence
      // (see NullspaceBiasImprovesVerticalAlignmentAtConvergence) -- its
      // leakage can transiently raise ||e_task||^2 well before primary
      // convergence too, which a cost-based rejection can't tell apart
      // from genuinely diverging. That leakage is an accepted trade-off
      // WHILE still working toward primary convergence -- so no
      // acceptance check applies here at all. Once primary has been
      // satisfied even once, though, the drift-cap branch below takes
      // over instead of this unconditional accept (see its own comment).
      for (size_t i = 0; i < n; ++i) {
        q[i] = std::min(std::max(q[i] + dq(i), limits[i].first), limits[i].second);
      }
      continue;
    }

    if (have_nullspace_bias && primary_ever_converged) {
      // Real bug, found live via a day-to-day /reach_target_pose call:
      // once primary is satisfied, remaining iterations are pure
      // secondary-task work, and letting that run completely unchecked
      // for the rest of the iteration budget (as the branch above does,
      // deliberately, while primary hasn't converged even once yet) can
      // leak the primary residual far past its own tolerance -- measured
      // on a real case, 0.0064m against a 1e-4m tolerance (64x), after
      // 200 unconditional iterations from an already-exactly-converged
      // seed. That's not a marginal wobble to shrug off: it was enough to
      // flip `converged` to false, which then sent Solve()'s restart loop
      // off to a completely unrelated random basin -- discarding whatever
      // alignment progress had already been made from a good seed in
      // favor of an arbitrary "exactly converged" one with no relation to
      // vertical alignment at all. Fix: cap how far a step is allowed to
      // drift the primary task back out, relative to its own tolerance --
      // small and well-justified now that it bounds drift FROM an
      // already-good state, not (like max_inner_retries' backtracking)
      // trying to recover an unbounded blowup after the fact. Halve and
      // retry within bound like the no-bias case above; if nothing within
      // bound is found, stop -- there's nothing more to safely gain.
      Eigen::VectorXd trial_dq = dq;
      bool step_within_bound = false;
      for (int inner = 0; inner < options_.max_inner_retries; ++inner) {
        std::vector<double> q_candidate = q;
        for (size_t i = 0; i < n; ++i) {
          q_candidate[i] =
              std::min(std::max(q[i] + trial_dq(i), limits[i].first), limits[i].second);
        }
        TaskError e_candidate = ComputeTaskError(solver_, target_pose, q_candidate);

        if (e_candidate.position_error.norm() < kDriftCapMultiplier * options_.position_tolerance &&
            std::abs(e_candidate.pitch_error) < kDriftCapMultiplier * options_.pitch_tolerance) {
          q = q_candidate;
          step_within_bound = true;
          break;
        }
        trial_dq *= 0.5;
      }

      if (!step_within_bound) {
        break;  // no further secondary-task progress possible without breaching the drift cap
      }
      continue;
    }

    // Backtracking line search on the primary task cost (see
    // max_inner_retries' own comment for the real bug this fixes): only
    // take this step if it actually reduces ||e_task||^2, halving and
    // retrying otherwise. e_task itself is untouched by this -- it's
    // recomputed fresh next outer iteration from whatever q ends up being.
    double cost = e_task.squaredNorm();
    Eigen::VectorXd trial_dq = dq;
    bool step_accepted = false;
    for (int inner = 0; inner < options_.max_inner_retries; ++inner) {
      std::vector<double> q_candidate = q;
      for (size_t i = 0; i < n; ++i) {
        q_candidate[i] =
            std::min(std::max(q[i] + trial_dq(i), limits[i].first), limits[i].second);
      }
      TaskError e_candidate = ComputeTaskError(solver_, target_pose, q_candidate);
      Eigen::Vector4d e_task_candidate(e_candidate.position_error.x(), e_candidate.position_error.y(),
                                        e_candidate.position_error.z(), e_candidate.pitch_error);
      double cost_candidate = e_task_candidate.squaredNorm();

      if (cost_candidate < cost) {
        q = q_candidate;
        step_accepted = true;
        break;
      }
      trial_dq *= 0.5;
    }

    if (!step_accepted) {
      // No improving step found even after halving max_inner_retries
      // times: a genuine local minimum (or a target this solver can't
      // actually reach -- expected for some interior Cartesian-path
      // samples, see docs/position_pitch_ik.md), not something more outer
      // iterations alone would fix. Stop rather than keep wandering.
      break;
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
  TaskError final_error = ComputeTaskError(solver_, target_pose, q);
  residual.head<3>() = final_error.position_error;
  residual.tail<3>() = final_error.rotation_error;
  converged = final_error.position_error.norm() < options_.position_tolerance &&
              std::abs(final_error.pitch_error) < options_.pitch_tolerance;

  PositionPitchIkResult result;
  result.joint_positions = q;
  result.residual = residual;
  result.iterations = iterations;
  result.converged = converged;
  result.restarts_used = 0;  // caller (Solve) fills this in
  return result;
}

}  // namespace arm_kinematics
