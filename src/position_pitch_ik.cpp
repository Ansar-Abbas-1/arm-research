#include "arm_kinematics/position_pitch_ik.h"

#include <algorithm>
#include <cmath>
#include <functional>
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

// After the primary task is inside its tolerance, SolveFromSeed refines for at
// most kRefineIterations more Newton steps, until the error is below
// kRefineFraction of the tolerance. Heuristic values, not derived.
constexpr int kRefineIterations = 6;
constexpr double kRefineFraction = 0.1;

// Shared mechanics behind SolveFromSeed's two backtracking regimes (the
// drift cap once primary has converged, and the primary-task line search
// before it has) -- both are "try dq, and if the resulting TaskError
// doesn't satisfy some condition, halve dq and retry, up to
// max_inner_retries times." Only that condition differs between the two
// (see their own call sites), so it's the one thing left as a parameter;
// everything else -- building/clamping q_candidate, the halving, the
// retry budget -- was identical duplicated code before this. On success,
// *q is updated to the accepted candidate and this returns true; on
// failure (no candidate within max_inner_retries halvings satisfied
// `accept`), *q is left untouched and this returns false.
bool TryBacktrackingStep(const ForwardKinematicsSolver& solver, const Eigen::Isometry3d& target_pose,
                          const std::vector<std::pair<double, double>>& limits, int max_inner_retries,
                          const Eigen::VectorXd& dq, std::vector<double>* q,
                          const std::function<bool(const TaskError&)>& accept) {
  const size_t n = q->size();
  Eigen::VectorXd trial_dq = dq;
  for (int inner = 0; inner < max_inner_retries; ++inner) {
    std::vector<double> q_candidate = *q;
    for (size_t i = 0; i < n; ++i) {
      q_candidate[i] = std::min(std::max((*q)[i] + trial_dq(i), limits[i].first), limits[i].second);
    }
    TaskError e_candidate = ComputeTaskError(solver, target_pose, q_candidate);
    if (accept(e_candidate)) {
      *q = q_candidate;
      return true;
    }
    trial_dq *= 0.5;
  }
  return false;
}


// Signed alignment a = up . (link axis z) of the secondary link, from the
// chain configuration q (only the first `m` joints affect that link).
double LinkAlignment(const ForwardKinematicsSolver& link_solver, const std::vector<double>& q) {
  const size_t m = link_solver.GetActuatedJointNames().size();
  std::vector<double> q_prefix(q.begin(), q.begin() + m);
  return (link_solver.ComputePose(q_prefix).rotation() * Eigen::Vector3d::UnitZ()).z();
}

// 4 x n task Jacobian (position rows + the pitch row) and the matching task
// error, the same quantities SolveFromSeed's main loop uses.
void BuildTask(const ForwardKinematicsSolver& chain, const Eigen::Isometry3d& target,
               const std::vector<double>& q, Eigen::Matrix<double, 6, Eigen::Dynamic>* J_full,
               Eigen::Matrix<double, 4, Eigen::Dynamic>* J_task, Eigen::Vector4d* e_task,
               TaskError* e) {
  *e = ComputeTaskError(chain, target, q);
  *J_full = ComputeGeometricJacobian(chain, q);
  J_task->resize(4, q.size());
  J_task->topRows<3>() = J_full->topRows<3>();
  J_task->row(3) = e->pitch_axis.transpose() * J_full->bottomRows<3>();
  *e_task = Eigen::Vector4d(e->position_error.x(), e->position_error.y(), e->position_error.z(),
                            e->pitch_error);
}

// Moves q along the self-motion manifold of the primary task (position +
// pitch fixed) toward the most vertical secondary link, maximising |a| with
// a = up . z_link. Projected-gradient ascent with a Newton retraction onto
// the manifold; see docs/position_pitch_ik.md "Posture optimisation".
//   g   = s * grad(a),  grad(a)_i = up . (w_i x z_link) for the joints upstream
//         of the link (w_i = world joint axis), s = sign(a) with s(0) = +1
//   g_N = g projected onto ker(J_task)
//   step: lambda = min(max_step, (1 - |a|) / |g_N|) along g_N / |g_N|, halved
//         until the retracted point keeps the primary task within tolerance
//         and strictly raises |a|.
// Total joint motion (sum of the infinity norms of accepted steps) is bounded by
// options.nullspace_max_displacement when that is > 0. Leaves *q unchanged if no
// step is accepted. Never worsens the primary task: every accepted point passes
// the strict tolerances.
void OptimizePosture(const ForwardKinematicsSolver& chain, const PositionPitchIkOptions& o,
                     const Eigen::Isometry3d& target, std::vector<double>* q) {
  const auto& limits = chain.GetActuatedJointLimits();
  const size_t n = q->size();
  const ForwardKinematicsSolver& link = *o.nullspace_secondary_link_solver;
  const size_t m = link.GetActuatedJointNames().size();
  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  const bool budgeted = o.nullspace_max_displacement > 0.0;
  double used = 0.0;

  auto within_tolerance = [&](const TaskError& e) {
    return e.position_error.norm() < o.position_tolerance && std::abs(e.pitch_error) < o.pitch_tolerance;
  };

  for (int it = 0; it < o.nullspace_max_iterations; ++it) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> J;
    Eigen::Matrix<double, 4, Eigen::Dynamic> J_task;
    Eigen::Vector4d e_task;
    TaskError e;
    BuildTask(chain, target, *q, &J, &J_task, &e_task, &e);

    std::vector<double> q_prefix(q->begin(), q->begin() + m);
    const Eigen::Vector3d axis = link.ComputePose(q_prefix).rotation() * Eigen::Vector3d::UnitZ();
    const double a = up.dot(axis);
    const double s = a >= 0.0 ? 1.0 : -1.0;
    Eigen::VectorXd g = Eigen::VectorXd::Zero(n);
    for (size_t i = 0; i < m; ++i) {
      g(i) = s * up.dot(J.block<3, 1>(3, i).cross(axis));
    }

    // Projector onto ker(J_task) from the SVD (rank decided relative to sigma_max).
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_task, Eigen::ComputeFullV);
    const Eigen::VectorXd sv = svd.singularValues();
    int rank = 0;
    for (int k = 0; k < sv.size(); ++k) {
      if (sv(k) > 1e-6 * sv(0) && sv(k) > 1e-9) ++rank;
    }
    Eigen::VectorXd gN = g;
    for (int k = 0; k < rank; ++k) {
      gN -= svd.matrixV().col(k) * svd.matrixV().col(k).dot(g);
    }
    const double gnorm = gN.norm();
    if (gnorm < o.nullspace_tolerance) {
      break;  // at a constrained optimum (or exactly vertical)
    }

    const double h0 = std::abs(a);
    double lambda = std::min(o.nullspace_max_step, (1.0 - h0) / gnorm);
    bool accepted = false;
    double h_new = h0;
    for (int bt = 0; bt < 14 && lambda > 1e-7; ++bt) {
      std::vector<double> trial = *q;
      for (size_t i = 0; i < n; ++i) {
        trial[i] = std::min(std::max(trial[i] + lambda * gN(i) / gnorm, limits[i].first), limits[i].second);
      }
      // Newton retraction onto the primary-task manifold (half the tolerance
      // as the target so the accept test below has margin).
      for (int r = 0; r < 8; ++r) {
        Eigen::Matrix<double, 6, Eigen::Dynamic> Jr;
        Eigen::Matrix<double, 4, Eigen::Dynamic> Jt;
        Eigen::Vector4d et;
        TaskError er;
        BuildTask(chain, target, trial, &Jr, &Jt, &et, &er);
        if (er.position_error.norm() < 0.5 * o.position_tolerance &&
            std::abs(er.pitch_error) < 0.5 * o.pitch_tolerance) {
          break;
        }
        Eigen::VectorXd dq = detail::SolveDampedRightPseudoInverse(Jt, et, o.damping, nullptr);
        for (size_t i = 0; i < n; ++i) {
          trial[i] = std::min(std::max(trial[i] + dq(i), limits[i].first), limits[i].second);
        }
      }
      TaskError ef = ComputeTaskError(chain, target, trial);
      const double h = std::abs(LinkAlignment(link, trial));
      double step_inf = 0.0;
      for (size_t i = 0; i < n; ++i) step_inf = std::max(step_inf, std::abs(trial[i] - (*q)[i]));

      if (budgeted && used + step_inf > o.nullspace_max_displacement) {
        const double remaining = o.nullspace_max_displacement - used;
        if (remaining <= 1e-9 || step_inf <= 0.0) {
          return;  // budget spent
        }
        lambda *= 0.95 * remaining / step_inf;
        continue;
      }
      if (within_tolerance(ef) && h > h0 + 1e-9) {
        *q = trial;
        used += step_inf;
        h_new = h;
        accepted = true;
        break;
      }
      lambda *= 0.5;
    }
    if (!accepted || h_new - h0 < 1e-7) {
      break;
    }
  }
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

  // Evaluated once per restart candidate, right after SolveFromSeed
  // returns -- never inside SolveFromSeed's own Newton loop, since only a
  // Solve() call's FINAL joint_positions is ever executed/commanded;
  // transient iterates are never observed, so checking them would add
  // cost for no safety benefit. !have_collision_check short-circuits to
  // "always collision_free," so every existing caller (no obstacle
  // configured) sees byte-identical behavior to before this field existed.
  bool have_collision_check = options_.collision_obstacle != nullptr;
  auto NotColliding = [this, have_collision_check](const PositionPitchIkResult& r) {
    return !have_collision_check ||
           !ArmIntersectsBase(solver_, r.joint_positions, *options_.collision_obstacle,
                               options_.collision_capsule_radius,
                               options_.collision_first_checked_segment_index);
  };

  PositionPitchIkResult best = SolveFromSeed(target_pose, seed);
  best.collision_free = NotColliding(best);

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
    return std::abs(Eigen::Vector3d::UnitZ().dot(axis));  // vertical LINE: up or down both count
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
    for (int r = 0; r < options_.max_restarts && !(IsAcceptable(best) && best.collision_free); ++r) {
      std::vector<double> restart_seed(limits.size());
      for (size_t i = 0; i < limits.size(); ++i) {
        restart_seed[i] = dists[i](rng_);
      }
      PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
      candidate.collision_free = NotColliding(candidate);
      ++restarts_attempted;
      bool best_acceptable = IsAcceptable(best) && best.collision_free;
      bool candidate_acceptable = IsAcceptable(candidate) && candidate.collision_free;
      if (candidate_acceptable && (!best_acceptable ||
                                    Alignment(candidate.joint_positions) > Alignment(best.joint_positions))) {
        best = candidate;
      } else if (!candidate_acceptable && !best_acceptable) {
        // Neither clears the acceptable+collision-free bar yet. A
        // colliding candidate is never preferred over a non-colliding one
        // on residual grounds alone; only fall back to the old
        // residual-comparison policy when collision status is tied
        // (including the common "no obstacle configured" case, where both
        // are always collision_free and this is byte-identical to before).
        if (candidate.collision_free != best.collision_free) {
          if (candidate.collision_free) {
            best = candidate;
          }
        } else if (candidate.residual.head<3>().norm() < best.residual.head<3>().norm()) {
          best = candidate;
        }
      }
    }
  } else {
    // Same restart bookkeeping as InverseKinematicsSolver::Solve: count
    // every attempt made, not just whichever one produced the best result.
    for (int r = 0; r < options_.max_restarts && !(best.converged && best.collision_free); ++r) {
      std::vector<double> restart_seed(limits.size());
      for (size_t i = 0; i < limits.size(); ++i) {
        restart_seed[i] = dists[i](rng_);
      }
      PositionPitchIkResult candidate = SolveFromSeed(target_pose, restart_seed);
      candidate.collision_free = NotColliding(candidate);
      ++restarts_attempted;
      // Same rule as the have_nullspace_bias branch above: collision
      // status gates the old converged/residual comparison rather than
      // being compared on equal footing with it.
      if (candidate.collision_free != best.collision_free) {
        if (candidate.collision_free) {
          best = candidate;
        }
      } else if (candidate.converged ||
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

  const bool posture_task =
      options_.nullspace_secondary_link_solver != nullptr && options_.nullspace_gain != 0.0;
  if (posture_task &&
      options_.nullspace_secondary_link_solver->GetActuatedJointNames().size() > n) {
    throw std::invalid_argument(
        "PositionPitchIkSolver: nullspace_secondary_link_solver has more actuated joints than "
        "the chain -- it must be an upstream sub-chain of solver's");
  }

  Eigen::Matrix<double, 6, 1> residual = Eigen::Matrix<double, 6, 1>::Zero();
  int iterations = 0;
  bool converged = false;

  // Phase 1: primary task only (position + pitch), damped Newton with a
  // backtracking line search on the task cost.
  int refine_steps = 0;
  for (; iterations < options_.max_iterations; ++iterations) {
    TaskError e = ComputeTaskError(solver_, target_pose, q);
    residual.head<3>() = e.position_error;
    residual.tail<3>() = e.rotation_error;  // full 3-component rotation error, informational (see header)

    if (e.position_error.norm() < options_.position_tolerance &&
        std::abs(e.pitch_error) < options_.pitch_tolerance) {
      // Inside the acceptance tolerance. Keep refining for a few cheap Newton
      // steps down to kRefineFraction of it: stopping at the first iterate
      // inside a loose tolerance (as path-tracking samples use) leaves each
      // sample off by up to tolerance/|J|, and that offset jumps from sample to
      // sample, giving a finite-differenced joint velocity that does not shrink
      // when the duration is stretched (trajectory_generation.md, Finding 7).
      if (refine_steps >= kRefineIterations ||
          (e.position_error.norm() < kRefineFraction * options_.position_tolerance &&
           std::abs(e.pitch_error) < kRefineFraction * options_.pitch_tolerance)) {
        break;
      }
      ++refine_steps;
    }

    Eigen::Matrix<double, 6, Eigen::Dynamic> J = ComputeGeometricJacobian(solver_, q);
    Eigen::Matrix<double, 4, Eigen::Dynamic> J_task(4, n);
    J_task.topRows<3>() = J.topRows<3>();
    J_task.row(3) = e.pitch_axis.transpose() * J.bottomRows<3>();
    Eigen::Vector4d e_task(e.position_error.x(), e.position_error.y(), e.position_error.z(),
                            e.pitch_error);

    Eigen::VectorXd dq = detail::SolveDampedRightPseudoInverse(J_task, e_task, options_.damping, nullptr);
    double step_norm = dq.lpNorm<Eigen::Infinity>();
    if (step_norm > options_.max_step) {
      dq *= (options_.max_step / step_norm);
    }

    // Only take a step that actually reduces ||e_task||^2, halving it
    // otherwise (see max_inner_retries for the real bug this fixes).
    double cost = e_task.squaredNorm();
    bool step_accepted = TryBacktrackingStep(
        solver_, target_pose, limits, options_.max_inner_retries, dq, &q,
        [cost](const TaskError& e_candidate) {
          Eigen::Vector4d e_task_candidate(e_candidate.position_error.x(), e_candidate.position_error.y(),
                                            e_candidate.position_error.z(), e_candidate.pitch_error);
          return e_task_candidate.squaredNorm() < cost;
        });
    if (!step_accepted) {
      // A genuine local minimum or an unreachable target: more outer
      // iterations alone would not fix it.
      break;
    }
  }

  // Phase 2: posture optimisation along the self-motion manifold, only from
  // a primary-converged point so the secondary task can never be what fails
  // the primary one.
  {
    TaskError e = ComputeTaskError(solver_, target_pose, q);
    if (posture_task && e.position_error.norm() < options_.position_tolerance &&
        std::abs(e.pitch_error) < options_.pitch_tolerance) {
      OptimizePosture(solver_, options_, target_pose, &q);
    }
  }

  // `residual`/`converged` come from the TRUE final state, not from the last
  // loop iteration (which is one step stale).
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
