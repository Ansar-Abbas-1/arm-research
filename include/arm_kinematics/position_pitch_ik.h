#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_kinematics/forward_kinematics.h"

namespace arm_kinematics {

struct PositionPitchIkResult {
  // Position + pitch within tolerance (roll/yaw are never part of
  // convergence) -- EXCEPT: when Solve() (not SolveFromSeed) is used with
  // a null-space secondary task configured, this reflects a slightly
  // looser, deliberate bar instead: "within kDriftCapMultiplier x
  // tolerance, which is what Solve()'s own restart/selection logic used
  // to pick this result as the best available." That's a genuine
  // trade-off, not an oversight -- see docs/position_pitch_ik.md and
  // Solve()'s own comment for the real bug (a good, well-aligned near-seed
  // result being discarded in favor of a random, poorly-aligned but
  // "exactly converged" one) this exists to avoid.
  bool converged;
  std::vector<double> joint_positions;  // best iterate found, always populated
  Eigen::Matrix<double, 6, 1> residual;  // [position(m) ; orientation(rad, axis-angle)] at the
                                          // final iterate, same layout as InverseKinematicsSolver's
                                          // IkResult -- unlike that solver, only the position (3)
                                          // and pitch-axis-projected component of orientation (1 of
                                          // the 3 rotation rows) are DRIVEN toward zero; the other
                                          // two rotation components are reported for information
                                          // only (what roll/yaw the null-space bias, or the
                                          // minimum-norm solution if no bias, happened to settle at)
  int iterations;
  int restarts_used;
};

struct PositionPitchIkOptions {
  double position_tolerance = 1e-4;  // meters; same scale as InverseKinematicsSolver's default
  double pitch_tolerance = 1e-3;     // radians; same scale as InverseKinematicsSolver's default
  // Higher than InverseKinematicsSolver's 100: a primary-only solve here
  // still exits in ~10-20 iterations regardless (position+pitch is an
  // easier target than a full 6D pose, see the class comment), so this
  // only matters when a null-space secondary task is configured, where
  // the extra budget is needed -- see nullspace_tolerance below.
  int max_iterations = 200;
  // Chiaverini variable-damping threshold (see resolved_rate.h/.md) --
  // this chain is redundant for the 4D task here (5 joints, 4 rows), the
  // opposite situation from InverseKinematicsSolver's overdetermined 6D
  // case, so it reuses resolved_rate's damping/null-space math (shared
  // via task_space_math.h) for the step DIRECTION. Chiaverini damping only
  // conditions the step near a singularity, though -- it says nothing
  // about whether a step is actually a good idea, which is what
  // max_inner_retries (below) is for.
  double damping = 0.05;
  double max_step = 0.2;  // radians; per-iteration inf-norm clamp on the joint update, safety only

  // Backtracking line search on the primary (position+pitch) task cost: a
  // step is only taken if it actually reduces ||e_task||^2, otherwise it's
  // halved and retried, up to this many times, before giving up on the
  // iteration entirely (breaking out of the outer loop rather than
  // continuing to burn iterations). Added after a real bug, found via
  // arm_trajectory's Cartesian-path tracking: on a target this solver
  // couldn't actually reach (an interior waypoint off the reachable
  // submanifold -- expected, see docs/position_pitch_ik.md), it kept
  // taking full max_step-clamped steps every iteration regardless of
  // whether they helped, for the entire max_iterations budget -- unlike
  // InverseKinematicsSolver, which never took a non-improving step in the
  // first place (its own Levenberg-Marquardt accept/reject). Measured
  // consequence: over 20 iterations with no cost check, joint values
  // wandered 2-3.5 rad away from a good warm-started seed even though the
  // seed was already close to the best this solver could do -- versus
  // InverseKinematicsSolver's <0.03 rad drift in the same stuck scenario.
  // This is a backtracking LINE SEARCH on the already-damped step
  // direction, not InverseKinematicsSolver's lambda-escalation LM scheme:
  // Chiaverini's damping already serves the conditioning role LM's lambda
  // would, so only the step LENGTH needs a fallback here, not the
  // direction.
  //
  // Only engaged when no null-space secondary task is configured: a
  // configured bias mixes secondary-task work into every step from the
  // very first iteration (not just after primary convergence), and its
  // leakage can transiently raise the primary cost well before the
  // primary task has actually converged -- a cost-based rejection can't
  // tell that apart from genuine divergence, and rejecting it would
  // starve the secondary task exactly the way the OTHER documented fix in
  // this file (see nullspace_tolerance) exists to prevent. The bug this
  // option fixes was itself only ever observed with no secondary task
  // active, so gating it that way costs nothing.
  int max_inner_retries = 5;

  // Same restart rationale as InverseKinematicsSolver (see that class's
  // comment and docs/inverse_kinematics.md): this chain has distinct
  // solution basins a single seed's local descent won't always find,
  // independent of how well the per-step math behaves.
  int max_restarts = 20;
  unsigned rng_seed = std::random_device{}();

  // Optional null-space secondary task (see resolved_rate.h/.md), applied
  // at every Newton step so the roll/yaw left free by only targeting
  // position+pitch settle at a consistent, configured posture instead of
  // an arbitrary seed-dependent one. `nullspace_gain = 0` (the default,
  // matching ComputeResolvedRateVelocity's own default) disables this
  // entirely. Unlike ComputeResolvedRateVelocity (which takes these
  // per-call, since a live streaming caller may want to change them
  // tick-to-tick), they're construction-time options here: this solver
  // is meant to be configured once with whatever secondary posture policy
  // a caller wants applied to every Solve(), not opted into per call.
  //
  // WARNING, confirmed the hard way: this gain is NOT the same quantity
  // as ComputeResolvedRateVelocity's nullspace_gain, even though it feeds
  // the identical formula. There, it's a rad/s rate the caller later
  // multiplies by dt (~0.02s at 50Hz) before it becomes a position
  // change; here, with no per-iteration dt to speak of, it's added
  // directly as a per-Newton-step radians contribution -- ~50x larger in
  // effect for the same number. Reusing resolved-rate's own default
  // (0.4) here made a plainly-convergeable target fail to converge at
  // all; 0.05 converged the same target cleanly. See docs/position_pitch_ik.md.
  const ForwardKinematicsSolver* nullspace_secondary_link_solver = nullptr;
  double nullspace_gain = 0.0;

  // Real bug, confirmed empirically and fixed: the iteration loop used to
  // stop the instant position+pitch converged, giving the null-space bias
  // exactly as many iterations as the PRIMARY task happened to need --
  // typically far too few for the bias's own, much slower, per-step-
  // limited convergence to get anywhere close to its fixed point (a
  // 100-target sweep isolating restart randomness showed the bias always
  // improved alignment relative to no bias at all, so its direction was
  // never wrong, but still landed worse than the seed's own starting
  // alignment in the majority of cases simply because the loop stopped
  // too early). Now the loop keeps running -- at no extra cost to the
  // primary task, whose own contribution is already ~0 once converged --
  // until the null-space bias vector's own norm drops below this
  // tolerance too, or max_iterations runs out. `converged` still reflects
  // only the primary task (see IkResult), regardless of whether the
  // secondary task got there.
  //
  // Honest limitation, not hidden: at the safe default gain (0.05
  // above), this is a slow, linear-rate fixed-point iteration -- a
  // 40-target sweep (isolating restart randomness) after this fix still
  // exhausted max_iterations without reaching this tolerance in most
  // cases, though every one of them still measurably improved over no
  // bias at all, often substantially (some by 0.5+ in alignment score).
  // Update after max_inner_retries' backtracking fix was added: that fix
  // only touches the NO-bias convergence path (see its own comment), but
  // that's exactly the baseline these sweeps compared against -- a
  // redundant task's final basin is path-dependent, so changing the
  // no-bias path's own trajectory can change which basin IT lands in,
  // independent of anything about the bias itself. Re-measured (8-target
  // sweep, same isolation methodology): bias no longer wins every single
  // comparison (5/8, not 8/8) -- NullspaceBiasImprovesVerticalAlignmentAtConvergence
  // was updated to assert "wins the majority," not "wins always," to
  // match. The bias's own math and behavior are unchanged; only the
  // no-bias baseline it's compared against moved.
  //
  // A stronger gain converges faster but needs more restarts (a larger
  // per-step secondary contribution leaks more through the damped, not
  // exact, null-space projection and can occasionally knock the primary
  // task off its own convergence path) -- a real trade-off, not a single
  // better number, so left as a tunable rather than "fixed" further. See
  // docs/position_pitch_ik.md.
  double nullspace_tolerance = 1e-3;
};

// Reduced-task IK: solves for position (3) + pitch (1, about the CURRENT
// iterate's own local Y axis, exactly ComputeResolvedRateVelocity's own
// convention -- see resolved_rate.h) only, leaving roll and yaw
// unconstrained. Exists because InverseKinematicsSolver's full 6D pose
// task is OVERDETERMINED on this 5-joint chain (6 task rows > 5 joints,
// generically full column rank away from singularities => trivial null
// space => nothing for a secondary objective to bias into, and no
// orientation freedom to leave unconstrained even if a caller doesn't
// care about roll/yaw). Dropping roll/yaw from the task -- the same
// choice ComputeResolvedRateVelocity already made for velocity control,
// now applied to the iterative position solve -- makes the task 4D over
// 5 joints, genuinely redundant, with a real null space the optional
// secondary task (below) can use.
//
// See docs/resolved_rate.md for the shared damping/null-space derivation
// and docs/inverse_kinematics.md for why InverseKinematicsSolver itself
// can't do this. Full derivation of this solver's iteration is in
// docs/position_pitch_ik.md.
class PositionPitchIkSolver {
 public:
  // `solver` must outlive this object -- stored by reference, not copied,
  // matching InverseKinematicsSolver's own convention.
  explicit PositionPitchIkSolver(const ForwardKinematicsSolver& solver,
                                  PositionPitchIkOptions options = PositionPitchIkOptions());

  // Iterates from `seed` (must match solver's actuated-joint count) toward
  // matching `target_pose`'s position and pitch-about-target's-own-local-Y
  // component only; `target_pose`'s roll/yaw are read (to report residual
  // against, informationally) but never enforced. If `seed` doesn't
  // converge, retries from up to Options::max_restarts random seeds
  // within joint limits, keeping the best (lowest position+pitch
  // residual, preferring converged) result -- same policy as
  // InverseKinematicsSolver::Solve. Throws std::invalid_argument if
  // `seed`'s size doesn't match the solver's actuated-joint count, or if
  // a configured secondary_link_solver has more actuated joints than the
  // chain (it must be an upstream sub-chain).
  PositionPitchIkResult Solve(const Eigen::Isometry3d& target_pose,
                               const std::vector<double>& seed) const;

 private:
  PositionPitchIkResult SolveFromSeed(const Eigen::Isometry3d& target_pose,
                                       const std::vector<double>& seed) const;

  const ForwardKinematicsSolver& solver_;
  PositionPitchIkOptions options_;
  mutable std::mt19937 rng_;
};

}  // namespace arm_kinematics
