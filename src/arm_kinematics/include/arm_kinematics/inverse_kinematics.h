#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "arm_kinematics/forward_kinematics.h"

namespace arm_kinematics {

struct IkResult {
  bool converged;
  std::vector<double> joint_positions;  // best iterate found, always populated
  Eigen::Matrix<double, 6, 1> residual;  // [position(m) ; orientation(rad, axis-angle)] at the
                                          // final iterate -- unweighted, physical units,
                                          // regardless of Options::weights
  int iterations;      // of the attempt that produced this result (not summed across restarts)
  int restarts_used;   // 0 if the given seed converged directly
};

// Deliberately a free struct, not nested in InverseKinematicsSolver:
// giving it a default-valued Options parameter (Options() as a default
// argument) needs Options to already be a fully complete type, which a
// class nested in InverseKinematicsSolver isn't yet while
// InverseKinematicsSolver itself is still being parsed.
struct IkOptions {
  double position_tolerance = 1e-4;      // meters; heuristic, matches M1's validation scale
  double orientation_tolerance = 1e-3;   // radians; heuristic, matches M1's validation scale
  int max_iterations = 100;              // heuristic; outer (accepted-or-abandoned-step) budget
  double initial_damping = 0.05;         // lambda_0; heuristic starting point, adapted per-step below
  double damping_min = 1e-6;
  double damping_max = 1e6;
  double damping_decrease_factor = 0.5;  // on an accepted (cost-reducing) step
  double damping_increase_factor = 3.0;  // on a rejected step
  int max_inner_retries = 10;            // damping re-tries per outer iteration before giving up
  double max_step = 0.2;                 // radians; per-attempt inf-norm clamp on the joint update
  Eigen::Matrix<double, 6, 1> weights = Eigen::Matrix<double, 6, 1>::Ones();

  // This chain is redundant (4 parallel "elbow" joints + 1 yaw -- see
  // ROADMAP.md M2 section): reachable targets can live in a solution
  // basin the given seed's local descent never finds, independent of how
  // well-tuned the damping is (confirmed empirically: adaptive damping
  // alone still failed ~65% of random reachable targets from a fixed
  // zero seed; random restarts fixed all of them, typically within 0-2).
  // If the primary seed doesn't converge, retry from up to this many
  // random seeds within joint limits, keeping the best result found.
  // Each attempt is cheap (~tens of ms on this chain), and the benchmark
  // above sometimes needs more than 10 for a harder target/seed
  // combination, so this is generously padded above the typical 0-4.
  int max_restarts = 20;
  unsigned rng_seed = std::random_device{}();  // override for reproducible restarts (e.g. tests)
};

// Damped least-squares (Levenberg-Marquardt) numerical IK over a
// ForwardKinematicsSolver's actuated joints, with adaptive damping (a step
// is only accepted if it actually reduces the weighted error; rejecting
// increases lambda, accepting decreases it) plus random-seeded restarts
// on non-convergence (IkOptions::max_restarts).
//
// Both matter, for different reasons, confirmed empirically on this
// chain's 20-random-reachable-target benchmark: a fixed lambda converged
// only 40% of the time (single lambda can't serve both the
// far-from-target and near-convergence regimes well); adaptive lambda
// alone converged *worse*, ~35% (better per-step quality doesn't help if
// the seed's basin of attraction just doesn't contain the solution --
// this chain is redundant, 4 parallel "elbow" joints + 1 yaw, so distinct
// solution branches genuinely exist); adaptive lambda + restarts reached
// 100%, typically within 0-2 restarts. Restarts fix *which basin*;
// adaptive damping fixes *how well a basin is descended* -- neither
// substitutes for the other.
//
// See docs/inverse_kinematics.md for the full derivation; short version:
// task space here is 6D (position+orientation) over 5 joints, so the
// Jacobian has more rows than columns (overdetermined) -- this uses the
// corresponding *left* pseudo-inverse / normal-equations form, not the
// redundant-manipulator (right pseudo-inverse) form.
class InverseKinematicsSolver {
 public:
  // `solver` must outlive this object -- stored by reference, not copied,
  // since it's expected to be a long-lived chain description shared across
  // many Solve() calls (e.g. from a ROS node).
  explicit InverseKinematicsSolver(const ForwardKinematicsSolver& solver,
                                    IkOptions options = IkOptions());

  // Iterates from `seed` (must match solver's actuated-joint count) toward
  // `target_pose` (in solver's root_link). If `seed` doesn't converge,
  // retries from up to IkOptions::max_restarts random seeds within joint
  // limits, keeping the best (lowest-residual, preferring converged)
  // result. Always returns a result -- `converged` is false, not an
  // exception, when every attempt is exhausted without both tolerances
  // met; `joint_positions`/`residual` are still the best iterate found in
  // that case, clamped to the solver's joint limits throughout.
  // Throws std::invalid_argument if `seed`'s size doesn't match the
  // solver's actuated-joint count.
  IkResult Solve(const Eigen::Isometry3d& target_pose, const std::vector<double>& seed) const;

 private:
  IkResult SolveFromSeed(const Eigen::Isometry3d& target_pose,
                          const std::vector<double>& seed) const;

  const ForwardKinematicsSolver& solver_;
  IkOptions options_;
  mutable std::mt19937 rng_;
};

}  // namespace arm_kinematics
