# Inverse kinematics — derivation

Solves for the 5 actuated joint angles (`joint_1`..`joint_5`) that achieve a
commanded `link_5` pose in `base_link`, built on top of
[`forward_kinematics.md`](forward_kinematics.md)'s `ForwardKinematicsSolver`.

## Task-space dimension vs. DOF: why this is a least-squares problem

Task space is 6D (3 position + 3 orientation). There are only 5 joints. So
the Jacobian `J` is 6×5 — **more rows than columns**. This is the opposite
situation from a classically "redundant manipulator" (more joints than task
dimensions, e.g. a 7-DOF arm reaching a 6D pose), which is what the
right-pseudo-inverse damped-least-squares formula (`J^T(JJ^T+λ²I)⁻¹`) is
for. Here the system is **overdetermined**: generically no exact solution
exists for an arbitrary target (consistent with the ROADMAP finding that
this arm can't reach a general 3D orientation, only position + one in-plane
angle). The right tool is ordinary least squares with Levenberg-Marquardt
damping, i.e. the **left** pseudo-inverse / normal-equations form:

```
minimize  ||J·Δq − e||² + λ²||Δq||²
   =>     Δq = (JᵀWJ + λ²I)⁻¹ · JᵀW·e
```

`W`: 6×6 diagonal task-space weight (identity by default — least squares
already produces the best-compromise residual on the components that
structurally can't be zeroed, no need to hand-weight orientation down).
`λ`: damping, adaptive (see below).

## Geometric Jacobian

For actuated joint *i* with axis-origin position `p_i` and axis direction
`z_i` (both in `base_link`, at the current configuration — see
`ForwardKinematicsSolver::ComputeActuatedJointFrames`), and end-effector
position `p_e`:

```
J_v,i = z_i × (p_e − p_i)     (linear velocity column)
J_ω,i = z_i                    (angular velocity column)
J = [J_v ; J_ω] ∈ R^(6×5)
```

Standard result for a serial revolute chain, using the *spatial* (base-
frame) Jacobian convention throughout (all `z_i`/`p_i` expressed in
`base_link`, matching how `ForwardKinematicsSolver` already accumulates
the chain).

**Independent cross-check** (`test/test_jacobian.cpp`): a central
finite-difference numerical Jacobian of `ComputePose` — not the analytical
formula above — computed via perturbing one joint at a time. For the
angular block: perturbing only joint *i* by ±h and linearizing,
`R(q+h eᵢ) ≈ Exp(h zᵢ) R(q)` and `R(q−h eᵢ) ≈ Exp(−h zᵢ) R(q)`, so
`R(q+h eᵢ)·R(q−h eᵢ)ᵀ ≈ Exp(2h zᵢ)`; its rotation-vector divided by `2h`
recovers `zᵢ` directly, independent of the analytical derivation. Matches
to ~1e-5 (finite-difference truncation error) at several configurations.

## Orientation error

`R_err = R_target · R_curᵀ`, converted to a rotation vector (axis·angle) via
`Eigen::AngleAxisd(R_err)`. Numerically safe at the identity: the axis
becomes ill-defined only where the angle is already 0, and their product
still → 0 there regardless of which axis is picked.

## Adaptive damping: why a fixed λ isn't enough

A step is only **accepted** if it actually reduces the weighted cost
`e^T W e`; a rejected step increases `λ` (falls back toward gradient
descent, small conservative steps) and retries, an accepted one decreases
`λ` (Gauss-Newton-like, faster) for the next step. This is standard
Levenberg-Marquardt, not a simplification — an earlier version used a
single fixed `λ` throughout and **converged only 40% of the time** on 20
random reachable targets from a fixed seed, because one damping value has
to serve both the far-from-target regime (wants aggressive steps) and the
near-convergence regime (wants small, careful ones).

## Random restarts: why adaptive damping alone still wasn't enough

Switching to adaptive damping *without* restarts actually converged
**worse** — ~35% — on the same benchmark. This is the tell that the
problem wasn't step quality: **this chain is redundant.** From
[`forward_kinematics.md`](forward_kinematics.md): joint_1 is a base yaw,
and joints 2-5 are four *parallel*-axis "elbow" joints. A planar 4R
sub-chain like that has multiple distinct joint configurations reaching
the same end-effector pose (the familiar "elbow up / elbow down" style
ambiguity, generalized), i.e. genuinely separate solution basins — no
amount of better-tuned local descent from one fixed seed reaches a
solution in a *different* basin.

Standard fix, and what's implemented: if the given seed doesn't converge,
retry from up to `IkOptions::max_restarts` (default 10) random seeds
within joint limits, keeping the best (lowest-residual, preferring
converged) result. On the same 20-target benchmark this reaches **100%**,
typically within 0-2 restarts, worst case 4. Adaptive damping and restarts
fix different things — restarts fix *which basin* is explored, damping
fixes *how well* a basin is descended — and the benchmark shows neither
one substitutes for the other (fixed-λ: 40%; adaptive-λ alone: 35%;
adaptive-λ + restarts: 100%).

`IkOptions::rng_seed` makes restart sequences reproducible when needed
(e.g. `test/test_inverse_kinematics.cpp` pins it); left at its default
(`std::random_device`-seeded) in production so repeated calls against a
genuinely stuck target get a fresh set of attempts each time.

## Heuristic constants (starting points, not derived)

| constant | value | note |
|---|---|---|
| position tolerance | 1e-4 m | matches M1's validation scale |
| orientation tolerance | 1e-3 rad | matches M1's validation scale |
| max iterations | 100 | per attempt; arbitrary, generous for a 5-joint chain |
| initial damping λ₀ | 0.05 | arbitrary starting point; adapted from there per-step |
| damping decrease/increase factor | 0.5 / 3.0 | standard LM defaults, not tuned specifically for this arm |
| max step/attempt | 0.2 rad (∞-norm clamp on Δq) | prevents a single step from overshooting wildly |
| max restarts | 10 | empirically, 0-4 needed in the benchmark above; generous margin |

## Not covered here

Cartesian-space trajectory following (interpolating a sequence of IK
solves smoothly, respecting velocity limits) is M3's problem, not this
milestone's — `Solve()` here is a one-shot pose-to-joints map with no
notion of continuity between calls beyond whatever seed the caller
chooses to pass.
