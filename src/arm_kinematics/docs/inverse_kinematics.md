# Inverse kinematics — derivation

Solves for the 5 actuated joint angles (`joint_1`..`joint_5`) that achieve a
commanded `end_effector` pose in `base_link` (default `tip_link`; the math
below is derived generically against whatever `ForwardKinematicsSolver`
was built for), built on top of
[`forward_kinematics.md`](forward_kinematics.md)'s `ForwardKinematicsSolver`.
`end_effector` sits at a fixed, gripper-angle-independent offset from
`link_5` — see forward_kinematics.md's "Extending to end_effector" for why
that required a URDF fix (`joint_6` was wrongly modeled as revolute) and
why it needed no changes to the solver itself.

## Task-space dimension vs. DOF: why this is a least-squares problem

Task space is 6D (3 position + 3 orientation) and there are 5 joints, so the
Jacobian `J` is 6×5. The relevant number is its **rank, which is 4 at every
configuration** (not 5), for a structural reason (derived in full in
[`kinematics_report.md`](kinematics_report.md) §2.2): joint_1 is a `z` rotation
and joints 2–5 share one horizontal axis `â = Rz(q₁)ŷ`, so in the cylindrical
basis `(ê_r, ê_θ, ẑ)` the end-effector twist always satisfies two linear
constraints, `ω_r = 0` (roll rate identically zero) and `v_θ = r·ω_z` (yaw
rate slaved to the tangential velocity). Two of the six rows are therefore
dependent on the others:

- **The reachable poses form a 4-dimensional submanifold of SE(3)**,
  `{(p, Rz(atan2(p_y,p_x)[+π])·Ry(φ))}`: roll is 0 and yaw is determined by
  the position. An arbitrary 6D target is generically *off* that manifold, so
  no exact solution exists (consistent with the M2 finding that the arm
  reaches position + one in-plane angle, pitch).
- **`J` still has a 1-dimensional null space** (5 columns, rank 4): the planar
  self-motion of the four parallel elbow joints that keeps the tip position
  and pitch fixed (null vector: zero `q₁` component, `Σ q̇₂..₅ = 0`). So the
  chain is *both* overdetermined for a generic 6D target (6 rows, only 4 of
  them independent) *and* redundant for a reachable one (4 independent rows,
  5 joints). The earlier version of this section called it purely
  overdetermined with a trivial null space; that was wrong (verified
  numerically: `σ₅(J) ≈ 1e-16…1e-19` at random in-limit configurations).

For the full-pose solve the right tool is ordinary least squares with
Levenberg-Marquardt damping (the **left** / normal-equations form). By the
push-through identity `(JᵀJ+λ²I)⁻¹Jᵀ = Jᵀ(JJᵀ+λ²I)⁻¹` this is the same
operator as the right-pseudo-inverse form used by `resolved_rate`/
`position_pitch_ik`; the two differ in which task rows they are given, not in
the algebra. The damping is what keeps the normal matrix `JᵀWJ+λ²I`
invertible despite the structural rank deficiency:

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
sub-chain like that has a 1-parameter family (the self-motion curve, e.g.
parametrised by the angle of link_4) of joint configurations reaching the
same end-effector pose, split into several discrete branches (elbow up /
down, and the two base branches `q₁` / `q₁+π`) — genuinely separate solution
basins — no
amount of better-tuned local descent from one fixed seed reaches a
solution in a *different* basin.

Standard fix, and what's implemented: if the given seed doesn't converge,
retry from up to `IkOptions::max_restarts` (default 20) random seeds
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
| max restarts | 20 | empirically, 0-4 needed in the benchmark above; generous margin |

## Not covered here

Cartesian-space trajectory following (interpolating a sequence of IK
solves smoothly, respecting velocity limits) is M3's problem, not this
milestone's — `Solve()` here is a one-shot pose-to-joints map with no
notion of continuity between calls beyond whatever seed the caller
chooses to pass.
