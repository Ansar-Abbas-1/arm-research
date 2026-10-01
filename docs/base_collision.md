# Base-collision avoidance — derivation

Builds on [`position_pitch_ik.md`](position_pitch_ik.md) (the restart/selection machinery this reuses) and `arm_trajectory/docs/trajectory_generation.md` (how this threads through Cartesian trajectory generation).

## Why this exists

M4's original scope (reach + teleop) shipped without any check that the arm's own links don't pass through its mounting base. `base_collision.h`/`.cpp` model the base as a swappable `BoxObstacle` (axis-aligned in its own frame) and the arm's links as capsules (segments between consecutive actuated-joint origins, plus the final origin-to-tip segment), and provide an exact, closed-form, no-iteration collision test between them.

## `CapsuleIntersectsBox`: minimizing a convex piecewise-quadratic along a segment

Transform both segment endpoints into the box's own frame (`box_to_root⁻¹`), where the box is a plain `[-h,h]³` — this avoids needing general OBB–OBB machinery for what is, in that frame, an AABB-vs-segment problem. The squared distance from a point `x` to the box is separable per axis:

```
d²(x) = Σᵢ max(0, |xᵢ| − hᵢ)²
```

Along the segment `x(t) = q0 + t·u`, `t ∈ [0,1]`, each axis's contribution `max(0, |xᵢ(t)| − hᵢ)²` is:
- `0` while `xᵢ(t)` is within `[-hᵢ, hᵢ]` (axis "inactive"),
- `(xᵢ(t) − sign·hᵢ)²` — a quadratic in `t` — once it overshoots past `±hᵢ` ("active"),

and these two pieces match in **value and derivative** at the junction (both are 0 there), so each axis's own contribution is `C¹` and convex in `t`. The sum over 3 axes is therefore also convex and piecewise-quadratic in `t`, with breakpoints exactly where some axis's active/inactive status changes — i.e. where `xᵢ(t) = ±hᵢ`, at most 2 per axis (6 interior breakpoints), plus the two segment endpoints `t=0,1`.

Between consecutive breakpoints, which axes are active (and their sign) is constant, so `g(t) = d²(x(t))` is a single quadratic `a·t² + b·t + c` there, built directly from the active axes' own `(d0 + t·uᵢ)²` terms. The minimum over that sub-interval is either an endpoint or, if `a > 0`, the quadratic's own unconstrained vertex `t* = -b/2a` when it falls strictly inside. Evaluating every sub-interval's candidate set and taking the global minimum gives the exact minimum squared distance from the segment to the box — no iteration, no tolerance parameter. `CapsuleIntersectsBox` returns `d²_min ≤ radius²`.

**Multi-axis case**: when two or three axes are simultaneously active (the segment passes near a box edge or corner), the quadratic's `a`, `b`, `c` simply sum each active axis's own contribution — verified by hand in `test_base_collision.cpp`'s `CornerCaseWithTwoActiveAxes` (segment at `x=2,y=2`, both overshooting a unit cube by 1 each: `d² = 1² + 1² = 2`, independent of the third axis, confirmed against the closed form).

## `ArmIntersectsBase`: reusing FK's existing accessors

Builds segments from `ForwardKinematicsSolver::ComputeActuatedJointFrames()`'s `N` origins plus `ComputePose()`'s tip — `N+1` points, `N` segments (indices `0..N-1`). Two separate chain walks, since `ComputeChainState` (which would give both frames and the final pose from one walk) is private to `ForwardKinematicsSolver` — a real but small inefficiency, not worth exposing that private combined accessor for.

## Why `first_checked_segment_index = 1` is a provable optimization, not a correctness requirement

Verified by hand against `arm.urdf`'s real joint origins: `joint_1`'s own origin is at `(0,0,0.03)`, `joint_2`'s at `(0,0,0.095)`. Both lie on the world Z axis — which is exactly `joint_1`'s own rotation axis — so **both points are invariant to every joint angle**, for any `q`.

The deployed box placement (see `trajectory_service_node.cpp`) has the robot sitting **on top of** the cube: `base_link`'s own origin is at the box's top face (`z=0` in root frame), with the box extending downward (`z ∈ [-0.40, 0]`), centered in X/Y. Since `frame[0].z = 0.03 > 0` and `frame[1].z = 0.095 > 0`, and the straight segment between them stays within `z ∈ [0.03, 0.095]`, segment 0 (≈ `link_1`) **never dips into the box's `z ≤ 0` region, for any joint configuration, by construction**. Skipping it is a pure optimization.

This is NOT true of segment 1 (`frame[1]→frame[2]`, ≈ `link_2`): `frame[2]` depends on `q1, q2`, and `link_2`'s own ~0.34m length is easily enough to swing it down into the box's region for plausible elbow configurations — confirmed directly (see "Live-verified configurations" below). Segment 1 is checked by default, same as segments 2 onward.

**If `half_extents`/`box_to_root` ever change** (the explicit "swappable later" requirement this design satisfies), this default must be re-derived — it is a structural fact about *this specific placement*, not a general property of the chain, and the header comment says so explicitly rather than leaving it as an unexplained magic number.

### An earlier, wrong placement assumption — and why it mattered

The first version of this plan assumed the box was centered on `base_link`'s origin in **all three axes** (straddling `z ∈ [-0.20, 0.20]`), based on an early open-question answer. Under that placement, `frame[0]`/`frame[1]` (both at small positive `z`, well within `±0.20`) would have been **permanently inside the box, for every joint angle** — making `first_checked_segment_index=2` a *correctness requirement* (checking segment 0 would make the collision check permanently, unsatisfiably "colliding" regardless of `q`), not an optimization. The user corrected this mid-review: the robot is centered on the cube in X/Y but sits **on top of** it in Z. The corrected placement reverses the conclusion — segment 0 is always *outside*, not always inside — which is what let the default change from a load-bearing `2` to an optimization-only `1`. Recorded here since the "always just above" fact is easy to get backwards if the box placement is ever revisited without re-deriving it.

## Capsule radius

`arm.urdf`'s link radii: `link_1`–`link_5` all `0.01`, `end_effector` `0.02`. Default `0.02` (the max over every checked segment) — one flat, conservative value, trading a negligible (1cm) extra margin on the thinner links for not having to plumb per-link geometry into the check.

## Live-verified configurations

Confirmed via a standalone probe compiled directly against the built `libarm_kinematics.so` (this project's established verify-before-asserting discipline — see `test_base_collision.cpp`'s own comments) before any test value was hand-asserted:

- `q = (0, -1.6, 0, 0, 0)`: `frame[2] = (0.0097, 0, -0.2449)` — inside the box's X/Y footprint and well within its Z range. `ArmIntersectsBase` → `true` (segment 1).
- `q = (0.01, 0.5, -0.5, 0.5, -0.5)` and a normal outstretched config (`q2=π/2`-ish): both `false`, confirming the near-zero-angle regression guard.
- A sweep over `q2 ∈ [-1.6, 1.6], q3 ∈ [-2.5, 2.5]` (q1=q4=q5=0) found a wide band of colliding configurations — collision is common, not a corner case, for this arm/box geometry, which is exactly why avoidance (not just detection) matters for M4.

## Collision in `PositionPitchIkSolver::Solve()`: reusing the existing restart machinery, not a parallel path

`collision_obstacle`/`collision_capsule_radius`/`collision_first_checked_segment_index` are opt-in `PositionPitchIkOptions` fields (`nullptr` = off, same convention as `nullspace_secondary_link_solver`). Checked **once per restart candidate**, in `Solve()`, immediately after each `SolveFromSeed` call returns — never inside `SolveFromSeed`'s own Newton loop, since only a `Solve()` call's *final* `joint_positions` is ever executed or commanded; transient iterates are never observed, so checking them would add cost for no safety benefit.

Both of `Solve()`'s restart branches (`have_nullspace_bias` true/false) gate their loop condition on `collision_free` in addition to their existing acceptability bar, and extend candidate-replacement with collision as an **additional, higher-priority tier**: a colliding candidate is never preferred over a non-colliding one on residual/convergence grounds alone — mirroring the exact pattern `position_pitch_ik.md`'s Bug 4 fix already established for the primary acceptable/non-acceptable gate (a qualifying candidate always wins over a non-qualifying one; residual/alignment only break ties *within* a tier). `converged` keeps its existing, collision-unaware meaning; `collision_free` is a separate signal a caller must check in addition.

### A real consequence of "qualifying-tier-dominates", found by testing the actual implementation

Before implementing, the plan's own pseudocode assumed an infeasible target (one only reachable by colliding) would exhaust its restart budget and report `collision_free=false` — "best effort, but still colliding." **Measured behavior is different**, and was caught specifically by testing it rather than trusting the a-priori assumption: for a target whose own position sits inside the box (so *every* converging configuration necessarily collides), the qualifying-tier-dominates policy means **any** collision-free candidate — even a wildly unconverged, poorly-aligned one from a random restart — beats a perfectly-converged-but-colliding exact match. Confirmed deterministically across 5 rng seeds at 50 restarts each (`test_position_pitch_ik.cpp`'s `InfeasibleTargetPrefersCollisionFreeOverExactCollidingMatch`): the result is always `collision_free=true, converged=false`, never `collision_free=false`. This is a deliberate, safety-favoring consequence of the design (never silently accept a colliding "solution" just because it happens to be numerically exact), not a bug — but it does mean **a caller must check `converged`, not `collision_free`, to detect "this target could not be reached without colliding."**

### Restart budget: collision avoidance can need noticeably more restarts than plain convergence

Live-measured (standalone probes against the built library, then confirmed through the actual `/generate_trajectory` service): a goal reachable only via an alternate elbow configuration took **10 restarts** to find collision-free with no null-space bias active, but **34 restarts** with a null-space bias also configured (the bias competing for the same restart search makes a collision-free *and* well-aligned candidate rarer). `trajectory_service_node`'s own default (`max_restarts=20`, library default) was **not always enough** for the biased case in live testing — occasionally reporting an honest "did not converge" rather than finding the reroute. This is expected, inherent stochastic restart-search behavior (see `position_pitch_ik.md`'s own restart-budget tradeoffs), not a defect: the system never silently accepts a colliding pose, it just sometimes needs a larger `~max_restarts` (a ROS param) to find an existing reroute when a null-space bias is also active. Not treated as a bug to fix here, since a caller that cares can simply raise the budget — but worth knowing before assuming every rerouteable target reroutes at default settings.

### A pose everyone in this session had been using turned out to be inside the box

The session's own recurring test pose, `position=(0.15, 0.1, -0.2)`, sits **literally inside** the deployed 40cm box (`|x|,|y| < 0.20`, `z ∈ [-0.40, 0]`) — confirmed by direct arithmetic and by the live `/generate_trajectory` call failing to converge at all (not a collision-specific message — the target is simply unreachable without colliding, see the "qualifying-tier-dominates" note above) once collision checking went live. This isn't a regression in the new code; it's the collision checker correctly identifying that a pose used throughout earlier, pre-M4 testing happens to sit inside the very region M4 now protects. Regression verification for this change used a different pose (`z=+0.3` instead of `z=-0.2`, otherwise identical) to confirm the rest of the pipeline is unaffected.
