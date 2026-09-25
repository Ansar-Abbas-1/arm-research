# Trajectory generation — derivation and findings

Point-to-point trajectory generation for the 5-DOF arm, on top of `arm_kinematics`'s `ForwardKinematicsSolver`/`InverseKinematicsSolver` (M1/M2). Two goal types: joint-space (`GenerateJointSpaceTrajectory`) and Cartesian-path (`GenerateCartesianTrajectory`). Velocity-limited only — no acceleration/torque limits (URDF gives velocity limits reliably; `motors.yaml`'s inertia values were already flagged in M0 as inconsistent placeholders, not something to build safety-relevant math on).

## Quintic time-scaling

Standard smooth point-to-point profile: zero velocity **and** acceleration at both endpoints (avoids the velocity discontinuities of a naive trapezoidal profile). Normalized time `τ = t/T ∈ [0,1]`:

```
s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵            s(0)=0, s(1)=1
s'(τ)  = 30τ² − 60τ³ + 30τ⁴            s'(0)=s'(1)=0
s''(τ) = 60τ − 180τ² + 120τ³           s''(0)=s''(1)=0
q(t) = q₀ + (q_goal − q₀)·s(t/T)
```

`s'(τ)` is maximized at `τ=0.5`: `s'(0.5) = 30(0.25) − 60(0.125) + 30(0.0625) = 7.5 − 7.5 + 1.875 = 15/8`. Since `q̇(t) = Δq·s'(t/T)/T`, peak joint velocity is `|Δq|·(15/8)/T`. Setting equal to the joint's velocity limit `v_max` (`ForwardKinematicsSolver::GetActuatedJointVelocityLimits()`, URDF-sourced, same construction-time pattern as the position-limit accessor added in M1):

```
T_min,i = (15/8)·|Δq_i| / v_max,i
T = max_i(T_min,i)     — synchronized arrival, same T for every joint
```

**Independent cross-check** (`test/test_quintic_time_scaling.cpp`): `QuinticVelocity`/`QuinticAcceleration` checked against central finite-differences of `QuinticPosition`/`QuinticVelocity` respectively; the peak location and value (`τ=0.5`, `15/8`) confirmed by scanning rather than trusting the closed form alone; and a generated profile's *actual* numerically-scanned peak velocity confirmed to equal `v_max` exactly (not just `≤`), i.e. the formula is tight, not just conservative.

## Joint-space trajectories

Direct application of the above per joint, synchronized to a shared `T`. No IK involved — always exact, always fast.

## Cartesian-path trajectories

`pose(t)`: position lerp + orientation SLERP (`Eigen::Quaterniond::slerp`), using `s(t/T)` as the interpolation fraction — same timing profile as joint-space, applied in pose space. Each sample's joint solution is warm-started from the *previous* sample's — not optional on this redundant chain (M2 finding: 1 yaw + 4 parallel-axis "elbow" joints, distinct solution basins genuinely exist) — to avoid jumping between elbow configurations sample-to-sample.

Duration is seeded from the joint-space endpoint difference (`Δq_i = q_goal_i − q_start_i`, IK-solved once at the goal) — a heuristic, since the actual induced joint-space path along a Cartesian line isn't linear — then verified by finite-differencing consecutive samples' actual joint velocities and re-scaling `duration` upward (by the worst violation ratio, capped at `max_duration_iterations` attempts and a `kMaxReasonableSamples` sanity limit) until every joint stays within its velocity limit throughout.

### Finding 1: `{0,0,0,0,0}` is a genuine kinematic singularity

Confirmed via Jacobian SVD: smallest singular value at the all-zero configuration is `~1.5e-33` (machine zero) — a true rank-4, not rank-5, Jacobian. Structurally obvious in hindsight: joints 2, 3, 4, and 5 (the four parallel-axis "elbow" joints) each sit at *one of their own limit boundaries* simultaneously at `q=0` — the classic fully-extended-arm singularity. IK correctly can't make local progress from exactly on top of a singularity, however tiny the required correction; this isn't something to route around in the solver, it's a fact about the robot's geometry. Practical consequence: don't seed Cartesian trajectory generation (or general IK use) from `{0,0,0,0,0}` — pick any nearby, non-boundary configuration instead. Worth remembering for M4: if a joystick teleop's "home" position is the zero configuration, driving away from it needs a nudge off the singularity first.

### Finding 2: an interpolated Cartesian path is generally NOT exactly reachable, even between two exactly-reachable endpoints

This chain has 5 DOF over a 6D task space (M2 finding: it can't achieve arbitrary orientation independent of position). The reachable poses form a 5-dimensional submanifold of SE(3). `pose_start` and `pose_goal` sit exactly on that submanifold (they're `FK` of real joint configurations) — but **linear position + SLERP orientation interpolation between two points on a submanifold does not generally stay on it.** An interior waypoint is therefore usually only *approximately* reachable, with a residual that has no fixed upper bound and grows with distance into the interpolation.

Found empirically, not assumed: an interior sample's IK residual sat consistently just above the (already loosened) tolerance, and — the actual diagnostic signature — **every restart from every random seed converged to the same residual**, regardless of starting point. A real bug (bad seed, damping, local minimum) would vary with the seed; a structural best-fit doesn't. The residual also grew monotonically deeper into the path, exactly matching "distance from the submanifold" rather than "solver got unlucky."

**Consequence for the API (confirmed with the user):** interior-sample tracking is **best-effort**, not a hard requirement. Only the *goal* (the actual target the caller asked for) is required to be exactly reachable — checked once, strictly, before any sampling. Interior samples take whatever IK finds, however large the residual; `CartesianTrajectoryResult::peak_position_deviation`/`peak_orientation_deviation` report the worst one so a caller can judge whether the path was actually a good fit. This is standard behavior for reduced-DOF Cartesian motion planning generally, not a shortcut specific to this implementation.

### Finding 3: restarts must be disabled for interior samples — they break warm-starting, and that breaks the velocity-scaling loop

Once interior samples were made best-effort, a *different* failure appeared: some test runs hung, generating an exploding number of samples. Cause: interior samples were still solved via `InverseKinematicsSolver::Solve()` (the restart-enabled entry point). When the warm-started attempt doesn't hit tolerance — expected now, per Finding 2 — `Solve()`'s restart logic tries seeds from *anywhere* in joint space, and accepts one if its residual happens to be marginally lower, **even if it lands in a completely different elbow configuration.** That silently destroys the warm-start continuity the whole scheme depends on, producing a huge finite-differenced joint velocity between consecutive samples — which then feeds `GenerateCartesianTrajectory`'s duration-rescaling loop and can blow up `duration` (and hence sample count) without bound, since nothing was checking for *that*.

Fix: interior samples use a **separate internal solver with `max_restarts = 0`** (`PathTrackingIkOptions()`), so a struggling sample always keeps its warm-started, continuity-preserving result — never a discontinuous "improvement." A `kMaxReasonableSamples` sanity cap on projected sample count was also added as defense in depth, independent of this fix, in case some other pathology produces the same symptom later.

### Finding 4 (performance, not correctness): large Cartesian moves are slow to generate

Even with the above fixed, generating a trajectory for a large multi-joint Cartesian move can take tens of seconds: iterative velocity-limit duration-scaling (up to `max_duration_iterations` attempts) times a growing sample count (duration × sample_rate) times a bounded-but-nonzero per-sample IK cost (`PathTrackingIkOptions` caps this at 20 iterations × 5 inner retries specifically to bound this — full defaults would be far worse). This is a known, accepted limitation of the iterative rescaling approach for this milestone, not something fixed here — a faster approach (e.g. analytical duration estimation instead of iterate-and-check) is a candidate for later if M4's interactive use needs it. `test_cartesian_trajectory.cpp`'s move sizes are chosen to exercise the real properties (warm-start continuity, velocity-limit satisfaction) without turning the unit test suite itself into a multi-minute run.

## Execution: actionlib, not a blocking service

Decided during planning (see `ROADMAP.md`'s M3 entry): a blocking `ExecuteTrajectory.srv` was the original plan, changed to the standard `control_msgs/FollowJointTrajectoryAction` because M4 (joystick teleop — a new target arriving mid-motion) will almost certainly need preemption, and M5 (hardware) needs a cancellation path for safety. Building a custom blocking interface now would mean redoing it as an action later — a real control-flow migration, not a small diff. `trajectory_executor_node` publishes samples to `/joint_states_out` at each point's `time_from_start`, checks for a preempt request between samples (not just between points, so a long final segment doesn't delay responding to a cancel), and publishes basic feedback (desired vs. `/joint_states_in` actual). Deliberately minimal: no queuing/blending, one goal at a time.

## Not covered here

Acceleration/torque limits, obstacle avoidance, multi-waypoint paths — all out of scope for this milestone's point-to-point, velocity-limited-only remit.
