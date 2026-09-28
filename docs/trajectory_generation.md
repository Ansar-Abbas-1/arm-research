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

### Finding 4, revisited: large Cartesian moves are slow to generate — a real but partial fix

Originally flagged as an accepted, unaddressed limitation (iterative velocity-limit duration-scaling re-sampled the *whole* path at full `sample_rate_hz` resolution on every rescaling attempt — the dominant cost for a large move). Revisited on the `milestone-4-joy-teleop-v2` branch, since this branch's milestone is trajectory generation itself rather than teleop.

**The fix**: coarse-to-fine duration search. The velocity-limit check only needs approximately the right duration to converge, not a full-resolution trajectory at every guess — so search first at a small, fixed sample count (25, independent of `duration`/`sample_rate_hz`, so it stays cheap even for a large move), then run the real `sample_rate_hz` resolution only to validate/correct the converged estimate. A coarser search can only *underestimate* a true peak (narrower windows catch spikes a wide average smooths over), never overestimate in a way that would make the real check unnecessarily strict, so the full-resolution pass is still the one actually gating success — same criteria and output as before, just usually cheaper to get there. The full-resolution correction loop's pass cap is tied to `max_duration_iterations` itself (not a separate smaller constant): a hardcoded cap of 2 was tried first and **failed a real move** whose true worst ratio was only 1.023 (one more pass would have converged it) — confirmed by `LargeMoveNeedsMultipleFullResolutionCorrectionPasses`, which is now a permanent regression test for exactly this.

**Measured, not assumed** (same machine, same move, `test/test_cartesian_trajectory.cpp`'s existing moves):
- `WarmStartKeepsConsecutiveSamplesClose`'s move: **3293 ms → 1829 ms** (44% faster) — the coarse phase converges in one or two cheap passes here, and the full-resolution phase needs only one confirming pass.
- A larger, deliberately harder move (5 joints moved simultaneously, up to ~5 rad each — see `LargeMoveNeedsMultipleFullResolutionCorrectionPasses`): **12250 ms → ~12400-13300 ms** — essentially *no* improvement, and slightly worse. Root cause, confirmed by instrumenting both phases: this move's true worst-velocity window is narrow enough that even the coarse phase's very first guess "looks converged" (0 coarse rescaling iterations needed), so almost none of the real convergence work is deferred to the cheap phase — it all still happens across 4 full-resolution correction passes, just as it would have in the original single-phase algorithm, plus a small amount of coarse-phase overhead on top.

**Honest conclusion**: this is a real win for the common case (a Cartesian move whose required duration the coarse phase can actually estimate well) and a wash — not a regression, thanks to the pass cap fix, but not a win either — for a harder move where the coarse phase's cheap check simply isn't sensitive enough to the true bottleneck. A genuinely robust fix for the worst case would need something that doesn't rely on uniform time-sampling catching a possibly-narrow velocity spike (e.g. an actual analytical bound on the induced joint-space velocity along the interpolated path, not sampled at any resolution) — out of scope for what was revisited here. `test_cartesian_trajectory.cpp`'s other move sizes are still chosen to exercise warm-start continuity and velocity-limit satisfaction without turning the suite into a multi-minute run; the one large-move regression test above is the deliberate exception, needed because every smaller move tried during development converged in a single full-resolution pass and didn't exercise the bug it guards against.

## `/reach_target_pose`: a non-interactive go-to-pose entry point

This branch replaces M4's joystick-teleop scope with trajectory generation itself. `reach_target_pose_node` (`ReachTargetPose.srv`) is the resulting entry point: a single blocking service call that orchestrates the two pieces above (`/generate_trajectory` then `follow_joint_trajectory`) and, critically, **verifies the result** — after execution finishes, it reads `/joint_states_in`, computes FK, and compares against the requested pose, rather than trusting "trajectory generation succeeded" as the whole story. `final_position_error`/`final_orientation_error` are always populated (not just on failure) so a caller can judge margin, not just pass/fail. Tolerance defaults match `InverseKinematicsSolver`'s own (`1e-4` m / `1e-3` rad) — the goal IK solve inside `/generate_trajectory` was already held to that bar, so holding the executed, physical (simulated) result to the same one is the natural check, not a new arbitrary number.

Deliberately thin: no new planning/IK logic, just orchestration + verification. Runs a 2-thread `AsyncSpinner` (not plain `ros::spin()`), since the service callback blocks inside `waitForResult()` for the whole motion — a single-threaded spinner would starve the `/joint_states_in` subscriber callback for that entire window, making the post-execution FK check read a stale, pre-motion pose.

**Tested** (`test/reach_target_pose_test.py`, headless rostest against the real simulator): a real reachable pose (captured live from `/computed_end_effector_pose` after a setup move, not hand-computed) is actually reached — checked two ways, the service's own self-reported error AND an independent post-call read of `/computed_end_effector_pose`, so a bug that made the service misreport its own success wouldn't be missed; an unreachable pose fails cleanly with a non-empty explanatory message rather than hanging.

## Execution: actionlib, not a blocking service

Decided during planning (see `ROADMAP.md`'s M3 entry): a blocking `ExecuteTrajectory.srv` was the original plan, changed to the standard `control_msgs/FollowJointTrajectoryAction` because M4 (joystick teleop — a new target arriving mid-motion) will almost certainly need preemption, and M5 (hardware) needs a cancellation path for safety. Building a custom blocking interface now would mean redoing it as an action later — a real control-flow migration, not a small diff. `trajectory_executor_node` publishes samples to `/joint_states_out` at each point's `time_from_start`, checks for a preempt request between samples (not just between points, so a long final segment doesn't delay responding to a cancel), and publishes basic feedback (desired vs. `/joint_states_in` actual). Deliberately minimal: no queuing/blending, one goal at a time.

## Not covered here

Acceleration/torque limits, obstacle avoidance, multi-waypoint paths — all out of scope for this milestone's point-to-point, velocity-limited-only remit.
