# Joystick teleop — design notes

Incremental Cartesian jogging on top of `arm_kinematics` (FK/IK) and `arm_trajectory`'s quintic time-scaling (`GenerateJointSpaceTrajectory`) and execution interface (`follow_joint_trajectory`). `teleop_node` does its own FK/IK in-process — see "Why not `/generate_trajectory`" below for why that changed from the original plan.

## Scope decisions

- **Position-only jogging, orientation held fixed** at whatever it was when `teleop_node` captured its initial joint state. Full 6-DOF jogging was considered and skipped: this chain is 5 DOF over 6D task space (M2/M3 finding), so IK's already-tested weighted least-squares solve already absorbs any residual against a held orientation with no new solver behavior needed.
- **Deadman/enable button gates all motion** (position and gripper) — standard joystick-teleop convention (e.g. `teleop_twist_joy`): without it held, stick/button input is ignored regardless of value.
- **A stale-`/joy` timeout (`~joy_timeout`, default 0.5s) also stops motion**, independent of the deadman button's last-known state — if the joystick topic dies, the arm should not keep executing whatever the last received message said.
- Axis/button indices are ROS params (`~axis_x/y/z`, `~deadman_button`, `~gripper_open_button`, `~gripper_close_button`), defaulted to a common Xbox-style layout, not hardcoded to one controller model.

## Why not `/generate_trajectory`

The original plan routed jog targets through `arm_trajectory`'s existing `/generate_trajectory` service (`cartesian_path=false`: solve IK once, then a plain joint-space move). Self-test found this doesn't work for a continuous per-tick jog:

That service's `InverseKinematicsSolver` instance uses its own default `IkOptions` (restarts enabled, ~20 budget). A tiny ~1cm jog step could still trigger a restart landing in a **different elbow configuration** — confirmed via debug logging: every tick produced a 150-200 point trajectory (~3-4s duration) for what should be a near-instant nudge, constantly preempted by the next tick's own barely-progressed trajectory, so the arm never really moved despite the internal target advancing correctly tick to tick. This is the same mechanism as arm_trajectory's Finding 3, just triggered by a different caller.

Fixing this needs control over the `InverseKinematicsSolver`'s options (specifically, disabling its restarts), which `GenerateTrajectory.srv` doesn't expose. Rather than add that surface to `arm_trajectory` (a closed milestone, and IK-option control isn't otherwise needed by any of its other callers), `teleop_node` constructs its **own** `ForwardKinematicsSolver`/`InverseKinematicsSolver` in-process — the same pattern `cartesian_trajectory.cpp` already uses — and only calls into `arm_trajectory` as a library for the quintic time-scaling math (`GenerateJointSpaceTrajectory`), not the service.

## Control loop

`teleop_node` owns an internal joint-space seed and Cartesian target pose (position free-running, orientation fixed), seeded once from the first `/joint_states_in` message it receives. A fixed-rate timer (`~rate`, default 10Hz):

1. Computes the tick's position/gripper deltas from the latest `/joy` message (`ComputeJoyTargetUpdate`, pure function, unit tested independent of any node).
2. If the gripper changed: clamps and publishes it directly to `/joint_states_out` as `name=["joint_6"]` — independent of the pose-jogging path entirely (see "Gripper" below).
3. If the stick moved: builds a candidate target pose and solves IK for it, warm-started from the current internal joint seed (`SolveWithLocalRetries`, see "IK options for jogging" below).
4. **Only commits the candidate (target pose and joint seed) if IK converges.** A candidate that doesn't converge is dropped, not retried with the same state — otherwise the internal target could drift outside reachable space (or get stuck on a single hard-to-converge point) and wedge every subsequent tick, since each new candidate is built by adding onto the previous target regardless of whether it was ever actually reached.
5. Turns the converged joint solution into a quintic-timed joint-space trajectory (`GenerateJointSpaceTrajectory`, from the current seed to the new solution) and sends it to `follow_joint_trajectory`. A new goal there preempts whatever is still executing (`SimpleActionServer`'s default behavior, already relied on by M3) — this is what makes jogging responsive rather than queued.

## IK options for jogging

`JogIkOptions()`: `max_restarts = 0` (see "Why not `/generate_trajectory`" above), `position_tolerance = 1e-3` (loosened from the 1e-4 default — still far tighter than a jog needs to feel precise; a single fresh `Solve()` call re-adapting its damping from scratch each tick lands consistently in the 1-2mm range even when genuinely converging, not stuck), `max_iterations = 300`, `max_inner_retries = 30` (both raised from the defaults 100/10 — found empirically necessary for a fresh per-tick solve near a poorly-conditioned point to actually finish converging rather than exhausting its budget just short of tolerance).

With restarts disabled, a tick whose primary warm-started attempt doesn't converge has no fallback unless `teleop_node` provides one — and just dropping it isn't enough on its own, because the internal seed doesn't advance on a dropped tick either, so an unchanged input reproduces the *exact same failure* forever (a permanent wedge). `SolveWithLocalRetries` (`teleop_node.cpp`) provides that fallback, but **only ever perturbs the seed by a small, bounded amount** (`kLocalRetryNudge = 0.1` rad per joint) — never `InverseKinematicsSolver`'s own restart mechanism, which draws a replacement seed uniformly across the *entire* joint range. A small fixed restart budget (3) was tried and rejected for the same reason as the unbounded case: confirmed via the rostest (`teleop_preemption_test.py`) that a single restart landed in a configuration whose intermediate joint-space path swung the *measured* Cartesian position in the wrong direction mid-flight (a joint-space interpolation between very different configurations doesn't trace a straight Cartesian line, same insight as Finding 2/3) — an operator jogging with a joystick would see the arm briefly move opposite to command. Not acceptable at any restart frequency, however rare, so recovery had to stay bounded-distance by construction instead.

`SolveWithLocalRetries` tries, in order: the primary warm-started seed; one **directed** nudge (every joint moved toward the middle of its own range, not randomly — because arm_trajectory's Finding 1 singularity has multiple joints sitting exactly at one of their own limit boundaries at once, so a *random* jitter only has good odds of nudging every affected joint the correct direction by chance); then up to `kLocalRetryAttempts = 5` random local jitters. Every attempted seed differs from the current continuous configuration by at most `kLocalRetryNudge`, so a recovered tick still can't jump to a distant elbow configuration.

### The `{0,0,0,0,0}` singularity still needs a separate, one-time startup nudge

Found empirically: `SolveWithLocalRetries`'s bounded recovery above is **not** enough to escape arm_trajectory's Finding 1 singularity if jogging starts exactly there (the arm's own home/default configuration) — every directed and random local retry plateaued at the same ~1-2mm residual, with `iterations` pinned at the `max_iterations` budget every time, confirming this is a hard kinematic restriction at that exact point, not a numerical-conditioning wobble a bit more iteration budget or a small perturbation can shake loose. `JointStatesCb` checks for this specifically (`NearZeroConfiguration`, all joints within `kSingularityZeroEpsilon = 0.01` rad of zero) and applies a one-time, larger, fixed nudge (`kSingularityEscapeNudge = 0.05` rad per joint) to the internal seed *before* jogging starts, clamped to joint limits. All thresholds/magnitudes here are heuristic, not derived, and specific to this one known singular point; `SolveWithLocalRetries` remains the general-purpose mechanism for anything else encountered mid-jog.

## Why `cartesian_path=false`'s reasoning still applies

A jog target is a single endpoint, not a path the user cares about tracing exactly. `teleop_node` builds a plain joint-space move to the converged IK solution (matching the old `cartesian_path=false` service branch's intent) rather than tracing an interpolated Cartesian path per tick, avoiding Finding 4 (large Cartesian moves being slow to generate) on every tick — moot anyway now that IK is solved directly rather than through the service.

## Gripper: independent publisher, confirmed safe

`joint_6` in `arm_interface/urdf/arm.urdf` is a **fixed** joint (`link_5` → `end_effector`) — not part of the pose kinematic chain (M1 finding: motor 6 drives the gripper mechanism, not a 6th kinematic DOF). `arm_sim_validator_node` still reserves a 6th state slot for it (`num_joints=6`) and its `commandedCb` updates only the joint names present in each incoming `/joint_states_out` message, leaving the rest untouched (verified by reading the code before committing to this design, not assumed) — so `teleop_node`'s gripper publisher (`name=["joint_6"]` only) and `trajectory_executor_node`'s publisher (`name=[joint_1..joint_5]` only) coexist on the same topic without either resetting the other.

Because `joint_6` is fixed, it has no real URDF `<limit>` to derive a range or rate from — `gripper_min`/`gripper_max`/`gripper_rate` are heuristic ROS-param defaults (`[-1.0, 1.0]` rad-equivalent, `0.5`/s), flagged here for recalibration once real gripper hardware/limits are known (M5).

## Testing

- `test/test_joy_target_mapper.cpp` (gtest): deadman gating, gain/dt scaling, deadzone, gripper open/close sign and both-held-is-no-op, out-of-range axis/button indices treated as inactive rather than crashing.
- `test/teleop_preemption.test` + `.py` (rostest, headless, against the real simulator + `fk_node` (test's own independent position readout only) + `trajectory_executor_node`): sustained stick input moves the tracked end-effector position and reversing it promptly reverses that motion (confirms per-tick goals are genuinely preempting, not queuing/stalling); releasing the deadman stops motion; gripper input moves `joint_6` independently of Cartesian position. Repeated 3x for flakiness given `SolveWithLocalRetries`' randomized fallback attempts.

## Known limitation

An occasional tick's IK still fails to converge even after all local-retry attempts (observed during self-test, residual a few mm above tolerance) — jogging tolerates this by design (the tick is dropped, next tick retries with fresh joystick input), but it means jog responsiveness isn't perfectly uniform across the workspace; some regions need more retries (hence more per-tick latency) than others. Not fixed further here since overall tracking (the rostest's actual pass criterion) isn't affected.

## Not covered here

Full 6-DOF (position+orientation) jogging, real joystick hardware validation (no physical controller assumed available — `scripts/manual_teleop_check.py` substitutes keyboard input), gripper limit/rate calibration against real hardware.
