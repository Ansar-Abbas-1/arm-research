# Joystick teleop — design notes

Two explicit, mutually exclusive modes, toggled by `~mode_toggle_button` (edge-triggered):

- **DIRECT_JOG** (default): resolved-rate velocity control of the end-effector (X/Y/Z + wrist pitch), streamed directly to `/joint_states_out` at the control rate.
- **SET_REFERENCE**: the same stick jogs a free-standing reference pose only — the real arm doesn't move. `~commit_button` (edge-triggered) sends the reference through `arm_trajectory`'s existing `/generate_trajectory` + `follow_joint_trajectory`.

This replaces an earlier, simpler design (continuous per-tick position-IK jogging) that got stuck in practice — see "History: why not per-tick position IK" below.

## Why two modes

Direct velocity jogging gives immediate, responsive feel but no path/goal semantics — you can't ask it "go to exactly this pose." Set-a-reference-then-commit gives a real planned, quintic-timed, velocity-limit-respecting move to an exact target, reusing M3's already-tested machinery unchanged — but isn't continuous/live. Switching between them covers both use cases without conflating their very different execution models (a continuous stream vs. a discrete planned action).

Switching modes always cancels any in-flight `follow_joint_trajectory` goal first (`action_client_.cancelGoal()`, idempotent), so the action executor and `teleop_node`'s own direct-publish path can never write `/joint_states_out` at the same time. Entering `SET_REFERENCE` reseeds the reference pose from the arm's actual current pose (`fk_solver_->ComputePose`), not wherever it was last left.

## DIRECT_JOG: resolved-rate control

Commanded task velocity, 4 rows — X, Y, Z linear (root frame) + pitch (about the end-effector's **own current local Y axis**, not a root-frame axis, so "pitch" means the same physical wrist motion regardless of the arm's current orientation):
```
ẋ_cmd = [vx, vy, vz, ωpitch]ᵀ ∈ ℝ⁴
```
Reusing the existing geometric Jacobian `J ∈ ℝ^{6×5}` (`ComputeGeometricJacobian`, unchanged), project the angular rows onto the tool's current pitch axis (`ê_pitch = R_ee·[0,1,0]ᵀ`) instead of commanding root-frame ωx,ωy,ωz directly:
```
J_task = [ J_v ; êₚᵀ J_ω ] ∈ ℝ^{4×5}
```
Task rows (4) < joints (5) here — **opposite** of the full-pose IK case (M2: 6>5, left/normal-equations pseudo-inverse). This is the classic redundant-manipulator case, so it's the **right** damped pseudo-inverse (Whitney resolved-rate control):
```
q̇ = J_taskᵀ (J_task J_taskᵀ + λ²I)⁻¹ ẋ_cmd
```
`λ = 0.05` (`~velocity_damping`) is heuristic, matching the existing IK's `initial_damping` scale. It tapers `q̇` smoothly as `J_task J_taskᵀ` approaches rank deficiency (a singularity, or a direction the 4-row projection can't reach) instead of blowing up — there is no iteration to adapt over here (a single per-tick velocity solve, not an iterative position solve), so the damping is fixed rather than adaptive.

Roll and yaw are **not commanded at all** — not held fixed, just absent from `J_task`'s rows, so they drift to whatever the minimum-norm `q̇` implies. (Confirmed decision, not a simplification: holding them fixed would add two more constrained rows fighting the primary command near a singularity, reintroducing the same class of conflict that broke the earlier per-tick position-IK design.)

`q̇` is then clamped componentwise to `GetActuatedJointVelocityLimits()` (defensive hardware-safety clamp, independent of the damping above), and the tick's actually-achieved task velocity is compared against what was commanded:
```
ratio = ‖J_task · q̇_clamped‖ / ‖ẋ_cmd‖
```
Below `~saturation_warn_ratio` (default 0.5), `ROS_WARN_THROTTLE` — informational only, never a failure, unlike the position-IK design this replaced.

**Integration**: the sim/HW interface (`/joint_states_out`) is position-only (confirmed in `arm_sim_validator_node::commandedCb`), so velocity control is realized as Euler integration published directly at the control tick:
```
q_{k+1} = clamp(q_k + q̇·dt, position limits)
```
`q_k` is read from `/joint_states_in` **fresh every tick** (closed-loop off measured state), not integrated open-loop from teleop's own memory — avoids drift between the commanded and actual arm if the sim/HW doesn't track instantly. Control rate: 50Hz (`~rate`), matching `arm_sim_validator_node`'s own `publish_rate` and `arm_trajectory`'s `sample_rate` default.

Pure math (`resolved_rate.h/.cpp`) is separated from the ROS node for testability, per project convention — `ComputeResolvedRateVelocity` takes an `ForwardKinematicsSolver` and joint positions, no ROS I/O.

## SET_REFERENCE: jog a marker, then commit a real trajectory

The reference pose accumulates the same (vx,vy,vz,pitch_rate) commands as DIRECT_JOG, but as a free-standing `Eigen::Isometry3d`, not through the Jacobian — no arm motion, so no IK/Jacobian needed until commit. Pitch is applied as a rotation about the reference pose's **own** evolving local Y axis, composed repeatedly; since every applied rotation is a pure rotation about that one evolving axis, roll and yaw (relative to whatever they were when this mode was entered) never drift — deliberately different from DIRECT_JOG's "roll/yaw free" policy, since a static target benefits from being fully predictable, whereas the live-jogging feel was the reason DIRECT_JOG leaves them free.

`~commit_button` calls the existing `/generate_trajectory` service (`cartesian_path=false`: solve IK once, then a plain joint-space move — this is a single endpoint, not a path to trace) with `start_joint_positions` left empty (resumes from wherever `/joint_states_in` actually is), then sends the result to `follow_joint_trajectory`. This is a **one-shot** call, not a per-tick stream, so the service's own default IK options (restarts enabled) are fine here — restarts only threaten continuity across a rapid sequence of preempting calls, and a single commit has no such sequence.

## History: why not per-tick position IK

The original implementation solved position-IK to a slightly-offset target every tick. Self-test found two compounding problems: (1) the service's default IK restarts could land a tiny jog step in a different elbow configuration, producing long, constantly-preempted trajectories that never really moved the arm; and (2) holding a *fixed* orientation while jogging position eventually reached a point where that combination wasn't achievable, and every subsequent tick failed identically forever (a permanent wedge — the state driving the failure never advanced on failure). A local-retry recovery scheme was built to work around both, but the second problem generalizes: any fixed-orientation-plus-moving-position scheme can wedge. Resolved-rate control sidesteps the whole class: there's no convergence check to fail, and pitch is now a genuinely free, live DOF rather than something the arm could get boxed out of.

## Gripper: independent publisher, confirmed safe

`joint_6` in `arm_interface/urdf/arm.urdf` is a **fixed** joint (`link_5` → `end_effector`) — not part of the pose kinematic chain (M1 finding: motor 6 drives the gripper mechanism, not a 6th kinematic DOF). `arm_sim_validator_node` still reserves a 6th state slot for it (`num_joints=6`) and its `commandedCb` updates only the joint names present in each incoming `/joint_states_out` message, leaving the rest untouched (verified by reading the code before committing to this design) — so `teleop_node`'s gripper publisher (`name=["joint_6"]` only) and both the DIRECT_JOG stream and `trajectory_executor_node`'s publisher (`name=[joint_1..joint_5]` only) coexist on the same topic without any of them resetting the others. Gripper handling is unconditional on mode — it never touches the real arm pose or the reference pose either way.

Because `joint_6` is fixed, it has no real URDF `<limit>` to derive a range or rate from — `gripper_min`/`gripper_max`/`gripper_rate` are heuristic ROS-param defaults (`[-1.0, 1.0]` rad-equivalent, `0.5`/s), flagged for recalibration once real gripper hardware/limits are known (M5).

## No real joystick hardware yet

No `/dev/input/js*` device and no `ros-noetic-joy` in the container at the time of this milestone — confirmed, not assumed. Deferred rather than wiring up now (would need a docker-config change plus a physical gamepad); `scripts/manual_teleop_check.py` substitutes a keyboard-driven synthetic `/joy` publisher for both manual and automated (rostest) testing. Real-hardware wiring is a documented follow-up, likely alongside M5 (sim-to-real).

## Testing

- `test/test_joy_target_mapper.cpp` (gtest): deadman gating (including of the edge-triggered buttons), velocity/gain scaling, deadzone, gripper sign and both-held-is-no-op, mode-toggle/commit firing exactly once per press (not once per tick held), out-of-range axis/button indices and an empty "previous message" treated as inactive rather than crashing.
- `test/test_resolved_rate.cpp` (gtest): zero command → zero velocity; a small linear command's *integrated, re-computed-via-FK* motion matches the commanded velocity (independent cross-check, not a self-consistency check against the same Jacobian call); a pitch-only command rotates about the end-effector's own local Y axis at the commanded rate; artificially tight velocity limits are respected and measurably reduce the achieved ratio.
- `test/teleop_modes.test` + `.py` (rostest, headless, against the real simulator + `fk_node` + full `arm_trajectory` stack): DIRECT_JOG jogging moves the tracked position and stops on deadman release; SET_REFERENCE jogging does *not* move the real arm; committing from SET_REFERENCE does; gripper moves independently of both. Every test is self-contained (probes and, if needed, corrects the current mode itself) rather than assuming whatever a previous test left behind, since unittest runs test methods alphabetically, not declaration order.

## Not covered here

Real joystick hardware validation, gripper limit/rate calibration against real hardware, position/velocity/current-based operating modes on the actual Dynamixel motors (currently position-control-only at the driver level — `arm_interface/robot_interface.cpp` never writes Operating Mode or Goal_Velocity; adding those is an `arm_interface`/M5 concern, not this package's).
