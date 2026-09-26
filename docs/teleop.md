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

### Null-space secondary task: bias toward a vertical link_4

This chain is redundant for the 4D task above (5 joints, 4 task rows), so the primary-task solve above is a *minimum-norm* choice among infinitely many `q̇` that achieve the same task velocity — it expresses no preference at all among them. Added on request: an optional secondary objective, projected into the task's null space (Liegeois gradient projection), biasing a chosen link's own local +Z axis toward vertical whenever there's spare capacity to do so, without ever changing the achieved task velocity:
```
q̇ = q̇_primary + (I − J_task⁺J_task)·q̇₀
```
`J_task⁺ = J_taskᵀ(J_task J_taskᵀ+λ²I)⁻¹` is the same damped pseudo-inverse already computed for the primary solve (reused, not recomputed), so `N = I − J_task⁺J_task` is one extra matrix product. With `â = R_link(q)·[0,0,1]ᵀ` (the chosen link's own local Z axis in root frame) and cost `H(q) = −ẑ_world·â` (minimizing H maximizes vertical alignment):
```
∂H/∂qᵢ = −ẑ_world·(zᵢ × â),   i upstream of the chosen link       q̇₀,ᵢ = k·ẑ_world·(zᵢ × â)
```
`zᵢ` (joint i's own world-frame rotation axis) is read directly off columns of the *same* 6×5 Jacobian `J` already computed for the primary task — it depends only on the shared root→joint_i sub-chain, not on which tip a solver was built for, so no second Jacobian evaluation is needed. Only joints upstream of the chosen link affect its orientation (`q̇₀` is zero for the rest) — for the default `link_4`, that's joints 1–4; joint 5 is downstream of it and has zero effect, confirmed from the URDF (`joint_5`'s origin in `link_4`'s own frame is `xyz="0 0 0.336"`, i.e. link_4's *local Z* is its own long axis — that's the axis this feature aligns to world Z, not an arbitrary choice).

`ComputeResolvedRateVelocity` takes this as two extra, defaulted parameters (`secondary_link_solver`, `nullspace_gain`) so every existing call site and test is unaffected unless it opts in. `teleop_node` constructs a second `ForwardKinematicsSolver` (`~nullspace_tip_link`, default `"link_4"`) purely to get `â` via `ComputePose` on the upstream joint sub-vector — reuses the existing multi-tip-link solver pattern (same as `fk_node`/`ik_node`'s configurable tip link), no new kinematics code.

**Disabling it**: `~nullspace_gain` (default `0.4`, heuristic — gentle enough not to noticeably fight a deliberate jog) is a plain ROS param; set `~nullspace_gain:=0.0` to turn the secondary task off entirely (`ComputeResolvedRateVelocity` treats zero gain as "off", regardless of whether a solver was passed).

**A caveat inherited from damping**: `N` is built from the *damped* pseudo-inverse, so `J_task·(N·q̇₀)` is only *approximately* zero in general (exactly zero only in the undamped limit) — the same kind of small, acknowledged bias as the primary task's own damped-tracking error (see `test_resolved_rate.cpp`'s `kNegligibleDamping` isolation). At `λ=0.05` this leakage into the primary task is negligible in practice (confirmed: `DoesNotDisturbThePrimaryTask` gtest, and `teleop_nullspace.test`'s live rostest, both pass comfortably), but it's worth knowing it's not a mathematical guarantee at the realistic damping magnitude.

## SET_REFERENCE: jog a marker, then commit a real trajectory

The reference pose accumulates the same (vx,vy,vz,pitch_rate) commands as DIRECT_JOG, but as a free-standing `Eigen::Isometry3d`, not through the Jacobian — no arm motion, so no IK/Jacobian needed until commit. Pitch is applied as a rotation about the reference pose's **own** evolving local Y axis, composed repeatedly; since every applied rotation is a pure rotation about that one evolving axis, roll and yaw (relative to whatever they were when this mode was entered) never drift — deliberately different from DIRECT_JOG's "roll/yaw free" policy, since a static target benefits from being fully predictable, whereas the live-jogging feel was the reason DIRECT_JOG leaves them free.

`~commit_button` calls the existing `/generate_trajectory` service (`cartesian_path=false`: solve IK once, then a plain joint-space move — this is a single endpoint, not a path to trace) with `start_joint_positions` left empty (resumes from wherever `/joint_states_in` actually is), then sends the result to `follow_joint_trajectory`. This is a **one-shot** call, not a per-tick stream, so the service's own default IK options (restarts enabled) are fine here — restarts only threaten continuity across a rapid sequence of preempting calls, and a single commit has no such sequence.

## SET_REFERENCE visualization: reference marker + ghost arm

Two rviz aids, added after initial hand-off feedback that the reference pose had no visual indication:

- **Reference marker** (`/teleop_reference_marker`, `visualization_msgs/Marker`, an `ARROW` at `reference_pose_`): published every SET_REFERENCE tick, and explicitly `DELETE`d the instant the mode switches away — so it never lingers stale once you're back in DIRECT_JOG.
- **Ghost arm** (`/ghost_joint_states`, a full joint configuration, meant to feed a *second* `robot_state_publisher` + a second, semi-transparent `RobotModel` display in rviz — see `launch/teleop_visualize.launch`): in DIRECT_JOG it's just set equal to the real arm's commanded configuration every tick, so the ghost RobotModel display exactly overlaps the real one — visually inert until it's actually needed. In SET_REFERENCE it instead previews a full best-effort IK solution for `reference_pose_`, warm-started tick to tick from its own last solution.

This ghost IK reuses a plain `arm_kinematics::InverseKinematicsSolver` with **default** options (restarts enabled) — a deliberate departure from DIRECT_JOG's careful restart-avoidance: this solve is a cosmetic preview, not a safety-relevant control loop, so an occasional restart-induced elbow-configuration jump or a slower tick (the service's default IK can take tens of ms under restarts) doesn't carry the weight it did for the per-tick position-IK design that motivated all of DIRECT_JOG's care in the first place. On a tick where it fails to converge, `ghost_joint_positions_` is simply left unchanged — the preview freezes at its last valid configuration rather than ever showing a wrong one.

### Making a *second* robot model render in rviz without touching `arm_interface`

`robot_state_publisher` in this ROS Noetic build has **no working `~tf_prefix`** (confirmed empirically: grepping its binary turns up no such string, and a live test bore this out — a second instance publishing under a different topic/namespace still broadcasts the *same* unprefixed frame names, colliding with the real robot's). The standard `moveit`-style "ghost state" trick (a second `RobotModel` display pointed at a differently-prefixed TF tree) therefore needs the *link names themselves* to differ, not just the topic.

Fix: `launch/teleop_visualize.launch` publishes a second `ghost_robot_description` parameter — the same `arm.urdf`, piped through `sed` to rename every non-root link (`link_1`…`link_5`, `link_intermediate`, `end_effector`) to a `ghost_`-prefixed name, while leaving `base_link` (the shared root) and all *joint* names untouched. A second `robot_state_publisher`, remapped to that description and to `/ghost_joint_states`, then broadcasts a fully independent, correctly-parented `ghost_*` frame tree rooted at the same `base_link` the real robot shares — confirmed empirically (a small script publishing distinct joint angles to both topics and reading back both `link_1`→`link_2` and `ghost_link_1`→`ghost_link_2` transforms via `tf2_ros.Buffer`) before committing to this approach. rviz's `RobotModel` display has its own top-level `Alpha` field (independent of any per-link URDF material alpha), used here to render the ghost at 0.35 vs. the real robot's 1.0 — no URDF material edits needed either.

One more thing confirmed the hard way: `<param command="...">` execs its argument list directly, with **no shell** — `xacro ... | sed ...` as a bare `command` string passes `|` to `xacro` as a literal argument rather than piping. Wrapping the whole pipeline in `bash -c "..."` (itself one shell-interpreted argument) is what actually works.

## History: why not per-tick position IK

The original implementation solved position-IK to a slightly-offset target every tick. Self-test found two compounding problems: (1) the service's default IK restarts could land a tiny jog step in a different elbow configuration, producing long, constantly-preempted trajectories that never really moved the arm; and (2) holding a *fixed* orientation while jogging position eventually reached a point where that combination wasn't achievable, and every subsequent tick failed identically forever (a permanent wedge — the state driving the failure never advanced on failure). A local-retry recovery scheme was built to work around both, but the second problem generalizes: any fixed-orientation-plus-moving-position scheme can wedge. Resolved-rate control sidesteps the whole class: there's no convergence check to fail, and pitch is now a genuinely free, live DOF rather than something the arm could get boxed out of.

## Gripper: independent publisher, confirmed safe

`joint_6` in `arm_interface/urdf/arm.urdf` is a **fixed** joint (`link_5` → `end_effector`) — not part of the pose kinematic chain (M1 finding: motor 6 drives the gripper mechanism, not a 6th kinematic DOF). `arm_sim_validator_node` still reserves a 6th state slot for it (`num_joints=6`) and its `commandedCb` updates only the joint names present in each incoming `/joint_states_out` message, leaving the rest untouched (verified by reading the code before committing to this design) — so `teleop_node`'s gripper publisher (`name=["joint_6"]` only) and both the DIRECT_JOG stream and `trajectory_executor_node`'s publisher (`name=[joint_1..joint_5]` only) coexist on the same topic without any of them resetting the others. Gripper handling is unconditional on mode — it never touches the real arm pose or the reference pose either way.

Because `joint_6` is fixed, it has no real URDF `<limit>` to derive a range or rate from — `gripper_min`/`gripper_max`/`gripper_rate` are heuristic ROS-param defaults (`[-1.0, 1.0]` rad-equivalent, `0.5`/s), flagged for recalibration once real gripper hardware/limits are known (M5).

## No real joystick hardware yet

No `/dev/input/js*` device and no `ros-noetic-joy` in the container at the time of this milestone — confirmed, not assumed. Deferred rather than wiring up now (would need a docker-config change plus a physical gamepad); `scripts/manual_teleop_check.py` substitutes a keyboard-driven synthetic `/joy` publisher for both manual and automated (rostest) testing. Real-hardware wiring is a documented follow-up, likely alongside M5 (sim-to-real).

## Testing

- `test/test_joy_target_mapper.cpp` (gtest): deadman gating (including of the edge-triggered buttons), velocity/gain scaling, deadzone, gripper sign and both-held-is-no-op, mode-toggle/commit firing exactly once per press (not once per tick held), out-of-range axis/button indices and an empty "previous message" treated as inactive rather than crashing.
- `test/test_resolved_rate.cpp` (gtest): zero command → zero velocity; a small linear command's *integrated, re-computed-via-FK* motion matches the commanded velocity (independent cross-check, not a self-consistency check against the same Jacobian call); a pitch-only command rotates about the end-effector's own local Y axis at the commanded rate; artificially tight velocity limits are respected and measurably reduce the achieved ratio. Plus (`ResolvedRateNullspaceTest`): `nullspace_gain=0` matches no secondary solver at all, byte-for-byte; a non-vertical starting configuration with *zero primary command* still produces nonzero joint velocities that measurably improve `link_4`'s vertical alignment (independent FD check, not a self-consistency check against the same cost function); the secondary task doesn't measurably disturb an active primary command.
- `test/teleop_modes.test` + `.py` (rostest, headless, against the real simulator + `fk_node` + full `arm_trajectory` stack; `~nullspace_gain:=0` here — see below for why): DIRECT_JOG jogging moves the tracked position and stops on deadman release; SET_REFERENCE jogging does *not* move the real arm; committing from SET_REFERENCE does; gripper moves independently of both; the ghost mirrors the real arm in DIRECT_JOG and diverges from it in SET_REFERENCE; the reference marker is shown while in SET_REFERENCE and `DELETE`d on leaving it. Every test is self-contained (probes and, if needed, corrects the current mode itself) rather than assuming whatever a previous test left behind, since unittest runs test methods alphabetically, not declaration order — this caught a real bug during self-test: the commit test originally declared success on the *first sign* of arm motion rather than waiting for the trajectory to actually finish, occasionally leaving it still executing in the background when the next (alphabetically) test ran and asserted the arm stayed still. A second, similar bug: a probe helper's post-motion settle sleep (0.2s) was shorter than `~joy_timeout` (0.5s) — since a jog burst ends by simply no longer publishing `/joy` rather than explicitly releasing the deadman, the arm keeps crawling on the last-received (stale-pending) command until the timeout elapses, so a shorter sleep could catch a "before" baseline reading mid-crawl. Both were intermittent-failure bugs in the test harness itself, not in `teleop_node`.
- `test/teleop_nullspace.test` + `.py` (rostest, headless, isolated on purpose): with `~nullspace_gain` enabled, holding the deadman with a *centered* stick (zero commanded task velocity) still measurably improves `link_4`'s real, TF-measured vertical alignment over a few seconds. Kept as its own suite rather than folded into `teleop_modes.test`, because that suite's tests assume deadman-held-but-idle means no motion at all — which this feature deliberately violates by design (that's the entire point: bias the leftover null-space freedom whenever there's spare capacity, not only while actively jogging).
- `launch/teleop_visualize.launch` + `rviz/teleop_visualize.rviz`: the interactive way to see all of the above (real + ghost robot models, reference marker) — not exercised by rostest (no display available headlessly), smoke-tested with `use_rviz:=false` instead.

## Not covered here

Real joystick hardware validation, gripper limit/rate calibration against real hardware, position/velocity/current-based operating modes on the actual Dynamixel motors (currently position-control-only at the driver level — `arm_interface/robot_interface.cpp` never writes Operating Mode or Goal_Velocity; adding those is an `arm_interface`/M5 concern, not this package's).
