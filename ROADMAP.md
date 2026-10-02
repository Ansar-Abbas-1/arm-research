# Roadmap — [ROBOT NAME] incremental capabilities

Multi-repo catkin workspace. This file is the only thing tracked in the
workspace-root repo (`/home/rialiotta/dev/claude_ros_ws`); each package
under `src/` is its own independently-versioned git repo (`arm_interface`,
and one new repo per milestone that adds a package, per the "new capability
→ new package" rule).

| Milestone | Status | Package(s) |
|---|---|---|
| M0 — Orientation | **Done** (2026-09-25) | `arm_interface` (existing) |
| M1 — Forward kinematics | **Done** (2026-09-25) | `arm_kinematics` (new) |
| M2 — Inverse kinematics | **Done** (2026-09-25) | `arm_kinematics` |
| M3 — Trajectory generation | **Done** (2026-09-26) | `arm_trajectory` (new) |
| M4 — Non-interactive reach + base-collision avoidance | **Done** (2026-10-01) | `arm_trajectory`, `arm_kinematics` |
| M5 — Sim-to-real | Stages 1 (read-only) and 2 (single-joint motion, all 6 joints) passed on real hardware; Stage 3 (full pipeline, reduced speed) next | `arm_interface` |
| M6 — Camera + calibration | Sketch | TBD |
| M7 — Target detection/localization | Sketch | TBD |
| M8 — Closed-loop reach | Sketch | — |

---

## M0 — Orientation (done, 2026-09-25)

Full architecture writeup: [`src/arm_interface/docs/architecture.md`](src/arm_interface/docs/architecture.md).

**Key decisions made:**
- **Kinematics scope: 5-DOF.** joint_6 is the gripper actuator, not part of arm pose. FK/IK (M1/M2) solve joints 1–5, `base_link → link_5`; joint_6/gripper is an independent open/close DOF, never part of the pose chain.
- `config/ArmInterfaceBad.yaml` (5-motor legacy config) confirmed dead, deleted.
- Gazebo references (`gazebo_ros` export, the once-build-blocking `scripts/arm_gazebo_bridge_node.py` install line) kept as an inactive placeholder — `arm_sim_validator_node`'s own kinematic simulator is the real sim path through at least M4.
- `command_timeout` (torque-disable watchdog) made runtime-tunable via `dynamic_reconfigure`, validated (rejects ≤0), default 0.5 s.
- Known-limitations cleanup (stale node naming, `bulk_read.cpp` bias/sign drift, `find_limits.cpp` hardcoded motor count, README drift, missing `package.xml` launch deps) done immediately rather than deferred.
- `arm_interface` git-initialized (wasn't before); workspace root now git-initialized too, multi-repo layout, to hold this file and future cross-cutting docs.

**Known limitations carried forward** (not blocking, see architecture.md for detail): `robot_interface`/`bulk_read`/`find_limits` have no automated test coverage beyond the new watchdog unit tests, since `robot_interface` opens a real serial port in its constructor — closing that gap needs either hardware-in-the-loop testing or a control/I-O separation refactor, candidate for a later milestone rather than M0/M1 scope-creep.

---

## M1 — Forward kinematics (done, 2026-09-25)

Full derivation: [`src/arm_kinematics/docs/forward_kinematics.md`](src/arm_kinematics/docs/forward_kinematics.md).

**Delivered:** `ForwardKinematicsSolver` (composes URDF joint transforms directly, walking `parent_joint` links from `tip_link` to `root_link` rather than hardcoding chain order — naturally includes the fixed `joint_intermediate` joint; geometry parsed from `urdf::Model` at construction, not duplicated as constants). `fk_node` wraps it: subscribes `/joint_states_in`, publishes `/computed_link5_pose` on a plain topic (deliberately not TF — `robot_state_publisher` already broadcasts the same frame from the same URDF; a second broadcaster would conflict with it). `launch/fk_visualize.launch` adds rviz visualization (sim + `fk_node` + rviz, `/computed_link5_pose` overlaid as its own axes display on `link_5`'s TF frame), with `use_rviz:=false` for headless runs.

**Validated:** gtest against an independently-computed reference (plain numpy, not this package's code) for zero-config and a multi-joint config, plus a hand-provable structural invariant (joint_1 conjugates the whole downstream pose by `Rz(θ)`, since its axis is z and it's the first transform in the chain). Headless rostest compares `/computed_link5_pose` against ground-truth TF across zero-config, all 10 joint-limit extremes, and 15 random in-limit samples, to ≤1e-4 m / 1e-3 rad. Manually confirmed in rviz (both build/test and the visual GUI check independently confirmed).

**Known limitation carried forward:** `robot_state_publisher` warns that `base_link` has an inertia KDL doesn't support on a root link — harmless (TF is correct regardless, confirmed by every passing test), pre-existing in the URDF, not fixed (out of scope, cosmetic warning only).

---

## M2 — Inverse kinematics (done, 2026-09-25)

Full derivation: [`src/arm_kinematics/docs/inverse_kinematics.md`](src/arm_kinematics/docs/inverse_kinematics.md).

**Delivered:** analytical geometric Jacobian (`J_v,i = z_i × (p_e−p_i)`, `J_ω,i = z_i`), cross-checked in gtest against an independent finite-difference Jacobian of M1's FK. Damped least-squares (Levenberg-Marquardt) IK on top: task space is 6D over 5 joints (overdetermined — the roadmap sketch's draft formula had this backwards, using the redundant-manipulator right-pseudo-inverse form; corrected to the left/normal-equations form). `InverseKinematicsSolver::Solve` accepts a full 6D target pose and minimizes weighted position+orientation error via damped least squares, returning the residual (including any structurally-unreachable component) rather than hiding it.

**Two things needed that weren't anticipated in this section's original sketch, both confirmed empirically on a 20-random-target benchmark:**
- **Adaptive damping**, not fixed: a single fixed `λ` converged only 40% of the time (one damping value can't serve both the far-from-target and near-convergence regimes well).
- **Random-seeded restarts on non-convergence**: adaptive damping alone converged *worse*, ~35% — this chain is redundant (1 yaw + 4 parallel-axis "elbow" joints, confirmed structurally, see M1's docs), so genuinely distinct solution basins exist and no amount of better local descent from one fixed seed finds a solution in a different basin. Restarts (`IkOptions::max_restarts`, default 20) fixed it to 100%, typically within 0–2.

**Also found and fixed mid-milestone: `joint_6` was wrongly modeled as revolute in `arm_interface`'s URDF.** M2's target needed to be `end_effector` (the actual tool point), not `link_5` — but `joint_6`'s translation was along the *same axis* it rotated about, so its position was already gripper-angle-independent while its orientation spun with the (physically unrelated) gripper motor. Fixed by making `joint_6` fixed in the URDF (`arm_interface`, separate commit) — matches what motor 6 actually does (gripper open/close, not a tool-frame rotation) and needed zero changes to the FK/IK solver itself, since it already treats any fixed chain joint as a constant offset. `fk_node`/`ik_node` now default to `end_effector`, not `link_5`.

**Validated:** gtest (Jacobian cross-check, IK∘FK round trip on random reachable targets, explicit unreachable case, numerical-stability stress case) + headless rostest closing the loop through the real simulator (target from the sim's own TF, solved via `/solve_ik`, driven back, confirmed against TF again, with one retry allowed given the solver's restart-based probabilistic nature). Manually confirmed live via `/solve_ik` + rviz, twice — once before the `end_effector` fix (showed `link_5` was wrong target), once after (confirmed `end_effector` lands exactly on target).

**Known limitation carried forward:** the position-only-reachable envelope changed slightly with the `end_effector` offset (some positions reachable at `link_5` aren't at the shifted `end_effector`, and vice versa) — expected geometrically, not a bug, not separately characterized/mapped.

---

## M3 — Trajectory generation (done, 2026-09-26)

Full derivation: [`src/arm_trajectory/docs/trajectory_generation.md`](src/arm_trajectory/docs/trajectory_generation.md).

**Delivered:** quintic time-scaling (`s(τ)=10τ³−15τ⁴+6τ⁵`, zero velocity+acceleration at both endpoints), with the velocity-limit-derived minimum duration `T_min,i = (15/8)|Δq_i|/v_max,i` (peak of `s'(τ)` at `τ=0.5` equals exactly `15/8`, confirmed by numerically scanning rather than trusting the closed form alone). Joint-space trajectories are exact and fast (no IK). Cartesian-path trajectories interpolate position-lerp + orientation-SLERP, warm-started per-sample IK. Execution via a proper `control_msgs/FollowJointTrajectoryAction` server (`trajectory_executor_node`), not the originally-planned blocking service — changed during planning specifically because M4 needs preemption (a new joystick target arriving mid-motion) and M5 needs a cancellation path for hardware safety; building a custom blocking interface first would've meant redoing it as an action later anyway. New package `arm_trajectory`, confirmed as separate from `arm_kinematics` per your explicit instruction going into the Plan step.

**Four real findings during implementation, none assumed up front:**
1. **`{0,0,0,0,0}` is a genuine kinematic singularity** — confirmed via Jacobian SVD (smallest singular value ~1e-33): all four parallel-axis "elbow" joints sit at a limit boundary simultaneously there (fully-extended-arm singularity). Don't seed IK/Cartesian generation from it.
2. **An interpolated Cartesian path between two exactly-reachable poses is generally NOT itself exactly reachable** — 5 DOF over a 6D task space (M2 finding) means the reachable set is a submanifold, and interpolating between two points on it doesn't stay on it. Diagnostic signature that confirmed this wasn't a bug: every restart from every seed converged to the *same*, path-position-dependent residual. Resolved with you: interior samples are **best-effort** (never fail the call), only the goal must be exactly reachable; peak deviation is reported in the result.
3. **Restarts must be disabled for interior path samples** — otherwise `Solve()`'s restart logic can silently replace a warm-started result with a discontinuous one from an unrelated elbow configuration, which then blew up the velocity-based duration-rescaling loop into generating an exploding number of samples (a test hung for 15+ minutes before this was caught). Fixed with `max_restarts=0` for interior samples plus a defensive sample-count sanity cap.
4. **Large Cartesian moves are slow to generate** (iterative duration-rescaling × per-sample IK) — documented as a known limitation, not solved in this milestone; worth revisiting if M4's interactive use needs faster response.

**Validated:** gtest (quintic math cross-checked against finite-differences and a numerically-scanned peak; joint-space synchronized arrival and velocity-limit satisfaction; Cartesian round-trip, velocity-limit satisfaction after rescaling, warm-start continuity, unreachable-goal handling) + headless rostest driving the real simulator through the actionlib interface end to end, including an actual mid-execution preemption test (cancel, confirm playback stops early — not just structurally present). Full suite repeated 3× for flakiness given randomized restarts throughout the stack. Manually confirmed live, twice: once via direct `rosservice`/`actionlib` calls, once via the `manual_trajectory_demo.py` hand-off script (added after the fact — the service response is a full multi-point `JointTrajectory`, impractical to hand-type through `rosservice call` the way M1/M2's simpler responses were).

**Known limitation carried forward:** Finding 4 above (large-Cartesian-move generation speed) — no fix attempted, flagged for reconsideration if M4 needs it.

---

## M4 — Non-interactive reach + base-collision avoidance

**Original goal (superseded):** joystick teleop in a new `arm_teleop` package. A prototype was built (`arm_teleop`: `teleop_node.cpp`, `joy_target_mapper.h`, docs, tests) but set aside in favor of the scope below — **ignore `arm_teleop`**, it is not part of this milestone's delivered or ongoing work.

**Delivered (2026-09-29), on `arm_trajectory`:**
- `/reach_target_pose` (non-interactive go-to-pose: orchestrates `/generate_trajectory` + `follow_joint_trajectory`, verifies the result via FK rather than trusting "generation succeeded") and two ways to drive it interactively without a joystick — an rviz 6-DOF interactive marker and `reach_pose_cli.py`.
- IK across the whole day-to-day path (`trajectory_service_node`, `GenerateCartesianTrajectory`) switched from `InverseKinematicsSolver` (full 6D pose) to `PositionPitchIkSolver` (position + pitch only — roll/yaw are mechanically irrelevant on this arm), which opened a real null-space the arm now uses **by default** to regulate `link_4` toward vertical on every reach, not just when asked.
- Found and fixed, all live-tested against the real simulator: null-space iteration starvation, a `converged`-staleness bug, a solver step-acceptance bug causing unbounded wandering on unreachable interior path samples, and a restart/selection bug that discarded a well-aligned seed for a random, poorly-aligned "exactly converged" one — see `src/arm_kinematics/docs/position_pitch_ik.md` for the full account of each.
- `use_hardware` toggle added to every sim-backed launch file (swaps `arm_sim_validator_node` for the real `robot_interface` driver, same topic interface either way); `use_rviz` added where missing. Hardware path wired but not exercised.
- A pass simplifying duplicated joint-state-lookup and distribution-sampling code across both packages, and `arm_interface`'s `joint_6` (gripper) reintroduced as a URDF branch off `end_effector` — invisible to every kinematics solver by construction, driven by a single linear (prismatic) `<mimic>`-paired two-finger joint.

Full test suites green throughout (`arm_kinematics` 86, `arm_trajectory` 51, `arm_interface` 11).

---

**Base-collision avoidance, delivered (2026-10-01):**

- New `arm_kinematics/base_collision.h`/`.cpp`: `BoxObstacle` (swappable value type, not baked into the URDF) + `CapsuleIntersectsBox` (exact, closed-form, no iteration — minimizes a convex piecewise-quadratic along the segment) + `ArmIntersectsBase` (reuses `ForwardKinematicsSolver`'s existing accessors, no new FK machinery). Full derivation: [`src/arm_kinematics/docs/base_collision.md`](src/arm_kinematics/docs/base_collision.md).
- `PositionPitchIkOptions`/`Solve()` made collision-aware: an opt-in `collision_obstacle` pointer (same "pointer=off" convention as the null-space task), checked once per restart candidate, reusing the *existing* restart/selection machinery rather than a parallel rejection path — exactly the resolved open question below. `GenerateCartesianTrajectory` threads it through via the same `PathTrackingIkOptions` copy the null-space fields already ride on, but treats interior-sample collision as a **hard failure** (not best-effort like tracking deviation) since interior samples run with `max_restarts=0` and retiming can't fix a geometrically-colliding path — see `arm_trajectory/docs/trajectory_generation.md` Finding 6.
- `trajectory_service_node` wires it on by default (`~enable_base_collision_check`, `~base_obstacle_half_extent`, `~collision_capsule_radius`), box placement derived from the corrected open-question answer below.
- 21 new tests (10 `test_base_collision.cpp`, 3 `test_position_pitch_ik.cpp`, 3 `test_cartesian_trajectory.cpp` — plus pre-existing suites, all still green: `arm_kinematics` 52 total, `arm_trajectory` 25 total). Verified live against the running simulator, not just gtest: restarted `trajectory_service_node`, confirmed `base-collision check on (0.400000m cube)` at startup, drove `/generate_trajectory` through both a successful live reroute and an honest (never silent) rejection.
- **Mid-plan correction, worth recording:** the box placement open question below was originally answered "centered on `base_link`'s own origin" (all 3 axes). The user corrected this during the formal Plan review: the robot is centered on the cube in X/Y but sits **on top of** it in Z (box top face at `base_link`'s origin, extending downward). This reverses which segments near the root are provably collision-free vs. which must be checked — see `base_collision.md`'s own "earlier, wrong placement assumption" section for the full before/after.
- **Live finding, not a regression:** this session's own recurring test pose (`position=(0.15,0.1,-0.2)`) turns out to sit inside the deployed box — a real discovery, not a bug introduced by this work. See `base_collision.md`.

The 4 original "Open questions — resolved" below are superseded by the corrected placement above where they conflict; kept for the historical record of what was decided before implementation:
1. ~~Box placement: centered on `base_link`'s own origin, all 3 axes~~ — corrected mid-plan, see above.
2. Regulation scope: generation-time checking only — confirmed, no separate runtime monitor was added.
3. Safety margin: the 40cm box dimension is literal; capsule radius (0.02m, from the URDF's own max link radius) provides the actual margin — confirmed as built.
4. Collision response: not a hard failure on the first colliding attempt at the goal — confirmed as built, with one measured nuance: the restart search's "qualifying candidate always wins" policy means an infeasible target (reachable only by colliding) reports `collision_free=true, converged=false` rather than "found the least-bad colliding match" — see `base_collision.md`'s "qualifying-tier-dominates" section.

## M5 — Sim-to-real

Run M1–M4 (FK/IK/trajectory generation+execution, base-collision avoidance) on real hardware through `arm_interface`/`DynamixelSDK`, under the project's hardware safety rules (joint/velocity/torque limits enforced on every hardware-facing command path, reduced speed on first real-hardware run of any new motion, explicit stop/disable-torque path, user runs anything touching motors). No hardware connected yet -- this section is prep/architecture only, detailed before any live-motor work per the project's established per-milestone pattern.

**Correction to this milestone's own framing:** M1-M4 were validated against `arm_sim_validator_node` (`arm_interface`'s own lightweight, URDF-limit-clamped kinematic simulator -- no Gazebo dependency, confirmed via `arm_interface/docs/architecture.md`), not Gazebo Classic. `use_hardware` already toggles the same `/joint_states_in`/`/joint_states_out` topic contract between `arm_sim_validator_node` and `robot_interface` (M4's delivery), so M1-M4's own code needs zero changes for M5 -- this milestone is entirely about `robot_interface`'s own hardware-facing safety path, which predates this session and has never been reviewed here.

**Audit finding (read-only review of `arm_interface/src/robot_interface.cpp`, `config/ArmInterface.yaml`, `urdf/arm.urdf`, launch files -- no code changed): `use_hardware:=true` is compile-tested only, never run against real motors, and has real safety gaps that must close before it is:**

1. **The one runtime safety interlock is inert.** A commanded-vs-actual position mismatch check exists (`robot_interface.cpp:248-260`) but its threshold defaults to 10 rad (`config/ArmInterface.yaml`) -- larger than any joint's full range (max ~5.5 rad per `docs/architecture.md`), so it can never fire.
2. **No velocity or torque/current limiting on outgoing commands at all.** Every write is an instantaneous goal-position setpoint; no Dynamixel profile-velocity register is configured anywhere, so a trajectory sample (or a bug) producing a large jump has nothing stopping it hardware-side except whatever the motor firmware defaults to.
3. **No software or hardware e-stop.** Only options today: kill the ROS node (destructor retry-disables torque, but only up to 5 attempts during shutdown before giving up and logging CRITICAL), the `/arm_interface/set_torque` service, or the physical breaker (per `README.md`, explicitly the slowest option of the three).
4. **Joint limits live in two unsynced places.** `arm_sim_validator_node` reads limits from the URDF (radians); `robot_interface` uses a hand-maintained encoder-tick array in `config/ArmInterface.yaml`, never cross-checked against the URDF. Measured drift: joints 1-3 agree to ≤0.02 rad, joint_4 ~0.017 rad off, joint_5 ~0.046 rad off -- not alarming alone, but an unverified, silently-divergent source of truth.
5. **`joint_6` (gripper) has a real unit-domain mismatch, not just a documentation gap.** The URDF models it as prismatic (meters, 0-0.025 m range -- M4's gripper reintroduction) with an explicit in-URDF comment flagging "no calibrated source anywhere in this repo." `config/ArmInterface.yaml`'s bias/sign/encoder bounds for that same index still encode the *old rotary* convention (±1.29 rad range). `robot_interface.cpp` applies one uniform radian-per-tick formula to all 6 joints -- there is no per-joint unit distinction, so a meters-domain command for joint_6 would be silently misinterpreted as radians.
6. **No calibration step is wired in.** `find_limits.cpp` (manual min/max tracking while a human moves the arm, printed on SIGINT) exists but isn't integrated into any launch file or verified against the current URDF/YAML pair.
7. **Bulk-write has no failure escalation** (unlike bulk-read's `max_read_failures_` threshold) -- a single failed write just logs and the control loop continues.

**Proposed staged plan** (no live motor testing happens before Stage 1; user runs every stage that touches motors, per the project's hardware-safety rule):

- **Stage 0 -- harden `robot_interface`'s safety path (code only, no hardware needed):** fix the inert mismatch threshold to something that can actually catch a runaway command; add profile-velocity configuration derived from the URDF's own per-joint velocity limits (the same `GetActuatedJointVelocityLimits()` M1-M4 already use); add a software e-stop (topic or service, independent of the command-timeout watchdog, callable by any node or a human); resolve the joint_6 unit mismatch (decide in/out of scope for M5 -- see open questions); either derive `robot_interface`'s encoder bounds from the URDF at startup instead of a second hand-maintained array, or add a loud startup consistency check; add a consecutive-write-failure threshold mirroring the existing read one.
- **Stage 1 -- read-only verification:** `robot_interface` running, torque disabled, confirm `/joint_states_in` reports plausible real positions and that the zero/sign calibration actually matches the physical arm at a few known poses.
- **Stage 2 -- first live motion:** torque enabled, tiny single-joint moves through the now-velocity-limited write path, reduced speed, operator present and confirming each move.
- **Stage 3 -- full M1-M4 pipeline at reduced speed:** `/reach_target_pose`/`/generate_trajectory` end-to-end against real hardware, global velocity scale-down, e-stop armed, operator-supervised.
- **Stage 4 -- normal-speed operation**, once Stage 3 has run cleanly enough times to build confidence.

**Decisions on the 7 audit findings** (resolved, replacing the open questions this section used to carry — full technical account in `arm_interface/docs/sim_to_real.md`):

1. **Inert mismatch check (finding 1): left as-is.** Judged not worth hardening on its own — the velocity clamp and e-stop interface below cover the scenarios that actually matter more directly. Revisit only if real-hardware testing finds a gap neither of those would catch.
2. **No velocity limiting (finding 2): addressed via a second hardware interface, not a retrofit of the first.** New `robot_interface_velocity.cpp` (Dynamixel Velocity Control Mode) with a real per-joint velocity clamp (`~max_velocity_rad_s`, default 0.1 rad/s — deliberately slow). Nothing upstream commands velocity yet (explicitly deferred); this is hardware-facing scaffolding only, toggled via `arm_interface.launch control_mode:=velocity`. The position-mode `robot_interface` itself is unchanged on this point.
3. **No e-stop (finding 3): implemented 2026-10-02 on position-mode `robot_interface`** as a latched "hold in place" (goal := fresh present position, torque on, commands ignored, `estop_reset` to release; `arm_interface/docs/sim_to_real.md` section 15). Latch logic unit-tested; verified on hardware: at rest, and mid-motion (arm stopped within ~0.3 s, <5 mrad travel, no creep, at 0.1 velocity scale); the lock-after-sag case is not yet observed. Velocity node still has the stub.
4. **Unsynced joint-limit sources (finding 4): confirmed coherent, no fix needed.** Centering/scaling checked and found consistent between the URDF and YAML; the small measured drift (≤0.046 rad) is within acceptable tolerance.
5. **`joint_6` unit mismatch (finding 5): fixed as a real conversion, not a type reversion.** The gripper stays prismatic (correct — the real motion is linear) and keeps the real actuator (correct — it's actuated by rotation); the missing piece was the conversion between those two domains, now `include/arm_interface/gripper_conversion.h`, applied at the ROS-interface boundary in both hardware nodes. Still an explicitly-flagged uncalibrated placeholder (affine approximation) pending a real mechanism measurement.
6. **No calibration step (finding 6): procedures proposed, none built.** Per instruction, not expected to be necessary — see `sim_to_real.md`'s own list (zero-offset check, startup URDF/YAML consistency check, gripper mechanism measurement) kept on hand rather than implemented.
7. **No write-failure escalation (finding 7): not yet addressed**, carried forward to Stage 0 if/when real-hardware testing resumes.

New files: `arm_interface/include/arm_interface/gripper_conversion.h` (+ test), `arm_interface/src/robot_interface_velocity.cpp`, `arm_interface/docs/sim_to_real.md`.

**Stage 1 run against real hardware (2026-10-01)**, first time any of this code touched the actual arm: read-only verification passed, and it caught a real bug — the gripper's reported open/close sense was opposite `arm_sim_validator_node`'s, fixed with an explicit `GripperConversion::invert` flag and confirmed matching afterward, all without any motor command. See `arm_interface/docs/sim_to_real.md` §5/§7 for both this and the full account of 8 below.

**A second gap found while preparing Stage 2** (first live motion, not yet attempted): `enableTorque()` ran unconditionally at startup, before any command was ever received — on a Dynamixel in Position Control Mode this means torque-enable alone could move the arm toward whatever stale value was already in its Goal Position register, independent of this node or any script. Never hit before because every prior session ran in `debug:=true` (torque-enable a no-op). Fixed: present position is read and re-written as the goal position *before* torque ever enables (`seedGoalPositionToCurrentPosition()`), same fix applied to `robot_interface_velocity.cpp`'s Goal Velocity register. New `arm_interface/scripts/stage2_single_joint_test.py` for the actual first move, not yet run.

`arm_interface` test suite: 13 total (watchdog + gripper-conversion, all green). Stage 2 itself has not started.

## M6 — Camera integration and calibration (sketch)

Intrinsics, camera-to-robot extrinsics.

## M7 — Target detection and 3D localization (sketch)

Start simple: colored object or ArUco marker.

## M8 — Closed loop (sketch)

Detect target → transform to robot frame → plan → reach.
