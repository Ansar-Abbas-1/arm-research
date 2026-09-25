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
| M4 — Joy-controller teleop | Next, detailed below | `arm_teleop` (new, proposed) |
| M5 — Sim-to-real | Sketch | — |
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

## M4 — Joy-controller teleop (detailed)

**Goal:** drive the arm interactively via a joystick in simulation — the joystick moves a target (Cartesian pose, most naturally), and the arm continuously (re)plans and follows it, preempting an in-flight motion whenever a new target arrives. This is the intended consumer of M3's preemption support, not a new mechanism of its own.

**Scope:** simulation only (`arm_sim_validator_node`), per the roadmap; hardware teleop is M5's concern under the hardware safety rules. 5-DOF Cartesian jogging; joint_6/gripper toggled independently (a direct, non-kinematic command, consistent with every earlier milestone's treatment of it).

**Package (proposed):** new repo `arm_teleop`, depending on `arm_trajectory` (its `/generate_trajectory` service and `follow_joint_trajectory` action client) — a clear-cut case for the "new capability → new package" convention (an interactive control loop is a different kind of thing from trajectory math), unlike M3's more ambiguous case.

**Approach:**
- Subscribe to `sensor_msgs/Joy` (standard `joy` package output; a real joystick device is needed to *use* this, but automated tests substitute a synthetic publisher — see Tests).
- Maintain an internal target pose, updated incrementally from joystick axes at a fixed, throttled rate (e.g. 10 Hz, not raw joystick rate ~50-100 Hz): `target_{k+1} = target_k + gain · axis · dt`, clamped to a sane jogging speed. Simple integration, not a new control law — no derivation needed beyond what M1-M3 already built.
- On each update tick where the target actually changed: call `/generate_trajectory` (joint-space goal, i.e. `cartesian_path=false` — solve IK once for the new target, then a plain joint-space move to it; a full traced Cartesian path per tick isn't needed for jogging responsiveness, and avoids Finding 4's slow-path cost on every tick) and send the result to `follow_joint_trajectory`, which **preempts** whatever was already executing — exactly the mechanism M3 built and tested in isolation, now exercised by a real caller.
- A button (or axis) drives `joint_6` (gripper) directly — bypasses the kinematics/trajectory stack entirely, consistent with it never being part of the pose chain since M0.

**Tests:**
- Unit test: the joystick-axis-to-target-pose-increment mapping as a pure function (Joy msg + previous target + dt → new target) — no ROS/ hardware needed.
- Integration/rostest: publish synthetic `sensor_msgs/Joy` messages (no real joystick required) simulating a stick push, confirm the target updates and the sim tracks it; then publish a *second* synthetic input while the first motion is still executing and confirm the resulting action goal actually preempts the first (validating M3's preemption is exercised by a real consumer, not just its own isolated test).

**Done when:**
- Joystick input drives the simulated arm's end-effector target smoothly, jogging speed bounded.
- A new joystick input while a motion is executing correctly preempts it (tested, not assumed).
- Gripper (joint_6) toggles independently of the pose-tracking loop.
- No changes to `arm_interface`/`DynamixelSDK`.
- Hand-off test script given, same pattern as M1–M3 (a synthetic-Joy-publisher script, likely, given no real joystick hardware is assumed available for the reviewer either).

The formal **Plan** step (file list, exact interface shape, wait for approval before implementing) happens when M4 is actually started, per the working protocol — this section is the roadmap-level definition, not that plan.

## M5 — Sim-to-real (sketch)

Run M2–M4 on real hardware through `arm_interface`/`DynamixelSDK`, under the project's hardware safety rules (joint/velocity/torque limits enforced on every hardware-facing command path, reduced speed on first real-hardware run of any new motion, explicit stop/disable-torque path, user runs anything touching motors).

## M6 — Camera integration and calibration (sketch)

Intrinsics, camera-to-robot extrinsics.

## M7 — Target detection and 3D localization (sketch)

Start simple: colored object or ArUco marker.

## M8 — Closed loop (sketch)

Detect target → transform to robot frame → plan → reach.
