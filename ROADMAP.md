# Roadmap — [ROBOT NAME] incremental capabilities

Multi-repo catkin workspace. This file is the only thing tracked in the
workspace-root repo (`/home/rialiotta/dev/claude_ros_ws`); each package
under `src/` is its own independently-versioned git repo (`arm_interface`,
and one new repo per milestone that adds a package, per the "new capability
→ new package" rule).

| Milestone | Status | Package(s) |
|---|---|---|
| M0 — Orientation | **Done** (2026-09-25) | `arm_interface` (existing) |
| M1 — Forward kinematics | Next, detailed below | `arm_kinematics` (new) |
| M2 — Inverse kinematics | Sketch | `arm_kinematics` |
| M3 — Trajectory generation | Sketch | TBD |
| M4 — Joy-controller teleop | Sketch | TBD |
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

## M1 — Forward kinematics (detailed)

**Goal:** given the 5 arm joint angles (joints 1–5), compute the pose (position + orientation) of `link_5` in `base_link`, correct across the full joint-limit range, validated against the existing simulator's own kinematics.

**Scope:** 5-DOF only, per the M0 decision above. joint_6/gripper is explicitly out of scope for this milestone.

**Package:** new repo `arm_kinematics` under `src/`, git-initialized the same way `arm_interface` was for M0 (branch `milestone-1-forward-kinematics`). Does not modify `arm_interface` or `DynamixelSDK`.

**Approach:**
- Derive the kinematic chain's homogeneous-transform parameters (link offsets, joint axes) from `urdf/arm.urdf` — the URDF is already the source of truth for link geometry (per M0), so FK parameters should be derived from it once (e.g. via a small offline script or by hand from the `<origin>`/`<axis>` tags) rather than re-guessed, to avoid a second, driftable copy of the same numbers `bulk_read.cpp` already showed can go stale.
- Implement FK as a small, dependency-light C++ library (pure math, no ROS I/O in the core function — testable in isolation, per the project's control/HW-I-O separation convention) exposing something like `Pose ForwardKinematics(const std::array<double,5>& joint_angles)`.
- Thin ROS wrapper on top (node or service) for interactive use against `/joint_states_in` — exact interface (service vs. topic, message types) to be settled in the Plan step when M1 formally starts.

**Validation — against the simulator, as the roadmap requires:**
`robot_state_publisher` already computes FK from the same URDF (via KDL) and broadcasts it as TF whenever `arm_sim.launch`/`arm_sim_gui.launch` is running. Cross-check our FK implementation against that TF (`base_link → link_5`) as ground truth:
- Sweep each joint individually across its URDF limit range, several joints jointly at sampled/random configurations within limits, plus edge cases (all-zero, each joint at both limits).
- Assert position/orientation error against TF is at or near floating-point precision (e.g. <1e-6 m / rad) — this is two exact FK formulations being compared, not a noisy estimate, so tolerance should be tight.

**Tests (per project convention — every milestone gets automated tests):**
- gtest on the pure FK function: deterministic known poses (zero config; at least one hand-derived non-trivial case) — no ROS/hardware dependency, fast, always run.
- An integration-level test that launches the sim, drives joint configurations, and diffs FK output against TF — this is the "validated against the simulator" requirement made concrete and automated, not just a manual check.

**Done when:**
- FK library implemented and unit tested.
- Automated sim-vs-FK validation passes across the full joint range (not just a few manual spot checks).
- No changes to `arm_interface`/`DynamixelSDK`.
- Hand-off test script given: exact commands to run the sim + validation, what to observe, what indicates a problem.

The formal **Plan** step (file list, exact interface shape, wait for approval before implementing) happens when M1 is actually started, per the working protocol — this section is the roadmap-level definition, not that plan.

---

## M2 — Inverse kinematics (sketch)

Solve for joint angles 1–5 given a commanded end-effector (`link_5`) pose, in simulation. Handle unreachable targets and singularities explicitly (reject/report, don't silently return garbage). Builds directly on M1's FK (and likely its Jacobian, for a numerical IK approach) — package: `arm_kinematics`, same repo as M1.

## M3 — Trajectory generation (sketch)

Joint-space and Cartesian-space trajectories respecting position/velocity limits, in simulation. Package TBD (possibly `arm_kinematics` again, or a new `arm_trajectory` — decide when M2 closes).

## M4 — Joy-controller teleop (sketch)

User commands via a joy-compatible controller: manage trajectory targets, drive the arm (sim first). Package TBD.

## M5 — Sim-to-real (sketch)

Run M2–M4 on real hardware through `arm_interface`/`DynamixelSDK`, under the project's hardware safety rules (joint/velocity/torque limits enforced on every hardware-facing command path, reduced speed on first real-hardware run of any new motion, explicit stop/disable-torque path, user runs anything touching motors).

## M6 — Camera integration and calibration (sketch)

Intrinsics, camera-to-robot extrinsics.

## M7 — Target detection and 3D localization (sketch)

Start simple: colored object or ArUco marker.

## M8 — Closed loop (sketch)

Detect target → transform to robot frame → plan → reach.
