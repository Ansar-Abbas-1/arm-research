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
| M2 — Inverse kinematics | Next, detailed below | `arm_kinematics` |
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

## M1 — Forward kinematics (done, 2026-09-25)

Full derivation: [`src/arm_kinematics/docs/forward_kinematics.md`](src/arm_kinematics/docs/forward_kinematics.md).

**Delivered:** `ForwardKinematicsSolver` (composes URDF joint transforms directly, walking `parent_joint` links from `tip_link` to `root_link` rather than hardcoding chain order — naturally includes the fixed `joint_intermediate` joint; geometry parsed from `urdf::Model` at construction, not duplicated as constants). `fk_node` wraps it: subscribes `/joint_states_in`, publishes `/computed_link5_pose` on a plain topic (deliberately not TF — `robot_state_publisher` already broadcasts the same frame from the same URDF; a second broadcaster would conflict with it). `launch/fk_visualize.launch` adds rviz visualization (sim + `fk_node` + rviz, `/computed_link5_pose` overlaid as its own axes display on `link_5`'s TF frame), with `use_rviz:=false` for headless runs.

**Validated:** gtest against an independently-computed reference (plain numpy, not this package's code) for zero-config and a multi-joint config, plus a hand-provable structural invariant (joint_1 conjugates the whole downstream pose by `Rz(θ)`, since its axis is z and it's the first transform in the chain). Headless rostest compares `/computed_link5_pose` against ground-truth TF across zero-config, all 10 joint-limit extremes, and 15 random in-limit samples, to ≤1e-4 m / 1e-3 rad. Manually confirmed in rviz (both build/test and the visual GUI check independently confirmed).

**Known limitation carried forward:** `robot_state_publisher` warns that `base_link` has an inertia KDL doesn't support on a root link — harmless (TF is correct regardless, confirmed by every passing test), pre-existing in the URDF, not fixed (out of scope, cosmetic warning only).

---

## M2 — Inverse kinematics (detailed)

**Goal:** given a commanded `link_5` pose (position + orientation) in `base_link`, solve for joint angles 1–5 that best achieve it, reporting reachability explicitly rather than returning garbage for infeasible targets.

**Scope:** 5-DOF only (joints 1–5), consistent with M0/M1. Builds directly on M1's `ForwardKinematicsSolver` — package: `arm_kinematics`, same repo, branch `milestone-2-inverse-kinematics`. Does not modify `arm_interface`/`DynamixelSDK`.

**A structural fact that shapes this milestone:** joint_1's axis is `z` (base yaw); joints 2–5 all end up with *parallel* axes in the base frame — each of their origins only rotates about `y` before applying the joint's own `y`-axis rotation, and a rotation about `y` doesn't move the `y` axis itself. So this chain is effectively "1 yaw + 4 coplanar pitch joints": any reachable *position* is reachable, plus exactly one orientation angle (the end-effector's pitch within the vertical plane joint_1 selects) — but not full 3D orientation. **Decided:** the IK API still accepts a full 6D target pose (position + orientation) — natural for future callers like M8's vision pipeline, who shouldn't need to know the arm's internal DOF structure — and the solver minimizes a *weighted* position+orientation error via damped least squares rather than requiring an exact match. "Reachable" means the converged residual is under a documented tolerance; the residual itself (including any structurally-unreachable orientation component) is returned, not hidden.

**Approach:**
- **Analytical geometric Jacobian** (decided over numerical finite-difference — more work now, but exact and faster, avoiding a later revisit): for each actuated joint *i* with axis `z_i` and origin position `p_i` (both expressed in `base_link`, evaluated at the current configuration) and end-effector position `p_e`:
  ```
  J_v_i = z_i × (p_e − p_i)      (linear velocity column)
  J_ω_i = z_i                     (angular velocity column)
  J = [J_v; J_ω] ∈ R^(6×5)
  ```
  `z_i`/`p_i` come from the same accumulated transform chain `ForwardKinematicsSolver` already builds — extending it (or adding a sibling class) to expose per-joint intermediate transforms, not just the final pose, is the natural implementation path.
- **Damped least squares (Levenberg-Marquardt-style) iteration** from a seed configuration (default: current `/joint_states_in`, overridable):
  ```
  Δq = J^T (J·J^T + λ²I)^-1 · W · e
  q_{k+1} = clamp(q_k + α·Δq, joint_limits)
  ```
  where `e` is the weighted [position; orientation] error and `λ` provides numerical stability near singularities (e.g. joint_3/joint_4/joint_5 axes aligning). Iterate until `|e| < tolerance` (success) or max iterations exceeded (report unreachable, return best-effort configuration + residual, never silently wrong).

**Tests:**
- gtest: **analytical Jacobian validated against a numerical (finite-difference) Jacobian of M1's FK** at several configurations — cheap, independent cross-check that costs nothing since M1's FK already exists, catches analytical-derivation mistakes directly (sign/axis errors) without needing hand computation.
- gtest: IK-then-FK round trip — for many random reachable targets (generated *by* running FK on random valid joint configs, so reachability is known by construction), solve IK and verify `FK(IK(target)) ≈ target` within tolerance.
- gtest: explicit unreachable case (e.g. a position outside the arm's max reach) returns "not reached" with the max-iteration residual, not a false success.
- Integration/rostest: drive the sim through IK-computed joint commands for several targets, confirm `/joint_states_in` converges to the commanded pose (closing the loop through the actual simulator, not just the solver in isolation).

**Done when:**
- IK solver implemented, unit tested (including the Jacobian cross-check and IK∘FK round trip).
- Unreachable targets and near-singular configurations handled explicitly and tested, not just assumed.
- No changes to `arm_interface`/`DynamixelSDK`.
- Hand-off test script given, same pattern as M1.

The formal **Plan** step (file list, exact interface shape, wait for approval before implementing) happens when M2 is actually started, per the working protocol — this section is the roadmap-level definition, not that plan.

---

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
