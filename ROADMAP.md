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
| M3 — Trajectory generation | Next, detailed below | `arm_trajectory` (new, proposed) |
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

## M3 — Trajectory generation (detailed)

**Goal:** given a start and goal (joint-space configuration, or Cartesian pose to be solved via M2's IK), generate a time-parameterized sequence of joint waypoints connecting them that respects each joint's velocity limit (from the URDF, same source-of-truth convention as M1/M2), for the simulator to execute — no dynamics/torque limits yet, matching M0–M4's sim-only scope.

**Scope:** 5-DOF joint-space, consistent with M0–M2. Point-to-point trajectories only (one start, one goal) — multi-waypoint paths/blending are a plausible later extension, not in this cut.

**Package (proposed, confirm in the Plan step):** new repo `arm_trajectory`, consuming `arm_kinematics` (M1/M2) as a dependency rather than extending it — trajectory generation is a distinct architectural layer (adds timing/velocity-profile concerns on top of instantaneous kinematics) even though M2 itself extended M1's package rather than starting a new one. Flagging this as a decision point, not assuming it.

**Approach:**
- **Joint-space profile: quintic (5th-order) time-scaling**, the standard smooth point-to-point profile (zero velocity *and* acceleration at both endpoints, avoiding the velocity discontinuities of a naive linear/trapezoidal profile at the start/end). For normalized time `τ = t/T ∈ [0,1]`:
  ```
  s(τ)  = 10τ³ − 15τ⁴ + 6τ⁵                    (s(0)=0, s(1)=1)
  s'(τ) = 30τ² − 60τ³ + 30τ⁴
  q(t) = q₀ + (q_goal − q₀)·s(t/T)
  ```
  Boundary conditions `s(0)=s'(0)=s''(0)=0` and `s(1)=1, s'(1)=s''(1)=0` hold by construction (standard quintic time-scaling, e.g. Siciliano *Robotics* ch. 4).

- **Velocity-limit-driven duration, derived (not assumed):** `s'(τ)` is maximized at `τ=0.5`: `s'(0.5) = 30(0.25) − 60(0.125) + 30(0.0625) = 7.5 − 7.5 + 1.875 = 15/8`. Since `q̇(t) = Δq · s'(t/T) / T`, peak joint velocity is `|Δq|·(15/8)/T`. Setting that equal to the joint's velocity limit `v_max` (from `ForwardKinematicsSolver::GetActuatedJointLimits()`'s URDF-source sibling — velocity limits need their own small accessor, same pattern) gives the **minimum feasible duration for that joint**:
  ```
  T_min,i = (15/8) · |Δq_i| / v_max,i
  ```
  Take `T = max_i(T_min,i)` and use the *same* `T` for every joint (synchronized arrival — standard point-to-point convention, avoids joints finishing at different times and needing independent replanning).

- **Cartesian-space trajectories**: interpolate the *pose* (position linearly, orientation via SLERP) between start and goal, sample at a fixed rate, and solve M2's IK at each sample — **warm-started from the previous sample's joint solution**, not from scratch each time. This isn't optional polish: since the arm is redundant (M2's finding), independent from-scratch IK at each waypoint could land in a *different* solution basin sample-to-sample, producing a trajectory that jumps between elbow configurations instead of moving smoothly. Warm-starting is what keeps consecutive samples in the same basin.
- **Reachability is checked before returning a trajectory, not discovered mid-execution**: every sampled waypoint (Cartesian mode) or just the endpoints (joint-space mode, trivially always "reachable" since both ends are joint configurations already) is validated; if any IK solve fails to converge, the whole trajectory request fails with a clear report of which waypoint and why, rather than executing a partial/broken path.

**Tests:**
- gtest: quintic time-scaling boundary conditions (`s(0)=0, s(1)=1, s'(0)=s'(1)=0`) and the derived `T_min` formula, checked against a numerical velocity peak (differentiate `s(τ)` numerically, confirm the peak matches `15/8` and occurs at `τ=0.5`) — same independent-cross-check pattern as M1/M2's Jacobian test.
- gtest: synchronized arrival — for a multi-joint goal, every joint reaches its target at exactly `T`, no joint's velocity exceeds its limit at any sampled `t`.
- gtest: Cartesian trajectory warm-starting — construct a case where independent per-waypoint IK (no warm-start) would plausibly jump basins (e.g. near a redundancy-heavy region) and confirm the warm-started version doesn't.
- Integration/rostest: command a trajectory through the real simulator, confirm `/joint_states_in` tracks the generated profile within tolerance throughout, not just at the endpoint.

**Done when:**
- Joint-space and Cartesian-space trajectory generation implemented and unit tested.
- Velocity limits provably respected (tested, not assumed) for every joint, every sampled time.
- Unreachable/infeasible requests reported before execution, not silently truncated.
- No changes to `arm_interface`/`DynamixelSDK`.
- Hand-off test script given, same pattern as M1/M2.

The formal **Plan** step (file list, exact interface shape, package decision, wait for approval before implementing) happens when M3 is actually started, per the working protocol — this section is the roadmap-level definition, not that plan.

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
