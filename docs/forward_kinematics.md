# Forward kinematics — derivation

Chain: `base_link → link_1 → link_2 → link_intermediate → link_3 → link_4 → link_5 → end_effector`, from `arm_interface/urdf/arm.urdf`. 5 actuated (revolute) joints — `joint_1..joint_5` — plus two fixed joints: `joint_intermediate` (mid-chain) and `joint_6` (`link_5 → end_effector`, the true tip). Both contribute a constant transform with no joint variable. `end_effector`, not `link_5`, is `fk_node`/`ik_node`'s default `tip_link` — see "Extending to end_effector" below for why `joint_6` is fixed rather than actuated.

The math below (per-joint transforms, the numeric cross-check, the joint_1 invariant) is all derived for the `base_link → link_5` sub-chain; the `link_5 → end_effector` extension is a separate, much simpler fixed offset, covered at the end.

## Assumptions

- Rigid-body kinematics only: links are perfectly rigid, joints have zero backlash/compliance, no dynamics (mass/inertia/torque) enters here — this module is position kinematics only.
- Joint values are commanded/measured angles in radians, matching `/joint_states_in`'s convention (see `arm_interface`).
- Geometry (per-joint translation, fixed rotation, and axis) is read from the URDF at runtime, not hardcoded — see rationale in `include/arm_kinematics/forward_kinematics.h`.

## Per-joint transform

Each joint in the chain contributes

```
T_i = Trans(xyz_i) · Rot(rpy_i) · Rot(axis_i, θ_i)
```

where `Trans`/`Rot(rpy)` are the joint's fixed `<origin>` (parent→joint frame), and `Rot(axis_i, θ_i)` is the variable joint rotation (identity for the one fixed joint in the chain, `joint_intermediate`).

URDF's `rpy` is fixed-axis (extrinsic) roll-pitch-yaw about X, Y, Z of the *parent* frame, composing as

```
Rot(rpy) = Rz(yaw) · Ry(pitch) · Rx(roll)
```

(matches `tf2::Quaternion::setRPY` / what `robot_state_publisher` and every other TF-based tool assume). The implementation doesn't build this from the rpy text itself — it reads `urdf::Joint::parent_to_joint_origin_transform.rotation`, which `liburdfdom` has already parsed into a quaternion using exactly this convention, removing one source of transcription error.

**Simplifying observation for this specific arm:** every joint's origin `rpy` in `arm.urdf` has roll = yaw = 0 — only pitch is ever nonzero. So every fixed origin rotation in this chain reduces to a pure `Ry(pitch)`:

| joint | xyz (m) | rpy (rad) | axis | actuated |
|---|---|---|---|---|
| joint_1 | (0, 0, 0.03) | (0, 0, 0) | z | yes |
| joint_2 | (0, 0, 0.065) | (0, −1.57, 0) | y | yes |
| joint_3 | (0, 0, 0.34) | (0, 0, 0) | y | yes |
| joint_intermediate | (0.029, 0, 0) | (0, 3.1412, 0) | — | no (fixed) |
| joint_4 | (0, 0, 0.376) | (0, −1.8791, 0) | y | yes |
| joint_5 | (0, 0, 0.336) | (0, 1.81, 0) | y | yes |

Full pose: `T(base_link→link_5) = T_1 · T_2 · T_3 · T_intermediate · T_4 · T_5`.

## Independent numeric cross-check

Reference values used in `test/test_forward_kinematics.cpp` were computed by a second, independent implementation of the same equations (plain numpy, not this package's Eigen/C++ code) — [`fk_reference.py`](fk_reference.py), included here for reproducibility. Two cases:

- **Zero configuration** (`q = [0,0,0,0,0]`): position `(−0.065804339, 0, 0.444317584)` m.
- **Nontrivial configuration** (`q = [0.3, 0.5, −0.2, 0.4, −0.3]`): position `(0.189062287, 0.058483819, 0.48518007)` m.

Full rotation matrices are in the script's output / the gtest source.

## A hand-provable structural invariant

`joint_1`'s axis is `z` and it is the *first* transform in the chain (rotates about a fixed point at the base origin), so for any values of joints 2–5:

```
Pose(θ, q₂, q₃, q₄, q₅) = Rz(θ) · Pose(0, q₂, q₃, q₄, q₅)
```

i.e. rotating joint_1 by θ is exactly a rigid rotation of the *entire* downstream pose about the base Z axis. This holds independent of the exact numeric link geometry, so it's a strong correctness check on composition order (a bug that applied joint_1 on the wrong side of the product, or out of order relative to the other joints, would break this even if it happened to pass the two numeric spot-checks above). Tested in `ForwardKinematicsTest.Joint1RotatesWholeChainAboutBaseZ`.

## Extending to end_effector

`joint_6` (`link_5 → end_effector`) was originally modeled in the URDF as **revolute** about the same axis its own `<origin>` translates along (`xyz="0 0 0.105"`, `axis="0 0 1"`). For a translation along the *same* axis a joint rotates about, rotation and translation commute and — more importantly — rotating a point that already sits on the rotation axis doesn't move it. So `end_effector`'s *position* relative to `link_5` was always exactly `(0, 0, 0.105)` along `link_5`'s own z-axis, **independent of `joint_6`'s angle**; only its *orientation* would spin with `joint_6`. That's physically backwards for what `joint_6` actually is: motor 6 actuates gripper open/close, not a rotation of the tool frame. Modeling it as revolute made `end_effector`'s orientation depend on an unrelated/unknown gripper angle for no kinematic benefit (position was unaffected either way) — this surfaced directly when M2's IK needed to target `end_effector` and hit a case where an achievable position combined with a *specific* required orientation had no exact solution with `joint_6` in the mix.

Fixed by making `joint_6` a **fixed** joint in `arm.urdf` (translation-only, no `<axis>`/`<limit>`). `ForwardKinematicsSolver` already treats any FIXED joint in the requested chain as a constant transform excluded from `GetActuatedJointNames()` — the same machinery that already handled `joint_intermediate` — so extending `tip_link` from `link_5` to `end_effector` needed zero changes to the solver itself, only to the URDF and the ROS nodes' default `tip_link` param. `end_effector`'s pose is now simply:

```
T(base_link→end_effector) = T(base_link→link_5) · Trans(0, 0, 0.105)
```

exactly, for any joint configuration — verified in `ForwardKinematicsConstruction.EndEffectorChainAddsFixedOffsetPastLink5`.

## Not covered here

No Jacobian/velocity kinematics — M2 (inverse kinematics) will need one if it goes the numerical-IK route; scoping that is M2's problem, not this milestone's.
