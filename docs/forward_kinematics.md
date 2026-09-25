# Forward kinematics — derivation

Chain: `base_link → link_1 → link_2 → link_intermediate → link_3 → link_4 → link_5`, from `arm_interface/urdf/arm.urdf`. 5 actuated (revolute) joints — `joint_1..joint_5` — plus one fixed joint, `joint_intermediate`, which contributes a constant transform with no joint variable. joint_6 (gripper) is outside this chain entirely, per the M0 scoping decision.

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

## Not covered here

No Jacobian/velocity kinematics — M2 (inverse kinematics) will need one if it goes the numerical-IK route; scoping that is M2's problem, not this milestone's.
