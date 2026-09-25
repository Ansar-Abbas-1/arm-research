# arm_interface — Architecture (M0 Orientation)

Robot: 6-motor Dynamixel arm, PRISMA Lab, mounted on ANYmal. ROS Noetic.
Two repos in this workspace:

- **`DynamixelSDK`** — vendored ROBOTIS hardware SDK (`noetic-devel` branch). Read-only reference; never modified by this project.
- **`arm_interface`** — this package: real-hardware I/O node, calibration/replay utilities, and a physics-free kinematic simulator.

## Kinematic structure

Serial chain, 6 revolute joints + 1 fixed offset link:

```
base_link → joint_1 → link_1 → joint_2 → link_2 → joint_3 → link_intermediate
          → joint_4 → link_3 → link_4 → joint_5 → link_5 → joint_6 → end_effector
```

(`joint_intermediate` between `link_2` and `link_3` is a fixed geometric offset, not an actuated DOF — URDF comment: "mimic actual structure without weird movement and axes".)

**joint_6 actuates the gripper, not arm pose.** This is a 5-DOF positioning arm + 1 gripper DOF, not a 6-DOF arm for kinematics purposes. This directly scopes forward/inverse kinematics (M1/M2): the pose chain to solve is joints 1–5 (to `link_5`), with joint_6/gripper handled as an independent open/close DOF. The URDF's `robot name="five_dof_arm"` is consistent with this (arm pose = 5 DOF), not the stale artifact it first appeared to be.

### Joint limits (from `urdf/arm.urdf`, radians)

| joint | role | axis | lower | upper | effort (URDF) | velocity (URDF) |
|---|---|---|---|---|---|---|
| joint_1 | arm | z | -2.745 | 2.745 | 10 Nm | 2 rad/s |
| joint_2 | arm | y | 0 | 3.14 | 10 Nm | 2 rad/s |
| joint_3 | arm | y | -2.220 | 0 | 10 Nm | 2 rad/s |
| joint_4 | arm | y | 0 | 4.505 | 10 Nm | 2 rad/s |
| joint_5 | arm | y | -3.62 | 0 | 10 Nm | 2 rad/s |
| joint_6 | gripper | z | -1.29 | 1.29 | 10 Nm | 2 rad/s |

The URDF's effort/velocity are flat placeholders (10 Nm / 2 rad/s for every joint), **not** per-motor values. Real per-motor figures live in `config/motors.yaml` (comments only, not loaded at runtime):
- joints 1, 4, 5, 6: Dynamixel **MX-28AT** — effort 3.1 Nm, velocity 6.0 rad/s, mass 0.077 kg
- joints 2, 3: Dynamixel **MX-106** — effort 8.4 Nm, velocity 4.7 rad/s, mass 0.153 kg
- Total target mass: 1.815 kg

**Position-control only.** `config/ArmInterface.yaml` configures goal/present position registers (116/132) and torque-enable (64); no velocity, current, or torque control mode is configured anywhere in this package.

Motor config: IDs `[1..6]`, protocol 2.0, baud 1000000, device `/dev/ttyUSB0`.

## Interface boundary to DynamixelSDK

Only three source files touch the SDK directly, all via `#include <dynamixel_sdk/dynamixel_sdk.h>`:

- **`src/robot_interface.cpp`** (class `DynamixelArmInterface`) — the real-hardware bridge node. Uses `PortHandler`, `PacketHandler`, `GroupBulkRead`, `GroupBulkWrite`.
- **`src/bulk_read.cpp`** — read-only hardware diagnostic (bulk-reads present position only, no writes).
- **`src/find_limits.cpp`** — links `dynamixel_sdk` but doesn't call it; consumes topics published by `bulk_read` instead.

`dynamixel_sdk` is a normal catkin package (`DynamixelSDK/ros/dynamixel_sdk/`), pulled in via standard `find_package(catkin REQUIRED COMPONENTS dynamixel_sdk ...)` / `<depend>dynamixel_sdk</depend>` — same workspace build, no special step. Its public surface: `PortHandler`/`PacketHandler` (port open/baud/raw byte I/O, ping/reboot, single-register read/write) and `GroupBulkRead`/`GroupBulkWrite`/`GroupSyncRead`/`GroupSyncWrite` (multi-motor batched I/O). No symbolic register-name API — addresses are raw ints per the Dynamixel model's control table (ROBOTIS e-manual, not vendored here).

## Nodes

| Node (target) | Source | Hardware? | Purpose |
|---|---|---|---|
| `robot_interface` | `robot_interface.cpp` | yes | Bridges real motors ↔ ROS. Bulk-reads present position + hw error status @100Hz → `/joint_states_in`; subscribes `/joint_states_out` → bulk-writes goal positions. Torque-enable on start, watchdog disables torque on command timeout or destruction. |
| `bulk_read` | `bulk_read.cpp` | yes | Read-only diagnostic: publishes raw + converted positions. Used as the "does the arm respond" smoke test and as the record step for replay-mode calibration. |
| `find_limits` | `find_limits.cpp` | no (consumes `bulk_read`'s topics) | Calibration utility: tracks running min/max of `/motor_N_position`, prints on SIGINT. |
| `replay_bag` | `run_bagfile.cpp` | no (shells to `rosbag play`) | Interactive helper to pick and replay a recorded bag against `/joint_states_out`. |
| `arm_sim_validator_node` | `arm_sim_validator_node.cpp` | **no — pure kinematic sim** | Subscribes `/joint_states_out`, publishes `/joint_states_in`. No `dynamixel_sdk` dependency; parses joint limits from the `robot_description` URDF param. Two modes: `ideal` (instant passthrough) / `lag` (first-order lag, τ default 0.1s). Clamps output to URDF limits. |

### Topic/service graph (real hardware and sim both use the same topic names)

- `/joint_states_out` (`sensor_msgs/JointState`, commanded) → consumed by `robot_interface` (hardware) or `arm_sim_validator_node` (sim)
- `/joint_states_in` (`sensor_msgs/JointState`, actual/simulated state) → published by whichever of the above is running; consumed by `robot_state_publisher` for TF/rviz
- `robot_interface` also publishes per-joint `/motor_N_position_in` / `/motor_N_position_out` (`std_msgs/Int32`, raw ticks) and exposes services `/arm_interface/reenable_failed_motors` (`std_srvs/Trigger`), `/arm_interface/set_torque` (`std_srvs/SetBool`)

This shared topic contract is what makes `arm_sim_validator_node` a drop-in stand-in for `robot_interface` during development — any downstream node (teleop, trajectory generator, etc.) built against `/joint_states_out` → `/joint_states_in` works unchanged against sim or hardware.

## Running the simulation

```bash
roslaunch arm_interface arm_sim_gui.launch   # GUI slider control, sim_mode:=lag by default
# or
roslaunch arm_interface arm_sim.launch       # no GUI node; publish /joint_states_out yourself, sim_mode:=ideal by default
```

Both load the URDF, start `robot_state_publisher` (remapped to `/joint_states_in`), start `arm_sim_validator_node`, and start rviz with `rviz/arm.rviz`. `arm_sim_gui.launch` additionally starts `joint_state_publisher_gui` (remapped to publish `/joint_states_out`).

Verified this session (M0), headless — no X11 available in this session's container access, so rviz/GUI-slider interaction wasn't observed visually; the underlying control loop was verified directly via `rostopic pub`/`echo` against `arm_sim_validator_node` run standalone with the same params the launch file uses:
- Commanded `[0.5, 1.0, -1.0, 2.0, -1.5, 0.3]` rad → `/joint_states_in` converged to that pose (lag mode, τ=0.1s).
- Commanded joint_1 = 5.0 rad (beyond its 2.745 rad limit) → clamped to exactly 2.745 rad in `/joint_states_in`.

**You should confirm the GUI/rviz path yourself** (`roslaunch arm_interface arm_sim_gui.launch`, move a slider, watch the arm move in rviz) since that visual path wasn't exercised here.

## Build

`catkin_make` in this workspace (container: `claude`, `osrf/ros:noetic-desktop-full`, mounts this workspace at `/home/dev`, plus `/dev/bus/usb` and `/tmp/.X11-unix` for hardware/GUI access) now succeeds and builds all 5 `arm_interface` executables plus `dynamixel_sdk`/`dynamixel_sdk_examples`. It did **not** succeed as found — see Known Limitations.

## Test coverage

**None.** No gtest/rostest, no `test/` directory, no active `catkin_add_gtest`/`catkin_add_nosetests` targets (only commented-out boilerplate in `CMakeLists.txt`). This is a gap to close starting M1, per the project's working protocol (every milestone gets automated tests).

## Known limitations (found as-is, not fixed beyond the one build-blocker below)

1. **Fixed this session (build-blocking):** `CMakeLists.txt` called `catkin_install_python()` on `scripts/arm_gazebo_bridge_node.py`, which doesn't exist — this failed CMake *configure*, blocking the entire build (not just `install`). Commented out the three lines with an explanatory comment; nothing else changed. If this script is meant to exist (Gazebo bridge, tying to the `gazebo_ros` export in `package.xml`), it needs to be written or the export/install both removed — open question below.
2. `robot_interface.cpp` self-identifies as `ros::init(..., "bulk_read_node")` and logs `"BulkWriteLimited node initialized"` — stale identifiers copy-pasted from `bulk_read.cpp`/an earlier prototype. Cosmetic (doesn't affect function) but confusing for `rosnode`/log filtering.
3. `bulk_read.cpp`'s hardcoded `DXL_BIAS`/`DXL_SIGN` arrays diverge from `config/ArmInterface.yaml`'s `bias`/`sign` at the joint-4/5 indices. `bulk_read` doesn't read the yaml at all — everything is hardcoded — so these two sources of truth can silently drift, as they apparently already have.
4. `find_limits.cpp` hardcodes `num_motors = 5`, so it never sees joint_6/gripper data.
5. `config/ArmInterface.yaml` sets `command_timeout` twice (`0.2` then `0.5`); YAML last-value-wins, so the effective value is `0.5`. Unclear which was intended.
6. README describes `arm_sim.launch` as the GUI-control launch file; it isn't — `arm_sim_gui.launch` is. Documentation drift.
7. `config/ArmInterfaceBad.yaml` (`num_joints: 5`, `ids: [1..5]`) is unreferenced by any launch file. Given joint_6 is the gripper, this may be a legitimate arm-only (no-gripper) config rather than the "5-DOF-era leftover" it first looked like — see open questions.

## Open questions

1. **M1/M2 kinematics scope:** solve FK/IK over joints 1–5 (to `link_5`, pre-gripper) and treat joint_6/gripper as a separate independent DOF? This doc assumes yes based on joint_6 = gripper, but worth confirming before M1 is scoped in detail.
2. Is `config/ArmInterfaceBad.yaml` the intended arm-only (no-gripper, 5-motor) config, or genuinely dead/legacy? Should it be deleted, kept as a documented alternate mode, or something else?
3. Is `scripts/arm_gazebo_bridge_node.py` (referenced by the now-commented-out install line) something that should be written, or should that install line — and the `gazebo_ros` export block in `package.xml` — be removed outright since no Gazebo node currently exists in this package? (Relevant to scope for M6+ camera/Gazebo work, if Gazebo is even still the intended sim path vs. this package's own kinematic simulator.)
4. `command_timeout`: is `0.5`s (the effective value) correct, or was `0.2`s intended?
5. Should items 2–6 in Known Limitations above be fixed now (small, low-risk) or deferred to a dedicated cleanup milestone, to keep milestone branches focused on new capability rather than mixed with unrelated fixes?

## Repo/workflow state

- `arm_interface` was **not** a git repo before this session; git-initialized as part of M0 (initial commit captures pre-M0 baseline state) so `milestone-N-<name>` branches can be used from M1 onward, per the working protocol.
- `DynamixelSDK` is untouched — clean checkout of `noetic-devel`, no local modifications.
