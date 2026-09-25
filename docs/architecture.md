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

`test/test_watchdog.cpp` (gtest, target `arm_interface-test-watchdog`) covers the torque-disable watchdog staleness check and the `command_timeout` validation used by the dynamic_reconfigure callback below — both extracted as pure functions in `include/arm_interface/watchdog.h` so they're testable without hardware. `roslaunch_add_file_check(launch)` (already configured, just never ran until the build was unblocked) is also part of `catkin_make run_tests_arm_interface`. Run with:

```bash
catkin_make run_tests_arm_interface
```

Everything else (the hardware bridge logic in `robot_interface.cpp`, `bulk_read`, `find_limits`) still has no coverage — `robot_interface` opens a real serial port in its constructor, so unit-testing it directly requires either hardware or a larger refactor to separate control logic from I/O (see Known Limitations #1 below). Closing that gap is a candidate for M1+ as new capability code gets added on top.

## Known limitations

Numbering below reflects what was found during M0; items marked **fixed** were resolved in a follow-up cleanup pass after the open questions were answered (see git log). All fixes were verified with a clean `catkin_make run_tests_arm_interface`.

1. `CMakeLists.txt` called `catkin_install_python()` on `scripts/arm_gazebo_bridge_node.py`, which doesn't exist — failed CMake *configure*, blocking the entire build. **Fixed:** commented out with an explanation. **Decision:** Gazebo references (this install line + the `gazebo_ros` export block in `package.xml`) are kept as an inactive placeholder for now, not removed — this package's own kinematic simulator (`arm_sim_validator_node`) is the actual sim path for M1–M4 per the roadmap; Gazebo may become relevant later.
2. `robot_interface.cpp` self-identified as `ros::init(..., "bulk_read_node")` and logged `"BulkWriteLimited node initialized"` — stale, copy-pasted from `bulk_read.cpp`. **Fixed:** renamed to `robot_interface_node` / corrected log message.
3. `bulk_read.cpp`'s hardcoded `DXL_BIAS`/`DXL_SIGN` diverged from `config/ArmInterface.yaml` at the joint-4/5 indices. **Fixed:** corrected to match, with a comment noting these are duplicated (not loaded from yaml) and must be kept in sync manually.
4. `find_limits.cpp` hardcoded `num_motors = 5`, missing joint_6/gripper. **Fixed:** corrected to 6.
5. `config/ArmInterface.yaml` set `command_timeout` twice (`0.2` then `0.5`). **Fixed:** deduped to the effective value, `0.5`, and made runtime-tunable via `dynamic_reconfigure` (`cfg/ArmInterface.cfg`) — since this is the torque-disable watchdog's timeout, a safety-relevant value, the reconfigure callback rejects non-positive values (keeps the previous one, logs a warning) instead of accepting anything blindly. Adjust live with `rosrun rqt_reconfigure rqt_reconfigure` while `robot_interface` is running, or `rosservice call /robot_interface/set_parameters ...`.
6. README described `arm_sim.launch` as the GUI-control launch file; it isn't — `arm_sim_gui.launch` is. **Fixed:** corrected.
7. `config/ArmInterfaceBad.yaml` (5-motor legacy config) was unreferenced by any launch file. **Fixed:** deleted — confirmed dead now that joint_6 is known to be the gripper (not a missing 6th arm-pose joint this file predates).
8. **Found during the fix pass, not in original M0 scan:** `package.xml` was missing `exec_depend`s on `robot_state_publisher`, `rviz`, and `joint_state_publisher_gui`, all used by the launch files. `roslaunch_add_file_check(launch)` (already configured in `CMakeLists.txt`) was silently never catching this because the build never got far enough to run tests until the scripts/arm_gazebo_bridge_node.py fix above. **Fixed:** added the three `exec_depend`s; `roslaunch_add_file_check` now passes.

## Open questions — resolved

1. **M1/M2 kinematics scope:** solve FK/IK over joints 1–5 (to `link_5`, pre-gripper); joint_6/gripper is an independent DOF, not part of the pose chain. **Decided: yes**, this scopes M1.
2. `config/ArmInterfaceBad.yaml`: **decided dead legacy — deleted** (see Known Limitations #7).
3. `scripts/arm_gazebo_bridge_node.py` / Gazebo references: **decided keep as inactive placeholder** (see Known Limitations #1) — revisit only if/when Gazebo is actually adopted.
4. `command_timeout`: **decided keep 0.5 as default, made runtime-tunable** via dynamic_reconfigure rather than picking one static value (see Known Limitations #5).
5. Known-limitations cleanup timing: **decided fix now**, in a separate commit from new capability work — done (see git log: config cleanup, stale-value/doc-drift fixes, dynamic_reconfigure feature commits, each isolated from the M0 baseline commit).

## Repo/workflow state

- `arm_interface` was **not** a git repo before this session; git-initialized as part of M0 (initial commit captures pre-M0 baseline state) so `milestone-N-<name>` branches can be used from M1 onward, per the working protocol.
- `DynamixelSDK` is untouched — clean checkout of `noetic-devel`, no local modifications.
