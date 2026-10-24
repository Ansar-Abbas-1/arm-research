# arm_master

ROS Noetic workspace for a 5-DOF Dynamixel arm with a gripper and an eye-in-hand camera. It covers the whole chain from the robot driver to a detector-driven grasp and uproot of plants: kinematics, trajectory generation, a driver with e-stop and motor health monitoring, camera and marker localization, a YOLO plant detector, a behavior-tree grasp sequencer with an arm-sag model, and an operator interface (rviz panel or rqt plugin), all started from four nested launch files.

The project is developed in simulation first and then on the real arm, with the operator present and the e-stop in reach. Simulation is the default everywhere: nothing opens the serial port unless you pass `use_hardware:=true`.

| | |
|---|---|
| Arm | 6 Dynamixel motors on a serial bus (joints 1 to 5 form the arm, joint 6 is the gripper): MX-28AT (IDs 1, 4, 5, 6) and MX-106 (IDs 2, 3), Protocol 2.0, 1 Mbaud |
| Camera | USB webcam (Microdia Vitade AF), mounted on the end effector |
| Software | ROS Noetic (Ubuntu 20.04, Python 3.8, C++14), run in the Docker container `claude` |
| Status | Real arm: reach pipeline, e-stop, camera, marker localization, detector, grasp and uproot verified on hardware (2026-10-01 to 2026-10-19). Visual servo: simulation only. See `ARM_USAGE_GUIDE.md` for what is verified and what is not. |

## Repository layout

One repository; every package keeps the history it had as its own repository (imported on 2026-10-24). `src/DynamixelSDK` is a git submodule of the ROBOTIS upstream.

| Package | Content |
|---|---|
| `src/arm_interface` | Dynamixel driver `robot_interface` (position mode, hold, slew limit, latched e-stop, motor health), kinematic simulator `arm_sim_validator_node`, URDF, `motor_diag`, `estop_cli.py` |
| `src/arm_kinematics` | Forward kinematics, Jacobian, position + pitch IK with a posture optimiser, base collision, workspace tools; the technical report `docs/kinematics_report.md` |
| `src/arm_trajectory` | Quintic joint and Cartesian trajectories, trajectory executor (`follow_joint_trajectory`), `/reach_target_pose`, preset poses backend |
| `src/arm_gui` | rviz panels: Arm Control (presets, e-stop, torque, motors, gripper) and Mission Control |
| `src/arm_camera` | USB camera bring-up, intrinsic and mount calibration tools, static camera transform |
| `src/arm_perception` | ArUco `marker_localizer`, YOLO `object_detector`, frame capture, dataset and training tools |
| `src/arm_servo` | Visual servo to a marker (simulation only) |
| `src/arm_grasp` | Grasp sequencer (behavior tree), `gripper_control`, grasp target, arm-sag model, free-air tests and the integral-gain experiments |
| `src/arm_mission` | Mission control backend and rqt plugin |
| `src/arm_bringup` | The four nested launch files, rviz configs, detector profile |
| `src/arm_teleop` | Joystick teleop prototype, set aside (not part of the current work) |
| `src/DynamixelSDK` | ROBOTIS Dynamixel SDK (submodule, branch `noetic-devel`) |

Top-level documents: [`ARM_USAGE_GUIDE.md`](ARM_USAGE_GUIDE.md) (setup, calibration, deployment, history, who acts after launch) and [`ROADMAP.md`](ROADMAP.md) (milestones M0 to M8 and what was found at each).

## Getting the code

```bash
git clone --recurse-submodules git@github.com:RiAliotta/arm_master.git
# or, in an existing clone:
git submodule update --init
```

## Requirements

- Docker with a container based on `osrf/ros:noetic-desktop-full`. The configuration used: `--privileged`, `--network host`, the clone mounted as `/home/dev`, `/dev/bus/usb` and `/tmp/.X11-unix` mounted. The original `docker run` line is not recorded.
- Extra ROS packages in the container: `ros-noetic-usb-cam v4l-utils ros-noetic-joy ros-noetic-py-trees ros-noetic-py-trees-msgs ros-noetic-py-trees-ros ros-noetic-rqt-py-trees python3-pip`.
- For the detector: onnxruntime in a separate path because the system Python 3.8 has numpy 1.17 (`pylibs_yolo` for CPU, `pylibs_gpu` for CUDA); training needs a CUDA PyTorch with ultralytics on the host. Setup commands: `src/arm_perception/docs/yolo_detector.md`.
- For the real arm: a 12 V supply for the motors, the FTDI serial adapter (udev rule setting its latency timer to 1 ms, see the guide), the camera, and the operator with the e-stop in reach.

## Build

```bash
docker exec claude bash -lc 'source /opt/ros/noetic/setup.bash; cd /home/dev && catkin_make -DCMAKE_BUILD_TYPE=Release'
```

The default catkin build is unoptimised and about 250 to 300 times slower for the kinematics (`kinematics_report.md`, section 15.1): use `Release`.

## Quick start

All ROS commands run in the container; `source /home/dev/devel/setup.bash` first. Use one master per purpose: `11312` for simulation tests, an own port (for example `11313`) for the real arm, and do not share a master with unrelated ROS systems (a simulated clock on a shared master broke the arm nodes once). Start a launch on a chosen master with `roslaunch --port <port> ...` and `export ROS_MASTER_URI=http://localhost:<port> ROS_HOSTNAME=localhost`.

**Simulation** (no hardware, camera and detector off):

```bash
roslaunch --port 11312 arm_bringup bringup_grasp.launch camera:=false detector:=false mission_control:=rqt
```

**Real arm** (supervised, e-stop in reach, free space around the arm):

```bash
roslaunch --port 11313 arm_bringup bringup_grasp.launch use_hardware:=true serial_port:=/dev/ttyUSB0 \
  device:=/dev/video2 mission_control:=rviz
```

The four launch files are nested: `bringup_arm` is included by `bringup_vision`, then `bringup_detect`, then `bringup_grasp` (the complete stack); each repeats all arguments of the lower one with the working settings as defaults. `mission_control:=rviz | rqt | none` selects the operator interface. Crashed nodes restart by themselves (`respawn`), except rviz, the rqt windows and the hardware driver. Details, all arguments and the operating procedure: `src/arm_bringup/docs/bringup.md`.

**Grasping a plant** (arm at a viewing pose, `marker_0` on the floor in view): in the Mission Control panel press *Refresh plants*, pick the plant, *Dry run*, then *GO* (confirmation). GO is only enabled after a successful dry run of exactly the current settings. The sequence (open, move above the plant, re-aim from the measured end-effector error, descend, close on a load target, lift, return to the viewing pose) runs by itself; STOP and the e-stop are always live.

## Safety

- Anything that moves the arm is run by the operator with the e-stop in reach; new motions are first run in simulation on their own master, and on the arm at reduced speed.
- **Stopping the driver turns the torque off and the arm drops.** Put the arm at the `default` preset first (`rosservice call /preset_manager/command "{action: go, name: default}"`), then stop the stack with SIGINT. If the serial port is lost the driver retries forever and needs `kill -9`.
- E-stop: the Arm Control panel button or `rosrun arm_interface estop_cli.py trigger` (locks the arm in place with torque on); reset with `estop_cli.py reset`.
- Never write motor EEPROM from code without an explicit decision; the motors' temperature limits were set to 80 C by hand.

## Calibration

| Item | Where |
|---|---|
| Camera intrinsics (chessboard, GUI, offline refit) | `src/arm_camera/docs/camera_calibration.md` |
| Camera mount (hand-measured transform in use; ChArUco and hand-eye procedures built, not run on the arm) | `src/arm_camera/docs/charuco_mount_calibration.md` |
| Marker sizes and the floor marker | `src/arm_perception/config/markers.yaml` |
| Plant detector (dataset, training, export) | `src/arm_perception/docs/yolo_detector.md` |
| Arm sag (free-air test, fit, feed-forward model) | `src/arm_grasp/docs/sag_compensation.md` |
| Grasp aim bias, grasp height, hold load | runtime parameters, see `ARM_USAGE_GUIDE.md` section 2.6 |

## Tests

Unit tests per package (gtest, nose, rostest), for example:

```bash
cd /home/dev/src/arm_grasp && PYTHONPATH=src:$PYTHONPATH python3 test/test_grasp_tree.py
rostest arm_gui mission_panel.test            # needs the test target built: catkin_make test_mission_panel
cd /home/dev/src/arm_bringup && python3 test/test_bringup.py
python3 /home/dev/src/arm_grasp/test/e2e_sim_grasp.py      # closed-loop simulation test, own master 11312
```

## What is not in the repository

Large local data is ignored by `.gitignore` and needs a separate backup: trained models (`models/`), recordings and logs (`recordings/`, including the raw sag and gain-test measurements behind the reports), datasets and training runs (`yolo/`), and the Python environments of the detector.

## Key documents

- [`ARM_USAGE_GUIDE.md`](ARM_USAGE_GUIDE.md): use the repository from scratch (setup, calibration, deployment), who acts after launch, history, open items.
- [`ROADMAP.md`](ROADMAP.md): milestones and findings.
- [`src/arm_kinematics/docs/kinematics_report.md`](src/arm_kinematics/docs/kinematics_report.md): kinematics, calibration, implementation and the manipulation chapter (thesis-chapter draft).
- [`src/arm_grasp/docs/sag_compensation.md`](src/arm_grasp/docs/sag_compensation.md): arm sag, the correction loop and the motor integral-gain experiments.
- [`src/arm_interface/docs/sim_to_real.md`](src/arm_interface/docs/sim_to_real.md) and `architecture.md`: the driver, the safety path and the hardware verification log.
- [`src/arm_bringup/docs/bringup.md`](src/arm_bringup/docs/bringup.md): launch sets, parameters, respawn policy, Mission Control.

## Known limits

- The camera mount is hand-measured (a few millimetres and degrees of error); the real calibration is deferred.
- Grasp results come from one operator on one set-up (a few grasps on 2026-10-14, 15 on 2026-10-16); no repeatability claim.
- The sag model covers the 45 degree approach at grasp heights, with a validity range of 0.46 to 0.55 m radius (plus 5 cm); outside it the offset is zero.
- The e-stop has been verified at 0.2 rad/s but not at the full configured speed; the visual servo has never commanded the real arm; the gripper unit conversion is a placeholder.
- There is no license file yet.

## Author

Riccardo Aliotta (GitHub: RiAliotta).
