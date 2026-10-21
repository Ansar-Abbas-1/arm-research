# Using the arm repository from scratch: setup, calibration, deployment

Status as of 2026-10-21. Built from `ROADMAP.md`, the package docs and the sessions in which the steps were actually performed. Dates are the dates the step was done. Anything marked **not done** or **untested** has not been run on the real arm. Commands are the ones used; `docker exec claude bash -lc '...'` wraps everything that needs ROS (see 2.1).

## 0. How to read this guide

**Who acts after launch** (the label on every task):

| Label | Meaning |
|---|---|
| **A** | Autonomous: after the launch command nothing more is needed; it runs until finished or stopped. |
| **A+S** | Autonomous, but a person must be present (e-stop in reach, free space around the arm). Used for everything that moves the arm by itself. |
| **T** | Needs one trigger from the user (a service call, a button), then runs by itself. |
| **U** | Needs the user during the run: GUI clicks, moving the arm or a board by hand, measuring, confirming. |

**Machines and masters.** Host: the laptop (Ubuntu, NVIDIA RTX 5070, Wayland through XWayland). Container `claude` (image `osrf/ros:noetic-desktop-full`): the workspace is `/home/dev` in it and `~/dev/claude_ros_ws` on the host. Three ROS masters have been used and must not be mixed: **11311** (default, shared with the ANYmal simulation that appeared on 2026-10-19), **11312** (simulation tests only), **11313** (the real arm since that day). `ROS_MASTER_URI=http://localhost:<port>` and `ROS_HOSTNAME=localhost`; start a launch on a chosen master with `roslaunch --port <port> ...`.

**Repository layout.** `ROADMAP.md` is the only file tracked by the workspace-root repo; every package under `src/` is its own git repository.

| Package | Purpose | Milestone / date |
|---|---|---|
| `arm_interface` | Dynamixel driver (`robot_interface`), simulator (`arm_sim_validator_node`), URDF, e-stop, motor health, `motor_diag` | M0 2026-09-25; hardware stages 2026-10-01 to 10-06 |
| `arm_kinematics` | FK, Jacobian, IK (position + pitch), null-space posture, base collision; the report `docs/kinematics_report.md` | M1/M2 2026-09-25 |
| `arm_trajectory` | Quintic joint and Cartesian trajectories, executor (action server), `reach_target_pose`, presets backend | M3 2026-09-26, M4 2026-10-01 |
| `arm_gui` | rviz panels: Arm Control (presets, e-stop, torque, motors, gripper, hold-load slider), Mission Control | 2026-10-06, 10-15 |
| `arm_camera` | USB camera, intrinsics, hand-eye and ChArUco mount calibration, static camera TF | M6 2026-10-06/07 |
| `arm_perception` | ArUco `marker_localizer`, YOLO `object_detector`, frame capture, dataset and training tools | M7 2026-10-07; YOLO 2026-10-13/14 |
| `arm_servo` | Visual servo to a marker | M8 2026-10-07 (simulation only) |
| `arm_grasp` | Grasp sequencer (behavior tree), gripper control, grasp target, sag model, free-air tests | 2026-10-08 to 10-19 |
| `arm_mission` | Mission control backend and rqt plugin | 2026-10-15 |
| `arm_bringup` | The four nested launch files, rviz configs, detector profile | 2026-10-15 |
| `arm_teleop` | Joystick prototype, set aside by the operator in M4: ignore | 2026-09 |

## 1. Setup

One-time work, or work to repeat after a reinstall, a container rebuild, or a hardware change.

### 1.1 Host, container, display

| Step | Who | Notes |
|---|---|---|
| Create the container `claude` | U (once) | Inspected configuration: image `osrf/ros:noetic-desktop-full`, `--privileged`, `--network host`, mounts `~/dev/claude_ros_ws -> /home/dev`, `/dev/bus/usb`, `/tmp/.X11-unix`. The original `docker run` line is not recorded in the repository. |
| Start it after a reboot | U | `docker start claude`. Nothing in it starts by itself. |
| Allow rviz windows | U (every login) | `xhost +local:` on the host; `DISPLAY=:0` in the container. rviz uses software GL (harmless `libGL error` lines). |
| Extra ROS packages (installed by hand after the base image, reinstall after a rebuild) | U (once) | `apt install ros-noetic-usb-cam v4l-utils ros-noetic-joy ros-noetic-py-trees ros-noetic-py-trees-msgs ros-noetic-py-trees-ros ros-noetic-rqt-py-trees python3-pip` |
| Detector Python environment (the system Python 3.8 has numpy 1.17; onnxruntime needs 1.21+, so it lives apart) | U (once) | `/home/dev/pylibs_yolo` (CPU onnxruntime), `/home/dev/pylibs_gpu` (onnxruntime-gpu 1.19.2 + CUDA 12.9 / cuDNN 9 libraries, `pip install --target`), `/home/dev/venv_yolo` (export and offline tests). Details: `arm_perception/docs/yolo_detector.md` "Setup" and "GPU inference". |
| Training environment | U (once) | The host's existing `~/.venv` (torch 2.13+cu130, ultralytics 8.4.95): the container's torch is CPU-only. |
| Build | U | `docker exec claude bash -lc 'source /opt/ros/noetic/setup.bash; cd /home/dev && catkin_make'`. `DynamixelSDK` is vendored in `src/`. The default build is **unoptimised**: add `-DCMAKE_BUILD_TYPE=Release` (about 250 to 300 times faster IK, `kinematics_report.md` 15.1). |
| Run the unit tests | A | Per package, e.g. `PYTHONPATH=src:$PYTHONPATH python3 test/test_grasp_tree.py` (arm_grasp), `rostest arm_gui mission_panel.test`, `python3 test/test_bringup.py` (arm_bringup), gtest binaries in `devel/lib/<pkg>/`. Test targets are built with `catkin_make <target>`. |

### 1.2 Hardware connection

| Step | Who | Notes |
|---|---|---|
| Power the motors (12 V) and check the supply | U | Added 2026-10-19 after a supply collapse (8.2 to 9.9 V on all six motors, bus fault). With the stack stopped: `rosrun arm_interface motor_diag --device /dev/ttyUSB1` (read-only, writes nothing, needs the port to itself). Healthy: 12.0 to 12.5 V, no hardware error, 33 to 41 C. |
| Serial adapter (FTDI, 1 Mbaud, Protocol 2.0, motor IDs 1 to 6) | U | udev rule `/etc/udev/rules.d/99-ftdi-latency.rules` (`ATTR{latency_timer}="1"`, since 2026-10-07; the kernel resets it to 16 ms at every enumeration otherwise). The adapter dropped off USB on 2026-10-07 and 2026-10-19 and came back as `/dev/ttyUSB1`: then `docker exec claude bash -c 'mknod /dev/ttyUSB1 c 188 1; chmod 666 /dev/ttyUSB1'` and launch with `serial_port:=/dev/ttyUSB1`. |
| Camera (Microdia Vitade AF, MJPG 1920x1080 about 20 fps) | U | Lock the autofocus. A camera plugged in after the container started is invisible: restart the container or `mknod /dev/video2 c 81 2`. It renumbers after replugs: `device:=` (`/dev/video2`, `/dev/video3`). Check the sysfs name: `for d in /sys/class/video4linux/video*; do echo $d $(cat $d/name) $(cat $d/index); done` (index 0 of "Webcam" is the real one). |
| Motor temperature limits | U (once, by hand) | Raised to 80 C on all motors by the operator on 2026-10-06 (they were 50 C on IDs 1 to 4). **Never write motor EEPROM from code without an explicit go.** |
| Gripper | U | The gripper parameters are decided by the hardware (jaw closing load, speed): see 2.4. |

### 1.3 The verification ladder that brought the real arm into use (done, 2026-10-01 to 10-06)

Every stage that touches motors is run by the user with the e-stop in reach; this is the rule of the project. Nothing needs repeating unless the driver or the wiring changed.

| Stage | What | Tool | Who | Result |
|---|---|---|---|---|
| 0 | Hardening: velocity-limited second interface, e-stop interface, gripper unit conversion | code | A (tests) | done 2026-10-01 to 10-02 |
| 1 | Read-only: driver running, torque off, positions plausible, signs and zero match the arm | `robot_interface` (debug), `motor_diag`, `bulk_read` | U | passed 2026-10-01; found the gripper open/close sense inverted, fixed with `gripper_invert: true` |
| 2 | First live motion, one joint a few mrad at a time | `scripts/stage2_single_joint_test.py --joint N --delta 0.01` | U | passed, all six joints, 2026-10-01 |
| 3 | Joint-space and Cartesian moves at reduced speed | `stage3_joint_move.py`, `stage3_cartesian_reach.py`, `/reach_target_pose` | U | passed 2026-10-02 at 0.2 rad/s |
| e-stop | Latched hold-in-place, mid-motion stop | `estop_cli.py trigger / reset`, rviz E-STOP | U | PASS at 0.2 rad/s (stops in about 0.3 s, under 5 mrad); **not re-verified at full speed** |
| Restructure | Hold instead of drop, slew-limited goals, e-stop visible to all senders | driver | U | verified 2026-10-02 to 10-05 |
| Health | Motor load, voltage, temperature, hardware-error register, e-stop escalation | driver, panel | A | verified at startup and after a power cycle |

Speed settings: `velocity_scale` 1.0 and `max_goal_slew_rad_s` 2.0 are the defaults by operator decision (0.2 was only used for first checks); `velocity_scale` is read once, so changing it needs a restart of the stack.

### 1.4 Simulation-first rule

New code is run in simulation on its **own master (11312)** before it touches the arm: a test on the real master once moved the real arm. `arm_sim_validator_node` is the simulator (URDF-limited, kinematic, no Gazebo); `use_hardware:=false` is the default of every bringup. Example: `python3 src/arm_grasp/test/e2e_sim_grasp.py` starts its own roscore and runs a dry run, an unreachable target, a full grasp, an e-stop, a stale target, a moved target and an operator stop (all PASS, 2026-10-16). The e2e test logs the sag measurements to the temp directory so it cannot pollute the real log.

## 2. Calibration

What was calibrated, how, and what is still open.

### 2.1 Joint zero, sign and limits (`arm_interface/config/ArmInterface.yaml`)

`bias`, `sign`, `encoder_min/max` per motor, checked against the URDF limits (drift at most 0.046 rad, judged coherent, 2026-10-01). `find_limits` (manual min/max while a person moves the arm) exists but is not wired into a launch file. The gripper (`joint_6`) is prismatic in the URDF and rotational in the motor: an affine conversion (`gripper_conversion.h`, 0 to 0.025 m, inverted) is applied at the interface and is **an uncalibrated placeholder** until the mechanism is measured. **U** (measuring), tools **A** after that.

### 2.2 Camera intrinsics (done 2026-10-06, rechecked 2026-10-07)

| Step | Who |
|---|---|
| Print a 9 x 6 inner-corner chessboard on a rigid plate, **measure a square** with a ruler, edit `arm_camera/config/board.yaml` (23 mm) | U |
| `roslaunch arm_camera intrinsics_calibration.launch cols:=9 rows:=6 square:=0.023`; move the board until the X, Y, Size, Skew bars are green, CALIBRATE, SAVE, COMMIT (40 to 60 sharp, still views) | U |
| Offline refit from the saved images: `rosrun arm_camera calibrate_intrinsics.py --images <folder with left-*.png> ...` (the GUI gives no per-view diagnostics; blurred views were the main error) | A after launch |
| Result installed as `config/camera_info.yaml`: 0.7 px RMS on sharp views, fx about 1520 (GUI, installed and refit agree to 0.3 %) | |
| Cross-check against a ruler: `roslaunch arm_camera aruco_test.launch marker_size:=0.100`, `rosrun arm_camera aruco_distance_report.py --id 0 --measured <mm>` | U |

### 2.3 Camera mount (end_effector to camera_optical_frame)

**In use: a hand-measured static transform** (since 2026-10-07): x = -0.025, y = 0, z = -0.06 m, yaw +90 deg, pitch 0, roll 0 (the camera is upside down, looking along the tool axis): `camera_tf:=static x:=-0.025 z:=-0.06 yaw:=1.5708` (the defaults of `bringup_vision.launch`). Expect a few mm and a few degrees of error; a rotation error theta costs up to 2 sin(theta/2) times the 16 cm camera-to-marker distance at the stand-off.

**Real calibration: not done (deferred, not dropped).** Procedures built and tested on synthetic data only:

| Method | Command | Who |
|---|---|---|
| ChArUco board of known pose on the floor (9x7, 25 mm squares, `config/charuco.yaml`) | `roslaunch arm_camera charuco_mount_calibration.launch known_pose:=true x:= y:= z:= yaw:= origin:=corner`, then `rosservice call /charuco_mount_calibration/capture` (8 or more views) and `.../solve` | U (moves the arm to each view, captures, measures the board pose) |
| Hand-eye with a chessboard | `roslaunch arm_camera hand_eye_collect.launch`, `rosservice call /hand_eye_collector/capture` per pose, `/solve` | U |
| Check at held-out poses | `roslaunch arm_camera overlay.launch` (RMS in 3D) | U |

The collector never moves the arm. Return to this if the grasp accuracy ever needs better than the roughly 1 to 2 cm that the arm's kinematics and the mount allow.

### 2.4 Markers and the floor reference

Print the markers and **measure the black square of each** (the reported distance scales with the size): `arm_perception/config/markers.yaml` (id 0: 100 mm, dictionary `4X4_1000`, the floor reference `marker_0`; 14 markers of 50 mm from `ARUCO_ORIGINAL`). `marker_localizer` reports a marker only while the camera has been still for 0.5 s. Measured 2026-10-07: repeatability 0.1 to 0.4 mm, accuracy 1 to 2 cm. **U** (print, measure, place `marker_0` on the floor in view of the viewing pose).

### 2.5 Plant detector (done 2026-10-13/14, model `vignax_small_v45A.onnx`)

| Step | Command / tool | Who |
|---|---|---|
| Capture arm-camera frames with their poses | `roslaunch arm_perception frame_capture.launch`, `rosservice call /frame_capture/capture` or `/capture_burst` | T |
| Label frames and photos (classes `Sorgo_Row`, `Vigna`) | Roboflow, YOLO segmentation export | U |
| Build the crop dataset | `PYTHONPATH=src python3 scripts/make_tile_dataset.py --src <labeled folder> --out yolo/vignax_tiles --val <photo ids> --scales 0.7,1.0,1.5,2.0,2.5` | A |
| Train (GPU, host `~/.venv`) | `nice -n 10 ~/.venv/bin/python scripts/train_small_yolo.py --data yolo/vignax_tiles/data.yaml --weights yolo26s-seg.pt --name vignax_s --export` (50 epochs about 0.9 h) | A after launch |
| Offline check | `scripts/detect_images.py --model <onnx> --source <photos>` | A |
| Result | mask mAP50 0.91 on held-out arm poses (optimistic: same plants, other viewpoint), 6 of 7 plants on live frames, 10 to 17 ms per frame | |

The aim pixel is the **centroid of the segmentation mask** (the box centre missed the stem by 3.9 cm). The detector sees every plant on the plane `floor + grasp height`; the camera has no depth.

### 2.6 Grasp aim and gripper load (found through real grasps, 2026-10-14)

All values are runtime parameters and are in `arm_bringup` as defaults.

| Quantity | Value | How it was found | Who |
|---|---|---|---|
| Approach angle | 45 deg below horizontal (horizontal failed: pose too low; one joint-2 overload) | trial | U decided |
| Pinch plane | 9 cm above the floor (11 cm for the last three grasps of 2026-10-16) | trial, operator feedback | U |
| Aim bias in the arm frame (radial, tangential, vertical) | (-10, +10, 0) mm (-5 radial for the last grasp) | repeated misses: none = 1 to 3 cm too far out; -20 or -30 mm = 1 cm too far in | U (watches each grasp and says "too far in / out / left") |
| Hold load | 15 % target, 5 % contact load (a 25 % hold was also used) | operator; the 15 % grip needs `contact_load` well below the target | U |
| Gripper close | 4 mm/s approach, 2 mm/s trim, load PI (`hold_kp` 3e-6, `hold_ki` 1.4e-5) | the load rises 65 to 80 % per mm | U |

### 2.7 Arm sag (gravity) feed-forward (2026-10-14; full report `arm_grasp/docs/sag_compensation.md`)

| Step | Command | Who |
|---|---|---|
| Free-air probes: the arm moves to PRE, GRASP, LIFT of a grid of points and measures its error from the encoders (gripper never commanded) | `rosrun arm_grasp sag_test.py --azimuth <deg> --go` (dry run without `--go`); the area under every point must be clear | A+S |
| Fit the model | `python3 scripts/sag_fit.py --out config/sag_model.json` (reads `recordings/sag/sag_*.jsonl`, not `sim/`) | A |
| Report | `python3 scripts/sag_report.py --since <run id>` | A |
| Use it | `sag_model` and `sag_gain` in `arm_grasp/config/grasp.yaml` | |

Result: 45 probes at azimuths -40, 0, +40: raw error 23.6 mm at grasp heights, 8.0 mm with the model (validity range: radius 0.46 to 0.55 m +-5 cm, height +-4 cm; zero offset outside). Real grasps at azimuth 73 to 110 degrees (2026-10-16) gave 7.7 mm at GRASP with the model against 20 mm without it. The correction loop (2 steps, gain 0.7, trigger 10 mm) trims the rest. Open: refit with the 57 real rows, the radius dependence above 0.55 m, the tangential term.

### 2.8 Motor integral gain (experiment of 2026-10-19; **not adopted**)

A driver service (`/arm_interface/apply_position_gains`, RAM only, I gain only, gripper never written) and `gain_sweep.py` / `compare_methods.py` were built to test whether the motors' Position I gain removes the sag. With a settle of 5 s: I 128 leaves 2.1 mm, I 256 1.6 mm, I 512 1.1 mm (17 mm at I 0); at the 0.8 s of the grasp sequence only I 256 and above match the model (4.7 and 1.6 mm against 4.9). Small gains do nothing within seconds; stability margin shrinks with the gain (joint 2 once 1.09 deg at 58 C; wrist 0.37 deg at 512). The pipeline still uses the feed-forward model plus the correction loop. Details: `sag_compensation.md`, `kinematics_report.md` section 20.

### 2.9 Calibration summary

| Item | Status | Needs the user |
|---|---|---|
| Joint zero / sign / limits | done, coherent | measuring only |
| Gripper conversion | placeholder | yes (measure the mechanism) |
| Intrinsics | done | yes (GUI) |
| Camera mount | hand-measured; real calibration **not done** | yes |
| Marker sizes | measured | yes |
| Detector | trained, v45A | yes (labels) |
| Grasp aim bias, heights, load | tuned by hand | yes |
| Sag model | fitted at 45 degrees, grasp heights | no after launch (arm must be supervised) |
| Motor I gain | tested, not adopted | no after launch (supervised) |

## 3. Deployment

### 3.1 Before every session

1. Power on the supply; if anything was unplugged: `motor_diag` (stack stopped) for voltages and errors.
2. `docker start claude`; `xhost +local:`; check `/dev/ttyUSB*` and the camera node (section 1.2); create `/dev/ttyUSB1` in the container if needed.
3. Check that nothing else occupies the master you use; use your own port for the arm (11313).
4. Put the arm at the default pose (`default`: joints 0, 0.02, -0.16, 0.02, -0.02) with the rest clear.

### 3.2 Launch (one command, any functional set)

The four sets are nested: `bringup_arm` is included by `bringup_vision`, which is included by `bringup_detect`, which is included by `bringup_grasp` (the complete stack); each repeats all arguments of the lower one, with the working settings as defaults. `use_hardware` defaults to **false**.

| Set | Adds | Nodes |
|---|---|---|
| `bringup_arm` | driver or simulator, planner, executor, reach node, presets, gripper | `arm_interface`, `robot_state_publisher`, `trajectory_*`, `reach_target_pose_node`, `preset_manager`, `gripper_control`, rviz |
| `bringup_vision` | camera, mount TF, marker localizer | `usb_cam`, `camera_static_tf`, `marker_localizer` |
| `bringup_detect` | detector, grasp target | `object_detector`, `grasp_target` |
| `bringup_grasp` | grasp sequencer, mission control | `grasp_sequencer`, `mission_control` (+ `mission_control_ui` for rqt) |

```bash
docker exec -d claude bash -lc 'source /opt/ros/noetic/setup.bash; source /home/dev/devel/setup.bash; \
  export ROS_MASTER_URI=http://localhost:11313 ROS_HOSTNAME=localhost DISPLAY=:0; \
  setsid roslaunch --port 11313 arm_bringup bringup_grasp.launch use_hardware:=true serial_port:=/dev/ttyUSB1 \
  device:=/dev/video2 mission_control:=rqt > /tmp/stack.log 2>&1'
```

`mission_control:=rviz | rqt | none` selects the operator interface (rviz panel, a separate rqt window, or none). `camera:=false detector:=false target:=false use_rviz:=false` leave parts out (the sweep and sag tests ran without the camera). Parameters: `approach` (45), `bias_r/t/z`, `grasp_height`, `plant_class`, `model`, `load_target`/`contact_load`, `sag_gain`, `sag_log_dir`, `rviz_config`, `respawn`, `respawn_driver`. Settings that can change while running: `/grasp_target/approach|bias|offset`, `/object_detector/grasp_heights`, the gripper load (panel slider or `/gripper_control/set_load_target`).

**Crash restart.** `respawn:=true` restarts a crashed node after 2 s (A). Not respawned: rviz, the rqt windows, and the hardware driver (`respawn_driver`, default false: a driver that dies on a motor error would re-enable the torque again and again). A restarted sequencer comes back idle (a run in progress is aborted; the driver holds), a restarted gripper node stops regulating (the jaws stay).

### 3.3 Grasping a plant (the operating procedure)

| Step | Who | After this step |
|---|---|---|
| Arm at a viewing pose with `marker_0` in view: `Pre-approach-New` (front) or `viewing-left` (joint 1 +90 deg); open the gripper and clear any plant from it (a held plant hides the marker) | U | |
| **Refresh plants** (Mission Control): snapshots the Vigna in view, ranked by distance from the gripper axis, selects the nearest, writes the target offset | T | the snapshot is valid 60 s |
| Pick another plant in the list (optional); change approach, aim bias or height and **Apply** (a height change needs a new Refresh) | U | |
| **Dry run**: plans PRE, DESCEND, LIFT and the return to view, checks reachability and the Cartesian deviation; nothing moves | T | valid for 120 s |
| **GO** (confirmation dialog): enabled only after a successful dry run of exactly the current settings, no e-stop, nothing running; one dry run releases one go | T | |
| The sequence: open, move above the plant, re-aim from the measured end-effector error (sag feed-forward plus correction loop), re-check the target (aborts if it moved 5 mm or 3 deg), descend, close on the load target, lift 15 cm, return to the viewing pose with the plant held (about 25 to 34 s) | **A+S** | the e-stop and STOP are always live |
| Release the plant from the gripper (panel button) and repeat | U | |

Command-line equivalents: `rosservice call /grasp_sequencer/grasp "{target_frame: grasp_target, dry_run: true, probe: false}"`, `rosservice call /grasp_sequencer/stop`, `rosservice call /mission_control/command "{action: refresh}"`. Real results: 2026-10-14 two plants in a row (first GRASP error 3.8 and 0.5 mm); 2026-10-16 15 grasps from the left viewing pose, 13 uprooted and returned to the viewing pose, 2 aborted by the e-stop at targets beyond the sag model's radius range.

### 3.4 Who has to be there

| Activity | Label |
|---|---|
| Everything started by a bringup launch while idle (state publishing, TF, detection at about 10 Hz, marker localization, respawn, logging) | A |
| Detector inference, marker localization | A |
| Grasp sequence after GO, including the correction steps, the lift and the return | **A+S** |
| Free-air sag test, gain sweep, method comparison (they move the arm for 20 to 60 minutes by themselves, with abort rules on load 80 %, temperature 70 C, oscillation 1.0 deg std, hardware error, e-stop) | **A+S** |
| Dataset building, training, offline tests, unit tests, report tools | A |
| Intrinsics and hand-eye / ChArUco calibration | U |
| Labeling, printing and measuring markers, placing `marker_0`, clearing space, releasing the plant | U |
| Choosing a plant, aim bias, height, approach, GO | U |
| Power, cables, USB re-seating, supply checks | U |
| Parking the arm and stopping the stack | U (or T with a check of the pose) |

### 3.5 Stopping

1. STOP (Mission Control) or `rosservice call /grasp_sequencer/stop` ends a run; the driver holds. E-stop: rviz button or `estop_cli.py trigger` (locks the arm in place, torque on); reset with `estop_cli.py reset` (refused while a motor has a latched hardware error: reboot that motor first).
2. Move to the default pose (`rosservice call /preset_manager/command "{action: go, name: default}"`) **before** stopping the driver: stopping it (SIGINT) turns the torque off and the arm drops.
3. Stop the launch with SIGINT (explicit PID); if the port is lost the driver retries forever and needs `kill -9` (no torque command). Never use `pkill -f` inside a shell whose command line contains the pattern: it kills the shell.
4. If the master is shared with other work, never kill the launch that owns the master.

### 3.6 Monitoring and logs

`/mission_control/state`, `/grasp_sequencer/status` (JSON with the behavior-tree path), rviz Arm Control (motor table, e-stop, gripper load), `rqt_py_trees`, `/arm_interface/motor_status` (temperature, load, voltage, errors). Logs: `recordings/sag/` (raw end-effector measurements of every probe and grasp; `sim/` holds simulated rows and is ignored by the fit tools), `recordings/gain_test*/` (I-gain sweeps), the per-node ROS logs under `/root/.ros/log/`. Save the launch log (`/tmp/stack.log`) with the run.

### 3.7 Known failure modes and what to do

| Symptom | Cause seen | Action |
|---|---|---|
| Driver FATAL "Incorrect status packet", "Bus fault", all motors silent | USB adapter drop, or motor supply collapse (8.2 to 9.9 V) | stop the driver (`kill -9`), check supply and cables, `motor_diag`, relaunch |
| Motor 3 or others: hardware error 0x01 (input voltage), 0x20 (overload), 0x04 (overheat) | supply dip; joint 4 under load; joint 2 at an extended pose (53 to 73 C) | reboot the motor from the panel ("Reset failed motors"), let it cool, check the supply |
| `marker_0` not seen | the held plant covers it; arm moving; camera dropped off USB | release the plant; wait for still frames; `device:=` after a replug |
| Camera shows nothing, `usb_cam` respawns | webcam off the bus | re-seat the cable; check the sysfs name; relaunch the camera |
| Sequencer aborts "another goal was received" | another master with `use_sim_time` true (ANYmal simulation on 11311) | use an own master (11313); do not share |
| Gripper not opening or a service hangs | the gripper loop died (fixed 2026-10-14, `step_safe`) | reset the `gripper_control` node |
| A grasp is refused: "go refused ..." | no dry run of the current settings, stale list, e-stop, run in progress | follow the reason text on the panel |
| Joint 2 warm, oscillation at high I gain | thermal and gain margin | keep the I gain at 0 (default) |

## 4. History of the steps taken

| Date | What |
|---|---|
| 2026-09-25 | M0: architecture review of `arm_interface`; scope fixed to 5 joints. M1: forward kinematics (`fk_node`). M2: damped least-squares IK with adaptive damping and restarts; `joint_6` made fixed in the URDF. |
| 2026-09-26 | M3: quintic joint and Cartesian trajectories, executor as an action server (preemption). |
| 2026-09-29 to 10-01 | M4: `/reach_target_pose`, rviz marker and CLI, position + pitch IK with a null-space on by default, base-collision avoidance. First contact with the real arm: Stage 1 (read-only), Stage 2 (single joints), gripper sense fixed. |
| 2026-10-02 | Stage 3: joint and Cartesian reaches on hardware at 0.2 rad/s; latched e-stop; drive state machine (hold instead of drop), slew limit; e-stop mid-motion PASS. |
| 2026-10-05 | Motor health monitoring after motor 4 (overload) and motor 3 (overheat) shut down silently; posture optimiser; acceleration limit; speeds set back to full by the operator. |
| 2026-10-06 | rviz Arm Control panel (presets, e-stop, torque, motors, reset); operator raised the motor temperature limits to 80 C; M6 camera: `usb_cam`, intrinsics calibrated on real images, calibration tooling built. M5 put on hold by the operator. |
| 2026-10-07 | udev rule for the FTDI latency; USB drops of the adapter and webcam; hand-measured camera mount; M7 `marker_localizer` (line-fit corner refinement: 9.5 mm to 0.1 mm depth noise); M8 visual servo built and verified in simulation only. Kinematics report written. |
| 2026-10-08 to 10-09 | Grasp sequence (behavior tree, load-limited close, `gripper_control` node) in simulation, then the first load-limited closes on the hardware. |
| 2026-10-13 | Detector: GPU onnxruntime; classical methods compared; a small YOLO trained on the lab photos. |
| 2026-10-14 | Retrained with arm-camera labels; detector running live; first detector-driven grasps (mask-centroid aim; box centre failed); runtime settings (approach, aim bias, grasp height, load slider); sag measured (45 probes), modelled and compensated; two plants grasped and uprooted in a row; correction trigger proposed at 10 mm. |
| 2026-10-15 | `arm_bringup` (four launch sets, `respawn`), `arm_mission` and the rviz / rqt Mission Control (go gate); return to the viewing pose after the lift; `correction_trigger` set to 10 mm. |
| 2026-10-16 | Left viewing pose (`viewing-left`); 15 real grasps with Mission Control; sag log cleaned (simulated rows moved to `sag/sim`); report update and kinematics report Part IV. |
| 2026-10-19 | Motor I-gain tooling and experiments (not adopted); incidents: serial adapter drop, motor supply collapse (repaired by the user), an ANYmal simulation joined the master and set `use_sim_time`: the arm stack moved to its own master 11313; `motor_diag` confirmed 12 V on all motors after the repair. |

## 5. What has not been done or verified

- Real hand-eye or ChArUco camera-mount calibration (the mount is hand-measured).
- E-stop at the full speed now configured; thermal lockout; the remaining M5 items (`sim_to_real.md` sections 16 to 18).
- The visual servo (`arm_servo`) has never commanded the real arm.
- The gripper unit conversion is a placeholder; the detector's aim point is a mask centroid, not a stem point; the sequencer's "object" outcome cannot tell a plant from a torn leaf.
- Sag model: only the 45 degree approach; azimuths 40 to 73 degrees, the radius dependence above 0.55 m, the tangential part and the LIFT leg with a plant are not measured.
- Integral motor gain: above 512, with a plant, other poses, hot motors, long holds, and combined with the model.
- The gain tests ran without the camera (fixed target frame), and the full stack (detector + mission control) has not been run on the real arm since the 2026-10-19 supply repair.
- Statistics: the grasp results are one operator on one setup (a few grasps on 2026-10-14 and 15 on 2026-10-16); no repeatability claim.

## 6. Command cheat sheet

```bash
# container and display
docker start claude; xhost +local:
# build
docker exec claude bash -lc 'source /opt/ros/noetic/setup.bash; cd /home/dev && catkin_make'
# power / bus check (stack stopped)
docker exec claude bash -lc 'source /opt/ros/noetic/setup.bash; source /home/dev/devel/setup.bash; rosrun arm_interface motor_diag --device /dev/ttyUSB1'
# stack (own master 11313), see 3.2; arm status and parking
export ROS_MASTER_URI=http://localhost:11313
rosrun arm_interface estop_cli.py status
rosservice call /preset_manager/command "{action: go, name: default}"
# grasp (or use the Mission Control panel)
rosservice call /mission_control/command "{action: refresh}"
rosservice call /grasp_sequencer/grasp "{target_frame: grasp_target, dry_run: true, probe: false}"
# free-air sag test and gain sweep (move the arm; e-stop in reach, free space)
rosrun arm_grasp sag_test.py --azimuth 0 --go
rosrun arm_grasp gain_sweep.py --azimuth 0 --no-marker --floor-z -0.596 --values 64,128 --go
```
