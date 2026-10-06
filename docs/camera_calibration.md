# Eye-in-hand camera calibration (M6)

Goal: know, at every arm pose, where the camera is and how it maps pixels to rays, so that a
later milestone (M7 detection, M8 closed loop) can turn "pixel (u, v)" into "point in
`base_link`". Two things are calibrated, both with a **real printed chessboard and the real
arm**; the synthetic scenes in `test/` only test the code, nothing in the workflow runs in
simulation.

| What | How | Result |
|---|---|---|
| Intrinsics (focal lengths, principal point, lens distortion) | `camera_calibration` GUI, board shown from many angles | `config/camera_info.yaml` |
| Hand-eye `X = T_end_effector_camera` (where the camera sits on the arm) | `hand_eye_collector` + OpenCV `calibrateHandEye`, board fixed, arm moved | `config/hand_eye.yaml`, published as static TF `end_effector -> camera_optical_frame` |

For every hand-eye sample `B = T_base_ee * X * T_cam_board` must give the same `B` (the board
does not move). `T_base_ee` comes from TF (joint states and forward kinematics),
`T_cam_board` from `solvePnP` on the detected corners.

## Hardware (measured 2026-10-06)

Microdia Webcam Vitade AF (USB 0c45:6366), `/dev/video2` (`/dev/video3` is its metadata node).
MJPG 1920x1080 and 1280x720 at 30 fps (the first OpenCV test measured about 20 fps because
auto exposure lowered the rate in dim light, `exposure_dynamic_framerate`); YUYV only 25 or 10
fps at those sizes (USB 2.0), so the driver uses MJPEG. **Focus: despite the "AF" in its name
the camera exposes no focus control to V4L2** (`v4l2-ctl --list-ctrls` lists none), so the
focus cannot be locked in software. A focus that changes after or between calibrations changes
the focal length: calibrate with the board at the working distance in steady light and check
the sharpness does not hunt (move the board closer and farther and watch whether the image
keeps pumping). Exposure stays automatic (operator preference; `auto_exposure` is on in the camera): it
follows the light, and only `exposure_dynamic_framerate` (lets the frame rate drop to about
20 fps in dim light) and `brightness` (biases the auto-exposure target) are worth touching,
see `config/camera.yaml`. The first test frame was bright and slightly soft: add diffuse
light (or lower `brightness`); a sharp high-contrast board matters more than resolution. The
image arrived rotated (ceiling at the bottom): the mount orientation is whatever it is, `X`
absorbs it, but **never flip or mirror an image** that goes into a calibration.

## Setup

1. `ros-noetic-usb-cam` and `v4l-utils` are installed in the container (2026-10-06); after a
   container rebuild: `apt install ros-noetic-usb-cam v4l-utils`.
2. A camera plugged in after the container started is not visible in it. Restart the container
   or add the nodes: `docker exec claude bash -c 'mknod /dev/video2 c 81 2; chmod 666 /dev/video2'`
   (major 81, minor from `ls -l /dev/video2` on the host).
3. Print a chessboard, glue it to a rigid flat plate, **measure a square with a ruler** and edit
   `config/board.yaml` (`cols`, `rows` = INNER corners, `square_size` in metres). Use an even x
   odd corner count (9 x 6): otherwise the detector can flip the board by 180 degrees between
   views. The configured board is 9 x 6 inner corners (10 x 7 squares) with 23 mm between corners.

## Step 1: intrinsics

```bash
roslaunch arm_camera intrinsics_calibration.launch cols:=9 rows:=6 square:=0.025
```

Move the board (or the hand-held camera) until the X, Y, Size and Skew bars are green, press
CALIBRATE, SAVE, COMMIT. Copy `~/.ros/camera_info/vitade_af.yaml` to `config/camera_info.yaml`.
Check the reported reprojection error: well under 0.5 px at 1080p is good; much more means
bad focus, a bent board or a wrong `square`/`cols`/`rows`.

## Step 2: hand-eye

Start the arm stack first (TF `base_link -> end_effector` must exist), then:

```bash
roslaunch arm_camera hand_eye_collect.launch
```

Fix the board somewhere the camera can see it from many arm poses (flat on the table in
front of the arm is easiest) and do not move it during the session. Move the arm with the
usual tools (rviz marker, `reach_pose_cli.py`), wait until it is still, then
`rosservice call /hand_eye_collector/capture`; the reply tells you what is wrong if it
refuses (arm still moving, board not found, fit worse than 1 px, no intrinsics). The node
**never moves the arm**. Collect 12 to 20 samples, then
`rosservice call /hand_eye_collector/solve` (or offline: `rosrun arm_camera hand_eye_solve.py`).

What makes a good set, specific to this arm: it can rotate the camera only about the vertical
(joint_1) and about a horizontal pitch axis (joints 2 to 5). The rotation part of `X` needs
rotation axes that are not parallel, so use BOTH: vary joint_1 by 30 degrees or more each way
and vary the total pitch by 40 degrees or more, and vary the distance (0.25 to 0.5 m). The
collector prints the largest rotation between poses and the largest angle between rotation
axes after each capture; the solver refuses sets with under 20 degrees of either unless
`--force`. Avoid the singular upright pose and keep clear of the no-access zone.

Reading the result (`hand_eye_solve.py` table): "board scatter" is how much the fixed board
moves in `base_link` between samples when `X` is applied; "reprojection RMS" is the pixel
error. These are limited by the arm: its repeatability under load is about 4 to 7 mm, joints
sag under gravity, and forward kinematics assumes the URDF dimensions, so expect scatter of a
few millimetres and a few pixels of reprojection error, not the sub-pixel values of the
synthetic tests. (Heuristic expectation, to be replaced by the first real run.)

## Step 3: check with the overlay

```bash
roslaunch arm_camera overlay.launch
```

Opens rviz with the robot model, the board drawn as a checkerboard at its calibrated position,
the camera frustum, the RMS error as text, and the camera image with the prediction drawn on
it: green rings are detected corners, red crosses are where joint states, forward kinematics,
hand-eye and intrinsics say they must be, yellow lines join each pair. Move the arm to poses
you did NOT calibrate with: the crosses should stay on the rings everywhere. Systematic
offsets that grow with distance point at the intrinsics or `square_size`; offsets that depend
on the arm pose point at the hand-eye or at arm accuracy (sag, backlash).

## Frames and files

`base_link -> ... -> end_effector` (robot_state_publisher) `-> camera_optical_frame` (static,
from `hand_eye.yaml`; z forward, x right, y down in the image). The board frame has its origin
at the first inner corner, x along the columns, y along the rows.

| File | Role |
|---|---|
| `src/arm_camera/{geometry,board,hand_eye,session,overlay}.py` | pure library, no ROS |
| `scripts/hand_eye_collector.py` | capture, undo and solve services, debug image |
| `scripts/hand_eye_solve.py` | offline solve on a stored session |
| `scripts/hand_eye_tf_publisher.py` | static TF from `hand_eye.yaml` |
| `scripts/calibration_overlay_node.py` | image overlay and rviz markers |
| `config/{camera,board}.yaml`, `rviz/camera_overlay.rviz`, `launch/*.launch` | configuration |
| `test/test_calibration_pipeline.py` | 17 unit tests (geometry, detection on a rendered board, hand-eye with 0.25 deg / 1 mm noise, overlay geometry) |
| `test/e2e_synthetic.py` | nodes end to end with a synthetic camera on a private master (port 11312) |

Run: `cd src/arm_camera && PYTHONPATH=src python3 test/test_calibration_pipeline.py` and, with
ROS sourced, `python3 test/e2e_synthetic.py`. Last result (2026-10-06): 17/17; end to end 14/14
captures, camera pose recovered within 0.35 mm / 0.011 deg, overlay 0.1 px.

## Known limits

- Not run with the real camera or board yet; all numbers above from synthetic data.
- A detection that labels the board from the other end (180 degree flip) is consistent for an
  even x odd board and harmless to calibration; `first_square_dark` is read from the image so the
  rviz board is drawn correctly.
- Hand-eye uses the pose at the capture time from TF; an arm that is still moving or a TF
  delay larger than a few ms corrupts a sample, hence the stillness check.
- Rolling-shutter and motion blur: capture only when the arm is still.
- Intrinsics assume the plumb-bob (radial plus tangential) model; a very wide lens may need
  `rational_polynomial`, which the `camera_calibration` GUI offers.
