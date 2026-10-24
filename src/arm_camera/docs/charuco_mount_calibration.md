# Camera mount from a ChArUco board on the ground

**Does it work?** Yes, provided the board's full pose in `base_link` is known, not just a distance.
With the board's pose `T_base_board`, the arm's own pose `T_base_ee` (forward kinematics) and the camera's
view of the board `T_cam_board` (detected corners), one view already fixes the camera mount:

    X = T_end_effector_camera = inv(T_base_ee) * T_base_board * inv(T_cam_board)

A single distance is not enough: `X` has 6 degrees of freedom and a distance fixes one number. For a board
lying face up and level on the ground the pose reduces to what you can measure: the position of one
reference point (x, y, z) and one angle (yaw). Roll and pitch are zero if the ground is level.
Without any measurement the same views can be solved by hand-eye calibration (no board pose needed, but
8+ views with rotation about two axes); the tool does both and compares them.

ChArUco is a better target than a plain chessboard here: the marker ids make the board orientation
unambiguous (no 180 degree flip, which broke half the views of the plain chessboard) and partial views
still give a pose, which matters because a ground board seen from a downward-looking camera on this arm is
often partly out of frame.

## Print the board

```bash
rosrun arm_camera make_charuco_board.py --out-dir ~/charuco
```
Writes `charuco_8x6_30mm_4X4_50.svg` (the board at exact physical size, 240 x 180 mm plus margin: print at
100 %, "actual size", never "fit to page") and `..._frame.png`, which draws the board frame. Glue it flat on
a rigid plate. **Measure a printed square and the black marker inside it** and put the measured values in
`config/charuco.yaml` (`square_length`, `marker_length`, metres): they set the scale of the result.
A print is already in `printables/`.

## The board frame (what the board pose refers to)

Origin at the board's **bottom-left outer corner** as printed, x along the columns (to the right), y up
the page, z out of the paper toward you. Lying face up on the ground, that is a normal frame with z up:
its pose in `base_link` is the position of that corner and the board's **yaw**, the direction of its x
axis measured counter-clockwise from base x seen from above. If it is easier to locate the middle of the
board, pass `origin:=center`. z is the height of the printed surface above the `base_link` origin.

### Measuring the board pose

1. **With a ruler** relative to a point you know on the arm base, in the axes of `base_link` (check them with
   `rosrun tf tf_echo base_link end_effector` at a pose you can judge by eye). Use a long straight edge for
   the yaw.
2. **With the arm itself (no ruler):** release the arm or jog it slowly, touch the end-effector tip to the
   board's origin corner and read `rosrun tf tf_echo base_link end_effector`: that is x, y, z. Touch a
   second point along the board's bottom edge (its +x direction): `yaw = atan2(dy, dx)`. This inherits the
   arm's own accuracy (4 to 7 mm), which is also what limits the result anyway.
   Do this with the usual care: slow moves, a hand on the power switch.

## Procedure

Start the arm stack (TF `base_link -> end_effector`), then:

```bash
roslaunch arm_camera charuco_mount_calibration.launch known_pose:=true x:=0.40 y:=0.10 z:=0.0 yaw:=15 origin:=corner
```
(Without `known_pose` the node runs hand-eye mode.) Move the arm with the usual tools so the camera sees the
board from 6 to 15 different poses, hold still, and for each:

```bash
rosservice call /charuco_mount_calibration/capture
```
The reply says what is wrong if it refuses (arm moving, fewer than 12 corners, poor fit, board too frontal
to tell its orientation) and otherwise shows the mount estimate from that view and from all views so far,
with the spread. Vary the distance (0.25 to 0.5 m), tilt the camera at least 10 degrees to the board, and
move joint_1 as well as the pitch joints (needed for the hand-eye cross-check). Then:

```bash
rosservice call /charuco_mount_calibration/solve
```
It prints the camera pose in the end-effector frame, the view-to-view spread, the cross-check, and the
arguments for `roslaunch arm_camera camera_static_tf.launch x:= y:= z:= yaw:= pitch:= roll:=`, and writes the
result (hand_eye.yaml format) to `output_file` so `hand_eye_tf_publisher.py` publishes it. The node never
moves the arm.

**Wrong board pose? Do not re-capture.** The views are stored (`~/.ros/arm_camera/charuco_<time>/`):
```bash
rosrun arm_camera charuco_mount_solve.py --session latest --board-pose 0.40 0.10 0.0 16.5 --origin corner --out <file>
```

## Reading the result

* **view-to-view spread**: how much the single-view estimates differ. It contains the arm's accuracy
  (sag, repeatability), noise in the detections and any error in the board pose.
* **cross-check** (needs 8+ views with 20 degrees of rotation about two axes): hand-eye solves the mount
  without using your measured board pose and also finds where the board is. "The board pose hand-eye finds
  differs from the one you entered by X mm / Y deg" is an estimate of how wrong your measurement is, and
  "mount differs by" is how much that error matters. In the synthetic test a yaw entered 2 degrees wrong
  was reported as 1.99 degrees.

## Accuracy

Every error in the measured board pose goes into the mount: a position error 1:1, a yaw error as a rotation
error of the same angle plus a translation error of about distance x angle (3 degrees at 0.4 m: about 10 mm
in the synthetic test). Add the arm's own error. Expect a few millimetres and a degree or so if the board
is placed carefully; the cross-check tells you which it is. Averaging over many views reduces the noise,
not a systematic error in the board pose.

## Tests

`test/test_charuco.py`: 21 tests (pose from rendered boards, partial views, the frame conventions,
recovering the mount from a known board pose, sensitivity to a wrong yaw, the robust average, the
hand-eye cross-check, output file round trip). `test/e2e_charuco_synthetic.py`: the node and the offline
tool against a synthetic camera and simulated TF on a private master: a yaw entered 2 degrees wrong gives a
4.9 mm / 1.97 degree mount error and a cross-check of 1.99 degrees; re-solving the stored views with the
right yaw gives 0.35 mm / 0.002 degrees. Not yet run with the real camera and a real board.

## Limits

* Needs the intrinsic calibration (`config/camera_info.yaml`); a wrong focal length scales the result.
* The overlay (`overlay.launch`) draws a chessboard and refuses a ChArUco-based file (the camera pose in
  the file is still published by `hand_eye_tf_publisher.py`).
* A board that is not level, or a ground that is not level, breaks the "yaw only" input: pass a tilted
  pose only through the library (`charuco.mount_from_known_board`).
