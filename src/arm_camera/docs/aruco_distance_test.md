# ArUco distance test

Question answered: does the camera report the same distance to a marker as a ruler does? That
checks the camera's scale (focal length, from the intrinsic calibration) and your marker size.
It does NOT check where the camera is mounted on the arm (that needs the hand-eye calibration,
see "Calibrating the camera mount" below).

## Procedure

1. **Print** a marker: `printables/aruco_4X4_50_id0_100mm.svg` (and 150 mm for distances over
   about 1 m; ids 1 and 2 for more markers). Print at 100 % ("actual size", never "fit to page").
   Glue it flat on a rigid board: a bent marker biases the pose. Or make others:
   `rosrun arm_camera make_aruco_markers.py --dictionary 4X4_50 --ids 3 4 --size-mm 120 --out-dir ~/aruco`.
2. **Measure the black square** of the printed marker with a ruler (the whole black square,
   border included, not the white margin). A 1 mm error on 100 mm is 1 % of the distance: at
   0.5 m that is 5 mm. The PNG also has a 50 mm bar to check the print scale.
3. **Start** (camera plugged in, container started after it):
   ```bash
   roslaunch arm_camera aruco_test.launch marker_size:=0.100
   ```
   `dictionary:=auto` finds the dictionary by itself and locks it. If your marker is not from
   these printables, `auto` tries 21 dictionaries; pass the name once known. rviz shows the
   marker, a yellow line from the camera to it with "id N: d mm", the TF frames, and the
   annotated image (green outline, axes, `d=`, the fit error and an ambiguity value).
4. **Measure with the ruler** from the same point the camera measures from: the camera's
   *optical centre* (the origin of `camera_optical_frame`), which is inside the lens, about one
   focal length in front of the sensor, roughly 2 to 4 mm for this camera. Measuring to the
   marker centre (crossing of the diagonals). If you measure from the sensor surface, subtract that
   few mm with `--offset`.
5. **Compare** (marker id 0 here, ruler 452 mm):
   ```bash
   rosrun arm_camera aruco_distance_report.py --id 0 --measured 452 --offset 3
   ```
   It reads the TF `camera_optical_frame -> aruco_0` (the same one rviz draws; you can also see
   it with `rosrun tf tf_echo camera_optical_frame aruco_0`), averages it for 5 s, and prints the
   mean, the spread, and the error in mm and percent.

Repeat at 3 or 4 distances (0.3, 0.5, 0.8, 1.2 m), with the marker not exactly frontal (about
20 to 40 degrees of tilt avoids the planar pose ambiguity). Reading the errors: a constant
offset means you measured from a different point; an error proportional to the distance means
`marker_size` or the focal length; an error that grows faster than the distance means a
distortion model problem.

## Expected accuracy (heuristic, to be replaced by your measurements)

Depth error is about `distance^2 / (focal_px * marker_size) * corner_error_px`. With
focal 1521 px, a 100 mm marker and 0.3 px corner noise: about 0.5 mm at 0.5 m and 2 mm at 1 m;
use a 150 mm marker beyond about 1 m. On top of that: the marker size you typed (the dominant
term), the intrinsic calibration (the principal-point uncertainty of about 20 px matters for
the lateral position, hardly for the distance), and the ruler. Expect agreement within about 3 to
5 mm at 0.5 m if the marker size is measured to 0.3 mm.

## Marker in base_link (once the camera mount is known)

The marker's pose in the base frame follows from the TF chain
`base_link -> ... -> end_effector -> camera_optical_frame -> aruco_N`. The middle link comes from
the hand-eye calibration (`config/hand_eye.yaml`, published by `overlay.launch`) or, until then,
from static values you provide:
```bash
roslaunch arm_camera aruco_test.launch static_tf:=true x:=0.05 y:=0.0 z:=0.08 yaw:=0 pitch:=1.57 roll:=0
```
(`x y z` metres, `yaw pitch roll` radians, parent `end_effector`, child `camera_optical_frame`).
Run the arm stack as well. Then `rosrun tf tf_echo base_link aruco_0` gives the marker in the
base frame. The camera-to-marker distance above does not depend on these values.

## Calibrating the camera mount from markers (instead of typing static TF values)

The static transform `end_effector -> camera_optical_frame` is the hand-eye result. Options, from
least to most effort:

1. **Hand-eye with fixed markers (recommended, nothing to measure).** Print 4 to 6 markers and
   fix them on the floor or table around the arm, anywhere, with unknown positions. Move the arm to
   12 to 20 poses (vary joint_1 and the pitch, as for the chessboard), and each time record the
   arm pose (TF) and the pose of every visible marker. Each marker is a fixed target, so it gives
   its own constraint `T_base_marker = T_base_ee * X * T_cam_marker`; the existing hand-eye solver
   (`arm_camera.hand_eye`) takes one target at a time, so run it per marker and average the
   results (the spread between markers is the quality estimate). This is the chessboard
   procedure with a more convenient target: many markers visible at once means more data per
   pose and views from further away. It needs the detector node's poses to be recorded by the
   existing collector, which is a small extension (the collector currently detects a chessboard).
2. **Known marker poses (quickest if you can measure).** Place markers at positions you know in
   `base_link` (for example a printed grid sheet aligned to the base mounting, so x, y, yaw are
   read off the sheet and z is the floor height). The arm stays still: for each visible marker
   `X = T_base_ee^-1 * T_base_marker * T_cam_marker^-1`, and the median over markers and a few arm
   poses is the answer, the spread is the quality. No arm motion needed, but any error in the
   marker poses (2 mm, 1 degree) goes straight into X.
3. **ChArUco board with a known pose (implemented, `docs/charuco_mount_calibration.md`).** It does options 1 and 2 on one printed board with unique ids: single views give the mount if the board pose is known, and hand-eye gives it without. Prefer it to loose markers.
4. **Marker board of known layout** (an ArUco GridBoard on one printed sheet): the relative
   positions are known from the print to about 0.2 mm, which makes each pose estimate better (many
   corners, no planar ambiguity) and improves both options above. Costs a printed sheet.

The single-marker distance test above cannot give the mount: one marker constrains only the
distance to it, not the camera's orientation on the arm.

## Files

`src/arm_camera/aruco.py` (pure library), `scripts/aruco_detector_node.py`,
`scripts/aruco_distance_report.py`, `scripts/make_aruco_markers.py`, `launch/aruco_test.launch`,
`launch/camera_static_tf.launch`, `rviz/aruco.rviz`, `printables/`; tests `test/test_aruco.py`
(9 unit tests: pose and distance of rendered markers, linear dependence on the marker size,
ambiguity, two markers, dictionary auto-detection, SVG size) and `test/e2e_aruco_synthetic.py`
(nodes against a synthetic camera on a private master: reported 471.67 mm for a true 471.38 mm).
