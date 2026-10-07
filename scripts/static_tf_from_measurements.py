#!/usr/bin/env python3
"""Turns hand measurements of the camera mount into the arguments of camera_static_tf.launch.

Put the arm in a pose (the `default` preset is convenient), then measure, in room directions (forward = base +x,
left = base +y, up = base +z):
  --offset A B C   lens centre relative to the end_effector origin [m], signed (e.g. 0.04 0 0.03); in forward / left /
                   up room directions by default, or in the link_5 / end_effector x y z axes with --frame end_effector
  --optical-axis   where the camera looks: forward|back|left|right|up|down or a vector "x,y,z"
  --image-right    which way the right edge of the image is
  --image-down     (optional check) which way the bottom edge of the image is
--frame end_effector: offset and directions are given in the axes of link_5 / end_effector (they share their
orientation), so no pose is needed; --origin link_5 means the offset is measured from link_5's origin (the
0.105 m to the end_effector origin along z is subtracted); --sensor-to-optical-centre M moves a point measured
on the sensor M metres along the optical axis to the lens (heuristic, about 0.004 for this webcam).
Otherwise the end effector pose of that moment is read from TF (base_link -> end_effector) unless --ee-pose is given
(x y z qx qy qz qw, for offline use). Prints the launch arguments; --output writes a hand_eye.yaml-format file
that hand_eye_tf_publisher.py reads. Nothing is sent to the arm. See docs/camera_calibration.md.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src"))
from arm_camera import static_mount  # noqa: E402
from arm_camera.geometry import make_transform, matrix_from_quat, quat_from_matrix  # noqa: E402


def read_tf_pose(base, ee):
    import rospy
    import tf2_ros
    rospy.init_node("static_tf_from_measurements", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    t = buf.lookup_transform(base, ee, rospy.Time(0), rospy.Duration(5.0)).transform
    return make_transform(matrix_from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
                          [t.translation.x, t.translation.y, t.translation.z])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offset", nargs=3, type=float, required=True, metavar=("FORWARD", "LEFT", "UP"))
    ap.add_argument("--optical-axis", required=True)
    ap.add_argument("--image-right", required=True)
    ap.add_argument("--image-down")
    ap.add_argument("--ee-pose", nargs=7, type=float, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    ap.add_argument("--frame", choices=["room", "end_effector"], default="room")
    ap.add_argument("--origin", choices=["end_effector", "link_5"], default="end_effector")
    ap.add_argument("--sensor-to-optical-centre", type=float, default=0.0, metavar="M")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--ee-frame", default="end_effector")
    ap.add_argument("--output", help="write a hand_eye.yaml-format file here")
    a = ap.parse_args()
    if a.frame == "end_effector":
        T_be = np.eye(4)          # directions already in the end_effector axes
    elif a.ee_pose:
        v = a.ee_pose
        T_be = make_transform(matrix_from_quat(v[3:]), v[:3])
    else:
        T_be = read_tf_pose(a.base_frame, a.ee_frame)
    try:
        axis = static_mount.parse_direction(a.optical_axis)
        offset = static_mount.optical_centre(a.offset, axis, a.sensor_to_optical_centre)
        if a.origin == "link_5":
            offset = static_mount.offset_from_end_effector(offset)
        T, skew = static_mount.mount_from_measurements(
            T_be, offset, axis, static_mount.parse_direction(a.image_right),
            static_mount.parse_direction(a.image_down) if a.image_down else None)
    except ValueError as e:
        print("ERROR:", e)
        return 1
    if a.frame == "room":
        print("end_effector in %s: %s" % (a.base_frame, np.round(T_be[:3, 3], 4)))
    print("image-right / optical axis were %.1f deg from perpendicular (corrected)" % skew)
    print("camera in end_effector: t = %s m" % np.round(T[:3, 3], 4))
    print("\nroslaunch arm_camera camera_static_tf.launch " + static_mount.launch_args(T))
    print("or: roslaunch arm_camera arm_with_camera.launch camera_tf:=static " + static_mount.launch_args(T))
    if a.output:
        import yaml
        d = {"parent_frame": a.ee_frame, "child_frame": "camera_optical_frame",
             "translation": [float(c) for c in T[:3, 3]], "quaternion_xyzw": [float(c) for c in quat_from_matrix(T[:3, :3])],
             "method": "hand_measured", "note": "measured by hand in room directions; a placeholder, not a calibration"}
        with open(a.output, "w") as f:
            yaml.safe_dump(d, f, sort_keys=False)
        print("written", a.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
