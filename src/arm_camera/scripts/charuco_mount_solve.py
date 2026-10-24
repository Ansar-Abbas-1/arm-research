#!/usr/bin/env python3
"""Offline solve of the camera mount from stored ChArUco views (no ROS master, no camera).

  rosrun arm_camera charuco_mount_solve.py --session ~/.ros/arm_camera/charuco_YYYYmmdd_HHMMSS \\
        --board-pose 0.40 0.10 0.0 15 --origin center --out $(rospack find arm_camera)/config/hand_eye.yaml

Use it to re-solve the same views with a corrected board pose (re-measuring the board is much cheaper
than re-capturing). Without --board-pose it uses hand-eye, which needs no board pose but 8+ views with
rotation about two axes. Prints the camera pose also as x y z yaw pitch roll for camera_static_tf.launch.
"""
import argparse
import glob
import os
import sys

import numpy as np
import yaml

from arm_camera import charuco
from arm_camera import mount_calibration as mc


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", default="latest")
    ap.add_argument("--board", default=os.path.join(here, "config/charuco.yaml"))
    ap.add_argument("--board-pose", type=float, nargs=4, metavar=("X", "Y", "Z", "YAW_DEG"))
    ap.add_argument("--origin", default="corner", choices=["corner", "center"], help="what X Y Z refer to")
    ap.add_argument("--method", default="auto", choices=["auto", "known", "handeye"])
    ap.add_argument("--out")
    args = ap.parse_args()

    directory = args.session
    if directory == "latest":
        found = sorted(glob.glob(os.path.expanduser("~/.ros/arm_camera/charuco_*")))
        if not found:
            sys.exit("no session under ~/.ros/arm_camera")
        directory = found[-1]
    c = yaml.safe_load(open(args.board))
    spec = charuco.CharucoSpec(int(c["squares_x"]), int(c["squares_y"]), float(c["square_length"]), float(c["marker_length"]), c["dictionary"])
    samples = mc.load_samples(directory)
    T_bb = charuco.board_pose_in_base(*args.board_pose, spec=spec, origin=args.origin) if args.board_pose else None
    print("session %s: %d views" % (directory, len(samples)))
    res = mc.solve(samples, T_bb, args.method)
    print(mc.report(res))
    if args.out:
        info = dict(zip(["x", "y", "z", "yaw_deg"], args.board_pose), origin=args.origin) if args.board_pose else None
        mc.save_mount_yaml(args.out, res, spec, info)
        print("written to " + args.out)


if __name__ == "__main__":
    main()
