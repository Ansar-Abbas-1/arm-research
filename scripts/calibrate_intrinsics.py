#!/usr/bin/env python3
"""Offline intrinsic calibration from saved chessboard images (no ROS master, no camera).

  rosrun arm_camera calibrate_intrinsics.py --images /tmp/calibrationdata \\
        --out $(rospack find arm_camera)/config/camera_info.yaml --compare /root/.ros/camera_info/vitade_af.yaml

Takes the images the camera_calibration GUI saves with SAVE (left-*.png) or any folder of board
photos, detects the corners with the same detector the hand-eye code uses, fits the plumb-bob
model, drops the worst views one at a time, and reports the error of every view, the image
coverage and how well each parameter is determined (resampling). Real detections only.
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml

from arm_camera import board as board_mod
from arm_camera import intrinsics


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", required=True, help="folder with the board images (*.png / *.jpg)")
    ap.add_argument("--board", default=os.path.join(here, "config/board.yaml"))
    ap.add_argument("--out", help="write a ROS camera_info yaml here")
    ap.add_argument("--name", default="vitade_af")
    ap.add_argument("--compare", help="an existing camera_info yaml to compare with (e.g. the GUI result)")
    ap.add_argument("--target-rms", type=float, default=0.6, help="stop dropping views at this RMS [px]")
    ap.add_argument("--min-keep", type=float, default=0.6, help="never keep fewer than this fraction of the views")
    ap.add_argument("--bootstrap", type=int, default=8, help="resamples for the parameter spread (each fit takes a while at full HD)")
    args = ap.parse_args()

    b = yaml.safe_load(open(args.board))
    board = board_mod.Board(int(b["cols"]), int(b["rows"]), float(b["square_size"]))
    files = sorted(sum((glob.glob(os.path.join(args.images, e)) for e in ("*.png", "*.jpg", "*.jpeg")), []))
    files = [f for f in files if os.path.basename(f).startswith(("left", "img", "image", "frame")) or True]
    corner_sets, used, size = [], [], None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        size = (img.shape[1], img.shape[0])
        c = board_mod.detect_corners(img, board)
        if c is not None:
            corner_sets.append(c)
            used.append(os.path.basename(f))
    print("%d images, board %dx%d found in %d (size %s)" % (len(files), board.cols, board.rows, len(corner_sets), size))
    if len(corner_sets) < 12:
        sys.exit("need at least 12 views with the board found")
    print("image coverage by the corners of all views: %.0f %%" % (100 * intrinsics.coverage(corner_sets, size)))

    res = intrinsics.calibrate(corner_sets, board, size, target_rms=args.target_rms, min_keep=args.min_keep, bootstrap=args.bootstrap)
    print("\nkept %d of %d views, dropped %d (worst first removed): %s" % (len(res.kept), len(corner_sets), len(res.dropped),
          ", ".join(used[i] for i in res.dropped) or "none"))
    print("overall RMS %.3f px (per view: median %.2f, max %.2f); coverage of the kept views %.0f %%" % (
        res.rms, np.median(res.per_view_rms), res.per_view_rms.max(),
        100 * intrinsics.coverage([corner_sets[i] for i in res.kept], size)))
    K, D = res.K, res.D
    print("\nfx %.1f  fy %.1f  cx %.1f  cy %.1f" % (K[0, 0], K[1, 1], K[0, 2], K[1, 2]))
    print("k1 %.4f  k2 %.4f  p1 %.5f  p2 %.5f  (k3 fixed at 0)" % tuple(D.reshape(-1)[:4]))
    if res.spread:
        print("resampling std (%d resamples): " % args.bootstrap + "  ".join("%s %.2f" % (k, v) for k, v in res.spread.items()))
    if args.compare:
        c = yaml.safe_load(open(args.compare))
        K0 = np.array(c["camera_matrix"]["data"]).reshape(3, 3)
        D0 = np.array(c["distortion_coefficients"]["data"])
        print("\nversus %s: dfx %+.1f  dfy %+.1f  dcx %+.1f  dcy %+.1f  dk1 %+.4f  dk2 %+.4f" % (
            os.path.basename(args.compare), K[0, 0] - K0[0, 0], K[1, 1] - K0[1, 1], K[0, 2] - K0[0, 2], K[1, 2] - K0[1, 2],
            D.reshape(-1)[0] - D0[0], D.reshape(-1)[1] - D0[1]))
    if args.out:
        with open(args.out, "w") as f:
            yaml.safe_dump(intrinsics.camera_info_dict(K, D, size, args.name), f, default_flow_style=None, sort_keys=False)
        print("\nwritten to " + args.out)


if __name__ == "__main__":
    main()
