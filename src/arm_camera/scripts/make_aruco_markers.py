#!/usr/bin/env python3
"""Writes printable ArUco markers: an SVG with exact millimetre dimensions (print at 100 %, 'actual
size', never 'fit to page') and a PNG. After printing, MEASURE the black square with a ruler and
pass that value as marker_size: the distance the camera reports scales linearly with it.

  rosrun arm_camera make_aruco_markers.py --dictionary 4X4_50 --ids 0 1 2 --size-mm 100 --out-dir ~/aruco
"""
import argparse
import os

import cv2
import numpy as np

from arm_camera import aruco


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dictionary", default="4X4_50", choices=sorted(aruco.DICTIONARIES))
    ap.add_argument("--ids", type=int, nargs="+", default=[0])
    ap.add_argument("--size-mm", type=float, default=100.0, help="side of the black square")
    ap.add_argument("--px-per-mm", type=float, default=10.0)
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()
    out = os.path.expanduser(args.out_dir)
    os.makedirs(out, exist_ok=True)
    for mid in args.ids:
        stem = os.path.join(out, "aruco_%s_id%d_%dmm" % (args.dictionary, mid, round(args.size_mm)))
        with open(stem + ".svg", "w") as f:
            f.write(aruco.marker_svg(args.dictionary, mid, args.size_mm))
        side = int(round(args.size_mm * args.px_per_mm))
        margin = side // 4
        img = np.full((side + 2 * margin + int(8 * args.px_per_mm), side + 2 * margin), 255, np.uint8)
        img[margin:margin + side, margin:margin + side] = aruco.render_marker(args.dictionary, mid, side)
        y = side + 2 * margin + int(5 * args.px_per_mm)
        cv2.putText(img, "%s id %d, black square %.1f mm" % (args.dictionary, mid, args.size_mm), (margin, y), cv2.FONT_HERSHEY_SIMPLEX, 0.0035 * args.px_per_mm * 10, 0, 2)
        bar = int(50 * args.px_per_mm)
        cv2.line(img, (img.shape[1] - margin - bar, y - 6), (img.shape[1] - margin, y - 6), 0, 3)
        cv2.imwrite(stem + ".png", img)
        print("wrote %s.svg and .png  (black square %.1f mm; the 50 mm bar on the PNG checks the print scale)" % (stem, args.size_mm))


if __name__ == "__main__":
    main()
