#!/usr/bin/env python3
"""Writes a printable ChArUco board for the mount calibration: an SVG at exact physical size (print at
100 %, 'actual size', never 'fit to page') and a PNG that shows the board frame (origin at the bottom-left
outer corner, x right, y up, z toward you), which is what the board pose you measure refers to.

  rosrun arm_camera make_charuco_board.py --out-dir ~/charuco
Size and layout come from config/charuco.yaml unless overridden. After printing, MEASURE a printed square
and the marker and put the measured values in config/charuco.yaml.
"""
import argparse
import os

import cv2
import yaml

from arm_camera import charuco


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(here, "config/charuco.yaml"))
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args()
    c = yaml.safe_load(open(args.config))
    spec = charuco.CharucoSpec(int(c["squares_x"]), int(c["squares_y"]), float(c["square_length"]), float(c["marker_length"]), c["dictionary"])
    out = os.path.expanduser(args.out_dir)
    os.makedirs(out, exist_ok=True)
    stem = os.path.join(out, "charuco_%dx%d_%dmm_%s" % (spec.squares_x, spec.squares_y, round(spec.square_length * 1000), spec.dictionary))
    with open(stem + ".svg", "w") as f:
        f.write(charuco.board_svg(spec))
    cv2.imwrite(stem + "_frame.png", charuco.annotated_board_png(spec))
    print("wrote %s.svg (%.0f x %.0f mm board, plus margin) and %s_frame.png" % (stem, 1000 * spec.width, 1000 * spec.height, stem))
    print("origin = bottom-left outer corner as printed; x right, y up, z out of the paper toward you.")


if __name__ == "__main__":
    main()
