#!/usr/bin/env python3
"""Offline hand-eye solve on a stored capture session (no ROS master needed).

  rosrun arm_camera hand_eye_solve.py --session ~/.ros/arm_camera/hand_eye_20261006_101500 \\
        --out $(rospack find arm_camera)/config/hand_eye.yaml

Runs all five OpenCV methods, prints how consistent each one makes the fixed board look
(the smaller the scatter, the better) and writes the best one.
"""
import argparse
import glob
import os
import sys

from arm_camera import hand_eye, session


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", default="latest", help="session directory, or 'latest' (newest under ~/.ros/arm_camera)")
    ap.add_argument("--out", default=os.path.expanduser("~/.ros/arm_camera/hand_eye.yaml"))
    ap.add_argument("--min-samples", type=int, default=8)
    ap.add_argument("--force", action="store_true", help="solve even with too few samples or poor pose diversity")
    args = ap.parse_args()

    directory = args.session
    if directory == "latest":
        found = sorted(glob.glob(os.path.expanduser("~/.ros/arm_camera/hand_eye_*")))
        if not found:
            sys.exit("no session found under ~/.ros/arm_camera")
        directory = found[-1]
    samples = session.load_samples(directory)
    board, K, D, size, dark = session.read_session_meta(directory)
    rot, axis = hand_eye.motion_diversity(samples)
    print("session %s: %d samples, board %dx%d @ %.1f mm" % (directory, len(samples), board.cols, board.rows, 1000 * board.square_size))
    print("pose diversity: largest rotation between poses %.0f deg, largest angle between rotation axes %.0f deg" % (rot, axis))
    problems = []
    if len(samples) < args.min_samples:
        problems.append("only %d samples (use 10 to 20)" % len(samples))
    if rot < 20.0:
        problems.append("poses differ by at most %.0f deg of rotation (want 40 or more)" % rot)
    if axis < 20.0:
        problems.append("rotation axes are nearly parallel (%.0f deg apart): vary joint_1 AND the pitch joints" % axis)
    for p in problems:
        print("WARNING: " + p)
    if problems and not args.force:
        sys.exit("refusing to solve; fix the above or pass --force")

    results = hand_eye.solve_all(samples, K, D, board)
    if not results:
        sys.exit("no method converged")
    print("\n%-11s %14s %14s %12s" % ("method", "board scatter", "board scatter", "reprojection"))
    print("%-11s %14s %14s %12s" % ("", "position [mm]", "rotation [deg]", "RMS [px]"))
    for r in results:
        print("%-11s %14.1f %14.2f %12.2f" % (r.method, r.position_scatter_mm, r.rotation_scatter_deg, r.reprojection_rms_px))
    best = results[0]
    t = best.X[:3, 3]
    print("\nbest: %s   camera in end_effector frame: x=%.4f y=%.4f z=%.4f m" % (best.method, t[0], t[1], t[2]))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    session.save_hand_eye(args.out, best, board, len(samples), (rot, axis), dark, size)
    print("written to " + args.out)


if __name__ == "__main__":
    main()
