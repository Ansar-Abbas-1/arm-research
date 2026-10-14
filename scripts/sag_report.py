#!/usr/bin/env python3
"""Residual error of the arm at each leg, from the sequencer's sag log (NO ROS): how far the end effector was from the WAYPOINT at the first measurement of a leg,
split by whether the sag model's feed-forward was active (ff_mm non-zero in the log) and by the side of the start azimuth the target was on.

    python3 scripts/sag_report.py [--log-dir /home/dev/recordings/sag] [--since 20261014_150000]

Components are in the arm's frame at the waypoint's azimuth: radial (outward), tangential (+ = toward increasing azimuth), vertical; `norm` is the distance.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from arm_grasp import sag_model as sm  # noqa: E402


def residuals(rows, modes=("real", "probe"), since=""):
    """[(key, ff_on, side, [radial, tangential, vertical] of measured - goal, norm)] for the first measurement of each leg of every run."""
    out = []
    for r in rows:
        try:
            if r.get("mode", "real") not in modes or int(r["step"]) != 0 or r["key"] not in sm.KEYS or str(r.get("run", "")) < since:
                continue
            goal, e = np.asarray(r["goal"], float), np.asarray(r["measured"], float) - np.asarray(r["goal"], float)
            az = float(np.arctan2(goal[1], goal[0]))
            az0 = float(r["q_start"][0]) if r.get("q_start") else 0.0
            u, v = np.array([np.cos(az), np.sin(az), 0.0]), np.array([-np.sin(az), np.cos(az), 0.0])
            ff_on = any(abs(x) > 1e-6 for x in r.get("ff_mm", [0, 0, 0]))
            out.append((r["key"], ff_on, "+az" if az > az0 else "-az", np.array([e @ u, e @ v, e[2]]), float(np.linalg.norm(e))))
        except (KeyError, TypeError, ValueError, IndexError):
            continue
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log-dir", default="/home/dev/recordings/sag")
    ap.add_argument("--modes", default="real,probe")
    ap.add_argument("--since", default="", help="only runs whose id (YYYYmmdd_HHMMSS) is not before this")
    a = ap.parse_args()
    lines = []
    for f in sorted(glob.glob(os.path.join(a.log_dir, "sag_*.jsonl"))):
        with open(f) as fh:
            lines += fh.readlines()
    res = residuals(sm.read_rows(lines), tuple(a.modes.split(",")), a.since)
    if not res:
        print("no measurements")
        return 1
    print("leg    model  side  runs   mean error vs the waypoint [mm]: radial  tangent  vertical |   norm mean (max)")
    groups = {}
    for key, ff, side, comp, norm in res:
        groups.setdefault((sm.KEYS.index(key), ff, side), []).append((comp, norm))
    for (ki, ff, side), items in sorted(groups.items()):
        c = 1000.0 * np.mean([i[0] for i in items], axis=0)
        n = 1000.0 * np.array([i[1] for i in items])
        print("%-6s %-5s  %s   %3d                                    %7.1f %8.1f %9.1f |   %6.1f (%5.1f)" % (sm.KEYS[ki], "on" if ff else "off", side, len(items), *c, n.mean(), n.max()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
