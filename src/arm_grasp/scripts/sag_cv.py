#!/usr/bin/env python3
"""Residual error of sag models, by group, with cross-validation (NO ROS): how well does the feed-forward predict the error at points the model was not fitted on?

    python3 scripts/sag_cv.py [--log-dir /home/dev/recordings/sag] [--model config/sag_model.json]

Groups = (side of the start azimuth: -az / 0 / +az) x (height: low = grasp heights up to 0.15 m above the floor, high above). For each group it prints the mean
distance to the waypoint after applying (a) the current model file, (b) a model refitted on ALL the data (in-sample), (c) a model refitted on all the OTHER groups
(out-of-group: the honest number for a new situation). The residual of a sample is |sag + feed-forward|: aim = waypoint + ff, measured = aim + sag.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from arm_grasp import sag_model as sm  # noqa: E402


def load_rows(log_dir):
    lines = []
    for f in sorted(glob.glob(os.path.join(log_dir, "sag_*.jsonl"))):
        with open(f) as fh:
            lines += fh.readlines()
    return sm.read_rows(lines)


def describe(r):
    """(group, key, sag_base, az, az0, approach_deg, radius, height) of a first-measurement row, or None."""
    try:
        if int(r["step"]) != 0 or r["key"] not in sm.KEYS:
            return None
        goal = np.asarray(r["goal"], float)
        s = np.asarray(r["measured"], float) - np.asarray(r["aim"], float)
        az = float(np.arctan2(goal[1], goal[0]))
        az0 = float(r["q_start"][0]) if r.get("q_start") else 0.0
        d = az - az0
        side = "0" if abs(d) < sm.TRAVEL_RAMP else ("+az" if d > 0 else "-az")
        t = np.asarray(r["target"], float) if r.get("target") else goal
        height = float(t[2]) + 0.605                      # about the floor (marker_0 z), to the nearest centimetre is enough
        return ("%s %s" % (side, "low" if height <= 0.15 else "high"), r["key"], s, az, az0, float(r["approach_deg"]), float(np.hypot(t[0], t[1])), height)
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def residual(model, d):
    group, key, s, az, az0, approach, _, _ = d
    ff = model.offsets(approach, az, az0, 1.0, min_n=1, max_offset=1.0)[key]
    return float(np.linalg.norm(s + ff))


def fit_rows(rows):
    return sm.SagModel.fit(sm.sag_samples(rows))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log-dir", default="/home/dev/recordings/sag")
    ap.add_argument("--model", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "sag_model.json"))
    ap.add_argument("--out", default="", help="write the model refitted on all the data here")
    a = ap.parse_args()
    rows = load_rows(a.log_dir)
    info = [(r, describe(r)) for r in rows]
    info = [(r, d) for r, d in info if d is not None]
    current = sm.load(a.model)
    full = fit_rows([r for r, _ in info])
    groups = sorted(set(d[0] for _, d in info))
    print("%d first measurements, %d groups; current model %s\n" % (len(info), len(groups), os.path.basename(a.model)))
    print("residual |sag + feed-forward| in mm, mean (max); the first columns use no model: the raw sag")
    print("%-9s %-6s %4s   %-14s %-14s %-14s %-14s" % ("group", "leg", "n", "no model", "current model", "refit (all)", "refit (others)"))
    totals = {"raw": [], "cur": [], "all": [], "cv": []}
    for g in groups:
        others = fit_rows([r for r, d in info if d[0] != g])
        for key in sm.KEYS:
            ds = [d for _, d in info if d[0] == g and d[1] == key]
            if not ds:
                continue
            raw = 1000 * np.array([np.linalg.norm(d[2]) for d in ds])
            cur, al, cv = (1000 * np.array([residual(m, d) for d in ds]) for m in (current, full, others))
            for k, v in zip(("raw", "cur", "all", "cv"), (raw, cur, al, cv)):
                totals[k] += list(v)
            print("%-9s %-6s %4d   %5.1f (%5.1f)  %5.1f (%5.1f)  %5.1f (%5.1f)  %5.1f (%5.1f)" % (
                g, key, len(ds), raw.mean(), raw.max(), cur.mean(), cur.max(), al.mean(), al.max(), cv.mean(), cv.max()))
    print("\nall samples: no model %.1f mm, current %.1f mm, refit (in-sample) %.1f mm, refit (out-of-group) %.1f mm" % tuple(np.mean(totals[k]) for k in ("raw", "cur", "all", "cv")))
    if a.out:
        sm.save(full, a.out)
        print("refitted model written to", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
