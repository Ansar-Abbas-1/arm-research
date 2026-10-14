#!/usr/bin/env python3
"""Fits the arm-sag model from the grasp sequencer's measurement log (NO ROS) and prints what it found.

    python3 scripts/sag_fit.py --out config/sag_model.json                       # all logs in /home/dev/recordings/sag
    python3 scripts/sag_fit.py --log-dir /path --modes probe --out m.json         # only the free-air probes

Per approach-angle bucket and leg (PRE, GRASP, LIFT): the number of runs, the mean sag (the displacement the arm adds to a commanded aim, measured minus aim)
and its spread; the feed-forward offset is minus that. Point the sequencer's `sag_model` parameter at the file (config/grasp.yaml) to use it.
"""
import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from arm_grasp import sag_model as sm  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--log-dir", default="/home/dev/recordings/sag")
    ap.add_argument("--modes", default="real,probe", help="which measurements to use: real, probe or both")
    ap.add_argument("--bucket-deg", type=float, default=15.0)
    ap.add_argument("--out", default="", help="write the model here (json)")
    a = ap.parse_args()
    lines = []
    for f in sorted(glob.glob(os.path.join(a.log_dir, "sag_*.jsonl"))):
        with open(f) as fh:
            lines += fh.readlines()
    rows = sm.read_rows(lines)
    samples = sm.sag_samples(rows, tuple(a.modes.split(",")))
    print("%d log lines, %d first measurements used (%s)" % (len(rows), len(samples), a.modes))
    if not samples:
        print("nothing to fit")
        return 1
    model = sm.SagModel.fit(samples, a.bucket_deg)
    print("approach  leg    runs   mean sag [mm] (x, y, z)        spread [mm]            feed-forward [mm]")
    for centre in sorted(model.buckets):
        off = model.offsets(centre, 1.0, min_n=1)
        for key in sm.KEYS:
            b = model.buckets[centre].get(key)
            if not b:
                continue
            m, sd, o = 1000.0 * np.array(b["mean"]), 1000.0 * np.array(b["std"]), 1000.0 * off[key]
            print("%5.0f deg %-6s %4d   (%6.1f, %6.1f, %6.1f)   (%5.1f, %5.1f, %5.1f)   (%6.1f, %6.1f, %6.1f)" % (centre, key, b["n"], *m, *sd, *o))
    if a.out:
        sm.save(model, a.out)
        print("model written to", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
