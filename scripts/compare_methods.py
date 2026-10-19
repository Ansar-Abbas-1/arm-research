#!/usr/bin/env python3
"""Free-air COMPARISON of the ways to remove the arm's sag, on one grid (it COMMANDS THE ARM and WRITES THE MOTOR I GAIN through gain_sweep.py).

Configurations (azimuth 0, 45 degree approach, the same 4 points, the sequencer's settle 0.8 s as deployed, free air, gripper never commanded):
    raw          no feed-forward, no corrections, I 0
    sag          the sag model's feed-forward (sag_gain 1), no corrections
    corr         no feed-forward, the correction loop re-aims (probe_corrections, 2 steps, gain 0.7, trigger 10 mm)
    sag+corr     the deployed pipeline: feed-forward and corrections
    I128         motor integral gain 128 on joints 1 to 5, nothing else
    I128+corr    motor integral gain 128 and the correction loop
For every configuration it sets the sequencer's parameters (sag_gain, correction_settle, probe_corrections), respawns the sequencer, runs gain_sweep.py and keeps its JSON.
It prints, per configuration and leg, the residual to the waypoint at the first measurement ("first") and after the correction steps ("final"), in mm.

    rosrun arm_grasp compare_methods.py --azimuth 0 --no-marker --floor-z -0.596            # prints the plan, moves nothing
    rosrun arm_grasp compare_methods.py --azimuth 0 --no-marker --floor-z -0.596 --go       # the real runs (e-stop in reach, free space)
"""
import argparse
import json
import os
import subprocess
import sys
import time

import rospy

CONFIGS = [
    # name, sag_gain, corrections in the probe, I gain
    ("raw", 0.0, False, 0),
    ("sag", 1.0, False, 0),
    ("corr", 0.0, True, 0),
    ("sag+corr", 1.0, True, 0),
    ("I128", 0.0, False, 128),
    ("I128+corr", 0.0, True, 128),
]


def restart_sequencer(settle, sag_gain, corrects):
    rospy.set_param("/grasp_sequencer/correction_settle", float(settle))
    rospy.set_param("/grasp_sequencer/sag_gain", float(sag_gain))
    rospy.set_param("/grasp_sequencer/probe_corrections", bool(corrects))
    subprocess.call(["rosnode", "kill", "/grasp_sequencer"], stdout=subprocess.DEVNULL)
    time.sleep(3.0)
    rospy.wait_for_service("/grasp_sequencer/grasp", 30.0)                      # respawned by the launch file (2 s)
    time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--azimuth", type=float, required=True)
    ap.add_argument("--radii", default="0.46,0.52")
    ap.add_argument("--heights", default="0.07,0.09")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--settle", type=float, default=0.8, help="the sequencer's measurement settle [s] (the deployed value)")
    ap.add_argument("--log-dir", default="/home/dev/recordings/gain_test")
    ap.add_argument("--only", default="", help="comma list of configuration names to run")
    ap.add_argument("--no-marker", action="store_true")
    ap.add_argument("--floor-z", type=float, default=-0.605)
    ap.add_argument("--max-std-deg", type=float, default=1.0)
    ap.add_argument("--go", action="store_true", help="MOVE THE ARM and write the motor gain (without it: prints the plan)")
    a, _ = ap.parse_known_args(rospy.myargv()[1:])
    configs = [c for c in CONFIGS if not a.only or c[0] in a.only.split(",")]
    print("configurations: " + ", ".join("%s (sag_gain %g, corrections %s, I %d)" % c for c in configs))
    if not a.go:
        print("dry: nothing run (add --go)")
        return 0
    rospy.init_node("compare_methods")
    here = os.path.dirname(os.path.abspath(__file__))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    results = {}
    try:
        for name, sag, corrects, igain in configs:
            print("\n######## %s" % name, flush=True)
            restart_sequencer(a.settle, sag, corrects)
            out = os.path.join(a.log_dir, "compare_%s_%s_%s.json" % (stamp, name.replace("+", "_"), igain))
            cmd = [sys.executable, "-u", os.path.join(here, "gain_sweep.py"), "--azimuth", str(a.azimuth), "--radii", a.radii, "--heights", a.heights,
                   "--repeats", str(a.repeats), "--values", str(igain), "--log-dir", a.log_dir, "--out", out, "--max-std-deg", str(a.max_std_deg),
                   "--curve-times", "0.8", "--go"] + (["--allow-sag-model"] if sag else []) + (["--no-marker", "--floor-z", str(a.floor_z)] if a.no_marker else [])
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            print("\n".join(l for l in proc.stdout.splitlines() if "TF_OLD" not in l and "TF_REPEATED" not in l and "buffer_core" not in l), flush=True)
            if proc.returncode != 0 or not os.path.exists(out):
                print("configuration %s stopped (exit %d): not comparing the rest" % (name, proc.returncode))
                break
            results[name] = json.load(open(out))["results"][0]["legs"]
    finally:
        subprocess.call(["rosparam", "set", "/grasp_sequencer/probe_corrections", "false"], stdout=subprocess.DEVNULL)
        subprocess.call(["rosparam", "set", "/grasp_sequencer/sag_gain", "0.0"], stdout=subprocess.DEVNULL)
        print("\n== comparison (mm to the waypoint, mean of the points; first = first measurement, final = after the correction steps) ==")
        print("%-10s %-14s %-14s %-14s %s" % ("config", "PRE first/final", "GRASP f/f", "LIFT f/f", "mean of the three (final)"))
        for name, legs in results.items():
            cells, finals = [], []
            for leg in ("PRE", "GRASP", "LIFT"):
                l = legs.get(leg, {})
                first, fin = l.get("norm_mean_mm"), l.get("final", {}).get("norm_mean_mm")
                cells.append("%s / %s" % ("-" if first is None else "%.1f" % first, "-" if fin is None else "%.1f" % fin))
                if fin is not None:
                    finals.append(fin)
            print("%-10s %-14s %-14s %-14s %s" % (name, cells[0], cells[1], cells[2], "%.1f" % (sum(finals) / len(finals)) if finals else "-"))
        json.dump(results, open(os.path.join(a.log_dir, "compare_%s_summary.json" % stamp), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
