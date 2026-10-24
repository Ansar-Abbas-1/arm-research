#!/usr/bin/env python3
"""Summary (and optional plot) of a recorded visual servo run.

  rosrun arm_servo servo_report.py ~/.ros/arm_servo/runs/servo_run_20261008_101500.jsonl [--png run.png]
  rosrun arm_servo servo_report.py latest        the newest run in ~/.ros/arm_servo/runs
"""
import argparse
import glob
import os

from arm_servo import run_log, run_report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--png")
    a = ap.parse_args()
    path = a.run
    if path == "latest":
        files = sorted(glob.glob(os.path.expanduser("~/.ros/arm_servo/runs/servo_run_*.jsonl")))
        if not files:
            raise SystemExit("no recorded runs in ~/.ros/arm_servo/runs")
        path = files[-1]
    records = run_log.load_run(path)
    print(path)
    print(run_report.format_summary(run_report.summarize(records)))
    if a.png:
        run_report.plot_run(records, a.png)
        print("plot written to " + a.png)


if __name__ == "__main__":
    main()
