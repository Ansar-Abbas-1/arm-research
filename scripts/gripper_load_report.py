#!/usr/bin/env python3
"""Report on a gripper recording (rosbag from gripper_monitor.launch): the real load behaviour, per parameter it informs.

    rosrun arm_grasp gripper_load_report.py latest                 # newest bag in the recordings directory
    rosrun arm_grasp gripper_load_report.py path/to/x.bag --png load.png

Reads joint_6 from /joint_states_in (measured) and /joint_states_out (commanded), and |load| from
/arm_interface/motor_load, then prints arm_grasp/load_report.analyse's facts. Offline, sends nothing.
"""
import argparse
import glob
import os
import sys

import numpy as np
import rosbag

from arm_grasp import load_report as lr

DEFAULT_DIR = "/home/dev/recordings/gripper"


def series(bag):
    meas, cmd, load = ([], []), ([], []), ([], [])
    for topic, msg, t in bag.read_messages(topics=["/joint_states_in", "/joint_states_out", "/arm_interface/motor_load"]):
        if "joint_6" not in msg.name:
            continue
        i = msg.name.index("joint_6")
        ts = t.to_sec()
        if topic == "/joint_states_in" and i < len(msg.position):
            meas[0].append(ts); meas[1].append(msg.position[i])
        elif topic == "/joint_states_out" and i < len(msg.position):
            cmd[0].append(ts); cmd[1].append(msg.position[i])
        elif topic == "/arm_interface/motor_load" and i < len(msg.effort):
            load[0].append(ts); load[1].append(abs(msg.effort[i]))
    t0 = min(s[0][0] for s in (meas, cmd, load) if s[0])
    return [(np.array(s[0]) - t0, np.array(s[1])) for s in (meas, cmd, load)]


def plot(png, meas, cmd, load, target, limit=45.0):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(10, 9))
    ax[0].plot(*meas, label="measured"); ax[0].step(*cmd, where="post", label="command", alpha=0.7)
    ax[0].set_ylabel("joint_6 [m]"); ax[0].legend(); ax[0].grid(True)
    ax[1].plot(*load, ".-"); ax[1].axhline(target, color="g", ls="--", label="target %.0f %%" % target)
    ax[1].axhline(limit, color="r", ls="--", label="limit %.0f %%" % limit)
    ax[1].axhline(40.0, color="k", ls=":", label="steady state must stay under 40 %")
    ax[1].set_ylabel("|load| [%]"); ax[1].set_xlabel("t [s]"); ax[1].legend(); ax[1].grid(True)
    tl = load[0]
    err = 1000.0 * (np.interp(tl, *meas) - np.interp(tl, *cmd))
    ax[2].scatter(err, load[1], s=8)
    ax[2].set_xlabel("measured - command [mm] (positive: the command is ahead of the jaws)"); ax[2].set_ylabel("|load| [%]"); ax[2].grid(True)
    fig.tight_layout()
    fig.savefig(png, dpi=110)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag", help="a .bag file or 'latest'")
    ap.add_argument("--dir", default=DEFAULT_DIR)
    ap.add_argument("--png")
    ap.add_argument("--target", type=float, default=38.5, help="load_target of config/grasp.yaml")
    ap.add_argument("--limit", type=float, default=45.0, help="load_limit (peak) of config/grasp.yaml")
    a = ap.parse_args()
    path = a.bag
    if path == "latest":
        bags = sorted(glob.glob(os.path.join(a.dir, "*.bag")), key=os.path.getmtime)
        if not bags:
            print("no .bag in %s" % a.dir)
            return 1
        path = bags[-1]
    print("bag:", path)
    with rosbag.Bag(path) as bag:
        meas, cmd, load = series(bag)
    if len(load[0]) < 3 or len(meas[0]) < 3:
        print("not enough joint_6 / motor_load data in the bag (is the driver publishing /arm_interface/motor_load?)")
        return 1
    for line in lr.format_report(lr.analyse(meas, cmd if len(cmd[0]) else meas, load, a.target), a.target, a.limit):
        print(line)
    if a.png:
        plot(a.png, meas, cmd if len(cmd[0]) else meas, load, a.target, a.limit)
        print("plot:", a.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
