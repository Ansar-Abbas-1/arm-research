#!/usr/bin/env python3
"""Prints the marker poses in the base frame reported by marker_localizer_node.

    rosrun arm_perception marker_pose_cli.py            wait for one report with a valid marker, print it, exit 0
                                                         (exit 1 after --timeout seconds without one, printing the last report)
    rosrun arm_perception marker_pose_cli.py --watch    print every report
    --id N      only marker N
Reads ~report only; it sends nothing.
"""
import argparse
import json
import sys

import rospy
from std_msgs.msg import String

from arm_perception import marker_logic as ml


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true")
    ap.add_argument("--id", type=int)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--topic", default="/marker_localizer/report")
    a = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("marker_pose_cli", anonymous=True)
    state = {"done": False, "ok": False, "last": None}

    def on_report(msg):
        rep = json.loads(msg.data)
        if a.id is not None:
            rep["markers"] = [m for m in rep["markers"] if m["id"] == a.id]
        state["last"] = rep
        if a.watch:
            print(ml.format_report(rep))
            sys.stdout.flush()
        elif any(m["valid"] for m in rep["markers"]):
            print(ml.format_report(rep))
            state["done"], state["ok"] = True, True

    rospy.Subscriber(a.topic, String, on_report, queue_size=1)
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and not state["done"]:
        if not a.watch and (rospy.Time.now() - t0).to_sec() > a.timeout:
            if state["last"] is not None:
                print(ml.format_report(state["last"]))
            print("no valid marker within %.0f s (is the camera still, the marker in view, marker_localizer running?)" % a.timeout)
            return 1
        rospy.sleep(0.1)
    return 0 if state["ok"] or a.watch else 1


if __name__ == "__main__":
    sys.exit(main())
