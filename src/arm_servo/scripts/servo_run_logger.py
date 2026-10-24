#!/usr/bin/env python3
"""Records every visual servo run to a JSON-lines file (read-only: it only subscribes). One file per run in ~output_dir
(default ~/.ros/arm_servo/runs), named servo_run_<time>.jsonl; summarise it with servo_report.py. Run it next to
visual_servo (launch/visual_servo.launch starts it with logging:=true)."""
import json
import os
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_servo import run_log


def main():
    rospy.init_node("servo_run_logger")
    out_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/arm_servo/runs"))
    rec = run_log.RunRecorder(tail_s=float(rospy.get_param("~tail_s", 1.0)))
    names = ["joint_%d" % i for i in range(1, 6)]
    q = {"v": None}

    def on_joints(m):
        d = dict(zip(m.name, m.position))
        if all(n in d for n in names):
            q["v"] = [d[n] for n in names]

    def on_status(m):
        run = rec.feed(rospy.Time.now().to_sec(), json.loads(m.data), q["v"])
        if run:
            path = os.path.join(out_dir, "servo_run_%s.jsonl" % time.strftime("%Y%m%d_%H%M%S"))
            run_log.save_run(path, run)
            rospy.logwarn("servo run saved: %s (%d records, final state %s)", path, len(run), run[-1]["state"])

    rospy.Subscriber("/joint_states_in", JointState, on_joints, queue_size=1)
    rospy.Subscriber("/visual_servo/status", String, on_status, queue_size=20)
    rospy.spin()


if __name__ == "__main__":
    main()
