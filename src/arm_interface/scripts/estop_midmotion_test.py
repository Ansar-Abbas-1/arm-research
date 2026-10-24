#!/usr/bin/env python3
"""E-stop check (b) (docs/sim_to_real.md section 15): trigger the e-stop in
the MIDDLE of a reach and measure whether the arm stops and holds.

Starts the vetted pose-A reach (8 cm lateral of the pivot pose; from the rest
pose a ~6.4 s joint-space move, largest joint move 0.68 rad, collision-free)
through /reach_target_pose in a background thread, waits --trigger-after
seconds (default 2.5, near peak velocity), then publishes
/arm_interface/estop_trigger (std_msgs/Empty) from here. Joint positions are
sampled from /joint_states_in the whole time, and the report shows:
  - speed just before the trigger (the motion that has to be stopped),
  - how far each joint travelled after the trigger and how fast it was
    moving 0.1 / 0.3 / 1.0 s later,
  - drift between +0.5 s and +3 s (a held arm should not creep).

It does NOT reset the e-stop: the arm stays latched and holding when this
ends; call /arm_interface/estop_reset yourself.

Preconditions (aborts, nothing sent, if not met): the e-stop is released
(/arm_interface/estop_state false), and the arm is near the rest pose
(joints 2-5 within 0.2 rad of zero) so the vetted plan applies. Requires the
stack running with the correction launch args as usual. Support the arm.

Usage (run by the user):
  rosrun arm_interface estop_midmotion_test.py
"""
import argparse
import sys
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from arm_trajectory.srv import ReachTargetPose, ReachTargetPoseRequest

JOINTS = ["joint_%d" % i for i in range(1, 6)]
POSE_A = (0.1305, 0.0997, 0.5509)
QUAT = (-0.05113, 0.68049, 0.05477, 0.72892)  # pitch 86.07 deg, yaw 8.60 deg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--trigger-after", type=float, default=2.5, help="seconds after the reach starts")
    ap.add_argument("--no-state-check", action="store_true",
                    help="skip the /arm_interface/estop_state precondition (simulator tests only)")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("estop_midmotion_test", anonymous=True)

    samples = []  # (wall time, [q1..q5])

    def cb(msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            samples.append((time.time(), [d[j] for j in JOINTS]))

    rospy.Subscriber("/joint_states_in", JointState, cb, queue_size=200)
    pub = rospy.Publisher("/arm_interface/estop_trigger", Empty, queue_size=1)

    if not args.no_state_check:
        try:
            latched = rospy.wait_for_message("/arm_interface/estop_state", Bool, timeout=3.0).data
        except rospy.ROSException:
            print("No /arm_interface/estop_state -- is this the e-stop-enabled build? Nothing sent.")
            sys.exit(1)
        if latched:
            print("E-stop is currently latched -- reset it first (/arm_interface/estop_reset). Nothing sent.")
            sys.exit(1)

    rospy.sleep(1.0)  # let the subscriber fill and the publisher connect
    if not samples:
        print("No /joint_states_in. Nothing sent.")
        sys.exit(1)
    q0 = samples[-1][1]
    if max(abs(q) for q in q0[1:]) > 0.2:
        print("Arm is not near the rest pose (joints 2-5 max |q| = %.3f > 0.2); the vetted plan does not apply. Nothing sent." %
              max(abs(q) for q in q0[1:]))
        sys.exit(1)

    print("")
    print("  Reach pose A %s (about 6.4 s, largest joint move 0.68 rad), then e-stop %.1f s into it." % (POSE_A, args.trigger_after))
    print("  The arm will move, then be stopped and HELD (latched) -- supported / clear?")
    if input("Type 'yes' to run on the real arm: ").strip() != "yes":
        print("Aborted, nothing sent.")
        return

    result = {}

    def reach():
        srv = rospy.ServiceProxy("/reach_target_pose", ReachTargetPose)
        req = ReachTargetPoseRequest()
        req.target_pose.position.x, req.target_pose.position.y, req.target_pose.position.z = POSE_A
        o = req.target_pose.orientation
        o.x, o.y, o.z, o.w = QUAT
        req.cartesian_path = False
        try:
            result["res"] = srv(req)
        except rospy.ServiceException as e:
            result["err"] = str(e)

    t_start = time.time()
    th = threading.Thread(target=reach)
    th.start()
    time.sleep(args.trigger_after)
    t_trig = time.time()
    pub.publish(Empty())
    print("E-STOP TRIGGER published %.2f s after the reach started." % (t_trig - t_start))
    time.sleep(3.2)
    th.join(timeout=15.0)

    def q_at(t):
        best = min(samples, key=lambda s: abs(s[0] - t))
        return best[1]

    def speed(t0, t1):
        seg = [s for s in samples if t0 <= s[0] <= t1]
        if len(seg) < 2:
            return float("nan")
        return max(abs(seg[-1][1][i] - seg[0][1][i]) for i in range(5)) / (seg[-1][0] - seg[0][0])

    q_trig = q_at(t_trig)
    pre = speed(t_trig - 0.5, t_trig)
    print("")
    print("  max joint speed in the 0.5 s BEFORE the trigger : %.3f rad/s" % pre)
    for dt in (0.1, 0.3, 1.0):
        q = q_at(t_trig + dt)
        print("  +%.1f s: max joint travel since trigger %.4f rad, max speed over the following 0.2 s %.3f rad/s" % (
            dt, max(abs(q[i] - q_trig[i]) for i in range(5)), speed(t_trig + dt, t_trig + dt + 0.2)))
    qa, qb = q_at(t_trig + 0.5), q_at(t_trig + 3.0)
    drift = max(abs(qb[i] - qa[i]) for i in range(5))
    print("  drift between +0.5 s and +3.0 s: %.4f rad  (a held arm should stay under ~0.01)" % drift)
    if "res" in result:
        print("  reach_target_pose returned: success=%s  %s" % (result["res"].success, result["res"].message[:110]))
    elif "err" in result:
        print("  reach_target_pose call error:", result["err"])
    stopped = speed(t_trig + 0.5, t_trig + 0.9) < 0.02
    held = drift < 0.01
    try:
        state = rospy.wait_for_message("/arm_interface/estop_state", Bool, timeout=1.0).data
    except rospy.ROSException:
        state = None
    print("  /arm_interface/estop_state: %s" % state)
    print("  VERDICT: %s (stopped by +0.5 s: %s, no creep afterwards: %s)" % (
        "PASS - arm stopped and held" if (stopped and held and state) else "FAIL", stopped, held))
    if state:
        print("  The e-stop is LATCHED and the arm is holding. Release with: rosservice call /arm_interface/estop_reset")


if __name__ == "__main__":
    main()
