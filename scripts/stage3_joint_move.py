#!/usr/bin/env python3
"""Stage 3 manual test (ROADMAP M5): execute ONE joint-space move through the
real M3 pipeline (/generate_trajectory -> follow_joint_trajectory), with a
human confirmation gate before anything is sent to the motors.

Flow: read the live pose from /joint_states_in, ask trajectory_service_node
for a joint-space trajectory to --goal (so the trajectory starts exactly
where the arm is, and its timing already reflects ~velocity_scale), print
what is about to happen, and execute only if you type 'yes'.

Refuses (nothing generated or sent) when: any goal joint is outside the
URDF limits (the joint-space service does NOT check this itself;
robot_interface would silently clip to encoder bounds instead), or any
joint would move more than --max-delta radians.

SAFETY: robot_interface's command-timeout watchdog disables torque ~0.5 s
after the last command, so the arm LOSES HOLDING TORQUE right after the
move ends. Support it, or stand clear of where it would fall.

Usage (run by the user):
  rosrun arm_interface stage3_joint_move.py --goal 0.15 0.25 -0.25 0.25 -0.25
"""
import argparse
import sys
import xml.etree.ElementTree as ET

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--goal", type=float, nargs=5, required=True, metavar="Q", help="joint_1..joint_5 targets, radians")
    ap.add_argument("--max-delta", type=float, default=0.6, help="refuse if any joint moves more than this (rad)")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("stage3_joint_move", anonymous=True)

    state = rospy.wait_for_message("/joint_states_in", JointState, timeout=5.0)
    now = dict(zip(state.name, state.position))
    start = [now[j] for j in JOINTS]

    urdf = ET.fromstring(rospy.get_param("robot_description"))
    limits = {jt.get("name"): (float(jt.find("limit").get("lower")), float(jt.find("limit").get("upper")))
              for jt in urdf.iter("joint") if jt.find("limit") is not None}
    for j, q in zip(JOINTS, args.goal):
        lo, hi = limits[j]
        if not (lo <= q <= hi):
            rospy.logfatal("%s goal %.3f outside URDF limits [%.3f, %.3f] -- nothing sent.", j, q, lo, hi)
            sys.exit(1)
    deltas = [g - s for g, s in zip(args.goal, start)]
    if max(abs(d) for d in deltas) > args.max_delta:
        rospy.logfatal("Largest joint move %.3f rad exceeds --max-delta %.3f -- nothing sent.",
                       max(abs(d) for d in deltas), args.max_delta)
        sys.exit(1)

    rospy.wait_for_service("/generate_trajectory", timeout=5.0)
    gen = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
    req = GenerateTrajectoryRequest()
    req.start_joint_positions = start
    req.goal_joint_positions = list(args.goal)
    res = gen(req)
    if not res.success:
        rospy.logfatal("Trajectory generation failed: %s", res.message)
        sys.exit(1)

    traj = res.trajectory
    duration = traj.points[-1].time_from_start.to_sec()
    peak = [max(abs(p.velocities[i]) for p in traj.points) for i in range(5)]
    print("")
    print("  joint      now      goal     delta   peak|vel| (rad/s)")
    for i, j in enumerate(JOINTS):
        print("  %s  %7.3f  %7.3f  %+7.3f   %.3f" % (j, start[i], args.goal[i], deltas[i], peak[i]))
    print("  duration %.2f s, %d samples. joint_6 (gripper) will hold its current position." % (duration, len(traj.points)))
    print("  Torque drops ~0.5 s after the move ends -- arm supported / clear?")
    if input("Type 'yes' to execute on the real arm: ").strip() != "yes":
        print("Aborted, nothing sent.")
        return

    client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server(rospy.Duration(5.0))
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj
    client.send_goal(goal)
    try:
        client.wait_for_result(rospy.Duration(duration + 5.0))
    except KeyboardInterrupt:
        pass
    if client.get_state() not in (3,):  # actionlib SUCCEEDED
        rospy.logwarn("Not finished/succeeded -- cancelling the goal.")
        client.cancel_all_goals()
    # Error right at the end of the action, then again 0.3 s later (still
    # inside the 0.5 s watchdog window): if the second is smaller, the arm
    # was still settling; if equal, it is a steady-state offset (gravity
    # load / motor compliance in position mode).
    def error_now():
        msg = rospy.wait_for_message("/joint_states_in", JointState, timeout=5.0)
        cur = dict(zip(msg.name, msg.position))
        return ["%+.3f" % (cur[j] - g) for j, g in zip(JOINTS, args.goal)]
    print("final error per joint (rad), at end of move:", error_now())
    rospy.sleep(0.3)
    print("final error per joint (rad), 0.3 s later:   ", error_now())


if __name__ == "__main__":
    main()
