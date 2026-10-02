#!/usr/bin/env python3
"""Stage 3 manual test (ROADMAP M5): first Cartesian reach on the real arm,
through the real M3/M4 pipeline (position+pitch IK with null-space bias and
base-collision avoidance in /generate_trajectory -> follow_joint_trajectory),
with a human confirmation gate before anything is sent to the motors.

Target = the arm's CURRENT end-effector pose plus a small --offset (meters)
with the same orientation, so the move is small and the start is wherever
the arm really is. The live pose comes from /computed_end_effector_pose,
published by the read-only fk_node, which must be running:
  rosrun arm_kinematics fk_node

Flow: read live joints and EE pose, request a Cartesian-path trajectory to
the target, print the plan, execute only if you type 'yes'.

Refuses (nothing sent) when: generation fails (unreachable, or a goal or
interior sample would collide with the base box), any joint would move more
than --max-delta rad, the planned path deviates from the straight line by
more than --max-deviation m, or an end joint is outside the URDF limits.

SAFETY: robot_interface's watchdog disables torque ~0.5 s after the last
command, so the arm LOSES HOLDING TORQUE right after the move. Support it,
or stand clear of where it would fall.

Usage (run by the user):
  rosrun arm_interface stage3_cartesian_reach.py --offset 0 0 0.03
"""
import argparse
import sys
import xml.etree.ElementTree as ET

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offset", type=float, nargs=3, default=[0.0, 0.0, 0.03], metavar=("DX", "DY", "DZ"),
                    help="meters, added to the current EE position (default: 3 cm up)")
    ap.add_argument("--max-delta", type=float, default=0.4, help="refuse if any joint moves more than this (rad)")
    ap.add_argument("--max-deviation", type=float, default=0.005,
                    help="refuse if the planned path strays from the target path by more than this (m)")
    args = ap.parse_args(rospy.myargv()[1:])
    if max(abs(o) for o in args.offset) > 0.10:
        print("Offsets above 10 cm are refused for this first-reach script.")
        sys.exit(1)

    rospy.init_node("stage3_cartesian_reach", anonymous=True)

    js = rospy.wait_for_message("/joint_states_in", JointState, timeout=5.0)
    now = dict(zip(js.name, js.position))
    start = [now[j] for j in JOINTS]
    try:
        cur = rospy.wait_for_message("/computed_end_effector_pose", PoseStamped, timeout=5.0).pose
    except rospy.ROSException:
        rospy.logfatal("No /computed_end_effector_pose -- start the read-only 'rosrun arm_kinematics fk_node'. Nothing sent.")
        sys.exit(1)

    target = type(cur)()
    target.orientation = cur.orientation
    target.position.x = cur.position.x + args.offset[0]
    target.position.y = cur.position.y + args.offset[1]
    target.position.z = cur.position.z + args.offset[2]

    rospy.wait_for_service("/generate_trajectory", timeout=5.0)
    gen = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
    req = GenerateTrajectoryRequest()
    req.start_joint_positions = start
    req.goal_pose = target
    req.cartesian_path = True
    res = gen(req)
    if not res.success:
        rospy.logfatal("Trajectory generation failed: %s -- nothing sent.", res.message)
        sys.exit(1)

    traj = res.trajectory
    end = list(traj.points[-1].positions)
    deltas = [e - s for e, s in zip(end, start)]
    duration = traj.points[-1].time_from_start.to_sec()
    peak = [max(abs(p.velocities[i]) for p in traj.points) for i in range(5)]

    urdf = ET.fromstring(rospy.get_param("robot_description"))
    limits = {jt.get("name"): (float(jt.find("limit").get("lower")), float(jt.find("limit").get("upper")))
              for jt in urdf.iter("joint") if jt.find("limit") is not None}
    for j, q in zip(JOINTS, end):
        lo, hi = limits[j]
        if not (lo - 1e-6 <= q <= hi + 1e-6):
            rospy.logfatal("%s end value %.3f outside URDF limits [%.3f, %.3f] -- nothing sent.", j, q, lo, hi)
            sys.exit(1)

    print("")
    print("  EE now    (%.3f, %.3f, %.3f)" % (cur.position.x, cur.position.y, cur.position.z))
    print("  EE target (%.3f, %.3f, %.3f)  offset %s m, orientation unchanged" % (
        target.position.x, target.position.y, target.position.z, args.offset))
    print("  joint      now      end     delta   peak|vel| (rad/s)")
    for i, j in enumerate(JOINTS):
        print("  %s  %7.3f  %7.3f  %+7.3f   %.3f" % (j, start[i], end[i], deltas[i], peak[i]))
    print("  duration %.2f s, %d samples; path deviation %.4f m; joint_6 holds its position." % (
        duration, len(traj.points), res.peak_position_deviation))

    if max(abs(d) for d in deltas) > args.max_delta:
        print("Largest joint move %.3f rad exceeds --max-delta %.3f -- nothing sent." % (max(abs(d) for d in deltas), args.max_delta))
        sys.exit(1)
    if res.peak_position_deviation > args.max_deviation:
        print("Path deviation %.4f m exceeds --max-deviation %.4f -- nothing sent." % (res.peak_position_deviation, args.max_deviation))
        sys.exit(1)

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
    if client.get_state() != 3:  # actionlib SUCCEEDED
        rospy.logwarn("Not finished/succeeded -- cancelling the goal.")
        client.cancel_all_goals()

    def ee_error():
        p = rospy.wait_for_message("/computed_end_effector_pose", PoseStamped, timeout=5.0).pose.position
        return "(%+.4f, %+.4f, %+.4f) m" % (p.x - target.position.x, p.y - target.position.y, p.z - target.position.z)
    print("EE error vs target, at end of move:", ee_error())
    rospy.sleep(0.3)
    print("EE error vs target, 0.3 s later:   ", ee_error())


if __name__ == "__main__":
    main()
