#!/usr/bin/env python3
"""Stage 3 manual test (ROADMAP M5): first Cartesian reach on the real arm,
through the real M3/M4 pipeline (position+pitch IK with base-collision
avoidance in /generate_trajectory -> follow_joint_trajectory), with a human
confirmation gate before anything is sent to the motors.

Why a pivot: when robot_interface's watchdog drops torque the arm sags back
to the near-all-zeros upright pose, which is a kinematic singularity (M3
Finding 1) where Cartesian IK cannot plan (the generator trips its
runaway-duration guard). So from that pose the plan is ONE continuous
trajectory: joint-space move to PIVOT_Q (a non-singular config already
exercised on hardware), then the Cartesian reach from there. One trajectory
means torque never drops between the two phases.

Target = the pivot's end-effector pose (PIVOT_POSE, verified against the
simulator's FK) plus a small --offset, same orientation. Without the pivot
(--pivot never, or auto when the arm is already away from upright) the
target is the live EE pose from /computed_end_effector_pose (read-only
fk_node: rosrun arm_kinematics fk_node) plus --offset.

Refuses (nothing sent) when: generation fails (unreachable, or a goal or
interior sample would collide with the base box), any joint would move more
than --max-delta rad in a phase, the Cartesian path deviates by more than
--max-deviation m, or an end joint is outside the URDF limits. The
joint-space pivot phase is NOT collision-checked by the generator; the
straight joint line from upright to PIVOT_Q was checked offline against the
base box (0 of 401 samples colliding).

SAFETY: since the 2026-10-02 hold restructure, robot_interface keeps torque
ON and holds the final pose after the move (no ~0.5 s drop). Keep clear of it,
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
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]

PIVOT_Q = [0.15, 0.25, -0.25, 0.25, -0.25]
UPRIGHT_TOL = 0.2  # rad
# FK of PIVOT_Q on the simulator (fk_node), 2026-10-02.
PIVOT_POS = (0.13049, 0.01972, 0.55087)
PIVOT_QUAT = (-0.05113, 0.68049, 0.05477, 0.72892)  # x y z w


def pose_at(pos, quat, offset):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
    return p


def table(title, start, end, traj):
    peak = [max(abs(p.velocities[i]) for p in traj.points) for i in range(5)]
    print("  %s" % title)
    print("  joint      from       to     delta   peak|vel| (rad/s)")
    for i, j in enumerate(JOINTS):
        print("  %s  %7.3f  %7.3f  %+7.3f   %.3f" % (j, start[i], end[i], end[i] - start[i], peak[i]))
    print("  %.2f s, %d samples" % (traj.points[-1].time_from_start.to_sec(), len(traj.points)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offset", type=float, nargs=3, default=[0.0, 0.0, 0.03], metavar=("DX", "DY", "DZ"),
                    help="meters, added to the reference EE position (default: 3 cm up)")
    ap.add_argument("--pivot", choices=["auto", "always", "never"], default="auto",
                    help="route via PIVOT_Q first; auto = when joints 2-5 are all within UPRIGHT_TOL (0.2 rad) of zero")
    ap.add_argument("--max-delta", type=float, default=0.4, help="refuse if any joint moves more than this in a phase (rad)")
    ap.add_argument("--max-deviation", type=float, default=0.005,
                    help="refuse if the Cartesian path strays from the target path by more than this (m)")
    args = ap.parse_args(rospy.myargv()[1:])
    if max(abs(o) for o in args.offset) > 0.10:
        print("Offsets above 10 cm are refused for this first-reach script.")
        sys.exit(1)

    rospy.init_node("stage3_cartesian_reach", anonymous=True)

    js = rospy.wait_for_message("/joint_states_in", JointState, timeout=5.0)
    now = dict(zip(js.name, js.position))
    live = [now[j] for j in JOINTS]
    # The arm's sagged rest pose is near-all-zeros but joint_3 settles at about
    # -0.157 rad; the pivot's joints are all >= 0.25. 0.2 separates them.
    upright = max(abs(q) for q in live[1:]) < UPRIGHT_TOL
    use_pivot = args.pivot == "always" or (args.pivot == "auto" and upright)
    print("Live joints 2-5 max |q| = %.3f (upright threshold %.2f) -> %s" % (
        max(abs(q) for q in live[1:]), UPRIGHT_TOL, "routing via the pivot" if use_pivot else "planning directly from the live pose"))

    rospy.wait_for_service("/generate_trajectory", timeout=5.0)
    gen = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)

    def generate(start, goal_q=None, goal_pose=None):
        req = GenerateTrajectoryRequest()
        req.start_joint_positions = start
        if goal_q is not None:
            req.goal_joint_positions = goal_q
        else:
            req.goal_pose = goal_pose
            req.cartesian_path = True
        res = gen(req)
        if not res.success:
            rospy.logfatal("Trajectory generation failed: %s -- nothing sent.", res.message)
            sys.exit(1)
        return res

    phases = []  # (title, start, end, trajectory)
    if use_pivot:
        a = generate(live, goal_q=PIVOT_Q)
        target = pose_at(PIVOT_POS, PIVOT_QUAT, args.offset)
        b = generate(PIVOT_Q, goal_pose=target)
        phases.append(("Phase A: live pose -> pivot (joint space)", live, PIVOT_Q, a.trajectory))
        phases.append(("Phase B: pivot -> Cartesian target", PIVOT_Q, list(b.trajectory.points[-1].positions), b.trajectory))
        deviation = b.peak_position_deviation
        print("\n  Arm is %s: routing via the pivot (one continuous trajectory)." %
              ("upright (singular)" if upright else "not upright, but --pivot always"))
    else:
        try:
            cur = rospy.wait_for_message("/computed_end_effector_pose", PoseStamped, timeout=5.0).pose
        except rospy.ROSException:
            rospy.logfatal("No /computed_end_effector_pose -- start the read-only 'rosrun arm_kinematics fk_node'. Nothing sent.")
            sys.exit(1)
        target = pose_at((cur.position.x, cur.position.y, cur.position.z),
                         (cur.orientation.x, cur.orientation.y, cur.orientation.z, cur.orientation.w), args.offset)
        b = generate(live, goal_pose=target)
        phases.append(("Cartesian reach from the live pose", live, list(b.trajectory.points[-1].positions), b.trajectory))
        deviation = b.peak_position_deviation

    urdf = ET.fromstring(rospy.get_param("robot_description"))
    limits = {jt.get("name"): (float(jt.find("limit").get("lower")), float(jt.find("limit").get("upper")))
              for jt in urdf.iter("joint") if jt.find("limit") is not None}

    print("")
    for title, s, e, t in phases:
        table(title, s, e, t)
        for j, q in zip(JOINTS, e):
            lo, hi = limits[j]
            if not (lo - 1e-6 <= q <= hi + 1e-6):
                print("%s end value %.3f outside URDF limits [%.3f, %.3f] -- nothing sent." % (j, q, lo, hi))
                sys.exit(1)
        if max(abs(x - y) for x, y in zip(e, s)) > args.max_delta:
            print("A joint moves more than --max-delta %.3f in this phase -- nothing sent." % args.max_delta)
            sys.exit(1)
    print("  EE target (%.3f, %.3f, %.3f), orientation unchanged; Cartesian path deviation %.4f m; joint_6 holds." % (
        target.position.x, target.position.y, target.position.z, deviation))
    if deviation > args.max_deviation:
        print("Path deviation exceeds --max-deviation %.4f -- nothing sent." % args.max_deviation)
        sys.exit(1)

    # One continuous trajectory: later phases shifted in time, duplicate
    # junction point (identical to the previous phase's last) dropped.
    merged = phases[0][3]
    for _, _, _, nxt in phases[1:]:
        offset_t = merged.points[-1].time_from_start
        for p in nxt.points[1:]:
            p.time_from_start = p.time_from_start + offset_t
            merged.points.append(p)
    duration = merged.points[-1].time_from_start.to_sec()
    print("  Total %.2f s, %d samples. The arm then HOLDS its final pose with torque on -- clear?" % (
        duration, len(merged.points)))
    if input("Type 'yes' to execute on the real arm: ").strip() != "yes":
        print("Aborted, nothing sent.")
        return

    client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
    client.wait_for_server(rospy.Duration(5.0))
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = merged
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
    try:
        print("EE error vs target, at end of move:", ee_error())
        rospy.sleep(0.3)
        print("EE error vs target, 0.3 s later:   ", ee_error())
    except rospy.ROSException:
        print("(fk_node not running, EE error not reported)")


if __name__ == "__main__":
    main()
