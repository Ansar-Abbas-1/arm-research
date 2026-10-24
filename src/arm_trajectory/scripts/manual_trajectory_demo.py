#!/usr/bin/env python3
"""Manual hand-off demo for M3: calls /generate_trajectory, sends the
result to the follow_joint_trajectory action, and prints progress.

Requires trajectory_service_node and trajectory_executor_node running
(plus the sim, if you want to see it move) -- this script only drives
them, it doesn't start them.

Usage:
  rosrun arm_trajectory manual_trajectory_demo.py
      joint-space move to a default demo configuration

  rosrun arm_trajectory manual_trajectory_demo.py --goal 0.5 0.8 -0.4 0.6 -0.6
      joint-space move to the given 5 joint values

  rosrun arm_trajectory manual_trajectory_demo.py --cartesian x y z qx qy qz qw
      Cartesian-path move to the given end_effector pose (7 numbers:
      position xyz, orientation quaternion xyzw)
"""
import argparse
import sys

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose

DEFAULT_JOINT_GOAL = [0.5, 0.8, -0.4, 0.6, -0.6]

_STATUS_NAMES = {
    v: k for k, v in vars(GoalStatus).items()
    if isinstance(v, int) and k.isupper()
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", nargs=5, type=float, metavar=("J1", "J2", "J3", "J4", "J5"),
                         help="joint-space goal (5 values, radians)")
    parser.add_argument("--cartesian", nargs=7, type=float,
                         metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                         help="Cartesian-path goal: end_effector position + orientation quaternion")
    parser.add_argument("--sample-rate", type=float, default=20.0)
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    rospy.init_node("manual_trajectory_demo")

    rospy.loginfo("Waiting for /generate_trajectory ...")
    rospy.wait_for_service("/generate_trajectory", timeout=30.0)
    generate = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)

    req = GenerateTrajectoryRequest()
    req.sample_rate = args.sample_rate
    if args.cartesian:
        x, y, z, qx, qy, qz, qw = args.cartesian
        req.goal_pose = Pose()
        req.goal_pose.position.x, req.goal_pose.position.y, req.goal_pose.position.z = x, y, z
        req.goal_pose.orientation.x = qx
        req.goal_pose.orientation.y = qy
        req.goal_pose.orientation.z = qz
        req.goal_pose.orientation.w = qw
        req.cartesian_path = True
        rospy.loginfo("Requesting Cartesian-path trajectory to (%.3f, %.3f, %.3f)", x, y, z)
    else:
        req.goal_joint_positions = args.goal if args.goal else DEFAULT_JOINT_GOAL
        rospy.loginfo("Requesting joint-space trajectory to %s", req.goal_joint_positions)

    res = generate(req)
    if not res.success:
        rospy.logerr("generate_trajectory failed: %s", res.message)
        sys.exit(1)

    n = len(res.trajectory.points)
    duration = res.trajectory.points[-1].time_from_start.to_sec() if n else 0.0
    rospy.loginfo("Generated %d points, duration %.2fs", n, duration)
    if req.cartesian_path:
        rospy.loginfo("Peak path-tracking deviation: %.5f m / %.5f rad",
                       res.peak_position_deviation, res.peak_orientation_deviation)

    rospy.loginfo("Waiting for follow_joint_trajectory action server ...")
    client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
    if not client.wait_for_server(timeout=rospy.Duration(30.0)):
        rospy.logerr("follow_joint_trajectory action server not available")
        sys.exit(1)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = res.trajectory
    goal.trajectory.header.stamp = rospy.Time.now()
    rospy.loginfo("Sending goal, executing ...")
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration(duration + 10.0))
    state = client.get_state()
    rospy.loginfo("Final action state: %s", _STATUS_NAMES.get(state, state))


if __name__ == "__main__":
    main()
