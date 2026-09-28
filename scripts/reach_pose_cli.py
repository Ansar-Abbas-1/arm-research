#!/usr/bin/env python3
"""Interactive CLI front end for /reach_target_pose (see
docs/trajectory_generation.md) -- type a pose, watch the arm go there,
see whether it actually did. The typing counterpart to
interactive_pose_marker_node's rviz drag-and-commit marker; both are
thin, they just feed the same service.

Requires reach_target_pose_node running (plus the sim, if you want to see
it move) -- this script only drives it, it doesn't start it.

Usage:
  rosrun arm_trajectory reach_pose_cli.py
      interactive REPL: type "x y z roll pitch yaw" (meters, degrees) at
      the prompt, repeatedly; 'q' or Ctrl-D to quit.

  rosrun arm_trajectory reach_pose_cli.py --pose 0.3 0.0 0.5 0 0 0
      one-shot: x y z roll pitch yaw (meters, degrees), then exit.

  rosrun arm_trajectory reach_pose_cli.py --pose 0.3 0.0 0.5 0 0 0 --cartesian
      same, but traces the interpolated Cartesian path there instead of
      a plain joint-space move (see ReachTargetPose.srv's cartesian_path).
"""
import argparse
import math
import sys

import rospy
from arm_trajectory.srv import ReachTargetPose, ReachTargetPoseRequest


def rpy_to_quaternion(roll, pitch, yaw):
    """roll/pitch/yaw in radians -> (x, y, z, w). Matches the URDF/tf2
    fixed-axis convention R = Rz(yaw)*Ry(pitch)*Rx(roll) used throughout
    this project (see arm_kinematics/docs/fk_reference.py's rpy())."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def build_request(x, y, z, roll_deg, pitch_deg, yaw_deg, cartesian_path):
    req = ReachTargetPoseRequest()
    req.target_pose.position.x = x
    req.target_pose.position.y = y
    req.target_pose.position.z = z
    qx, qy, qz, qw = rpy_to_quaternion(
        math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    )
    req.target_pose.orientation.x = qx
    req.target_pose.orientation.y = qy
    req.target_pose.orientation.z = qz
    req.target_pose.orientation.w = qw
    req.cartesian_path = cartesian_path
    return req


def send_and_report(reach, req):
    print(f"-> sending target: pos=({req.target_pose.position.x:.3f}, "
          f"{req.target_pose.position.y:.3f}, {req.target_pose.position.z:.3f}) "
          f"cartesian_path={req.cartesian_path}")
    try:
        res = reach(req)
    except rospy.ServiceException as e:
        print(f"   service call failed: {e}")
        return
    if res.success:
        print(f"   reached (position error {res.final_position_error:.5f} m, "
              f"orientation error {res.final_orientation_error:.5f} rad)")
    else:
        print(f"   FAILED: {res.message}")


def run_repl(reach, cartesian_path):
    print("Enter a pose as: x y z roll pitch yaw   (meters, degrees)")
    print("'q' or Ctrl-D to quit.\n")
    while True:
        try:
            line = input("pose> ").strip()
        except EOFError:
            print()
            return
        if line.lower() in ("q", "quit", "exit"):
            return
        if not line:
            continue
        parts = line.split()
        if len(parts) != 6:
            print("   expected 6 numbers: x y z roll pitch yaw")
            continue
        try:
            x, y, z, roll, pitch, yaw = (float(p) for p in parts)
        except ValueError:
            print("   couldn't parse those as 6 numbers")
            continue
        send_and_report(reach, build_request(x, y, z, roll, pitch, yaw, cartesian_path))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--pose", nargs=6, type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="one-shot mode: send this pose (meters, degrees) and exit",
    )
    parser.add_argument(
        "--cartesian", action="store_true",
        help="trace the interpolated Cartesian path instead of a plain joint-space move",
    )
    args = parser.parse_args()

    rospy.init_node("reach_pose_cli", anonymous=True)
    rospy.wait_for_service("/reach_target_pose", timeout=30.0)
    reach = rospy.ServiceProxy("/reach_target_pose", ReachTargetPose)

    if args.pose:
        send_and_report(reach, build_request(*args.pose, args.cartesian))
    else:
        run_repl(reach, args.cartesian)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)
