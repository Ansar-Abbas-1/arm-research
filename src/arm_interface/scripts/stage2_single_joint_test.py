#!/usr/bin/env python3
"""Stage 2 manual test (ROADMAP M5): commands ONE joint a small delta from
its current position, holding every other joint exactly where it already
is, then stops publishing. robot_interface treats the end of the stream
as "hold" (since 2026-10-02: torque stays on, last goal held); it no longer
drops torque after command_timeout. Release with
`rosservice call /arm_interface/set_torque "data: false"`.

SAFETY-CRITICAL DETAIL, not optional: robot_interface.cpp initializes its
internal commanded-position array to all-zero RADIANS at startup, not
"current position" -- any joint NOT named in the first /joint_states_out
message would snap toward 0 rad (not hold still) the instant torque
enables. This script avoids that by reading one live /joint_states_in
message first and explicitly echoing every joint's CURRENT position back
in the same command, except the one joint under test.

Per-joint hardware limits (config/ArmInterface.yaml's encoder_min/max)
are still enforced by robot_interface.cpp itself on every write
regardless of what this script sends, as defense in depth.

Usage (run by the user, not launched automatically by anything):
  rosrun arm_interface stage2_single_joint_test.py --joint 6 --delta 0.01
"""
import argparse
import sys

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--joint", type=int, required=True, choices=range(1, 7),
                         help="1-6, which joint to move")
    parser.add_argument("--delta", type=float, required=True,
                         help="radians to add to the joint's current position "
                              "(meters for joint 6, the gripper -- see gripper_conversion.h)")
    parser.add_argument("--publish-seconds", type=float, default=1.0,
                         help="how long to keep publishing the command; the watchdog "
                              "disables torque ~command_timeout seconds after the last one")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("stage2_single_joint_test", anonymous=True)

    # This script publishes straight to /joint_states_out, bypassing the
    # trajectory executor (the e-stop choke point for trajectories), so it
    # checks the state itself: fail-closed on engaged OR unknown.
    try:
        engaged = rospy.wait_for_message("/arm_interface/estop_state", Bool, timeout=3.0).data
    except rospy.ROSException:
        rospy.logfatal("No /arm_interface/estop_state -- backend not running or old build; refusing to move. Nothing published.")
        sys.exit(1)
    if engaged:
        rospy.logfatal("E-stop is engaged -- reset it first. Nothing published.")
        sys.exit(1)

    rospy.loginfo("Waiting for one /joint_states_in message...")
    current = rospy.wait_for_message("/joint_states_in", JointState, timeout=5.0)
    current_by_name = dict(zip(current.name, current.position))
    for name in JOINT_NAMES:
        if name not in current_by_name:
            rospy.logfatal("Missing %s in /joint_states_in -- aborting, nothing published.", name)
            sys.exit(1)

    target_name = "joint_%d" % args.joint
    targets = dict(current_by_name)
    targets[target_name] = current_by_name[target_name] + args.delta

    rospy.loginfo("Current %s: %.4f -> commanding %.4f (delta %+.4f). Every other joint held at its current value.",
                  target_name, current_by_name[target_name], targets[target_name], args.delta)
    rospy.loginfo("Full commanded set: %s", {n: round(targets[n], 4) for n in JOINT_NAMES})

    pub = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
    rospy.sleep(0.5)  # let the publisher connect before the first publish

    msg = JointState()
    msg.name = JOINT_NAMES
    msg.position = [targets[n] for n in JOINT_NAMES]

    rate = rospy.Rate(10)
    t0 = rospy.Time.now()
    while (rospy.Time.now() - t0).to_sec() < args.publish_seconds and not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Stopped publishing. robot_interface now HOLDS this pose with torque on (set_torque false to release).")


if __name__ == "__main__":
    main()
