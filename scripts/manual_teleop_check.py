#!/usr/bin/env python3
"""Manual hand-off script for M4: publishes synthetic sensor_msgs/Joy
messages from keyboard presses, so teleop_node can be exercised without
real joystick hardware.

A terminal keypress is a discrete event, not a continuous "held" state, so
each key publishes a short burst (default 0.3s at 20Hz) of Joy messages
with the deadman button held and the corresponding axis/button set --
approximating "holding that direction briefly". Axis/button indices here
must match teleop_node's params (defaults below match teleop_node's own
defaults; pass --deadman/--axis-x/etc, or just change teleop_node's params
consistently, if you've customized either).

Requires teleop_node (and the rest of the stack: sim, fk_node,
trajectory_service_node, trajectory_executor_node) running -- this script
only drives it.

Keys: w/s = +X/-X, a/d = +Y/-Y, r/f = +Z/-Z, o/c = gripper open/close,
      q = quit
"""
import argparse
import sys
import termios
import tty

import rospy
from sensor_msgs.msg import Joy

NUM_AXES = 8
NUM_BUTTONS = 8

KEY_BINDINGS = {
    "w": ("axis", 0, 1.0),
    "s": ("axis", 0, -1.0),
    "a": ("axis", 1, 1.0),
    "d": ("axis", 1, -1.0),
    "r": ("axis", 2, 1.0),
    "f": ("axis", 2, -1.0),
    "o": ("button", "open", 1),
    "c": ("button", "close", 1),
}


def read_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--deadman", type=int, default=4)
    parser.add_argument("--axis-x", type=int, default=1)
    parser.add_argument("--axis-y", type=int, default=0)
    parser.add_argument("--axis-z", type=int, default=4)
    parser.add_argument("--gripper-open", type=int, default=1)
    parser.add_argument("--gripper-close", type=int, default=0)
    parser.add_argument("--burst-duration", type=float, default=0.3)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    axis_index = {0: args.axis_x, 1: args.axis_y, 2: args.axis_z}
    button_index = {"open": args.gripper_open, "close": args.gripper_close}

    rospy.init_node("manual_teleop_check")
    pub = rospy.Publisher("/joy", Joy, queue_size=1)
    rospy.sleep(0.5)  # let the publisher connect before the first burst

    print(__doc__)
    print("Publishing to /joy. Press a bound key, or 'q' to quit.")

    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        key = read_key()
        if key == "q" or key == "\x03":  # q or Ctrl-C
            break
        binding = KEY_BINDINGS.get(key)
        if binding is None:
            continue

        kind, target, value = binding
        end = rospy.Time.now() + rospy.Duration(args.burst_duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.axes = [0.0] * NUM_AXES
            joy.buttons = [0] * NUM_BUTTONS
            joy.buttons[args.deadman] = 1
            if kind == "axis":
                joy.axes[axis_index[target]] = value
            else:
                joy.buttons[button_index[target]] = value
            pub.publish(joy)
            rate.sleep()

    print("Exiting.")


if __name__ == "__main__":
    main()
