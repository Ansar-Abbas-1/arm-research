#!/usr/bin/env python3
"""Manual hand-off script for M4: publishes synthetic sensor_msgs/Joy
messages from keyboard presses, so teleop_node can be exercised without
real joystick hardware.

A terminal keypress is a discrete event, not a continuous "held" state, so
each key publishes a short burst (default 0.3s at 20Hz) of Joy messages
with the deadman button held and the corresponding axis/button set --
approximating "holding that direction briefly" -- followed by a brief
fully-neutral release, so mode-toggle/commit (edge-triggered: they fire
once per press, not once per tick held) reliably see a fresh rising edge
on the *next* keypress rather than staying latched from this one.

Axis/button indices here must match teleop_node's params (defaults below
match teleop_node's own defaults; pass --axis-pitch/etc, or just change
teleop_node's params consistently, if you've customized either).

Requires teleop_node (and the rest of the stack: sim, fk_node,
trajectory_service_node, trajectory_executor_node) running -- this script
only drives it.

Keys: w/s = +X/-X, a/d = +Y/-Y, r/f = +Z/-Z, t/y = +pitch/-pitch,
      o/c = gripper open/close, m = toggle DIRECT_JOG <-> SET_REFERENCE,
      space = commit reference (only meaningful in SET_REFERENCE mode),
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
    "w": ("axis", "x", 1.0),
    "s": ("axis", "x", -1.0),
    "a": ("axis", "y", 1.0),
    "d": ("axis", "y", -1.0),
    "r": ("axis", "z", 1.0),
    "f": ("axis", "z", -1.0),
    "t": ("axis", "pitch", 1.0),
    "y": ("axis", "pitch", -1.0),
    "o": ("button", "gripper_open", 1),
    "c": ("button", "gripper_close", 1),
    "m": ("button", "mode_toggle", 1),
    " ": ("button", "commit", 1),
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
    parser.add_argument("--axis-pitch", type=int, default=3)
    parser.add_argument("--gripper-open", type=int, default=1)
    parser.add_argument("--gripper-close", type=int, default=0)
    parser.add_argument("--mode-toggle-button", type=int, default=6)
    parser.add_argument("--commit-button", type=int, default=7)
    parser.add_argument("--burst-duration", type=float, default=0.3)
    parser.add_argument("--release-duration", type=float, default=0.15)
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    axis_index = {"x": args.axis_x, "y": args.axis_y, "z": args.axis_z, "pitch": args.axis_pitch}
    button_index = {
        "gripper_open": args.gripper_open,
        "gripper_close": args.gripper_close,
        "mode_toggle": args.mode_toggle_button,
        "commit": args.commit_button,
    }

    rospy.init_node("manual_teleop_check")
    pub = rospy.Publisher("/joy", Joy, queue_size=1)
    rospy.sleep(0.5)  # let the publisher connect before the first burst

    print(__doc__)
    print("Publishing to /joy. Press a bound key, or 'q' to quit.")

    def publish_for(duration, deadman_held, axis=None, button=None):
        rate = rospy.Rate(args.rate)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.axes = [0.0] * NUM_AXES
            joy.buttons = [0] * NUM_BUTTONS
            if deadman_held:
                joy.buttons[args.deadman] = 1
            if axis is not None:
                name, value = axis
                joy.axes[axis_index[name]] = value
            if button is not None:
                joy.buttons[button_index[button]] = 1
            pub.publish(joy)
            rate.sleep()

    while not rospy.is_shutdown():
        key = read_key()
        if key == "q" or key == "\x03":  # q or Ctrl-C
            break
        binding = KEY_BINDINGS.get(key)
        if binding is None:
            continue

        kind, target, value = binding
        if kind == "axis":
            publish_for(args.burst_duration, deadman_held=True, axis=(target, value))
        else:
            publish_for(args.burst_duration, deadman_held=True, button=target)
        publish_for(args.release_duration, deadman_held=False)  # reset for the next edge

    print("Exiting.")


if __name__ == "__main__":
    main()
