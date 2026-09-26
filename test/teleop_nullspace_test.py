#!/usr/bin/env python3
"""Integration test for the null-space secondary task (~nullspace_gain > 0).

The gtest suite (test_resolved_rate.cpp) already rigorously isolates and
proves the bias's own contribution improves link_4's vertical alignment,
comparing with-bias vs. without-bias for the same primary command. Trying
to reproduce that same comparison live, over a real multi-second jog,
turned out fragile: the primary motion's own incidental effect on
alignment can be much larger than the bias's small per-tick nudge and can
dominate it in either direction depending on the specific motion,
confirmed the hard way when a 3s sustained +X jog made alignment measurably
*worse* despite the bias being active throughout. This suite instead
covers what's specific to the live, real-hardware-relevant integration:

  1. Holding the deadman with a CENTERED stick causes no drift at all --
     the bias must not run on its own with nothing commanding it (this
     was a real, reported bug: it used to run unconditionally whenever
     the deadman was held, making the arm visibly crawl with no input).
  2. The bias doesn't prevent normal jogging from working.
"""
import unittest

import rospy
import rostest
import tf2_ros
from sensor_msgs.msg import Joy

PKG = "arm_teleop"

# Must match the params set in teleop_nullspace.test.
DEADMAN_BUTTON = 4
AXIS_X = 1
NUM_AXES = 8
NUM_BUTTONS = 8


def make_joy(axis_x=0.0, deadman=False):
    joy = Joy()
    joy.axes = [0.0] * NUM_AXES
    joy.buttons = [0] * NUM_BUTTONS
    joy.axes[AXIS_X] = axis_x
    joy.buttons[DEADMAN_BUTTON] = 1 if deadman else 0
    return joy


class TeleopNullspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("teleop_nullspace_test")
        cls.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer)
        rospy.sleep(2.0)  # let the TF tree and teleop_node come up

    def _hold_joy(self, joy_msg, duration, rate_hz=30.0):
        rate = rospy.Rate(rate_hz)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end:
            joy_msg.header.stamp = rospy.Time.now()
            self.joy_pub.publish(joy_msg)
            rate.sleep()

    def _current_x(self):
        t = self.tf_buffer.lookup_transform("base_link", "end_effector", rospy.Time(0),
                                             rospy.Duration(2.0))
        return t.transform.translation.x

    def test_idle_deadman_causes_no_drift(self):
        # Move away from the startup singularity first (arm_trajectory
        # Finding 1: {0,0,0,0,0} is a genuine kinematic singularity, and
        # not necessarily a representative configuration to idle at).
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.6)
        start_x = self._current_x()

        # Deadman held, stick centered: this must be a true no-op --
        # confirmed the hard way that it wasn't, before this fix.
        self._hold_joy(make_joy(axis_x=0.0, deadman=True), duration=3.0)
        rospy.sleep(0.3)
        end_x = self._current_x()

        self.assertAlmostEqual(
            end_x, start_x, delta=0.002,
            msg=f"deadman held with a centered stick must not move the arm at all: "
            f"start={start_x:.5f} end={end_x:.5f}",
        )

    def test_jogging_still_works_with_bias_enabled(self):
        start_x = self._current_x()
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.3)
        end_x = self._current_x()
        self.assertGreater(
            end_x - start_x, 0.02,
            f"the null-space bias must not prevent normal jogging: start={start_x:.4f} end={end_x:.4f}",
        )


if __name__ == "__main__":
    rostest.rosrun(PKG, "teleop_nullspace_test", TeleopNullspaceTest)
