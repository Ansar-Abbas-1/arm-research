#!/usr/bin/env python3
"""Integration test: with the null-space secondary task enabled
(~nullspace_gain > 0), holding the deadman with a centered stick (zero
commanded task velocity) should still drive the real arm's link_4 toward
vertical -- confirmed against the real TF tree (robot_state_publisher's
own FK, an independent code path from arm_teleop's), not just the
pure-function gtest cross-check in test_resolved_rate.cpp.
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

    def _link4_vertical_alignment(self):
        """Dot of link_4's own local +Z axis with base_link's (world) +Z,
        read directly from the real TF tree. For a unit quaternion
        (x,y,z,w), the Z-component of the rotation matrix's third column
        -- i.e. exactly this dot product -- is 1 - 2(x^2+y^2)."""
        t = self.tf_buffer.lookup_transform("base_link", "link_4", rospy.Time(0), rospy.Duration(2.0))
        q = t.transform.rotation
        return 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

    def test_nullspace_bias_improves_link4_verticality_when_idle(self):
        # Move away from the startup singularity first (arm_trajectory
        # Finding 1: {0,0,0,0,0} is a genuine kinematic singularity, and
        # not necessarily a representative "before" configuration anyway).
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.5)
        rospy.sleep(0.6)
        before = self._link4_vertical_alignment()

        # Hold the deadman with the stick centered: zero commanded task
        # velocity, so any motion at all has to come from the null-space
        # bias, not the primary task.
        self._hold_joy(make_joy(axis_x=0.0, deadman=True), duration=3.0)
        rospy.sleep(0.3)
        after = self._link4_vertical_alignment()

        self.assertGreater(
            after, before - 1e-3,
            f"null-space bias should never make link_4 LESS vertical: "
            f"before={before:.4f} after={after:.4f}",
        )
        if before < 0.95:
            self.assertGreater(
                after, before + 0.01,
                f"expected a measurable improvement toward vertical given a non-trivial starting "
                f"misalignment: before={before:.4f} after={after:.4f}",
            )


if __name__ == "__main__":
    rostest.rosrun(PKG, "teleop_nullspace_test", TeleopNullspaceTest)
