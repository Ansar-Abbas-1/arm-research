#!/usr/bin/env python3
"""Integration test: /joy -> teleop_node -> /generate_trajectory +
follow_joint_trajectory, against the real simulator + fk_node.

Each test is self-contained (records its own before/after state) rather
than relying on unittest's run order, which is alphabetical, not
declaration order -- a real footgun for a stateful integration scenario
like this one.

Confirms:
  1. Sustained stick input moves the tracked end-effector position, and
     reversing it promptly reverses that motion -- i.e. teleop_node's
     per-tick goals are genuinely preempting each other, not queuing/
     stalling behind an in-flight trajectory.
  2. Releasing the deadman button stops motion even with a deflected stick.
  3. The gripper (joint_6) updates independently of Cartesian jogging.
"""
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState

PKG = "arm_teleop"

# Must match the params set in teleop_preemption.test.
DEADMAN_BUTTON = 4
AXIS_X = 1
GRIPPER_OPEN_BUTTON = 1
NUM_AXES = 8
NUM_BUTTONS = 8


def make_joy(axis_x=0.0, deadman=False, gripper_open=False):
    joy = Joy()
    joy.axes = [0.0] * NUM_AXES
    joy.buttons = [0] * NUM_BUTTONS
    joy.axes[AXIS_X] = axis_x
    joy.buttons[DEADMAN_BUTTON] = 1 if deadman else 0
    joy.buttons[GRIPPER_OPEN_BUTTON] = 1 if gripper_open else 0
    return joy


class TeleopIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("teleop_preemption_test")
        cls.joy_pub = rospy.Publisher("/joy", Joy, queue_size=1)

        cls.latest_pose = None
        cls.pose_sub = rospy.Subscriber("/computed_end_effector_pose", PoseStamped, cls._pose_cb)
        cls.latest_joint_state = None
        cls.joint_sub = rospy.Subscriber("/joint_states_in", JointState, cls._joint_cb)

        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while cls.latest_pose is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
        if cls.latest_pose is None:
            raise AssertionError("never received /computed_end_effector_pose")
        rospy.sleep(1.0)  # let teleop_node's own pose subscriber seed its target too

    @classmethod
    def _pose_cb(cls, msg):
        cls.latest_pose = msg

    @classmethod
    def _joint_cb(cls, msg):
        cls.latest_joint_state = msg

    def _hold_joy(self, joy_msg, duration, rate_hz=20.0):
        rate = rospy.Rate(rate_hz)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end:
            joy_msg.header.stamp = rospy.Time.now()
            self.joy_pub.publish(joy_msg)
            rate.sleep()

    def _gripper_position(self):
        idx = list(self.latest_joint_state.name).index("joint_6")
        return self.latest_joint_state.position[idx]

    def test_sustained_and_reversed_stick_moves_tracked_position(self):
        # Small move sizes deliberately (linear_gain=0.1 m/s, ~1.5s hold ->
        # ~0.15m commanded) -- large enough to clear scheduling-jitter noise
        # and the occasional dropped tick (restart-disabled single-attempt
        # IK can occasionally miss tolerance depending on local Jacobian
        # conditioning -- teleop_node just drops that tick, see
        # docs/teleop.md), small enough to stay well inside the workspace
        # regardless of where in it the arm happens to start.
        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.5)
        rospy.sleep(0.5)  # let the last commanded trajectory settle
        mid_x = self.latest_pose.pose.position.x
        self.assertGreater(
            mid_x - start_x, 0.015,
            f"expected meaningful +X motion, start={start_x:.4f} mid={mid_x:.4f}",
        )

        self._hold_joy(make_joy(axis_x=-1.0, deadman=True), duration=1.5)
        rospy.sleep(0.5)
        end_x = self.latest_pose.pose.position.x
        self.assertLess(
            end_x, mid_x - 0.015,
            f"expected reversal to move back in -X, mid={mid_x:.4f} end={end_x:.4f} "
            "-- if per-tick goals were queuing instead of preempting, this would lag or stall",
        )

    def test_deadman_released_stops_motion(self):
        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, deadman=False), duration=1.0)
        rospy.sleep(0.5)
        end_x = self.latest_pose.pose.position.x
        self.assertAlmostEqual(
            end_x, start_x, delta=0.01,
            msg=f"deadman released but position moved anyway: start={start_x:.4f} end={end_x:.4f}",
        )

    def test_gripper_moves_independently_of_pose(self):
        start_x = self.latest_pose.pose.position.x
        start_gripper = self._gripper_position()

        self._hold_joy(make_joy(axis_x=0.0, deadman=True, gripper_open=True), duration=1.0)
        rospy.sleep(0.5)

        end_x = self.latest_pose.pose.position.x
        end_gripper = self._gripper_position()

        self.assertAlmostEqual(
            end_x, start_x, delta=0.01,
            msg="gripper-only input shouldn't move the end-effector position",
        )
        self.assertGreater(
            end_gripper, start_gripper + 0.05,
            f"gripper open button should have increased joint_6: "
            f"start={start_gripper:.4f} end={end_gripper:.4f}",
        )


if __name__ == "__main__":
    rostest.rosrun(PKG, "teleop_preemption_test", TeleopIntegrationTest)
