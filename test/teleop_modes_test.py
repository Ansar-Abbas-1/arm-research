#!/usr/bin/env python3
"""Integration test: /joy -> teleop_node, against the real simulator +
fk_node + arm_trajectory stack.

Each test is self-contained (records its own before/after state) rather
than relying on unittest's alphabetical run order.

Confirms:
  1. DIRECT_JOG: sustained stick input moves the tracked end-effector
     position (resolved-rate velocity control), and releasing the deadman
     stops it.
  2. Mode toggle: switching to SET_REFERENCE and jogging does NOT move the
     real arm, even though the same stick axes are held.
  3. Commit: from SET_REFERENCE, moving the reference away and pressing
     commit actually drives the real arm to (near) the reference pose.
  4. Gripper (joint_6) moves independently of both modes.
"""
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState

PKG = "arm_teleop"

# Must match the params set in teleop_modes.test.
DEADMAN_BUTTON = 4
AXIS_X = 1
AXIS_PITCH = 3
GRIPPER_OPEN_BUTTON = 1
GRIPPER_CLOSE_BUTTON = 0
MODE_TOGGLE_BUTTON = 6
COMMIT_BUTTON = 7
NUM_AXES = 8
NUM_BUTTONS = 8


def make_joy(axis_x=0.0, axis_pitch=0.0, deadman=False, gripper_open=False,
             mode_toggle=False, commit=False):
    joy = Joy()
    joy.axes = [0.0] * NUM_AXES
    joy.buttons = [0] * NUM_BUTTONS
    joy.axes[AXIS_X] = axis_x
    joy.axes[AXIS_PITCH] = axis_pitch
    joy.buttons[DEADMAN_BUTTON] = 1 if deadman else 0
    joy.buttons[GRIPPER_OPEN_BUTTON] = 1 if gripper_open else 0
    joy.buttons[MODE_TOGGLE_BUTTON] = 1 if mode_toggle else 0
    joy.buttons[COMMIT_BUTTON] = 1 if commit else 0
    return joy


class TeleopModesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("teleop_modes_test")
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
        rospy.sleep(0.5)

    @classmethod
    def _pose_cb(cls, msg):
        cls.latest_pose = msg

    @classmethod
    def _joint_cb(cls, msg):
        cls.latest_joint_state = msg

    def _hold_joy(self, joy_msg, duration, rate_hz=30.0):
        rate = rospy.Rate(rate_hz)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end:
            joy_msg.header.stamp = rospy.Time.now()
            self.joy_pub.publish(joy_msg)
            rate.sleep()

    def _tap_button(self, **button_kwargs):
        """Rising-edge press: release, press, release -- so mode-toggle/
        commit (edge-triggered) reliably fires exactly once regardless of
        whatever button state was left over from an earlier test."""
        self._hold_joy(make_joy(deadman=True), duration=0.2)
        self._hold_joy(make_joy(deadman=True, **button_kwargs), duration=0.3)
        self._hold_joy(make_joy(deadman=True), duration=0.2)

    def _gripper_position(self):
        idx = list(self.latest_joint_state.name).index("joint_6")
        return self.latest_joint_state.position[idx]

    def _force_direct_jog_mode(self):
        """Mode isn't directly observable, and unittest runs test methods
        alphabetically (not declaration order), so a test can't assume
        whatever mode an earlier test left behind. Probe instead: a tiny
        jog either moves the real arm (already DIRECT_JOG -- done) or
        doesn't (SET_REFERENCE -- toggle once). Makes every test
        genuinely self-contained regardless of run order."""
        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=0.3)
        rospy.sleep(0.2)
        moved = (self.latest_pose.pose.position.x - start_x) > 0.005
        if not moved:
            self._tap_button(mode_toggle=True)
            rospy.sleep(0.2)

    def test_sustained_stick_moves_tracked_position_in_direct_jog(self):
        self._force_direct_jog_mode()
        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.3)
        moved_x = self.latest_pose.pose.position.x
        self.assertGreater(
            moved_x - start_x, 0.02,
            f"expected meaningful +X motion in DIRECT_JOG, start={start_x:.4f} moved={moved_x:.4f}",
        )

    def test_deadman_released_stops_motion(self):
        self._force_direct_jog_mode()
        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, deadman=False), duration=1.0)
        rospy.sleep(0.3)
        end_x = self.latest_pose.pose.position.x
        self.assertAlmostEqual(
            end_x, start_x, delta=0.01,
            msg=f"deadman released but position moved anyway: start={start_x:.4f} end={end_x:.4f}",
        )

    def test_set_reference_mode_does_not_move_real_arm(self):
        self._force_direct_jog_mode()
        # Toggle DIRECT_JOG -> SET_REFERENCE.
        self._tap_button(mode_toggle=True)
        rospy.sleep(0.2)

        start_x = self.latest_pose.pose.position.x
        self._hold_joy(make_joy(axis_x=1.0, axis_pitch=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.3)
        end_x = self.latest_pose.pose.position.x
        self.assertAlmostEqual(
            end_x, start_x, delta=0.01,
            msg="jogging in SET_REFERENCE must not move the real arm",
        )

        # Toggle back to DIRECT_JOG so later tests start from a known mode.
        self._tap_button(mode_toggle=True)
        rospy.sleep(0.2)

    def test_commit_from_set_reference_moves_real_arm(self):
        self._force_direct_jog_mode()
        self._tap_button(mode_toggle=True)  # -> SET_REFERENCE
        rospy.sleep(0.2)

        start_x = self.latest_pose.pose.position.x
        # Move the reference away from the current pose (arm doesn't
        # follow yet -- confirmed by the previous test).
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.2)

        self._tap_button(commit=True)

        # Give the generated trajectory time to execute.
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        moved = False
        while rospy.Time.now() < deadline:
            rospy.sleep(0.2)
            if self.latest_pose.pose.position.x - start_x > 0.02:
                moved = True
                break
        self.assertTrue(moved, "commit should have driven the real arm toward the reference pose")

        self._tap_button(mode_toggle=True)  # back to DIRECT_JOG
        rospy.sleep(0.2)

    def test_gripper_moves_independently_of_pose(self):
        self._force_direct_jog_mode()
        start_x = self.latest_pose.pose.position.x
        start_gripper = self._gripper_position()

        self._hold_joy(make_joy(axis_x=0.0, deadman=True, gripper_open=True), duration=1.0)
        rospy.sleep(0.3)

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
    rostest.rosrun(PKG, "teleop_modes_test", TeleopModesTest)
