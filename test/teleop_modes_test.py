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
  5. Ghost preview (/ghost_joint_states): mirrors the real arm in
     DIRECT_JOG, diverges from it in SET_REFERENCE, and the reference
     marker (/teleop_reference_marker) is deleted on leaving SET_REFERENCE.
"""
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker

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
        cls.latest_ghost_state = None
        cls.ghost_sub = rospy.Subscriber("/ghost_joint_states", JointState, cls._ghost_cb)
        cls.latest_markers = {}  # keyed by namespace: both the reference and jog-target
        cls.marker_sub = rospy.Subscriber("/teleop_reference_marker", Marker, cls._marker_cb)  # markers share this topic, distinguished by ns
        cls.latest_mode = None
        cls.mode_sub = rospy.Subscriber("/teleop_mode", String, cls._mode_cb)

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

    @classmethod
    def _ghost_cb(cls, msg):
        cls.latest_ghost_state = msg

    @classmethod
    def _marker_cb(cls, msg):
        cls.latest_markers[msg.ns] = msg

    @classmethod
    def _mode_cb(cls, msg):
        cls.latest_mode = msg.data

    def _ghost_position(self, joint_name):
        idx = list(self.latest_ghost_state.name).index(joint_name)
        return self.latest_ghost_state.position[idx]

    def _actual_position(self, joint_name):
        idx = list(self.latest_joint_state.name).index(joint_name)
        return self.latest_joint_state.position[idx]

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
        """Mode isn't directly observable via arm motion alone -- an
        earlier test can leave the arm pinned against a workspace boundary
        in either direction, which used to fool a jog-and-see-if-it-moved
        probe into concluding "must be SET_REFERENCE" when it was actually
        just saturated (confirmed the hard way). /teleop_mode (latched)
        exists precisely to sidestep this: read it directly instead of
        inferring anything from arm motion. Toggling is still needed since
        unittest runs test methods alphabetically, not declaration order,
        so a test can't assume whatever mode an earlier one left behind."""
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while self.latest_mode is None and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        if self.latest_mode is None:
            raise AssertionError("never received /teleop_mode")
        if self.latest_mode != "DIRECT_JOG":
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

        # Wait for the committed trajectory to actually FINISH (position
        # settles), not just for the first sign of movement -- otherwise a
        # still-executing trajectory can bleed into whichever test happens
        # to run next (unittest runs alphabetically, not declaration
        # order): confirmed the hard way, this caused
        # test_ghost_diverges_and_marker_clears_in_set_reference to see
        # the "real arm must not move" assertion fail intermittently.
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        last_x = self.latest_pose.pose.position.x
        moved_at_all = False
        settled_ticks = 0
        while rospy.Time.now() < deadline and settled_ticks < 3:
            rospy.sleep(0.2)
            current_x = self.latest_pose.pose.position.x
            if current_x - start_x > 0.02:
                moved_at_all = True
            settled_ticks = settled_ticks + 1 if abs(current_x - last_x) < 0.001 else 0
            last_x = current_x
        self.assertTrue(moved_at_all, "commit should have driven the real arm toward the reference pose")

        self._tap_button(mode_toggle=True)  # back to DIRECT_JOG
        rospy.sleep(0.2)

    def test_commit_still_works_after_extended_jogging(self):
        # Regression test: the reference pose used to accumulate jog input
        # unconditionally, with no reachability check, so pushing it hard
        # enough could silently drift outside the workspace -- at which
        # point the ghost preview would freeze AND commit's own IK (same
        # solver, same options) would also silently fail, with no feedback
        # to the controller either way. The fix (TryUpdateReference in
        # teleop_node.cpp) rejects any candidate that doesn't converge
        # before ever committing to it, so the reference can never actually
        # become unreachable -- confirmed here by jogging hard for several
        # seconds (much longer than the other tests) and checking commit
        # still drives the real arm afterward.
        self._force_direct_jog_mode()
        self._tap_button(mode_toggle=True)  # -> SET_REFERENCE
        rospy.sleep(0.2)

        start_x = self.latest_pose.pose.position.x
        # 2s: deliberately more than the other tests' 1s jogs (that's the
        # "extended" this test is about), but not so much that it risks
        # pinning the arm against a real workspace boundary and breaking
        # later tests' own "+X should move it" assumptions -- confirmed
        # the hard way that 4s did exactly that.
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=2.0)
        rospy.sleep(0.3)

        self._tap_button(commit=True)

        deadline = rospy.Time.now() + rospy.Duration(10.0)
        last_x = self.latest_pose.pose.position.x
        moved_at_all = False
        settled_ticks = 0
        while rospy.Time.now() < deadline and settled_ticks < 3:
            rospy.sleep(0.2)
            current_x = self.latest_pose.pose.position.x
            if abs(current_x - start_x) > 0.01:
                moved_at_all = True
            settled_ticks = settled_ticks + 1 if abs(current_x - last_x) < 0.001 else 0
            last_x = current_x
        self.assertTrue(
            moved_at_all,
            "commit should still work after extended jogging, even if some of it overshot "
            "the reachable workspace and got rejected along the way",
        )

        self._tap_button(mode_toggle=True)  # back to DIRECT_JOG
        rospy.sleep(0.2)
        # Undo the net displacement this test adds, so it doesn't leave the
        # arm nearer a workspace boundary for whichever test happens to run
        # next (unittest runs alphabetically, not declaration order).
        self._hold_joy(make_joy(axis_x=-1.0, deadman=True), duration=2.0)
        rospy.sleep(0.3)

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

    def test_ghost_mirrors_real_arm_in_direct_jog(self):
        self._force_direct_jog_mode()
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=0.5)
        rospy.sleep(0.3)
        for name in ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]:
            self.assertAlmostEqual(
                self._ghost_position(name), self._actual_position(name), delta=0.01,
                msg=f"ghost should mirror the real arm in DIRECT_JOG (joint {name})",
            )

    def test_ghost_diverges_and_marker_clears_in_set_reference(self):
        self._force_direct_jog_mode()
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        start_positions = {name: self._actual_position(name) for name in joint_names}

        self._tap_button(mode_toggle=True)  # -> SET_REFERENCE
        rospy.sleep(0.2)
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=1.0)
        rospy.sleep(0.3)

        # The ghost should have moved to preview the (uncommitted)
        # reference, while the real arm stayed exactly where it was.
        for name in joint_names:
            self.assertAlmostEqual(
                self._actual_position(name), start_positions[name], delta=0.001,
                msg=f"real arm must not move while jogging in SET_REFERENCE (joint {name})",
            )
        # Compare the WHOLE configuration, not one specific joint: a pure
        # +X reach might leave any single joint (e.g. a base yaw) nearly
        # unchanged depending on the arm's current orientation, but the
        # ghost (previewing a real Cartesian displacement) and the
        # unmoved real arm should differ somewhere across the 5 joints.
        total_divergence = sum(
            abs(self._ghost_position(name) - self._actual_position(name)) for name in joint_names
        )
        self.assertGreater(
            total_divergence, 0.02,
            "ghost should diverge from the real arm to preview the reference in SET_REFERENCE",
        )
        self.assertEqual(
            self.latest_markers["teleop_reference"].action, Marker.ADD,
            "reference marker should be actively shown while in SET_REFERENCE",
        )

        self._tap_button(mode_toggle=True)  # back to DIRECT_JOG
        rospy.sleep(0.2)
        self.assertEqual(
            self.latest_markers["teleop_reference"].action, Marker.DELETE,
            "reference marker should be deleted on leaving SET_REFERENCE",
        )

    def test_jog_target_marker_shown_while_jogging_and_hidden_on_release(self):
        self._force_direct_jog_mode()
        self._hold_joy(make_joy(axis_x=1.0, deadman=True), duration=0.5)
        rospy.sleep(0.2)
        self.assertEqual(
            self.latest_markers["teleop_jog_target"].action, Marker.ADD,
            "jog target marker should be shown while actively jogging in DIRECT_JOG",
        )

        self._hold_joy(make_joy(axis_x=0.0, deadman=False), duration=0.3)
        rospy.sleep(0.2)
        self.assertEqual(
            self.latest_markers["teleop_jog_target"].action, Marker.DELETE,
            "jog target marker should be hidden once the deadman is released",
        )


if __name__ == "__main__":
    rostest.rosrun(PKG, "teleop_modes_test", TeleopModesTest)
