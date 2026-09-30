#!/usr/bin/env python3
"""Integration test: /reach_target_pose against the real simulator.

Confirms the full generate+execute+verify path this branch's milestone
adds: a reachable target pose is actually driven to (checked two ways --
the service's own self-reported final_position_error/final_orientation_error
(orientation is pitch-only, see ReachTargetPose.srv -- roll/yaw are never
enforced), AND an independent read of /computed_end_effector_pose after the
call returns, so a bug that made the service lie about its own success
wouldn't be missed), and an unreachable target pose fails cleanly with an
informative message rather than hanging or crashing.
"""
import unittest

import actionlib
import rospy
import rostest
from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest
from arm_trajectory.srv import ReachTargetPose, ReachTargetPoseRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped

PKG = "arm_trajectory"

POS_TOL = 0.01   # meters -- independent check via /computed_end_effector_pose,
ORI_TOL = 0.05   # radians    looser than the service's own 1e-4/1e-3 internal tolerance
                 # to leave room for the topic-read timing/staleness this test can't control.


class ReachTargetPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("reach_target_pose_test")

        rospy.wait_for_service("/generate_trajectory", timeout=30.0)
        cls.generate = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
        rospy.wait_for_service("/reach_target_pose", timeout=30.0)
        cls.reach = rospy.ServiceProxy("/reach_target_pose", ReachTargetPose)

        cls.client = actionlib.SimpleActionClient(
            "follow_joint_trajectory", FollowJointTrajectoryAction
        )
        cls.client.wait_for_server(timeout=rospy.Duration(30.0))

        cls.latest_pose = None
        cls.pose_sub = rospy.Subscriber("/computed_end_effector_pose", PoseStamped, cls._pose_cb)

        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while cls.latest_pose is None and rospy.Time.now() < deadline:
            rospy.sleep(0.1)
        if cls.latest_pose is None:
            raise AssertionError("never received /computed_end_effector_pose")

    @classmethod
    def _pose_cb(cls, msg):
        cls.latest_pose = msg

    def _drive_to_joint_config(self, goal_joint_positions):
        """Direct joint-space move, bypassing /reach_target_pose -- used to
        set up test preconditions (a known reference pose, and a
        deliberately-different starting point), not to test it."""
        req = GenerateTrajectoryRequest()
        req.goal_joint_positions = goal_joint_positions
        req.sample_rate = 50.0
        res = self.generate(req)
        self.assertTrue(res.success, f"setup move failed to generate: {res.message}")

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = res.trajectory
        goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(timeout=rospy.Duration(20.0))
        self.assertTrue(finished, "setup move did not finish")
        self.assertEqual(self.client.get_state(), actionlib.GoalStatus.SUCCEEDED)
        rospy.sleep(0.3)  # let /computed_end_effector_pose catch up

    def test_reaches_known_reachable_target_pose(self):
        # Capture a real, guaranteed-reachable pose from the sim's own FK.
        self._drive_to_joint_config([0.3, 0.6, -0.4, 0.6, -0.6])
        target_pose = self.latest_pose.pose

        # Move away first, so the upcoming call has to actually do something.
        self._drive_to_joint_config([-0.5, 1.2, -1.0, 1.5, -1.5])

        req = ReachTargetPoseRequest()
        req.target_pose = target_pose
        req.cartesian_path = False
        res = self.reach(req)

        self.assertTrue(res.success, f"reach_target_pose failed: {res.message}")
        self.assertLess(res.final_position_error, 1e-3,
                         f"self-reported position error too large: {res.final_position_error}")
        self.assertLess(res.final_orientation_error, 1e-2,
                         f"self-reported orientation error too large: {res.final_orientation_error}")

        # Independent check: did the arm actually get there, per its own FK topic.
        actual = self.latest_pose.pose
        dx = actual.position.x - target_pose.position.x
        dy = actual.position.y - target_pose.position.y
        dz = actual.position.z - target_pose.position.z
        position_error = (dx * dx + dy * dy + dz * dz) ** 0.5
        self.assertLess(position_error, POS_TOL,
                         f"arm did not actually reach the target pose (err {position_error:.4f} m)")

    def test_unreachable_target_pose_fails_cleanly(self):
        req = ReachTargetPoseRequest()
        req.target_pose.position.x = 10.0
        req.target_pose.position.y = 10.0
        req.target_pose.position.z = 10.0
        req.target_pose.orientation.w = 1.0
        req.cartesian_path = False

        res = self.reach(req)
        self.assertFalse(res.success)
        self.assertTrue(res.message, "failure should come with an explanatory message")


if __name__ == "__main__":
    rostest.rosrun(PKG, "reach_target_pose_test", ReachTargetPoseTest)
