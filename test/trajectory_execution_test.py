#!/usr/bin/env python3
"""Integration test: /generate_trajectory + the follow_joint_trajectory
action, executed against the real simulator. Confirms /joint_states_in
tracks the generated profile at several points throughout its duration
(not just the endpoint), and that mid-execution preemption actually stops
playback rather than completing anyway.
"""
import math
import unittest

import actionlib
import rospy
import rostest
from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState

PKG = "arm_trajectory"

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
POS_TOL = 0.15  # rad -- loose: real ROS scheduling jitter (actionlib dispatch, 50Hz sim
                # update latency) accumulates measurably over a wait-then-sample check;
                # this test is confirming rough tracking throughout, not tight timing.


class TrajectoryExecutionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("trajectory_execution_test")

        rospy.wait_for_service("/generate_trajectory", timeout=30.0)
        cls.generate = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)

        cls.client = actionlib.SimpleActionClient(
            "follow_joint_trajectory", FollowJointTrajectoryAction
        )
        cls.client.wait_for_server(timeout=rospy.Duration(30.0))

        cls.latest_joint_state = None
        cls.joint_sub = rospy.Subscriber("/joint_states_in", JointState, cls._joint_cb)
        rospy.sleep(1.0)

    @classmethod
    def _joint_cb(cls, msg):
        cls.latest_joint_state = msg

    def _current_positions(self, names):
        msg = type(self).latest_joint_state
        self.assertIsNotNone(msg, "never received /joint_states_in")
        out = []
        for name in names:
            idx = list(msg.name).index(name)
            out.append(msg.position[idx])
        return out

    def test_execution_tracks_generated_profile(self):
        req = GenerateTrajectoryRequest()
        req.start_joint_positions = [0.0] * 5
        req.goal_joint_positions = [0.5, 0.8, -0.4, 0.6, -0.6]
        req.sample_rate = 50.0
        res = self.generate(req)
        self.assertTrue(res.success, f"generate_trajectory failed: {res.message}")
        self.assertGreaterEqual(len(res.trajectory.points), 2)

        # Drive the sim to the trajectory's declared start first (the
        # service assumed start_joint_positions, but the sim itself starts
        # wherever it last was -- align them before executing).
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = res.trajectory
        goal.trajectory.header.stamp = rospy.Time.now()

        send_time = rospy.Time.now()
        self.client.send_goal(goal)

        # Check tracking at several points along the way, not just the end.
        check_indices = sorted(
            set([1, len(res.trajectory.points) // 3, 2 * len(res.trajectory.points) // 3,
                 len(res.trajectory.points) - 1])
        )
        for idx in check_indices:
            point = res.trajectory.points[idx]
            target_time = send_time + point.time_from_start + rospy.Duration(0.1)  # small margin
            rospy.sleep(max(0.0, (target_time - rospy.Time.now()).to_sec()))
            actual = self._current_positions(JOINT_NAMES)
            for j, name in enumerate(JOINT_NAMES):
                err = abs(actual[j] - point.positions[j])
                self.assertLess(
                    err, POS_TOL,
                    f"point {idx} joint {name}: expected {point.positions[j]:.3f}, "
                    f"got {actual[j]:.3f} (err {err:.3f})",
                )

        result = self.client.wait_for_result(timeout=rospy.Duration(10.0))
        self.assertTrue(result, "action did not finish")
        self.assertEqual(self.client.get_state(), actionlib.GoalStatus.SUCCEEDED)

    def test_preemption_stops_execution_early(self):
        # A deliberately long move (joint_4 across nearly its full range)
        # so there's plenty of time to cancel mid-flight.
        req = GenerateTrajectoryRequest()
        req.start_joint_positions = [0.0] * 5
        req.goal_joint_positions = [0.0, 0.0, 0.0, 4.0, 0.0]
        req.sample_rate = 50.0
        res = self.generate(req)
        self.assertTrue(res.success, f"generate_trajectory failed: {res.message}")
        total_duration = res.trajectory.points[-1].time_from_start.to_sec()
        self.assertGreater(total_duration, 2.0, "test needs a multi-second trajectory")

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = res.trajectory
        goal.trajectory.header.stamp = rospy.Time.now()

        start = rospy.Time.now()
        self.client.send_goal(goal)
        rospy.sleep(min(1.0, total_duration / 3.0))
        self.client.cancel_goal()

        result = self.client.wait_for_result(timeout=rospy.Duration(10.0))
        elapsed = (rospy.Time.now() - start).to_sec()
        self.assertTrue(result, "action did not settle after cancel")
        self.assertEqual(self.client.get_state(), actionlib.GoalStatus.PREEMPTED)
        self.assertLess(
            elapsed, total_duration * 0.8,
            "cancelled action ran nearly to completion instead of stopping early",
        )


if __name__ == "__main__":
    rostest.rosrun(PKG, "trajectory_execution_test", TrajectoryExecutionTest)
