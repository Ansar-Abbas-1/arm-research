#!/usr/bin/env python3
"""Integration test: fk_node's /computed_end_effector_pose vs. the ground-truth
TF (base_link -> end_effector) that robot_state_publisher derives from the same
URDF via KDL. Two independent computations of the same chain -- this is
the "validated against the simulator" requirement from the milestone plan,
made automated rather than a manual spot check.
"""
import math
import random
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

PKG = "arm_kinematics"

# From urdf/arm.urdf, joints 1-5.
JOINT_LIMITS = [
    (-2.745, 2.745),
    (0.0, 3.14),
    (-2.220, 0.0),
    (0.0, 4.505),
    (-3.62, 0.0),
]

POS_TOL = 1e-4   # meters -- ideal-mode sim clamps/converges instantly, so
ROT_TOL = 1e-3   # radians    this should be tight; not a control-loop tolerance


def quat_angle_diff(q1, q2):
    """Angle (rad) between two quaternions, sign-ambiguity-safe."""
    dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)


class ForwardKinematicsValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("fk_validation_test")
        cls.pub = rospy.Publisher("/joint_states_out", JointState, queue_size=10)
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer)

        cls.latest_pose = None
        cls.pose_sub = rospy.Subscriber(
            "/computed_end_effector_pose", PoseStamped, cls._pose_cb
        )

        # Let publishers/subscribers/TF connect and the sim start publishing.
        deadline = rospy.Time.now() + rospy.Duration(10.0)
        while rospy.Time.now() < deadline and cls.pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        rospy.sleep(1.0)

    @classmethod
    def _pose_cb(cls, msg):
        cls.latest_pose = msg

    def _drive_and_check(self, q, label):
        msg = JointState()
        msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        msg.position = list(q) + [0.0]

        type(self).latest_pose = None
        end = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < end:
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rospy.sleep(0.05)
            if type(self).latest_pose is not None:
                break

        self.assertIsNotNone(
            type(self).latest_pose, f"[{label}] never received /computed_end_effector_pose"
        )

        try:
            tf = self.tf_buffer.lookup_transform(
                "base_link", "end_effector", rospy.Time(0), rospy.Duration(2.0)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.fail(f"[{label}] TF lookup base_link->end_effector failed: {e}")

        computed = type(self).latest_pose.pose
        expected = tf.transform

        pos_err = math.sqrt(
            (computed.position.x - expected.translation.x) ** 2
            + (computed.position.y - expected.translation.y) ** 2
            + (computed.position.z - expected.translation.z) ** 2
        )
        rot_err = quat_angle_diff(computed.orientation, expected.rotation)

        self.assertLess(
            pos_err,
            POS_TOL,
            f"[{label}] q={q} position error {pos_err:.6f} m "
            f"(computed={computed.position}, tf={expected.translation})",
        )
        self.assertLess(
            rot_err,
            ROT_TOL,
            f"[{label}] q={q} orientation error {rot_err:.6f} rad",
        )

    def test_zero_config(self):
        self._drive_and_check([0.0] * 5, "zero")

    def test_joint_limit_extremes(self):
        for i in range(5):
            for bound_name, bound_val in [("lower", JOINT_LIMITS[i][0]), ("upper", JOINT_LIMITS[i][1])]:
                q = [0.0] * 5
                q[i] = bound_val
                self._drive_and_check(q, f"joint_{i+1}_{bound_name}")

    def test_random_configs_within_limits(self):
        rng = random.Random(42)  # fixed seed: reproducible failures
        for trial in range(15):
            q = [rng.uniform(lo, hi) for lo, hi in JOINT_LIMITS]
            self._drive_and_check(q, f"random_{trial}")


if __name__ == "__main__":
    rostest.rosrun(PKG, "fk_validation_test", ForwardKinematicsValidationTest)
