#!/usr/bin/env python3
"""rostest of the load-limited gripper close ALONE: the simulated arm and the gripper_close node, nothing else.
The simulator's gripper has a contact model (/arm_sim/gripper_object, a joint_6 position at which closing is blocked;
NaN = closes freely): the load grows once the command goes past it. Only the open/close CONDITIONS are tested:
  load target reached before fully closed -> 'object', held near the target
  fully closed with the load below the target -> 'thin' (counted as grasped)
  watch only -> nothing is sent; e-stop -> refused; no measurement -> refused
"""
import math
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from arm_grasp.srv import GripperClose

OPEN, TARGET, BAND = 0.025, 40.0, 5.0


class GripperCloseAlone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("gripper_close_alone")
        cls.contact = rospy.Publisher("/arm_sim/gripper_object", Float64, queue_size=1, latch=True)
        cls.out = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
        cls.pos = {}
        rospy.Subscriber("/joint_states_in", JointState, lambda m: cls.pos.update(dict(zip(m.name, m.position))), queue_size=1)
        rospy.wait_for_service("/gripper_close/close", 30)
        cls.close = rospy.ServiceProxy("/gripper_close/close", GripperClose)
        t0 = time.time()
        while "joint_6" not in cls.pos and time.time() - t0 < 10:
            time.sleep(0.1)

    def setUp(self):
        self.contact.publish(Float64(math.nan))

    def gripper_to(self, x):
        for _ in range(60):
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name, js.position = ["joint_6"], [x]
            self.out.publish(js)
            time.sleep(0.05)

    def test_a_only_the_simulator_and_the_close_node_run(self):
        import rosnode
        nodes = set(rosnode.get_node_names()) - {"/rosout", rospy.get_name()}
        self.assertEqual(nodes, {"/arm_sim_validator_node", "/gripper_close"})

    def test_b_watch_only_sends_nothing(self):
        self.contact.publish(Float64(0.012))
        self.gripper_to(OPEN)
        r = self.close(False, False)
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.outcome, "not executed")
        self.assertAlmostEqual(self.pos["joint_6"], OPEN, places=6)

    def test_c_load_reached_before_fully_closed_is_held_near_the_target(self):
        self.contact.publish(Float64(0.012))
        self.gripper_to(OPEN)
        t0 = time.time()
        r = self.close(True, False)
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.outcome, "object")
        self.assertLessEqual(abs(r.load_percent - TARGET), BAND)
        self.assertGreater(self.pos["joint_6"], 0.001)               # stopped before fully closed
        self.assertGreater(time.time() - t0, 3.0)                    # a slow close, not the fast ramp

    def test_d_open_first_then_close(self):
        self.contact.publish(Float64(0.020))
        self.gripper_to(0.0)
        r = self.close(True, True)
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.outcome, "object")

    def test_e_fully_closed_below_the_target_is_thin(self):
        self.gripper_to(OPEN)
        r = self.close(True, False)                                  # closes freely: the load never rises
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.outcome, "thin")
        self.assertLess(r.load_percent, TARGET - BAND)

    def test_f_a_weak_contact_that_never_reaches_the_target_is_thin(self):
        self.contact.publish(Float64(0.003))                         # blocks only near the end: the load stays low even when fully closed
        self.gripper_to(OPEN)
        r = self.close(True, False)
        self.assertTrue(r.success, r.message)
        self.assertEqual(r.outcome, "thin")
        self.assertLess(r.load_percent, TARGET - BAND)

    def test_g_e_stop_refuses_and_sends_nothing(self):
        self.gripper_to(OPEN)
        rospy.ServiceProxy("/arm_interface/estop", Trigger)()
        time.sleep(0.5)
        r = self.close(True, False)
        self.assertFalse(r.success)
        self.assertIn("e-stop", r.message)
        self.assertAlmostEqual(self.pos["joint_6"], OPEN, places=6)
        rospy.ServiceProxy("/arm_interface/estop_reset", Trigger)()

    def test_h_operator_stop_ends_a_close(self):
        import threading
        self.contact.publish(Float64(0.012))
        self.gripper_to(OPEN)
        res = {}
        th = threading.Thread(target=lambda: res.update(r=self.close(True, False)))
        th.start()
        time.sleep(1.5)
        rospy.ServiceProxy("/gripper_close/stop", Trigger)()
        th.join(10)
        self.assertFalse(res["r"].success)
        self.assertIn("operator", res["r"].message)

    def test_z_without_the_simulator_nothing_is_commanded(self):
        import rosnode
        rosnode.kill_nodes(["/arm_sim_validator_node"])
        time.sleep(1.0)
        # the last measurements are kept by the node, but their load is stale: the close must refuse or fault, never succeed
        r = self.close(True, False)
        self.assertFalse(r.success)
        self.assertIn("load", r.message)


if __name__ == "__main__":
    rostest.rosrun("arm_grasp", "gripper_close_alone", GripperCloseAlone)
