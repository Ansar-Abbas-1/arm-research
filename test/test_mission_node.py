#!/usr/bin/env python3
"""rostest of mission_control_node.py against stubs: detections, TF (end_effector pointing down, marker_0), a fake grasp sequencer and e-stop state.
The node must publish its state, rank the plants, write the target parameters, and release a real grasp only after a dry run of the same settings."""
import json
import threading
import unittest

import rospy
import rostest
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from arm_grasp.srv import Grasp, GraspResponse
from arm_gui.srv import MissionCommand
from arm_perception.msg import ObjectDetection, ObjectDetectionArray


def tf(parent, child, xyz, q):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    return t


def detection(x, y, z=-0.51, cls="Vigna"):
    d = ObjectDetection()
    d.class_name, d.valid, d.confidence = cls, True, 0.9
    d.position.x, d.position.y, d.position.z = x, y, z
    return d


class MissionNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("test_mission_node")
        cls.grasp_calls = []
        cls.state = {}
        cls.lock = threading.Lock()
        cls.static = tf2_ros.StaticTransformBroadcaster()
        # end effector 0.3 m above the origin, z axis pointing DOWN (rotation of pi about x); marker_0 on the floor
        cls.static.sendTransform([tf("base_link", "end_effector", (0.0, 0.0, 0.3), (1.0, 0.0, 0.0, 0.0)), tf("base_link", "marker_0", (0.4, 0.0, -0.6), (0.0, 0.0, 0.0, 1.0))])
        cls.det_pub = rospy.Publisher("/object_detector/detections", ObjectDetectionArray, queue_size=1, latch=True)
        cls.seq_pub = rospy.Publisher("/grasp_sequencer/status", String, queue_size=1, latch=True)
        cls.estop_pub = rospy.Publisher("/arm_interface/estop_state", Bool, queue_size=1, latch=True)
        arr = ObjectDetectionArray()
        arr.detections = [detection(0.45, 0.2), detection(0.02, 0.01), detection(0.3, -0.3)]
        arr.floor_source = "marker_0"
        cls.det_pub.publish(arr)
        cls.set_run_state("IDLE")
        cls.estop_pub.publish(Bool(False))
        rospy.Service("/grasp_sequencer/grasp", Grasp, cls.on_grasp)
        rospy.Service("/grasp_sequencer/stop", Trigger, lambda _: TriggerResponse(True, "stopped"))
        rospy.Subscriber("/mission_control/state", String, cls.on_state, queue_size=1)
        rospy.wait_for_service("/mission_control/command", 20.0)
        cls.cmd = rospy.ServiceProxy("/mission_control/command", MissionCommand)

    @classmethod
    def set_run_state(cls, state):
        cls.seq_pub.publish(String(json.dumps({"state": state, "message": ""})))

    @classmethod
    def on_grasp(cls, req):
        cls.grasp_calls.append((req.target_frame, req.dry_run, req.probe))
        return GraspResponse(True, "plan feasible", 21.0, 0.002)

    @classmethod
    def on_state(cls, m):
        with cls.lock:
            cls.state = json.loads(m.data)

    def st(self):
        with self.lock:
            return dict(self.state)

    def wait(self, pred, timeout=10.0):
        t = rospy.get_time() + timeout
        while rospy.get_time() < t and not rospy.is_shutdown():
            if pred(self.st()):
                return True
            rospy.sleep(0.05)
        return pred(self.st())

    def test_1_state_is_published_and_the_gate_is_closed_at_first(self):
        self.assertTrue(self.wait(lambda s: s.get("marker_visible") and s.get("live_count") == 3))
        s = self.st()
        self.assertFalse(s["go_enabled"]); self.assertEqual(s["selected"], -1)
        self.assertEqual(s["settings"]["approach"], "45"); self.assertEqual(s["floor_source"], "marker_0")

    def test_2_refresh_ranks_the_plants_and_writes_the_offset_parameter(self):
        r = self.cmd("refresh", 0, [], "")
        self.assertTrue(r.success, r.message)
        self.assertTrue(self.wait(lambda s: len(s.get("plants", [])) == 3 and s["selected"] == 0))
        s = self.st()
        self.assertAlmostEqual(s["plants"][0]["position"][0], 0.02)          # the one on the gripper axis
        off = rospy.get_param("/grasp_target/offset")
        self.assertAlmostEqual(off[0], 0.02 - 0.4, places=3)                  # identity marker: base axes
        self.assertAlmostEqual(off[2], -0.51 + 0.6, places=3)

    def test_3_go_is_refused_without_a_dry_run_and_released_after_one(self):
        self.cmd("refresh", 0, [], "")
        r = self.cmd("go", 0, [], "")
        self.assertFalse(r.success); self.assertEqual([c for c in self.grasp_calls if not c[1]], [])
        self.assertTrue(self.cmd("dry_run", 0, [], "").success)
        self.assertTrue(self.wait(lambda s: s["plan"]["state"] == "ok" and s["go_enabled"]))
        self.assertEqual(self.grasp_calls[-1], ("grasp_target", True, False))
        self.assertTrue(self.cmd("go", 0, [], "").success)
        self.assertTrue(self.wait(lambda s: False, 0.3) or True)
        self.assertEqual(self.grasp_calls[-1], ("grasp_target", False, False))
        self.assertFalse(self.cmd("go", 0, [], "").success)                   # one dry run, one go

    def test_4_settings_reach_the_parameter_server_and_invalidate_the_plan(self):
        self.cmd("refresh", 0, [], ""); self.cmd("dry_run", 0, [], "")
        self.assertTrue(self.wait(lambda s: s["plan"]["state"] == "ok"))
        self.assertTrue(self.cmd("set_bias", 0, [-0.02, 0.0, 0.0], "").success)
        self.assertEqual(rospy.get_param("/grasp_target/bias"), [-0.02, 0.0, 0.0])
        self.assertFalse(self.cmd("go", 0, [], "").success)
        self.assertTrue(self.cmd("set_approach", 0, [], "down").success)
        self.assertEqual(rospy.get_param("/grasp_target/approach"), "down")
        self.assertFalse(self.cmd("set_approach", 0, [], "200").success)
        self.assertTrue(self.cmd("set_height", 0, [0.07], "").success)
        self.assertEqual(rospy.get_param("/object_detector/grasp_heights"), {"Vigna": 0.07})
        self.assertTrue(self.wait(lambda s: s["snapshot_stale"]))

    def test_5_estop_and_a_running_grasp_block_everything_but_stop(self):
        self.cmd("set_height", 0, [0.09], ""); self.cmd("set_approach", 0, [], "45")
        self.cmd("refresh", 0, [], ""); self.cmd("dry_run", 0, [], "")
        self.assertTrue(self.wait(lambda s: s["plan"]["state"] == "ok" and s["go_enabled"]))
        self.estop_pub.publish(Bool(True))
        self.assertTrue(self.wait(lambda s: s["estop"] and not s["go_enabled"]))
        self.assertFalse(self.cmd("go", 0, [], "").success)
        self.estop_pub.publish(Bool(False))
        self.set_run_state("EXECUTING")
        self.assertTrue(self.wait(lambda s: s["busy"]))
        self.assertFalse(self.cmd("refresh", 0, [], "").success)
        r = self.cmd("stop", 0, [], "")
        self.assertTrue(r.success)
        self.set_run_state("IDLE")

    def test_6_unknown_action_is_refused(self):
        self.assertFalse(self.cmd("fly", 0, [], "").success)


if __name__ == "__main__":
    rostest.rosrun("arm_mission", "test_mission_node", MissionNodeTest)
