#!/usr/bin/env python3
"""Mission control backend: the one place that knows how a plant becomes a grasp (see arm_mission/mission_core.py). The rviz panel (arm_gui MissionPanel) and
the rqt plugin (arm_mission) are clients of ~command and ~state only.

    ~state    (std_msgs/String, JSON, latched, 2 Hz): plants, selection, settings, dry-run result, sequencer state, go gate
    ~command  (arm_gui/MissionCommand): refresh | select | set_approach | set_bias | set_height | dry_run | go | stop

It sets the runtime parameters of grasp_target (offset, approach, bias) and object_detector (grasp_heights) and calls /grasp_sequencer/grasp. It never moves
anything by itself: a real grasp needs a successful dry run of the current settings (go gate).

Parameters: ~plant_class (Vigna), ~marker_frame (marker_0), ~plan_valid_s (120), ~snapshot_valid_s (60), ~state_rate_hz (2), ~base_frame (base_link),
~tool_frame (end_effector), ~target_frame (grasp_target), ~grasp_target_ns, ~detector_ns, ~sequencer_ns (node names the settings are written to).
"""
import json
import threading

import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf.transformations import quaternion_matrix

from arm_grasp.srv import Grasp
from arm_gui.srv import MissionCommand, MissionCommandResponse
from arm_mission.mission_core import MissionController
from arm_perception.msg import ObjectDetectionArray


class RosIO(object):
    def __init__(self):
        P = rospy.get_param
        self.base, self.tool, self.marker = P("~base_frame", "base_link"), P("~tool_frame", "end_effector"), P("~marker_frame", "marker_0")
        self.target = P("~target_frame", "grasp_target")
        self.ns = {"grasp_target": P("~grasp_target_ns", "/grasp_target"), "object_detector": P("~detector_ns", "/object_detector")}
        self.seq = P("~sequencer_ns", "/grasp_sequencer")
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.det = None
        self.seq_status = {"state": "UNKNOWN", "message": "no /grasp_sequencer/status"}
        self.estop_state = None
        rospy.Subscriber("/object_detector/detections", ObjectDetectionArray, self.on_det, queue_size=1)
        rospy.Subscriber(self.seq + "/status", String, self.on_seq, queue_size=1)
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: setattr(self, "estop_state", bool(m.data)), queue_size=1)

    def on_det(self, m):
        self.det = (m.detections, m.floor_source)

    def on_seq(self, m):
        try:
            d = json.loads(m.data)
            self.seq_status = {"state": d.get("state", "UNKNOWN"), "message": d.get("message", "")}
        except ValueError:
            pass

    def now(self):
        return rospy.get_time()

    def _tf(self, frame):
        t = self.buf.lookup_transform(self.base, frame, rospy.Time(0), rospy.Duration(0.0)).transform
        return t

    def tool_axis(self):
        try:
            t = self._tf(self.tool)
        except tf2_ros.TransformException:
            return None
        M = quaternion_matrix([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        return np.array([t.translation.x, t.translation.y, t.translation.z]), M[:3, 2]

    def marker_pose(self):
        try:
            t = self._tf(self.marker)
        except tf2_ros.TransformException:
            return None
        return [t.translation.x, t.translation.y, t.translation.z], [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]

    def detections(self):
        return self.det

    def _name(self, name):
        node, key = name.split("/", 1)
        return "%s/%s" % (self.ns[node], key)

    def get_param(self, name, default):
        return rospy.get_param(self._name(name), default)

    def set_param(self, name, value):
        rospy.set_param(self._name(name), value)

    def grasp(self, dry_run):
        rospy.wait_for_service(self.seq + "/grasp", timeout=2.0)
        r = rospy.ServiceProxy(self.seq + "/grasp", Grasp)(target_frame=self.target, dry_run=dry_run, probe=False)
        return r.success, r.message, r.plan_duration_s, r.max_cartesian_deviation_m

    def stop(self):
        try:
            rospy.wait_for_service(self.seq + "/stop", timeout=2.0)
            r = rospy.ServiceProxy(self.seq + "/stop", Trigger)()
            return r.success, r.message
        except (rospy.ROSException, rospy.ServiceException) as e:
            return False, "stop failed: %s" % e

    def run_state(self):
        return self.seq_status["state"], self.seq_status["message"]

    def estop(self):
        return self.estop_state

    def spawn(self, fn):
        threading.Thread(target=fn, daemon=True).start()


def main():
    rospy.init_node("mission_control")
    P = rospy.get_param
    io = RosIO()
    ctl = MissionController(io, plant_class=P("~plant_class", "Vigna"), plan_valid_s=P("~plan_valid_s", 120.0), snapshot_valid_s=P("~snapshot_valid_s", 60.0))
    pub = rospy.Publisher("~state", String, queue_size=1, latch=True)

    def on_command(req):
        try:
            ok, msg = ctl.command(req.action, req.index, req.values, req.text)
        except Exception as e:                                   # a front end must get an answer, not a hung call
            rospy.logerr("mission_control: %s failed: %s", req.action, e)
            ok, msg = False, "%s failed: %s" % (req.action, e)
        rospy.loginfo("mission_control: %s -> %s %s", req.action, ok, msg)
        pub.publish(String(json.dumps(ctl.state())))
        return MissionCommandResponse(ok, msg)

    rospy.Service("~command", MissionCommand, on_command)
    rate = rospy.Rate(P("~state_rate_hz", 2.0))
    while not rospy.is_shutdown():
        try:
            pub.publish(String(json.dumps(ctl.state())))
        except Exception as e:
            rospy.logerr_throttle(5.0, "mission_control: state failed: %s", e)
        rate.sleep()


if __name__ == "__main__":
    main()
