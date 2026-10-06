#!/usr/bin/env python3
"""Preset-pose backend for the rviz Arm Control panel (arm_gui) and any other client.

Holds the named joint poses (config/presets.yaml shipped; ones saved from the panel live in the user file,
default ~/.ros/arm_trajectory/presets.yaml, and override shipped ones by name) and runs the moves.

  /preset_manager/command  (arm_gui/PresetCommand)  go <name> | save <name> | cancel
  /preset_manager/state    (std_msgs/String, latched JSON) {"presets": [{"name", "note", "joints", "user"}],
                           "move": {"name", "state": idle|moving|done|failed, "message"}}

A move goes through exactly the path of stage3_joint_move.py: the joint-space trajectory service
(/generate_trajectory: velocity, acceleration and URDF limits) and the follow_joint_trajectory action, whose
executor refuses to move while the e-stop is engaged. Joint 6 (gripper) is not touched. The confirmation
dialogs live in the panel; this node only enforces the limits (URDF limits; max_delta if set) and one move at a time.
"""
import json
import os
import re
import threading
import xml.etree.ElementTree as ET

import yaml

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_ -]{0,31}$")


# ---- pure logic (no ROS) -----------------------------------------------------------------------------------

class PresetBook:
    """Named joint poses: shipped defaults overridden by the user's saved ones, validated against limits."""

    def __init__(self, shipped, user=None, limits=None):
        self.limits = limits or {}
        self.poses = {}
        self.notes = {}
        for source in (shipped or {}, user or {}):
            for name, entry in source.items():
                joints = [float(v) for v in entry["joints"]]
                self.validate(name, joints)
                self.poses[name] = joints
                self.notes[name] = entry.get("note", "")
        self.order = list(shipped.keys()) + [n for n in (user or {}) if n not in (shipped or {})]

    def validate(self, name, joints):
        if len(joints) != len(JOINTS):
            raise ValueError("preset '%s' has %d joints, expected %d" % (name, len(joints), len(JOINTS)))
        for j, q in zip(JOINTS, joints):
            if j in self.limits and not (self.limits[j][0] <= q <= self.limits[j][1]):
                raise ValueError("preset '%s': %s = %.3f is outside the URDF limits [%.3f, %.3f]" % (name, j, q, *self.limits[j]))

    def set_pose(self, name, joints, note=""):
        self.validate(name, joints)
        self.poses[name] = [float(v) for v in joints]
        if note:
            self.notes[name] = note
        if name not in self.order:
            self.order.append(name)


def load_user_file(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("presets", {})


def save_user_pose(path, name, joints, note="saved from rviz"):
    """Merge one pose into the user's preset file (atomic replace)."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    presets = load_user_file(path)
    presets[name] = {"joints": [round(float(v), 4) for v in joints], "note": note}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        yaml.safe_dump({"presets": presets}, f, default_flow_style=None, sort_keys=False)
    os.replace(tmp, path)


def check_move(start, goal, max_delta):
    """None if the move is allowed, else the reason it is refused. max_delta <= 0 disables the size guard."""
    if max_delta <= 0:
        return None
    delta = max(abs(g - s) for g, s in zip(goal, start))
    if delta > max_delta:
        return "largest joint move %.2f rad exceeds max_delta %.2f rad (preset_max_delta:=0 disables this check)" % (delta, max_delta)
    return None


def limits_from_urdf(urdf_xml):
    root = ET.fromstring(urdf_xml)
    return {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in root.iter("joint") if j.find("limit") is not None and j.get("type") in ("revolute", "prismatic")}


def valid_name(name):
    """Preset names: a letter first, then letters, digits, space, underscore or hyphen, at most 32 characters."""
    return bool(NAME_RE.match(name or ""))


def state_json(book, user_names, move):
    """JSON text of the /preset_manager/state message."""
    return json.dumps({"presets": [{"name": n, "note": book.notes.get(n, ""), "joints": book.poses[n], "user": n in user_names}
                                   for n in book.order],
                       "move": move})


# ---- ROS node ------------------------------------------------------------------------------------------------

class Node:
    def __init__(self):
        import actionlib
        import rospy
        from control_msgs.msg import FollowJointTrajectoryAction
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from arm_gui.srv import PresetCommand
        from arm_trajectory.srv import GenerateTrajectory
        self.rospy, self.GenerateTrajectory = rospy, GenerateTrajectory
        limits = limits_from_urdf(rospy.get_param("robot_description"))
        shipped = rospy.get_param("~presets")
        self.user_file = os.path.expanduser(rospy.get_param("~presets_file", "~/.ros/arm_trajectory/presets.yaml"))
        user = load_user_file(self.user_file)
        self.book = PresetBook(shipped, user, limits)
        self.user_names = set(user)
        self.max_delta = float(rospy.get_param("~max_delta", 0.0))
        self.move = {"name": "", "state": "idle", "message": ""}
        self.lock = threading.Lock()   # guards self.move: a move is running exactly while move['state'] == 'moving'
        self.latest = None
        self.client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        self.pub = rospy.Publisher("/preset_manager/state", String, queue_size=1, latch=True)
        self.String = String
        rospy.Service("/preset_manager/command", PresetCommand, self.on_command)
        self.publish()
        rospy.loginfo("preset manager: %s (move limit %.1f rad); your saved poses: %s",
                      ", ".join(self.book.order), self.max_delta, self.user_file)

    def publish(self):
        self.pub.publish(self.String(state_json(self.book, self.user_names, dict(self.move))))

    def on_joints(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    def current(self):
        if self.latest is None or any(j not in self.latest for j in JOINTS):
            return None
        return [self.latest[j] for j in JOINTS]

    def on_command(self, req):
        from arm_gui.srv import PresetCommandResponse as R
        if req.action == "go":
            if req.name not in self.book.poses:
                return R(False, "unknown preset '%s'" % req.name)
            start = self.current()
            with self.lock:
                if self.move["state"] == "moving":
                    return R(False, "a move is already running")
                why = "no joint states yet" if start is None else check_move(start, self.book.poses[req.name], self.max_delta)
                if why:
                    return R(False, why)
                self.move = {"name": req.name, "state": "moving", "message": ""}
                self.publish()
            threading.Thread(target=self.run_move, args=(req.name, start), daemon=True).start()
            return R(True, "moving to '%s'" % req.name)
        if req.action == "save":
            if not valid_name(req.name):
                return R(False, "name must start with a letter and use letters, digits, space, _ or - (32 max)")
            joints = self.current()
            if joints is None:
                return R(False, "no joint states yet")
            try:
                self.book.set_pose(req.name, joints, "saved from rviz")
            except ValueError as e:
                return R(False, str(e))
            save_user_pose(self.user_file, req.name, joints)
            self.user_names.add(req.name)
            self.publish()
            self.rospy.logwarn("preset '%s' saved: %s (in %s)", req.name, [round(q, 3) for q in joints], self.user_file)
            return R(True, "saved '%s'" % req.name)
        if req.action == "cancel":
            self.client.cancel_all_goals()
            return R(True, "cancel sent")
        return R(False, "unknown action '%s' (go, save, cancel)" % req.action)

    def run_move(self, name, start):
        from control_msgs.msg import FollowJointTrajectoryGoal
        from arm_trajectory.srv import GenerateTrajectoryRequest
        rospy = self.rospy
        try:
            goal = self.book.poses[name]
            rospy.wait_for_service("/generate_trajectory", timeout=3.0)
            req = GenerateTrajectoryRequest()
            req.start_joint_positions, req.goal_joint_positions = start, list(goal)
            res = rospy.ServiceProxy("/generate_trajectory", self.GenerateTrajectory)(req)
            if not res.success:
                raise RuntimeError("planning failed: " + res.message[:80])
            duration = res.trajectory.points[-1].time_from_start.to_sec()
            rospy.logwarn("preset '%s': moving, %.1f s (largest joint move %.2f rad)", name, duration, max(abs(g - s) for g, s in zip(goal, start)))
            if not self.client.wait_for_server(rospy.Duration(3.0)):
                raise RuntimeError("follow_joint_trajectory server not available")
            g = FollowJointTrajectoryGoal()
            g.trajectory = res.trajectory
            self.client.send_goal(g)
            if not self.client.wait_for_result(rospy.Duration(duration + 10.0)):
                self.client.cancel_all_goals()
                raise RuntimeError("timed out")
            if self.client.get_state() != 3:   # actionlib SUCCEEDED
                raise RuntimeError(self.client.get_goal_status_text() or "not completed")
            result = {"name": name, "state": "done", "message": ""}
            rospy.loginfo("preset '%s' reached", name)
        except Exception as e:
            result = {"name": name, "state": "failed", "message": str(e)[:80]}
            rospy.logerr("preset '%s' not executed: %s", name, e)
        with self.lock:
            self.move = result
            self.publish()


def main():
    import rospy
    rospy.init_node("preset_manager")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
