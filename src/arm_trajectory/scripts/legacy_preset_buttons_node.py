#!/usr/bin/env python3
"""LEGACY, NOT ENABLED BY DEFAULT: the 3D version of the preset / e-stop / torque controls (interactive markers
floating in the rviz scene). Superseded by the 2D "Arm Control" panel of arm_gui (preset_manager_node.py); kept as a
quirky alternative. Start it explicitly: roslaunch arm_trajectory legacy_preset_markers.launch

Clickable preset-pose buttons in rviz (interactive markers on /preset_buttons).

One button per preset in config/presets.yaml (default, ready, pre_grasp, ...), plus a red E-STOP button.
  click a preset button   arms it (yellow, "click again to GO"); a second click within confirm_timeout
                          runs the move, so a stray click never moves the arm
  right-click menu        "Go to this pose" (starts the move at once: choosing it is already deliberate)
                          and "Overwrite with current pose" (saves where the arm is now into this slot,
                          in ~/.ros/arm_trajectory/presets.yaml)
  E-STOP button           calls /arm_interface/estop at once; its menu has "Reset e-stop" and "Cancel move"
  RESET E-STOP button     two clicks (like a preset); shows the reason if the driver refuses (a motor with a
                          latched hardware error must be rebooted first)
  TORQUE button           shows the torque state (from the drive state); two clicks flip it. Releasing torque
                          lets the arm SAG: it is asked twice, and refused while the e-stop is engaged
  MOTORS button           shows or hides the per-motor report next to the buttons: temperature and its limit,
                          load, voltage and hardware error of every motor, coloured by health
  status marker           green = e-stop released, red = ENGAGED, with the drive state and the motor health;
                          its menu has "Re-enable failed motors (reboot)" and "Cancel current move"
A move goes through exactly the path of stage3_joint_move.py: the joint-space trajectory service
(/generate_trajectory: velocity, acceleration and URDF limits) and the follow_joint_trajectory action, whose
executor refuses to move while the e-stop is engaged. Joint 6 (gripper) is not touched.
Add an InteractiveMarkers display on /preset_buttons/update in rviz (the shipped config has it).
"""
import os
import threading
import time
import xml.etree.ElementTree as ET

import yaml

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


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


class ConfirmGate:
    """Two-click confirmation: the first click on a name arms it, a second click on the SAME name within
    `timeout` seconds confirms. A click on another name re-arms that one; a late click re-arms."""

    def __init__(self, timeout=3.0):
        self.timeout = timeout
        self.armed = None
        self.armed_at = 0.0

    def click(self, name, now):
        if self.armed == name and now - self.armed_at <= self.timeout:
            self.armed = None
            return "confirmed"
        self.armed, self.armed_at = name, now
        return "armed"

    def expired(self, now):
        """The name whose arming just ran out (and clears it), or None."""
        if self.armed is not None and now - self.armed_at > self.timeout:
            name, self.armed = self.armed, None
            return name
        return None


def check_move(start, goal, max_delta):
    """None if the move is allowed, else the reason it is refused."""
    delta = max(abs(g - s) for g, s in zip(goal, start))
    if delta > max_delta:
        return "largest joint move %.2f rad exceeds max_delta %.2f rad" % (delta, max_delta)
    return None


def limits_from_urdf(urdf_xml):
    root = ET.fromstring(urdf_xml)
    return {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in root.iter("joint") if j.find("limit") is not None and j.get("type") in ("revolute", "prismatic")}


def worst_health(levels):
    """'OK', 'WARN' or 'ERROR' from diagnostic levels (0, 1, 2+); 'n/a' when there are none."""
    if not levels:
        return "n/a"
    worst = max(levels)
    return "OK" if worst == 0 else ("WARN" if worst == 1 else "ERROR")


def torque_state(drive):
    """'ON', 'OFF' or 'unknown' from the drive state (the driver has no separate torque topic)."""
    if drive is None:
        return "unknown"
    return "OFF" if drive == "TORQUE_OFF" else "ON"


def motor_rows(statuses):
    """One (text, level) row per motor from DiagnosticStatus-like tuples (name, level, message, values dict).
    None means no motor_status topic (the simulator without injected data, or a driver without monitoring)."""
    if statuses is None:
        return [("no motor data", 0)]
    rows = []
    for name, level, message, v in statuses:
        label = name.split("/")[-1]
        try:
            line = "%s  %3.0f/%.0f C  load %5.1f%%  %4.1f V" % (label, float(v["temperature_c"]), float(v["temperature_limit_c"]),
                                                                  float(v["load_percent"]), float(v["input_voltage_v"]))
        except (KeyError, ValueError):
            line = "%s  (no readings)" % label
        if message and message != "ok":
            line += "  " + message[:46]
        rows.append((line, level))
    return rows


def estop_status(estop, drive, levels, problem=""):
    """(description, colour name) of the e-stop status marker. estop is True/False/None (unknown).
    An unknown state is shown as engaged, because senders treat it that way (fail closed)."""
    if estop is None:
        head, colour = "e-stop: UNKNOWN (treated as engaged)", "red"
    elif estop:
        head, colour = "e-stop: ENGAGED", "red"
    else:
        head, colour = "e-stop: released", "green"
    text = "%s | drive: %s | motors: %s" % (head, drive or "n/a", worst_health(levels))
    if problem:
        text += " | " + problem
    return text, colour


# ---- ROS node ------------------------------------------------------------------------------------------------

IDLE, ARMED, MOVING, DONE, FAILED = "idle", "armed", "moving", "done", "failed"
COLORS = {IDLE: (0.25, 0.45, 0.85), ARMED: (0.95, 0.80, 0.10), MOVING: (0.20, 0.75, 0.30), DONE: (0.25, 0.45, 0.85), FAILED: (0.85, 0.20, 0.20)}
STATUS_COLORS = {"green": (0.15, 0.70, 0.25), "red": (0.85, 0.10, 0.10)}


class Node:
    def __init__(self):
        import actionlib
        import rospy
        from control_msgs.msg import FollowJointTrajectoryAction
        from interactive_markers.interactive_marker_server import InteractiveMarkerServer
        from interactive_markers.menu_handler import MenuHandler
        from sensor_msgs.msg import JointState
        from std_srvs.srv import Trigger
        from arm_trajectory.srv import GenerateTrajectory
        self.rospy, self.JointState, self.Trigger, self.GenerateTrajectory = rospy, JointState, Trigger, GenerateTrajectory

        limits = limits_from_urdf(rospy.get_param("robot_description"))
        shipped = rospy.get_param("~presets")
        self.user_file = os.path.expanduser(rospy.get_param("~presets_file", "~/.ros/arm_trajectory/presets.yaml"))
        self.book = PresetBook(shipped, load_user_file(self.user_file), limits)
        self.max_delta = float(rospy.get_param("~max_delta", 3.2))
        self.gate = ConfirmGate(float(rospy.get_param("~confirm_timeout", 3.0)))
        lay = rospy.get_param("~layout", {"x": 0.0, "y": 0.35, "z": 0.55, "spacing": 0.07})
        self.layout = lay
        self.state = {n: IDLE for n in self.book.order}
        self.text = {n: "" for n in self.book.order}
        self.busy = threading.Lock()
        self.client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
        self.latest = None
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        self.server = InteractiveMarkerServer("preset_buttons")
        self.menus = {}
        for i, name in enumerate(self.book.order):
            self.add_button(name, i, MenuHandler)
        n = len(self.book.order)
        self.estop_state, self.drive_state, self.levels, self.problem = None, None, [], ""
        self.reset_text = ""
        self.MenuHandler = MenuHandler
        from std_msgs.msg import Bool, String
        from diagnostic_msgs.msg import DiagnosticArray
        rospy.Subscriber("/arm_interface/estop_state", Bool, self.on_estop_state, queue_size=1)
        rospy.Subscriber("/arm_interface/drive_state", String, self.on_drive_state, queue_size=1)
        rospy.Subscriber("/arm_interface/motor_status", DiagnosticArray, self.on_motor_status, queue_size=1)
        self.add_status(n, MenuHandler)
        self.add_estop(n + 1, MenuHandler)
        self.add_reset(n + 2, MenuHandler)
        self.statuses = None
        self.torque_text = ""
        self.show_report = True
        self.add_torque(n + 3)
        self.add_motors_button(n + 4)
        from visualization_msgs.msg import MarkerArray
        self.report_pub = rospy.Publisher("/preset_buttons/motor_report", MarkerArray, queue_size=1)
        self.report_dirty = True
        self.status_dirty = False
        self.server.applyChanges()
        rospy.Timer(rospy.Duration(0.5), self.on_timer)
        rospy.loginfo("preset buttons: %s (move limit %.1f rad, confirm %.1f s); your saved poses: %s",
                      ", ".join(self.book.order), self.max_delta, self.gate.timeout, self.user_file)

    # --- markers -----------------------------------------------------------------------------------------
    def make_marker(self, name, index, description, color, size=(0.05, 0.14, 0.035)):
        from interactive_markers.interactive_marker_server import InteractiveMarker
        from visualization_msgs.msg import InteractiveMarkerControl, Marker
        m = InteractiveMarker()
        m.header.frame_id = "base_link"
        m.name = name
        m.description = description
        m.scale = 0.15
        m.pose.position.x = float(self.layout["x"])
        m.pose.position.y = float(self.layout["y"])
        m.pose.position.z = float(self.layout["z"]) - index * float(self.layout["spacing"])
        m.pose.orientation.w = 1.0
        cube = Marker(type=Marker.CUBE)
        cube.scale.x, cube.scale.y, cube.scale.z = size
        cube.color.r, cube.color.g, cube.color.b, cube.color.a = color[0], color[1], color[2], 1.0
        c = InteractiveMarkerControl()
        c.interaction_mode = InteractiveMarkerControl.BUTTON
        c.always_visible = True
        c.markers.append(cube)
        m.controls.append(c)
        return m

    def add_button(self, name, index, MenuHandler):
        marker = self.make_marker("preset_" + name, index, self.describe(name), COLORS[IDLE])
        menu = MenuHandler()
        menu.insert("Go to this pose", callback=lambda fb, n=name: self.start_move(n))
        menu.insert("Overwrite with current pose", callback=lambda fb, n=name: self.overwrite(n))
        self.server.insert(marker, lambda fb, n=name: self.on_feedback(fb, n))
        menu.apply(self.server, marker.name)
        self.menus[name] = menu

    def add_estop(self, index, MenuHandler):
        marker = self.make_marker("preset_ESTOP", index, "E-STOP (click)", (0.9, 0.05, 0.05), size=(0.05, 0.14, 0.05))
        menu = MenuHandler()
        menu.insert("Reset e-stop", callback=lambda fb: self.call_trigger("/arm_interface/estop_reset"))
        menu.insert("Cancel current move (arm holds)", callback=lambda fb: self.client.cancel_all_goals())
        self.server.insert(marker, self.on_estop_feedback)
        menu.apply(self.server, marker.name)
        self.menus["ESTOP"] = menu

    def add_torque(self, index):
        self.torque_index = index
        self.torque_dirty = False
        self.refresh_torque()

    def torque_description(self):
        t = torque_state(self.drive_state)
        if self.gate.armed == "TORQUE":
            return ("RELEASE TORQUE? the arm will SAG - click again" if t == "ON" else "ENABLE TORQUE? it holds the present pose - click again"), COLORS[ARMED]
        base = {"ON": "TORQUE: ON - click twice to release", "OFF": "TORQUE: OFF - click twice to enable", "unknown": "TORQUE: unknown"}[t]
        if self.torque_text:
            base += " | " + self.torque_text
        return base, ((0.15, 0.55, 0.60) if t == "ON" else (0.45, 0.45, 0.45))

    def refresh_torque(self):
        desc, color = self.torque_description()
        marker = self.make_marker("preset_TORQUE", self.torque_index, desc, color, size=(0.05, 0.14, 0.035))
        self.server.insert(marker, self.on_torque_feedback)
        self.server.applyChanges()

    def on_torque_feedback(self, fb):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if self.arm("TORQUE") == "armed":
            self.torque_text = ""
            self.refresh_torque()
        else:
            threading.Thread(target=self.do_torque, daemon=True).start()

    def do_torque(self):
        from std_srvs.srv import SetBool
        want = torque_state(self.drive_state) != "ON"          # flip: ON -> release, OFF -> enable
        if torque_state(self.drive_state) == "unknown":
            self.torque_text = "state unknown: not changing torque"
            self.refresh_torque()
            return
        try:
            self.rospy.wait_for_service("/arm_interface/set_torque", timeout=1.0)
            res = self.rospy.ServiceProxy("/arm_interface/set_torque", SetBool)(want)
            self.torque_text = "" if res.success else "REFUSED: " + res.message[:70]
            (self.rospy.logwarn if res.success else self.rospy.logerr)("set_torque(%s): %s", want, res.message)
        except Exception as e:
            self.torque_text = "failed: %s" % str(e)[:50]
            self.rospy.logerr("set_torque failed: %s", e)
        self.refresh_torque()

    def add_motors_button(self, index):
        self.motors_index = index
        self.refresh_motors_button()

    def refresh_motors_button(self):
        desc = "MOTOR REPORT: shown - click to hide" if self.show_report else "MOTOR REPORT: hidden - click to show"
        marker = self.make_marker("preset_MOTORS", self.motors_index, desc, (0.35, 0.35, 0.65), size=(0.05, 0.14, 0.035))
        self.server.insert(marker, self.on_motors_feedback)
        self.server.applyChanges()

    def on_motors_feedback(self, fb):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if fb.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.show_report = not self.show_report
            self.report_dirty = True
            self.refresh_motors_button()

    def publish_report(self):
        from visualization_msgs.msg import Marker, MarkerArray
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        if self.show_report:
            rows = [("motor status", 0)] + motor_rows(self.statuses)
            lay = self.layout
            palette = {0: (0.85, 0.95, 0.85), 1: (1.0, 0.85, 0.2), 2: (1.0, 0.25, 0.25)}
            for i, (text, level) in enumerate(rows):
                m = Marker(ns="motor_report", id=i, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
                m.header.frame_id = "base_link"
                m.pose.position.x = float(lay["x"])
                m.pose.position.y = float(lay["y"]) + 0.22
                m.pose.position.z = float(lay["z"]) - i * 0.045
                m.pose.orientation.w = 1.0
                m.scale.z = 0.032
                c = palette.get(level, palette[2])
                m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 1.0
                m.text = text
                arr.markers.append(m)
        self.report_pub.publish(arr)

    def add_status(self, index, MenuHandler):
        text, colour = estop_status(self.estop_state, self.drive_state, self.levels)
        marker = self.make_marker("preset_ESTOP_STATUS", index, text, STATUS_COLORS[colour], size=(0.05, 0.14, 0.02))
        menu = MenuHandler()
        menu.insert("Re-enable failed motors (reboot)", callback=lambda fb: self.reenable_motors())
        menu.insert("Cancel current move (arm holds)", callback=lambda fb: self.client.cancel_all_goals())
        self.server.insert(marker, lambda fb: None)
        menu.apply(self.server, marker.name)
        self.menus["ESTOP_STATUS"] = menu
        self.status_index = index

    def add_reset(self, index, MenuHandler):
        marker = self.make_marker("preset_ESTOP_RESET", index, "RESET E-STOP", (0.95, 0.55, 0.05), size=(0.05, 0.14, 0.035))
        self.server.insert(marker, self.on_reset_feedback)
        self.reset_index = index

    def refresh_reset(self):
        desc = "RESET E-STOP"
        color = (0.95, 0.55, 0.05)
        if self.gate.armed == "ESTOP_RESET":
            desc, color = "RESET E-STOP - click again to RELEASE", COLORS[ARMED]
        elif self.reset_text:
            desc = "RESET E-STOP - " + self.reset_text
        marker = self.make_marker("preset_ESTOP_RESET", self.reset_index, desc, color, size=(0.05, 0.14, 0.035))
        self.server.insert(marker, self.on_reset_feedback)
        self.server.applyChanges()

    def on_reset_feedback(self, fb):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        if self.arm("ESTOP_RESET") == "armed":
            self.reset_text = ""
            self.refresh_reset()
        else:
            threading.Thread(target=self.do_reset, daemon=True).start()

    def do_reset(self):
        try:
            self.rospy.wait_for_service("/arm_interface/estop_reset", timeout=1.0)
            res = self.rospy.ServiceProxy("/arm_interface/estop_reset", self.Trigger)()
            self.reset_text = "released" if res.success else "REFUSED: " + res.message[:90]
            (self.rospy.logwarn if res.success else self.rospy.logerr)("estop_reset: %s", res.message)
        except Exception as e:
            self.reset_text = "failed: %s" % str(e)[:60]
            self.rospy.logerr("estop_reset failed: %s", e)
        self.refresh_reset()

    def reenable_motors(self):
        try:
            self.rospy.wait_for_service("/arm_interface/reenable_failed_motors", timeout=1.0)
            res = self.rospy.ServiceProxy("/arm_interface/reenable_failed_motors", self.Trigger)()
            self.problem = "" if res.success else res.message[:60]
            self.rospy.logwarn("reenable_failed_motors: %s", res.message)
        except Exception as e:
            self.problem = "re-enable failed: %s" % str(e)[:50]
            self.rospy.logerr("reenable_failed_motors failed: %s", e)
        self.status_dirty = True

    def on_estop_state(self, msg):
        if msg.data != self.estop_state:
            self.estop_state = msg.data
            self.status_dirty = True

    def on_drive_state(self, msg):
        if msg.data != self.drive_state:
            self.drive_state = msg.data
            self.status_dirty = True
            self.torque_dirty = True

    def on_motor_status(self, msg):
        self.statuses = [(s.name, s.level, s.message, {kv.key: kv.value for kv in s.values}) for s in msg.status]
        self.report_dirty = True
        levels = [s.level for s in msg.status]
        problems = [s.message for s in msg.status if s.level >= 2]
        problem = ("%s: %s" % (msg.status[[s.level for s in msg.status].index(2)].name.split("/")[-1], problems[0][:50])) if problems else ""
        if levels != self.levels or problem != self.problem:
            self.levels, self.problem = levels, problem
            self.status_dirty = True

    def refresh_status(self):
        text, colour = estop_status(self.estop_state, self.drive_state, self.levels, self.problem)
        marker = self.make_marker("preset_ESTOP_STATUS", self.status_index, text, STATUS_COLORS[colour], size=(0.05, 0.14, 0.02))
        self.server.insert(marker, lambda fb: None)
        self.menus["ESTOP_STATUS"].apply(self.server, marker.name)
        self.server.applyChanges()

    def describe(self, name):
        s = self.state[name]
        base = name
        if s == ARMED:
            return "%s - click again to GO" % base
        if s == MOVING:
            return "%s - moving..." % base
        if s == FAILED:
            return "%s - %s" % (base, self.text[name] or "failed")
        if s == DONE:
            return "%s - reached" % base
        return base

    def refresh(self, name):
        old = self.server.get("preset_" + name)
        idx = self.book.order.index(name)
        marker = self.make_marker("preset_" + name, idx, self.describe(name), COLORS[self.state[name]])
        self.server.insert(marker, lambda fb, n=name: self.on_feedback(fb, n))
        self.menus[name].apply(self.server, marker.name)
        self.server.applyChanges()

    # --- events ------------------------------------------------------------------------------------------
    def on_joints(self, msg):
        self.latest = dict(zip(msg.name, msg.position))

    def on_feedback(self, fb, name):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        result = self.arm(name)
        if result == "armed":
            self.state[name] = ARMED
            self.refresh(name)
        else:
            self.start_move(name)

    def on_estop_feedback(self, fb):
        from visualization_msgs.msg import InteractiveMarkerFeedback
        if fb.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self.call_trigger("/arm_interface/estop")

    def arm(self, name):
        """Gate click; when it arms `name` and another button was armed, redraw that one as disarmed."""
        previous = self.gate.armed
        result = self.gate.click(name, time.time())
        if result == "armed" and previous and previous != name:
            self.redraw(previous)
        return result

    def redraw(self, name):
        if name in self.state:
            self.state[name] = IDLE
            self.refresh(name)
        elif name == "ESTOP_RESET":
            self.refresh_reset()
        elif name == "TORQUE":
            self.refresh_torque()

    def on_timer(self, _):
        name = self.gate.expired(time.time())
        if name == "ESTOP_RESET":
            self.refresh_reset()
        elif name is not None and self.state.get(name) == ARMED:
            self.state[name] = IDLE
            self.refresh(name)
        if name == "TORQUE":
            self.refresh_torque()
        if self.status_dirty:
            self.status_dirty = False
            self.refresh_status()
        if self.torque_dirty:
            self.torque_dirty = False
            self.refresh_torque()
        if self.report_dirty:
            self.report_dirty = False
            self.publish_report()

    def call_trigger(self, service):
        try:
            self.rospy.wait_for_service(service, timeout=1.0)
            res = self.rospy.ServiceProxy(service, self.Trigger)()
            self.rospy.logwarn("%s: %s", service, res.message)
        except Exception as e:
            self.rospy.logerr("%s failed: %s", service, e)

    def overwrite(self, name):
        if self.latest is None or any(j not in self.latest for j in JOINTS):
            self.rospy.logerr("no joint states yet: cannot save the current pose")
            return
        joints = [self.latest[j] for j in JOINTS]
        try:
            self.book.set_pose(name, joints, "saved from rviz")
        except ValueError as e:
            self.rospy.logerr("not saved: %s", e)
            return
        save_user_pose(self.user_file, name, joints)
        self.rospy.logwarn("preset '%s' overwritten with the current pose %s (saved in %s)", name, [round(q, 3) for q in joints], self.user_file)
        self.state[name] = DONE
        self.refresh(name)

    def start_move(self, name):
        if not self.busy.acquire(False):
            self.rospy.logwarn("a move is already running: ignoring '%s'", name)
            return
        threading.Thread(target=self.run_move, args=(name,), daemon=True).start()

    def run_move(self, name):
        from control_msgs.msg import FollowJointTrajectoryGoal
        from arm_trajectory.srv import GenerateTrajectoryRequest
        rospy = self.rospy
        try:
            goal = self.book.poses[name]
            if self.latest is None or any(j not in self.latest for j in JOINTS):
                raise RuntimeError("no joint states yet")
            start = [self.latest[j] for j in JOINTS]
            why = check_move(start, goal, self.max_delta)
            if why:
                raise RuntimeError(why)
            self.state[name], self.text[name] = MOVING, ""
            self.refresh(name)
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
            self.state[name] = DONE
            rospy.loginfo("preset '%s' reached", name)
        except Exception as e:
            self.state[name], self.text[name] = FAILED, str(e)[:70]
            rospy.logerr("preset '%s' not executed: %s", name, e)
        finally:
            self.refresh(name)
            self.busy.release()


def main():
    import rospy
    rospy.init_node("preset_buttons")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
