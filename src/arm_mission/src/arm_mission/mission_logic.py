"""Pure logic of the mission control (no ROS): plant ranking, the marker-frame offset of a plant, the go gate and the state a front end shows.

Frames: all positions are in base_link. A plant is a detection of the configured class with a valid floor-plane position; it is ranked by its
distance from the TOOL AXIS (the line through the end effector along its z axis) measured on the plant's plane, which is what the operator means by
"the plant nearest the gripper". The grasp target is then marker + R_v^T (p - marker), R_v = the upright marker frame (target_offset.vertical_frame).
"""
import numpy as np

from arm_grasp import target_offset as to

# Commands of ~command (MissionCommand.action).
ACTIONS = ("refresh", "select", "set_approach", "set_bias", "set_height", "dry_run", "go", "stop")
RUN_IDLE_STATES = ("IDLE", "DONE", "ABORTED")        # sequencer states in which a new plan or run may start


def axis_distance(plant, origin, axis):
    """Distance (m) in the horizontal plane between a plant position and the point where the tool axis (origin + t * axis) crosses the plant's z.
    inf when the axis is horizontal (it never reaches the plane)."""
    plant, origin, axis = (np.asarray(v, float) for v in (plant, origin, axis))
    if abs(axis[2]) < 1e-6:
        return float("inf")
    g = origin + ((plant[2] - origin[2]) / axis[2]) * axis
    return float(np.hypot(*(plant[:2] - g[:2])))


def rank_plants(detections, origin, axis, plant_class, marker_position, marker_quaternion):
    """Valid detections of plant_class, nearest to the tool axis first.

    detections: iterable of objects with class_name, valid, confidence, position (x, y, z) as in arm_perception/ObjectDetection.
    Returns a list of dicts {index, confidence, position, axis_mm, offset} (offset = the marker-frame offset of the grasp target)."""
    R_t = to.vertical_frame(marker_quaternion).T
    marker = np.asarray(marker_position, float)
    rows = []
    for d in detections:
        if not d.valid or d.class_name != plant_class:
            continue
        p = np.array([d.position.x, d.position.y, d.position.z], float)
        rows.append((axis_distance(p, origin, axis), float(d.confidence), p))
    rows.sort(key=lambda r: r[0])
    return [{"index": i, "confidence": round(c, 3), "position": [round(float(v), 4) for v in p], "axis_mm": round(1000.0 * dist, 1),
             "offset": [round(float(v), 4) for v in R_t.dot(p - marker)]} for i, (dist, c, p) in enumerate(rows)]


def parse_triple(values, name):
    """Three finite numbers or ValueError."""
    if len(values) != 3:
        raise ValueError("%s needs 3 numbers (radial, tangential, vertical in metres), got %d" % (name, len(values)))
    v = [float(x) for x in values]
    if not all(np.isfinite(v)):
        raise ValueError("%s must be finite" % name)
    return v


def normalize_approach(text):
    """Validated approach setting as the string the grasp_target node accepts ('down', 'forward' or an angle such as '45')."""
    angle = to.parse_approach(text)
    t = str(text).strip().lower()
    if t in ("down", "forward", "side"):
        return "forward" if t == "side" else t
    return ("%g" % angle)


class PlanRecord(object):
    """The last dry run: what was planned (a key of the settings it was planned for) and its outcome."""

    def __init__(self):
        self.state = "none"          # none | planning | ok | failed
        self.message = ""
        self.duration_s = 0.0
        self.deviation_mm = 0.0
        self.key = None
        self.time = None

    def begin(self, key, now):
        self.state, self.message, self.key, self.time = "planning", "", key, now

    def finish(self, ok, message, duration_s, deviation_mm, now):
        self.state = "ok" if ok else "failed"
        self.message, self.duration_s, self.deviation_mm, self.time = message, float(duration_s), 1000.0 * float(deviation_mm), now

    def clear(self):
        self.__init__()


def settings_key(selected, offset, approach, bias, height):
    """Everything a plan depends on, rounded so that parameter echo noise does not invalidate it."""
    return (selected, tuple(round(float(v), 4) for v in offset), str(approach), tuple(round(float(v), 4) for v in bias), round(float(height), 4))


def go_gate(plan, key, now, valid_s, run_state, estop, snapshot_stale, marker_visible):
    """(allowed, reason): a real grasp is released only after a successful dry run of EXACTLY the current settings, not older than valid_s."""
    if estop is not False:
        return False, "e-stop engaged or its state unknown"
    if run_state not in RUN_IDLE_STATES:
        return False, "a grasp is in progress (%s)" % run_state
    if key[0] is None:
        return False, "no plant selected"
    if snapshot_stale:
        return False, "the plant list is stale: refresh"
    if not marker_visible:
        return False, "marker not seen"
    if plan.state == "planning":
        return False, "dry run in progress"
    if plan.state != "ok":
        return False, "run the dry run first"
    if plan.key != key:
        return False, "settings changed since the dry run"
    if plan.time is None or now - plan.time > valid_s:
        return False, "dry run older than %.0f s" % valid_s
    return True, ""


def plant_label(p):
    """One line per plant for a list: rank, confidence, distance from the gripper axis and position."""
    return "#%d  conf %.2f  %.0f mm from axis  (%.3f, %.3f, %.3f)" % (p["index"], p["confidence"], p["axis_mm"], p["position"][0], p["position"][1], p["position"][2])


def run_text(state):
    """Status line of a front end from the published state dict."""
    run = state.get("run", {})
    parts = ["grasp: %s" % run.get("state", "?")]
    if run.get("message"):
        parts.append(run["message"])
    if state.get("estop"):
        parts.insert(0, "E-STOP")
    return " - ".join(parts)


def plan_text(state):
    """Plan line of a front end from the published state dict."""
    plan = state.get("plan", {})
    st = plan.get("state", "none")
    if st == "ok":
        return "dry run ok: %.1f s, deviation %.1f mm" % (plan.get("duration_s", 0.0), plan.get("deviation_mm", 0.0)) + ("" if plan.get("current") else "  (settings changed: repeat)")
    if st == "failed":
        return "dry run FAILED: " + plan.get("message", "")
    if st == "planning":
        return "planning..."
    return "no dry run yet"
