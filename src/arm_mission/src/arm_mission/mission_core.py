"""The mission controller without ROS: it owns the plant snapshot, applies the settings, runs the dry run and releases the go.

All ROS I/O goes through an `io` object (arm_mission/scripts/mission_control_node.py implements it; the tests use a fake):

    io.now()                      -> float seconds
    io.tool_axis()                -> (origin, axis) of the tool in base_link, or None
    io.marker_pose()              -> (position, quaternion) of the marker in base_link, or None when it was never seen
    io.detections()               -> (list of ObjectDetection-like, floor_source) or None before the first message
    io.get_param(name, default) / io.set_param(name, value)    names relative to the nodes: 'grasp_target/approach', 'object_detector/grasp_heights'
    io.grasp(dry_run)             -> (success, message, duration_s, deviation_m); blocking call of /grasp_sequencer/grasp with the grasp_target frame
    io.stop()                     -> (success, message)           /grasp_sequencer/stop
    io.run_state()                -> (state, message) of the sequencer; io.estop() -> True | False | None (unknown)
    io.spawn(fn)                  -> runs fn in a worker (a thread; the tests run it at once)

Nothing moves unless go() is accepted by logic.go_gate: a successful dry run of exactly the current settings, young enough, with no run in progress.
"""
import threading

from arm_mission import mission_logic as ml


class MissionController(object):
    def __init__(self, io, plant_class="Vigna", plan_valid_s=120.0, snapshot_valid_s=60.0):
        self.io = io
        self.plant_class = plant_class
        self.plan_valid_s = float(plan_valid_s)
        self.snapshot_valid_s = float(snapshot_valid_s)
        self.plants = []
        self.selected = None
        self.snapshot_time = None
        self.snapshot_height = None          # grasp height the snapshot was taken at (the plane of every plant depends on it)
        self.plan = ml.PlanRecord()
        self.last_go = None                  # (success, message) of the last real grasp request
        self.lock = threading.RLock()

    # ---- settings as they are on the parameter server --------------------------------------------------------------------
    def height(self):
        heights = self.io.get_param("object_detector/grasp_heights", {})
        try:
            return float(heights.get(self.plant_class, 0.0))
        except (AttributeError, TypeError, ValueError):
            return 0.0

    def settings(self):
        return {"approach": str(self.io.get_param("grasp_target/approach", "down")),
                "bias": [float(v) for v in self.io.get_param("grasp_target/bias", [0.0, 0.0, 0.0])][:3],
                "offset": [float(v) for v in self.io.get_param("grasp_target/offset", [0.0, 0.0, 0.0])][:3],
                "height": self.height()}

    def key(self):
        s = self.settings()
        return ml.settings_key(self.selected, s["offset"], s["approach"], s["bias"], s["height"])

    def snapshot_stale(self):
        if self.selected is None:
            return False
        if self.snapshot_time is None or self.io.now() - self.snapshot_time > self.snapshot_valid_s:
            return True
        return abs(self.height() - (self.snapshot_height if self.snapshot_height is not None else 0.0)) > 1e-4

    # ---- commands ---------------------------------------------------------------------------------------------------------
    def command(self, action, index=0, values=(), text=""):
        """Returns (success, message). Refused while a grasp is running, except stop."""
        with self.lock:
            if action not in ml.ACTIONS:
                return False, "unknown action '%s' (one of %s)" % (action, ", ".join(ml.ACTIONS))
            if action == "stop":
                return self.io.stop()
            state, _ = self.io.run_state()
            if state not in ml.RUN_IDLE_STATES:
                return False, "refused: a grasp is in progress (%s); stop it first" % state
            handler = getattr(self, "do_" + action)
            return handler(index=index, values=list(values), text=text)

    def do_refresh(self, **_):
        det = self.io.detections()
        axis, marker = self.io.tool_axis(), self.io.marker_pose()
        if det is None:
            return False, "no detections yet (/object_detector/detections)"
        if axis is None:
            return False, "no TF base_link -> end_effector"
        if marker is None:
            return False, "marker never seen: the plant offset is measured from it"
        plants = ml.rank_plants(det[0], axis[0], axis[1], self.plant_class, marker[0], marker[1])
        if not plants:
            self.plants, self.selected = [], None
            self.plan.clear()
            return False, "no %s in view" % self.plant_class
        self.plants, self.snapshot_time, self.snapshot_height = plants, self.io.now(), self.height()
        return self.do_select(index=0)

    def do_select(self, index=0, **_):
        if not 0 <= index < len(self.plants):
            return False, "no plant #%d in the list (%d plants): refresh" % (index, len(self.plants))
        self.selected = index
        off = self.plants[index]["offset"]
        self.io.set_param("grasp_target/offset", [float(v) for v in off])
        self.plan.clear()
        return True, "target #%d, offset %.3f %.3f %.3f from the marker" % (index, off[0], off[1], off[2])

    def do_set_approach(self, text="", **_):
        try:
            value = ml.normalize_approach(text)
        except ValueError as e:
            return False, str(e)
        self.io.set_param("grasp_target/approach", value)
        self.plan.clear()
        return True, "approach %s" % value

    def do_set_bias(self, values=(), **_):
        try:
            bias = ml.parse_triple(values, "bias")
        except ValueError as e:
            return False, str(e)
        self.io.set_param("grasp_target/bias", bias)
        self.plan.clear()
        return True, "bias %.1f %.1f %.1f mm" % tuple(1000 * v for v in bias)

    def do_set_height(self, values=(), **_):
        if len(values) != 1 or not 0.0 <= float(values[0]) <= 0.5:
            return False, "height needs one number between 0 and 0.5 m (pinch point above the floor)"
        heights = dict(self.io.get_param("object_detector/grasp_heights", {}))
        heights[self.plant_class] = float(values[0])
        self.io.set_param("object_detector/grasp_heights", heights)
        self.plan.clear()
        return True, "%s grasp height %.3f m: refresh the plant list" % (self.plant_class, float(values[0]))

    def do_dry_run(self, **_):
        if self.selected is None:
            return False, "no plant selected"
        if self.snapshot_stale():
            return False, "the plant list is stale: refresh"
        key = self.key()
        self.plan.begin(key, self.io.now())

        def work():
            try:
                ok, msg, duration, dev = self.io.grasp(True)
            except Exception as e:                       # a service that vanished must end 'planning', not freeze it
                ok, msg, duration, dev = False, "dry run call failed: %s" % e, 0.0, 0.0
            with self.lock:
                if self.plan.key == key:
                    self.plan.finish(ok, msg, duration, dev, self.io.now())
        self.io.spawn(work)
        return True, "dry run started"

    def do_go(self, **_):
        allowed, reason = ml.go_gate(self.plan, self.key(), self.io.now(), self.plan_valid_s, self.io.run_state()[0], self.io.estop(),
                                     self.snapshot_stale(), self.io.marker_pose() is not None)
        if not allowed:
            return False, "go refused: " + reason
        self.plan.clear()                                    # one dry run releases one go

        def work():
            try:
                ok, msg, _d, _v = self.io.grasp(False)
            except Exception as e:
                ok, msg = False, "grasp call failed: %s" % e
            self.last_go = (ok, msg)
        self.io.spawn(work)
        return True, "go: grasp requested"

    # ---- state for the front ends -----------------------------------------------------------------------------------------
    def state(self):
        with self.lock:
            run_state, run_msg = self.io.run_state()
            det = self.io.detections()
            marker = self.io.marker_pose() is not None
            key = self.key()
            stale = self.snapshot_stale()
            allowed, reason = ml.go_gate(self.plan, key, self.io.now(), self.plan_valid_s, run_state, self.io.estop(), stale, marker)
            live = None
            axis = self.io.tool_axis()
            if det is not None and axis is not None:
                live = sum(1 for d in det[0] if d.valid and d.class_name == self.plant_class)
            return {
                "plants": self.plants,
                "selected": -1 if self.selected is None else self.selected,
                "snapshot_age_s": None if self.snapshot_time is None else round(self.io.now() - self.snapshot_time, 1),
                "snapshot_stale": stale,
                "live_count": live,
                "marker_visible": marker,
                "floor_source": None if det is None else det[1],
                "plant_class": self.plant_class,
                "settings": self.settings(),
                "plan": {"state": self.plan.state, "message": self.plan.message, "duration_s": round(self.plan.duration_s, 1),
                         "deviation_mm": round(self.plan.deviation_mm, 1), "current": self.plan.key == key},
                "run": {"state": run_state, "message": run_msg},
                "estop": self.io.estop() is True,
                "go_enabled": allowed, "go_reason": reason,
                "busy": run_state not in ml.RUN_IDLE_STATES or self.plan.state == "planning",
            }
