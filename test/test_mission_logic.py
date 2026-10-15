"""Unit tests of mission_logic and mission_core (no ROS master: a fake io)."""
import os
import sys
import unittest
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "arm_grasp", "src"))

from arm_mission import mission_core as mc          # noqa: E402
from arm_mission import mission_logic as ml         # noqa: E402

IDENTITY_Q = [0.0, 0.0, 0.0, 1.0]


def det(x, y, z=0.09, cls="Vigna", conf=0.9, valid=True):
    return NS(class_name=cls, valid=valid, confidence=conf, position=NS(x=x, y=y, z=z))


class FakeIO(object):
    def __init__(self):
        self.t = 100.0
        self.params = {"grasp_target/approach": "45", "grasp_target/bias": [-0.01, 0.01, 0.0], "grasp_target/offset": [0.0, 0.0, 0.0],
                       "object_detector/grasp_heights": {"Vigna": 0.09}}
        self.axis = (np.array([0.0, 0.0, 0.3]), np.array([0.0, 0.0, -1.0]))      # straight down above the origin
        self.marker = ([0.4, 0.0, -0.6], IDENTITY_Q)
        self.det = ([det(0.30, 0.10), det(0.02, 0.01), det(0.10, -0.2), det(0.0, 0.0, cls="Sorgo_Row"), det(0.0, 0.0, valid=False)], "marker_0")
        self.state = ("IDLE", "")
        self.estop_value = False
        self.grasp_calls, self.stop_calls = [], 0
        self.grasp_result = (True, "plan feasible", 20.0, 0.002)

    def now(self): return self.t
    def tool_axis(self): return self.axis
    def marker_pose(self): return self.marker
    def detections(self): return self.det
    def get_param(self, name, default): return self.params.get(name, default)
    def set_param(self, name, value): self.params[name] = value
    def grasp(self, dry_run):
        self.grasp_calls.append(dry_run)
        return self.grasp_result
    def stop(self):
        self.stop_calls += 1
        return True, "stopped"
    def run_state(self): return self.state
    def estop(self): return self.estop_value
    def spawn(self, fn): fn()


class RankTest(unittest.TestCase):
    def test_nearest_to_the_tool_axis_first_and_other_classes_and_invalid_dropped(self):
        io = FakeIO()
        plants = ml.rank_plants(io.det[0], io.axis[0], io.axis[1], "Vigna", io.marker[0], io.marker[1])
        self.assertEqual(len(plants), 3)
        self.assertEqual([p["index"] for p in plants], [0, 1, 2])
        self.assertAlmostEqual(plants[0]["position"][0], 0.02)
        self.assertLess(plants[0]["axis_mm"], plants[1]["axis_mm"])
        self.assertLess(plants[1]["axis_mm"], plants[2]["axis_mm"])

    def test_axis_distance_follows_a_tilted_axis_to_the_plant_plane(self):
        origin, axis = np.array([0.0, 0.0, 0.5]), np.array([1.0, 0.0, -1.0]) / np.sqrt(2.0)
        # the axis reaches z = 0.1 at x = 0.4
        self.assertAlmostEqual(ml.axis_distance([0.4, 0.05, 0.1], origin, axis), 0.05)

    def test_a_horizontal_axis_never_reaches_the_plane(self):
        self.assertEqual(ml.axis_distance([0.3, 0, 0.1], [0, 0, 0.3], [1, 0, 0]), float("inf"))

    def test_offset_is_in_the_upright_marker_frame(self):
        # marker rotated +90 deg about z: its x points along base +y, so a plant 0.1 m along base +y is at offset x = +0.1
        s = np.sin(np.pi / 4)
        plants = ml.rank_plants([det(0.4, 0.1, -0.5)], [0, 0, 0.3], [0, 0, -1], "Vigna", [0.4, 0.0, -0.6], [0, 0, s, s])
        o = plants[0]["offset"]
        self.assertAlmostEqual(o[0], 0.1, places=3)
        self.assertAlmostEqual(o[1], 0.0, places=3)
        self.assertAlmostEqual(o[2], 0.1, places=3)

    def test_the_offset_added_back_to_the_marker_gives_the_plant(self):
        io = FakeIO()
        p = ml.rank_plants(io.det[0], io.axis[0], io.axis[1], "Vigna", io.marker[0], io.marker[1])[0]
        back = np.array(io.marker[0]) + np.array(p["offset"])           # identity marker: base axes
        np.testing.assert_allclose(back, p["position"], atol=1e-3)


class ParseTest(unittest.TestCase):
    def test_triples(self):
        self.assertEqual(ml.parse_triple([0.1, 0, -0.2], "bias"), [0.1, 0.0, -0.2])
        with self.assertRaises(ValueError): ml.parse_triple([0.1, 0.2], "bias")
        with self.assertRaises(ValueError): ml.parse_triple([0.1, float("nan"), 0], "bias")

    def test_approach_normalisation(self):
        self.assertEqual(ml.normalize_approach("down"), "down")
        self.assertEqual(ml.normalize_approach(" Side "), "forward")
        self.assertEqual(ml.normalize_approach("45"), "45")
        self.assertEqual(ml.normalize_approach("22.5"), "22.5")
        with self.assertRaises(ValueError): ml.normalize_approach("120")
        with self.assertRaises(ValueError): ml.normalize_approach("sideways")


class GateTest(unittest.TestCase):
    def ok_plan(self, key, t=100.0):
        p = ml.PlanRecord()
        p.begin(key, t - 1.0)
        p.finish(True, "ok", 20.0, 0.002, t)
        return p

    def test_released_only_for_a_fresh_dry_run_of_the_same_key(self):
        key = ml.settings_key(0, [0.1, 0, 0.1], "45", [0, 0, 0], 0.09)
        plan = self.ok_plan(key)
        self.assertEqual(ml.go_gate(plan, key, 110.0, 120.0, "IDLE", False, False, True), (True, ""))
        other = ml.settings_key(0, [0.1, 0, 0.1], "45", [0, 0.01, 0], 0.09)
        self.assertFalse(ml.go_gate(plan, other, 110.0, 120.0, "IDLE", False, False, True)[0])
        self.assertFalse(ml.go_gate(plan, key, 300.0, 120.0, "IDLE", False, False, True)[0])

    def test_refused_for_estop_unknown_estop_running_stale_list_no_marker_and_failed_plan(self):
        key = ml.settings_key(0, [0, 0, 0], "45", [0, 0, 0], 0.09)
        plan = self.ok_plan(key)
        for args in [("IDLE", True, False, True), ("IDLE", None, False, True), ("EXECUTING", False, False, True), ("IDLE", False, True, True), ("IDLE", False, False, False)]:
            self.assertFalse(ml.go_gate(plan, key, 101.0, 120.0, args[0], args[1], args[2], args[3])[0], args)
        bad = ml.PlanRecord(); bad.begin(key, 99.0); bad.finish(False, "unreachable", 0, 0, 100.0)
        self.assertFalse(ml.go_gate(bad, key, 101.0, 120.0, "IDLE", False, False, True)[0])
        self.assertFalse(ml.go_gate(ml.PlanRecord(), key, 101.0, 120.0, "IDLE", False, False, True)[0])
        self.assertFalse(ml.go_gate(plan, ml.settings_key(None, [0, 0, 0], "45", [0, 0, 0], 0.09), 101.0, 120.0, "IDLE", False, False, True)[0])


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.io = FakeIO()
        self.c = mc.MissionController(self.io, plan_valid_s=120.0, snapshot_valid_s=60.0)

    def test_refresh_selects_the_nearest_and_sets_the_target_offset(self):
        ok, _ = self.c.command("refresh")
        self.assertTrue(ok)
        self.assertEqual(self.c.selected, 0)
        self.assertEqual(len(self.c.plants), 3)
        np.testing.assert_allclose(self.io.params["grasp_target/offset"], self.c.plants[0]["offset"])

    def test_refresh_without_plants_markers_or_detections_fails_and_says_why(self):
        self.io.det = ([det(0, 0, cls="Sorgo_Row")], "param")
        ok, msg = self.c.command("refresh")
        self.assertFalse(ok); self.assertIn("no Vigna", msg); self.assertIsNone(self.c.selected)
        self.io.marker = None
        self.assertIn("marker", self.c.command("refresh")[1])
        self.io.det = None
        self.assertIn("no detections", self.c.command("refresh")[1])
        self.io.det, self.io.axis = ([det(0, 0)], "x"), None
        self.assertIn("end_effector", self.c.command("refresh")[1])

    def test_select_out_of_range_is_refused_and_a_valid_select_changes_the_offset(self):
        self.c.command("refresh")
        self.assertFalse(self.c.command("select", index=7)[0])
        self.assertTrue(self.c.command("select", index=2)[0])
        np.testing.assert_allclose(self.io.params["grasp_target/offset"], self.c.plants[2]["offset"])

    def test_settings_are_validated_before_they_reach_the_parameter_server(self):
        self.assertFalse(self.c.command("set_approach", text="200")[0])
        self.assertEqual(self.io.params["grasp_target/approach"], "45")
        self.assertTrue(self.c.command("set_approach", text="60")[0])
        self.assertEqual(self.io.params["grasp_target/approach"], "60")
        self.assertFalse(self.c.command("set_bias", values=[0.1])[0])
        self.assertTrue(self.c.command("set_bias", values=[-0.02, 0.0, 0.01])[0])
        self.assertEqual(self.io.params["grasp_target/bias"], [-0.02, 0.0, 0.01])
        self.assertFalse(self.c.command("set_height", values=[0.9])[0])
        self.assertTrue(self.c.command("set_height", values=[0.05])[0])
        self.assertEqual(self.io.params["object_detector/grasp_heights"], {"Vigna": 0.05})

    def test_set_height_keeps_the_other_classes(self):
        self.io.params["object_detector/grasp_heights"] = {"Vigna": 0.09, "Cup": 0.03}
        self.c.command("set_height", values=[0.06])
        self.assertEqual(self.io.params["object_detector/grasp_heights"], {"Vigna": 0.06, "Cup": 0.03})

    def test_go_needs_a_dry_run_of_the_current_settings(self):
        self.c.command("refresh")
        ok, msg = self.c.command("go")
        self.assertFalse(ok); self.assertIn("dry run", msg); self.assertEqual(self.io.grasp_calls, [])
        self.assertTrue(self.c.command("dry_run")[0])
        self.assertEqual(self.io.grasp_calls, [True])
        self.assertTrue(self.c.state()["go_enabled"])
        self.assertTrue(self.c.command("go")[0])
        self.assertEqual(self.io.grasp_calls, [True, False])

    def test_one_dry_run_releases_one_go(self):
        self.c.command("refresh"); self.c.command("dry_run"); self.c.command("go")
        self.assertFalse(self.c.command("go")[0])
        self.assertEqual(self.io.grasp_calls, [True, False])

    def test_any_setting_change_after_the_dry_run_blocks_go(self):
        for change in (("set_bias", dict(values=[0.0, 0.0, 0.0])), ("set_approach", dict(text="90")), ("select", dict(index=1)), ("set_height", dict(values=[0.05]))):
            io = FakeIO(); c = mc.MissionController(io)
            c.command("refresh"); c.command("dry_run")
            self.assertTrue(c.command(change[0], **change[1])[0])
            self.assertFalse(c.command("go")[0], change)
            self.assertEqual(io.grasp_calls, [True])

    def test_a_failed_dry_run_blocks_go_and_reports(self):
        self.io.grasp_result = (False, "plan not feasible: PRE unreachable", 0.0, 0.0)
        self.c.command("refresh"); self.c.command("dry_run")
        st = self.c.state()
        self.assertEqual(st["plan"]["state"], "failed"); self.assertIn("unreachable", st["plan"]["message"]); self.assertFalse(st["go_enabled"])
        self.assertFalse(self.c.command("go")[0])

    def test_a_dry_run_that_raises_ends_in_failed_not_planning(self):
        def boom(dry): raise RuntimeError("service vanished")
        self.io.grasp = boom
        self.c.command("refresh"); self.c.command("dry_run")
        self.assertEqual(self.c.state()["plan"]["state"], "failed")

    def test_a_height_change_makes_the_snapshot_stale_until_refresh(self):
        self.c.command("refresh")
        self.io.params["object_detector/grasp_heights"] = {"Vigna": 0.05}        # changed from outside, e.g. rosparam set
        self.assertTrue(self.c.state()["snapshot_stale"])
        self.assertFalse(self.c.command("dry_run")[0])
        self.c.command("refresh")
        self.assertFalse(self.c.state()["snapshot_stale"])

    def test_an_old_snapshot_is_stale(self):
        self.c.command("refresh")
        self.io.t += 61.0
        self.assertFalse(self.c.command("dry_run")[0])

    def test_a_dry_run_expires(self):
        self.c.snapshot_valid_s = 1e9
        self.c.command("refresh"); self.c.command("dry_run")
        self.io.t += 121.0
        ok, msg = self.c.command("go")
        self.assertFalse(ok); self.assertIn("older", msg)

    def test_estop_blocks_go_and_unknown_estop_too(self):
        self.c.command("refresh"); self.c.command("dry_run")
        for v in (True, None):
            self.io.estop_value = v
            self.assertFalse(self.c.command("go")[0])
        self.io.estop_value = False
        self.assertTrue(self.c.command("go")[0])

    def test_while_a_grasp_runs_everything_but_stop_is_refused(self):
        self.c.command("refresh"); self.c.command("dry_run")
        self.io.state = ("EXECUTING", "descending")
        for a in ("refresh", "select", "set_bias", "set_approach", "set_height", "dry_run", "go"):
            self.assertFalse(self.c.command(a, values=[0, 0, 0], text="45")[0], a)
        self.assertTrue(self.c.state()["busy"])
        self.assertTrue(self.c.command("stop")[0]); self.assertEqual(self.io.stop_calls, 1)

    def test_unknown_action(self):
        self.assertFalse(self.c.command("fly")[0])

    def test_state_is_json_serialisable_and_complete(self):
        import json
        self.c.command("refresh")
        st = self.c.state()
        json.dumps(st)
        for k in ("plants", "selected", "settings", "plan", "run", "go_enabled", "go_reason", "busy", "estop", "marker_visible", "snapshot_stale", "live_count"):
            self.assertIn(k, st)
        self.assertEqual(st["live_count"], 3)
        self.assertEqual(st["settings"]["approach"], "45")

    def test_state_before_any_data(self):
        self.io.det, self.io.marker = None, None
        st = mc.MissionController(self.io).state()
        self.assertEqual(st["selected"], -1); self.assertIsNone(st["live_count"]); self.assertFalse(st["marker_visible"]); self.assertFalse(st["go_enabled"])


class TextTest(unittest.TestCase):
    def test_texts(self):
        st = {"run": {"state": "IDLE", "message": "dry run ok"}, "estop": True, "plan": {"state": "ok", "duration_s": 20.0, "deviation_mm": 1.5, "current": False}}
        self.assertEqual(ml.run_text(st), "E-STOP - grasp: IDLE - dry run ok")
        self.assertIn("repeat", ml.plan_text(st))
        self.assertEqual(ml.plan_text({}), "no dry run yet")
        self.assertIn("FAILED", ml.plan_text({"plan": {"state": "failed", "message": "x"}}))
        self.assertIn("mm from axis", ml.plant_label({"index": 0, "confidence": 0.9, "axis_mm": 12.0, "position": [0.1, 0.2, 0.3]}))


if __name__ == "__main__":
    unittest.main()
