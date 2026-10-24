"""LEGACY (3D marker version, not enabled by default; run by hand with python3 -m unittest). Unit tests of the pure logic of preset_buttons_node (no ROS master): preset book and validation, the
two-click confirmation gate, the move-size guard, the user-file round trip."""
import importlib.util
import os
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("preset_buttons_node", os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts", "legacy_preset_buttons_node.py"))
pb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pb)

LIMITS = {"joint_1": (-2.745, 2.736), "joint_2": (0.0, 3.14), "joint_3": (-2.22, 0.0), "joint_4": (0.0, 4.505), "joint_5": (-3.62, 0.0)}
SHIPPED = {"default": {"joints": [0.0, 0.02, -0.16, 0.02, -0.02], "note": "rest"},
           "ready": {"joints": [0.15, 0.25, -0.25, 0.25, -0.25]}}


class PresetBookTest(unittest.TestCase):
    def test_shipped_presets_are_loaded_in_order(self):
        book = pb.PresetBook(SHIPPED, limits=LIMITS)
        self.assertEqual(book.order, ["default", "ready"])
        self.assertEqual(book.poses["ready"], [0.15, 0.25, -0.25, 0.25, -0.25])

    def test_user_poses_override_by_name_and_new_ones_are_appended(self):
        user = {"ready": {"joints": [0.2, 0.3, -0.3, 0.3, -0.3]}, "mine": {"joints": [0.0, 0.5, -0.5, 0.5, -0.5]}}
        book = pb.PresetBook(SHIPPED, user, LIMITS)
        self.assertEqual(book.poses["ready"][0], 0.2)
        self.assertEqual(book.order, ["default", "ready", "mine"])

    def test_a_pose_outside_the_urdf_limits_is_refused(self):
        with self.assertRaises(ValueError) as cm:
            pb.PresetBook({"bad": {"joints": [0.0, -0.5, -0.1, 0.1, -0.1]}}, limits=LIMITS)    # joint_2 lower limit is 0
        self.assertIn("joint_2", str(cm.exception))

    def test_wrong_number_of_joints_is_refused(self):
        with self.assertRaises(ValueError):
            pb.PresetBook({"bad": {"joints": [0.0, 0.1, -0.1]}}, limits=LIMITS)

    def test_set_pose_validates_and_adds_a_slot(self):
        book = pb.PresetBook(SHIPPED, limits=LIMITS)
        book.set_pose("new", [0.1, 0.2, -0.2, 0.2, -0.2])
        self.assertIn("new", book.order)
        with self.assertRaises(ValueError):
            book.set_pose("new", [9.0, 0.2, -0.2, 0.2, -0.2])
        self.assertEqual(book.poses["new"][0], 0.1)               # the refused pose did not overwrite it


class UserFileTest(unittest.TestCase):
    def test_save_merges_and_round_trips(self):
        path = os.path.join(tempfile.mkdtemp(), "sub", "presets.yaml")
        self.assertEqual(pb.load_user_file(path), {})              # missing file is fine
        pb.save_user_pose(path, "pre_grasp", [0.1234567, 0.3, -0.3, 0.3, -0.3])
        pb.save_user_pose(path, "other", [0.0, 0.1, -0.1, 0.1, -0.1])
        data = pb.load_user_file(path)
        self.assertEqual(sorted(data), ["other", "pre_grasp"])
        self.assertEqual(data["pre_grasp"]["joints"][0], 0.1235)    # rounded to 4 decimals
        pb.save_user_pose(path, "pre_grasp", [0.5, 0.3, -0.3, 0.3, -0.3])
        self.assertEqual(pb.load_user_file(path)["pre_grasp"]["joints"][0], 0.5)
        self.assertEqual(len(pb.load_user_file(path)), 2)


class ConfirmGateTest(unittest.TestCase):
    def test_second_click_on_the_same_button_confirms(self):
        g = pb.ConfirmGate(3.0)
        self.assertEqual(g.click("ready", 10.0), "armed")
        self.assertEqual(g.click("ready", 11.5), "confirmed")
        self.assertEqual(g.click("ready", 12.0), "armed")          # after confirming, the next click arms again

    def test_a_click_on_another_button_rearms_that_one(self):
        g = pb.ConfirmGate(3.0)
        g.click("ready", 10.0)
        self.assertEqual(g.click("default", 10.5), "armed")
        self.assertEqual(g.click("ready", 11.0), "armed")          # the first one was disarmed by the other click

    def test_a_late_second_click_does_not_confirm(self):
        g = pb.ConfirmGate(3.0)
        g.click("ready", 10.0)
        self.assertEqual(g.click("ready", 13.5), "armed")

    def test_expiry_reports_the_name_once(self):
        g = pb.ConfirmGate(3.0)
        g.click("ready", 10.0)
        self.assertIsNone(g.expired(11.0))
        self.assertEqual(g.expired(13.5), "ready")
        self.assertIsNone(g.expired(14.0))


class TorqueAndMotorReportTest(unittest.TestCase):
    def test_torque_state_follows_the_drive_state(self):
        self.assertEqual(pb.torque_state("TORQUE_OFF"), "OFF")
        for s in ("HOLDING", "TRACKING", "ESTOPPED"):
            self.assertEqual(pb.torque_state(s), "ON")           # the e-stop holds with torque on
        self.assertEqual(pb.torque_state(None), "unknown")

    def test_motor_rows_format_readings_and_carry_the_level(self):
        values = {"temperature_c": "47", "temperature_limit_c": "50", "load_percent": "12.5", "input_voltage_v": "11.8"}
        rows = pb.motor_rows([("arm_interface/joint_2", 1, "temperature 47 C near limit 50 C", values),
                              ("arm_interface/joint_3", 0, "ok", dict(values, temperature_c="36"))])
        self.assertEqual([lvl for _, lvl in rows], [1, 0])
        self.assertIn("joint_2", rows[0][0])
        self.assertIn("47/50 C", rows[0][0])
        self.assertIn("11.8 V", rows[0][0])
        self.assertIn("near limit", rows[0][0])
        self.assertNotIn("ok", rows[1][0].split("V")[-1])           # no message appended for a healthy motor

    def test_missing_data_is_reported_not_crashed_on(self):
        self.assertEqual(pb.motor_rows(None), [("no motor data", 0)])
        rows = pb.motor_rows([("arm_interface/joint_5", 0, "ok", {})])
        self.assertIn("no readings", rows[0][0])


class EStopStatusTest(unittest.TestCase):
    def test_worst_health(self):
        self.assertEqual(pb.worst_health([]), "n/a")
        self.assertEqual(pb.worst_health([0, 0, 0]), "OK")
        self.assertEqual(pb.worst_health([0, 1, 0]), "WARN")
        self.assertEqual(pb.worst_health([0, 1, 2]), "ERROR")

    def test_released_engaged_and_unknown_states(self):
        text, colour = pb.estop_status(False, "HOLDING", [0, 0])
        self.assertEqual(colour, "green")
        self.assertIn("released", text)
        self.assertIn("HOLDING", text)
        text, colour = pb.estop_status(True, "ESTOPPED", [0, 2], "joint_4: hardware error 0x20 (overload)")
        self.assertEqual(colour, "red")
        self.assertIn("ENGAGED", text)
        self.assertIn("ERROR", text)
        self.assertIn("overload", text)
        text, colour = pb.estop_status(None, None, [])
        self.assertEqual(colour, "red")                       # unknown is shown as engaged: senders treat it that way
        self.assertIn("UNKNOWN", text)
        self.assertIn("n/a", text)


class MoveGuardTest(unittest.TestCase):
    def test_large_moves_are_refused_small_ones_allowed(self):
        self.assertIsNone(pb.check_move([0, 0, 0, 0, 0], [1, 0.5, -0.5, 0.5, -0.5], 3.2))
        self.assertIn("exceeds", pb.check_move([0, 0, 0, 0, 0], [3.5, 0, 0, 0, 0], 3.2))

    def test_limits_are_read_from_a_urdf(self):
        urdf = '<robot><joint name="joint_1" type="revolute"><limit lower="-1" upper="2" effort="1" velocity="1"/></joint><joint name="fixed" type="fixed"/></robot>'
        self.assertEqual(pb.limits_from_urdf(urdf), {"joint_1": (-1.0, 2.0)})

    def test_shipped_config_loads_and_stays_inside_the_arm_limits(self):
        import yaml
        cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.dirname(HERE)), "config", "presets.yaml")))
        book = pb.PresetBook(cfg["presets"], limits=LIMITS)
        self.assertEqual(book.order, ["default", "ready", "pre_grasp"])


if __name__ == "__main__":
    unittest.main()
