"""Unit tests of the status text of estop_cli.py (no ROS master)."""
import importlib.util
import os
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("estop_cli", os.path.join(os.path.dirname(HERE), "scripts", "estop_cli.py"))
cli = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cli)


class FormatStatusTest(unittest.TestCase):
    def test_released_and_healthy(self):
        lines = cli.format_status(False, "HOLDING", [("joint_1", 0, "ok"), ("joint_2", 0, "ok")])
        self.assertEqual(lines[0], "e-stop: released")
        self.assertIn("HOLDING", lines[1])
        self.assertIn("all OK", lines[2])
        self.assertEqual(len(lines), 3)

    def test_engaged_with_a_motor_error_lists_the_motor(self):
        lines = cli.format_status(True, "ESTOPPED", [("joint_3", 0, "ok"), ("joint_4", 2, "hardware error 0x20 (overload)")])
        self.assertEqual(lines[0], "e-stop: ENGAGED")
        self.assertIn("1 with a warning or error", lines[2])
        self.assertTrue(any("ERROR joint_4" in l and "overload" in l for l in lines))

    def test_warning_is_listed_too(self):
        lines = cli.format_status(False, "TRACKING", [("joint_2", 1, "temperature 47 C near limit 50 C")])
        self.assertTrue(any("WARN joint_2" in l for l in lines))

    def test_unknown_state_and_missing_motor_topic(self):
        lines = cli.format_status(None, None, None)
        self.assertIn("UNKNOWN", lines[0])
        self.assertIn("n/a", lines[1])
        self.assertIn("no motor_status", lines[2])


class FormatMotorsTest(unittest.TestCase):
    def test_table_has_a_header_and_one_line_per_motor(self):
        v = {"temperature_c": "47", "temperature_limit_c": "50", "load_percent": "46.3", "input_voltage_v": "11.8",
             "hardware_error": "0", "hardware_id": "2"}
        lines = cli.format_motors([("arm_interface/joint_2", 1, "temperature 47 C near limit 50 C", v),
                                   ("arm_interface/joint_4", 2, "hardware error 0x20 (overload)", dict(v, hardware_error="32", hardware_id="4"))])
        self.assertEqual(len(lines), 3)
        self.assertIn("47/50", lines[1])
        self.assertIn("46.3%", lines[1])
        self.assertIn("WARN", lines[1])
        self.assertIn("0x20", lines[2])
        self.assertIn("ERROR", lines[2])

    def test_missing_topic_and_missing_values(self):
        self.assertIn("no motor_status", cli.format_motors(None)[0])
        lines = cli.format_motors([("arm_interface/joint_1", 0, "ok", {})])
        self.assertIn("-/-", lines[1])


if __name__ == "__main__":
    unittest.main()
