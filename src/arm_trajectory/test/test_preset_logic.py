"""Unit tests of the pure logic of preset_manager_node (no ROS master): preset book and validation, the
move-size guard, preset names, the state JSON, the user-file round trip."""
import importlib.util
import os
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("preset_manager_node", os.path.join(os.path.dirname(HERE), "scripts", "preset_manager_node.py"))
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


class NameAndStateTest(unittest.TestCase):
    def test_valid_names(self):
        for ok in ("pre_grasp", "above table", "A1", "x-y"):
            self.assertTrue(pb.valid_name(ok), ok)
        for bad in ("", "1abc", "_x", "a/b", "a" * 40, None, "../x", "a\nb"):
            self.assertFalse(pb.valid_name(bad), repr(bad))

    def test_state_json_lists_presets_in_order_with_user_flag_and_the_move(self):
        import json
        user = {"mine": {"joints": [0.0, 0.5, -0.5, 0.5, -0.5], "note": "saved"}}
        book = pb.PresetBook(SHIPPED, user, LIMITS)
        d = json.loads(pb.state_json(book, set(user), {"name": "ready", "state": "moving", "message": ""}))
        self.assertEqual([p["name"] for p in d["presets"]], ["default", "ready", "mine"])
        self.assertEqual([p["user"] for p in d["presets"]], [False, False, True])
        self.assertEqual(d["presets"][2]["joints"], [0.0, 0.5, -0.5, 0.5, -0.5])
        self.assertEqual(d["move"]["state"], "moving")


class MoveGuardTest(unittest.TestCase):
    def test_large_moves_are_refused_small_ones_allowed(self):
        self.assertIsNone(pb.check_move([0, 0, 0, 0, 0], [1, 0.5, -0.5, 0.5, -0.5], 3.2))
        self.assertIn("exceeds", pb.check_move([0, 0, 0, 0, 0], [3.5, 0, 0, 0, 0], 3.2))

    def test_zero_or_negative_max_delta_disables_the_guard(self):
        self.assertIsNone(pb.check_move([0, 0, 0, 0, 0], [3.5, 0, 0, 0, 0], 0))
        self.assertIsNone(pb.check_move([0, 0, 0, 0, 0], [3.5, 0, 0, 0, 0], -1))

    def test_limits_are_read_from_a_urdf(self):
        urdf = '<robot><joint name="joint_1" type="revolute"><limit lower="-1" upper="2" effort="1" velocity="1"/></joint><joint name="fixed" type="fixed"/></robot>'
        self.assertEqual(pb.limits_from_urdf(urdf), {"joint_1": (-1.0, 2.0)})

    def test_shipped_config_loads_and_stays_inside_the_arm_limits(self):
        import yaml
        cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), "config", "presets.yaml")))
        book = pb.PresetBook(cfg["presets"], limits=LIMITS)
        self.assertEqual(book.order, ["default", "ready", "pre_grasp"])


if __name__ == "__main__":
    unittest.main()
