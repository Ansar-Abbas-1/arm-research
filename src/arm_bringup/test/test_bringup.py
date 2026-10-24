"""Static checks of the bringup launch files (no master, no hardware): `roslaunch --nodes` / `--dump-params` resolve them.

- every argument of a lower set is repeated in the upper ones with the same default (a forgotten pass-through would silently use the lower default);
- the node set of each file, and the mission_control switch (rviz | rqt | none);
- crashed nodes restart: every node but rviz / rqt windows / the driver has respawn on, the driver only with respawn_driver:=true;
- the working grasp settings are what the files say they are.
"""
import os
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(HERE, "..", "launch")
ORDER = ["bringup_arm", "bringup_vision", "bringup_detect", "bringup_grasp"]


def top_args(name):
    root = ET.parse(os.path.join(LAUNCH, name + ".launch")).getroot()
    return {a.get("name"): a.get("default") for a in root.findall("arg")}


def roslaunch(name, *args):
    cmd = ["roslaunch", "--nodes", os.path.join(LAUNCH, name + ".launch")] + list(args)
    out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
    assert out.returncode == 0, out.stderr
    return [l.strip() for l in out.stdout.splitlines() if l.strip()]


def dump(name, *args):
    cmd = ["roslaunch", "--dump-params", os.path.join(LAUNCH, name + ".launch")] + list(args)
    out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
    assert out.returncode == 0, out.stderr
    import yaml
    flat = yaml.safe_load(out.stdout)                  # {'/node/key/sub': value}
    nested = {}
    for path, value in flat.items():
        d = nested
        parts = [x for x in path.split("/") if x]
        for k in parts[:-1]:
            d = d.setdefault(k, {})
        d[parts[-1]] = value
    return nested


def nodes_with_respawn(name, *args):
    """{node name: respawn} from `roslaunch --dump` is not available: read it from the resolved launch tree through rosgraph's loader."""
    import roslaunch.config
    import roslaunch.xmlloader
    config = roslaunch.config.ROSLaunchConfig()
    loader = roslaunch.xmlloader.XmlLoader(resolve_anon=False)
    loader.load(os.path.join(LAUNCH, name + ".launch"), config, argv=["roslaunch", name + ".launch"] + list(args), verbose=False)
    return {n.name: (n.type, n.respawn) for n in config.nodes}


class ArgsTest(unittest.TestCase):
    def test_every_lower_argument_is_repeated_with_the_same_default_in_the_upper_sets(self):
        for lower, upper in zip(ORDER, ORDER[1:]):
            lo, up = top_args(lower), top_args(upper)
            for name, default in lo.items():
                if name == "rviz_config":
                    continue                                       # each set has its own rviz file
                self.assertIn(name, up, "%s lacks %s of %s" % (upper, name, lower))
                self.assertEqual(up[name], default, "%s: default of %s differs from %s" % (upper, name, lower))

    def test_every_include_passes_every_declared_argument_down(self):
        for lower, upper in zip(ORDER, ORDER[1:]):
            root = ET.parse(os.path.join(LAUNCH, upper + ".launch")).getroot()
            inc = [i for i in root.findall("include") if lower in i.get("file")][0]
            passed = {a.get("name") for a in inc.findall("arg")}
            self.assertEqual(passed, set(top_args(lower)), upper)

    def test_use_hardware_defaults_to_false(self):
        for n in ORDER:
            self.assertEqual(top_args(n)["use_hardware"], "false")


class NodesTest(unittest.TestCase):
    ARM = {"robot_state_publisher", "arm_sim_validator_node", "trajectory_service_node", "trajectory_executor_node", "reach_target_pose_node",
           "interactive_pose_marker_node", "preset_manager", "gripper_control", "rviz"}

    def test_arm_set(self):
        self.assertEqual(set(nodes_with_respawn("bringup_arm")), self.ARM)

    def test_arm_set_with_the_driver_instead_of_the_simulator(self):
        n = nodes_with_respawn("bringup_arm", "use_hardware:=true")
        self.assertIn("arm_interface", n)
        self.assertNotIn("arm_sim_validator_node", n)

    def test_arm_set_options(self):
        n = nodes_with_respawn("bringup_arm", "gripper:=false", "presets:=false", "use_rviz:=false")
        self.assertNotIn("gripper_control", n); self.assertNotIn("preset_manager", n); self.assertNotIn("rviz", n)

    def test_vision_adds_camera_tf_and_localizer(self):
        n = set(nodes_with_respawn("bringup_vision"))
        self.assertEqual(n - self.ARM, {"usb_cam", "camera_static_tf", "marker_localizer"})
        n = set(nodes_with_respawn("bringup_vision", "camera_tf:=none", "localizer:=false"))
        self.assertEqual(n - self.ARM, {"usb_cam"})
        n = set(nodes_with_respawn("bringup_vision", "camera:=false"))
        self.assertEqual(n - self.ARM, set())

    def test_detect_adds_detector_and_target(self):
        n = set(nodes_with_respawn("bringup_detect"))
        self.assertEqual(n - set(nodes_with_respawn("bringup_vision")), {"object_detector", "grasp_target"})
        n = set(nodes_with_respawn("bringup_detect", "detector:=false", "target:=false"))
        self.assertEqual(n, set(nodes_with_respawn("bringup_vision")))

    def test_grasp_set_and_the_mission_control_switch(self):
        base = set(nodes_with_respawn("bringup_detect"))
        rviz = set(nodes_with_respawn("bringup_grasp"))
        self.assertEqual(rviz - base, {"grasp_sequencer", "mission_control"})
        rqt = set(nodes_with_respawn("bringup_grasp", "mission_control:=rqt"))
        self.assertEqual(rqt - base, {"grasp_sequencer", "mission_control", "mission_control_ui"})
        none = set(nodes_with_respawn("bringup_grasp", "mission_control:=none"))
        self.assertEqual(none - base, {"grasp_sequencer"})
        view = set(nodes_with_respawn("bringup_grasp", "detector_view:=true"))
        self.assertIn("detector_view", view)

    def test_the_rviz_file_follows_the_switch(self):
        def rviz_args(*a):
            import roslaunch.config, roslaunch.xmlloader
            config = roslaunch.config.ROSLaunchConfig()
            roslaunch.xmlloader.XmlLoader(resolve_anon=False).load(os.path.join(LAUNCH, "bringup_grasp.launch"), config, argv=["roslaunch", "x"] + list(a), verbose=False)
            return [n.args for n in config.nodes if n.type == "rviz"][0]
        self.assertIn("mission.rviz", rviz_args())
        self.assertIn("detect.rviz", rviz_args("mission_control:=rqt"))
        self.assertIn("detect.rviz", rviz_args("mission_control:=none"))
        self.assertIn("custom.rviz", rviz_args("rviz_config:=/tmp/custom.rviz"))


class RespawnTest(unittest.TestCase):
    NEVER = {"rviz", "mission_control_ui", "detector_view"}

    def test_every_node_restarts_except_rviz_windows_and_the_driver(self):
        n = nodes_with_respawn("bringup_grasp", "use_hardware:=true", "mission_control:=rqt", "detector_view:=true", "camera_tf:=hand_eye")
        for name, (typ, respawn) in n.items():
            if name in self.NEVER:
                self.assertFalse(respawn, name)
            elif typ == "robot_interface":
                self.assertFalse(respawn, "the driver must not respawn by default")
            else:
                self.assertTrue(respawn, "%s does not respawn" % name)

    def test_the_driver_respawns_only_when_asked(self):
        n = nodes_with_respawn("bringup_grasp", "use_hardware:=true", "respawn_driver:=true")
        self.assertTrue(n["arm_interface"][1])

    def test_respawn_false_turns_all_off(self):
        n = nodes_with_respawn("bringup_grasp", "respawn:=false")
        self.assertFalse([k for k, v in n.items() if v[1] and v[0] != "usb_cam_node"])      # camera.launch keeps its own respawn (the webcam drops off USB)


class ParamsTest(unittest.TestCase):
    def test_working_grasp_settings_are_the_defaults(self):
        p = dump("bringup_grasp")
        self.assertEqual(str(p["grasp_target"]["approach"]), "45")
        self.assertAlmostEqual(p["grasp_target"]["bias"][0], -0.010)
        self.assertAlmostEqual(p["grasp_target"]["bias"][1], 0.010)
        self.assertEqual(p["object_detector"]["grasp_heights"], {"Vigna": 0.09})
        self.assertEqual(p["object_detector"]["target_point"], "mask")
        self.assertTrue(p["object_detector"]["model"].endswith("vignax_small_v45A.onnx"))
        self.assertEqual(p["gripper_control"]["close"]["load_target"], 15.0)
        self.assertEqual(p["gripper_control"]["close"]["contact_load"], 5.0)
        self.assertEqual(p["mission_control"]["plant_class"], "Vigna")
        self.assertEqual(p["grasp_sequencer"]["correction_trigger"], 0.01)

    def test_sim_logs_the_sag_measurements_away_from_the_real_ones(self):
        self.assertEqual(dump("bringup_grasp")["grasp_sequencer"]["sag_log_dir"], "/home/dev/recordings/sag/sim")
        self.assertEqual(dump("bringup_grasp", "use_hardware:=true")["grasp_sequencer"]["sag_log_dir"], "/home/dev/recordings/sag")
        self.assertEqual(dump("bringup_grasp", "sag_log_dir:=/tmp/x")["grasp_sequencer"]["sag_log_dir"], "/tmp/x")

    def test_arguments_override_the_profile(self):
        p = dump("bringup_grasp", "model:=/tmp/other.onnx", "device_ep:=cpu", "grasp_height:=0.05", "approach:=down", "bias_r:=0.0", "load_target:=25",
                 "plant_class:=Cup", "plan_valid_s:=30")
        self.assertEqual(p["object_detector"]["model"], "/tmp/other.onnx")
        self.assertEqual(p["object_detector"]["device"], "cpu")
        self.assertEqual(p["object_detector"]["grasp_heights"]["Cup"], 0.05)       # the yaml default of the other class stays
        self.assertEqual(p["grasp_target"]["approach"], "down")
        self.assertEqual(p["gripper_control"]["close"]["load_target"], 25.0)
        self.assertEqual(p["mission_control"]["plant_class"], "Cup")
        self.assertEqual(p["mission_control"]["plan_valid_s"], 30.0)

    def test_camera_mount_arguments_reach_the_static_transform(self):
        import roslaunch.config, roslaunch.xmlloader
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader(resolve_anon=False).load(os.path.join(LAUNCH, "bringup_vision.launch"), config, argv=["roslaunch", "x", "z:=-0.07"], verbose=False)
        args = [n.args for n in config.nodes if n.name == "camera_static_tf"][0]
        self.assertEqual(args.split()[:6], ["-0.025", "0", "-0.07", "1.5708", "0", "0"])


if __name__ == "__main__":
    unittest.main()
