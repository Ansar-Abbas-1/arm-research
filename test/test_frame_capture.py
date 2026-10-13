#!/usr/bin/env python3
"""Unit tests of frame_capture: file names and the sidecar record."""
import json
import unittest

from arm_perception import frame_capture as fc


class NameTest(unittest.TestCase):
    def test_format_and_uniqueness(self):
        a, b = fc.frame_name("lab", 1791909207.6444874, 0), fc.frame_name("lab", 1791909207.6444874, 1)
        self.assertTrue(a.startswith("lab_"))
        self.assertNotEqual(a, b)
        self.assertEqual(a.split("_")[-2], "644")
        self.assertEqual(a.split("_")[-1], "000")

    def test_sorts_by_time(self):
        self.assertLess(fc.frame_name("x", 1791909200.0, 0), fc.frame_name("x", 1791909201.0, 0))


class SidecarTest(unittest.TestCase):
    def test_json_serialisable_and_complete(self):
        rec = fc.sidecar(12.5, "camera_optical_frame", 1920, 1080, range(9), [0.1, 0.2, 0.0, 0.0, 0.0],
                         fc.pose_record((0.1, 0.2, 0.3), (0, 0, 0, 1)), None, {"joint_1": 0.5}, "pre grasp")
        d = json.loads(json.dumps(rec))
        self.assertEqual(d["K"], [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0])
        self.assertEqual(d["camera_pose_base"]["position"], [0.1, 0.2, 0.3])
        self.assertIsNone(d["marker_0_pose_base"])
        self.assertEqual(d["joints"], {"joint_1": 0.5})

    def test_unknown_pose_is_none(self):
        self.assertIsNone(fc.pose_record(None, (0, 0, 0, 1)))


if __name__ == "__main__":
    unittest.main()
