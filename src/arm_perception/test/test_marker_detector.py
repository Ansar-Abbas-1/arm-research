"""Tests of the per-frame detector against the marker table: sizes per marker, two dictionaries, ignored markers."""
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "arm_camera", "test"))
import test_aruco as TA  # noqa: E402

from arm_perception import marker_config, marker_detector  # noqa: E402

CFG = marker_config.MarkerConfig.from_dict({"dictionary": "4X4_1000", "markers": [
    {"id": 0, "size": 0.100}, {"id": 94, "size": 0.050, "dictionary": "ARUCO_ORIGINAL"}]})


def two_markers(T0, T94):
    big = TA.render_scene([("4X4_1000", 0, T0), ("4X4_1000", 5, TA.pose_facing_camera(0.8, 0.3, (0.21, 0.08, 0.45)))])
    TA.SIZE = 0.050
    try:
        small = TA.render_scene([("ARUCO_ORIGINAL", 94, T94)])
    finally:
        TA.SIZE = 0.100
    return np.where(np.abs(small.astype(int) - 190) > 3, small, big).astype(np.uint8)


class DetectorTest(unittest.TestCase):
    def test_each_marker_is_posed_with_its_own_size_and_dictionary(self):
        T0 = TA.pose_facing_camera(0.3, 0.4, (0.02, -0.01, 0.45))
        T94 = TA.pose_facing_camera(-0.5, 0.35, (-0.10, -0.02, 0.40))
        found, ignored = marker_detector.MarkerDetector(CFG).detect(two_markers(T0, T94), TA.K, np.zeros(5))
        by = {d.id: d for d in found}
        self.assertEqual(sorted(by), [0, 94])
        self.assertEqual(by[0].size, 0.100)
        self.assertEqual(by[94].size, 0.050)
        self.assertLess(np.linalg.norm(by[0].T_cam_marker[:3, 3] - T0[:3, 3]) * 1000.0, 3.0)
        self.assertLess(np.linalg.norm(by[94].T_cam_marker[:3, 3] - T94[:3, 3]) * 1000.0, 4.0)
        self.assertAlmostEqual(by[94].distance, np.linalg.norm(T94[:3, 3]), delta=0.005)

    def test_a_marker_not_in_the_table_is_ignored_not_posed(self):
        T0 = TA.pose_facing_camera(0.3, 0.4, (0.02, -0.01, 0.45))
        found, ignored = marker_detector.MarkerDetector(CFG).detect(two_markers(T0, TA.pose_facing_camera(-0.5, 0.35, (-0.10, -0.02, 0.40))), TA.K, np.zeros(5))
        self.assertNotIn(5, [d.id for d in found])
        self.assertIn((5, "4X4_1000"), ignored)

    def test_an_empty_image_gives_nothing(self):
        found, ignored = marker_detector.MarkerDetector(CFG).detect(np.full((1080, 1920), 190, np.uint8), TA.K, np.zeros(5))
        self.assertEqual((found, ignored), ([], []))


if __name__ == "__main__":
    unittest.main()
