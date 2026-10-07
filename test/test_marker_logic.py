"""Hardware-free tests of the marker pose logic: chaining camera and marker poses, stillness of the camera,
averaging with outlier rejection, the sliding window, and the report format."""
import json
import unittest

import numpy as np

from arm_camera.geometry import invert_transform, make_transform, rot_x, rot_y, rot_z, rotation_error_deg
from arm_perception import marker_logic as ml

T_BASE_CAM = make_transform(rot_z(0.4) @ rot_y(0.9), [0.20, -0.05, 0.50])
T_CAM_MARKER = make_transform(rot_x(np.pi) @ rot_y(0.3), [0.02, -0.01, 0.45])
T_TRUE = ml.base_pose(T_BASE_CAM, T_CAM_MARKER)


def feed(tracker, n, t0=0.0, dt=0.07, noise_mm=0.5, rot_noise_deg=0.1, seed=1, cam_shift_mm_per_s=0.0):
    rng = np.random.RandomState(seed)
    for k in range(n):
        t = t0 + k * dt
        T_cam = T_BASE_CAM.copy()
        T_cam[:3, 3] += np.array([cam_shift_mm_per_s * t / 1000.0, 0, 0])
        tracker.add_camera_pose(t, T_cam)
        e = rng.randn(3) * np.radians(rot_noise_deg)
        Tn = make_transform(rot_x(e[0]) @ rot_y(e[1]) @ rot_z(e[2]) @ T_TRUE[:3, :3], T_TRUE[:3, 3] + rng.randn(3) * noise_mm / 1000.0)
        tracker.add_detection(0, t, Tn, ambiguity=0.3, rms=0.4, distance=0.45)
    return t0 + (n - 1) * dt


class ChainTest(unittest.TestCase):
    def test_base_pose_is_the_product(self):
        np.testing.assert_allclose(ml.base_pose(T_BASE_CAM, T_CAM_MARKER), T_BASE_CAM @ T_CAM_MARKER)
        # and recovers the camera-frame pose back
        np.testing.assert_allclose(invert_transform(T_BASE_CAM) @ T_TRUE, T_CAM_MARKER, atol=1e-12)

    def test_camera_motion_measure(self):
        T2 = T_BASE_CAM.copy()
        T2[:3, 3] += [0.003, 0.004, 0.0]
        mm, deg = ml.camera_motion(T_BASE_CAM, T2)
        self.assertAlmostEqual(mm, 5.0, places=6)
        self.assertAlmostEqual(deg, 0.0, places=6)
        T3 = make_transform(T_BASE_CAM[:3, :3] @ rot_z(np.radians(2.0)), T_BASE_CAM[:3, 3])
        self.assertAlmostEqual(ml.camera_motion(T_BASE_CAM, T3)[1], 2.0, places=4)


class TrackerTest(unittest.TestCase):
    def test_a_still_camera_gives_the_averaged_pose_and_its_spread(self):
        tr = ml.MarkerTracker(window_s=0.5, min_samples=3)
        now = feed(tr, 8)
        rep = tr.report(now)
        self.assertEqual(len(rep), 1)
        m = rep[0]
        self.assertTrue(m["valid"], m["reason"])
        self.assertLess(1000 * np.linalg.norm(m["T_base_marker"][:3, 3] - T_TRUE[:3, 3]), 1.0)
        self.assertLess(rotation_error_deg(m["T_base_marker"][:3, :3], T_TRUE[:3, :3]), 0.3)
        self.assertTrue(all(s < 1.0 for s in m["std_mm"]))
        self.assertGreaterEqual(m["n"], 6)

    def test_a_moving_camera_is_not_reported(self):
        tr = ml.MarkerTracker(window_s=0.5, still_translation_mm=2.0)
        now = feed(tr, 8, cam_shift_mm_per_s=60.0)        # 60 mm/s: about 30 mm over the window
        self.assertTrue(tr.camera_moving())
        rep = tr.report(now)
        self.assertFalse(rep[0]["valid"])
        self.assertEqual(rep[0]["reason"], "camera moving")
        self.assertIsNone(rep[0]["T_base_marker"])

    def test_small_motion_below_the_threshold_is_still_still(self):
        tr = ml.MarkerTracker(window_s=0.5, still_translation_mm=2.0)
        feed(tr, 8, cam_shift_mm_per_s=1.0)               # 1 mm/s: 0.5 mm over the window
        self.assertFalse(tr.camera_moving())

    def test_too_few_samples_are_not_reported(self):
        tr = ml.MarkerTracker(min_samples=3)
        now = feed(tr, 2)
        rep = tr.report(now)
        self.assertFalse(rep[0]["valid"])
        self.assertIn("only 2 of 3", rep[0]["reason"])

    def test_an_outlier_detection_is_rejected(self):
        tr = ml.MarkerTracker(window_s=1.0)
        now = feed(tr, 8)
        bad = T_TRUE.copy()
        bad[:3, 3] += [0.04, 0, 0]                          # one detection 40 mm off
        tr.add_detection(0, now + 0.01, bad)
        tr.add_camera_pose(now + 0.01, T_BASE_CAM)
        m = tr.report(now + 0.01)[0]
        self.assertTrue(m["valid"])
        self.assertLess(1000 * np.linalg.norm(m["T_base_marker"][:3, 3] - T_TRUE[:3, 3]), 1.0)
        self.assertIn("rejected", m["reason"])

    def test_samples_leave_the_window_and_a_vanished_marker_disappears(self):
        tr = ml.MarkerTracker(window_s=0.5)
        now = feed(tr, 8)
        self.assertEqual(len(tr.report(now)), 1)
        self.assertEqual(tr.report(now + 0.6), [])           # nothing newer than the window

    def test_the_worst_ambiguity_of_the_window_is_reported(self):
        tr = ml.MarkerTracker()
        now = feed(tr, 5)
        tr.add_detection(0, now + 0.01, T_TRUE, ambiguity=0.9)
        tr.add_camera_pose(now + 0.01, T_BASE_CAM)
        self.assertAlmostEqual(tr.report(now + 0.01)[0]["ambiguity"], 0.9)

    def test_two_markers_are_tracked_separately(self):
        tr = ml.MarkerTracker()
        now = feed(tr, 6)
        T2 = make_transform(T_TRUE[:3, :3], T_TRUE[:3, 3] + [0.1, 0, 0])
        for k in range(6):
            tr.add_detection(5, k * 0.07, T2)
        rep = tr.report(now)
        self.assertEqual([m["id"] for m in rep], [0, 5])
        self.assertAlmostEqual(rep[1]["T_base_marker"][0, 3] - rep[0]["T_base_marker"][0, 3], 0.1, places=3)


class ReportTest(unittest.TestCase):
    def test_json_and_text(self):
        tr = ml.MarkerTracker()
        now = feed(tr, 8)
        rep = ml.report_dict(tr.report(now), "base_link", tr.camera_moving(), now)
        rep = json.loads(json.dumps(rep))                      # must be JSON-serialisable
        m = rep["markers"][0]
        np.testing.assert_allclose(m["position"], T_TRUE[:3, 3], atol=1e-3)
        self.assertEqual(len(m["quaternion_xyzw"]), 4)
        text = ml.format_report(rep)
        self.assertIn("marker 0:", text)
        self.assertIn("pose in base_link (camera still)", text)

    def test_size_dictionary_and_ignored_ids_appear_in_the_report(self):
        from arm_perception.marker_config import MarkerSpec
        tr = ml.MarkerTracker()
        now = feed(tr, 8)
        specs = {0: MarkerSpec(0, 0.1, "4X4_1000")}
        rep = ml.report_dict(tr.report(now), "base_link", False, now, specs, [(32, "4X4_1000")])
        self.assertEqual(rep["markers"][0]["size"], 0.1)
        self.assertEqual(rep["markers"][0]["dictionary"], "4X4_1000")
        self.assertEqual(rep["ignored"], [{"id": 32, "dictionary": "4X4_1000"}])
        text = ml.format_report(json.loads(json.dumps(rep)))
        self.assertIn("marker 0 (100 mm):", text)
        self.assertIn("not in the marker table (ignored): 32 (4X4_1000)", text)

    def test_text_for_a_moving_camera_and_for_nothing_seen(self):
        tr = ml.MarkerTracker()
        now = feed(tr, 8, cam_shift_mm_per_s=80.0)
        text = ml.format_report(ml.report_dict(tr.report(now), "base_link", True, now))
        self.assertIn("MOVING", text)
        self.assertIn("not reported (camera moving)", text)
        self.assertIn("no marker seen", ml.format_report(ml.report_dict([], "base_link", False, 0.0)))

    def test_an_ambiguous_orientation_is_flagged(self):
        tr = ml.MarkerTracker()
        now = feed(tr, 5)
        tr.add_detection(0, now + 0.01, T_TRUE, ambiguity=0.95)
        tr.add_camera_pose(now + 0.01, T_BASE_CAM)
        text = ml.format_report(ml.report_dict(tr.report(now + 0.01), "base_link", False, now))
        self.assertIn("ORIENTATION AMBIGUOUS", text)


if __name__ == "__main__":
    unittest.main()
