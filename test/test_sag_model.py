#!/usr/bin/env python3
"""Unit tests of sag_model: log parsing, the first-measurement samples, the per-angle fit and the feed-forward offsets."""
import json
import unittest

import numpy as np

from arm_grasp import sag_model as sm


def row(key="PRE", step=0, aim=(0.4, 0.0, -0.4), sag=(0.0, 0.0, -0.03), angle=45.0, mode="real"):
    aim = np.array(aim, float)
    return {"mode": mode, "key": key, "step": step, "approach_deg": angle, "goal": aim.tolist(), "aim": aim.tolist(), "measured": (aim + sag).tolist()}


class ReadTest(unittest.TestCase):
    def test_bad_lines_are_skipped(self):
        rows = sm.read_rows(["", "not json", json.dumps(row()), "[1, 2]", '{"cut": '])
        self.assertEqual(len(rows), 1)


class SamplesTest(unittest.TestCase):
    def test_only_first_measurements_of_known_keys_and_modes_count(self):
        rows = [row(), row(step=1), row(key="XYZ"), row(mode="sim"), {"key": "PRE"}, row(key="GRASP", sag=(0.01, 0, -0.02))]
        samples = sm.sag_samples(rows)
        self.assertEqual([(k, tuple(np.round(s, 3))) for _, k, s in samples], [("PRE", (0.0, 0.0, -0.03)), ("GRASP", (0.01, 0.0, -0.02))])

    def test_the_sag_does_not_depend_on_an_aim_that_already_includes_a_compensation(self):
        a = sm.sag_samples([row(aim=(0.4, 0, -0.4))])[0][2]
        b = sm.sag_samples([row(aim=(0.4, 0, -0.37))])[0][2]                           # aimed 3 cm higher: the arm still adds the same displacement
        np.testing.assert_allclose(a, b)


class FitTest(unittest.TestCase):
    def setUp(self):
        rows = [row(sag=(0, 0, -0.030)), row(sag=(0, 0, -0.034)), row(key="GRASP", sag=(0.002, 0, -0.020)), row(key="GRASP", sag=(0.004, 0, -0.024)),
                row(sag=(0, 0, -0.010), angle=90.0), row(sag=(0, 0, -0.012), angle=90.0)]
        self.model = sm.SagModel.fit(sm.sag_samples(rows))

    def test_mean_and_count_per_bucket_and_key(self):
        s, n = self.model.sag("PRE", 45.0)
        self.assertEqual(n, 2)
        np.testing.assert_allclose(s, [0, 0, -0.032])
        np.testing.assert_allclose(self.model.sag("PRE", 90.0)[0], [0, 0, -0.011])           # another approach angle has its own sag

    def test_offsets_are_minus_the_sag_scaled_by_the_gain_and_limited(self):
        off = self.model.offsets(45.0, gain=0.5)
        np.testing.assert_allclose(off["PRE"], [0, 0, 0.016])
        np.testing.assert_allclose(self.model.offsets(45.0)["GRASP"], [-0.003, 0, 0.022])
        big = sm.SagModel.fit(sm.sag_samples([row(sag=(0, 0, -0.2)), row(sag=(0, 0, -0.2))]))
        self.assertAlmostEqual(float(np.linalg.norm(big.offsets(45.0)["PRE"])), 0.08)       # limited to max_offset

    def test_lift_falls_back_to_the_grasp_sag_until_a_probe_measured_it(self):
        np.testing.assert_allclose(self.model.offsets(45.0)["LIFT"], self.model.offsets(45.0)["GRASP"])
        lifted = sm.SagModel.fit(sm.sag_samples([row(key="LIFT", sag=(0, 0, -0.05)), row(key="LIFT", sag=(0, 0, -0.05)), row(key="GRASP", sag=(0, 0, -0.01)),
                                                 row(key="GRASP", sag=(0, 0, -0.01))]))
        np.testing.assert_allclose(lifted.offsets(45.0)["LIFT"], [0, 0, 0.05])

    def test_no_offset_without_enough_samples_or_for_a_far_angle(self):
        one = sm.SagModel.fit(sm.sag_samples([row()]))
        np.testing.assert_allclose(one.offsets(45.0)["PRE"], [0, 0, 0])                      # a single sample is not a model
        np.testing.assert_allclose(self.model.offsets(0.0)["PRE"], [0, 0, 0])                # no bucket within 15 degrees of 0
        np.testing.assert_allclose(self.model.offsets(52.0)["PRE"], [0, 0, 0.032])           # 7 degrees from the 45 bucket: used

    def test_save_and_load_round_trip(self):
        import os
        import tempfile
        path = os.path.join(tempfile.mkdtemp(), "m.json")
        sm.save(self.model, path)
        back = sm.load(path)
        np.testing.assert_allclose(back.offsets(45.0)["GRASP"], self.model.offsets(45.0)["GRASP"])


class ResetTest(unittest.TestCase):
    def test_the_correction_can_start_from_a_feed_forward_value(self):
        from arm_grasp.correction import PositionCorrection
        c = PositionCorrection()
        c.reset([0.0, 0.0, 0.03])
        np.testing.assert_allclose(c.comp, [0, 0, 0.03])
        c.reset()
        np.testing.assert_allclose(c.comp, [0, 0, 0])


class GridTest(unittest.TestCase):
    def test_points_are_at_the_radius_height_and_azimuth(self):
        pts = sm.grid_points(-40.0, [0.52, 0.40], [0.12, 0.07], floor_z=-0.60)
        self.assertEqual([(r, h) for r, h, _ in pts], [(0.40, 0.07), (0.40, 0.12), (0.52, 0.07), (0.52, 0.12)])      # nearest, lowest first
        for r, h, p in pts:
            self.assertAlmostEqual(float(np.hypot(p[0], p[1])), r)
            self.assertAlmostEqual(float(np.degrees(np.arctan2(p[1], p[0]))), -40.0)
            self.assertAlmostEqual(p[2], -0.60 + h)


class FitToolTest(unittest.TestCase):
    def test_the_fit_script_writes_a_model_from_a_log_directory(self):
        import os
        import subprocess
        import sys
        import tempfile
        d = tempfile.mkdtemp()
        with open(os.path.join(d, "sag_20261014.jsonl"), "w") as f:
            for z in (-0.030, -0.034, -0.032):
                f.write(json.dumps(row(sag=(0, 0, z))) + "\n")
                f.write(json.dumps(row(key="GRASP", sag=(0, 0, z / 2))) + "\n")
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts", "sag_fit.py")
        out = os.path.join(d, "m.json")
        res = subprocess.run([sys.executable, script, "--log-dir", d, "--out", out], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("6 first measurements used", res.stdout)
        np.testing.assert_allclose(sm.load(out).offsets(45.0)["PRE"], [0, 0, 0.032], atol=1e-9)


if __name__ == "__main__":
    unittest.main()
