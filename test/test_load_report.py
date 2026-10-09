#!/usr/bin/env python3
"""Unit tests of the gripper recording analysis with synthetic series (known rate, noise, slope)."""
import unittest

import numpy as np

from arm_grasp import load_report as lr


def synth(slope=5.0, base=3.0, noise=0.0, rate=10.0, seed=1):
    """Gripper that closes at 4 mm/s, is blocked at 0.012 and then pushes 8 mm past it; a still phase before and after."""
    rng = np.random.RandomState(seed)
    t = np.arange(0.0, 12.0, 1.0 / rate)
    cmd = np.where(t < 2.0, 0.025, np.maximum(0.025 - 0.004 * (t - 2.0), 0.004))
    pos = np.maximum(cmd, 0.012)
    err = pos - cmd
    load = base + slope * err * 1000.0 + noise * rng.randn(len(t))
    return (t, pos), (t, cmd), (t, load)


class LoadReportTest(unittest.TestCase):
    def test_rate_and_largest_gap(self):
        t = np.concatenate([np.arange(0, 2, 0.1), np.arange(2.5, 4, 0.1)])
        rate, gap = lr.rate_and_gap(t)
        self.assertAlmostEqual(rate, 10.0, places=3)
        self.assertAlmostEqual(gap, 0.6, places=3)

    def test_the_load_slope_against_the_command_error_is_recovered(self):
        f = lr.analyse(*synth(slope=5.0), target=40.0)
        self.assertAlmostEqual(f["load_slope_pct_per_mm"], 5.0, delta=0.3)
        self.assertAlmostEqual(f["load_offset_pct"], 3.0, delta=1.0)
        self.assertGreater(f["load_error_correlation"], 0.99)

    def test_static_load_noise_is_measured(self):
        f = lr.analyse(*synth(noise=1.5), target=40.0)
        self.assertAlmostEqual(f["static_load_std"], 1.5, delta=0.6)
        self.assertAlmostEqual(f["static_load_median"], 3.0, delta=1.0)

    def test_motion_load_without_pushing_is_separated(self):
        m, c, l = synth()
        f = lr.analyse(m, c, l)
        self.assertLess(f["motion_load_median"], 5.0)

    def test_time_at_target_and_max_load(self):
        f = lr.analyse(*synth(), target=40.0)
        self.assertGreater(f["first_time_at_target_s"], 4.0)         # only after the command went past the block
        self.assertGreaterEqual(f["max_load"], 40.0)
        g = lr.analyse(*synth(slope=0.5), target=40.0)
        self.assertNotIn("first_time_at_target_s", g)                # never reached

    def test_a_gap_is_reported_for_the_load_max_age_check(self):
        m, c, (t, l) = synth()
        keep = (t < 5.0) | (t > 6.0)
        f = lr.analyse(m, c, (t[keep], l[keep]))
        self.assertGreater(f["load_max_gap_s"], 1.0)

    def test_too_few_samples_do_not_crash(self):
        f = lr.analyse(([0.0], [0.0]), ([0.0], [0.0]), ([0.0, 0.1], [1.0, 1.0]))
        self.assertIn("load_rate_hz", f)

    def test_report_lines_name_the_parameter_each_fact_informs(self):
        text = "\n".join(lr.format_report(lr.analyse(*synth(noise=0.5))))
        for key in ("close/load_max_age", "close/load_band", "close/trim_speed"):
            self.assertIn(key, text)


if __name__ == "__main__":
    unittest.main()
