#!/usr/bin/env python3
"""Unit tests of the marker-plus-offset target math."""
import unittest

import numpy as np

from arm_grasp import target_offset as to


def yaw(deg):
    h = np.radians(deg) / 2.0
    return [0.0, 0.0, np.sin(h), np.cos(h)]


class TargetOffsetTest(unittest.TestCase):
    def test_zero_offset_is_the_marker_position(self):
        np.testing.assert_allclose(to.offset_target([0.4, 0.1, -0.45], [0, 0, 0, 1], [0, 0, 0]), [0.4, 0.1, -0.45])

    def test_offset_in_an_unrotated_marker_frame_adds_directly(self):
        np.testing.assert_allclose(to.offset_target([0.4, 0.1, -0.45], [0, 0, 0, 1], [0.03, -0.02, 0.01]), [0.43, 0.08, -0.44])

    def test_offset_follows_the_marker_rotation_about_z(self):
        # marker yawed 90 deg: its x axis points along base y, its y along base -x
        np.testing.assert_allclose(to.offset_target([0, 0, 0], yaw(90), [0.05, 0, 0]), [0, 0.05, 0], atol=1e-12)
        np.testing.assert_allclose(to.offset_target([0, 0, 0], yaw(90), [0, 0.05, 0]), [-0.05, 0, 0], atol=1e-12)

    def test_z_offset_is_along_the_marker_normal(self):
        q = [np.sin(np.radians(15)), 0.0, 0.0, np.cos(np.radians(15))]          # marker tilted 30 deg about x
        p = to.offset_target([0, 0, 0], q, [0, 0, 0.1])
        self.assertAlmostEqual(np.linalg.norm(p), 0.1)
        self.assertAlmostEqual(p[1], -0.1 * np.sin(np.radians(30)))

    def test_a_pure_rotation_keeps_the_distance(self):
        for deg in (0, 33, 90, 180):
            self.assertAlmostEqual(np.linalg.norm(to.offset_target([0, 0, 0], yaw(deg), [0.03, 0.04, 0.0])), 0.05)


if __name__ == "__main__":
    unittest.main()
