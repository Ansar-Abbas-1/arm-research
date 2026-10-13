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


class VerticalFrameTest(unittest.TestCase):
    def test_a_tilted_marker_gives_an_upright_frame_with_the_same_heading(self):
        q = [np.sin(np.radians(6)), 0.0, 0.0, np.cos(np.radians(6))]               # 12 deg tilt about x
        R = to.vertical_frame(q)
        np.testing.assert_allclose(R[:, 2], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(R.T.dot(R), np.eye(3), atol=1e-12)
        np.testing.assert_allclose(R[:, 0], [1, 0, 0], atol=1e-12)                 # the heading (marker x) is kept

    def test_the_heading_follows_the_marker_yaw(self):
        R = to.vertical_frame(yaw(90))
        np.testing.assert_allclose(R[:, 0], [0, 1, 0], atol=1e-12)

    def test_z_offset_is_straight_up_even_for_a_tilted_marker(self):
        q = [np.sin(np.radians(10)), 0.0, 0.0, np.cos(np.radians(10))]
        p, R = to.offset_target_frame([0.4, 0.1, -0.6], q, [0, 0, 0.03], vertical=True)
        np.testing.assert_allclose(p, [0.4, 0.1, -0.57], atol=1e-12)
        p2, _ = to.offset_target_frame([0.4, 0.1, -0.6], q, [0, 0, 0.03], vertical=False)
        self.assertGreater(np.linalg.norm(p2 - p), 1e-3)                            # the marker-frame offset leans with the tilt

    def test_a_marker_standing_on_its_edge_falls_back_to_the_identity_heading(self):
        q = [0.0, np.sin(np.radians(45)), 0.0, np.cos(np.radians(45))]               # x axis pointing straight up
        np.testing.assert_allclose(to.vertical_frame(q), np.eye(3), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
