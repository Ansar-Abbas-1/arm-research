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


class SideFrameTest(unittest.TestCase):
    def test_a_side_frame_is_horizontal_pointing_back_toward_the_base_with_y_perpendicular_to_the_arm_plane(self):
        for pos in ([0.5, 0.0, -0.55], [0.4, 0.3, -0.5], [-0.2, 0.4, -0.6]):
            R = to.side_frame(pos)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0)
            radial = np.array([pos[0], pos[1], 0.0]) / np.hypot(pos[0], pos[1])
            np.testing.assert_allclose(R[:, 2], -radial, atol=1e-12)               # z: horizontal, toward the base
            np.testing.assert_allclose(R[:, 0], [0, 0, 1], atol=1e-12)             # x: up
            self.assertAlmostEqual(R[2, 1], 0.0)                                    # y horizontal
            self.assertAlmostEqual(float(R[:, 1] @ radial), 0.0)                    # and perpendicular to the arm's plane

    def test_a_target_over_the_base_has_a_defined_frame(self):
        R = to.side_frame([0.0, 0.0, -0.5])
        np.testing.assert_allclose(R[:, 2], [-1, 0, 0])
        self.assertAlmostEqual(np.linalg.det(R), 1.0)


class AngledFrameTest(unittest.TestCase):
    def test_90_degrees_is_the_floor_frame_and_0_the_side_frame(self):
        pos = [0.45, 0.2, -0.55]
        np.testing.assert_allclose(to.angled_frame(pos, 0.0), to.side_frame(pos), atol=1e-12)
        np.testing.assert_allclose(to.angled_frame(pos, 90.0)[:, 2], [0, 0, 1], atol=1e-12)         # retreat straight up = approach straight down

    def test_the_frame_is_a_rotation_in_the_arms_plane_at_the_requested_angle(self):
        for pos in ([0.5, 0.0, -0.55], [0.4, 0.3, -0.5], [-0.2, 0.4, -0.6]):
            R = to.angled_frame(pos, 45.0)
            np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(R), 1.0)
            radial = np.array([pos[0], pos[1], 0.0]) / np.hypot(pos[0], pos[1])
            np.testing.assert_allclose(R[:, 2], -np.sqrt(0.5) * radial + np.sqrt(0.5) * np.array([0, 0, 1]), atol=1e-12)   # back and up at 45 degrees
            self.assertAlmostEqual(float(R[:, 1] @ radial), 0.0)                                      # y perpendicular to the arm's plane
            self.assertAlmostEqual(R[2, 1], 0.0)                                                       # and horizontal

    def test_parse_approach(self):
        self.assertEqual(to.parse_approach("down"), 90.0)
        self.assertEqual(to.parse_approach(" Forward "), 0.0)
        self.assertEqual(to.parse_approach("45"), 45.0)
        self.assertEqual(to.parse_approach(30), 30.0)
        for bad in ("sideways", -1, 91, "nan"):
            with self.assertRaises(ValueError):
                to.parse_approach(bad)


class BiasTest(unittest.TestCase):
    def test_the_bias_is_in_the_arms_frame_at_the_targets_azimuth(self):
        for az_deg in (-40.0, 0.0, 30.0):
            az = np.radians(az_deg)
            p = np.array([0.5 * np.cos(az), 0.5 * np.sin(az), -0.5])
            q = to.apply_bias(p, (-0.010, 0.010, 0.002))
            d = q - p
            u, v = np.array([np.cos(az), np.sin(az), 0.0]), np.array([-np.sin(az), np.cos(az), 0.0])
            self.assertAlmostEqual(float(d @ u), -0.010, places=12)         # closer to the base
            self.assertAlmostEqual(float(d @ v), 0.010, places=12)          # toward increasing azimuth: left seen from the robot
            self.assertAlmostEqual(d[2], 0.002, places=12)

    def test_zero_bias_changes_nothing(self):
        np.testing.assert_allclose(to.apply_bias([0.4, 0.1, -0.5], (0, 0, 0)), [0.4, 0.1, -0.5])


if __name__ == "__main__":
    unittest.main()
