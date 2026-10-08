"""Tests of the pure grasp planning logic."""
import unittest

import numpy as np

from arm_grasp import grasp_plan as gp


def floor_target(x=0.45, y=0.10, z=-0.60, yaw=0.5):
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])        # +z up
    T[:3, 3] = [x, y, z]
    return T


class OrientationTest(unittest.TestCase):
    def test_a_floor_target_gives_a_tool_axis_straight_down_with_y_horizontal(self):
        R, dev = gp.tool_rotation([0, 0, -1], np.array([0.4, 0.2, -0.5]))
        np.testing.assert_allclose(R[:, 2], [0, 0, -1], atol=1e-12)
        self.assertAlmostEqual(R[2, 1], 0.0)                       # y horizontal
        az = np.arctan2(0.2, 0.4)
        np.testing.assert_allclose(R[:, 1], [-np.sin(az), np.cos(az), 0.0], atol=1e-12)   # perpendicular to the arm plane
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        self.assertAlmostEqual(dev, 0.0, places=6)

    def test_at_zero_azimuth_it_is_the_pose_the_simulator_uses_for_straight_down(self):
        R, _ = gp.tool_rotation([0, 0, -1], np.array([0.4, 0.0, -0.5]))
        np.testing.assert_allclose(R, np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]]), atol=1e-12)    # Ry(pi)

    def test_an_approach_out_of_the_arm_plane_is_projected_and_reported(self):
        a = np.array([0.3, 0.0, -1.0]); a /= np.linalg.norm(a)
        R, dev = gp.tool_rotation(a, np.array([0.4, 0.0, -0.5]))   # in the plane (azimuth 0): fine
        self.assertAlmostEqual(dev, 0.0, places=6)
        R, dev = gp.tool_rotation(np.array([0.0, 0.3, -1.0]) / np.linalg.norm([0.0, 0.3, -1.0]), np.array([0.4, 0.0, -0.5]))
        self.assertGreater(dev, 10.0)                              # sideways tilt cannot be matched by a pitch-only wrist


class WaypointTest(unittest.TestCase):
    def test_pre_grasp_lift_are_straight_above_the_target_point_at_their_heights(self):
        spec = gp.GraspSpec()
        wps = gp.waypoints(floor_target(), spec)
        self.assertEqual([w.name for w in wps], ["PRE", "GRASP", "LIFT"])
        for w, h in zip(wps, (0.10, 0.02, 0.15)):
            np.testing.assert_allclose(w.position, [0.45, 0.10, -0.60 + h], atol=1e-12)
            np.testing.assert_allclose(w.R[:, 2], [0, 0, -1], atol=1e-12)

    def test_the_grasp_offset_is_in_the_target_frame_and_the_tool_offset_along_the_tool_axis(self):
        spec = gp.GraspSpec(grasp_offset=(0.03, 0.0), tool_offset=0.05)
        T = floor_target(yaw=np.pi / 2)                              # target x axis points along base +y
        w = gp.waypoints(T, spec)[1]
        np.testing.assert_allclose(w.grasp_point, [0.45, 0.13, -0.60 + 0.02], atol=1e-12)
        np.testing.assert_allclose(w.position, w.grasp_point - 0.05 * w.R[:, 2], atol=1e-12)    # origin 5 cm ABOVE the grasp point
        self.assertAlmostEqual(w.position[2] - w.grasp_point[2], 0.05, places=9)

    def test_a_tilted_target_is_refused_if_the_wrist_cannot_match_its_normal(self):
        T = floor_target()
        T[:3, :3] = np.array([[1, 0, 0], [0, np.cos(0.8), -np.sin(0.8)], [0, np.sin(0.8), np.cos(0.8)]])   # normal tilted 46 deg sideways
        T[:3, 3] = [0.45, 0.0, -0.6]
        with self.assertRaises(ValueError):
            gp.waypoints(T, gp.GraspSpec())

    def test_quaternion_round_trip(self):
        R, _ = gp.tool_rotation([0, 0, -1], np.array([0.4, 0.2, -0.5]))
        x, y, z, w = gp.quaternion_from_matrix(R)
        self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0, places=12)
        Rq = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(Rq, R, atol=1e-12)


class TargetMovedTest(unittest.TestCase):
    def test_translation_and_rotation_of_a_moved_target(self):
        a = floor_target(yaw=0.2)
        b = floor_target(x=0.455, yaw=0.2 + np.radians(2.0))
        mm, deg = gp.target_moved(a, b)
        self.assertAlmostEqual(mm, 5.0, places=6)
        self.assertAlmostEqual(deg, 2.0, places=6)
        self.assertEqual(gp.target_moved(a, a), (0.0, 0.0))


class PlanCheckTest(unittest.TestCase):
    def test_feasible_plan_has_no_problems_and_a_duration_with_the_slow_down(self):
        spec = gp.GraspSpec()
        legs = [gp.Leg("PRE", True, duration=4.0), gp.Leg("DESCEND", True, duration=2.0, deviation=0.002, cartesian=True),
                gp.Leg("LIFT", True, duration=2.5, deviation=0.001, cartesian=True)]
        self.assertEqual(gp.check_plan(legs, spec), [])
        self.assertAlmostEqual(gp.total_duration(legs, spec, gripper_time=1.0), 4.0 + 2.0 * 3.0 + 2.5 * 2.0 + 1.0)

    def test_an_unreachable_leg_and_a_wandering_straight_leg_are_reported(self):
        spec = gp.GraspSpec()
        legs = [gp.Leg("PRE", True, duration=4.0), gp.Leg("DESCEND", False, "goal_pose is unreachable"),
                gp.Leg("LIFT", True, duration=2.0, deviation=0.012, cartesian=True)]
        problems = gp.check_plan(legs, spec)
        self.assertEqual(len(problems), 2)
        self.assertIn("DESCEND: not reachable", problems[0])
        self.assertIn("12.0 mm", problems[1])


class TrajectoryStretchTest(unittest.TestCase):
    def test_stretching_scales_time_and_derivatives(self):
        pts = [(0.0, [0.0], [0.0], [0.0]), (1.0, [1.0], [2.0], [4.0])]
        out = gp.stretch_times(pts, 2.0)
        self.assertEqual(out[1][0], 2.0)
        self.assertEqual(out[1][2], [1.0])
        self.assertEqual(out[1][3], [1.0])
        self.assertEqual(out[1][1], [1.0])                          # positions unchanged
        self.assertEqual(gp.stretch_times(pts, 0.5)[1][0], 1.0)     # never faster than planned


class GripperTest(unittest.TestCase):
    def test_ramp_moves_at_most_the_speed_and_lands_on_the_target(self):
        v = 0.0
        for _ in range(100):
            nv = gp.gripper_ramp(v, 0.025, 0.05, 0.1)
            self.assertLessEqual(abs(nv - v), 0.005 + 1e-12)
            v = nv
        self.assertEqual(v, 0.025)
        self.assertEqual(gp.gripper_ramp(0.025, 0.0, 0.05, 0.1), 0.02)

    def test_closed_position_tells_object_from_empty(self):
        spec = gp.GraspSpec()
        self.assertEqual(gp.grasp_outcome(0.012, spec), "object")
        self.assertEqual(gp.grasp_outcome(0.001, spec), "empty")


if __name__ == "__main__":
    unittest.main()
