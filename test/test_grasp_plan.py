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


class Plant(object):
    """Gripper whose closing is blocked at `obj` (a joint_6 position; None: it closes freely). The load is
    baseline + gain [%/mm] x (how far the command is past the block); the default gain is the first real measurement (about 80 %/mm), sampled at 10 Hz (the driver's rate); the
    controller runs at 20 Hz."""

    def __init__(self, obj=None, gain=80.0, base=0.8, params=None, start=0.025):
        self.obj, self.gain, self.base = obj, gain, base
        self.p = params or gp.CloseParams()
        self.closer = gp.LoadLimitedCloser(start, 0.0, 0.025, self.p)
        self.t, self.cmd, self.load, self.stamp, self.k = 0.0, start, base, 0.0, 0
        self.loads, self.cmds = [], []

    def measured(self):
        return self.cmd if self.obj is None else max(self.cmd, self.obj)

    def run(self, seconds, load_override=None):
        step = None
        for _ in range(int(seconds * 20)):
            self.t += 0.05
            self.k += 1
            if self.k % 2 == 0:                                                   # a new load sample at 10 Hz
                squeeze = 0.0 if self.obj is None else max(0.0, self.obj - self.cmd) * 1000.0
                self.load = self.base + self.gain * squeeze if load_override is None else load_override
                self.stamp = self.t
            step = self.closer.step(self.t, self.measured(), self.load, self.stamp)
            if step.fault or step.done:
                return step
            self.cmd = step.cmd
            self.loads.append(self.load)
            self.cmds.append(self.cmd)
        return step


class LoadLimitedCloserTest(unittest.TestCase):
    def test_closes_slowly_until_the_load_target_and_holds_there(self):
        pl = Plant(obj=0.012)
        step = pl.run(60)
        self.assertTrue(step.done)
        self.assertEqual(step.outcome, "object")
        self.assertLess(abs(pl.load - 40.0), 5.0 + 1e-9)                         # held in the band around the target
        speeds = [abs(b - a) / 0.05 for a, b in zip(pl.cmds, pl.cmds[1:])]
        self.assertLessEqual(max(speeds), pl.p.close_speed + 1e-9)               # never faster than the slow close
        self.assertGreater(pl.cmd, 0.0)                                           # stopped before fully closed
        self.assertLess(max(pl.loads), pl.p.load_abort)                           # never near the limit

    def test_fully_closed_below_the_load_target_is_thin(self):
        pl = Plant(obj=None)
        step = pl.run(60)
        self.assertTrue(step.done)
        self.assertEqual(step.outcome, "thin")
        self.assertEqual(pl.cmd, 0.0)

    def test_a_stiff_contact_overshoots_but_the_hold_relaxes_back_into_the_band(self):
        pl = Plant(obj=0.012, gain=240.0)                                         # 3x stiffer than the measured one
        step = pl.run(60)
        self.assertEqual(step.outcome, "object")
        self.assertLess(abs(pl.load - 40.0), 5.0 + 1e-9)
        self.assertLess(max(pl.loads), pl.p.load_abort)

    def _spike(self, confirm):
        pl = Plant(obj=None, params=gp.CloseParams(confirm_samples=confirm))
        for _ in range(40):                                                       # 2 s of closing, then one spike sample
            pl.run(0.05)
        pl.closer.step(pl.t + 0.05, pl.measured(), 60.0, pl.t + 0.05)
        return pl.closer.step(pl.t + 0.10, pl.measured(), 3.0, pl.t + 0.10)       # the next NEW sample is low again

    def test_with_two_confirm_samples_a_single_load_spike_does_not_stop_the_close(self):
        self.assertEqual(self._spike(2).phase, "CLOSING")

    def test_with_the_default_one_confirm_sample_the_first_load_at_target_stops_the_close(self):
        self.assertEqual(gp.CloseParams().confirm_samples, 1)
        self.assertEqual(self._spike(1).phase, "HOLDING")

    def test_no_or_stale_load_is_a_fault_and_the_command_does_not_move(self):
        pl = Plant(obj=0.012)
        step = pl.closer.step(1.0, 0.025, None, None)
        self.assertIn("no fresh gripper load", step.fault)
        self.assertEqual(step.cmd, 0.025)
        step = pl.closer.step(1.0, 0.025, 10.0, 0.2)                              # 0.8 s old, limit 0.5 s
        self.assertIn("no fresh gripper load", step.fault)
        self.assertEqual(step.cmd, 0.025)

    def test_a_load_that_stays_over_the_limit_fails_after_the_hold_timeout(self):
        pl = Plant(obj=0.012)
        step = pl.run(60, load_override=90.0)
        self.assertIn("stays at", step.fault)

    def test_the_command_stays_inside_the_open_and_closed_limits(self):
        for obj in (None, 0.012, 0.0):
            pl = Plant(obj=obj)
            pl.run(60)
            self.assertGreaterEqual(min(pl.cmds), 0.0)
            self.assertLessEqual(max(pl.cmds), 0.025)

    def test_require_nothing_in_the_closer_itself_thin_is_reported_not_judged(self):
        pl = Plant(obj=None, params=gp.CloseParams(closed_settle=0.2))
        self.assertEqual(pl.run(60).outcome, "thin")


if __name__ == "__main__":
    unittest.main()
