#!/usr/bin/env python3
"""Unit tests of the grasp sequence's position correction against a model arm with a constant offset (sag)."""
import unittest

import numpy as np

from arm_grasp.correction import PositionCorrection


class SaggingArm(object):
    """Reaches (aim + offset): the commanded position is missed by a constant offset."""

    def __init__(self, offset):
        self.offset = np.asarray(offset, float)
        self.pos = np.zeros(3)

    def move(self, aim):
        self.pos = np.asarray(aim, float) + self.offset


def run(arm, corr, goal):
    arm.move(goal)
    corr.begin()
    moves = 0
    for _ in range(10):
        action, aim, err = corr.step(goal, arm.pos)
        if action != "move":
            return action, err, moves
        arm.move(aim)
        moves += 1
    raise AssertionError("did not terminate")


class PositionCorrectionTest(unittest.TestCase):
    def test_a_constant_offset_is_removed_in_one_step(self):
        arm, corr, goal = SaggingArm([0.037, 0.0, -0.001]), PositionCorrection(steps=2), np.array([0.33, -0.05, -0.48])
        action, err, moves = run(arm, corr, goal)
        self.assertEqual((action, moves), ("ok", 1))
        self.assertLess(np.linalg.norm(arm.pos - goal), 1e-9)

    def test_within_the_trigger_nothing_moves(self):
        arm, corr = SaggingArm([0.004, 0, 0]), PositionCorrection(steps=2, trigger=0.006)
        self.assertEqual(run(arm, corr, np.zeros(3))[::2], ("ok", 0))

    def test_the_move_count_never_exceeds_the_steps(self):
        corr = PositionCorrection(steps=2, trigger=0.001, max_error=0.1)
        goal, meas, moves = np.zeros(3), np.array([0.03, 0, 0]), 0
        for _ in range(5):                                         # the arm never improves
            action, aim, err = corr.step(goal, meas)
            if action != "move":
                break
            moves += 1
        self.assertEqual(moves, 2)
        self.assertEqual(action, "ok")                             # 3 cm is within max_error 10 cm: accepted

    def test_a_residual_beyond_max_error_fails(self):
        corr = PositionCorrection(steps=1, trigger=0.001, max_error=0.02)
        goal, meas = np.zeros(3), np.array([0.05, 0, 0])
        self.assertEqual(corr.step(goal, meas)[0], "move")
        self.assertEqual(corr.step(goal, meas)[0], "fail")

    def test_the_compensation_is_capped(self):
        corr = PositionCorrection(steps=2, trigger=0.001, max_total=0.04)
        action, aim, err = corr.step(np.zeros(3), np.array([-0.2, 0, 0]))
        self.assertEqual(action, "move")
        self.assertAlmostEqual(np.linalg.norm(aim), 0.04)

    def test_the_compensation_carries_over_to_the_next_position_but_the_steps_do_not(self):
        arm, corr = SaggingArm([0.03, 0, 0]), PositionCorrection(steps=1)
        run(arm, corr, np.array([0.3, 0, -0.48]))
        comp = corr.comp.copy()
        corr.begin()
        self.assertEqual(corr.used, 0)
        np.testing.assert_allclose(corr.comp, comp)
        corr.reset()
        np.testing.assert_allclose(corr.comp, 0)

    def test_gain_scales_each_step(self):
        corr = PositionCorrection(steps=2, trigger=0.001, gain=0.5)
        action, aim, err = corr.step(np.zeros(3), np.array([-0.04, 0, 0]))
        np.testing.assert_allclose(aim, [0.02, 0, 0])


if __name__ == "__main__":
    unittest.main()
