"""Steady-state position correction for the grasp sequence (numpy, no ROS).

The arm sags under gravity at a far reach, so a commanded pose is reached with an offset (measured: joint 3 0.053 rad off at the PRE
pose, the end effector 3.7 cm away). The correction mirrors arm_trajectory's reach correction: after a move, measure the end-effector
error e = goal - measured, add it to a running compensation `comp` (gain, capped at max_total) and re-aim at goal + comp, up to `steps`
times per position. The compensation is kept across positions of one grasp: the offset is roughly constant over the short PRE / GRASP
distance, so the legs planned afterwards are re-planned with it.

    ok    the error is within `trigger`, or the steps are used up and it is still within `max_error`: carry on
    move  re-aim at `aim` (a corrective move)
    fail  the steps are used up and the error is still beyond `max_error`: the pose is not being reached
"""
import numpy as np


class PositionCorrection(object):
    def __init__(self, steps=2, trigger=0.006, gain=1.0, max_total=0.06, max_error=0.05):
        self.steps, self.trigger, self.gain, self.max_total, self.max_error = int(steps), trigger, gain, max_total, max_error
        self.comp = np.zeros(3)
        self.used = 0

    def reset(self):
        """A new grasp: forget the compensation."""
        self.comp = np.zeros(3)
        self.used = 0

    def begin(self):
        """A new position (PRE, then GRASP): the steps are counted per position, the compensation carries over."""
        self.used = 0

    def step(self, goal, measured):
        """-> (action, aim, error [m]); `aim` is the position to re-aim at when action is 'move', else None."""
        err = np.asarray(goal, float) - np.asarray(measured, float)
        d = float(np.linalg.norm(err))
        if d <= self.trigger:
            return "ok", None, d
        if self.used >= self.steps:
            return ("ok" if d <= self.max_error else "fail"), None, d
        comp = self.comp + self.gain * err
        n = float(np.linalg.norm(comp))
        if n > self.max_total:
            comp = comp * (self.max_total / n)
        self.comp = comp
        self.used += 1
        return "move", np.asarray(goal, float) + self.comp, d
