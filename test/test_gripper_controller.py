#!/usr/bin/env python3
"""Unit tests of the gripper state machine (pure logic, driven against a model of mine: NOT the motor)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from arm_grasp import grasp_plan as gp  # noqa: E402
from arm_grasp import gripper_controller as gc  # noqa: E402

DT = 0.05


class World(object):
    """Gripper with a contact at `obj` (joint_6 position, None: closes freely); load = base + gain [%/mm] x command lead, sampled at
    10 Hz with `delay` samples of delay, as in test_grasp_plan.Plant."""

    def __init__(self, obj=0.012, gain=80.0, base=0.8, delay=1):
        self.obj, self.gain, self.base = obj, gain, base
        self.pos, self.cmd, self.t, self.k = 0.025, 0.025, 0.0, 0
        self.pipe = [base] * delay
        self.load, self.stamp, self.override = base, 0.0, None
        self.estop_ok = True
        self.c = gc.GripperController(gp.CloseParams())
        self.cmds, self.loads, self.results = [], [], []

    def measured(self):
        return self.cmd if self.obj is None else max(self.cmd, self.obj)

    def tick(self):
        self.t += DT
        self.k += 1
        if self.k % 2 == 0:
            squeeze = 0.0 if self.obj is None else max(0.0, self.obj - self.cmd) * 1000.0
            self.pipe.append(self.base + self.gain * squeeze if self.override is None else self.override)
            self.load, self.stamp = self.pipe.pop(0), self.t
        self.pos = self.measured()
        out = self.c.step(self.t, self.pos, self.load, self.stamp, self.estop_ok)
        if out.cmd is not None:
            self.cmd = out.cmd
        if out.result:
            self.results.append(out.result)
        self.cmds.append(out.cmd)
        self.loads.append(self.load)
        return out

    def run(self, seconds):
        for _ in range(int(seconds / DT)):
            self.tick()


class GripperControllerTest(unittest.TestCase):
    def test_open_completes_once_the_measurement_has_settled(self):
        w = World(obj=None)
        w.pos = w.cmd = 0.0
        w.c.request_open(w.t, w.pos)
        w.run(10)
        self.assertEqual([r.kind for r in w.results], ["open"])
        self.assertTrue(w.results[0].ok)
        self.assertEqual(w.c.mode, "IDLE")
        self.assertAlmostEqual(w.cmd, 0.025)

    def test_close_completes_and_then_keeps_regulating_instead_of_stopping(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        self.assertEqual([r.kind for r in w.results], ["close"])
        self.assertEqual(w.results[0].outcome, "object")
        self.assertEqual(w.c.mode, "HOLDING")                       # still in control after the action ended
        self.assertEqual(w.c.grip, "held")
        n = len(w.cmds)
        w.run(5)
        self.assertTrue(all(c is not None for c in w.cmds[n:]), "commands must keep being sent while holding")

    def test_the_hold_follows_a_slow_creep_of_the_object(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        for _ in range(8):                                          # the object gives 0.1 mm at a time: the load drops, the PI closes in
            w.obj -= 0.0001
            w.run(4)
        self.assertEqual(w.c.mode, "HOLDING")
        self.assertLessEqual(abs(w.load - w.c.p.load_target), w.c.p.load_band + 0.5)

    def test_losing_the_object_after_the_grasp_settled_is_grip_lost_and_the_command_freezes(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        held_cmd = w.cmd
        w.override = 0.6                                            # the load collapses: the object slipped out
        w.run(3)
        self.assertEqual(w.c.mode, "LOST")
        self.assertEqual(w.c.phase, "GRIP_LOST")
        self.assertEqual(w.c.grip, "lost")
        n = len(w.cmds)
        w.run(10)
        self.assertTrue(all(c is None for c in w.cmds[n:]), "no commands after the loss: the gripper must not creep closed")
        self.assertLessEqual(abs(w.cmd - held_cmd), 0.0005)

    def test_a_brief_load_dip_shorter_than_lost_time_is_not_a_loss(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        w.override = 0.6
        w.run(0.2)                                                  # two samples, under lost_time 0.3 s
        w.override = None
        w.run(5)
        self.assertEqual(w.c.mode, "HOLDING")

    def test_thin_is_held_as_thin_and_never_reported_lost(self):
        w = World(obj=None)
        w.c.request_close(w.t, w.pos)
        w.run(40)
        self.assertEqual(w.results[0].outcome, "thin")
        self.assertEqual(w.c.grip, "thin")
        w.run(10)
        self.assertEqual(w.c.mode, "HOLDING")

    def test_estop_freezes_without_sending_anything_and_fails_the_pending_action(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(2)
        w.estop_ok = False
        out = w.tick()
        self.assertIsNone(out.cmd)
        self.assertEqual(w.c.mode, "FROZEN")
        self.assertFalse(w.results[-1].ok)
        self.assertIn("e-stop", w.results[-1].message)

    def test_estop_while_holding_stops_the_regulation(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        w.estop_ok = False
        w.tick()
        self.assertEqual(w.c.mode, "FROZEN")
        self.assertEqual(w.c.grip, "none")

    def test_a_stale_load_freezes_the_gripper(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        out = w.c.step(w.t + 5.0, w.pos, w.load, w.stamp, True)     # the load stamp is 5 s old
        self.assertIsNone(out.cmd)
        self.assertEqual(w.c.mode, "FROZEN")

    def test_stop_interrupts_a_close_and_reports_it(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(2)
        r = w.c.request_stop("stopped by the operator")
        self.assertIsNotNone(r)
        self.assertEqual((r.kind, r.ok), ("close", False))
        self.assertEqual(w.c.mode, "FROZEN")

    def test_open_after_a_hold_releases_the_regulation(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        w.c.request_open(w.t, w.pos)
        self.assertEqual(w.c.mode, "OPENING")
        self.assertEqual(w.c.grip, "none")
        w.obj = None
        w.run(10)
        self.assertEqual(w.results[-1].kind, "open")
        self.assertTrue(w.results[-1].ok)

    def test_a_close_that_never_settles_times_out(self):
        w = World()
        w.c.timeout = 3.0
        w.c.request_close(w.t, w.pos)
        w.run(6)
        self.assertEqual(w.c.mode, "FROZEN")
        self.assertIn("timed out", w.results[-1].message)

    def test_close_may_be_requested_again_while_holding_to_regrip(self):
        w = World()
        w.c.request_close(w.t, w.pos)
        w.run(20)
        w.c.request_close(w.t, w.pos)
        self.assertEqual(w.c.mode, "CLOSING")
        w.run(20)
        self.assertEqual(w.c.mode, "HOLDING")
        self.assertEqual([r.kind for r in w.results], ["close", "close"])


if __name__ == "__main__":
    unittest.main()
