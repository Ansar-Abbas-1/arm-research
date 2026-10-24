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


class RuntimeLoadTargetTest(unittest.TestCase):
    """The steady-state target changed at run time (service / panel slider), clamped to [0, load_target_max]. Model of mine, not the motor."""

    @staticmethod
    def holding_world(target=38.5, contact=15.0, gain=40.0):
        w = World(obj=0.012, gain=gain)
        w.c = gc.GripperController(gp.CloseParams(load_target=target, contact_load=contact))
        w.c.request_close(w.t, w.pos)
        w.run(60)
        assert w.c.mode == "HOLDING", w.c.mode
        return w

    def test_the_value_is_clamped_and_a_nan_is_refused(self):
        c = gc.GripperController(gp.CloseParams())
        self.assertEqual(c.set_load_target(50.0)[0], 38.5)
        self.assertEqual(c.set_load_target(-5.0)[0], 0.0)
        with self.assertRaises(ValueError):
            c.set_load_target(float("nan"))

    def test_contact_load_follows_the_target_and_stays_under_it(self):
        c = gc.GripperController(gp.CloseParams(load_target=15.0, contact_load=5.0))        # the light-grip launch values
        for x in (38.5, 25.0, 15.0, 8.0, 3.0, 1.0, 0.0):
            applied, contact = c.set_load_target(x)
            self.assertAlmostEqual(applied, x)
            self.assertLess(contact, applied + 1e-12 if x == 0.0 else applied)               # never at or above the target (0: no loss detection)
            self.assertGreaterEqual(contact, 0.0)
            if x >= 3.0:
                self.assertGreaterEqual(contact, c.p.contact_floor)                          # never below the noise floor
        self.assertAlmostEqual(c.set_load_target(15.0)[1], 5.0)                              # the configured pair is reproduced at its own target
        self.assertAlmostEqual(gc.GripperController(gp.CloseParams()).set_load_target(38.5)[1], 15.0)

    def test_lowering_the_target_while_holding_relaxes_the_grip_without_a_fault_or_a_loss(self):
        w = self.holding_world()
        before = w.cmd
        w.c.set_load_target(15.0)
        w.run(40)
        self.assertEqual(w.c.mode, "HOLDING")
        self.assertEqual(w.c.grip, "held")
        self.assertLessEqual(abs(w.load - 15.0), w.c.p.load_band + 1.0)
        self.assertGreater(w.cmd, before)                                                    # the jaws opened a little
        self.assertLess(min(w.loads[-100:]), 15.0 + w.c.p.load_band + 1.0)

    def test_raising_the_target_while_holding_squeezes_harder_but_never_past_the_peak_limit(self):
        w = self.holding_world(target=15.0, contact=5.0)
        w.c.set_load_target(30.0)
        w.run(60)
        self.assertEqual(w.c.mode, "HOLDING")
        self.assertLessEqual(abs(w.load - 30.0), w.c.p.load_band + 1.0)
        self.assertLess(max(w.loads), w.c.p.load_limit)

    def test_the_command_does_not_jump_when_the_target_changes(self):
        w = self.holding_world()
        before = w.cmd
        w.c.set_load_target(10.0)
        w.tick()
        self.assertLessEqual(abs(w.cmd - before), w.c.p.trim_speed * DT + 1e-9)             # still rate-limited: no proportional kick

    def test_a_lost_grip_is_still_detected_after_the_target_was_lowered(self):
        w = self.holding_world()
        w.c.set_load_target(15.0)
        w.run(40)
        self.assertEqual(w.c.grip, "held")                                                   # a hold at the new target is not a loss
        w.override = 1.0
        w.run(5)
        self.assertEqual(w.c.grip, "lost")

    def test_a_target_set_while_idle_is_used_by_the_next_close(self):
        w = World(obj=0.012, gain=40.0)
        w.c.set_load_target(15.0)
        w.c.request_close(w.t, w.pos)
        w.run(60)
        self.assertEqual(w.c.mode, "HOLDING")
        self.assertLessEqual(abs(w.load - 15.0), w.c.p.load_band + 1.0)


class StepSafeTest(unittest.TestCase):
    def test_an_internal_error_freezes_the_gripper_and_releases_the_pending_action_instead_of_raising(self):
        w = World(obj=0.012)
        w.c.request_close(w.t, w.pos)
        w.run(1)

        def boom(*a, **k):
            raise TypeError("unsupported operand type(s) for -: 'float' and 'NoneType'")
        w.c.closer.step = boom
        out = w.c.step_safe(w.t + DT, w.pos, w.load, w.stamp, True)                   # must not raise
        self.assertIsNone(out.cmd)
        self.assertEqual((out.result.kind, out.result.ok), ("close", False))
        self.assertIn("internal error", out.result.message)
        self.assertEqual(w.c.mode, "FROZEN")
        self.assertIn("TypeError", w.c.last_error)

    def test_the_gripper_can_be_opened_again_after_an_internal_error(self):
        w = World(obj=None)
        w.pos = w.cmd = 0.0
        w.c.request_close(w.t, w.pos)
        w.c.closer.step = lambda *a, **k: 1 / 0
        w.c.step_safe(w.t + DT, w.pos, w.load, w.stamp, True)
        self.assertEqual(w.c.mode, "FROZEN")
        w.c.request_open(w.t, w.pos)
        w.run(10)
        self.assertEqual(w.c.mode, "IDLE")
        self.assertTrue(w.results[-1].ok and w.results[-1].kind == "open")


if __name__ == "__main__":
    unittest.main()
