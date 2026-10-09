#!/usr/bin/env python3
"""Unit tests of the grasp behavior tree with a fake context (no ROS master, no hardware)."""
import unittest

import py_trees
from py_trees.common import Status

from arm_grasp import grasp_plan as gp
from arm_grasp import grasp_tree as gt

DT = 0.05
SPEEDS = {}


class FakeContext(gt.GraspContext):
    """Scripted world: time advances DT per tick, the gripper follows its command (or is blocked at `contact`) and reports a load,
    trajectories take `traj_ticks` ticks."""

    def __init__(self, traj_ticks=4):
        self.t = 0.0
        self.gripper = 0.0
        self.contact = None              # joint_6 position at which closing is blocked (None: closes freely)
        self.load_gain = 80.0            # % per mm the command is past the contact (first real measurement)
        self.no_load = False             # the load topic is silent
        self.cmd = 0.0
        self.cmds = []
        self.states = []
        self.reason = None
        self.warnings = []
        self.violation = None
        self.drift = (0.0, 0.0)
        self.drift_error = None
        self.outcome = None
        self.traj_ticks = traj_ticks
        self.traj = None
        self.started = []
        self.cancelled = 0
        self.traj_fail = {}
        self.start_error = {}

    def now(self): return self.t
    def set_state(self, state, message=""): self.states.append(state) if not self.states or self.states[-1] != state else None
    def fail(self, reason): self.reason = self.reason or reason
    def warn(self, message): self.warnings.append(message)
    def safety_violation(self): return self.violation
    def gripper_position(self): return self.gripper

    def command_gripper(self, position):
        self.cmds.append(position)
        self.cmd = position
        self.gripper = position if self.contact is None else max(position, self.contact)

    def gripper_load(self):
        if self.no_load:
            return None, None
        squeeze = 0.0 if self.contact is None else max(0.0, self.contact - self.cmd) * 1000.0
        return 0.8 + self.load_gain * squeeze, int(self.t / 0.1) * 0.1           # a new sample every 0.1 s

    def start_trajectory(self, name, slow):
        self.started.append((name, slow))
        self.traj = [name, self.traj_ticks]
        return self.start_error.get(name)

    def poll_trajectory(self):
        name = self.traj[0]
        if name in self.traj_fail:
            return "failed", self.traj_fail[name]
        self.traj[1] -= 1
        return ("succeeded", "") if self.traj[1] <= 0 else ("running", "")

    def cancel_trajectory(self): self.cancelled += 1

    def target_drift(self):
        if self.drift_error:
            raise RuntimeError(self.drift_error)
        return self.drift

    def record_outcome(self, outcome, load): self.outcome, self.final_load = outcome, load


def make(ctx=None, **pk):
    ctx = ctx or FakeContext()
    spec = gp.GraspSpec()
    params = gt.SequenceParams(**pk)
    root = gt.build_tree(ctx, spec, params, lambda: ctx.outcome)
    return ctx, spec, root, py_trees.trees.BehaviourTree(root)


def run(ctx, tree, max_ticks=2000, until=None):
    for i in range(max_ticks):
        tree.tick()
        ctx.t += DT
        if tree.root.status in (Status.SUCCESS, Status.FAILURE) or (until and until(i)):
            break
    return tree.root.status


class TestGraspTree(unittest.TestCase):
    def test_full_sequence_order_and_slow_factors(self):
        ctx, spec, root, tree = make()
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.states, ["OPENING", "MOVING_PRE", "RECHECK", "DESCENDING", "CLOSING", "LIFTING"])
        self.assertEqual(ctx.started, [("PRE", 1.0), ("DESCEND", spec.descent_slow_factor), ("LIFT", spec.lift_slow_factor)])
        self.assertIsNone(ctx.reason)
        self.assertEqual(ctx.outcome, "thin")                     # fully closed with the load below the target

    def test_opening_is_fast_and_closing_is_the_slow_load_limited_close(self):
        ctx, spec, root, tree = make()
        run(ctx, tree)
        steps = [b - a for a, b in zip(ctx.cmds, ctx.cmds[1:])]
        self.assertLessEqual(max(steps), 0.02 * DT + 1e-9)                         # opening ramp
        self.assertLessEqual(max(-x for x in steps), 0.001 * DT + 1e-9)            # closing at close_speed, much slower
        self.assertAlmostEqual(max(ctx.cmds), spec.gripper_open)
        self.assertAlmostEqual(ctx.cmds[-1], spec.gripper_closed)

    def test_estop_during_descent_cancels_and_never_closes(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "DESCENDING")
        ctx.violation = "e-stop engaged or its state unknown"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.cancelled, 1)
        self.assertIn("e-stop", ctx.reason)
        self.assertNotIn("CLOSING", ctx.states)
        self.assertEqual(ctx.started[-1][0], "DESCEND")

    def test_stop_while_opening_never_starts_a_move(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: i == 3)
        ctx.violation = "stopped by the operator"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.started, [])
        self.assertEqual(ctx.cancelled, 0)                        # no trajectory was running

    def test_first_failure_reason_wins(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "MOVING_PRE")
        ctx.violation = "stopped by the operator"
        ctx.traj_fail["PRE"] = "PRE failed: aborted"
        run(ctx, tree)
        self.assertEqual(ctx.reason, "stopped by the operator")

    def test_target_moved_aborts_before_descent(self):
        ctx, spec, root, tree = make()
        ctx.drift = (20.0, 0.0)
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("moved", ctx.reason)
        self.assertEqual([s[0] for s in ctx.started], ["PRE"])
        self.assertNotIn("DESCENDING", ctx.states)

    def test_rotation_drift_also_aborts(self):
        ctx, spec, root, tree = make()
        ctx.drift = (0.0, 10.0)
        self.assertEqual(run(ctx, tree), Status.FAILURE)

    def test_no_newer_observation_is_trusted_with_a_warning(self):
        ctx, spec, root, tree = make()
        ctx.drift = None
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(len(ctx.warnings), 1)

    def test_tf_failure_in_recheck_fails_closed(self):
        ctx, spec, root, tree = make()
        ctx.drift_error = "lookup failed"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("lookup failed", ctx.reason)
        self.assertNotIn("DESCENDING", ctx.states)

    def test_trajectory_failure_and_start_error(self):
        ctx, spec, root, tree = make()
        ctx.traj_fail["DESCEND"] = "DESCEND failed: aborted"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.reason, "DESCEND failed: aborted")
        self.assertNotIn("CLOSING", ctx.states)
        ctx, spec, root, tree = make()
        ctx.start_error["PRE"] = "server not available"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("server not available", ctx.reason)

    def test_load_reached_before_fully_closed_holds_it_and_the_arm_lifts(self):
        ctx, spec, root, tree = make()
        ctx.contact = 0.012
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.outcome, "object")
        self.assertLess(abs(ctx.final_load - 40.0), 5.0 + 1e-9)
        self.assertGreater(ctx.cmd, spec.gripper_closed)                           # stopped before fully closed
        self.assertEqual(ctx.states[-1], "LIFTING")

    def test_fully_closed_below_the_load_target_is_thin_and_counts_as_grasped(self):
        ctx, spec, root, tree = make()
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.outcome, "thin")

    def test_require_object_fails_on_thin_and_does_not_lift(self):
        ctx, spec, root, tree = make(require_object=True)
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("require_object", ctx.reason)
        self.assertNotIn("LIFTING", ctx.states)

    def test_no_load_measurement_fails_closed_before_the_lift(self):
        ctx, spec, root, tree = make()
        ctx.no_load = True
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("no fresh gripper load", ctx.reason)
        self.assertNotIn("LIFTING", ctx.states)
        self.assertEqual(ctx.started[-1][0], "DESCEND")

    def test_tree_is_reusable(self):
        ctx, spec, root, tree = make()
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        ctx.states.clear(); ctx.started.clear(); ctx.cmds.clear()
        ctx.gripper = 0.0
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.states[0], "OPENING")
        self.assertEqual(len(ctx.started), 3)
        ctx.violation = "e-stop"
        ctx.reason = None
        self.assertEqual(run(ctx, tree), Status.FAILURE)          # and a guard failure after a success works too

    def test_active_path(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "DESCENDING")
        self.assertEqual(gt.active_path(root), "stop / e-stop guard/grasp/DESCEND")


if __name__ == "__main__":
    unittest.main()
