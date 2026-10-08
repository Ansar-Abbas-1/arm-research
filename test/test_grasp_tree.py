#!/usr/bin/env python3
"""Unit tests of the grasp behavior tree with a fake context (no ROS master, no hardware)."""
import unittest

import py_trees
from py_trees.common import Status

from arm_grasp import grasp_plan as gp
from arm_grasp import grasp_tree as gt

DT = 0.05


class FakeContext(gt.GraspContext):
    """Scripted world: time advances DT per tick, the gripper follows its command (or is blocked at `block_at`),
    trajectories take `traj_ticks` ticks."""

    def __init__(self, traj_ticks=4):
        self.t = 0.0
        self.gripper = 0.0
        self.block_at = None
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
        self.gripper = position if self.block_at is None else max(position, self.block_at)

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

    def record_outcome(self, outcome): self.outcome = outcome


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
        self.assertEqual(ctx.outcome, "empty")                    # the fake jaws close all the way

    def test_gripper_ramp_is_rate_limited_and_reaches_both_ends(self):
        ctx, spec, root, tree = make()
        run(ctx, tree)
        steps = [abs(b - a) for a, b in zip(ctx.cmds, ctx.cmds[1:])]
        self.assertLessEqual(max(steps), 0.02 * DT + 1e-9)
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

    def test_blocked_jaws_time_out_and_the_outcome_is_object(self):
        ctx, spec, root, tree = make()
        ctx.block_at = 0.012                                      # an object stops the jaws; opening is not blocked below it
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.outcome, "object")

    def test_require_object_fails_on_empty_and_does_not_lift(self):
        ctx, spec, root, tree = make(require_object=True)
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("require_object", ctx.reason)
        self.assertNotIn("LIFTING", ctx.states)

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
