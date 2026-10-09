#!/usr/bin/env python3
"""Unit tests of the grasp behavior tree with a fake context (no ROS master, no hardware). The gripper is a scripted fake of the
gripper_control node: how it regulates the load is tested in test_gripper_controller.py, here only how the tree uses it."""
import unittest

import py_trees
from py_trees.common import Status

from arm_grasp import grasp_plan as gp
from arm_grasp import grasp_tree as gt

DT = 0.05


class FakeContext(gt.GraspContext):
    """Scripted world: time advances DT per tick; trajectories and gripper actions take `traj_ticks` / `grip_ticks` ticks."""

    def __init__(self, traj_ticks=4, grip_ticks=3):
        self.t = 0.0
        self.states = []
        self.reason = None
        self.warnings = []
        self.violation = None
        self.drift = (0.0, 0.0)
        self.drift_error = None
        self.outcome = None
        self.final_load = None
        self.traj_ticks, self.traj = traj_ticks, None
        self.started, self.cancelled = [], 0
        self.traj_fail, self.start_error = {}, {}
        self.grip_ticks, self.grip_job = grip_ticks, None
        self.gripper_started, self.gripper_cancelled = [], 0
        self.gripper_fail, self.gripper_start_error = {}, {}
        self.close_outcome, self.close_load = "object", 38.0
        self.grip = "held"                                         # what the gripper_control node reports

    def now(self): return self.t
    def set_state(self, state, message=""): self.states.append(state) if not self.states or self.states[-1] != state else None
    def fail(self, reason): self.reason = self.reason or reason
    def warn(self, message): self.warnings.append(message)
    def safety_violation(self): return self.violation

    def start_gripper(self, action):
        self.gripper_started.append(action)
        self.grip_job = [action, self.grip_ticks]
        return self.gripper_start_error.get(action)

    def poll_gripper(self):
        action = self.grip_job[0]
        if action in self.gripper_fail:
            return "failed", self.gripper_fail[action], "", 0.0
        self.grip_job[1] -= 1
        if self.grip_job[1] > 0:
            return "running", "", "", 0.0
        return ("succeeded", "done", self.close_outcome if action == "close" else "open", self.close_load if action == "close" else 0.8)

    def cancel_gripper(self): self.gripper_cancelled += 1
    def grip_state(self): return self.grip

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
        self.assertEqual(ctx.gripper_started, ["open", "close"])
        self.assertIsNone(ctx.reason)
        self.assertEqual((ctx.outcome, ctx.final_load), ("object", 38.0))

    def test_the_tree_never_commands_the_gripper_itself_and_leaves_it_to_the_node_after_the_sequence(self):
        ctx, spec, root, tree = make()
        run(ctx, tree)
        self.assertEqual(ctx.gripper_cancelled, 0)                  # nothing stops the gripper at the end: its regulation goes on

    def test_estop_during_descent_cancels_and_never_closes(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "DESCENDING")
        ctx.violation = "e-stop engaged or its state unknown"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.cancelled, 1)
        self.assertIn("e-stop", ctx.reason)
        self.assertNotIn("CLOSING", ctx.states)
        self.assertEqual(ctx.gripper_started, ["open"])
        self.assertEqual(ctx.started[-1][0], "DESCEND")

    def test_stop_while_opening_cancels_the_gripper_action_and_starts_no_move(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: i == 1)
        ctx.violation = "stopped by the operator"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.started, [])
        self.assertEqual(ctx.gripper_cancelled, 1)                  # the open was running: it is stopped

    def test_estop_during_the_close_stops_the_gripper_action_and_does_not_lift(self):
        ctx, spec, root, tree = make(FakeContext(grip_ticks=40))
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "CLOSING")
        ctx.violation = "e-stop engaged or its state unknown"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.gripper_cancelled, 1)
        self.assertNotIn("LIFTING", ctx.states)

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

    def test_a_gripper_action_that_fails_stops_the_sequence(self):
        ctx, spec, root, tree = make()
        ctx.gripper_fail["close"] = "no fresh gripper load (none)"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("no fresh gripper load", ctx.reason)
        self.assertNotIn("LIFTING", ctx.states)
        ctx, spec, root, tree = make()
        ctx.gripper_start_error["open"] = "service missing"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.started, [])

    def test_fully_closed_below_the_load_target_is_thin_and_counts_as_grasped(self):
        ctx, spec, root, tree = make()
        ctx.close_outcome, ctx.close_load, ctx.grip = "thin", 3.0, "thin"
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.outcome, "thin")

    def test_require_object_fails_on_thin_and_does_not_lift(self):
        ctx, spec, root, tree = make(require_object=True)
        ctx.close_outcome, ctx.grip = "thin", "thin"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("require_object", ctx.reason)
        self.assertNotIn("LIFTING", ctx.states)

    def test_losing_the_grip_during_the_lift_aborts_and_cancels_the_lift(self):
        ctx, spec, root, tree = make(FakeContext(traj_ticks=30))
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "LIFTING")
        before = ctx.cancelled
        ctx.grip = "lost"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("grip was lost", ctx.reason)
        self.assertEqual(ctx.cancelled, before + 1)                 # the lift trajectory was cancelled

    def test_a_gripper_that_stops_regulating_during_the_lift_aborts_it(self):
        ctx, spec, root, tree = make(FakeContext(traj_ticks=30))
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "LIFTING")
        ctx.grip = "none"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("no longer regulating", ctx.reason)

    def test_missing_gripper_status_during_the_lift_aborts_it(self):
        ctx, spec, root, tree = make(FakeContext(traj_ticks=30))
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "LIFTING")
        ctx.grip = None
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("not being supervised", ctx.reason)

    def test_the_lift_succeeds_with_the_grip_held_throughout(self):
        ctx, spec, root, tree = make(FakeContext(traj_ticks=30))
        self.assertEqual(run(ctx, tree), Status.SUCCESS)

    def test_tree_is_reusable(self):
        ctx, spec, root, tree = make()
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        ctx.states.clear(); ctx.started.clear(); ctx.gripper_started.clear()
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.states[0], "OPENING")
        self.assertEqual(len(ctx.started), 3)
        self.assertEqual(ctx.gripper_started, ["open", "close"])
        ctx.violation = "e-stop"
        ctx.reason = None
        self.assertEqual(run(ctx, tree), Status.FAILURE)            # and a guard failure after a success works too

    def test_active_path(self):
        ctx, spec, root, tree = make()
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "DESCENDING")
        self.assertEqual(gt.active_path(root), "stop / e-stop guard/grasp/DESCEND")
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "LIFTING")
        self.assertEqual(gt.active_path(root), "stop / e-stop guard/grasp/lift while holding/LIFT")


if __name__ == "__main__":
    unittest.main()
