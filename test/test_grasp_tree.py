#!/usr/bin/env python3
"""Unit tests of the grasp behavior tree with a fake context (no ROS master, no hardware). The gripper is a scripted fake of the
gripper_control node: how it regulates the load is tested in test_gripper_controller.py, here only how the tree uses it."""
import unittest

import numpy as np
import py_trees
from py_trees.common import Status

from arm_grasp import grasp_plan as gp
from arm_grasp import grasp_tree as gt
from arm_grasp.correction import PositionCorrection

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
        # correction: the arm reaches (aim + offset), i.e. it misses its aim by `offset` (gravity sag)
        self.offset = np.zeros(3)
        self.waypoints = {"PRE": np.array([0.330, -0.052, -0.480]), "GRASP": np.array([0.338, -0.056, -0.560]), "LIFT": np.array([0.325, -0.050, -0.430])}
        self.ee_aim, self.comp = None, np.zeros(3)
        self.arm_responds, self.measure_none = True, False
        self.corrective, self.replans = [], []
        self.corrective_error, self.replan_problem = None, None
        self.measurements = []                                       # (key, step, measured xyz) as the leaves log them

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
        if name == "PRE":
            self.ee_aim = self.waypoints["PRE"].copy()
        elif name == "DESCEND":
            self.ee_aim = self.waypoints["GRASP"] + self.comp
        elif name == "LIFT":
            self.ee_aim = self.waypoints["LIFT"] + self.comp
        return self.start_error.get(name)

    def measure_ee(self):
        return None if self.measure_none else self.ee_aim + self.offset

    def waypoint(self, key): return self.waypoints[key]
    def log_measurement(self, key, step, measured): self.measurements.append((key, step, np.array(measured)))

    def start_corrective(self, key, aim):
        self.corrective.append((key, np.array(aim)))
        if self.arm_responds:
            self.ee_aim = np.array(aim)
        self.traj = ["CORR_" + key, self.traj_ticks]
        return self.corrective_error

    def replan_legs(self, comp, legs):
        self.replans.append((tuple(legs), np.array(comp)))
        if not self.replan_problem:
            self.comp = np.array(comp)
        return self.replan_problem

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


def make(ctx=None, correction_steps=0, **pk):
    ctx = ctx or FakeContext()
    spec = gp.GraspSpec()
    if correction_steps:
        pk["correction"], pk["correction_settle"] = PositionCorrection(steps=correction_steps), 0.2
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

    def test_after_the_lift_the_arm_returns_to_the_viewing_pose_and_the_gripper_is_left_alone(self):
        ctx, spec, root, tree = make(return_to_view=True)
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.states, ["OPENING", "MOVING_PRE", "RECHECK", "DESCENDING", "CLOSING", "LIFTING", "RETURNING"])
        self.assertEqual(ctx.started[-2:], [("LIFT", spec.lift_slow_factor), ("VIEW", 1.0)])
        self.assertEqual(ctx.gripper_started, ["open", "close"])
        self.assertEqual(ctx.gripper_cancelled, 0)

    def test_without_the_setting_there_is_no_return(self):
        ctx, spec, root, tree = make()
        run(ctx, tree)
        self.assertNotIn("VIEW", [n for n, _ in ctx.started])

    def test_a_failed_lift_does_not_return(self):
        ctx, spec, root, tree = make(return_to_view=True)
        ctx.traj_fail["LIFT"] = "lift failed"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertNotIn("VIEW", [n for n, _ in ctx.started])

    def test_a_lost_grip_during_the_lift_does_not_return(self):
        ctx, spec, root, tree = make(return_to_view=True)
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "LIFTING")
        ctx.grip = "lost"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertNotIn("VIEW", [n for n, _ in ctx.started])

    def test_a_failure_of_the_return_aborts_with_its_reason(self):
        ctx, spec, root, tree = make(return_to_view=True)
        ctx.traj_fail["VIEW"] = "VIEW failed: blocked"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("VIEW", ctx.reason)

    def test_a_stop_during_the_return_cancels_it(self):
        ctx, spec, root, tree = make(return_to_view=True)
        run(ctx, tree, until=lambda i: ctx.states and ctx.states[-1] == "RETURNING")
        ctx.violation = "stop requested"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertGreaterEqual(ctx.cancelled, 1)

    def test_a_probe_never_returns_to_view(self):
        ctx = FakeContext(); ctx.probe = True
        ctx, spec, root, tree = make(ctx, return_to_view=True)
        run(ctx, tree)
        self.assertNotIn("VIEW", [n for n, _ in ctx.started])

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


class TestCorrection(unittest.TestCase):
    def test_a_sag_is_corrected_and_the_later_legs_are_replanned_with_the_learned_offset(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.037, 0.0, -0.001])                 # the arm misses its aim by 3.7 cm (measured on the hardware)
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual([k for k, _ in ctx.corrective], ["PRE"])   # one corrective move at PRE, none needed after the re-planned descent
        np.testing.assert_allclose(ctx.corrective[0][1], ctx.waypoints["PRE"] - ctx.offset, atol=1e-9)
        self.assertEqual([legs for legs, _ in ctx.replans], [("DESCEND", "LIFT"), ("LIFT",)])
        for _, comp in ctx.replans:
            np.testing.assert_allclose(comp, -ctx.offset, atol=1e-9)
        self.assertEqual([n for n, _ in ctx.started], ["PRE", "DESCEND", "LIFT"])
        self.assertIn("CORRECTING", ctx.states)
        self.assertIn("REPLANNING", ctx.states)

    def test_the_order_is_move_correct_replan_recheck_descend_correct_close_replan_lift(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.02, 0.0, 0.0])
        run(ctx, tree)
        self.assertEqual(ctx.states, ["OPENING", "MOVING_PRE", "CORRECTING", "REPLANNING", "RECHECK", "DESCENDING", "CORRECTING", "CLOSING", "REPLANNING", "LIFTING"])

    def test_no_error_means_no_corrective_move_and_no_replanning(self):
        ctx, spec, root, tree = make(correction_steps=2)
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual((ctx.corrective, ctx.replans), ([], []))
        self.assertNotIn("REPLANNING", ctx.states)

    def test_correction_steps_zero_keeps_the_original_sequence(self):
        ctx, spec, root, tree = make(correction_steps=0)
        ctx.offset = np.array([0.037, 0.0, 0.0])
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.states, ["OPENING", "MOVING_PRE", "RECHECK", "DESCENDING", "CLOSING", "LIFTING"])

    def test_an_arm_that_does_not_respond_fails_before_the_descent(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.08, 0.0, 0.0])                     # 8 cm off and the corrective moves have no effect
        ctx.arm_responds = False
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("not reached", ctx.reason)
        self.assertEqual(len(ctx.corrective), 2)                    # exactly the two steps
        self.assertNotIn("DESCEND", [n for n, _ in ctx.started])

    def test_a_replan_that_is_not_feasible_aborts_before_descending(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.037, 0.0, 0.0])
        ctx.replan_problem = "DESCEND: not reachable (goal pose did not converge)"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("re-planning", ctx.reason)
        self.assertNotIn("DESCENDING", ctx.states)
        self.assertEqual(ctx.gripper_started, ["open"])             # the gripper was never asked to close

    def test_no_end_effector_measurement_fails_closed(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.measure_none = True
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("cannot measure", ctx.reason)
        self.assertNotIn("DESCENDING", ctx.states)

    def test_a_corrective_move_that_cannot_be_planned_fails(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.037, 0.0, 0.0])
        ctx.corrective_error = "PRE: not reachable"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertIn("correction step 1", ctx.reason)

    def test_an_estop_during_a_corrective_move_cancels_it(self):
        ctx, spec, root, tree = make(FakeContext(traj_ticks=30), correction_steps=2)
        ctx.offset = np.array([0.037, 0.0, 0.0])
        run(ctx, tree, until=lambda i: bool(ctx.corrective))
        before = ctx.cancelled
        ctx.violation = "e-stop engaged or its state unknown"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.cancelled, before + 1)

    def test_the_offset_after_the_descent_is_corrected_too(self):
        class Sagging(FakeContext):
            def start_trajectory(self, name, slow):
                out = super(Sagging, self).start_trajectory(name, slow)
                if name == "DESCEND":
                    self.offset = np.array([0.0, 0.0, 0.012])                    # from the low pose on the arm sits 1.2 cm high (more sag there)
                return out
        ctx, spec, root, tree = make(Sagging(), correction_steps=2)
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual([k for k, _ in ctx.corrective], ["GRASP"])
        np.testing.assert_allclose(ctx.corrective[0][1][2], ctx.waypoints["GRASP"][2] - 0.012, atol=1e-9)
        self.assertEqual([legs for legs, _ in ctx.replans], [("LIFT",)])        # only the lift is re-planned (nothing to re-plan before the descent)


class TestProbe(unittest.TestCase):
    """The sag probe: PRE, GRASP and LIFT are moved to and measured in free air; the gripper is never commanded."""

    def test_a_real_grasp_logs_its_first_measurements_of_pre_and_grasp(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.offset = np.array([0.0, 0.0, -0.03])
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        keys = [(k, step) for k, step, _ in ctx.measurements]
        self.assertEqual(keys[0], ("PRE", 0))
        self.assertIn(("GRASP", 0), keys)
        self.assertNotIn("LIFT", [k for k, _, _ in ctx.measurements])      # a real run does not measure the lift
        self.assertEqual(ctx.gripper_started, ["open", "close"])

    def test_a_probe_never_touches_the_gripper_and_measures_pre_grasp_and_lift(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.probe = True
        ctx.offset = np.array([0.0, 0.0, -0.03])
        self.assertEqual(run(ctx, tree), Status.SUCCESS)
        self.assertEqual(ctx.gripper_started, [])
        self.assertEqual(ctx.gripper_cancelled, 0)
        self.assertEqual(ctx.corrective, [])                                 # measured, never corrected: the raw sag is what the probe is for
        self.assertNotIn("OPENING", ctx.states)
        self.assertNotIn("CLOSING", ctx.states)
        self.assertEqual([n for n, _ in ctx.started if not n.startswith("CORR")], ["PRE", "DESCEND", "LIFT"])
        first = {k: m for k, step, m in ctx.measurements if step == 0}
        self.assertEqual(sorted(first), ["GRASP", "LIFT", "PRE"])
        for k in first:                                                      # the fake arm sags 3 cm: the raw first measurement shows it
            self.assertAlmostEqual(first[k][2] - ctx.waypoints[k][2], -0.03, delta=0.031)

    def test_a_probe_still_stops_on_a_safety_violation(self):
        ctx, spec, root, tree = make(correction_steps=2)
        ctx.probe = True
        ctx.violation = "e-stop engaged"
        self.assertEqual(run(ctx, tree), Status.FAILURE)
        self.assertEqual(ctx.started, [])


if __name__ == "__main__":
    unittest.main()
