"""Behavior tree of the grasp sequence (py_trees 0.7). ROS-free: every ROS / hardware interaction goes through a
`GraspContext`, so the tree is unit-tested with a fake context and run by `grasp_sequencer_node.py` with the real one.

    Guarded "stop / e-stop"            (fails and interrupts the running leaf the moment stop or e-stop is seen)
    └─ Sequence "grasp"
       ├─ open gripper                 OPENING         (the gripper_control node ramps it open)
       ├─ PRE                          MOVING_PRE      (joint-space move above the target)
       ├─ correct PRE *                CORRECTING      (re-measure the end effector, re-aim up to `steps` times: gravity sag)
       ├─ re-plan DESCEND, LIFT *      REPLANNING      (from the actual joints, with the learned offset; abort before descending if infeasible)
       ├─ recheck target               RECHECK         (fails if the target moved since planning)
       ├─ DESCEND                      DESCENDING      (straight down, slowed)
       ├─ correct GRASP *              CORRECTING
       ├─ close on load                CLOSING
       ├─ re-plan LIFT *               REPLANNING      (from the pose the arm really holds)
       ├─ close on load                CLOSING         (the gripper_control node closes slowly to the load target and keeps regulating it)
       └─ Parallel "lift while holding"
          ├─ LIFT                      LIFTING         (straight up, slowed)
          └─ grip watch                                (fails if the grip is lost or no longer regulated)

(*) only when correction steps are enabled (SequenceParams.correction): grasp_plan's open loop leaves the end effector off by the arm's gravity
sag (measured on the hardware: joint 3 0.053 rad, 3.7 cm at the PRE pose), see correction.PositionCorrection.
The gripper is owned by the gripper_control node: the leaves only ask it to open / close and watch its status. Its load PI keeps
running after the close, through the lift and after the sequence ends, until the gripper is opened or stopped.
The leaves publish their state name through `ctx.set_state`, which is what `~status` reports. A leaf that fails calls
`ctx.fail(reason)`; the first reason wins and becomes the ABORTED message. Interrupting a running leaf (guard failure, tree
stop, a failed sibling) cancels its trajectory goal or gripper action in `terminate`.
"""
import numpy as np
import py_trees
from py_trees.common import Status



class SequenceParams(object):
    """Thresholds of the leaves (the geometry and heights live in `grasp_plan.GraspSpec`, the gripper's law in the gripper_control node)."""

    def __init__(self, recheck_mm=5.0, recheck_deg=3.0, require_object=False, correction=None, correction_settle=0.8):
        self.correction = correction                # a correction.PositionCorrection, or None / steps == 0: no correction leaves at all
        self.correction_settle = correction_settle  # s to wait after a move before measuring the end effector
        self.recheck_mm = recheck_mm
        self.recheck_deg = recheck_deg
        self.require_object = require_object        # 'thin' (closed completely below the load target) fails instead of counting as grasped


class GraspContext(object):
    """Interface the leaves use. The node implements it with ROS; tests with a fake."""

    probe = False       # True: a SAG PROBE: PRE, GRASP and LIFT are moved to and measured, the gripper is never commanded (see build_tree)

    def log_measurement(self, key, step, measured):
        """One raw end-effector measurement of position `key` (PRE | GRASP | LIFT); `step` 0 is the first, before any corrective move. Default: ignore."""

    def now(self):
        """Monotonic time in seconds."""
        raise NotImplementedError

    def set_state(self, state, message=""):
        """Report the current step (OPENING, MOVING_PRE, ...)."""
        raise NotImplementedError

    def fail(self, reason):
        """Record why the sequence failed (the first call wins)."""
        raise NotImplementedError

    def warn(self, message):
        raise NotImplementedError

    def safety_violation(self):
        """None when it is safe to continue, else the reason (operator stop, e-stop engaged or unknown)."""
        raise NotImplementedError

    def start_gripper(self, action):
        """Ask the gripper_control node to 'open' or 'close' (the load-limited close). Returns None or an error message."""
        raise NotImplementedError

    def poll_gripper(self):
        """('running' | 'succeeded' | 'failed', message, outcome, load [%]); outcome is 'object' or 'thin' for a close."""
        raise NotImplementedError

    def cancel_gripper(self):
        """Stop the running gripper action (and the load regulation: the command stays where it is)."""
        raise NotImplementedError

    def grip_state(self):
        """The node's view of the grip: 'held', 'thin', 'lost', 'none', or None when its status is missing or stale."""
        raise NotImplementedError

    def start_trajectory(self, name, slow):
        """Send the planned trajectory `name` stretched by `slow`. Returns None or an error message."""
        raise NotImplementedError

    def poll_trajectory(self):
        """('running' | 'succeeded' | 'failed', message). The implementation owns the timeout and the endpoint check."""
        raise NotImplementedError

    def cancel_trajectory(self):
        raise NotImplementedError

    def measure_ee(self):
        """The measured end-effector position in the base frame (numpy xyz), or None."""
        raise NotImplementedError

    def waypoint(self, key):
        """The planned (uncompensated) position of 'PRE', 'GRASP' or 'LIFT' (numpy xyz)."""
        raise NotImplementedError

    def start_corrective(self, key, aim):
        """Plan from the current joints to `aim` at the waypoint `key`'s orientation and start it. Returns None or an error message."""
        raise NotImplementedError

    def replan_legs(self, comp, legs):
        """Re-plan the named legs ('DESCEND', 'LIFT' or just 'LIFT') from the CURRENT joints with `comp` added to their waypoints, check them
        like the original plan, and store them. Returns None or the problems as text."""
        raise NotImplementedError

    def target_drift(self):
        """(translation mm, rotation deg) of the target against the planning pose, or None when there is no newer
        observation than the planning one. Raises if the TF is unavailable."""
        raise NotImplementedError

    def record_outcome(self, outcome, load):
        """'object' (load target reached and held) or 'thin' (closed completely below it), with the final load [%]."""
        raise NotImplementedError


class Guarded(py_trees.decorators.Decorator):
    """Checks the context's safety condition before every tick of the child. On a violation the child is interrupted
    (its running leaf gets `terminate(INVALID)`, which cancels the motion) and this node FAILS, so no leaf ever runs
    a step after a stop or an e-stop."""

    def __init__(self, child, ctx, name="stop / e-stop guard"):
        super(Guarded, self).__init__(child=child, name=name)
        self.ctx = ctx

    def tick(self):
        reason = self.ctx.safety_violation()
        if reason:
            self.ctx.fail(reason)
            self.feedback_message = reason
            self.stop(Status.FAILURE)
            yield self
            return
        for node in super(Guarded, self).tick():
            yield node

    def update(self):
        return self.decorated.status


class ModeGate(py_trees.decorators.Decorator):
    """Runs the child only in the right mode: `run_when_probe` True = only in a sag probe, False = only in a real grasp. In the other mode it succeeds
    at once and never touches the child (no gripper command, no lift in a probe)."""

    def __init__(self, child, ctx, run_when_probe):
        super(ModeGate, self).__init__(child=child, name="%s (%s only)" % (child.name, "probe" if run_when_probe else "real grasp"))
        self.ctx, self.run_when_probe = ctx, bool(run_when_probe)

    def tick(self):
        if bool(self.ctx.probe) != self.run_when_probe:
            self.stop(Status.SUCCESS)
            yield self
            return
        for node in super(ModeGate, self).tick():
            yield node

    def update(self):
        return self.decorated.status


class _Leaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, ctx):
        super(_Leaf, self).__init__(name=name)
        self.ctx = ctx

    def _state(self, state, message):
        self.ctx.set_state(state, message() if callable(message) else message)


class GripperAction(_Leaf):
    """Asks the gripper_control node to open, or to close on load, and waits for the action to end. The node keeps regulating the
    load after a close; this leaf only reports the outcome ('object' | 'thin') and, with require_object, fails on 'thin'."""

    def __init__(self, name, ctx, params, action, state, message):
        super(GripperAction, self).__init__(name, ctx)
        self.params, self.action, self.state, self.message = params, action, state, message
        self.error = None

    def initialise(self):
        self._state(self.state, self.message)
        self.error = self.ctx.start_gripper(self.action)

    def update(self):
        if self.error:
            self.ctx.fail("gripper %s: %s" % (self.action, self.error))
            return Status.FAILURE
        status, message, outcome, load = self.ctx.poll_gripper()
        if status == "running":
            return Status.RUNNING
        if status == "failed":
            self.feedback_message = message
            self.ctx.fail(message)
            return Status.FAILURE
        if self.action == "close":
            self.ctx.record_outcome(outcome, load)
            if outcome == "thin" and self.params.require_object:
                self.ctx.fail("closed completely without reaching the load target and require_object is set")
                return Status.FAILURE
        return Status.SUCCESS

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.status == Status.RUNNING:
            self.ctx.cancel_gripper()


class CorrectPosition(_Leaf):
    """After a move: wait for the arm to settle, measure the end effector against the planned waypoint, and re-aim (a corrective move
    planned from the actual joints) up to correction.steps times. The compensation it learns is kept for the later legs."""

    def __init__(self, ctx, params, key):
        super(CorrectPosition, self).__init__("correct %s" % key, ctx)
        self.params, self.key = params, key
        self.phase, self.t = "settle", 0.0

    def initialise(self):
        self.params.correction.begin()
        self.phase, self.t = "settle", self.ctx.now()
        self.ctx.set_state("CORRECTING", "%s: measuring the end-effector error" % self.key)

    def update(self):
        c = self.params.correction
        if self.phase == "moving":
            status, message = self.ctx.poll_trajectory()
            if status == "running":
                return Status.RUNNING
            if status == "failed":
                self.ctx.fail(message)
                return Status.FAILURE
            self.phase, self.t = "settle", self.ctx.now()
        if self.phase == "settle":
            if self.ctx.now() - self.t < self.params.correction_settle:
                return Status.RUNNING
        measured = self.ctx.measure_ee()
        if measured is None:
            self.ctx.fail("cannot measure the end effector for the %s correction" % self.key)
            return Status.FAILURE
        self.ctx.log_measurement(self.key, c.used, measured)
        if self.ctx.probe:                                              # a sag probe only MEASURES: the raw sag is the data, a corrective move would hide it
            self.ctx.set_state("CORRECTING", "%s: sag probe, measured %.1f mm from the waypoint" % (
                self.key, 1000.0 * float(np.linalg.norm(np.asarray(self.ctx.waypoint(self.key), float) - np.asarray(measured, float)))))
            return Status.SUCCESS
        action, aim, err = c.step(self.ctx.waypoint(self.key), measured)
        if action == "ok":
            self.ctx.set_state("CORRECTING", "%s: %.1f mm off after %d correction step(s)" % (self.key, 1000.0 * err, c.used))
            return Status.SUCCESS
        if action == "fail":
            self.ctx.fail("%s not reached: %.1f mm off after %d correction steps (limit %.0f mm)" % (self.key, 1000.0 * err, c.used, 1000.0 * c.max_error))
            return Status.FAILURE
        problem = self.ctx.start_corrective(self.key, aim)
        if problem:
            self.ctx.fail("%s correction step %d: %s" % (self.key, c.used, problem))
            return Status.FAILURE
        self.ctx.set_state("CORRECTING", "%s: step %d, %.1f mm off, re-aiming" % (self.key, c.used, 1000.0 * err))
        self.phase = "moving"
        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.status == Status.RUNNING and self.phase == "moving":
            self.ctx.cancel_trajectory()


class ReplanLegs(_Leaf):
    """Re-plans the named legs from the actual joints with the learned compensation; fails (before anything moves) if they are not
    feasible. With no compensation learned there is nothing to do."""

    def __init__(self, ctx, params, legs, name):
        super(ReplanLegs, self).__init__(name, ctx)
        self.params, self.legs = params, legs

    def update(self):
        comp = self.params.correction.comp
        if float((comp ** 2).sum()) ** 0.5 < 1e-4:
            return Status.SUCCESS
        self.ctx.set_state("REPLANNING", "%s with the %.1f mm compensation" % (" and ".join(self.legs), 1000.0 * float((comp ** 2).sum()) ** 0.5))
        problem = self.ctx.replan_legs(comp, self.legs)
        if problem:
            self.ctx.fail("re-planning %s with the compensation failed: %s" % (" and ".join(self.legs), problem))
            return Status.FAILURE
        return Status.SUCCESS


class GripWatch(_Leaf):
    """Runs alongside the lift: RUNNING while the gripper_control node reports the grip held (or 'thin', counted as grasped), FAILURE if
    it is lost, no longer regulated or its status is missing. It never succeeds: the lift ending ends the Parallel."""

    def __init__(self, ctx):
        super(GripWatch, self).__init__("grip watch", ctx)

    def update(self):
        grip = self.ctx.grip_state()
        if grip in ("held", "thin"):
            return Status.RUNNING
        reason = {"lost": "the grip was lost during the lift", "none": "the gripper is no longer regulating its grip",
                  None: "no gripper_control status: the grip is not being supervised"}.get(grip, "unexpected grip state %r" % grip)
        self.ctx.fail(reason)
        return Status.FAILURE


class FollowTrajectory(_Leaf):
    """Runs the planned trajectory `key` (stretched by `slow`) and follows it to its end."""

    def __init__(self, name, ctx, key, slow, state, message):
        super(FollowTrajectory, self).__init__(name, ctx)
        self.key, self.slow, self.state, self.message = key, slow, state, message
        self.error = None

    def initialise(self):
        self._state(self.state, self.message)
        self.error = self.ctx.start_trajectory(self.key, self.slow)

    def update(self):
        if self.error:
            self.ctx.fail("%s: %s" % (self.key, self.error))
            return Status.FAILURE
        status, message = self.ctx.poll_trajectory()
        if status == "running":
            return Status.RUNNING
        if status == "succeeded":
            return Status.SUCCESS
        self.feedback_message = message
        self.ctx.fail(message)
        return Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.status == Status.RUNNING:
            self.ctx.cancel_trajectory()


class RecheckTarget(_Leaf):
    """Fails if the target moved since it was planned. With no newer observation (the target is out of view from the
    pre-grasp pose) there is nothing to compare: the plan is trusted, with a warning."""

    def __init__(self, ctx, params):
        super(RecheckTarget, self).__init__("recheck target", ctx)
        self.params = params

    def update(self):
        self.ctx.set_state("RECHECK", "is the target where it was planned?")
        try:
            drift = self.ctx.target_drift()
        except Exception as e:  # noqa: BLE001  (TF lookup failure and the like: fail closed)
            self.ctx.fail("cannot re-check the target: %s" % e)
            return Status.FAILURE
        if drift is None:
            self.ctx.warn("no newer observation of the target before the descent (it is not visible from here): trusting the plan")
            return Status.SUCCESS
        mm, deg = drift
        if mm > self.params.recheck_mm or deg > self.params.recheck_deg:
            self.ctx.fail("the target moved by %.1f mm / %.1f deg since it was planned" % (mm, deg))
            return Status.FAILURE
        return Status.SUCCESS


def build_tree(ctx, spec, params, outcome_text=lambda: ""):
    """The grasp tree. `outcome_text` returns the text for the LIFTING message (the recorded outcome).
    In a sag PROBE (ctx.probe) the gripper actions and the grip-watched lift are skipped and a plain LIFT that is measured too takes their place:
    the arm goes through PRE, GRASP and LIFT in free air, every position measured, nothing gripped."""
    corrected = params.correction is not None and params.correction.steps > 0
    lift = py_trees.composites.Parallel(name="lift while holding", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, children=[
        FollowTrajectory("LIFT", ctx, "LIFT", spec.lift_slow_factor, "LIFTING", lambda: "grasp outcome: %s" % outcome_text()),
        GripWatch(ctx),
    ])
    children = [
        ModeGate(GripperAction("open gripper", ctx, params, "open", "OPENING", "gripper to %.3f" % spec.gripper_open), ctx, False),
        FollowTrajectory("PRE", ctx, "PRE", 1.0, "MOVING_PRE", "above the target"),
    ]
    if corrected:
        children += [CorrectPosition(ctx, params, "PRE"), ReplanLegs(ctx, params, ("DESCEND", "LIFT"), "re-plan DESCEND, LIFT")]
    children += [
        RecheckTarget(ctx, params),
        FollowTrajectory("DESCEND", ctx, "DESCEND", spec.descent_slow_factor, "DESCENDING",
                         "straight down, %.1f times slower than planned" % spec.descent_slow_factor),
    ]
    if corrected:
        children += [CorrectPosition(ctx, params, "GRASP")]
    children += [ModeGate(GripperAction("close on load", ctx, params, "close", "CLOSING", "the gripper closes to its load target and keeps regulating it"), ctx, False)]
    if corrected:
        children += [ReplanLegs(ctx, params, ("LIFT",), "re-plan LIFT")]
    children += [ModeGate(lift, ctx, False)]
    probe_lift = [FollowTrajectory("LIFT", ctx, "LIFT", spec.lift_slow_factor, "LIFTING", "sag probe: the lift, measured")]
    if corrected:
        probe_lift += [CorrectPosition(ctx, params, "LIFT")]
    children += [ModeGate(py_trees.composites.Sequence(name="probe lift", children=probe_lift), ctx, True)]
    return Guarded(py_trees.composites.Sequence(name="grasp", children=children), ctx)


def active_path(root):
    """Names of the RUNNING nodes from the root down, e.g. 'stop / e-stop guard/grasp/DESCEND'."""
    names, node = [], root
    while node is not None and node.status == Status.RUNNING:
        if not isinstance(node, ModeGate):                         # a mode gate only wraps its child: it adds nothing to the path
            names.append(node.name)
        node = next((c for c in node.children if c.status == Status.RUNNING), None)
    return "/".join(names)
