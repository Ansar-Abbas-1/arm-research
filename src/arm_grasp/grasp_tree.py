"""Behavior tree of the grasp sequence (py_trees 0.7). ROS-free: every ROS / hardware interaction goes through a
`GraspContext`, so the tree is unit-tested with a fake context and run by `grasp_sequencer_node.py` with the real one.

    Guarded "stop / e-stop"            (fails and interrupts the running leaf the moment stop or e-stop is seen)
    └─ Sequence "grasp"
       ├─ open gripper                 OPENING
       ├─ PRE                          MOVING_PRE      (joint-space move above the target)
       ├─ recheck target               RECHECK         (fails if the target moved since planning)
       ├─ DESCEND                      DESCENDING      (straight down, slowed)
       ├─ close on load                CLOSING         (slow close until the motor load reaches the target, then hold)
       └─ LIFT                         LIFTING         (straight up, slowed)

The leaves publish their state name through `ctx.set_state`, which is what `~status` reports. A leaf that fails calls
`ctx.fail(reason)`; the first reason wins and becomes the ABORTED message. Interrupting a running trajectory leaf
(guard failure, tree stop) cancels the trajectory goal in `terminate`.
"""
import py_trees
from py_trees.common import Status

from arm_grasp import grasp_plan as gp


class SequenceParams(object):
    """Timing and threshold parameters of the leaves (the geometry and heights live in `grasp_plan.GraspSpec`)."""

    def __init__(self, gripper_speed=0.02, gripper_settle=0.6, gripper_tolerance=0.002, gripper_timeout=5.0,
                 recheck_mm=5.0, recheck_deg=3.0, require_object=False, close=None, max_dt=0.2):
        self.gripper_speed = gripper_speed          # m/s ramp of the joint_6 command
        self.gripper_settle = gripper_settle        # s to wait once the measurement is at the target
        self.gripper_tolerance = gripper_tolerance  # m
        self.gripper_timeout = gripper_timeout      # s after the ramp ended before giving up (jaws blocked)
        self.recheck_mm = recheck_mm
        self.recheck_deg = recheck_deg
        self.require_object = require_object        # 'thin' (closed completely below the load target) fails instead of counting as grasped
        self.close = close or gp.CloseParams()      # load-limited closing (grasp_plan.CloseParams)
        self.max_dt = max_dt                        # s: cap of the ramp step after a long gap between ticks


class GraspContext(object):
    """Interface the leaves use. The node implements it with ROS; tests with a fake."""

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

    def gripper_position(self):
        """Measured joint_6, or None when there is no measurement."""
        raise NotImplementedError

    def command_gripper(self, position):
        raise NotImplementedError

    def gripper_load(self):
        """(|motor load| of the gripper in % of max torque, the time it was received on the now() clock), or (None, None)."""
        raise NotImplementedError

    def start_trajectory(self, name, slow):
        """Send the planned trajectory `name` stretched by `slow`. Returns None or an error message."""
        raise NotImplementedError

    def poll_trajectory(self):
        """('running' | 'succeeded' | 'failed', message). The implementation owns the timeout and the endpoint check."""
        raise NotImplementedError

    def cancel_trajectory(self):
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


class _Leaf(py_trees.behaviour.Behaviour):
    def __init__(self, name, ctx):
        super(_Leaf, self).__init__(name=name)
        self.ctx = ctx

    def _state(self, state, message):
        self.ctx.set_state(state, message() if callable(message) else message)


class MoveGripper(_Leaf):
    """Ramps the joint_6 command to `target` at gripper_speed. SUCCESS once the measurement is at the target and has
    settled, or gripper_timeout after the ramp ended (jaws blocked by an object, or no measurement): the grasp
    outcome leaf decides what the final position means."""

    def __init__(self, name, ctx, params, target, other_end, state, message):
        super(MoveGripper, self).__init__(name, ctx)
        self.params, self.target, self.other_end, self.state, self.message = params, target, other_end, state, message

    def initialise(self):
        self._state(self.state, self.message)
        measured = self.ctx.gripper_position()
        self.cmd = measured if measured is not None else self.other_end
        self.t0 = self.last = self.ctx.now()
        self.ramp_time = abs(self.target - self.cmd) / self.params.gripper_speed
        self.reached_at = None

    def update(self):
        now = self.ctx.now()
        dt = min(max(now - self.last, 0.0), self.params.max_dt)
        self.last = now
        self.cmd = gp.gripper_ramp(self.cmd, self.target, self.params.gripper_speed, dt)
        self.ctx.command_gripper(self.cmd)
        if self.cmd != self.target:
            return Status.RUNNING
        measured = self.ctx.gripper_position()
        if measured is None or abs(measured - self.target) <= self.params.gripper_tolerance:
            if self.reached_at is None:
                self.reached_at = now
            if now - self.reached_at >= self.params.gripper_settle:
                return Status.SUCCESS
        if now - self.t0 > self.ramp_time + self.params.gripper_timeout:
            return Status.SUCCESS
        return Status.RUNNING


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


class CloseOnLoad(_Leaf):
    """Slow load-limited close (grasp_plan.LoadLimitedCloser), separate from the open ramp: closes at close_speed until
    the gripper's motor load reaches load_target, holds there, and succeeds when the load is stable in the band.
    Closing completely below the target is 'thin' (a thinner element than expected: counts as grasped unless
    require_object). A missing load measurement is a failure; the command then stays where it was (the driver holds)."""

    def __init__(self, ctx, spec, params):
        super(CloseOnLoad, self).__init__("close on load", ctx)
        self.spec, self.params = spec, params

    def initialise(self):
        self.ctx.set_state("CLOSING", "slowly to a load of %.0f %%" % self.params.close.load_target)
        measured = self.ctx.gripper_position()
        self.closer = gp.LoadLimitedCloser(measured if measured is not None else self.spec.gripper_open,
                                           self.spec.gripper_closed, self.spec.gripper_open, self.params.close)
        self.phase = self.closer.phase

    def update(self):
        load, stamp = self.ctx.gripper_load()
        step = self.closer.step(self.ctx.now(), self.ctx.gripper_position(), load, stamp)
        if step.fault:
            self.ctx.fail(step.fault)
            return Status.FAILURE
        self.ctx.command_gripper(step.cmd)
        if step.phase != self.phase:
            self.phase = step.phase
            self.ctx.set_state("CLOSING", {"HOLDING": "load %.0f %% reached: holding" % (load or 0.0),
                                           "CLOSED": "closed completely, load below the target"}.get(step.phase, ""))
        if not step.done:
            return Status.RUNNING
        self.ctx.record_outcome(step.outcome, load)
        if step.outcome == "thin" and self.params.require_object:
            self.ctx.fail("closed completely without reaching the load target and require_object is set")
            return Status.FAILURE
        return Status.SUCCESS


def build_tree(ctx, spec, params, outcome_text=lambda: ""):
    """The grasp tree. `outcome_text` returns the text for the LIFTING message (the recorded outcome)."""
    mission = py_trees.composites.Sequence(name="grasp", children=[
        MoveGripper("open gripper", ctx, params, spec.gripper_open, spec.gripper_closed, "OPENING", "gripper to %.3f" % spec.gripper_open),
        FollowTrajectory("PRE", ctx, "PRE", 1.0, "MOVING_PRE", "above the target"),
        RecheckTarget(ctx, params),
        FollowTrajectory("DESCEND", ctx, "DESCEND", spec.descent_slow_factor, "DESCENDING",
                         "straight down, %.1f times slower than planned" % spec.descent_slow_factor),
        CloseOnLoad(ctx, spec, params),
        FollowTrajectory("LIFT", ctx, "LIFT", spec.lift_slow_factor, "LIFTING", lambda: "grasp outcome: %s" % outcome_text()),
    ])
    return Guarded(mission, ctx)


def active_path(root):
    """Names of the RUNNING nodes from the root down, e.g. 'stop / e-stop guard/grasp/DESCEND'."""
    names, node = [], root
    while node is not None and node.status == Status.RUNNING:
        names.append(node.name)
        node = next((c for c in node.children if c.status == Status.RUNNING), None)
    return "/".join(names)
