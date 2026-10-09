"""The gripper's state machine (no ROS): opens, closes on load, and then KEEPS REGULATING the load until told otherwise.

The gripper_control node calls `step()` at 20 Hz with the latest measurements and publishes whatever command it returns.

    IDLE      nothing commanded yet (the driver holds whatever it was last given)
    OPENING   ramp the command to `open_pos`; done when the measurement is there and has settled
    CLOSING   the slow load-limited close (grasp_plan.LoadLimitedCloser) until the grasp settles
    HOLDING   the same closer keeps running: its PI holds the load at load_target, indefinitely, until open / stop / a loss
    LOST      the load collapsed after the grasp had settled: the object is gone. The command is frozen (no creeping closed)
    FROZEN    stopped (operator, e-stop, stale load, fault, timeout): no commands are sent; the driver holds the last goal

`grip` summarises it for others: 'held' (HOLDING with a settled object grasp), 'thin' (closed completely below the load
target, counted as grasped), 'lost', or 'none'. An action's completion is reported once, as a `Result`.
"""
from arm_grasp import grasp_plan as gp


class Result(object):
    """How a requested action ended."""

    def __init__(self, kind, ok, outcome="", message="", load=0.0):
        self.kind, self.ok, self.outcome, self.message, self.load = kind, ok, outcome, message, load


class StepOut(object):
    def __init__(self, cmd=None, result=None):
        self.cmd = cmd                  # joint_6 command to publish, or None (send nothing)
        self.result = result            # a Result when an action just ended, else None


class GripperController(object):
    def __init__(self, params, open_pos=0.025, closed_pos=0.0, open_speed=0.02, open_settle=0.6, open_tol=0.002, timeout=40.0):
        self.p = params
        self.open_pos, self.closed_pos = open_pos, closed_pos
        self.open_speed, self.open_settle, self.open_tol, self.timeout = open_speed, open_settle, open_tol, timeout
        self.mode = "IDLE"
        self.message = ""
        self.grip = "none"
        self.closer = None
        self.cmd = None
        self.t_action = None
        self.t_reached = None
        self.t_last = None
        self.pending = None             # 'open' | 'close' while an action waits for its completion

    # ---- requests ----------------------------------------------------------------------------------------------
    def request_open(self, now, pos):
        self.mode, self.message, self.grip, self.closer = "OPENING", "to %.4f" % self.open_pos, "none", None
        self.cmd, self.t_action, self.t_reached, self.t_last, self.pending = pos, now, None, now, "open"

    def request_close(self, now, pos):
        self.mode, self.message, self.grip = "CLOSING", "slowly to a load of %.1f %%" % self.p.load_target, "none"
        self.closer = gp.LoadLimitedCloser(pos, self.closed_pos, self.open_pos, self.p)
        self.cmd, self.t_action, self.t_last, self.pending = pos, now, now, "close"

    def request_stop(self, reason="stopped"):
        """Stop commanding. Returns the Result of the action it interrupts, if any."""
        return self._freeze(reason)

    # ---- the control step -----------------------------------------------------------------------------------------
    def step(self, now, pos, load, load_stamp, estop_ok):
        if self.mode in ("IDLE", "FROZEN", "LOST"):
            return StepOut()
        if not estop_ok:
            return StepOut(result=self._freeze("e-stop engaged or its state unknown"))
        if pos is None:
            return StepOut(result=self._freeze("no joint_6 measurement"))
        if self.mode == "OPENING":
            return self._opening(now, pos)
        return self._closing_or_holding(now, pos, load, load_stamp)

    def _opening(self, now, pos):
        dt = min(max(now - self.t_last, 0.0), 0.2)
        self.t_last = now
        self.cmd = gp.gripper_ramp(self.cmd, self.open_pos, self.open_speed, dt)
        if self.cmd == self.open_pos and abs(pos - self.open_pos) <= self.open_tol:
            self.t_reached = now if self.t_reached is None else self.t_reached
            if now - self.t_reached >= self.open_settle:
                self.mode, self.message = "IDLE", "open (joint_6 %.4f)" % pos
                return StepOut(self.cmd, self._complete(Result("open", True, "open", self.message)))
        if now - self.t_action > self.timeout:
            return StepOut(result=self._freeze("timed out opening (joint_6 %.4f, target %.4f)" % (pos, self.open_pos)))
        return StepOut(self.cmd)

    def _closing_or_holding(self, now, pos, load, load_stamp):
        step = self.closer.step(now, pos, load, load_stamp)
        if step.fault:
            lost = "contact was lost" in step.fault
            result = self._freeze(step.fault, grip="lost" if lost else "none", mode="LOST" if lost else "FROZEN")
            return StepOut(result=result)
        self.cmd = step.cmd
        out = StepOut(self.cmd)
        if self.mode == "CLOSING":
            if step.done:                                             # first settle: the action is complete, the regulation goes on
                self.mode = "HOLDING"
                self.grip = "thin" if step.outcome == "thin" else "held"
                self.message = "load %.1f %%, outcome %s" % (load, step.outcome)
                out.result = self._complete(Result("close", True, step.outcome, self.message, load))
            elif now - self.t_action > self.timeout:
                out.cmd, out.result = None, self._freeze("timed out closing")
        return out

    # ---- helpers ------------------------------------------------------------------------------------------------
    def _complete(self, result):
        self.pending = None
        return result

    def _freeze(self, reason, grip=None, mode="FROZEN"):
        was = self.pending
        self.mode, self.message = mode, reason
        if grip is not None:
            self.grip = grip
        elif self.mode == "FROZEN" and self.grip == "held":
            self.grip = "none"                                          # regulation is off: nothing is being held actively
        self.pending = None
        if was:
            return Result(was, False, "", reason)
        return None

    @property
    def phase(self):
        return "GRIP_LOST" if self.mode == "LOST" else self.mode
