"""Pure logic of the TF-driven grasp sequence (numpy, no ROS).

The target is a frame T_base_target (e.g. a marker lying on the floor: +z up, out of the surface). The sequence
approaches along -z_target: waypoints PRE (pre_height above the target), GRASP (grasp_height) and LIFT (lift_height),
all straight above the target point, with the tool axis pointing down onto it.

Tool orientation for this 5-DOF arm: the wrist can only pitch about its own Y axis, so the tool's y axis must be
horizontal and perpendicular to the arm's vertical plane through the base axis and the waypoint; the tool z axis is
the approach direction projected into that plane (exactly -z_base for a floor target); x = y cross z.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class GraspSpec:
    pre_height: float = 0.10          # m above the target point: the end-effector origin (grasp point) at PRE
    grasp_height: float = 0.02        # m above the target point at GRASP   (UNCONFIRMED: depends on the gripper's geometry)
    lift_height: float = 0.15         # m above the target point at LIFT
    tool_offset: float = 0.0          # m from the end-effector origin to the grasp point along the tool axis (UNKNOWN)
    grasp_offset: tuple = (0.0, 0.0)  # x, y of the grasp point in the target frame
    gripper_open: float = 0.025       # joint_6 [m]: open  (URDF placeholder, see config/grasp.yaml)
    gripper_closed: float = 0.0       # joint_6 [m]: closed
    descent_slow_factor: float = 3.0  # the descent and the lift take this many times longer than the planner's time (>= 1)
    lift_slow_factor: float = 2.0
    side_grasp_height: float = 0.0    # m along the target frame's z (back toward the base) at GRASP for a SIDE approach: 0 = the fingertips reach the target point itself (a stem)
    side_lift: str = "up"             # LIFT of a side approach: 'up' (straight up from GRASP, pulls a plant out) or 'back' (straight back along the approach)
    max_cartesian_deviation: float = 0.005   # m: allowed worst tracking residual of a straight leg


@dataclass
class Waypoint:
    name: str
    position: np.ndarray             # end-effector origin in base_link
    R: np.ndarray                    # end-effector orientation (columns x, y, z) in base_link
    grasp_point: np.ndarray = None   # the point the waypoint is about (target point + height)


def tool_rotation(approach, position):
    """Rotation of the end effector whose z axis is `approach` (a unit vector in the base frame) as far as the pitch-only
    wrist allows. Returns (R, deviation_deg): the angle between the requested approach and the achievable tool axis."""
    a = np.asarray(approach, float)
    a = a / np.linalg.norm(a)
    az = np.arctan2(position[1], position[0])
    y = np.array([-np.sin(az), np.cos(az), 0.0])      # horizontal, perpendicular to the arm's plane
    z = a - (a @ y) * y                                # the approach in the arm's plane
    n = np.linalg.norm(z)
    if n < 1e-9:
        raise ValueError("the approach direction is perpendicular to the arm's plane")
    z = z / n
    x = np.cross(y, z)
    deviation = float(np.degrees(np.arccos(np.clip(a @ z, -1.0, 1.0))))
    return np.column_stack([x, y, z]), deviation


def is_side_approach(T_base_target):
    """True when the target frame's z axis (the direction the gripper retreats along) is more than 60 degrees from vertical: a SIDE grasp, the tool looking
    horizontally. arm_grasp/target_offset.side_frame builds such a frame (grasp_target ~approach:=forward)."""
    return abs(np.asarray(T_base_target, float)[2, 2]) < 0.5


def waypoints(T_base_target, spec):
    """PRE, GRASP, LIFT as a list of Waypoint (end-effector pose in base_link) for a target frame.
    The tool approaches along -z of the target frame: straight down onto a floor target, or, when the frame is horizontal (is_side_approach), sideways
    from the robot's side; there GRASP is spec.side_grasp_height short of the target and LIFT goes straight up (side_lift 'up') or back.
    Raises ValueError if the target's normal cannot be matched by the wrist within 5 degrees."""
    T = np.asarray(T_base_target, float)
    normal = T[:3, 2]
    approach = -normal
    side = is_side_approach(T)
    out = []
    for name, h in (("PRE", spec.pre_height), ("GRASP", spec.side_grasp_height if side else spec.grasp_height), ("LIFT", spec.lift_height)):
        if name == "LIFT" and side and spec.side_lift == "up":
            g = out[1]
            out.append(Waypoint(name, g.position + h * np.array([0.0, 0.0, 1.0]), g.R, g.grasp_point + h * np.array([0.0, 0.0, 1.0])))
            continue
        local = np.array([spec.grasp_offset[0], spec.grasp_offset[1], h, 1.0])
        grasp_point = (T @ local)[:3]
        R, dev = tool_rotation(approach, grasp_point)
        if dev > 5.0:
            raise ValueError("the target's normal is %.1f degrees away from what the pitch-only wrist can approach along" % dev)
        position = grasp_point - spec.tool_offset * R[:, 2]
        out.append(Waypoint(name, position, R, grasp_point))
    return out


def quaternion_from_matrix(R):
    """(x, y, z, w) of a rotation matrix."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def target_moved(T_a, T_b):
    """(translation [mm], rotation [deg]) between two target poses: has the target moved since it was planned?"""
    T_a, T_b = np.asarray(T_a), np.asarray(T_b)
    mm = 1000.0 * float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3]))
    c = (np.trace(T_a[:3, :3].T @ T_b[:3, :3]) - 1.0) / 2.0
    return mm, float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


# ---- checking a dry-run plan ------------------------------------------------------------------------------------

@dataclass
class Leg:
    name: str
    success: bool
    message: str = ""
    duration: float = 0.0              # s, as planned (before any slow-down)
    deviation: float = 0.0             # m, worst Cartesian tracking residual (0 for joint-space legs)
    cartesian: bool = False


def check_plan(legs, spec):
    """List of problems of a planned sequence (empty = feasible)."""
    problems = []
    for leg in legs:
        if not leg.success:
            problems.append("%s: not reachable (%s)" % (leg.name, leg.message or "no reason given"))
        elif leg.cartesian and leg.deviation > spec.max_cartesian_deviation:
            problems.append("%s: the straight path is only followed to %.1f mm (limit %.1f mm)" % (
                leg.name, 1000 * leg.deviation, 1000 * spec.max_cartesian_deviation))
    return problems


def total_duration(legs, spec, gripper_time=0.0):
    """Predicted time of all the motions including the slow-down of the descent and the lift."""
    factors = {"DESCEND": spec.descent_slow_factor, "LIFT": spec.lift_slow_factor}
    return sum(leg.duration * max(1.0, factors.get(leg.name, 1.0)) for leg in legs) + gripper_time


def stretch_times(points, factor):
    """Slow a trajectory down by `factor` (>= 1): times x factor, velocities / factor, accelerations / factor^2.
    `points` are (time_from_start, positions, velocities, accelerations) tuples; returns new tuples."""
    f = max(1.0, float(factor))
    out = []
    for t, p, v, a in points:
        out.append((t * f, list(p), [x / f for x in v] if v else v, [x / (f * f) for x in a] if a else a))
    return out


# ---- gripper -----------------------------------------------------------------------------------------------------

def gripper_ramp(current, target, max_speed, dt):
    """Next gripper command: moves from `current` toward `target` at most max_speed [m/s]."""
    step = max_speed * dt
    if abs(target - current) <= step:
        return target
    return current + step * np.sign(target - current)


@dataclass
class CloseParams:
    """Load-limited closing. EVERY value is a FLAGGED, UNVALIDATED default (see config/grasp.yaml) except the operator's two
    numbers (2026-10-09): peak load 45 % (load_limit); steady state 37.5 %, raised to 38.5 % on request (load_target). The gripper motor tripped its
    overload protection (error 0x20) holding 43 to 48 %, so the STEADY state must stay under 40 %: target 38.5 +- 1 % (raised from 37.5)."""
    close_speed: float = 0.004       # m/s: APPROACH speed of the close (until the load reaches contact_load; operator: 4 mm/s); set it from outside via close/close_speed
    contact_speed: float = 0.002     # m/s: speed once the load has risen to contact_load (the final trim approach; operator: 2 mm/s)
    relief_speed: float = 0.001      # m/s: how fast the command opens when the load is at the peak limit (kept apart from contact_speed so a faster approach does not make the relief coarse)
    contact_load: float = 15.0       # % of max torque: the load at which the close slows from close_speed to contact_speed (operator: raised from 4 %; free closing measured 0.8 to 1.8 %)
    load_target: float = 38.5        # % of max torque: the STEADY-STATE load to hold the grasp at (raised from the operator's 37.5 on request, still under 40 with the band)
    load_target_max: float = 38.5    # % ceiling of the runtime-adjustable target (service ~set_load_target, rviz panel slider): the operator's steady-state number; the target can only be set within [0, this]
    contact_floor: float = 2.0       # % lowest contact_load a runtime change of the target may give (free closing measured 0.8 to 1.8 %)
    load_limit: float = 45.0         # % PEAK CEILING: at or above it the command opens at once (at relief_speed); if it persists for hold_timeout the grasp fails
    load_lead_s: float = 0.25        # s: look-ahead of the stop decision, predicted load = load + (load rate) x load_lead_s (covers the 10 Hz sampling and the lag)
    load_band: float = 1.0           # % band around load_target in which the grasp counts as settled (the hold stays at or under load_target + band = 39.5 %)
    hold_kp: float = 3e-6            # m per % of load error: proportional gain of the hold PI (command increment = -(kp d(e) + ki e Ts), e = target - load)
    hold_ki: float = 1.4e-5          # m per (% s): integral gain of the hold PI. From the measured 65 to 80 %/mm: loop time constant ~1 s, well above the load delay
    trim_speed: float = 0.002        # m/s: the PI's command rate limit, the final trim (operator: 2 mm/s)
    confirm_samples: int = 1         # consecutive NEW load samples with predicted load >= load_target before the close stops
    load_max_age: float = 0.5        # s: an older load sample is no measurement (the driver publishes at 10 Hz)
    hold_time: float = 1.0           # s in the band and still before the grasp counts as done
    hold_timeout: float = 10.0       # s of holding without settling in the band
    still_time: float = 0.5          # s the jaw position must stay within still_tol
    still_tol: float = 0.0003        # m
    closed_settle: float = 0.6       # s after the command reached fully closed before calling it 'thin'
    lost_time: float = 0.3           # s the load must stay below contact_load, after the grasp settled, to call the grip LOST
    over_time: float = 2.0           # s the load must stay at or above load_limit, after the grasp settled, to fail


@dataclass
class CloseStep:
    cmd: float                       # joint_6 command to send
    phase: str                       # CLOSING | HOLDING | CLOSED
    done: bool = False
    outcome: str = ""                # 'object' (load target reached and held) | 'thin' (closed completely below the target load)
    fault: str = ""                  # non-empty: stop, the sequence must not continue


class LoadLimitedCloser:
    """Closes the gripper at constant speed until the motor load is about to reach `load_target`, then holds there,
    never on purpose above `load_limit`.

    Command law (cmd decreases toward `closed`; load is |motor load| in %; predicted = load + rate x load_lead_s):
      CLOSING   cmd -= close_speed dt (approach), contact_speed dt once the load has reached contact_load,
                until `confirm_samples` consecutive NEW samples have predicted >= load_target
      HOLDING   load >= load_limit:       cmd += relief_speed dt (relief)
                otherwise a PI on e = target - load keeps the load AS CLOSE AS POSSIBLE to the target, above and below it:
                  once per new load sample  d = -(kp (e - e_prev) + ki e Ts)      (incremental form: no windup; d < 0 closes)
                  d < 0 is dropped while the PREDICTED load is already >= target (look-ahead against overshoot)
                  the command moves by at most trim_speed dt per step and stays inside [closed, opened]; what a limit blocks is discarded
                done once the load stayed within `band` of the target for hold_time and the jaws are still
                at the closed limit with the load still below target - band the outcome is 'thin' (nothing more can be closed)
      CLOSED    the command reached `closed` without the load target: after closed_settle with the jaws still the
                outcome is 'thin' (closed completely at a lower load). By the user's decision this counts as a successful
                grasp of a thinner element, although the load cannot tell it from an empty closure.
    CONTINUOUS: the caller keeps calling step() after the first `done`; the PI keeps regulating for as long as it does (the
    gripper_control node does, until the gripper is opened or stopped), and after the first settle step() also supervises:
    the load below contact_load for lost_time is a fault "the contact was lost" (the grip is gone), at or above load_limit
    for over_time a fault "stays at"; a 'thin' settle is exempt from the loss check.
    A load at or above load_limit in any phase switches to HOLDING (relief). No fresh load sample (older than load_max_age)
    is a fault: the gripper is never driven without a load reading. The heuristic parts (speeds, band, look-ahead) are not
    identified on the hardware.
    """

    def __init__(self, start, closed, opened, params):
        self.p, self.closed, self.opened = params, closed, opened
        self.cmd = start
        self.phase = "CLOSING"
        self.t_last = None
        self.hits = 0
        self.samples = []                    # the last two (stamp, load) with distinct stamps
        self.t_hold = None
        self.in_band_since = None
        self.t_closed = None
        self.ref = None                      # (position, time) the jaws have been within still_tol of
        self.slow = False                    # the load has reached contact_load: final approach at contact_speed
        self.predicted = 0.0
        self.e_prev = None                   # previous load error of the hold PI
        self.pending = 0.0                   # command increment not yet applied (rate-limited)
        self.sample_dt = 0.1
        self.new_sample = False
        self.settled = False                 # the grasp has settled once: from then on the PI runs for as long as the caller keeps stepping
        self.settled_outcome = ""
        self.thin = False                    # settled as 'thin': nothing to lose, so no loss detection
        self.t_low = None                    # since when the load has been below contact_load (loss detection)
        self.t_over = None                   # since when the load has been at or above load_limit

    def rebase_target(self, old_target):
        """The caller changed p.load_target (and p.contact_load, shared by reference) while closing or holding. Re-bases the hold PI so the step in the
        error does not kick the command (the integral term moves it, rate-limited), and restarts the timers that were measured against the old levels."""
        if self.e_prev is not None:
            self.e_prev += self.p.load_target - old_target                # e = load_target - load
        self.pending = 0.0
        self.in_band_since = self.t_closed = self.t_low = None
        self.hits = 0

    def _still(self, t, measured):
        if self.ref is None or abs(measured - self.ref[0]) > self.p.still_tol:
            self.ref = (measured, t)
        return t - self.ref[1] >= self.p.still_time

    def step(self, t, measured, load, load_stamp):
        """One control step. `measured` jaw position or None, `load` |motor load| in % or None, `load_stamp` its reception time."""
        p = self.p
        if load is None or load_stamp is None or t - load_stamp > p.load_max_age:
            age = "none" if load_stamp is None else "%.2f s old" % (t - load_stamp)
            return CloseStep(self.cmd, self.phase, fault="no fresh gripper load (%s)" % age)
        dt = 0.0 if self.t_last is None else min(max(t - self.t_last, 0.0), 0.2)
        self.t_last = t
        pos = self.cmd if measured is None else measured
        still = self._still(t, pos)
        self.new_sample = False
        if not self.samples or load_stamp != self.samples[-1][0]:                 # a NEW load sample
            self.new_sample = True
            if self.samples:
                self.sample_dt = min(max(load_stamp - self.samples[-1][0], 0.02), 0.3)
            self.samples = (self.samples + [(load_stamp, load)])[-2:]
            rate = 0.0
            if len(self.samples) == 2 and self.samples[1][0] > self.samples[0][0]:
                rate = max(0.0, (self.samples[1][1] - self.samples[0][1]) / (self.samples[1][0] - self.samples[0][0]))
            self.predicted = load + rate * p.load_lead_s
            self.hits = self.hits + 1 if self.predicted >= p.load_target else 0
        if self.phase in ("CLOSING", "CLOSED") and (self.hits >= p.confirm_samples or load >= p.load_limit):
            self.phase, self.t_hold, self.in_band_since = "HOLDING", t, None     # freeze the command where it is
            self.e_prev, self.pending = p.load_target - load, 0.0
            if load < p.load_limit:
                return CloseStep(self.cmd, self.phase)
        if self.phase == "CLOSING":
            if load >= p.contact_load:
                self.slow = True
            self.cmd = max(self.closed, self.cmd - (p.contact_speed if self.slow else p.close_speed) * dt)
            if self.cmd <= self.closed:
                self.phase, self.t_closed = "CLOSED", t
            return CloseStep(self.cmd, self.phase)
        if self.phase == "CLOSED":
            if t - self.t_closed >= p.closed_settle and still:
                self.settled, self.settled_outcome, self.thin = True, "thin", True
                return CloseStep(self.cmd, self.phase, done=True, outcome="thin")
            return CloseStep(self.cmd, self.phase)
        # HOLDING
        e = p.load_target - load
        if self.settled:                                                              # continuous supervision after the first settle
            self.t_over = (t if self.t_over is None else self.t_over) if load >= p.load_limit else None
            if self.t_over is not None and t - self.t_over >= p.over_time:
                return CloseStep(self.cmd, self.phase, fault="the gripper load stays at %.0f %% (limit %.0f %%)" % (load, p.load_limit))
            if not self.thin:
                self.t_low = (t if self.t_low is None else self.t_low) if load < p.contact_load else None
                if self.t_low is not None and t - self.t_low >= p.lost_time:
                    return CloseStep(self.cmd, self.phase, fault="the contact was lost while holding (load %.1f %%)" % load)
        if load >= p.load_limit:
            self.cmd = min(self.opened, self.cmd + p.relief_speed * dt)               # relief: at 65-80 %/mm this already sheds ~65 %/s, and does not lose the contact
            self.pending, self.in_band_since = 0.0, None
            self.e_prev = e
        else:
            if self.new_sample:
                d = -(p.hold_kp * (e - self.e_prev) + p.hold_ki * e * self.sample_dt) if self.e_prev is not None else 0.0
                if d < 0.0 and self.predicted >= p.load_target:
                    d = 0.0                                                           # the load is about to reach the target: no more closing
                self.pending += d
                self.e_prev = e
            step = max(-p.trim_speed * dt, min(p.trim_speed * dt, self.pending))
            new = min(self.opened, max(self.closed, self.cmd + step))
            if new != self.cmd + step:
                self.pending = 0.0                                                    # a command limit blocks it: discard (anti-windup)
            else:
                self.pending -= step
            self.cmd = new
            at_closed_limit = self.cmd <= self.closed + 1e-12
            if abs(e) <= p.load_band:
                self.t_closed = None
                if self.in_band_since is None:
                    self.in_band_since = t
                if t - self.in_band_since >= p.hold_time and still:
                    self.settled, self.settled_outcome = True, "object"
                    return CloseStep(self.cmd, self.phase, done=True, outcome="object")
            else:
                self.in_band_since = None
                if e > p.load_band and at_closed_limit:                               # cannot close any further, load still short of the target
                    if self.t_closed is None:
                        self.t_closed = t
                    elif t - self.t_closed >= p.closed_settle and still:
                        self.settled, self.settled_outcome, self.thin = True, "thin", True
                        return CloseStep(self.cmd, self.phase, done=True, outcome="thin")
                else:
                    self.t_closed = None
        if not self.settled and t - self.t_hold > p.hold_timeout:
            if load >= p.load_limit:
                return CloseStep(self.cmd, self.phase, fault="the gripper load stays at %.0f %% (limit %.0f %%)" % (load, p.load_limit))
            if load < p.contact_load:
                return CloseStep(self.cmd, self.phase, fault="the contact was lost while holding (load %.1f %%)" % load)
            self.settled, self.settled_outcome = True, "object"                  # in contact, load not in the band: report it and keep regulating
            return CloseStep(self.cmd, self.phase, done=True, outcome="object")
        return CloseStep(self.cmd, self.phase, done=self.settled, outcome=self.settled_outcome)
