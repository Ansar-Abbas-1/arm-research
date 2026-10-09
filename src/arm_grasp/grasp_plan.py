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


def waypoints(T_base_target, spec):
    """PRE, GRASP, LIFT as a list of Waypoint (end-effector pose in base_link) for a target frame.
    Raises ValueError if the target's normal cannot be matched by the wrist within 5 degrees."""
    T = np.asarray(T_base_target, float)
    normal = T[:3, 2]
    approach = -normal
    out = []
    for name, h in (("PRE", spec.pre_height), ("GRASP", spec.grasp_height), ("LIFT", spec.lift_height)):
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
    numbers (2026-10-09): peak load 45 % (load_limit) and steady-state load 37.5 % (load_target). The gripper motor tripped its
    overload protection (error 0x20) holding 43 to 48 %, so the STEADY state must stay under 40 %: the hold band is +-2 %."""
    close_speed: float = 0.002       # m/s: APPROACH speed of the close (until the load first rises); set it from outside via close/close_speed
    contact_speed: float = 0.0005    # m/s: speed once the load has risen to contact_load (the final approach)
    contact_load: float = 4.0        # % of max torque: the first sign of contact (free closing measured 0.8 to 1.7 %)
    load_target: float = 37.5        # % of max torque: the STEADY-STATE load to hold the grasp at (the close stops when the PREDICTED load reaches it)
    load_limit: float = 45.0         # % PEAK CEILING: at or above it the command opens at once at close_speed; if it persists for hold_timeout the grasp fails
    load_lead_s: float = 0.25        # s: look-ahead of the stop decision, predicted load = load + (load rate) x load_lead_s (covers the 10 Hz sampling and the lag)
    load_band: float = 2.0           # % deadband of the hold regulation around load_target (the hold stays at or under load_target + band = 39.5 %)
    trim_speed: float = 0.0003       # m/s: speed of the hold regulation (load too high: open, too low: close)
    confirm_samples: int = 1         # consecutive NEW load samples with predicted load >= load_target before the close stops
    load_max_age: float = 0.5        # s: an older load sample is no measurement (the driver publishes at 10 Hz)
    hold_time: float = 1.0           # s in the band and still before the grasp counts as done
    hold_timeout: float = 6.0        # s of holding without getting in the band
    still_time: float = 0.5          # s the jaw position must stay within still_tol
    still_tol: float = 0.0003        # m
    closed_settle: float = 0.6       # s after the command reached fully closed before calling it 'thin'


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
      HOLDING   load >= load_limit:       cmd += close_speed dt (relief, fast)
                load > target + band:     cmd += trim_speed dt  (open)
                load < target - band:     cmd -= trim_speed dt  (never past `closed`; only while the PREDICTED load is below target - band)
                done once the load stayed within the band for hold_time and the jaws are still
      CLOSED    the command reached `closed` without the load target: after closed_settle with the jaws still the
                outcome is 'thin' (closed completely at a lower load). By the user's decision this counts as a successful
                grasp of a thinner element, although the load cannot tell it from an empty closure.
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
        if not self.samples or load_stamp != self.samples[-1][0]:                 # a NEW load sample
            self.samples = (self.samples + [(load_stamp, load)])[-2:]
            rate = 0.0
            if len(self.samples) == 2 and self.samples[1][0] > self.samples[0][0]:
                rate = max(0.0, (self.samples[1][1] - self.samples[0][1]) / (self.samples[1][0] - self.samples[0][0]))
            self.predicted = load + rate * p.load_lead_s
            self.hits = self.hits + 1 if self.predicted >= p.load_target else 0
        if self.phase in ("CLOSING", "CLOSED") and (self.hits >= p.confirm_samples or load >= p.load_limit):
            self.phase, self.t_hold, self.in_band_since = "HOLDING", t, None     # freeze the command where it is
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
                return CloseStep(self.cmd, self.phase, done=True, outcome="thin")
            return CloseStep(self.cmd, self.phase)
        # HOLDING
        err = load - p.load_target
        if load >= p.load_limit:
            self.cmd = min(self.opened, self.cmd + p.close_speed * dt)                # relief, fast
            self.in_band_since = None
        elif err > p.load_band:
            self.cmd = min(self.opened, self.cmd + p.trim_speed * dt)
            self.in_band_since = None
        elif err < -p.load_band:
            if self.cmd > self.closed:
                if self.predicted < p.load_target - p.load_band:             # only keep closing while the PREDICTED load is still short of the band
                    self.cmd = max(self.closed, self.cmd - p.trim_speed * dt)
                self.t_closed = None
            elif self.t_closed is None:
                self.t_closed = t
            elif t - self.t_closed >= p.closed_settle and still:               # regulated all the way to closed, load still low
                return CloseStep(self.cmd, self.phase, done=True, outcome="thin")
            self.in_band_since = None
        else:
            if self.in_band_since is None:
                self.in_band_since = t
            if t - self.in_band_since >= p.hold_time and still:
                return CloseStep(self.cmd, self.phase, done=True, outcome="object")
        if t - self.t_hold > p.hold_timeout:
            if load >= p.load_limit:
                return CloseStep(self.cmd, self.phase, fault="the gripper load stays at %.0f %% (limit %.0f %%)" % (load, p.load_limit))
            return CloseStep(self.cmd, self.phase, done=True, outcome="object")   # in contact, load not in the band: report it
        return CloseStep(self.cmd, self.phase)
