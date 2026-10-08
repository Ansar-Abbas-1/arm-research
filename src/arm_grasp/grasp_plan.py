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


def grasp_outcome(measured_closed, spec, object_threshold=0.002):
    """What the closed gripper position says: 'object' if it stopped short of fully closed by more than the
    threshold (something is between the jaws), 'empty' if it closed all the way."""
    return "object" if measured_closed > spec.gripper_closed + object_threshold else "empty"
