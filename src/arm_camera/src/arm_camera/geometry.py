"""Rigid-transform helpers (numpy only, no ROS). All transforms are 4x4 homogeneous
matrices; T_a_b maps coordinates expressed in frame b into frame a (the pose of b in a)."""
import numpy as np


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def make_transform(R, t):
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def invert_transform(T):
    R, t = T[:3, :3], T[:3, 3]
    return make_transform(R.T, -R.T @ t)


def transform_points(T, pts):
    """Apply T to an (N, 3) array of points."""
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    return pts @ T[:3, :3].T + T[:3, 3]


def rotation_angle_deg(R):
    """Angle of rotation R in degrees, in [0, 180]."""
    # atan2 of (sin, cos): arccos of the trace alone loses all precision near 0 and 180 deg.
    s = 0.5 * np.linalg.norm([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    c = 0.5 * (np.trace(R) - 1.0)
    return float(np.degrees(np.arctan2(s, c)))


def rotation_error_deg(R1, R2):
    """Angle of the rotation taking R1 to R2, in degrees."""
    return rotation_angle_deg(R1.T @ R2)


def rotation_axis(R):
    """Unit axis of rotation R, or None for (numerically) no rotation."""
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(v)
    if n < 1e-9:
        return None
    return v / n


def quat_from_matrix(R):
    """Unit quaternion (x, y, z, w) with w >= 0 from a rotation matrix."""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def matrix_from_quat(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def average_rotation(rotations):
    """Chordal L2 mean of rotation matrices (projection of the sum onto SO(3))."""
    M = np.sum(np.asarray(rotations), axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def ypr_from_matrix(R):
    """(yaw, pitch, roll) in radians such that R = Rz(yaw) @ Ry(pitch) @ Rx(roll): the argument order of
    tf2_ros static_transform_publisher (and camera_static_tf.launch)."""
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    return yaw, pitch, roll
