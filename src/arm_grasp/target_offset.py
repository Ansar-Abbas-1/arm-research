"""Grasp target from a marker plus a hand-measured offset (numpy, no ROS).

The marker frame has +z out of the surface it lies on. The grasp target keeps the marker's ORIENTATION (so +z stays the surface
normal the grasp sequence approaches along) and moves its origin by `offset` expressed in the MARKER frame:
    p_target = p_marker + R_marker * offset
"""
import numpy as np


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def offset_target(position, quaternion, offset):
    """(position, quaternion) of the marker in the base frame and `offset` (x, y, z) in the marker frame
    -> position of the grasp target in the base frame (the orientation is the marker's)."""
    R = quat_to_matrix(quaternion)
    return np.asarray(position, float) + R.dot(np.asarray(offset, float))


def vertical_frame(quaternion):
    """Rotation matrix of an UPRIGHT frame with the marker's heading: z = base z, x = the marker's x axis projected onto the horizontal plane.
    For a marker lying on a level floor this removes the pose-estimation tilt (a few degrees of ambiguity) and keeps its yaw. Falls back to the
    identity heading when the marker's x axis is (nearly) vertical."""
    R = quat_to_matrix(quaternion)
    x = np.array([R[0, 0], R[1, 0], 0.0])
    n = np.linalg.norm(x)
    if n < 1e-6:
        return np.eye(3)
    x = x / n
    z = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def offset_target_frame(position, quaternion, offset, vertical=False):
    """-> (position, 3x3 rotation) of the grasp target: the marker's own frame, or with vertical=True its upright version, shifted by `offset`
    expressed in THAT frame."""
    R = vertical_frame(quaternion) if vertical else quat_to_matrix(quaternion)
    return np.asarray(position, float) + R.dot(np.asarray(offset, float)), R
