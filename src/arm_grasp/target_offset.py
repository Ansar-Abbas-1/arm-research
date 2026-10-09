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
