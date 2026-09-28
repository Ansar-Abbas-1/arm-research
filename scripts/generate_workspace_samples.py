#!/usr/bin/env python3
"""Generates the CSV point clouds visualize_workspace.m reads.

Independent reimplementation of the base_link -> end_effector forward
kinematics chain (extends docs/fk_reference.py, which stops at link_5, by
joint_6's fixed offset) -- same math family as
ForwardKinematicsSolver/workspace_sampler.h, but re-derived here rather
than called into, so this runs standalone with only numpy (no ROS/catkin,
no C++ build) for a MATLAB-only hand-off. See
docs/workspace_identification.md for the method (Monte Carlo over joint
space, not a grid -- combinatorially infeasible at 5 DOF) and its
known convergence caveat.

Usage:
    python3 generate_workspace_samples.py
"""
import csv

import numpy as np


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _rpy(r, p, y):
    return _Rz(y) @ _Ry(p) @ _Rx(r)


def _T(xyz, rpy_tuple):
    M = np.eye(4)
    M[:3, :3] = _rpy(*rpy_tuple)
    M[:3, 3] = xyz
    return M


def _axis_angle(axis, theta):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = np.cos(theta), np.sin(theta), 1 - np.cos(theta)
    R = np.array([
        [x * x * C + c, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, y * y * C + c, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, z * z * C + c],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    return M


# base_link -> end_effector, from arm_interface/urdf/arm.urdf. Origins/rpy
# match docs/fk_reference.py exactly for joints 1-5 (already cross-checked
# there against the C++ solver); joint_6's fixed offset (link_5 ->
# end_effector) is appended here since this script targets end_effector,
# not link_5.
_JOINTS = [
    dict(xyz=(0, 0, 0.03), rpy=(0, 0, 0), axis=(0, 0, 1), actuated=True),        # joint_1
    dict(xyz=(0, 0, 0.065), rpy=(0, -1.57, 0), axis=(0, 1, 0), actuated=True),   # joint_2
    dict(xyz=(0, 0, 0.34), rpy=(0, 0, 0), axis=(0, 1, 0), actuated=True),        # joint_3
    dict(xyz=(0.029, 0, 0), rpy=(0, 3.1412, 0), axis=None, actuated=False),      # joint_intermediate
    dict(xyz=(0, 0, 0.376), rpy=(0, -1.8791, 0), axis=(0, 1, 0), actuated=True), # joint_4
    dict(xyz=(0, 0, 0.336), rpy=(0, 1.81, 0), axis=(0, 1, 0), actuated=True),    # joint_5
    dict(xyz=(0, 0, 0.105), rpy=(0, 0, 0), axis=None, actuated=False),          # joint_6 (fixed)
]

# [lower, upper] radians, from arm_interface/urdf/arm.urdf <limit> tags.
_LIMITS = [(-2.745, 2.745), (0, 3.14), (-2.220, 0), (0, 4.505), (-3.62, 0)]


def fk(q):
    P = np.eye(4)
    qi = 0
    for j in _JOINTS:
        P = P @ _T(j["xyz"], j["rpy"])
        if j["actuated"]:
            P = P @ _axis_angle(j["axis"], q[qi])
            qi += 1
    return P[:3, 3]


def sample_to_csv(n, seed, path):
    rng = np.random.default_rng(seed)
    q = np.stack([rng.uniform(lo, hi, n) for lo, hi in _LIMITS], axis=1)
    points = np.array([fk(q[i]) for i in range(n)])
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        for p in points:
            writer.writerow([f"{p[0]:.5f}", f"{p[1]:.5f}", f"{p[2]:.5f}"])
    r = np.linalg.norm(points, axis=1)
    print(f"{path}: N={n} reach=[{r.min():.4f}, {r.max():.4f}] m")


if __name__ == "__main__":
    sample_to_csv(50_000, seed=101, path="workspace_points_50k.csv")
    sample_to_csv(200_000, seed=102, path="workspace_points_200k.csv")
