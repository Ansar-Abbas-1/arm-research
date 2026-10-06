"""Hand-eye calibration for an eye-in-hand camera (OpenCV + numpy, no ROS).

Frames: base (fixed), ee (the end_effector link carrying the camera), cam (camera
optical frame), board (the fixed chessboard). Per sample:
  T_base_ee    from forward kinematics / TF
  T_cam_board  from solvePnP on the detected corners
Unknown:
  X = T_ee_cam     (the pose of the camera in the end_effector frame, constant)
  B = T_base_board (constant, because the board does not move)
For every sample  B = T_base_ee * X * T_cam_board,  which is the classic AX = XB problem
once two samples are subtracted. A correct X makes B come out the same for all samples.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np

from arm_camera import board as board_mod
from arm_camera.geometry import (average_rotation, invert_transform, make_transform, rotation_angle_deg,
                                 rotation_axis, rotation_error_deg)

METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class Sample:
    T_base_ee: np.ndarray
    T_cam_board: np.ndarray
    corners: np.ndarray = None   # (N, 1, 2) detected pixels, needed for the reprojection metric
    name: str = ""


@dataclass
class Result:
    method: str
    X: np.ndarray                 # T_ee_cam
    B: np.ndarray                 # mean T_base_board over the samples
    position_scatter_mm: float    # max distance of any sample's B from the mean
    rotation_scatter_deg: float   # max angle of any sample's B from the mean
    reprojection_rms_px: float = float("nan")
    per_sample_px: list = field(default_factory=list)


def motion_diversity(samples, min_angle_deg=2.0):
    """How well the sample set constrains the problem.

    Returns (max_rotation_deg, max_axis_angle_deg): the largest relative rotation
    between any two arm poses, and the largest angle between two relative-rotation
    axes (both expressed in the end_effector frame). The rotation part of X is only
    determined if there are at least two non-parallel rotation axes, so a set whose
    axes are all parallel (or with no rotation at all) cannot be solved reliably.
    This arm rotates the camera about only two axes (joint_1 about the vertical,
    and the pitch joints about a horizontal axis), so both numbers matter."""
    axes, max_rot = [], 0.0
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            R = samples[i].T_base_ee[:3, :3].T @ samples[j].T_base_ee[:3, :3]
            ang = rotation_angle_deg(R)
            max_rot = max(max_rot, ang)
            if ang >= min_angle_deg:
                axes.append(rotation_axis(R))
    max_axis = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            if axes[i] is None or axes[j] is None:
                continue
            c = abs(float(np.dot(axes[i], axes[j])))
            max_axis = max(max_axis, float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))))
    return float(max_rot), max_axis


def board_poses_in_base(samples, X):
    return [s.T_base_ee @ X @ s.T_cam_board for s in samples]


def mean_pose(poses):
    R = average_rotation([p[:3, :3] for p in poses])
    t = np.mean([p[:3, 3] for p in poses], axis=0)
    return make_transform(R, t)


def consistency(samples, X):
    """(B_mean, max position deviation [mm], max rotation deviation [deg])."""
    poses = board_poses_in_base(samples, X)
    B = mean_pose(poses)
    pos = max(float(np.linalg.norm(p[:3, 3] - B[:3, 3])) for p in poses) * 1000.0
    rot = max(rotation_error_deg(B[:3, :3], p[:3, :3]) for p in poses)
    return B, pos, rot


def reprojection_per_sample(samples, X, B, K, D, board):
    """RMS pixel error per sample when the board pose is B (the same for every
    sample) and the camera pose comes from the arm and X. Needs sample.corners."""
    out = []
    for s in samples:
        T_cam_board_pred = invert_transform(s.T_base_ee @ X) @ B
        pred = board_mod.project(K, D, T_cam_board_pred, board.object_points())
        err = pred - s.corners.reshape(-1, 2)
        out.append(float(np.sqrt(np.mean(np.sum(err ** 2, axis=1)))))
    return out


def solve(samples, method="TSAI", K=None, D=None, board=None):
    """Solve X = T_ee_cam with one OpenCV method and evaluate it on the samples."""
    if len(samples) < 3:
        raise ValueError("hand-eye needs at least 3 samples (use 10 to 20 for a usable result)")
    R_g2b = [s.T_base_ee[:3, :3] for s in samples]
    t_g2b = [s.T_base_ee[:3, 3].reshape(3, 1) for s in samples]
    R_t2c = [s.T_cam_board[:3, :3] for s in samples]
    t_t2c = [s.T_cam_board[:3, 3].reshape(3, 1) for s in samples]
    R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=METHODS[method])
    X = make_transform(R, t.reshape(3))
    B, pos, rot = consistency(samples, X)
    res = Result(method=method, X=X, B=B, position_scatter_mm=pos, rotation_scatter_deg=rot)
    if K is not None and board is not None and all(s.corners is not None for s in samples):
        res.per_sample_px = reprojection_per_sample(samples, X, B, K, D if D is not None else np.zeros(5), board)
        res.reprojection_rms_px = float(np.sqrt(np.mean(np.square(res.per_sample_px))))
    return res


def solve_all(samples, K=None, D=None, board=None):
    """Every method that converges, best first (lowest board-pose scatter, i.e. the X
    that makes the fixed board look most fixed). Methods that throw are skipped."""
    results = []
    for name in METHODS:
        try:
            results.append(solve(samples, name, K, D, board))
        except (cv2.error, ValueError, np.linalg.LinAlgError):
            continue
    results.sort(key=lambda r: r.position_scatter_mm + 10.0 * r.rotation_scatter_deg)
    return results
