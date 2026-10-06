"""Intrinsic calibration from detected chessboard corners (OpenCV + numpy, no ROS).

The camera_calibration GUI fits every sample it accepts, including blurred ones, and gives no
per-view diagnostics. This module fits the same plumb-bob model but reports the error of
every view, drops the worst views one at a time, and estimates how well each parameter is
determined by resampling the views. It works on corners from REAL detections only.
"""
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class IntrinsicsResult:
    K: np.ndarray
    D: np.ndarray
    rms: float                       # RMS over all kept corners [px]
    per_view_rms: np.ndarray         # for the kept views
    kept: list                       # indices (into the input list) of the kept views
    dropped: list = field(default_factory=list)
    spread: dict = field(default_factory=dict)   # resampling std of fx, fy, cx, cy, k1, k2


def _object_points(board):
    return board.object_points().astype(np.float32)


def _fit(corner_sets, indices, board, image_size, flags):
    obj = [_object_points(board)] * len(indices)
    img = [corner_sets[i].reshape(-1, 1, 2).astype(np.float32) for i in indices]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, image_size, None, None, flags=flags)
    per_view = []
    for o, m, r, t in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        per_view.append(float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - m.reshape(-1, 2)) ** 2, axis=1)))))
    return float(rms), K, D, np.array(per_view)


def calibrate(corner_sets, board, image_size, flags=cv2.CALIB_FIX_K3, target_rms=0.6, min_keep=0.6,
              min_views=12, bootstrap=0, seed=0, outlier_factor=1.5):
    """Fit K and D (plumb-bob: k1, k2, p1, p2; k3 fixed to 0 by default).

    Outlier rounds: fit, and while the overall RMS is above target_rms, drop the views whose
    own RMS exceeds outlier_factor times the median view RMS (at least the single worst view,
    never below min_keep of the views and min_views), then refit. Dropping is a judgement
    call (blurred views, a flipped detection), so the dropped views are returned for
    inspection. One fit of ~90 full-HD views takes tens of seconds, hence rounds, not one
    view at a time. bootstrap > 0 refits that many random resamples of the kept views and
    returns the standard deviation of each parameter in `spread`."""
    flags = flags | cv2.CALIB_USE_LU
    kept = list(range(len(corner_sets)))
    floor = max(min_views, int(np.ceil(min_keep * len(kept))))
    dropped = []
    rms, K, D, per_view = _fit(corner_sets, kept, board, image_size, flags)
    while rms > target_rms and len(kept) > floor:
        limit = outlier_factor * float(np.median(per_view))
        order = np.argsort(per_view)[::-1]
        victims = [int(v) for v in order if per_view[v] > limit][: len(kept) - floor] or [int(order[0])]
        for v in sorted(victims, reverse=True):
            dropped.append(kept.pop(v))
        rms, K, D, per_view = _fit(corner_sets, kept, board, image_size, flags)
    result = IntrinsicsResult(K=K, D=D, rms=rms, per_view_rms=per_view, kept=kept, dropped=dropped)
    if bootstrap > 0:
        rng = np.random.RandomState(seed)
        samples = []
        for _ in range(bootstrap):
            pick = [kept[i] for i in rng.randint(0, len(kept), len(kept))]
            _, Kb, Db, _ = _fit(corner_sets, pick, board, image_size, flags)
            Db = np.asarray(Db).reshape(-1)
            samples.append([Kb[0, 0], Kb[1, 1], Kb[0, 2], Kb[1, 2], Db[0], Db[1]])
        sd = np.std(np.array(samples), axis=0)
        result.spread = dict(zip(["fx", "fy", "cx", "cy", "k1", "k2"], [float(v) for v in sd]))
    return result


def coverage(corner_sets, image_size, grid=(4, 3)):
    """Fraction of a grid x grid-cells image partition that contains at least one corner of
    the given views: a calibration that never saw the image edges extrapolates there."""
    w, h = image_size
    hit = np.zeros((grid[1], grid[0]), bool)
    for c in corner_sets:
        p = c.reshape(-1, 2)
        gx = np.clip((p[:, 0] / w * grid[0]).astype(int), 0, grid[0] - 1)
        gy = np.clip((p[:, 1] / h * grid[1]).astype(int), 0, grid[1] - 1)
        hit[gy, gx] = True
    return float(hit.mean())


def camera_info_dict(K, D, image_size, name):
    """The YAML structure of ROS camera_info (plumb_bob, no rectification, P = [K | 0])."""
    w, h = image_size
    P = np.hstack([K, np.zeros((3, 1))])
    flat = lambda a: [float(v) for v in np.asarray(a).reshape(-1)]
    return {
        "image_width": int(w), "image_height": int(h), "camera_name": name,
        "camera_matrix": {"rows": 3, "cols": 3, "data": flat(K)},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": 5, "data": flat(np.asarray(D).reshape(-1)[:5])},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": flat(np.eye(3))},
        "projection_matrix": {"rows": 3, "cols": 4, "data": flat(P)},
    }
