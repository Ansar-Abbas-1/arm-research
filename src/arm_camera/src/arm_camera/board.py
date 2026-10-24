"""Chessboard detection and board pose estimation (OpenCV, no ROS)."""
from dataclasses import dataclass

import cv2
import numpy as np

from arm_camera.geometry import make_transform


@dataclass(frozen=True)
class Board:
    """A printed chessboard.

    cols, rows: INNER corners along the board's x and y (a 10 x 7 squares board has
    9 x 6 inner corners). square_size: MEASURED edge of one square in metres; printers
    scale, so measure it with a ruler instead of trusting the print dialog.
    """
    cols: int
    rows: int
    square_size: float

    def object_points(self):
        """(N, 3) float32 corner coordinates in the board frame: origin at the first
        inner corner, x along the columns, y along the rows, z = 0. The order matches
        what cv2.findChessboardCorners returns (row-major)."""
        pts = np.zeros((self.rows * self.cols, 3), np.float32)
        pts[:, 0] = np.tile(np.arange(self.cols), self.rows) * self.square_size
        pts[:, 1] = np.repeat(np.arange(self.rows), self.cols) * self.square_size
        return pts

    def is_unambiguous(self):
        """A board with an even number of corners on one side and odd on the other
        has a unique orientation; otherwise detection can return the pattern rotated
        by 180 degrees, which silently ruins a hand-eye calibration."""
        return (self.cols % 2) != (self.rows % 2)

    @property
    def pattern_size(self):
        return (self.cols, self.rows)


def _cell_levels(gray, corners, board):
    """Mean grey level of the board cell that touches the origin corner on its +x,+y side
    and of the next cell along +x (which has the opposite colour)."""
    c = corners.reshape(-1, 2)
    first = c[0] + 0.5 * (c[1] - c[0]) + 0.5 * (c[board.cols] - c[0])
    neighbour = first + (c[1] - c[0])

    def level(p):
        x, y = int(round(p[0])), int(round(p[1]))
        patch = gray[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        return float(patch.mean()) if patch.size else float("nan")

    return level(first), level(neighbour)


def first_square_is_dark(gray, corners, board):
    """Colour of the board cell that touches the origin corner on its +x,+y side, read
    from the image. Detectors may label the board from either end (a 180 degree turn in
    its plane); detect_corners() uses this to make the labelling identical in every view."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    a, b = _cell_levels(gray, corners, board)
    return a < b


MIN_CELL_CONTRAST = 12.0   # grey levels between adjacent cells below which the colour is not trusted


def detect_corners(gray, board, canonical=True):
    """Sub-pixel inner corners as an (N, 1, 2) float32 array, or None if not found.

    The classic detector (findChessboardCorners plus cornerSubPix) is tried first: on 88
    real 1080p views of a 9 x 6 board it labelled the board identically in every view,
    while findChessboardCornersSB, tried second as a fallback, flipped the labelling by
    180 degrees in half of them. With canonical=True (default) the corners are then put in
    the one order in which the origin cell is dark, so the board frame is the same in every
    view whatever the detector did. Returns None if the cell colours cannot be told apart
    (low contrast, heavy blur), because the order would be a guess."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board.pattern_size, flags)
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-5)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    elif hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, board.pattern_size, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        return None
    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    if canonical:
        a, b = _cell_levels(gray, corners, board)
        if not np.isfinite(a) or not np.isfinite(b) or abs(a - b) < MIN_CELL_CONTRAST:
            return None
        if not a < b:
            corners = corners[::-1].copy()
    return corners


def project(K, D, T_cam_board, board_points):
    """Pixel coordinates (N, 2) of board-frame points seen by a camera at T_cam_board."""
    rvec, _ = cv2.Rodrigues(T_cam_board[:3, :3])
    px, _ = cv2.projectPoints(np.asarray(board_points, np.float64).reshape(-1, 1, 3), rvec,
                              T_cam_board[:3, 3].reshape(3, 1), K, D)
    return px.reshape(-1, 2)


def reprojection_rms(K, D, T_cam_board, board, corners):
    """RMS pixel distance between the detected corners and the projected board model."""
    err = project(K, D, T_cam_board, board.object_points()) - corners.reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(err ** 2, axis=1))))


def estimate_pose(corners, board, K, D):
    """T_cam_board (4x4) and the reprojection RMS [px] from detected corners.

    The board is planar, so the pose is first solved with IPPE (handles the planar
    two-fold ambiguity) and then refined with Levenberg-Marquardt on all corners."""
    obj = board.object_points().astype(np.float64)
    img = corners.reshape(-1, 1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, float("inf")
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    T = make_transform(R, tvec.reshape(3))
    return T, reprojection_rms(K, D, T, board, corners)


def camera_matrix_from_info(k):
    """3x3 matrix from a sensor_msgs/CameraInfo .K list; None if it is all zeros
    (usb_cam publishes zeros when no calibration file was loaded)."""
    K = np.array(k, dtype=np.float64).reshape(3, 3)
    return None if not np.any(K) else K
