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


def detect_corners(gray, board):
    """Sub-pixel inner corners as an (N, 1, 2) float32 array, or None if not found.

    Tries the sector-based detector (cv2.findChessboardCornersSB, more robust at high
    resolution, already sub-pixel) and falls back to the classic one plus cornerSubPix."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    found, corners = False, None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, board.pattern_size, cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, board.pattern_size, flags)
        if not found:
            return None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
        corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
    return np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)


def first_square_is_dark(gray, corners, board):
    """Colour of the board cell that touches the origin corner on its +x,+y side, read
    from the image. The detector's corner order depends on which square is dark at the
    printed board's corner: with a black top-left square the labelling starts at the far
    end (the board frame is rotated 180 degrees in its plane). That is consistent across
    frames, so calibration is unaffected, but anything that DRAWS the board needs it."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    c = corners.reshape(-1, 2)
    first = c[0] + 0.5 * (c[1] - c[0]) + 0.5 * (c[board.cols] - c[0])
    neighbour = first + (c[1] - c[0])   # the next cell along +x has the opposite colour

    def level(p):
        x, y = int(round(p[0])), int(round(p[1]))
        patch = gray[max(0, y - 3):y + 4, max(0, x - 3):x + 4]
        return float(patch.mean())

    return level(first) < level(neighbour)


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
