"""ChArUco board detection and camera-mount calibration from a board with a known pose
(OpenCV aruco + numpy, no ROS).

Board frame (OpenCV 4.2, checked empirically): origin at the board's BOTTOM-LEFT outer corner as it
is printed, x along the columns (to the right), y up the page, z out of the paper toward the viewer.
A board lying face up on the ground is therefore a normal right-handed frame with z up, and its pose
in base_link is the position of that corner plus the board's yaw (the direction of its x axis).

The calibration itself:  X = T_ee_cam = inv(T_base_ee) * T_base_board * inv(T_cam_board)
with T_base_ee from forward kinematics, T_cam_board from the detected corners, and T_base_board the
board pose you measured. Every view gives one X; the robust average over views is the answer and the
spread between views is the quality estimate. A DISTANCE alone is not enough: the full 6-DoF board
pose is needed (for a level board: x, y, z and yaw).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from arm_camera import aruco
from arm_camera.geometry import (average_rotation, invert_transform, make_transform, rot_z, rotation_error_deg)


@dataclass(frozen=True)
class CharucoSpec:
    squares_x: int
    squares_y: int
    square_length: float      # metres, MEASURED on the print
    marker_length: float      # metres, MEASURED on the print (the black marker square inside a white square)
    dictionary: str = "4X4_50"

    def create(self):
        return cv2.aruco.CharucoBoard_create(self.squares_x, self.squares_y, self.square_length, self.marker_length,
                                             aruco.get_dictionary(self.dictionary))

    @property
    def width(self):
        return self.squares_x * self.square_length

    @property
    def height(self):
        return self.squares_y * self.square_length

    @property
    def num_corners(self):
        return (self.squares_x - 1) * (self.squares_y - 1)

    def corner_points(self):
        """(N, 3) inner-corner coordinates in the board frame, indexed by ChArUco corner id."""
        return np.asarray(self.create().chessboardCorners, np.float64)


@dataclass
class CharucoDetection:
    corners: np.ndarray    # (M, 2) pixels
    ids: np.ndarray        # (M,) ChArUco corner ids
    num_markers: int


def detect(gray, spec, K=None, D=None, min_corners=6):
    """ChArUco corners found in a grey image, or None if fewer than min_corners."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    board = spec.create()
    mc, mi, _ = cv2.aruco.detectMarkers(gray, aruco.get_dictionary(spec.dictionary), parameters=aruco.detector_parameters())
    if mi is None or len(mi) == 0:
        return None
    kwargs = {}
    if K is not None:
        kwargs = {"cameraMatrix": np.asarray(K, np.float64), "distCoeffs": np.asarray(D if D is not None else np.zeros(5), np.float64)}
    n, ch, chid = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, board, **kwargs)
    if n is None or n < min_corners or ch is None:
        return None
    return CharucoDetection(corners=np.asarray(ch, np.float64).reshape(-1, 2), ids=np.asarray(chid).reshape(-1), num_markers=len(mi))


def estimate_pose(det, spec, K, D):
    """(T_cam_board, reprojection RMS [px], ambiguity ratio) from a detection.

    The corners are coplanar, so IPPE gives two candidate poses; the better one is refined with
    Levenberg-Marquardt. The ambiguity ratio is best error / second-best error: near 1 means the
    pose may be the mirrored one (a nearly frontal board) and its tilt must not be trusted."""
    obj = spec.corner_points()[det.ids].reshape(-1, 1, 3)
    img = det.corners.reshape(-1, 1, 2)
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE, reprojectionError=np.zeros((2, 1), np.float64))
    if n < 1:
        return None, float("inf"), 1.0
    errs = np.array([_rms(obj, img, r, t, K, D) for r, t in zip(rvecs, tvecs)])
    order = np.argsort(errs)
    ratio = float(errs[order[0]] / errs[order[1]]) if n > 1 and errs[order[1]] > 0 else 0.0
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvecs[order[0]], tvecs[order[0]])
    R, _ = cv2.Rodrigues(rvec)
    return make_transform(R, tvec.reshape(3)), _rms(obj, img, rvec, tvec, K, D), ratio


def _rms(obj, img, rvec, tvec, K, D):
    p, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    return float(np.sqrt(np.mean(np.sum((p.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))


def tilt_deg(T_cam_board):
    """Angle between the board normal and the optical axis, 0 = frontal."""
    return float(np.degrees(np.arccos(min(1.0, abs(T_cam_board[2, 2])))))


def board_pose_in_base(x, y, z, yaw_deg, spec, origin="corner"):
    """T_base_board (board frame = OpenCV's, origin at the bottom-left outer corner) for a board
    lying FACE UP and level: x, y, z in metres of the reference point, yaw_deg the direction of the
    board's x axis (counter-clockwise from base x, seen from above). origin='center' means the
    point is the board's centre instead of its corner; z is the height of the printed surface."""
    R = rot_z(np.radians(yaw_deg))
    corner = np.array([x, y, z], float)
    if origin == "center":
        corner = corner - R @ np.array([spec.width / 2.0, spec.height / 2.0, 0.0])
    elif origin != "corner":
        raise ValueError("origin must be 'corner' or 'center'")
    return make_transform(R, corner)


def mount_from_known_board(T_base_ee, T_cam_board, T_base_board):
    """X = T_ee_cam from one view of a board whose pose in the base frame is known."""
    return invert_transform(T_base_ee) @ T_base_board @ invert_transform(T_cam_board)


@dataclass
class MountEstimate:
    X: np.ndarray
    inliers: np.ndarray            # bool per view
    translation_std_mm: np.ndarray  # per axis, over the inliers
    rotation_spread_deg: float      # largest angle between an inlier's rotation and the mean
    translation_spread_mm: float    # largest distance of an inlier from the mean


def robust_mount(Xs, min_translation_tol_mm=2.0):
    """Robust average of per-view mount estimates: views whose translation is further from the
    median than 3 x 1.4826 x MAD (at least min_translation_tol_mm) are rejected as outliers, the
    rest are averaged (mean translation, chordal mean rotation)."""
    Xs = [np.asarray(X) for X in Xs]
    t = np.array([X[:3, 3] for X in Xs])
    med = np.median(t, axis=0)
    dev = np.linalg.norm(t - med, axis=1)
    mad = np.median(dev)
    tol = max(3.0 * 1.4826 * mad, min_translation_tol_mm / 1000.0)
    inliers = dev <= tol
    if inliers.sum() == 0:
        inliers[:] = True
    R = average_rotation([X[:3, :3] for X, ok in zip(Xs, inliers) if ok])
    tm = t[inliers].mean(axis=0)
    X = make_transform(R, tm)
    rot = max(rotation_error_deg(R, X_[:3, :3]) for X_, ok in zip(Xs, inliers) if ok)
    return MountEstimate(X=X, inliers=inliers, translation_std_mm=1000.0 * t[inliers].std(axis=0),
                         rotation_spread_deg=float(rot),
                         translation_spread_mm=float(1000.0 * np.max(np.linalg.norm(t[inliers] - tm, axis=1))))


def render_board_view(spec, T_cam_board, K, size=(1920, 1080), px_per_square=60, margin_px=30):
    """Grey image of the board under a pinhole camera K (no distortion); for tests and demos."""
    board = spec.create()
    w = spec.squares_x * px_per_square + 2 * margin_px
    h = spec.squares_y * px_per_square + 2 * margin_px
    src = board.draw((w, h), marginSize=margin_px, borderBits=1)
    s = spec.square_length / px_per_square                       # metres per source pixel
    # source pixel (u, v) -> board plane: x to the right from the left board edge, y UP from the bottom edge
    S = np.array([[s, 0, -margin_px * s], [0, -s, (h - margin_px) * s], [0, 0, 1]])
    H = K @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ S
    out = cv2.warpPerspective(src, H, size, flags=cv2.INTER_LINEAR, borderValue=170)
    return cv2.GaussianBlur(out, (3, 3), 0.7)


def board_svg(spec, margin_mm=10.0, px_per_mm=10):
    """SVG with the board at its exact physical size (the PNG is embedded at px_per_mm): print at
    100 %, 'actual size'. The page is the board plus a white margin of margin_mm."""
    import base64
    board = spec.create()
    w_mm, h_mm = spec.width * 1000.0, spec.height * 1000.0
    img = board.draw((int(round(w_mm * px_per_mm)), int(round(h_mm * px_per_mm))), marginSize=0, borderBits=1)
    ok, png = cv2.imencode(".png", img)
    data = base64.b64encode(png.tobytes()).decode()
    page_w, page_h = w_mm + 2 * margin_mm, h_mm + 2 * margin_mm + 8
    return ('<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="%.1fmm" height="%.1fmm" viewBox="0 0 %.2f %.2f">'
            '<rect width="%.2f" height="%.2f" fill="white"/>'
            '<image x="%.2f" y="%.2f" width="%.2f" height="%.2f" xlink:href="data:image/png;base64,%s"/>'
            '<text x="%.2f" y="%.2f" font-size="3.2" font-family="monospace">ChArUco %dx%d squares, %s, square %.1f mm, marker %.1f mm. Measure a printed square and the marker.</text></svg>'
            % (page_w, page_h, page_w, page_h, page_w, page_h, margin_mm, margin_mm, w_mm, h_mm, data,
               margin_mm, page_h - 3, spec.squares_x, spec.squares_y, spec.dictionary, spec.square_length * 1000, spec.marker_length * 1000))


def annotated_board_png(spec, px_per_mm=4):
    """PNG of the board with its frame drawn (origin at the bottom-left outer corner, x right, y up,
    z toward you) so you can see which corner and direction the measured pose refers to."""
    board = spec.create()
    w, h = int(round(spec.width * 1000 * px_per_mm)), int(round(spec.height * 1000 * px_per_mm))
    m = 90
    img = np.full((h + 2 * m, w + 2 * m), 255, np.uint8)
    img[m:m + h, m:m + w] = board.draw((w, h), marginSize=0, borderBits=1)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    o = (m, m + h)
    L = int(0.25 * min(w, h))
    cv2.arrowedLine(img, o, (o[0] + L, o[1]), (0, 0, 255), 5, tipLength=0.15)
    cv2.arrowedLine(img, o, (o[0], o[1] - L), (0, 160, 0), 5, tipLength=0.15)
    cv2.circle(img, o, 12, (255, 0, 0), -1)
    cv2.putText(img, "x", (o[0] + L + 12, o[1] + 8), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
    cv2.putText(img, "y", (o[0] - 8, o[1] - L - 14), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 160, 0), 3)
    cv2.putText(img, "origin (z toward you)", (o[0] + 14, o[1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return img
