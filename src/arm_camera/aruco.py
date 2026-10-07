"""ArUco marker detection and pose (OpenCV aruco + numpy, no ROS).

Marker frame (OpenCV convention): origin at the marker centre, x to the right and y UP in the
marker's own picture, z out of the marker toward the viewer. `marker_size` is the side of the
whole BLACK square including its black border (not the white margin around it, and not just the
inner bit pattern): the distance the pose gives scales linearly with it, so measure it on the
printed marker. Corner order of detections: top-left, top-right, bottom-right, bottom-left.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from arm_camera.geometry import make_transform

_NAMES = ["4X4_50", "4X4_100", "4X4_250", "4X4_1000", "5X5_50", "5X5_100", "5X5_250", "5X5_1000",
          "6X6_50", "6X6_100", "6X6_250", "6X6_1000", "7X7_50", "7X7_100", "7X7_250", "7X7_1000",
          "ARUCO_ORIGINAL", "APRILTAG_16h5", "APRILTAG_25h9", "APRILTAG_36h10", "APRILTAG_36h11"]
# Smallest dictionaries first: a marker of a small dictionary is also read by the bigger ones
# with the same id, so "auto" must try the small ones first to name the right dictionary.
DICTIONARIES = {n: getattr(cv2.aruco, "DICT_" + n) for n in _NAMES if hasattr(cv2.aruco, "DICT_" + n)}


@dataclass
class Detection:
    marker_id: int
    corners: np.ndarray        # (4, 2) float32 pixels
    dictionary: str


def get_dictionary(name):
    return cv2.aruco.Dictionary_get(DICTIONARIES[name])


def detector_parameters(refine=None, quad_decimate=0.0):
    """OpenCV detector settings. Corner refinement is by line fitting (CORNER_REFINE_APRILTAG): measured on the real
    camera (soft focus, 80 frames of a still scene, 2026-10-07) the 100 mm marker's depth noise was 9.5 mm with
    CORNER_REFINE_SUBPIX (window 5 px: the corner search latches onto the blur of a large marker; edge length
    std 6.8 px), 1.0 mm with a 15 px window, and 0.1 mm (0.04 px) with the line fit, which also gave the 50 mm
    markers 0.2 mm instead of 0.6 to 0.8 mm. `refine` overrides the method for experiments.

    quad_decimate: factor by which the image is shrunk for the quad (marker outline) search only; the corner fit
    still uses full resolution. Per frame at 1920x1080 with two dictionaries: 269 ms at 0, 96 ms at 2, 28 ms at 4,
    with the same accuracy. The cost is range: a marker needs an edge of roughly 20 px after the shrink, i.e. 40 px
    at 2 and 80 px at 4 (a 50 mm marker at 1.5 m is about 50 px). Default 0: exact, and the ChArUco board has 14 mm
    markers that the shrink would lose."""
    p = cv2.aruco.DetectorParameters_create()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG if refine is None else refine
    p.aprilTagQuadDecimate = float(quad_decimate)
    return p


def detect(gray, dictionary_name, parameters=None):
    """Markers of one dictionary found in a grey image."""
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, get_dictionary(dictionary_name), parameters=parameters or detector_parameters())
    if ids is None:
        return []
    return [Detection(int(i), np.asarray(c, np.float32).reshape(4, 2), dictionary_name) for i, c in zip(ids.reshape(-1), corners)]


def detect_auto(gray, names=None):
    """(dictionary name, detections) for the first dictionary, smallest first, that reads
    anything; (None, []) if none does. Use it once, then lock the dictionary. Markers whose id
    is small are identical in a dictionary and its larger siblings (id 7 of 5X5_100 is id 7 of
    5X5_50), so the name returned is the smallest dictionary that reads the marker, which is
    all a pose needs; only ids beyond the small dictionary's range name the bigger one."""
    for name in names or DICTIONARIES:
        found = detect(gray, name)
        if found:
            return name, found
    return None, []


def marker_object_points(marker_size):
    h = marker_size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], np.float64)


def estimate_pose(corners, marker_size, K, D):
    """(T_cam_marker 4x4, reprojection RMS [px], ambiguity ratio) for one detection.

    A square seen at a small angle has two nearly equally good poses (IPPE returns both). The
    ambiguity ratio is best error / second-best error: close to 1 means the pose may be the
    mirrored one and its tilt should not be trusted; well below 1 is unambiguous. The best
    solution is refined with Levenberg-Marquardt."""
    obj = marker_object_points(marker_size)
    img = np.asarray(corners, np.float64).reshape(4, 1, 2)
    # The OpenCV 4.2 python binding allocates solvePnPGeneric's error output as 8-bit unless a
    # float array is passed in; the errors are recomputed below anyway.
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE,
                                             reprojectionError=np.zeros((2, 1), np.float64))
    if n < 1:
        return None, float("inf"), 1.0
    errs = np.array([np.sqrt(np.mean(np.sum((cv2.projectPoints(obj, r, t, K, D)[0].reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1)))
                     for r, t in zip(rvecs, tvecs)])
    order = np.argsort(errs)
    best = order[0]
    ratio = float(errs[best] / errs[order[1]]) if n > 1 and errs[order[1]] > 0 else 0.0
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, D, rvecs[best], tvecs[best])
    R, _ = cv2.Rodrigues(rvec)
    T = make_transform(R, tvec.reshape(3))
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2, axis=1))))
    return T, rms, ratio


def distance(T_cam_marker):
    """Euclidean distance from the camera's optical centre to the marker centre [m]."""
    return float(np.linalg.norm(T_cam_marker[:3, 3]))


def render_marker(dictionary_name, marker_id, side_px):
    """Grey image of the marker: the black border square is the whole image, side_px wide."""
    return cv2.aruco.drawMarker(get_dictionary(dictionary_name), marker_id, side_px)


def marker_cells(dictionary_name, marker_id):
    """Boolean matrix of the marker's cells including the 1-cell black border (True = black)."""
    bits = get_dictionary(dictionary_name).markerSize
    img = cv2.aruco.drawMarker(get_dictionary(dictionary_name), marker_id, bits + 2, borderBits=1)
    return img < 128


def marker_svg(dictionary_name, marker_id, size_mm, margin_mm=None, label=True):
    """SVG of the marker with exact physical dimensions in millimetres (print at 100 %, no
    'fit to page'): the black square is size_mm wide, surrounded by a white quiet margin."""
    cells = marker_cells(dictionary_name, marker_id)
    n = cells.shape[0]
    margin = size_mm * 0.25 if margin_mm is None else margin_mm
    side = size_mm + 2 * margin
    cell = size_mm / n
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="%.2fmm" height="%.2fmm" viewBox="0 0 %.3f %.3f">' % (side, side + (8 if label else 0), side, side + (8 if label else 0)),
           '<rect width="%.3f" height="%.3f" fill="white"/>' % (side, side + (8 if label else 0))]
    for r in range(n):
        for c in range(n):
            if cells[r, c]:
                out.append('<rect x="%.4f" y="%.4f" width="%.4f" height="%.4f" fill="black"/>' % (margin + c * cell, margin + r * cell, cell + 0.001, cell + 0.001))
    if label:
        out.append('<text x="%.2f" y="%.2f" font-size="3.2" font-family="monospace">%s id %d, black square %.1f mm</text>' % (margin, side + 5, dictionary_name, marker_id, size_mm))
    out.append("</svg>")
    return "\n".join(out)
