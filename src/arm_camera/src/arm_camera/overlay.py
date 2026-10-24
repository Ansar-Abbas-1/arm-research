"""Overlay drawing and rviz geometry for the calibration check (numpy/OpenCV, no ROS)."""
import cv2
import numpy as np

GREEN = (0, 220, 0)
RED = (0, 0, 255)
YELLOW = (0, 220, 255)


def corner_errors(detected, predicted):
    """Per-corner pixel distance between detected corners and the chain's prediction."""
    return np.linalg.norm(np.asarray(detected).reshape(-1, 2) - np.asarray(predicted).reshape(-1, 2), axis=1)


def draw_overlay(image_bgr, detected, predicted, extra_text=None, scale=1.0):
    """Annotated copy: detected corners (green rings), the prediction from joint states +
    forward kinematics + hand-eye + intrinsics (red crosses), a yellow line joining each
    pair, and the RMS error. `detected` may be None (board not found in this frame)."""
    out = image_bgr.copy() if image_bgr.ndim == 3 else cv2.cvtColor(image_bgr, cv2.COLOR_GRAY2BGR)
    s = max(1.0, scale)
    pred = np.asarray(predicted).reshape(-1, 2)
    text = []
    if detected is not None:
        det = np.asarray(detected).reshape(-1, 2)
        err = corner_errors(det, pred)
        for d, p in zip(det, pred):
            cv2.line(out, tuple(int(v) for v in d), tuple(int(v) for v in p), YELLOW, max(1, int(s)))
            cv2.circle(out, tuple(int(v) for v in d), int(6 * s), GREEN, max(1, int(s)))
        text.append("calibration check: RMS %.1f px, max %.1f px" % (float(np.sqrt(np.mean(err ** 2))), float(err.max())))
    else:
        text.append("board not detected in this frame")
    for p in pred:
        c = (int(p[0]), int(p[1]))
        cv2.drawMarker(out, c, RED, cv2.MARKER_CROSS, int(12 * s), max(1, int(s)))
    if extra_text:
        text.extend(extra_text)
    for i, line in enumerate(text):
        y = int(28 * s) * (i + 1)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * s, (0, 0, 0), max(2, int(3 * s)), cv2.LINE_AA)
        cv2.putText(out, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * s, (255, 255, 255), max(1, int(1.5 * s)), cv2.LINE_AA)
    return out


def board_triangles(board, first_dark):
    """Checkerboard as coloured triangles in the board frame, for an rviz TRIANGLE_LIST.

    Returns (points (M, 3), colours (M, 3) in 0..1), M = 6 per cell. The board frame has its
    origin at the first inner corner, so the printed board spans cells i = -1 .. cols-1 and
    j = -1 .. rows-1; cell (0, 0) is the one touching the origin on the +x,+y side and is dark
    when first_dark (see board.first_square_is_dark)."""
    sq = board.square_size
    pts, cols = [], []
    for i in range(-1, board.cols):
        for j in range(-1, board.rows):
            dark = (((i + j) % 2) == 0) == bool(first_dark)
            c = (0.05, 0.05, 0.05) if dark else (0.95, 0.95, 0.95)
            x0, y0, x1, y1 = i * sq, j * sq, (i + 1) * sq, (j + 1) * sq
            for p in ((x0, y0), (x1, y0), (x1, y1), (x0, y0), (x1, y1), (x0, y1)):
                pts.append((p[0], p[1], 0.0))
                cols.append(c)
    return np.array(pts), np.array(cols)


def frustum_lines(K, image_size, depth=0.15):
    """Line-list endpoints (M, 3) in the camera optical frame: the four corner rays and the
    image rectangle at `depth` metres."""
    w, h = image_size
    corners_px = [(0, 0), (w, 0), (w, h), (0, h)]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    c3 = [((u - cx) / fx * depth, (v - cy) / fy * depth, depth) for u, v in corners_px]
    o = (0.0, 0.0, 0.0)
    lines = []
    for p in c3:
        lines += [o, p]
    for a, b in zip(c3, c3[1:] + c3[:1]):
        lines += [a, b]
    return np.array(lines)
