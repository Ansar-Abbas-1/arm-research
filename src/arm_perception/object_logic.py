"""Pure logic of the object detector (numpy + cv2, no ROS, no onnxruntime): YOLO preprocessing and decoding, NMS, and the pixel to
floor-plane geometry that turns a detection in ONE image into a 3D position (the camera has no depth).

Layouts of the first ONNX output, `fmt` (auto picks from the shape and the class count):
    v8   (1, 4+nc, N)   YOLOv8 / YOLO11: cx, cy, w, h, then one score per class (a -seg model has 32 extra mask coefficients, dropped)
    v5   (1, N, 5+nc)   YOLOv5:          cx, cy, w, h, objectness, then one score per class
    e2e  (1, N, 6)      NMS-free export:  x1, y1, x2, y2, score, class        (already suppressed; a YOLO26 -seg export has 32 more columns of mask
                        coefficients, 38 in all, which are not read: only the box is used)

Geometry: the camera ray through a pixel (undistorted with the calibration) is intersected with the horizontal plane z = z_plane in the
base frame. That is exact for a point lying in that plane, so the detector reports the point of the object at `grasp_height` above the floor
using the bounding-box centre: an approximation for a compact object, wrong for one whose box centre is not above its grasp point.
"""
import cv2
import numpy as np


def letterbox(img, size):
    """BGR image -> (NCHW float32 RGB blob in [0, 1], scale, (pad_x, pad_y)): resized to fit size x size keeping the aspect, padded grey."""
    h, w = img.shape[:2]
    scale = min(size / float(w), size / float(h))
    nw, nh = int(round(w * scale)), int(round(h * scale))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = resized
    blob = canvas[:, :, ::-1].astype(np.float32) / 255.0
    return np.ascontiguousarray(blob.transpose(2, 0, 1)[None]), scale, (px, py)


def detect_format(shape, num_classes=None):
    """'v8' (channels first), 'v8_t' (the same, transposed), 'v5' or 'e2e' from the output shape (1, A, B); the class count, when known,
    decides first (a real model has far more candidates than channels, but a count is unambiguous)."""
    _, a, b = shape
    if b in (6, 6 + 32) and a <= 1000:                  # NMS-free: (x1, y1, x2, y2, score, class[, 32 mask coefficients of a -seg model])
        return "e2e"
    if num_classes is not None:
        if a in (4 + num_classes, 4 + num_classes + 32):
            return "v8"
        if b == 5 + num_classes:
            return "v5"
        if b in (4 + num_classes, 4 + num_classes + 32):
            return "v8_t"
    return "v8" if a < b else "v5"


def decode(output, fmt="auto", conf=0.25, num_classes=None):
    """-> (boxes_xyxy (n, 4) in the letterboxed input pixels, scores (n,), class ids (n,)), before NMS (e2e is already suppressed)."""
    out = np.asarray(output)
    if out.ndim != 3:
        raise ValueError("expected a 3D output, got shape %s" % (out.shape,))
    if fmt == "auto":
        fmt = detect_format(out.shape, num_classes)
    if fmt == "e2e":
        arr = out[0]
        keep = arr[:, 4] >= conf
        arr = arr[keep]
        return arr[:, :4].astype(np.float32), arr[:, 4].astype(np.float32), arr[:, 5].astype(int)
    if fmt == "v8":
        arr = out[0].T                               # (N, 4+nc[+32])
    elif fmt == "v8_t":
        arr = out[0]
    elif fmt == "v5":
        arr = out[0]
    else:
        raise ValueError("unknown format %r" % fmt)
    if fmt == "v5":
        nc = arr.shape[1] - 5 if num_classes is None else num_classes
        scores_all = arr[:, 5:5 + nc] * arr[:, 4:5]
    else:
        nc = arr.shape[1] - 4 if num_classes is None else num_classes
        scores_all = arr[:, 4:4 + nc]
    cls = scores_all.argmax(axis=1)
    sc = scores_all[np.arange(len(cls)), cls]
    keep = sc >= conf
    arr, cls, sc = arr[keep], cls[keep], sc[keep]
    cx, cy, w, h = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
    boxes = np.stack([cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0], axis=1)
    return boxes.astype(np.float32), sc.astype(np.float32), cls.astype(int)


def nms(boxes, scores, classes, iou=0.45, metric="iou"):
    """Class-aware non-maximum suppression; returns the indices to keep, best first. metric 'iou' (intersection over union) or 'ios'
    (intersection over the SMALLER box, SAHI's default for merging tiles: it also removes a small box lying inside a bigger one)."""
    if len(boxes) == 0:
        return np.zeros(0, int)
    shifted = boxes + (classes[:, None].astype(np.float32) * 10000.0)         # boxes of different classes never overlap
    order = scores.argsort()[::-1]
    keep = []
    areas = (shifted[:, 2] - shifted[:, 0]) * (shifted[:, 3] - shifted[:, 1])
    while len(order):
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(shifted[i, 0], shifted[rest, 0])
        yy1 = np.maximum(shifted[i, 1], shifted[rest, 1])
        xx2 = np.minimum(shifted[i, 2], shifted[rest, 2])
        yy2 = np.minimum(shifted[i, 3], shifted[rest, 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        denom = np.minimum(areas[i], areas[rest]) if metric == "ios" else areas[i] + areas[rest] - inter
        order = rest[inter / np.maximum(denom, 1e-9) <= iou]
    return np.array(keep, int)


def unletterbox(boxes, scale, pad, shape):
    """Boxes in the letterboxed input -> boxes in the original image (h, w)."""
    out = boxes.copy().astype(np.float32)
    out[:, [0, 2]] = (out[:, [0, 2]] - pad[0]) / scale
    out[:, [1, 3]] = (out[:, [1, 3]] - pad[1]) / scale
    out[:, [0, 2]] = np.clip(out[:, [0, 2]], 0, shape[1] - 1)
    out[:, [1, 3]] = np.clip(out[:, [1, 3]], 0, shape[0] - 1)
    return out


def pixel_ray(u, v, K, D=None):
    """Unit direction in the camera optical frame (x right, y down, z forward) of the ray through pixel (u, v), undistorted."""
    pt = np.array([[[float(u), float(v)]]], np.float64)
    und = cv2.undistortPoints(pt, np.asarray(K, np.float64).reshape(3, 3), None if D is None else np.asarray(D, np.float64))
    d = np.array([und[0, 0, 0], und[0, 0, 1], 1.0])
    return d / np.linalg.norm(d)


def intersect_plane(origin, direction, z_plane, max_range=3.0):
    """Point where the ray origin + t * direction (t > 0, base frame) meets the horizontal plane z = z_plane, or None."""
    origin, direction = np.asarray(origin, float), np.asarray(direction, float)
    if abs(direction[2]) < 1e-6:
        return None
    t = (z_plane - origin[2]) / direction[2]
    if t <= 0.0 or t > max_range:
        return None
    return origin + t * direction


def object_position(box_xyxy, T_base_camera, K, D, z_plane, max_range=3.0):
    """3D position (base frame) of the bounding-box centre on the plane z = z_plane, or None when the ray does not reach it."""
    u, v = 0.5 * (box_xyxy[0] + box_xyxy[2]), 0.5 * (box_xyxy[1] + box_xyxy[3])
    d_cam = pixel_ray(u, v, K, D)
    R, o = T_base_camera[:3, :3], T_base_camera[:3, 3]
    return intersect_plane(o, R.dot(d_cam), z_plane, max_range)


class FloorMemory(object):
    """The floor height remembered from the ground marker (marker 0): the floor does not move, so the last value is used for max_age seconds
    after the marker was last seen, then the fallback parameter."""

    def __init__(self, max_age=600.0):
        self.max_age = max_age
        self.z, self.t = None, None

    def update(self, z, t):
        if self.t is None or t >= self.t:
            self.z, self.t = float(z), float(t)

    def get(self, now, fallback):
        """(z, source, age): source is 'marker_0' while the remembered value is fresh enough, else 'param'."""
        if self.z is not None and 0.0 <= now - self.t <= self.max_age:
            return self.z, "marker_0", now - self.t
        return fallback, "param", None


def tile_grid(width, height, tile=512, overlap=0.2):
    """Tiles (x0, y0, x1, y1) covering a width x height image like SAHI's slicing: `tile` px squares stepping by tile * (1 - overlap), the last
    row and column shifted back so every tile is full size; an image smaller than a tile is one tile. Small objects keep their native scale."""
    if width <= tile and height <= tile:
        return [(0, 0, width, height)]
    step = max(1, int(tile * (1.0 - overlap)))

    def starts(size):
        if size <= tile:
            return [0]
        out, x = [], 0
        while x + tile < size:
            out.append(x)
            x += step
        out.append(size - tile)
        return out

    return [(x, y, min(x + tile, width), min(y + tile, height)) for y in starts(height) for x in starts(width)]


def box_iou(a, b):
    """IoU of two xyxy boxes."""
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = iw * ih
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def match_detections(dets, gts, iou=0.5):
    """Greedy one-to-one matching of detections [(box, score)] to ground-truth boxes by decreasing score at IoU >= `iou`.
    -> (true positives, false positives, false negatives)."""
    used = set()
    tp = 0
    for box, _ in sorted(dets, key=lambda d: -d[1]):
        best, best_i = 0.0, None
        for i, g in enumerate(gts):
            if i in used:
                continue
            v = box_iou(box, g)
            if v > best:
                best, best_i = v, i
        if best_i is not None and best >= iou:
            used.add(best_i)
            tp += 1
    return tp, len(dets) - tp, len(gts) - tp
