"""Geometry for cutting labeled photos into fixed-size training crops at several scales (numpy only, no ROS).

Used by scripts/make_tile_dataset.py to turn a few large Roboflow-labeled photos into many 640 px crops: every photo is resized by each scale in a
list (so the same plant is seen at several apparent sizes: the arm camera sees plants much larger than the field seedlings the first model was
trained on), a regular grid of crops is laid over it, and each polygon label is clipped to the crop. A polygon that is mostly outside a crop
(less than `min_ratio` of its area inside) is dropped from that crop rather than kept as a sliver.

Polygons are (N, 2) arrays of pixel coordinates (x, y) in the SCALED photo. The clipping is Sutherland-Hodgman against an axis-aligned rectangle,
exact for convex polygons and acceptable for leaf outlines (a concave polygon may gain a zero-width bridge along the crop border).
"""
import numpy as np


def polygon_area(poly):
    """Absolute area of a polygon (shoelace)."""
    p = np.asarray(poly, float)
    if len(p) < 3:
        return 0.0
    x, y = p[:, 0], p[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _clip_half_plane(pts, axis, value, keep_greater):
    """Keeps the part of the polygon with coordinate[axis] >= value (keep_greater) or <= value."""
    out = []
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        da, db = (a[axis] - value), (b[axis] - value)
        ina, inb = (da >= 0) if keep_greater else (da <= 0), (db >= 0) if keep_greater else (db <= 0)
        if ina:
            out.append(a)
        if ina != inb:
            t = da / (da - db)
            out.append(a + t * (b - a))
    return out


def clip_polygon(poly, x0, y0, x1, y1):
    """The polygon clipped to the rectangle [x0, x1] x [y0, y1], as an (M, 2) array (empty when nothing is inside)."""
    pts = [np.asarray(p, float) for p in poly]
    for axis, value, greater in ((0, x0, True), (0, x1, False), (1, y0, True), (1, y1, False)):
        if not pts:
            break
        pts = _clip_half_plane(pts, axis, value, greater)
    return np.array(pts, float).reshape(-1, 2)


def grid_origins(length, tile, stride):
    """Start coordinates of tiles of size `tile` every `stride` along an axis of `length` pixels, the last one flush with the end.
    One origin (0) when the axis is not longer than a tile."""
    if length <= tile:
        return [0]
    starts = list(range(0, length - tile + 1, stride))
    if starts[-1] != length - tile:
        starts.append(length - tile)
    return starts


def tile_labels(polys, scale, x0, y0, tile, min_ratio=0.25, min_area=64.0):
    """Labels of the crop [x0, x0 + tile) x [y0, y0 + tile) of the photo resized by `scale`.
    `polys` is a list of (class_id, (N, 2) polygon in ORIGINAL photo pixels). Returns [(class_id, (M, 2) polygon in crop pixels)], keeping a
    polygon only when at least `min_ratio` of its area (and at least `min_area` px^2) lies inside the crop."""
    out = []
    for cls, poly in polys:
        p = np.asarray(poly, float) * float(scale)
        full = polygon_area(p)
        if full <= 0.0:
            continue
        clipped = clip_polygon(p, x0, y0, x0 + tile, y0 + tile)
        a = polygon_area(clipped)
        if len(clipped) >= 3 and a >= min_area and a / full >= min_ratio:
            out.append((cls, clipped - np.array([x0, y0], float)))
    return out


def to_yolo_seg_line(cls, poly, tile):
    """One line of a YOLO segmentation label file: the class and the polygon normalised by the crop size."""
    q = np.clip(np.asarray(poly, float) / float(tile), 0.0, 1.0)
    return "%d %s" % (cls, " ".join("%.6f" % v for v in q.reshape(-1)))
