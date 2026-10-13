"""Classical (non-network) plant segmentation baselines (numpy + cv2): green indices, shape filtering and instance splitting. Used to compare
against the YOLO network on labeled images; nothing here is ROS."""
import cv2
import numpy as np


def exg(img_bgr, min_sum=90):
    """Excess green 2g - r - b on chromatic coordinates (each channel divided by the sum), in [-1, 2]. Pixels whose channel sum is below `min_sum`
    (almost black: the ratios are noise there) get 0."""
    f = img_bgr.astype(np.float32)
    s = f.sum(axis=2)
    d = np.maximum(s, 1.0)
    b, g, r = f[:, :, 0] / d, f[:, :, 1] / d, f[:, :, 2] / d
    return np.where(s >= min_sum, 2.0 * g - r - b, 0.0).astype(np.float32)


def otsu(values, bins=256):
    """Otsu threshold of a float array."""
    lo, hi = float(values.min()), float(values.max())
    if hi - lo < 1e-9:
        return lo
    q = np.clip(((values - lo) / (hi - lo) * (bins - 1)).astype(np.int32), 0, bins - 1)
    hist = np.bincount(q.ravel(), minlength=bins).astype(np.float64)
    p = hist / hist.sum()
    w = np.cumsum(p)
    m = np.cumsum(p * np.arange(bins))
    mt = m[-1]
    sb = (mt * w - m) ** 2 / np.maximum(w * (1.0 - w), 1e-12)
    return lo + (int(np.argmax(sb)) + 0.5) / (bins - 1) * (hi - lo)


def green_mask_exg(img_bgr, thr=None):
    e = exg(img_bgr)
    return e > (otsu(e) if thr is None else thr)


def green_mask_hsv(img_bgr, hue=(30, 90), min_sat=60, min_val=40):
    """Green by hue (OpenCV scale, 0-179), with a saturation and value floor (soil, tray and pavement are dark or unsaturated)."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    return (h >= hue[0]) & (h <= hue[1]) & (s >= min_sat) & (v >= min_val)


def remove_thin(mask, radius):
    """Morphological opening with a disc: removes structures thinner than 2 * radius + 1 px (grass blades) and keeps broad leaves."""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    return cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, k).astype(bool)


def remove_small(mask, min_area):
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    keep = np.zeros(n, bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_area
    return keep[lab]


def split_instances(mask, min_dist=0.75, min_area=200):
    """Instances of a binary mask: each connected blob is split with a watershed seeded at the points whose distance to the border is at least
    `min_dist` times the blob's own maximum (touching leaves have a narrow neck; a blob of one leaf has one seed). Returns a list of dicts
    {area, centroid, box} in image pixels."""
    m = mask.astype(np.uint8)
    dist = cv2.distanceTransform(m, cv2.DIST_L2, 5)
    n_cc, cc = cv2.connectedComponents(m)
    seeds = np.zeros(m.shape, np.uint8)
    for k in range(1, n_cc):
        blob = cc == k
        mx = dist[blob].max()
        if mx > 0:
            seeds[blob & (dist >= min_dist * mx)] = 1
    n, markers = cv2.connectedComponents(seeds)
    markers = markers.astype(np.int32) + 1                          # background 1, seeds 2.. ; foreground not yet reached stays 0 (unknown)
    markers[(m > 0) & (seeds == 0)] = 0
    ws = cv2.watershed(cv2.cvtColor((m * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR), markers)
    out = []
    for lab in range(2, n + 1):
        ys, xs = np.nonzero((ws == lab) & (m > 0))
        if len(xs) < min_area:
            continue
        out.append({"area": int(len(xs)), "centroid": (float(xs.mean()), float(ys.mean())), "box": (int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1)})
    return out


def pixel_metrics(pred, gt):
    """Pixel-level precision, recall, F1 and IoU of two boolean masks."""
    tp = float(np.logical_and(pred, gt).sum())
    fp = float(np.logical_and(pred, ~gt).sum())
    fn = float(np.logical_and(~pred, gt).sum())
    p = tp / max(1.0, tp + fp)
    r = tp / max(1.0, tp + fn)
    return {"precision": p, "recall": r, "f1": 2 * p * r / max(1e-9, p + r), "iou": tp / max(1.0, tp + fp + fn)}


def instance_hits(centroids, gt_instance_masks):
    """Plant-level scores by centroid: recall = fraction of ground-truth instances containing at least one predicted centroid; precision = fraction
    of predicted centroids lying inside some ground-truth instance. `gt_instance_masks` is a list of boolean masks."""
    h, w = gt_instance_masks[0].shape if gt_instance_masks else (0, 0)
    covered = np.zeros(len(gt_instance_masks), bool)
    inside = 0
    for (cx, cy) in centroids:
        x, y = int(round(cx)), int(round(cy))
        hit = False
        if 0 <= x < w and 0 <= y < h:
            for i, m in enumerate(gt_instance_masks):
                if m[y, x]:
                    covered[i] = True
                    hit = True
        inside += int(hit)
    return {"recall": float(covered.mean()) if len(covered) else 0.0, "precision": inside / max(1, len(centroids)), "n_pred": len(centroids), "n_gt": len(gt_instance_masks)}


def instance_hits_labels(centroids, label_map):
    """Like instance_hits, with the ground truth as an integer label map (0 = none, 1..n = instance ids; overlaps resolved by drawing order):
    recall = fraction of the ids covered by a predicted centroid, precision = fraction of centroids on a labeled pixel."""
    n = int(label_map.max())
    h, w = label_map.shape
    covered, inside = set(), 0
    for (cx, cy) in centroids:
        x, y = int(round(cx)), int(round(cy))
        if 0 <= x < w and 0 <= y < h and label_map[y, x] > 0:
            covered.add(int(label_map[y, x]))
            inside += 1
    return {"recall": len(covered) / max(1, n), "precision": inside / max(1, len(centroids)), "n_pred": len(centroids), "n_gt": n}
