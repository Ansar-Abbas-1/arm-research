#!/usr/bin/env python3
"""Offline test of an exported YOLO ONNX detector on photos. NO ROS, no robot: onnxruntime + OpenCV only.

Tiled inference like SAHI (tile 512, overlap 0.2 by default, the setting the Vigna models were trained and used with): each tile keeps its native
scale, is letterboxed to the model input, decoded, suppressed, shifted back into the photo, and the tiles are merged by a global NMS.
Writes an overlay image and a JSON per photo; with --coco it also scores the detections against the COCO boxes (precision and recall at IoU 0.5).

    PYTHONPATH=src python3 scripts/detect_images.py --model vignax3.onnx --source photo_or_folder --out results \\
        [--tile 512 --overlap 0.2 --conf 0.25 --iou 0.5 --coco annotations.json --limit 3 --every 5 --threads 16]
Run it with an interpreter that has onnxruntime and cv2 (the container's /home/dev/venv_yolo/bin/python).
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort

from arm_perception import object_logic as ol


def load_names(session, given):
    if given:
        return [n.strip() for n in given.split(",")]
    meta = session.get_modelmeta().custom_metadata_map.get("names")
    if meta:
        import ast
        d = ast.literal_eval(meta)
        return [d[k] for k in sorted(d)]
    return []


def detect_photo(session, in_name, size, img, tile, overlap, conf, iou, fmt, nc, merge_metric="ios", merge_thr=0.5):
    """-> (boxes xyxy in photo pixels, scores, class ids, number of tiles, seconds in the model)."""
    h, w = img.shape[:2]
    all_b, all_s, all_c, t_model = [], [], [], 0.0
    tiles = ol.tile_grid(w, h, tile, overlap) if tile > 0 else [(0, 0, w, h)]
    for x0, y0, x1, y1 in tiles:
        crop = img[y0:y1, x0:x1]
        blob, scale, pad = ol.letterbox(crop, size)
        t = time.time()
        out = session.run(None, {in_name: blob})[0]
        t_model += time.time() - t
        boxes, scores, cls = ol.decode(out, fmt, conf, nc)
        keep = ol.nms(boxes, scores, cls, iou)
        boxes = ol.unletterbox(boxes[keep], scale, pad, crop.shape[:2])
        boxes[:, [0, 2]] += x0
        boxes[:, [1, 3]] += y0
        all_b.append(boxes)
        all_s.append(scores[keep])
        all_c.append(cls[keep])
    boxes = np.concatenate(all_b) if all_b else np.zeros((0, 4), np.float32)
    scores = np.concatenate(all_s) if all_s else np.zeros(0, np.float32)
    cls = np.concatenate(all_c) if all_c else np.zeros(0, int)
    keep = ol.nms(boxes, scores, cls, merge_thr, merge_metric)           # merge the overlapping tiles (SAHI's default metric: IOS)
    return boxes[keep], scores[keep], cls[keep], len(tiles), t_model


def overlay(img, boxes, scores, gts, path, width=2000):
    s = width / float(img.shape[1])
    small = cv2.resize(img, (width, int(img.shape[0] * s)))
    for g in gts:
        cv2.rectangle(small, (int(g[0] * s), int(g[1] * s)), (int(g[2] * s), int(g[3] * s)), (255, 120, 0), 1)       # ground truth: thin blue
    for b, sc in zip(boxes, scores):
        cv2.rectangle(small, (int(b[0] * s), int(b[1] * s)), (int(b[2] * s), int(b[3] * s)), (0, 220, 0), 2)         # detections: green
        cv2.putText(small, "%.2f" % sc, (int(b[0] * s), max(12, int(b[1] * s) - 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 0), 1)
    cv2.imwrite(path, small, [cv2.IMWRITE_JPEG_QUALITY, 85])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--model", required=True)
    ap.add_argument("--source", required=True, help="a photo or a folder of photos")
    ap.add_argument("--out", default="detect_images_out")
    ap.add_argument("--tile", type=int, default=512, help="tile size in photo pixels (0: the whole photo as one image)")
    ap.add_argument("--overlap", type=float, default=0.2)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.5, help="NMS IoU within a tile")
    ap.add_argument("--merge-metric", default="ios", choices=["ios", "iou"], help="how overlapping boxes of different tiles are merged (SAHI: ios)")
    ap.add_argument("--merge-thr", type=float, default=0.5)
    ap.add_argument("--format", default="auto")
    ap.add_argument("--names", default="", help="comma separated class names (default: from the model metadata)")
    ap.add_argument("--coco", default="", help="COCO annotations json: score the detections against its boxes")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--every", type=int, default=1, help="take every Nth photo of the folder")
    ap.add_argument("--threads", type=int, default=16)
    a = ap.parse_args()

    opts = ort.SessionOptions()
    opts.intra_op_num_threads = a.threads
    session = ort.InferenceSession(a.model, opts, providers=["CPUExecutionProvider"])
    inp = session.get_inputs()[0]
    size = int(inp.shape[2]) if isinstance(inp.shape[2], int) else 640
    names = load_names(session, a.names)
    nc = len(names) or None
    print("model %s | input %d | classes %s | output shapes %s" % (os.path.basename(a.model), size, names, [o.shape for o in session.get_outputs()]))

    photos = [a.source] if os.path.isfile(a.source) else sorted(f for ext in ("jpg", "jpeg", "png") for f in glob.glob(os.path.join(a.source, "*." + ext)))
    photos = photos[::max(1, a.every)]
    if a.limit:
        photos = photos[:a.limit]
    gt_by_name = {}
    if a.coco:
        d = json.load(open(a.coco))
        ids = {im["id"]: os.path.basename(im["file_name"]) for im in d["images"]}
        for an in d["annotations"]:
            x, y, w, h = an["bbox"]
            gt_by_name.setdefault(ids[an["image_id"]], []).append([x, y, x + w, y + h])
    os.makedirs(a.out, exist_ok=True)
    TP = FP = FN = 0
    t_all = time.time()
    for path in photos:
        img = cv2.imread(path)
        t0 = time.time()
        boxes, scores, cls, n_tiles, t_model = detect_photo(session, inp.name, size, img, a.tile, a.overlap, a.conf, a.iou, a.format, nc, a.merge_metric, a.merge_thr)
        base = os.path.basename(path)
        gts = gt_by_name.get(base, [])
        line = "%s %dx%d | %d tiles | %d detections | %.1f s (model %.1f s, %.2f s/tile)" % (base, img.shape[1], img.shape[0], n_tiles, len(boxes), time.time() - t0, t_model, t_model / n_tiles)
        if a.coco:
            tp, fp, fn = ol.match_detections(list(zip(boxes.tolist(), scores.tolist())), gts, 0.5)
            TP, FP, FN = TP + tp, FP + fp, FN + fn
            line += " | labels %d: TP %d FP %d FN %d (precision %.2f recall %.2f)" % (len(gts), tp, fp, fn, tp / max(1, tp + fp), tp / max(1, tp + fn))
        print(line, flush=True)
        overlay(img, boxes, scores, gts, os.path.join(a.out, os.path.splitext(base)[0] + "_det.jpg"))
        json.dump({"photo": base, "tile": a.tile, "overlap": a.overlap, "conf": a.conf,
                   "detections": [{"box": [round(float(v), 1) for v in b], "score": round(float(s), 3), "class": names[c] if c < len(names) else int(c)} for b, s, c in zip(boxes, scores, cls)]},
                  open(os.path.join(a.out, os.path.splitext(base)[0] + "_det.json"), "w"))
    if a.coco:
        print("TOTAL over %d photos: TP %d FP %d FN %d | precision %.3f recall %.3f | %.0f s" % (len(photos), TP, FP, FN, TP / max(1, TP + FP), TP / max(1, TP + FN), time.time() - t_all))
    return 0


if __name__ == "__main__":
    sys.exit(main())
