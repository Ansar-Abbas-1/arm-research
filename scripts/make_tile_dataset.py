#!/usr/bin/env python3
"""Builds a multi-scale crop dataset (YOLO segmentation format) from a few large Roboflow-labeled photos (NO ROS).

Each photo is resized by every scale in --scales, covered with --tile px crops every --stride-frac * tile, and the polygons are clipped to each crop
(arm_perception/dataset_tiles.py). Photos whose file name contains one of the --val strings go to the validation split, the rest to train, so the
validation crops never come from a photo the model trained on. Crops without any label are kept with probability --neg-frac (background examples).

    PYTHONPATH=src python3 scripts/make_tile_dataset.py --src yolo/Sorgo-Vigna.v3i.yolo26/train --out yolo/vignax_tiles \\
        --val Testbed_5,08-3- --scales 0.7,1.0,1.5,2.0,2.5
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np

from arm_perception import dataset_tiles as dt


def read_polys(label_path, w, h):
    """[(class_id, (N, 2) polygon in photo pixels)] from a YOLO segmentation label file."""
    polys = []
    if not os.path.isfile(label_path):
        return polys
    for line in open(label_path):
        v = line.split()
        if len(v) < 7:
            continue
        polys.append((int(v[0]), np.array(v[1:], float).reshape(-1, 2) * [w, h]))
    return polys


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--src", required=True, help="Roboflow YOLO export split folder holding images/ and labels/")
    ap.add_argument("--out", required=True)
    ap.add_argument("--val", default="", help="comma separated strings: photos whose name contains one are the validation split")
    ap.add_argument("--scales", default="0.7,1.0,1.5,2.0,2.5")
    ap.add_argument("--tile", type=int, default=640)
    ap.add_argument("--stride-frac", type=float, default=0.5)
    ap.add_argument("--min-ratio", type=float, default=0.25)
    ap.add_argument("--neg-frac", type=float, default=0.1)
    ap.add_argument("--names", default="Sorgo_Row,Vigna", help="class names in id order (as in the export's data.yaml)")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    val_keys = [s for s in a.val.split(",") if s]
    scales = [float(s) for s in a.scales.split(",")]
    for sp in ("train", "val"):
        os.makedirs(os.path.join(a.out, sp, "images"), exist_ok=True)
        os.makedirs(os.path.join(a.out, sp, "labels"), exist_ok=True)
    counts = {"train": [0, 0], "val": [0, 0]}                     # crops, polygons
    for path in sorted(glob.glob(os.path.join(a.src, "images", "*"))):
        stem = os.path.splitext(os.path.basename(path))[0]
        split = "val" if any(k in stem for k in val_keys) else "train"
        img = cv2.imread(path)
        h, w = img.shape[:2]
        polys = read_polys(os.path.join(a.src, "labels", stem + ".txt"), w, h)
        short = stem[:24].replace(".", "_")
        for s in scales:
            im = cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA if s < 1.0 else cv2.INTER_CUBIC)
            H, W = im.shape[:2]
            for y0 in dt.grid_origins(H, a.tile, int(a.tile * a.stride_frac)):
                for x0 in dt.grid_origins(W, a.tile, int(a.tile * a.stride_frac)):
                    labels = dt.tile_labels(polys, s, x0, y0, a.tile, a.min_ratio)
                    if not labels and rng.random() > a.neg_frac:
                        continue
                    crop = im[y0:y0 + a.tile, x0:x0 + a.tile]
                    if crop.shape[0] < a.tile or crop.shape[1] < a.tile:          # a photo smaller than a tile: pad
                        crop = cv2.copyMakeBorder(crop, 0, a.tile - crop.shape[0], 0, a.tile - crop.shape[1], cv2.BORDER_CONSTANT, value=(114, 114, 114))
                    name = "%s_s%.1f_%d_%d" % (short, s, x0, y0)
                    cv2.imwrite(os.path.join(a.out, split, "images", name + ".jpg"), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    with open(os.path.join(a.out, split, "labels", name + ".txt"), "w") as f:
                        f.write("".join(dt.to_yolo_seg_line(c, p, a.tile) + "\n" for c, p in labels))
                    counts[split][0] += 1
                    counts[split][1] += len(labels)
    names = a.names.split(",")
    with open(os.path.join(a.out, "data.yaml"), "w") as f:
        f.write("path: %s\ntrain: train/images\nval: val/images\nnc: %d\nnames: [%s]\n" % (os.path.abspath(a.out), len(names), ", ".join(names)))
    print("train: %d crops, %d polygons | val: %d crops, %d polygons" % (counts["train"][0], counts["train"][1], counts["val"][0], counts["val"][1]))


if __name__ == "__main__":
    main()
