#!/usr/bin/env python3
"""Compares plant detection methods on COCO-labeled photos (NO ROS): classical colour thresholds, a per-pixel classifier tested leave-one-image-out,
and the YOLO segmentation model. Scores the Vigna class at pixel level (precision, recall, F1, IoU against the union of the Vigna polygons, plus the share of
the predicted pixels falling on the Sorgo rows) and at plant level (a plant is covered when a predicted centroid lies inside it; a prediction is right when its
centroid lies inside some labeled plant).

    PYTHONPATH=src python3 scripts/eval_cv_methods.py --coco-dir <folder with _annotations.coco.json and the images> --out out_dir \\
        --methods exg,hsv,rf[,yolo --model vignax3.onnx --yolo-scale 3.5]
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

from arm_perception import classical_seg as cs


def load_gt(root, vigna="Vigna", other="Sorgo_Row"):
    d = json.load(open(os.path.join(root, "_annotations.coco.json")))
    cats = {c["id"]: c["name"] for c in d["categories"]}
    items = []
    for im in d["images"]:
        h, w = im["height"], im["width"]
        lab = np.zeros((h, w), np.int32)
        sorgo = np.zeros((h, w), np.uint8)
        n = 0
        for a in d["annotations"]:
            if a["image_id"] != im["id"]:
                continue
            polys = [np.array(p, np.float32).reshape(-1, 2).round().astype(np.int32) for p in a["segmentation"]]
            if cats[a["category_id"]] == vigna:
                n += 1
                cv2.fillPoly(lab, polys, n)
            elif cats[a["category_id"]] == other:
                cv2.fillPoly(sorgo, polys, 1)
        items.append({"name": im["file_name"], "path": os.path.join(root, im["file_name"]), "lab": lab, "sorgo": sorgo.astype(bool)})
    return items


def post(mask, radius, min_area):
    return cs.remove_small(cs.remove_thin(mask, radius) if radius > 0 else mask, min_area)


def method_exg(img, a):
    return post(cs.green_mask_exg(img), a.open_radius, a.min_area)


def method_hsv(img, a):
    return post(cs.green_mask_hsv(img), a.open_radius, a.min_area)


def pixel_features(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB).astype(np.float32)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    e = cs.exg(img)
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gx, gy = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3), cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    feats = [lab[:, :, 0], lab[:, :, 1], lab[:, :, 2], hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2], e, np.hypot(gx, gy)]
    for k in (9, 25):
        mean = cv2.blur(e, (k, k))
        feats += [mean, np.sqrt(np.maximum(cv2.blur(e * e, (k, k)) - mean * mean, 0.0))]
        feats.append(cv2.blur(np.hypot(gx, gy), (k, k)))
    return np.stack(feats, axis=-1)


def train_rf(items_train, per_class=40000, seed=0):
    from sklearn.ensemble import RandomForestClassifier
    rng = np.random.RandomState(seed)
    X, y = [], []
    for it in items_train:
        img = cv2.imread(it["path"])
        F = pixel_features(img)
        cls_map = np.zeros(it["lab"].shape, np.int8)                    # 0 background, 1 vigna, 2 sorgo
        cls_map[it["sorgo"]] = 2
        cls_map[it["lab"] > 0] = 1
        for c in (0, 1, 2):
            idx = np.flatnonzero(cls_map.ravel() == c)
            if len(idx) == 0:
                continue
            idx = rng.choice(idx, min(per_class, len(idx)), replace=False)
            X.append(F.reshape(-1, F.shape[-1])[idx])
            y.append(np.full(len(idx), c))
    clf = RandomForestClassifier(n_estimators=60, max_depth=14, min_samples_leaf=5, n_jobs=6, random_state=seed)
    clf.fit(np.concatenate(X), np.concatenate(y))
    return clf


def method_rf(img, clf, a):
    F = pixel_features(img)
    flat = F.reshape(-1, F.shape[-1])
    pred = np.empty(len(flat), np.int8)
    for i in range(0, len(flat), 400000):
        pred[i:i + 400000] = clf.predict(flat[i:i + 400000])
    return post(pred.reshape(img.shape[:2]) == 1, max(0, a.open_radius // 3), a.min_area)


def method_yolo(img, a, session_cache):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import detect_images as di
    from arm_perception.onnx_session import make_session
    if "s" not in session_cache:
        session_cache["s"], providers = make_session(a.model, a.threads, a.device)
        print("onnxruntime providers:", providers)
    s = session_cache["s"]
    inp = s.get_inputs()[0]
    dets, n_tiles, _ = di.detect_photo_seg(s, inp.name, int(inp.shape[2]), img, 512, 0.2, a.yolo_conf, 0.5, 1, [a.yolo_scale], "ios", 0.5)
    mask = np.zeros(img.shape[:2], bool)
    cents = []
    for d in dets:
        sc = d["scale"]
        m = cv2.resize(d["mask"].astype(np.uint8), (max(1, int(round(d["mask"].shape[1] / sc))), max(1, int(round(d["mask"].shape[0] / sc)))), interpolation=cv2.INTER_NEAREST).astype(bool)
        ox, oy = int(round(d["origin"][0] / sc)), int(round(d["origin"][1] / sc))
        y2, x2 = min(mask.shape[0], oy + m.shape[0]), min(mask.shape[1], ox + m.shape[1])
        if y2 > oy and x2 > ox:
            mask[oy:y2, ox:x2] |= m[:y2 - oy, :x2 - ox]
        st = di.det_stats(d)
        if st:
            cents.append(st["centroid"])
    return mask, cents, n_tiles


def score(mask, cents, it):
    gt = it["lab"] > 0
    r = cs.pixel_metrics(mask, gt)
    r["on_sorgo"] = float(np.logical_and(mask, it["sorgo"]).sum()) / max(1.0, float(mask.sum()))
    r.update({"plants_" + k: v for k, v in cs.instance_hits_labels(cents, it["lab"]).items()})
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--coco-dir", required=True)
    ap.add_argument("--out", default="cv_eval")
    ap.add_argument("--methods", default="exg,hsv,rf")
    ap.add_argument("--open-radius", type=int, default=7, help="opening disc radius (px): removes grass blades thinner than 2r+1")
    ap.add_argument("--min-area", type=int, default=400)
    ap.add_argument("--model", default="")
    ap.add_argument("--yolo-scale", type=float, default=3.5)
    ap.add_argument("--yolo-conf", type=float, default=0.25)
    ap.add_argument("--threads", type=int, default=6)
    ap.add_argument("--device", default="auto", choices=["auto", "gpu", "cpu"], help="ONNX device for the yolo method")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    items = load_gt(a.coco_dir)
    print("%d labeled images, plants per image: %s" % (len(items), [int(it["lab"].max()) for it in items]))
    methods = a.methods.split(",")
    results, cache = {m: [] for m in methods}, {}
    for k, it in enumerate(items):
        img = cv2.imread(it["path"])
        for m in methods:
            t0 = time.time()
            if m == "rf":
                clf = train_rf([o for j, o in enumerate(items) if j != k])               # leave-one-image-out
                mask = method_rf(img, clf, a)
                inst = cs.split_instances(mask, 0.75, a.min_area // 2)
                cents = [i["centroid"] for i in inst]
            elif m == "yolo":
                mask, cents, _ = method_yolo(img, a, cache)
            else:
                mask = method_exg(img, a) if m == "exg" else method_hsv(img, a)
                cents = [i["centroid"] for i in cs.split_instances(mask, 0.75, a.min_area // 2)]
            r = score(mask, cents, it)
            r.update({"image": it["name"][:10], "seconds": round(time.time() - t0, 1)})
            results[m].append(r)
            vis = img.copy()
            vis[mask] = (0.4 * vis[mask] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
            for (cx, cy) in cents:
                cv2.circle(vis, (int(cx), int(cy)), 6, (255, 255, 0), -1)
            cv2.imwrite(os.path.join(a.out, "%s_%d.jpg" % (m, k + 1)), cv2.resize(vis, (800, int(vis.shape[0] * 800 / vis.shape[1]))), [cv2.IMWRITE_JPEG_QUALITY, 85])
            print("%-5s %s | pixels P %.2f R %.2f IoU %.2f (on sorgo %.2f) | plants: recall %.2f precision %.2f (%d predicted, %d labeled) | %.1f s" % (
                m, r["image"], r["precision"], r["recall"], r["iou"], r["on_sorgo"], r["plants_recall"], r["plants_precision"], r["plants_n_pred"], r["plants_n_gt"], r["seconds"]), flush=True)
    print("\nMEAN over the images:")
    for m in methods:
        rs = results[m]
        mean = lambda key: float(np.mean([r[key] for r in rs]))
        print("%-5s pixels P %.2f R %.2f F1 %.2f IoU %.2f | on sorgo %.2f | plants recall %.2f precision %.2f" % (m, mean("precision"), mean("recall"), mean("f1"), mean("iou"), mean("on_sorgo"), mean("plants_recall"), mean("plants_precision")))
    json.dump(results, open(os.path.join(a.out, "results_%s.json" % "_".join(methods)), "w"), indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
