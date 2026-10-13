#!/usr/bin/env python3
"""Fine-tunes a small YOLO segmentation model on the crop dataset of make_tile_dataset.py (NO ROS). Needs a CUDA torch + ultralytics: run it with an
environment that has them (on this machine ~/.venv: torch 2.13+cu130 with sm_120, ultralytics 8.4.95), not the container's CPU venv.

    nice -n 10 ~/.venv/bin/python scripts/train_small_yolo.py --data yolo/vignax_tiles/data.yaml --weights yolo26s-seg.pt --name vignax_s
Exports ONNX next to the best weights when --export is given (the detector wants the NMS-free layout: ultralytics yolo26 default).
"""
import argparse


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--data", required=True)
    ap.add_argument("--weights", default="yolo26s-seg.pt")
    ap.add_argument("--project", default="runs_arm")
    ap.add_argument("--name", default="vignax_s")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--patience", type=int, default=25)
    ap.add_argument("--scale", type=float, default=0.5, help="ultralytics scale augmentation gain (+- fraction)")
    ap.add_argument("--export", action="store_true")
    a = ap.parse_args()
    from ultralytics import YOLO
    model = YOLO(a.weights)
    model.train(data=a.data, epochs=a.epochs, imgsz=a.imgsz, batch=a.batch, device=0, workers=a.workers, patience=a.patience, project=a.project, name=a.name,
                exist_ok=True, plots=False, scale=a.scale, flipud=0.5, fliplr=0.5, degrees=15.0, hsv_h=0.02, hsv_s=0.5, hsv_v=0.4, mosaic=1.0, close_mosaic=10)
    best = YOLO("%s/%s/weights/best.pt" % (a.project, a.name))
    print("val:", best.val(data=a.data, imgsz=a.imgsz, device=0, plots=False).results_dict)
    if a.export:
        print("onnx:", best.export(format="onnx", imgsz=a.imgsz, opset=17))


if __name__ == "__main__":
    main()
