# YOLO object detector (arm_perception)

`scripts/object_detector_node.py`: ONNX Runtime on the CPU, a 3D position from ONE camera (no depth) by intersecting the box-centre ray with
the floor plane, TF per detection, and a `grasp_target` frame for the grasp sequence. Read-only: it never commands the arm or the gripper.

## Setup (inside the `claude` container, once)
The system python is 3.8 with numpy 1.17; onnxruntime needs numpy >= 1.21, so it is installed apart and put first on the path by the node:
```
docker exec -u root claude bash -c 'apt-get install -y python3-pip && mkdir -p /home/dev/pylibs_yolo && \
  python3 -m pip install --target /home/dev/pylibs_yolo "onnxruntime<1.20" "onnx<1.17"'
```
(`/home/dev/pylibs_yolo` is `claude_ros_ws/pylibs_yolo` on the host, untracked; the Dockerfile lists no ROS or python packages, so redo this
after recreating the container. `YOLO_PYLIBS` overrides the folder.) There is no GPU path: the container's python 3.8 cannot run a PyTorch that
supports this RTX 5070; CPU inference of a YOLO nano/small is tens of ms, plenty for a still arm.

## The model
Export your trained weights to ONNX (opset 12 to 17, 640 input; layouts v8/v11, v5 and NMS-free end-to-end are recognised) and put the file
somewhere under the workspace, e.g. `/home/dev/models/`. Class names are read from the ONNX metadata (ultralytics stores them) or from `names`
in `config/objects.yaml`. The folder `claude_ros_ws/yolo` is a COPY of the operator's `dev/yolo` (cow, vigna and PRISMA datasets and weights; 3.7 GB, untracked).
The newest model is `yolo/VignaX-3/weights/best.pt`: a SEGMENTATION model (`yolo26x-seg`, the largest size, 140 MB, 640 px, 300 epochs), one class
`Vigna` (the legume genus, not grapevine); validation box mAP50 0.79 and mask mAP50 0.79, precision about 0.89 and recall about 0.73 (608 matched,
76 false positives, 221 missed). A YOLO26 export is NMS-free: the node reads layout `e2e` (box, score, class; the 32 mask coefficients of a -seg head
are not read yet, only the box is used). Exporting needs an environment with a recent ultralytics (the operator's `dev/yolo/docker` image
`ultralytics/ultralytics:latest`); the host conda env `ultralytics` is empty.
`test/make_stub_onnx.py` writes a stub model that always "detects" one object at the image centre, to check the plumbing without a real model.

## Run
```
roslaunch arm_perception object_detector.launch model:=/home/dev/models/<model>.onnx
rosrun rqt_image_view rqt_image_view /object_detector/image
rostopic echo /object_detector/detections
rostopic pub -1 /object_detector/select_class std_msgs/String "data: <class>"      # publish that class as the frame grasp_target
rosservice call /grasp_sequencer/grasp "{target_frame: grasp_target, dry_run: true}"
```
Needs the camera (`arm_camera camera.launch` and its static mount TF) and, for the floor, `marker_localizer` having seen the ground marker (ArUco id 0)
at least once in the last `floor_max_age_s` (600 s): its height is remembered, the floor does not move; otherwise the `floor_z` parameter. Detections need the camera pose at the IMAGE stamp: while the arm moves, or at a pose where the camera ray does not reach the
plane (a horizontal camera at the rest pose), a detection is `valid: false` and gets no TF.

## 3D position and its limits
The position is the bounding-box centre on the plane `z = floor_z + grasp_heights[class]` in `base_link`: exact for a point lying in that plane,
an approximation for a compact object, wrong for one whose box centre is not above its grasp point. `grasp_heights` per class and the true
`floor_z` are UNKNOWN until measured (`config/objects.yaml`); marker_0's z has 1 to 2 cm of accuracy. The grasp sequence then adds its own
`grasp_height` (2 cm above the target by default): set it to 0 to put the fingertips at the detected point.

## Tests
`test/test_object_logic.py` (17 tests: letterbox, the three output layouts, NMS, pixel to plane geometry round trips).

## Offline test on photos (no ROS, no robot): `scripts/detect_images.py`
```
docker exec claude bash -c 'cd /home/dev/src/arm_perception && PYTHONPATH=src /home/dev/venv_yolo/bin/python scripts/detect_images.py \
    --model /home/dev/models/vignax3.onnx --source /home/dev/yolo/dataset/New_Identify --tile 512 --out /home/dev/models/test_new'
```
Tiled inference like the operator's SAHI script (tile and overlap in PHOTO pixels, each tile letterboxed to the model input, tiles merged by NMS with
the IOS metric), an overlay and a JSON per photo, and with `--coco ann.json` precision and recall at IoU 0.5. `/home/dev/venv_yolo` (container, python 3.8,
CPU torch 2.4.1, ultralytics 8.4.176, onnxruntime) is where the model was exported:
`venv_yolo/bin/yolo export model=best.pt format=onnx imgsz=640 simplify=True` -> `(1, 37, 8400)` + `(1, 32, 160, 160)` for VignaX-3 (4 box + 1 class + 32 mask
coefficients, NMS outside the model).

### Results (2026-10-13), CPU, 0.4 s per 640 px tile
* July field photo (3968 x 2976, labeled, plants about 250 px, 512 px tiles): 22 of 23 labeled plants found with a plain IoU merge (precision 0.40: several nested
  boxes per plant), 17 of 23 with the IOS merge (precision 0.71, recall 0.74). These photos are probably in the training set.
* `New_Identify` (the operator's lab scene: a tray with two rows of Vigna, a grass row, a marker, tiled floor; 1530 x 2040 and 1724 x 1293 photos, NO labels): the
  model does not fire on the grass, the marker or the tray, but SCALE decides the result. The whole photo at once finds almost nothing; 512 px tiles cut the
  plants (boxes with straight edges at the tile borders: a lab plant is larger than a 512 px tile); 768 and 1024 px tiles give a few large boxes covering
  groups of leaves. Not usable for a grasp point yet: the model was trained on field seedlings of about 250 px, the lab plants are bigger, overhead, on a
  white tray: a domain gap. Next levers: masks instead of boxes, multi-scale inference, a few labeled lab images to fine-tune.

### Masks and multi-scale on the lab photos (2026-10-13): not enough
`detect_images.py --masks --scales 1.0,0.6,0.4` decodes the segmentation masks (area, centroid, major axis per plant; 22 s for a 1530 x 2040 photo, 30 tiles,
6 threads at low priority so the 100 Hz driver loop is not disturbed) and merges the three scales. On the operator's tray photo the masks are soft blobs that spill
onto the white tray, the soil and the floor beyond the leaves, plants are covered by several overlapping boxes of different scales, boxes are still cut at
tile borders, and there is a detection on the grass row at scale 0.6. Neither the masks nor the multi-scale tiling make VignaX-3 usable for a grasp point on the lab
setup: it was trained on field seedlings of about 250 px in 512 px tiles and the lab scene is a different domain (larger overhead plants, tiled floor, white tray).
The remaining lever is fine-tuning on labeled images of the lab setup, ideally taken by the arm camera at its working poses.

## GPU inference (2026-10-13)
`onnx_session.make_session(model, threads, device)` (used by `object_detector_node.py` via `~device`, by `detect_images.py` and `eval_cv_methods.py` via `--device
auto|gpu|cpu`) runs on the CUDA GPU when `/home/dev/pylibs_gpu` exists (onnxruntime-gpu 1.19.2 plus the pip CUDA 12.9 / cuDNN 9 libraries, installed with
`pip install --target /home/dev/pylibs_gpu onnxruntime-gpu==1.19.2 nvidia-cudnn-cu12 nvidia-cublas-cu12 nvidia-cuda-runtime-cu12 nvidia-cufft-cu12
nvidia-curand-cu12 nvidia-cuda-nvrtc-cu12 nvidia-nvjitlink-cu12 --no-deps`; override with `YOLO_GPULIBS`) and falls back to the CPU otherwise; `gpu` fails instead of
falling back. The RTX 5070 (sm_120) has no precompiled kernel in this onnxruntime build, so they are JIT-compiled from PTX: the FIRST run in a fresh driver cache
takes about 40 s, then 35 ms per 640 px tile (CPU: 0.4 s). VignaX-3 at 3.5x scale with 512 px tiles: 117 s -> 7 s per lab image, identical detections.
Not tried: TensorRT / FP16 (the provider is listed but its engine build is slow and was not needed).

## Classical methods vs VignaX-3 on the labeled lab images (2026-10-13)
`scripts/eval_cv_methods.py` scores methods against the Roboflow COCO polygons of Testbed_1 to 5 (pixels against the union of the Vigna polygons; plants by
centroid). Mean over the 5 images:

| method | pixel P | pixel R | IoU | plants recall | plants precision | pixels on Sorgo |
|---|---|---|---|---|---|---|
| ExG (`classical_seg.py`, Otsu) | 0.61 | 0.75 | 0.46 | 0.53 | 0.49 | 19 % |
| HSV | 0.64 | 0.60 | 0.45 | 0.60 | 0.60 | 32 % |
| RandomForest, leave-one-image-out (Lab/HSV/ExG/texture) | 0.84 | 0.80 | 0.68 | 0.43 | 0.87 | 1 % |
| VignaX-3, 3.5x, 512 px tiles | 0.72 | 0.14 | 0.13 | 0.06 | 0.85 | 1 % |

VignaX-3 finds 3 to 12 of 60 to 111 labeled plants per image (scale / domain gap): fine-tune it on labels of the lab setup. The RandomForest segments well
but merges touching plants (instance splitting is the weak point) and failed on the held-out Testbed_5 (IoU 0.18); ExG cannot tell Vigna from Sorgo.
