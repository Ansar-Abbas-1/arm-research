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
in `config/objects.yaml`. The folder `claude_ros_ws/yolo` is a COPY of the operator's `dev/yolo` (cow, vine and PRISMA datasets and weights;
3.7 GB, untracked): none of those weights is a detector for the arm's objects.
`test/make_stub_onnx.py` writes a stub model that always "detects" one object at the image centre, to check the plumbing without a real model.

## Run
```
roslaunch arm_perception object_detector.launch model:=/home/dev/models/<model>.onnx
rosrun rqt_image_view rqt_image_view /object_detector/image
rostopic echo /object_detector/detections
rostopic pub -1 /object_detector/select_class std_msgs/String "data: <class>"      # publish that class as the frame grasp_target
rosservice call /grasp_sequencer/grasp "{target_frame: grasp_target, dry_run: true}"
```
Needs the camera (`arm_camera camera.launch` and its static mount TF) and, for the floor, `marker_localizer` seeing marker_0 in the last 60 s
(otherwise `floor_z`). Detections need the camera pose at the IMAGE stamp: while the arm moves, or at a pose where the camera ray does not reach the
plane (a horizontal camera at the rest pose), a detection is `valid: false` and gets no TF.

## 3D position and its limits
The position is the bounding-box centre on the plane `z = floor_z + grasp_heights[class]` in `base_link`: exact for a point lying in that plane,
an approximation for a compact object, wrong for one whose box centre is not above its grasp point. `grasp_heights` per class and the true
`floor_z` are UNKNOWN until measured (`config/objects.yaml`); marker_0's z has 1 to 2 cm of accuracy. The grasp sequence then adds its own
`grasp_height` (2 cm above the target by default): set it to 0 to put the fingertips at the detected point.

## Tests
`test/test_object_logic.py` (17 tests: letterbox, the three output layouts, NMS, pixel to plane geometry round trips).
