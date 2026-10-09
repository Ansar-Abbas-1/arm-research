#!/usr/bin/env python3
"""YOLO object detector with a 3D position from ONE camera: ONNX Runtime on the CPU, floor-plane geometry, TF per detection and an
optional grasp_target for the grasp sequence. READ-ONLY: it subscribes to the image, camera_info and TF and publishes; it never commands
the arm or the gripper.

Per processed frame (max_rate_hz): letterbox -> ONNX -> decode -> NMS -> boxes in image pixels (arm_perception/object_logic.py); the
camera pose comes from TF AT THE IMAGE STAMP (no pose, no detection: the arm is moving or TF is missing); each box centre ray is
intersected with the plane z = floor_z + grasp_height[class] in the base frame. The floor is marker_0's z when that marker was seen
recently (floor_from_marker0), else the floor_z parameter. The position is exact for a point lying in that plane: set each class's
grasp_height to the height of the point to grasp (config/objects.yaml: UNKNOWN until you measure them).

Publishes (private names):
  ~detections  arm_perception/ObjectDetectionArray     every detection of the frame, best first
  ~image       sensor_msgs/Image                        the annotated frame (only while someone subscribes), 960 px wide
  ~status      std_msgs/String (JSON, latched)          model, rate, inference time, floor, target
  TF           <base_frame> -> object_<class>_<rank>    valid detections, at the IMAGE stamp
  TF           <base_frame> -> <target_frame>           only while grasp_class is set: the best valid detection of that class, +z up, the
               DETECTION's stamp re-broadcast at 2 Hz (it ages like the observation, so the grasp sequence's target_max_age check stays honest)
Subscribes ~select_class (std_msgs/String): the class to publish as the grasp target ("" = none; parameter grasp_class sets the start).

Parameters: see config/objects.yaml. Runtime: onnxruntime comes from /home/dev/pylibs_yolo (env YOLO_PYLIBS), a numpy-compatible install kept outside the system
python and put first on the path before numpy is imported (see docs/yolo_detector.md).
"""
import ast
import json
import os
import sys
import threading
import time

# onnxruntime needs numpy >= 1.21 but the system python has 1.17: its own numpy lives in this folder and must come FIRST, before anything
# imports numpy (otherwise onnxruntime loads against the old one and warns about the API version). YOLO_PYLIBS overrides the folder.
_PYLIBS = os.environ.get("YOLO_PYLIBS", "/home/dev/pylibs_yolo")
if os.path.isdir(_PYLIBS) and _PYLIBS not in sys.path:
    sys.path.insert(0, _PYLIBS)

import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from arm_perception import object_logic as ol
from arm_perception.msg import ObjectDetection, ObjectDetectionArray
from arm_camera.geometry import make_transform, matrix_from_quat


def sanitize(name):
    return "".join(c if c.isalnum() else "_" for c in name)


class Node(object):
    def __init__(self):
        P = rospy.get_param
        import onnxruntime as ort
        model = P("~model", "")
        if not model or not os.path.isfile(model):
            rospy.logfatal("object_detector: model file not found: %r (set ~model, an .onnx exported from your YOLO weights)", model)
            sys.exit(1)
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = int(P("~threads", 8))
        self.session = ort.InferenceSession(model, opts, providers=["CPUExecutionProvider"])
        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        shape = inp.shape
        self.size = int(shape[2]) if isinstance(shape[2], int) else int(P("~input_size", 640))
        names = P("~names", [])
        if not names:
            meta = self.session.get_modelmeta().custom_metadata_map.get("names")
            if meta:
                d = ast.literal_eval(meta)
                names = [d[k] for k in sorted(d)]
        self.names = list(names)
        self.fmt = P("~format", "auto")
        self.conf, self.iou = float(P("~conf", 0.35)), float(P("~iou", 0.45))
        self.max_rate = float(P("~max_rate_hz", 2.0))
        self.base = P("~base_frame", "base_link")
        self.prefix = P("~tf_prefix", "object_")
        self.floor_z_param = float(P("~floor_z", -0.60))
        self.floor_from_marker0 = bool(P("~floor_from_marker0", True))
        self.marker0 = P("~marker0_frame", "marker_0")
        self.heights = {str(k): float(v) for k, v in P("~grasp_heights", {}).items()}
        self.default_height = float(P("~default_grasp_height", 0.03))
        self.max_range = float(P("~max_range", 1.5))
        self.target_frame = P("~target_frame", "grasp_target")
        self.select(P("~grasp_class", ""))
        self.bridge = CvBridge()
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.br = tf2_ros.TransformBroadcaster()
        self.K = self.D = None
        self.last_t, self.lock = 0.0, threading.Lock()
        self.target = None                                            # (position, stamp, class, confidence)
        self.stats = {"frames": 0, "inference_ms": 0.0, "skipped_no_tf": 0, "last_error": ""}
        self.pub = rospy.Publisher("~detections", ObjectDetectionArray, queue_size=1)
        self.pub_img = rospy.Publisher("~image", Image, queue_size=1)
        self.pub_status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        rospy.Subscriber(P("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber("~select_class", String, lambda m: self.select(m.data), queue_size=1)
        rospy.Subscriber(P("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(0.5), self.on_timer)
        rospy.logwarn("object_detector ready: %s, input %d, %d classes %s, format %s, conf %.2f, floor z %.3f (%s)", os.path.basename(model), self.size,
                      len(self.names), self.names[:8], self.fmt, self.conf, self.floor_z_param, "marker_0 when seen" if self.floor_from_marker0 else "parameter")

    # ---- inputs ---------------------------------------------------------------------------------------------
    def select(self, cls):
        self.grasp_class = str(cls).strip()
        self.target = None
        rospy.logwarn("object_detector: grasp target class %r%s", self.grasp_class, "" if self.grasp_class else " (no grasp_target published)")

    def on_info(self, m):
        self.K = np.array(m.K, float).reshape(3, 3)
        self.D = np.array(m.D, float) if len(m.D) else None

    def floor(self):
        if self.floor_from_marker0:
            try:
                t = self.buf.lookup_transform(self.base, self.marker0, rospy.Time(0), rospy.Duration(0.0))
                if (rospy.Time.now() - t.header.stamp).to_sec() < 60.0:
                    return t.transform.translation.z, "marker_0"
            except tf2_ros.TransformException:
                pass
        return self.floor_z_param, "param"

    def on_image(self, msg):
        now = time.time()
        if now - self.last_t < 1.0 / self.max_rate or self.K is None or not self.lock.acquire(False):
            return
        try:
            self.last_t = now
            self.process(msg)
        except Exception as e:  # noqa: BLE001  (a bad frame must not kill the node)
            self.stats["last_error"] = str(e)
            rospy.logerr_throttle(5.0, "object_detector: %s", e)
        finally:
            self.lock.release()

    # ---- one frame ---------------------------------------------------------------------------------------------
    def process(self, msg):
        try:
            tf = self.buf.lookup_transform(self.base, msg.header.frame_id, msg.header.stamp, rospy.Duration(0.1))
        except tf2_ros.TransformException:
            self.stats["skipped_no_tf"] += 1
            return
        q, tr = tf.transform.rotation, tf.transform.translation
        T = make_transform(matrix_from_quat([q.x, q.y, q.z, q.w]), [tr.x, tr.y, tr.z])
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        t0 = time.time()
        blob, scale, pad = ol.letterbox(img, self.size)
        out = self.session.run(None, {self.input_name: blob})[0]
        boxes, scores, cls = ol.decode(out, self.fmt, self.conf, len(self.names) or None)
        keep = ol.nms(boxes, scores, cls, self.iou)
        boxes = ol.unletterbox(boxes[keep], scale, pad, img.shape[:2])
        scores, cls = scores[keep], cls[keep]
        ms = 1000.0 * (time.time() - t0)
        floor_z, source = self.floor()
        arr = ObjectDetectionArray()
        arr.header, arr.inference_ms, arr.floor_z, arr.floor_source = msg.header, ms, floor_z, source
        ranks, transforms = {}, []
        for b, s, c in zip(boxes, scores, cls):
            name = self.names[c] if c < len(self.names) else "class%d" % c
            z_plane = floor_z + self.heights.get(name, self.default_height)
            pos = ol.object_position(b, T, self.K, self.D, z_plane, self.max_range)
            d = ObjectDetection(class_name=name, class_id=int(c), confidence=float(s), box=[float(v) for v in b], valid=pos is not None, plane_z=float(z_plane))
            if pos is not None:
                k = ranks.get(name, 0)
                ranks[name] = k + 1
                d.position, d.frame = Point(*[float(v) for v in pos]), "%s%s_%d" % (self.prefix, sanitize(name), k)
                m = TransformStamped()
                m.header.stamp, m.header.frame_id, m.child_frame_id = msg.header.stamp, self.base, d.frame
                m.transform.translation = Point(*[float(v) for v in pos])
                m.transform.rotation.w = 1.0
                transforms.append(m)
            arr.detections.append(d)
        wanted = [d for d in arr.detections if d.valid and d.class_name == self.grasp_class]
        if wanted:                                                    # the grasp target: the best valid detection of the chosen class in this frame
            b = max(wanted, key=lambda d: d.confidence)
            self.target = (np.array([b.position.x, b.position.y, b.position.z]), msg.header.stamp, b.class_name, float(b.confidence))
        if transforms:
            self.br.sendTransform(transforms)
        self.pub.publish(arr)
        self.stats["frames"] += 1
        self.stats["inference_ms"] = ms
        if self.pub_img.get_num_connections():
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(self.annotate(img, arr), "bgr8"))

    def annotate(self, img, arr):
        import cv2
        for d in arr.detections:
            x1, y1, x2, y2 = [int(v) for v in d.box]
            color = (0, 200, 0) if d.valid else (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 4)
            label = "%s %.2f" % (d.class_name, d.confidence) + ("  (%.3f %.3f %.3f)" % (d.position.x, d.position.y, d.position.z) if d.valid else "  no floor hit")
            cv2.putText(img, label, (x1, max(30, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        scale = 960.0 / img.shape[1]
        return cv2.resize(img, (960, int(img.shape[0] * scale)))

    # ---- grasp target + status ------------------------------------------------------------------------------------
    def on_timer(self, _):
        if self.grasp_class and self.target is not None:
            pos, stamp, name, conf = self.target
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = stamp, self.base, self.target_frame      # the detection's stamp, not now
            m.transform.translation = Point(*[float(v) for v in pos])
            m.transform.rotation.w = 1.0
            self.br.sendTransform(m)
        age = None if self.target is None else (rospy.Time.now() - self.target[1]).to_sec()
        self.pub_status.publish(String(json.dumps(dict(self.stats, grasp_class=self.grasp_class, target=None if self.target is None else
                                                       {"class": self.target[2], "confidence": self.target[3], "xyz": [float(v) for v in self.target[0]], "age_s": age},
                                                       classes=self.names, floor_z_param=self.floor_z_param))))


def main():
    rospy.init_node("object_detector")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
