#!/usr/bin/env python3
"""Saves camera frames on demand for offline tests (read-only: it never commands the arm).

Each call of ~capture writes <out_dir>/<prefix>_<time>_<n>.jpg and a .json sidecar next to it (camera pose in the base frame, camera model, the ground
marker's pose when known, the joint angles; see arm_perception/frame_capture.py). Move the arm or rearrange the plants between calls to get variety.

    rosservice call /frame_capture/capture            # one frame
    rosservice call /frame_capture/capture_burst      # ~burst_count frames, ~burst_period_s apart

Services: ~capture (std_srvs/Trigger): one frame now. ~capture_burst (std_srvs/Trigger): ~burst_count frames, ~burst_period_s apart (blocks until done).
Parameters: ~out_dir (/home/dev/models/samples), ~prefix (arm), ~image_topic (/usb_cam/image_raw), ~camera_info_topic, ~base_frame (base_link),
~marker0_frame (marker_0), ~jpeg_quality (95), ~max_image_age_s (2.0: a frame older than this is refused), ~note (free text copied into every sidecar).
"""
import json
import os
import threading
import time

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_srvs.srv import Trigger, TriggerResponse

from arm_perception import frame_capture as fc


class Node(object):
    def __init__(self):
        P = rospy.get_param
        self.out_dir = P("~out_dir", "/home/dev/models/samples")
        self.prefix = P("~prefix", "arm")
        self.base = P("~base_frame", "base_link")
        self.marker0 = P("~marker0_frame", "marker_0")
        self.quality = int(P("~jpeg_quality", 95))
        self.max_age = float(P("~max_image_age_s", 2.0))
        self.count, self.period = int(P("~burst_count", 5)), float(P("~burst_period_s", 2.0))
        self.note = P("~note", "")
        os.makedirs(self.out_dir, exist_ok=True)
        self.bridge = CvBridge()
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.img, self.info, self.joints = None, None, {}
        self.index = 0
        self.lock = threading.Lock()
        rospy.Subscriber(P("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(P("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, lambda m: setattr(self, "info", m), queue_size=1)
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        rospy.Service("~capture", Trigger, lambda _: self.service(1, 0.0))
        rospy.Service("~capture_burst", Trigger, lambda _: self.service(self.count, self.period))
        rospy.logwarn("frame_capture ready: ~capture / ~capture_burst (%d frames, %.1f s apart) -> %s", self.count, self.period, self.out_dir)

    def on_image(self, m):
        self.img = m

    def on_joints(self, m):
        self.joints = dict(zip(m.name, m.position))

    def pose(self, frame, stamp):
        try:
            t = self.buf.lookup_transform(self.base, frame, stamp, rospy.Duration(0.2)).transform
        except tf2_ros.TransformException:
            try:
                t = self.buf.lookup_transform(self.base, frame, rospy.Time(0), rospy.Duration(0.2)).transform
            except tf2_ros.TransformException:
                return None
        return fc.pose_record((t.translation.x, t.translation.y, t.translation.z), (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def save_one(self):
        """Saves the latest frame; returns the path or raises RuntimeError with the reason."""
        m = self.img
        if m is None or self.info is None:
            raise RuntimeError("no image or no camera_info yet")
        age = (rospy.Time.now() - m.header.stamp).to_sec()
        if abs(age) > self.max_age:
            raise RuntimeError("the latest image is %.1f s old" % age)
        stamp = m.header.stamp.to_sec()
        stem = os.path.join(self.out_dir, fc.frame_name(self.prefix, stamp, self.index))
        self.index += 1
        img = self.bridge.imgmsg_to_cv2(m, "bgr8")
        cv2.imwrite(stem + ".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        rec = fc.sidecar(stamp, m.header.frame_id, m.width, m.height, self.info.K, self.info.D, self.pose(m.header.frame_id, m.header.stamp),
                         self.pose(self.marker0, m.header.stamp), self.joints, self.note)
        with open(stem + ".json", "w") as f:
            json.dump(rec, f, indent=1)
        return stem + ".jpg"

    def service(self, count, period):
        if not self.lock.acquire(False):
            return TriggerResponse(False, "refused: a capture is already running")
        try:
            saved = []
            for i in range(count):
                if i:
                    time.sleep(period)
                saved.append(os.path.basename(self.save_one()))
            return TriggerResponse(True, "saved %d: %s" % (len(saved), ", ".join(saved)))
        except RuntimeError as e:
            return TriggerResponse(False, "refused: %s" % e)
        finally:
            self.lock.release()


def main():
    rospy.init_node("frame_capture")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
