#!/usr/bin/env python3
"""Detects ArUco markers in the camera image and publishes their poses.

For every marker seen: a TF  <camera_frame> -> aruco_<id>  (the marker centre; z points out of the
marker toward the viewer), an rviz marker (flat square of the marker size, its axes, a line from
the camera's optical centre to the marker with the distance as text), and an annotated image.

Parameters (private):
  marker_size  side of the BLACK square including its black border, metres (measure the print!)
  dictionary   e.g. 4X4_50, 5X5_100, ARUCO_ORIGINAL, or "auto" (default): tries the dictionaries
               smallest first on the first frames and locks the first that reads a marker
  image_topic, camera_info_topic, tf_prefix ("aruco_"), max_rate_hz (10)
The distance is measured from the camera's OPTICAL CENTRE (the origin of camera_optical_frame),
not from the front glass or the sensor surface; see docs/aruco_distance_test.md.
"""
import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from arm_camera import aruco
from arm_camera import board as board_mod
from arm_camera.geometry import quat_from_matrix


def pt(p):
    return Point(float(p[0]), float(p[1]), float(p[2]))


class ArucoNode:
    def __init__(self):
        self.size = float(rospy.get_param("~marker_size", 0.10))
        if not rospy.has_param("~marker_size"):
            rospy.logwarn("~marker_size not set: using 0.10 m. Measure the black square of YOUR print and pass it, "
                          "the distance scales linearly with it.")
        name = rospy.get_param("~dictionary", "auto")
        self.dictionary = None if name == "auto" else name
        if self.dictionary is not None and self.dictionary not in aruco.DICTIONARIES:
            raise ValueError("unknown dictionary %s; choose from %s or auto" % (self.dictionary, ", ".join(aruco.DICTIONARIES)))
        self.prefix = rospy.get_param("~tf_prefix", "aruco_")
        self.period = 1.0 / float(rospy.get_param("~max_rate_hz", 10.0))
        self.bridge = CvBridge()
        self.br = tf2_ros.TransformBroadcaster()
        self.info = None
        self.last = rospy.Time(0)
        self.pub_img = rospy.Publisher("~image", Image, queue_size=1)
        self.pub_markers = rospy.Publisher("~markers", MarkerArray, queue_size=1)
        rospy.Subscriber(rospy.get_param("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("aruco detector: marker_size %.1f mm, dictionary %s", 1000 * self.size, name)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        if (msg.header.stamp - self.last).to_sec() < self.period or self.info is None:
            return
        self.last = msg.header.stamp
        K = board_mod.camera_matrix_from_info(self.info.K)
        if K is None:
            rospy.logwarn_throttle(5.0, "camera_info is all zeros: no intrinsic calibration loaded (config/camera_info.yaml)")
            return
        D = np.array(self.info.D, dtype=np.float64)
        frame = self.info.header.frame_id or msg.header.frame_id or "camera_optical_frame"
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.dictionary is None:
            name, dets = aruco.detect_auto(gray)
            if name is not None:
                self.dictionary = name
                rospy.loginfo("aruco dictionary locked to %s (first marker id %d)", name, dets[0].marker_id)
        else:
            dets = aruco.detect(gray, self.dictionary)

        out = bgr.copy()
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        for k, d in enumerate(dets):
            T, rms, amb = aruco.estimate_pose(d.corners, self.size, K, D)
            if T is None:
                continue
            dist = aruco.distance(T)
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = msg.header.stamp, frame, "%s%d" % (self.prefix, d.marker_id)
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = T[:3, 3]
            q = quat_from_matrix(T[:3, :3])
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
            self.br.sendTransform(tf)
            rospy.loginfo_throttle(1.0, "marker %d: distance %.1f mm (depth %.1f mm, lateral %.1f mm), fit %.2f px, ambiguity %.2f",
                                   d.marker_id, 1000 * dist, 1000 * T[2, 3], 1000 * np.hypot(T[0, 3], T[1, 3]), rms, amb)

            rvec, _ = cv2.Rodrigues(T[:3, :3])
            cv2.polylines(out, [d.corners.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 3)
            cv2.drawFrameAxes(out, K, D, rvec, T[:3, 3].reshape(3, 1), self.size * 0.6, 4)
            label = "id %d  d=%.1f mm  fit %.2f px  amb %.2f" % (d.marker_id, 1000 * dist, rms, amb)
            org = (int(d.corners[0][0]), max(30, int(d.corners[0][1]) - 12))
            cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 5, cv2.LINE_AA)
            cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)

            square = Marker(ns="marker", id=d.marker_id, type=Marker.CUBE, action=Marker.ADD)
            square.header.frame_id, square.header.stamp = tf.child_frame_id, msg.header.stamp
            square.pose.orientation.w = 1.0
            square.scale.x = square.scale.y = self.size
            square.scale.z = 0.002
            square.color = ColorRGBA(0.9, 0.9, 0.9, 0.9)
            square.lifetime = rospy.Duration(1.0)
            arr.markers.append(square)

            line = Marker(ns="to_marker", id=d.marker_id, type=Marker.LINE_LIST, action=Marker.ADD)
            line.header.frame_id, line.header.stamp = frame, msg.header.stamp
            line.pose.orientation.w = 1.0
            line.scale.x = 0.002
            line.color = ColorRGBA(1.0, 0.8, 0.0, 1.0)
            line.points = [pt((0, 0, 0)), pt(T[:3, 3])]
            line.lifetime = rospy.Duration(1.0)
            arr.markers.append(line)

            text = Marker(ns="distance", id=d.marker_id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
            text.header = line.header
            text.pose.position = pt(T[:3, 3] * 0.5 + np.array([0.0, -0.03, 0.0]))
            text.pose.orientation.w = 1.0
            text.scale.z = 0.03
            text.color = ColorRGBA(1, 1, 1, 1)
            text.text = "id %d: %.1f mm" % (d.marker_id, 1000 * dist)
            text.lifetime = rospy.Duration(1.0)
            arr.markers.append(text)

        if not dets:
            cv2.putText(out, "no marker found (dictionary %s)" % (self.dictionary or "auto"), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
        self.pub_markers.publish(arr)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv2.resize(out, None, fx=0.5, fy=0.5), "bgr8"))


if __name__ == "__main__":
    rospy.init_node("aruco_detector")
    ArucoNode()
    rospy.spin()
