#!/usr/bin/env python3
"""Shows how well the whole chain agrees with what the camera sees.

The fixed chessboard sits at T_base_board (from the calibration). For every camera image
the node predicts where its corners must appear using ONLY the arm and the calibration:
joint states -> TF (base -> end_effector) -> hand-eye (end_effector -> camera) -> intrinsics.
It also detects the corners in the image. Green rings = detected, red crosses = predicted,
yellow lines = the error. With a good calibration they coincide at every arm pose.

Outputs:
  ~image            annotated camera image (open in rviz as an Image display)
  /arm_camera/markers   MarkerArray: the board as a checkerboard, its axes, the camera
                        frustum and the RMS error as text, all in the rviz 3D view
"""
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from arm_camera import board as board_mod
from arm_camera import overlay, session
from arm_camera.geometry import invert_transform, make_transform, matrix_from_quat, quat_from_matrix


def tf_to_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return make_transform(matrix_from_quat([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


def pt(p):
    return Point(float(p[0]), float(p[1]), float(p[2]))


class OverlayNode:
    def __init__(self):
        data = session.load_hand_eye(rospy.get_param("~hand_eye_file"))
        self.B, self.board = data["B"], data["board"]
        if self.board is None:
            rospy.logfatal("the hand-eye file comes from the ChArUco mount calibration: the overlay draws a chessboard and "
                           "needs a chessboard-based calibration (hand_eye_collector). The camera pose in this file is still "
                           "published by hand_eye_tf_publisher.py.")
            raise SystemExit(1)
        self.first_dark = bool(data["raw"]["board"].get("first_square_dark") or False)
        self.base = rospy.get_param("~base_frame", "base_link")
        self.cam = rospy.get_param("~camera_frame", session.CHILD_FRAME)
        self.period = 1.0 / float(rospy.get_param("~detect_hz", 3.0))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.info = None
        self.last = rospy.Time(0)
        self.rms = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub_img = rospy.Publisher("~image", Image, queue_size=1)
        self.pub_markers = rospy.Publisher("/arm_camera/markers", MarkerArray, queue_size=1)
        rospy.Subscriber(rospy.get_param("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(0.5), self.publish_markers)
        m = data["raw"]["metrics"]
        rospy.loginfo("overlay: calibration from %s (%d samples, board scatter %.1f mm, reprojection %.2f px)",
                      rospy.get_param("~hand_eye_file"), data["raw"]["num_samples"], m["position_scatter_mm"], m["reprojection_rms_px"])

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def on_image(self, msg):
        if (msg.header.stamp - self.last).to_sec() < self.period:
            return
        self.last = msg.header.stamp
        with self.lock:
            info = self.info
        if info is None:
            return
        K = board_mod.camera_matrix_from_info(info.K)
        if K is None:
            rospy.logwarn_throttle(5.0, "camera_info is all zeros: no intrinsic calibration loaded")
            return
        D = np.array(info.D, dtype=np.float64)
        try:
            T_base_cam = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.cam, msg.header.stamp, rospy.Duration(0.2)))
        except tf2_ros.TransformException:
            try:
                T_base_cam = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.cam, rospy.Time(0), rospy.Duration(0.5)))
            except tf2_ros.TransformException as e:
                rospy.logwarn_throttle(5.0, "no TF %s -> %s: %s", self.base, self.cam, e)
                return
        pred = board_mod.project(K, D, invert_transform(T_base_cam) @ self.B, self.board.object_points())
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners = board_mod.detect_corners(gray, self.board)
        extra = None
        if corners is not None:
            self.rms = float(np.sqrt(np.mean(overlay.corner_errors(corners, pred) ** 2)))
        else:
            self.rms = None
            extra = ["(the predicted board position is drawn anyway)"]
        out = overlay.draw_overlay(bgr, corners, pred, extra, scale=bgr.shape[1] / 960.0)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(cv2.resize(out, None, fx=0.5, fy=0.5), "bgr8"))

    def publish_markers(self, _):
        arr = MarkerArray()
        now = rospy.Time.now()

        board = Marker(ns="board", id=0, type=Marker.TRIANGLE_LIST, action=Marker.ADD)
        board.header.frame_id, board.header.stamp = self.base, now
        q = quat_from_matrix(self.B[:3, :3])
        board.pose.position = pt(self.B[:3, 3])
        board.pose.orientation.x, board.pose.orientation.y, board.pose.orientation.z, board.pose.orientation.w = q
        board.scale.x = board.scale.y = board.scale.z = 1.0
        points, colours = overlay.board_triangles(self.board, self.first_dark)
        board.points = [pt(p) for p in points]
        board.colors = [ColorRGBA(c[0], c[1], c[2], 1.0) for c in colours]
        arr.markers.append(board)

        axes = Marker(ns="board_axes", id=0, type=Marker.LINE_LIST, action=Marker.ADD)
        axes.header = board.header
        axes.pose = board.pose
        axes.scale.x = 0.004
        L = 3 * self.board.square_size
        for tip, rgb in (((L, 0, 0), (1, 0, 0)), ((0, L, 0), (0, 1, 0)), ((0, 0, L), (0, 0, 1))):
            axes.points += [pt((0, 0, 0)), pt(tip)]
            axes.colors += [ColorRGBA(*rgb, 1.0)] * 2
        arr.markers.append(axes)

        with self.lock:
            info = self.info
        K = board_mod.camera_matrix_from_info(info.K) if info is not None else None
        if K is not None:
            fr = Marker(ns="frustum", id=0, type=Marker.LINE_LIST, action=Marker.ADD)
            fr.header.frame_id, fr.header.stamp = self.cam, now
            fr.pose.orientation.w = 1.0
            fr.scale.x = 0.003
            fr.color = ColorRGBA(0.2, 0.6, 1.0, 1.0)
            fr.points = [pt(p) for p in overlay.frustum_lines(K, (info.width, info.height), 0.12)]
            arr.markers.append(fr)

        txt = Marker(ns="rms", id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        txt.header.frame_id, txt.header.stamp = self.base, now
        txt.pose.position = pt(self.B[:3, 3] + np.array([0.0, 0.0, 0.08]))
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.035
        txt.color = ColorRGBA(1, 1, 1, 1)
        txt.text = "reprojection RMS %.1f px" % self.rms if self.rms is not None else "board not detected"
        arr.markers.append(txt)
        self.pub_markers.publish(arr)


if __name__ == "__main__":
    rospy.init_node("calibration_overlay")
    OverlayNode()
    rospy.spin()
