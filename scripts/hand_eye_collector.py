#!/usr/bin/env python3
"""Collects hand-eye calibration samples. It NEVER moves the arm: you move it (rviz marker,
reach_pose_cli, or by hand with torque off) and call the capture service once it is still.

Services (std_srvs/Trigger, under this node's name):
  ~capture  check that the arm is still and the board is found, then store one sample
  ~undo     delete the last sample
  ~solve    solve with every method on the stored samples and write the hand-eye yaml
Topic ~debug_image shows what the detector sees (open it in rqt_image_view or rviz).

Each sample = T_base_ee from TF (joint states + forward kinematics) and T_cam_board from
solvePnP on the detected chessboard corners. See docs/camera_calibration.md.
"""
import datetime
import os
import threading

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

from arm_camera import board as board_mod
from arm_camera import hand_eye, session
from arm_camera.geometry import invert_transform, make_transform, matrix_from_quat, rotation_error_deg


def tf_to_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return make_transform(matrix_from_quat([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


class Collector:
    def __init__(self):
        b = rospy.get_param("~board")
        self.board = board_mod.Board(int(b["cols"]), int(b["rows"]), float(b["square_size"]))
        if not self.board.is_unambiguous():
            rospy.logwarn("Board %dx%d has an ambiguous orientation (both inner-corner counts even or both odd): "
                          "detection can flip it by 180 degrees between frames and ruin the calibration. "
                          "Use an even x odd board such as 9x6.", self.board.cols, self.board.rows)
        self.base = rospy.get_param("~base_frame", "base_link")
        self.ee = rospy.get_param("~ee_frame", "end_effector")
        self.still_s = float(rospy.get_param("~still_window_s", 0.6))
        self.still_mm = float(rospy.get_param("~still_translation_mm", 0.5))
        self.still_deg = float(rospy.get_param("~still_rotation_deg", 0.1))
        self.max_rms = float(rospy.get_param("~max_reprojection_px", 1.0))
        default_dir = os.path.expanduser("~/.ros/arm_camera/hand_eye_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.dir = rospy.get_param("~session_dir", default_dir)
        self.output = rospy.get_param("~output_file", os.path.expanduser("~/.ros/arm_camera/hand_eye.yaml"))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image, self.info = None, None
        self.count = len(session.load_samples(self.dir)) if os.path.isdir(self.dir) else 0
        self.first_dark = None
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        rospy.Service("~capture", Trigger, self.capture)
        rospy.Service("~undo", Trigger, self.undo)
        rospy.Service("~solve", Trigger, self.solve)
        rospy.Timer(rospy.Duration(1.0), self.debug_tick)
        rospy.loginfo("hand_eye_collector: board %dx%d inner corners, %.1f mm squares; session %s (%d samples). "
                      "Move the arm, hold still, then: rosservice call %s/capture",
                      self.board.cols, self.board.rows, 1000 * self.board.square_size, self.dir, self.count, rospy.get_name())

    def on_image(self, msg):
        with self.lock:
            self.image = msg

    def on_info(self, msg):
        with self.lock:
            self.info = msg

    def intrinsics(self):
        with self.lock:
            info = self.info
        if info is None:
            return None, None, "no camera_info yet (is the camera node running?)"
        K = board_mod.camera_matrix_from_info(info.K)
        if K is None:
            return None, None, ("camera_info is all zeros: the camera has no intrinsic calibration loaded. "
                                "Run the intrinsics calibration first (docs/camera_calibration.md).")
        return K, np.array(info.D, dtype=np.float64), ""

    def arm_is_still(self):
        try:
            now = rospy.Time.now()
            a = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, rospy.Time(0), rospy.Duration(1.0)))
            b = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, now - rospy.Duration(self.still_s), rospy.Duration(1.0)))
        except tf2_ros.TransformException as e:
            return False, "cannot read %s -> %s from TF (is the arm stack running?): %s" % (self.base, self.ee, e)
        dt = np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000.0
        dr = rotation_error_deg(a[:3, :3], b[:3, :3])
        if dt > self.still_mm or dr > self.still_deg:
            return False, "the arm moved %.2f mm / %.2f deg in the last %.1f s: wait until it is still" % (dt, dr, self.still_s)
        return True, ""

    def detect(self, msg, K, D):
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners = board_mod.detect_corners(gray, self.board)
        if corners is None:
            return bgr, gray, None, None, float("inf")
        T, rms = board_mod.estimate_pose(corners, self.board, K, D)
        return bgr, gray, corners, T, rms

    def capture(self, _):
        K, D, why = self.intrinsics()
        if K is None:
            return TriggerResponse(False, why)
        with self.lock:
            msg = self.image
        if msg is None or (rospy.Time.now() - msg.header.stamp).to_sec() > 1.0:
            return TriggerResponse(False, "no fresh image from the camera")
        still, why = self.arm_is_still()
        if not still:
            return TriggerResponse(False, why)
        bgr, gray, corners, T_cb, rms = self.detect(msg, K, D)
        if corners is None:
            return TriggerResponse(False, "chessboard not found: check %dx%d inner corners, lighting, focus, and that it is fully in view"
                                   % (self.board.cols, self.board.rows))
        if rms > self.max_rms:
            return TriggerResponse(False, "board found but the pose fit is poor (%.2f px > %.2f): check board flatness, focus, motion blur"
                                   % (rms, self.max_rms))
        try:
            T_be = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, msg.header.stamp, rospy.Duration(0.5)))
        except tf2_ros.TransformException:
            T_be = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, rospy.Time(0), rospy.Duration(1.0)))
        dark = board_mod.first_square_is_dark(gray, corners, self.board)
        if self.first_dark is None:
            self.first_dark = dark
        elif dark != self.first_dark:
            return TriggerResponse(False, "the detector labelled the board from the other end in this view (180 degree flip): "
                                   "do not use this pose, tilt the camera less or rotate the board")
        session.write_session_meta(self.dir, self.board, K, D, (msg.width, msg.height), self.first_dark)
        sample = hand_eye.Sample(T_base_ee=T_be, T_cam_board=T_cb, corners=corners)
        session.save_sample(self.dir, self.count, sample, bgr)
        self.count += 1
        rot, axis = hand_eye.motion_diversity(session.load_samples(self.dir))
        return TriggerResponse(True, "sample %d saved (fit %.2f px, board %.2f m away, %.0f deg tilt). Diversity so far: "
                               "largest rotation between poses %.0f deg, largest angle between rotation axes %.0f deg."
                               % (self.count, rms, T_cb[2, 3], np.degrees(np.arccos(min(1.0, abs(T_cb[2, 2])))), rot, axis))

    def undo(self, _):
        if self.count == 0:
            return TriggerResponse(False, "no samples")
        self.count -= 1
        for ext in (".npz", ".png"):
            path = os.path.join(self.dir, "sample_%03d%s" % (self.count, ext))
            if os.path.exists(path):
                os.remove(path)
        return TriggerResponse(True, "removed sample %d, %d left" % (self.count, self.count))

    def solve(self, _):
        samples = session.load_samples(self.dir)
        if len(samples) < 6:
            return TriggerResponse(False, "%d samples: collect at least 10 (6 is the absolute minimum)" % len(samples))
        board, K, D, size, dark = session.read_session_meta(self.dir)
        results = hand_eye.solve_all(samples, K, D, board)
        if not results:
            return TriggerResponse(False, "no method converged: the poses are probably degenerate")
        best = results[0]
        div = hand_eye.motion_diversity(samples)
        os.makedirs(os.path.dirname(self.output), exist_ok=True)
        session.save_hand_eye(self.output, best, board, len(samples), div, dark, size)
        return TriggerResponse(True, "%s: board-pose scatter %.1f mm / %.2f deg, reprojection %.2f px over %d samples; written to %s"
                               % (best.method, best.position_scatter_mm, best.rotation_scatter_deg, best.reprojection_rms_px,
                                  len(samples), self.output))

    def debug_tick(self, _):
        K, D, _why = self.intrinsics()
        with self.lock:
            msg = self.image
        if msg is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        small = cv2.resize(bgr, None, fx=0.5, fy=0.5)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners = board_mod.detect_corners(gray, self.board)
        status = "samples: %d" % self.count
        if corners is not None:
            cv2.drawChessboardCorners(small, self.board.pattern_size, (corners * 0.5).astype(np.float32), True)
            status += " | board found"
            if K is not None:
                _, rms = board_mod.estimate_pose(corners, self.board, K, D)
                status += " (fit %.2f px)" % rms
        else:
            status += " | board NOT found"
        if K is None:
            status += " | NO INTRINSICS"
        cv2.putText(small, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(small, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 1, cv2.LINE_AA)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(small, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("hand_eye_collector")
    Collector()
    rospy.spin()
