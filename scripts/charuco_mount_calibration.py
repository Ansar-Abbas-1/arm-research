#!/usr/bin/env python3
"""Calibrates where the camera sits on the arm (end_effector -> camera_optical_frame) from a ChArUco
board lying on the ground. It NEVER moves the arm: you move it (rviz marker, reach_pose_cli, or by hand),
hold still, and call the capture service. See docs/charuco_mount_calibration.md.

Services (std_srvs/Trigger, under this node's name):
  ~capture  check the arm is still and the board is found well, store one view
  ~undo     delete the last view
  ~solve    solve and write the result (hand_eye.yaml format, read by hand_eye_tf_publisher.py)
Topic ~debug_image shows what the detector sees.

Parameters: ~board (squares_x, squares_y, square_length, marker_length, dictionary: config/charuco.yaml),
~board_pose [x, y, z, yaw_deg] (optional: the board's pose in base_link, board lying face up and level),
~board_origin ("corner": bottom-left outer corner as printed, or "center"), ~method (auto|known|handeye).
With ~board_pose the mount follows from each single view; without it hand-eye is used (8+ views with
rotation about two axes). With both available the two are compared.
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
from arm_camera import charuco, hand_eye
from arm_camera import mount_calibration as mc
from arm_camera.geometry import make_transform, matrix_from_quat, rotation_error_deg


def tf_to_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return make_transform(matrix_from_quat([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


class Node:
    def __init__(self):
        b = rospy.get_param("~board")
        self.spec = charuco.CharucoSpec(int(b["squares_x"]), int(b["squares_y"]), float(b["square_length"]),
                                        float(b["marker_length"]), b.get("dictionary", "4X4_50"))
        self.base = rospy.get_param("~base_frame", "base_link")
        self.ee = rospy.get_param("~ee_frame", "end_effector")
        pose = rospy.get_param("~board_pose", None)
        self.origin = rospy.get_param("~board_origin", "corner")
        self.T_bb = charuco.board_pose_in_base(*[float(v) for v in pose], spec=self.spec, origin=self.origin) if pose else None
        self.method = rospy.get_param("~method", "auto")
        self.min_corners = int(rospy.get_param("~min_corners", 12))
        self.max_rms = float(rospy.get_param("~max_reprojection_px", 1.0))
        self.min_tilt = float(rospy.get_param("~min_tilt_deg", 10.0))
        self.still_s = float(rospy.get_param("~still_window_s", 0.6))
        self.still_mm = float(rospy.get_param("~still_translation_mm", 0.5))
        self.still_deg = float(rospy.get_param("~still_rotation_deg", 0.1))
        default_dir = os.path.expanduser("~/.ros/arm_camera/charuco_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        self.dir = rospy.get_param("~session_dir", default_dir)
        self.output = rospy.get_param("~output_file", os.path.expanduser("~/.ros/arm_camera/hand_eye.yaml"))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image, self.info = None, None
        self.samples = mc.load_samples(self.dir) if os.path.isdir(self.dir) else []
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(rospy.get_param("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(rospy.get_param("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        rospy.Service("~capture", Trigger, self.capture)
        rospy.Service("~undo", Trigger, self.undo)
        rospy.Service("~solve", Trigger, self.solve)
        rospy.Timer(rospy.Duration(1.0), self.debug_tick)
        rospy.loginfo("charuco_mount_calibration: board %dx%d squares, %.1f mm, %s; board pose %s; session %s (%d views). "
                      "Move the arm, hold still, then: rosservice call %s/capture",
                      self.spec.squares_x, self.spec.squares_y, 1000 * self.spec.square_length, self.spec.dictionary,
                      "known (%s origin)" % self.origin if self.T_bb is not None else "NOT given: hand-eye mode", self.dir, len(self.samples), rospy.get_name())

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
            return None, None, "camera_info is all zeros: no intrinsic calibration loaded (config/camera_info.yaml)"
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
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        det = charuco.detect(bgr, self.spec, K, D, min_corners=self.min_corners)
        if det is None:
            return TriggerResponse(False, "fewer than %d ChArUco corners found: check that the board is in view, lit and in focus, "
                                   "and that config/charuco.yaml matches the print (squares, dictionary)" % self.min_corners)
        T_cb, rms, ratio = charuco.estimate_pose(det, self.spec, K, D)
        tilt = charuco.tilt_deg(T_cb)
        if rms > self.max_rms:
            return TriggerResponse(False, "board found (%d corners) but the pose fit is poor (%.2f px > %.2f): blur, a bent board, or a wrong square_length/marker_length" % (len(det.ids), rms, self.max_rms))
        if tilt < self.min_tilt and ratio > 0.8:
            return TriggerResponse(False, "the board is almost frontal (%.0f deg tilt) so its orientation is ambiguous (ambiguity %.2f): tilt the camera by at least %.0f degrees" % (tilt, ratio, self.min_tilt))
        try:
            T_be = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, msg.header.stamp, rospy.Duration(0.5)))
        except tf2_ros.TransformException:
            T_be = tf_to_matrix(self.tf_buffer.lookup_transform(self.base, self.ee, rospy.Time(0), rospy.Duration(1.0)))
        sample = mc.MountSample(T_be, T_cb, num_corners=len(det.ids), rms=rms)
        self.samples.append(sample)
        mc.append_sample(self.dir, len(self.samples) - 1, sample, bgr)
        text = "view %d saved: %d corners, fit %.2f px, board %.2f m away, %.0f deg tilt." % (len(self.samples), len(det.ids), rms, T_cb[2, 3], tilt)
        if self.T_bb is not None:
            Xs = [charuco.mount_from_known_board(s.T_base_ee, s.T_cam_board, self.T_bb) for s in self.samples]
            est = charuco.robust_mount(Xs)
            text += " Mount from this view: x=%.1f y=%.1f z=%.1f mm; all %d views: x=%.1f y=%.1f z=%.1f mm (spread %.1f mm)." % (
                1000 * Xs[-1][0, 3], 1000 * Xs[-1][1, 3], 1000 * Xs[-1][2, 3], len(Xs), 1000 * est.X[0, 3], 1000 * est.X[1, 3], 1000 * est.X[2, 3], est.translation_spread_mm)
        he = [hand_eye.Sample(s.T_base_ee, s.T_cam_board) for s in self.samples]
        rot, axis = hand_eye.motion_diversity(he)
        text += " Pose diversity: %.0f deg rotation, %.0f deg between rotation axes." % (rot, axis)
        return TriggerResponse(True, text)

    def undo(self, _):
        if not self.samples:
            return TriggerResponse(False, "no views")
        self.samples.pop()
        for ext in (".npz", ".png"):
            path = os.path.join(self.dir, "view_%03d%s" % (len(self.samples), ext))
            if os.path.exists(path):
                os.remove(path)
        return TriggerResponse(True, "removed the last view, %d left" % len(self.samples))

    def solve(self, _):
        if not self.samples:
            return TriggerResponse(False, "no views")
        try:
            res = mc.solve(self.samples, self.T_bb, self.method)
        except ValueError as e:
            return TriggerResponse(False, str(e))
        os.makedirs(os.path.dirname(os.path.abspath(self.output)), exist_ok=True)
        pose = rospy.get_param("~board_pose", None)
        info = dict(zip(["x", "y", "z", "yaw_deg"], [float(v) for v in pose]), origin=self.origin) if pose else None
        mc.save_mount_yaml(self.output, res, self.spec, info)
        text = mc.report(res) + "\nwritten to " + self.output
        rospy.loginfo("\n" + text)
        return TriggerResponse(True, text)

    def debug_tick(self, _):
        K, D, _why = self.intrinsics()
        with self.lock:
            msg = self.image
        if msg is None:
            return
        bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        small = cv2.resize(bgr, None, fx=0.5, fy=0.5)
        status = "views: %d" % len(self.samples)
        det = charuco.detect(bgr, self.spec, K, D, min_corners=4) if K is not None else charuco.detect(bgr, self.spec, min_corners=4)
        if det is not None:
            for p in det.corners:
                cv2.circle(small, (int(p[0] / 2), int(p[1] / 2)), 4, (0, 255, 0), -1)
            status += " | %d corners" % len(det.ids)
            if K is not None and len(det.ids) >= 6:
                T, rms, ratio = charuco.estimate_pose(det, self.spec, K, D)
                if T is not None:
                    rvec, _ = cv2.Rodrigues(T[:3, :3])
                    cv2.drawFrameAxes(small, K / 2.0 * np.array([[1, 1, 1], [1, 1, 1], [0, 0, 2.0]]), D, rvec, T[:3, 3].reshape(3, 1), 0.06, 3)
                    status += " | fit %.2f px, tilt %.0f deg, ambiguity %.2f" % (rms, charuco.tilt_deg(T), ratio)
        else:
            status += " | board NOT found"
        if K is None:
            status += " | NO INTRINSICS"
        cv2.putText(small, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(small, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(small, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("charuco_mount_calibration")
    Node()
    rospy.spin()
