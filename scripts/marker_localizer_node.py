#!/usr/bin/env python3
"""Detects an ArUco marker in the camera image and reports its pose in the robot base frame. READ-ONLY: it only
subscribes to the image, camera_info and TF; it never commands the arm.

For each marker (see arm_perception/marker_logic.py): solvePnP gives T_camera_marker, TF gives
T_base_camera AT THE IMAGE STAMP, and the product, averaged over a window in which the camera stayed still, is
the pose in base_link. A moving camera (arm in motion, images 50 to 100 ms late) means no report for that window.

Publishes (private names):
  TF  <base_frame> -> <tf_prefix><id>              (valid markers only, default marker_<id>)
  ~marker_<id>  geometry_msgs/PoseStamped         (valid markers only, in base_frame)
  ~report       std_msgs/String (JSON, latched)   every marker seen in the window, valid or not, with spread
  ~observations arm_perception/MarkerObservationArray  every listed marker found in EACH processed frame, in the camera
                frame, unaveraged and at the image stamp (also empty): what a controller (visual servoing) consumes

Markers: config/markers.yaml (~markers_file) lists every marker to look for with its MEASURED size and dictionary
(see arm_perception/marker_config.py); markers seen but not listed are ignored and named under "ignored" in the
report. The detector runs once per dictionary in the table.

Parameters (private): markers_file, base_frame (base_link), tf_prefix (marker_), window_s (0.5),
still_translation_mm (2.0), still_rotation_deg (0.3), min_samples (3), max_rate_hz (15), max_reprojection_px (2.0),
quad_decimate (2.0: marker search on an image shrunk by this factor, about 3x faster than 0 with the same accuracy; a marker
then needs an edge of 40 px, i.e. a 50 mm marker is found out to about 1.9 m; 0 = full resolution, 4 = fastest, 80 px),
image_topic, camera_info_topic, camera_frame ("" = the frame_id of the image).
"""
import json

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from arm_camera import aruco
from arm_camera.geometry import make_transform, matrix_from_quat, quat_from_matrix
from arm_perception import marker_config, marker_detector, marker_logic as ml
from arm_perception.msg import MarkerObservation, MarkerObservationArray


def tf_to_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return make_transform(matrix_from_quat([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


class Node:
    def __init__(self):
        P = rospy.get_param
        self.cfg = marker_config.MarkerConfig.load(P("~markers_file"), lambda name: aruco.get_dictionary(name).bytesList.shape[0])
        self.ignored = {}                                 # id -> (dictionary, stamp seen)
        self.base = P("~base_frame", "base_link")
        self.prefix = P("~tf_prefix", "marker_")
        self.camera_frame = P("~camera_frame", "")
        self.max_rms = float(P("~max_reprojection_px", 2.0))
        self.detector = marker_detector.MarkerDetector(self.cfg, float(P("~quad_decimate", 2.0)), self.max_rms)
        self.period = 1.0 / float(P("~max_rate_hz", 15.0))
        self.tracker = ml.MarkerTracker(float(P("~window_s", 0.5)), float(P("~still_translation_mm", 2.0)),
                                        float(P("~still_rotation_deg", 0.3)), int(P("~min_samples", 3)))
        self.bridge = CvBridge()
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.br = tf2_ros.TransformBroadcaster()
        self.info = None
        self.last = rospy.Time(0)
        self.pose_pubs = {}
        self.pub_report = rospy.Publisher("~report", String, queue_size=1, latch=True)
        self.pub_obs = rospy.Publisher("~observations", MarkerObservationArray, queue_size=2)
        rospy.Subscriber(P("~camera_info_topic", "/usb_cam/camera_info"), CameraInfo, self.on_info, queue_size=1)
        rospy.Subscriber(P("~image_topic", "/usb_cam/image_raw"), Image, self.on_image, queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("marker localizer: %d markers (%s), dictionaries %s, pose in %s (still-only, window %.2f s)",
                      len(self.cfg.specs), ", ".join("%d: %.0f mm" % (i, 1000 * sp.size) for i, sp in sorted(self.cfg.specs.items())),
                      ", ".join(self.cfg.dictionaries()), self.base, self.tracker.window_s)

    def on_info(self, msg):
        self.info = (np.array(msg.K, float).reshape(3, 3), np.array(msg.D, float) if len(msg.D) else np.zeros(5))

    def on_image(self, msg):
        if self.info is None or (msg.header.stamp - self.last).to_sec() < self.period:
            return
        self.last = msg.header.stamp
        frame = self.camera_frame or msg.header.frame_id
        try:
            T_bc = tf_to_matrix(self.buf.lookup_transform(self.base, frame, msg.header.stamp, rospy.Duration(0.2)))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "no TF %s <- %s at the image time: %s", self.base, frame, e)
            return
        t = msg.header.stamp.to_sec()
        self.tracker.add_camera_pose(t, T_bc)
        K, D = self.info
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, "bgr8"), cv2.COLOR_BGR2GRAY)
        found, ignored = self.detector.detect(gray, K, D)
        for i, dname in ignored:
            self.ignored[i] = (dname, t)
        obs = MarkerObservationArray()
        obs.header.stamp, obs.header.frame_id = msg.header.stamp, frame
        for d in found:
            self.tracker.add_detection(d.id, t, ml.base_pose(T_bc, d.T_cam_marker), d.ambiguity, d.rms, d.distance)
            o = MarkerObservation(id=d.id, dictionary=d.dictionary, size=d.size, reprojection_px=d.rms, ambiguity_ratio=d.ambiguity, distance=d.distance)
            o.pose.position.x, o.pose.position.y, o.pose.position.z = [float(v) for v in d.T_cam_marker[:3, 3]]
            q = quat_from_matrix(d.T_cam_marker[:3, :3])
            o.pose.orientation.x, o.pose.orientation.y, o.pose.orientation.z, o.pose.orientation.w = [float(v) for v in q]
            obs.markers.append(o)
        self.pub_obs.publish(obs)
        self.publish(t, msg.header.stamp)

    def publish(self, t, stamp):
        items = self.tracker.report(t)
        for it in items:
            if not it["valid"]:
                continue
            T = it["T_base_marker"]
            q = quat_from_matrix(T[:3, :3])
            tf = TransformStamped()
            tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, self.base, "%s%d" % (self.prefix, it["id"])
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = [float(v) for v in T[:3, 3]]
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = [float(v) for v in q]
            self.br.sendTransform(tf)
            pose = PoseStamped()
            pose.header.stamp, pose.header.frame_id = stamp, self.base
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            if it["id"] not in self.pose_pubs:
                self.pose_pubs[it["id"]] = rospy.Publisher("~marker_%d" % it["id"], PoseStamped, queue_size=1)
            self.pose_pubs[it["id"]].publish(pose)
        recent = [(i, dn) for i, (dn, ts) in self.ignored.items() if t - ts < 2.0][:10]
        self.pub_report.publish(String(json.dumps(ml.report_dict(items, self.base, self.tracker.camera_moving(), t,
                                                               self.cfg.specs, recent))))


def main():
    rospy.init_node("marker_localizer")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
