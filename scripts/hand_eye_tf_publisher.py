#!/usr/bin/env python3
"""Publishes the calibrated end_effector -> camera_optical_frame transform as a static TF."""
import os
import sys

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from arm_camera import session
from arm_camera.geometry import quat_from_matrix


def main():
    rospy.init_node("hand_eye_tf_publisher")
    path = rospy.get_param("~file")
    if not os.path.exists(path):
        rospy.logfatal("hand-eye file %s does not exist: run the hand-eye calibration first "
                       "(docs/camera_calibration.md). Not publishing a made-up camera pose.", path)
        sys.exit(1)
    data = session.load_hand_eye(path)
    X = data["X"]
    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = session.PARENT_FRAME
    msg.child_frame_id = session.CHILD_FRAME
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = X[:3, 3]
    q = quat_from_matrix(X[:3, :3])
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    tf2_ros.StaticTransformBroadcaster().sendTransform(msg)
    m = data["raw"]["metrics"]
    rospy.loginfo("static TF %s -> %s from %s (method %s, %d samples, board scatter %.1f mm, reprojection %.2f px)",
                  msg.header.frame_id, msg.child_frame_id, path, data["raw"]["method"], data["raw"]["num_samples"],
                  m["position_scatter_mm"], m["reprojection_rms_px"])
    rospy.spin()


if __name__ == "__main__":
    main()
