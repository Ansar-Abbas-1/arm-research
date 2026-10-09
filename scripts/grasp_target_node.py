#!/usr/bin/env python3
"""Publishes the grasp target frame as a marker plus a hand-measured offset, for the grasp sequence (it takes only a TF frame).

    p_target = p_marker + R_marker * offset      (offset in the MARKER frame, +z out of the surface; orientation = the marker's)

The marker localizer publishes marker_<id> only while the camera is still and sees it. This node remembers the last marker pose and
keeps broadcasting <target_frame> with the MARKER'S timestamp, not the current time: the target ages like the observation, so the grasp
sequence's freshness check (target_max_age) refuses a target nobody has seen recently. Nothing here commands the robot.

    roslaunch arm_grasp grasp_target.launch marker:=marker_984 offset_x:=0.03 offset_y:=0 offset_z:=0.0
"""
import json

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from arm_grasp import target_offset as to


def main():
    rospy.init_node("grasp_target")
    marker = rospy.get_param("~marker_frame")
    offset = [float(v) for v in rospy.get_param("~offset", [0.0, 0.0, 0.0])]
    target = rospy.get_param("~target_frame", "grasp_target")
    base = rospy.get_param("~base_frame", "base_link")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    br = tf2_ros.TransformBroadcaster()
    pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
    last = None                                                   # the latest marker transform (a TransformStamped)
    rate = rospy.Rate(2)                                          # re-broadcast at 2 Hz: tf2 warns about every repeat of a timestamp
    rospy.logwarn("grasp_target: %s = %s + offset %s (in the marker frame); nothing is published until the marker has been seen", target, marker, offset)
    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform(base, marker, rospy.Time(0), rospy.Duration(0.0))
            if last is None or t.header.stamp != last.header.stamp:
                if last is None:
                    rospy.logwarn("grasp_target: first observation of %s", marker)
                last = t
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
        if last is not None:
            tr, q = last.transform.translation, last.transform.rotation
            p = to.offset_target([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w], offset)
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = last.header.stamp, base, target
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = [float(v) for v in p]
            m.transform.rotation = last.transform.rotation
            br.sendTransform(m)
            pub.publish(String(json.dumps({"marker": marker, "offset": offset, "age_s": (rospy.Time.now() - last.header.stamp).to_sec(),
                                           "target_xyz": [float(v) for v in p]})))
        rate.sleep()


if __name__ == "__main__":
    main()
