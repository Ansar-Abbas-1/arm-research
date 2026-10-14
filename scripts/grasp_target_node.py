#!/usr/bin/env python3
"""Publishes the grasp target frame as a marker plus a hand-measured offset, for the grasp sequence (it takes only a TF frame).

    p_target = p_marker + R * offset      (offset in the frame R, +z out of the surface; R = the marker's own frame, or with ~vertical:=true
                                           its UPRIGHT version: z = up, the marker's heading kept, so the tilt noise of the marker pose is dropped)

The marker localizer publishes marker_<id> only while the camera is still and sees it. This node remembers the last marker pose and
keeps broadcasting <target_frame> with the MARKER'S timestamp, not the current time: the target ages like the observation, so the grasp
sequence's freshness check (target_max_age) refuses a target nobody has seen recently. Nothing here commands the robot.

    roslaunch arm_grasp grasp_target.launch marker:=marker_984 offset_x:=0.03 offset_y:=0 offset_z:=0.0

~approach (default down; read every cycle, so it can be changed while running: rosparam set /grasp_target/approach forward):
    down      the frame's z is the surface normal (up for a floor target): the gripper comes from above
    forward   a SIDE grasp: z is horizontal, back toward the robot's base, so the gripper looks outward along the radial direction and closes sideways
              on the target (arm_grasp/target_offset.side_frame; the planner's waypoints then approach horizontally, see config/grasp.yaml side_*)
"""
import json

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from arm_grasp import grasp_plan as gp
from arm_grasp import target_offset as to


def main():
    rospy.init_node("grasp_target")
    marker = rospy.get_param("~marker_frame")
    offset = [float(v) for v in rospy.get_param("~offset", [0.0, 0.0, 0.0])]
    target = rospy.get_param("~target_frame", "grasp_target")
    vertical = bool(rospy.get_param("~vertical", False))
    approach = "down"                                             # re-read below every cycle: rosparam set /grasp_target/approach forward
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
            p, R = to.offset_target_frame([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w], offset, vertical)
            approach = str(rospy.get_param("~approach", "down")).strip().lower()
            side = approach in ("forward", "side")
            if side:
                R = to.side_frame(p)                              # z horizontal, back toward the base: the gripper looks forward (radially outward)
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = last.header.stamp, base, target
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = [float(v) for v in p]
            if vertical or side:
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = gp.quaternion_from_matrix(R)
            else:
                m.transform.rotation = last.transform.rotation
            br.sendTransform(m)
            pub.publish(String(json.dumps({"marker": marker, "offset": offset, "age_s": (rospy.Time.now() - last.header.stamp).to_sec(),
                                           "approach": "forward" if side else "down", "target_xyz": [float(v) for v in p]})))
        rate.sleep()


if __name__ == "__main__":
    main()
