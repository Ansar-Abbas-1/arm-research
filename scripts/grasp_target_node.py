#!/usr/bin/env python3
"""Publishes the grasp target frame as a marker plus a hand-measured offset, for the grasp sequence (it takes only a TF frame).

    p_target = p_marker + R * offset      (offset in the frame R, +z out of the surface; R = the marker's own frame, or with ~vertical:=true
                                           its UPRIGHT version: z = up, the marker's heading kept, so the tilt noise of the marker pose is dropped)

The marker localizer publishes marker_<id> only while the camera is still and sees it. This node remembers the last marker pose and
keeps broadcasting <target_frame> with the MARKER'S timestamp, not the current time: the target ages like the observation, so the grasp
sequence's freshness check (target_max_age) refuses a target nobody has seen recently. Nothing here commands the robot.

    roslaunch arm_grasp grasp_target.launch marker:=marker_984 offset_x:=0.03 offset_y:=0 offset_z:=0.0

~bias ([radial, tangential, vertical] metres in the ARM's frame at the target's azimuth, default 0; read every cycle: rosparam set /grasp_target/bias "[-0.01, 0.01, 0]"):
    moves the final aim point (radial outward, tangential toward increasing azimuth = left as seen from the robot looking outward, vertical up). For a residual aim
    error found on the hardware (calibration, the mask centroid against the stem).

~offset ([x, y, z] in the marker frame, read every cycle: rosparam set /grasp_target/offset "[0.02, -0.2, 0.07]"): the grasp point relative to the marker.

~approach (default down; read every cycle, so it can be changed while running: rosparam set /grasp_target/approach 45):
    down      the frame's z is the surface normal (up for a floor target): the gripper comes from above (= 90)
    forward   a SIDE grasp: z is horizontal, back toward the robot's base, so the gripper looks outward along the radial direction (= 0)
    <number>  the angle of the tool axis BELOW the horizontal, in degrees (0 to 90): 45 looks outward and down at 45 degrees, from the robot's side, and
              closes sideways on the target (arm_grasp/target_offset.angled_frame; the planner then approaches along that line, see config/grasp.yaml side_*)
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
            try:                                                       # the offset is a parameter read every cycle too: rosparam set /grasp_target/offset "[x, y, z]"
                offset = [float(v) for v in rospy.get_param("~offset", offset)][:3]
            except (TypeError, ValueError):
                rospy.logerr_throttle(5.0, "grasp_target: ~offset must be three numbers; keeping %s", offset)
            p, R = to.offset_target_frame([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w], offset, vertical)
            try:                                                        # ~bias [radial, tangential, vertical] m in the arm's frame, read every cycle
                bias = [float(v) for v in rospy.get_param("~bias", [0.0, 0.0, 0.0])][:3]
                p = to.apply_bias(p, bias)
            except (TypeError, ValueError):
                rospy.logerr_throttle(5.0, "grasp_target: ~bias must be three numbers; ignoring it")
                bias = [0.0, 0.0, 0.0]
            try:
                angle = to.parse_approach(rospy.get_param("~approach", "down"))
            except ValueError as e:
                rospy.logerr_throttle(5.0, "grasp_target: %s; using down", e)
                angle = 90.0
            side = angle < 89.0
            if side:
                R = to.angled_frame(p, angle)                     # z back toward the base and up at this angle: the gripper looks outward and down
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = last.header.stamp, base, target
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = [float(v) for v in p]
            if vertical or side:
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = gp.quaternion_from_matrix(R)
            else:
                m.transform.rotation = last.transform.rotation
            br.sendTransform(m)
            pub.publish(String(json.dumps({"marker": marker, "offset": offset, "age_s": (rospy.Time.now() - last.header.stamp).to_sec(),
                                           "approach_deg": angle, "bias_m": bias, "target_xyz": [float(v) for v in p]})))
        rate.sleep()


if __name__ == "__main__":
    main()
