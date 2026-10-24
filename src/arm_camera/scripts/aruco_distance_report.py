#!/usr/bin/env python3
"""Compares the distance the camera reports to a marker (read from TF, as rviz shows it) with
the distance you measured with a ruler.

  rosrun arm_camera aruco_distance_report.py --id 3 --measured 452 [--offset 4] [--seconds 5]

Measure from the same point the camera measures from: the camera's OPTICAL CENTRE, which is
inside the lens, a few millimetres in front of the sensor surface (about the focal length,
roughly 3 to 4 mm for this camera). If you measured from the sensor surface, pass the
sensor-to-optical-centre distance as --offset (mm): the expected TF distance is then
measured - offset. Prints mean, spread and the error in mm and percent over --seconds.
"""
import argparse
import sys
import time

import numpy as np
import rospy
import tf2_ros


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--measured", type=float, help="ruler distance to the marker centre [mm]")
    ap.add_argument("--offset", type=float, default=0.0, help="sensor surface to optical centre [mm], subtracted from --measured")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--camera-frame", default="camera_optical_frame")
    ap.add_argument("--prefix", default="aruco_")
    args, _ = ap.parse_known_args(rospy.myargv()[1:])

    rospy.init_node("aruco_distance_report", anonymous=True)
    buf = tf2_ros.Buffer(rospy.Duration(20.0))
    tf2_ros.TransformListener(buf)
    child = "%s%d" % (args.prefix, args.id)
    print("reading TF %s -> %s for %.0f s ..." % (args.camera_frame, child, args.seconds))
    samples, stamps = [], set()
    t_end = time.time() + args.seconds
    while time.time() < t_end and not rospy.is_shutdown():
        try:
            m = buf.lookup_transform(args.camera_frame, child, rospy.Time(0), rospy.Duration(0.2))
            if m.header.stamp not in stamps:
                stamps.add(m.header.stamp)
                t = m.transform.translation
                samples.append([t.x, t.y, t.z])
        except tf2_ros.TransformException:
            pass
        time.sleep(0.02)
    if len(samples) < 3:
        sys.exit("no TF for %s: is the marker visible, and is aruco_detector running?" % child)
    p = np.array(samples) * 1000.0
    d = np.linalg.norm(p, axis=1)
    print("\n%d detections of marker %d" % (len(d), args.id))
    print("distance camera optical centre -> marker centre: mean %.2f mm, std %.2f mm, range %.2f..%.2f mm" % (d.mean(), d.std(), d.min(), d.max()))
    print("  depth (along the optical axis) %.2f mm, lateral offset %.2f mm" % (p[:, 2].mean(), np.hypot(p[:, 0].mean(), p[:, 1].mean())))
    if d.std() > 1.0:
        print("  NOTE: std above 1 mm: the reading is unsteady (light, blur, a marker tilted close to frontal, or a loose mount)")
    if args.measured is not None:
        expected = args.measured - args.offset
        err = d.mean() - expected
        print("\nmeasured with the ruler %.1f mm%s -> expected %.1f mm" % (args.measured, " (minus offset %.1f mm)" % args.offset if args.offset else "", expected))
        print("camera minus ruler: %+.2f mm (%+.2f %%)" % (err, 100.0 * err / expected))
        print("A constant offset points at where you measured from; an error proportional to distance points at marker_size or the focal length.")


if __name__ == "__main__":
    main()
