#!/usr/bin/env python3
"""End-to-end check of the ArUco chain WITHOUT a camera: a synthetic camera publishes an image of
markers at known poses, the real aruco_detector_node runs, and the real aruco_distance_report.py
reads the TF and compares it with a "ruler" distance. Starts its OWN roscore on port 11312 (never
11311) and kills only what it started. Run manually with ROS sourced:

    python3 src/arm_camera/test/e2e_aruco_synthetic.py
"""
import os
import re
import subprocess
import sys
import tempfile
import time

PORT = 11312
os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % PORT
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "src"))
sys.path.insert(0, HERE)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import rospy  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

import test_aruco as TA  # noqa: E402

procs = []


def start(cmd, log_name):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(PKG, "src") + os.pathsep + env.get("PYTHONPATH", "")
    log = open(os.path.join(tempfile.gettempdir(), "arm_camera_e2e_%s.log" % log_name), "w")
    p = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT)
    procs.append(p)
    return p


def wait_for(cond, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return
        time.sleep(0.1)
    raise RuntimeError("timeout waiting for " + what)


def main():
    ok = True
    try:
        start(["roscore", "-p", str(PORT)], "roscore")
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        assert "/arm_interface" not in subprocess.check_output(["rosnode", "list"]).decode(), "arm_interface on this master"
        rospy.init_node("e2e_aruco")
        bridge = CvBridge()
        pub_img = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1)
        info = CameraInfo()
        info.width, info.height, info.distortion_model = 1920, 1080, "plumb_bob"
        info.K = TA.K.reshape(-1).tolist()
        info.D = [0.0] * 5
        info.header.frame_id = "camera_optical_frame"
        # Marker 5 at 472.0 mm from the optical centre (tilted), marker 8 elsewhere.
        z5 = 0.470
        T5 = TA.pose_facing_camera(0.4, 0.45, (0.030, -0.020, z5))
        T8 = TA.pose_facing_camera(-0.7, 0.35, (-0.120, 0.050, 0.620))
        true5 = float(np.linalg.norm(T5[:3, 3])) * 1000.0
        img = cv2.cvtColor(TA.render_scene([("4X4_50", 5, T5), ("4X4_50", 8, T8)]), cv2.COLOR_GRAY2BGR)

        def tick(_):
            m = bridge.cv2_to_imgmsg(img, "bgr8")
            m.header.stamp, m.header.frame_id = rospy.Time.now(), "camera_optical_frame"
            pub_img.publish(m)
            info.header.stamp = m.header.stamp
            pub_info.publish(info)

        rospy.Timer(rospy.Duration(0.1), tick)
        rospy.set_param("/aruco_detector/marker_size", TA.SIZE)
        rospy.set_param("/aruco_detector/dictionary", "auto")
        start([os.path.join(PKG, "scripts/aruco_detector_node.py"), "__name:=aruco_detector"], "aruco_detector")
        got = {}
        rospy.Subscriber("/aruco_detector/markers", MarkerArray,
                         lambda m: got.update({(k.ns, k.id): k for k in m.markers if k.action == k.ADD}))
        got_img = {}
        rospy.Subscriber("/aruco_detector/image", Image, lambda m: got_img.update(size=(m.width, m.height)))
        wait_for(lambda: ("distance", 5) in got and ("distance", 8) in got and "size" in got_img, 40, "detector output")
        print("detector markers published: %s; annotated image %s" % (sorted({k for k in got}), got_img["size"]))
        print("rviz distance text for id 5: '%s'" % got[("distance", 5)].text)

        ruler = true5 + 3.0   # a "ruler" reading from the sensor surface, 3 mm behind the optical centre
        out = subprocess.run([os.path.join(PKG, "scripts/aruco_distance_report.py"), "--id", "5", "--measured", "%.1f" % ruler,
                              "--offset", "3.0", "--seconds", "4"], capture_output=True, text=True,
                             env=dict(os.environ, PYTHONPATH=os.path.join(PKG, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")))
        print(out.stdout)
        if out.returncode != 0:
            print("report failed:", out.stderr[-600:])
        out = out.stdout
        mean = float(re.search(r"mean ([\d.]+) mm", out).group(1))
        err = float(re.search(r"camera minus ruler: ([+-][\d.]+) mm", out).group(1))
        print("true optical-centre distance %.2f mm; report mean %.2f mm; reported error vs ruler-minus-offset %+.2f mm" % (true5, mean, err))
        ok &= abs(mean - true5) < 2.0 and abs(err) < 2.0 and "NOTE: std above" not in out
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
