#!/usr/bin/env python3
"""End-to-end check of the calibration nodes WITHOUT a camera or arm: a synthetic camera
publishes rendered chessboard images, a simulated arm publishes TF, and the real
hand_eye_collector (capture + solve), hand_eye_tf_publisher and calibration_overlay_node run
against them. Not part of the nose test collection (needs a ROS master). Run manually:

    source /opt/ros/noetic/setup.bash && source /home/dev/devel/setup.bash
    python3 src/arm_camera/test/e2e_synthetic.py

It starts its OWN roscore on port 11312 (never the default 11311, where the real arm's
master lives), and kills only the processes it started.
"""
import os
import shutil
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

import numpy as np  # noqa: E402
import rospy  # noqa: E402
import tf2_ros  # noqa: E402
import yaml  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from visualization_msgs.msg import MarkerArray  # noqa: E402

import test_calibration_pipeline as T  # noqa: E402
from arm_camera import session  # noqa: E402
from arm_camera.geometry import invert_transform, quat_from_matrix, rotation_error_deg  # noqa: E402

procs = []


def start(cmd, name=None):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.path.join(PKG, "src") + os.pathsep + env.get("PYTHONPATH", "")
    log = open(os.path.join(tempfile.gettempdir(), "arm_camera_e2e_%s.log" % os.path.basename(cmd[0])), "w")
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
    work = tempfile.mkdtemp(prefix="arm_camera_e2e_")
    ok = True
    try:
        # Guard: this must be OUR master, with no arm_interface on it.
        start(["roscore", "-p", str(PORT)])
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        out = subprocess.check_output(["rosnode", "list"]).decode()
        assert "/arm_interface" not in out, "an arm_interface node is on this master; refusing to continue"
        rospy.init_node("e2e_synthetic")
        bridge = CvBridge()
        br = tf2_ros.TransformBroadcaster()
        pub_img = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1)
        state = {"T_base_ee": T.arm_pose(0.0, 2.3, 0.3, 0.4), "image": None}

        info = CameraInfo()
        info.width, info.height = 1920, 1080
        info.K = T.K.reshape(-1).tolist()
        info.D = [0.0] * 5
        info.distortion_model = "plumb_bob"
        info.header.frame_id = "camera_optical_frame"

        def tick(_):
            now = rospy.Time.now()
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = now, "base_link", "end_effector"
            Tm = state["T_base_ee"]
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = Tm[:3, 3]
            q = quat_from_matrix(Tm[:3, :3])
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
            br.sendTransform(m)
            if state["image"] is not None:
                msg = bridge.cv2_to_imgmsg(state["image"], "bgr8")
                msg.header.stamp, msg.header.frame_id = now, "camera_optical_frame"
                pub_img.publish(msg)
                info.header.stamp = now
                pub_info.publish(info)

        rospy.Timer(rospy.Duration(0.05), tick)

        def set_pose(T_base_ee):
            T_cb = invert_transform(T_base_ee @ T.X_TRUE) @ T.B_TRUE
            gray = T.render_board(T_cb, black_parity=1)
            import cv2
            state["image"] = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            state["T_base_ee"] = T_base_ee

        # --- collector ------------------------------------------------------------------
        session_dir = os.path.join(work, "session")
        out_file = os.path.join(work, "hand_eye.yaml")
        rospy.set_param("/hand_eye_collector/board", {"cols": T.BOARD.cols, "rows": T.BOARD.rows, "square_size": T.BOARD.square_size})
        rospy.set_param("/hand_eye_collector/session_dir", session_dir)
        rospy.set_param("/hand_eye_collector/output_file", out_file)
        start([os.path.join(PKG, "scripts/hand_eye_collector.py"), "__name:=hand_eye_collector"])
        wait_for(lambda: "/hand_eye_collector/capture" in subprocess.check_output(["rosservice", "list"]).decode(), 30, "collector services")
        capture = rospy.ServiceProxy("/hand_eye_collector/capture", Trigger)
        solve = rospy.ServiceProxy("/hand_eye_collector/solve", Trigger)

        rng = np.random.RandomState(11)
        poses = []
        tries = 0
        while len(poses) < 14 and tries < 20000:
            tries += 1
            Tp = T.arm_pose(rng.uniform(-0.5, 0.5), rng.uniform(1.9, 2.7), rng.uniform(0.15, 0.40), rng.uniform(0.25, 0.50))
            T_cb = invert_transform(Tp @ T.X_TRUE) @ T.B_TRUE
            if not 0.25 < T_cb[2, 3] < 0.9 or T_cb[2, 2] < 0.5:   # distance, and the board's front facing the camera
                continue
            # The whole printed board (one square of border) must be inside the image.
            sq = T.BOARD.square_size
            frame = [(-sq, -sq, 0), (T.BOARD.cols * sq, -sq, 0), (T.BOARD.cols * sq, T.BOARD.rows * sq, 0), (-sq, T.BOARD.rows * sq, 0)]
            from arm_camera import board as board_mod
            px = board_mod.project(T.K, np.zeros(5), T_cb, frame)
            if px[:, 0].min() > 80 and px[:, 0].max() < 1840 and px[:, 1].min() > 80 and px[:, 1].max() < 1000:
                poses.append(Tp)
        print("%d visible poses found" % len(poses))
        saved = 0
        for i, Tp in enumerate(poses):
            set_pose(Tp)
            time.sleep(1.6)   # the collector wants the arm still for 0.6 s, and a fresh frame
            r = capture()
            print("capture %2d: %s %s" % (i, "ok  " if r.success else "FAIL", r.message))
            saved += int(r.success)
        r = solve()
        print("solve: %s %s" % ("ok" if r.success else "FAIL", r.message))
        ok &= saved >= 12 and r.success

        data = session.load_hand_eye(out_file)
        t_err = np.linalg.norm(data["X"][:3, 3] - T.X_TRUE[:3, 3]) * 1000.0
        r_err = rotation_error_deg(data["X"][:3, :3], T.X_TRUE[:3, :3])
        print("hand-eye result vs truth: translation error %.2f mm, rotation error %.3f deg (method %s)" % (t_err, r_err, data["raw"]["method"]))
        ok &= t_err < 5.0 and r_err < 0.8

        # --- overlay --------------------------------------------------------------------
        start([os.path.join(PKG, "scripts/hand_eye_tf_publisher.py"), "__name:=hand_eye_tf_publisher", "_file:=" + out_file])
        start([os.path.join(PKG, "scripts/calibration_overlay_node.py"), "__name:=calibration_overlay", "_hand_eye_file:=" + out_file])
        got = {}

        def on_markers(m):
            for mk in m.markers:
                if mk.ns == "rms":
                    got["text"] = mk.text
                got[mk.ns] = got.get(mk.ns, 0) + 1

        rospy.Subscriber("/arm_camera/markers", MarkerArray, on_markers)
        got_img = {}
        rospy.Subscriber("/calibration_overlay/image", Image, lambda m: got_img.update(size=(m.width, m.height)))
        set_pose(poses[3])
        wait_for(lambda: "reprojection" in got.get("text", "") and "size" in got_img, 40, "overlay output")
        print("overlay: %s; image %s; marker kinds %s" % (got["text"], got_img["size"], sorted(k for k in got if k != "text")))
        rms = float(got["text"].split("RMS")[1].split("px")[0])
        ok &= rms < 2.0 and {"board", "board_axes", "frustum", "rms"} <= set(got)
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):   # only the processes this script started
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
        shutil.rmtree(work, ignore_errors=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
