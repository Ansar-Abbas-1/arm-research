#!/usr/bin/env python3
"""End-to-end check of the ChArUco mount calibration WITHOUT a camera or arm: a synthetic camera publishes
rendered views of a ChArUco board lying on the ground, a simulated arm publishes TF, and the real
charuco_mount_calibration node (capture, solve) and charuco_mount_solve.py (offline re-solve) run against
them. Story: the board yaw is entered 2 degrees WRONG; the cross-check must expose it, and re-solving the
same stored views with the right yaw must remove the error. Own roscore on port 11312 (never 11311).

    python3 src/arm_camera/test/e2e_charuco_synthetic.py
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
import tf2_ros  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

import test_charuco as TC  # noqa: E402
from arm_camera import charuco  # noqa: E402
from arm_camera import mount_calibration as mc  # noqa: E402
from arm_camera.geometry import invert_transform, quat_from_matrix, rotation_error_deg  # noqa: E402

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
    work = tempfile.mkdtemp(prefix="arm_camera_e2e_charuco_")
    ok = True
    try:
        start(["roscore", "-p", str(PORT)], "roscore")
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        assert "/arm_interface" not in subprocess.check_output(["rosnode", "list"]).decode(), "arm_interface on this master"
        rospy.init_node("e2e_charuco")
        bridge = CvBridge()
        br = tf2_ros.TransformBroadcaster()
        pub_img = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1)
        info = CameraInfo()
        info.width, info.height, info.distortion_model = 1920, 1080, "plumb_bob"
        info.K = TC.K.reshape(-1).tolist()
        info.D = [0.0] * 5
        info.header.frame_id = "camera_optical_frame"
        state = {"T_be": TC.arm_pose(0.0, 2.3, 0.3, 0.4), "image": None}

        def tick(_):
            now = rospy.Time.now()
            m = TransformStamped()
            m.header.stamp, m.header.frame_id, m.child_frame_id = now, "base_link", "end_effector"
            Tm = state["T_be"]
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = Tm[:3, 3]
            q = quat_from_matrix(Tm[:3, :3])
            m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
            br.sendTransform(m)
            if state["image"] is not None:
                im = bridge.cv2_to_imgmsg(state["image"], "bgr8")
                im.header.stamp, im.header.frame_id = now, "camera_optical_frame"
                pub_img.publish(im)
                info.header.stamp = now
                pub_info.publish(info)

        rospy.Timer(rospy.Duration(0.05), tick)

        def set_pose(T_be):
            T_cb = invert_transform(T_be @ TC.X_TRUE) @ TC.MountFromKnownBoardTest.BOARD_TRUE
            state["image"] = cv2.cvtColor(charuco.render_board_view(TC.SPEC, T_cb, TC.K), cv2.COLOR_GRAY2BGR)
            state["T_be"] = T_be

        true_yaw = 12.0
        wrong_yaw = true_yaw + 2.0
        session = os.path.join(work, "session")
        out1 = os.path.join(work, "mount_wrong_yaw.yaml")
        s = TC.SPEC
        rospy.set_param("/charuco_mount_calibration/board", {"squares_x": s.squares_x, "squares_y": s.squares_y, "square_length": s.square_length,
                                                              "marker_length": s.marker_length, "dictionary": s.dictionary})
        rospy.set_param("/charuco_mount_calibration/board_pose", [0.34, 0.03, 0.0, wrong_yaw])
        rospy.set_param("/charuco_mount_calibration/board_origin", "center")
        rospy.set_param("/charuco_mount_calibration/session_dir", session)
        rospy.set_param("/charuco_mount_calibration/output_file", out1)
        start([os.path.join(PKG, "scripts/charuco_mount_calibration.py"), "__name:=charuco_mount_calibration"], "charuco_node")
        wait_for(lambda: "/charuco_mount_calibration/capture" in subprocess.check_output(["rosservice", "list"]).decode(), 30, "node services")
        capture = rospy.ServiceProxy("/charuco_mount_calibration/capture", Trigger)
        solve = rospy.ServiceProxy("/charuco_mount_calibration/solve", Trigger)

        poses = [a for a, _, _ in TC.MountSolveTest("test_handeye_method_needs_no_board_pose").partial_views(12, 21)]
        saved = 0
        for i, Tp in enumerate(poses):
            set_pose(Tp)
            time.sleep(1.5)
            r = capture()
            print("capture %2d: %s %s" % (i, "ok  " if r.success else "FAIL", r.message[:170]))
            saved += int(r.success)
        r = solve()
        print("\nsolve (board yaw entered %.0f deg, true %.0f): %s\n%s" % (wrong_yaw, true_yaw, "ok" if r.success else "FAIL", r.message))
        X1, d1 = mc.load_mount_yaml(out1)
        err1 = np.linalg.norm(X1[:3, 3] - TC.X_TRUE[:3, 3]) * 1000.0
        cross = d1.get("cross_check", {})
        print("mount error with the wrong yaw: %.1f mm, %.2f deg; cross-check says the board pose is off by %.2f deg"
              % (err1, rotation_error_deg(X1[:3, :3], TC.X_TRUE[:3, :3]), cross.get("board_pose_error_deg", float("nan"))))
        ok &= saved >= 10 and r.success and 0.5 < cross.get("board_pose_error_deg", 0.0) < 4.0

        out2 = os.path.join(work, "mount_fixed.yaml")
        res = subprocess.run([os.path.join(PKG, "scripts/charuco_mount_solve.py"), "--session", session, "--board-pose", "0.34", "0.03", "0.0",
                              str(true_yaw), "--origin", "center", "--out", out2], capture_output=True, text=True,
                             env=dict(os.environ, PYTHONPATH=os.path.join(PKG, "src") + os.pathsep + os.environ.get("PYTHONPATH", "")))
        print("\noffline re-solve with the corrected yaw:\n" + res.stdout)
        if res.returncode != 0:
            print("offline solve failed:", res.stderr[-500:])
        X2, _ = mc.load_mount_yaml(out2)
        err2 = np.linalg.norm(X2[:3, 3] - TC.X_TRUE[:3, 3]) * 1000.0
        rot2 = rotation_error_deg(X2[:3, :3], TC.X_TRUE[:3, :3])
        print("mount error after the correction: %.2f mm, %.3f deg  (before: %.1f mm)" % (err2, rot2, err1))
        ok &= err2 < 3.0 and rot2 < 0.8 and err2 < err1 and "static TF: roslaunch arm_camera camera_static_tf.launch" in res.stdout
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
        import shutil
        shutil.rmtree(work, ignore_errors=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
