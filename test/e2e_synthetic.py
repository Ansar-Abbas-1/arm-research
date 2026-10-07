#!/usr/bin/env python3
"""End-to-end check of marker_localizer WITHOUT hardware: a synthetic camera publishes images of a rendered
ArUco marker whose pose in the base frame is fixed and known, TF carries a camera that is still, then moves, then
stops at another pose, and the marker finally disappears. Starts its OWN roscore on port 11312 (never the real
arm's 11311) and kills only what it started. Run with ROS sourced:

    python3 src/arm_perception/test/e2e_synthetic.py
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

PORT = 11312
os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % PORT
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
ARM_CAMERA = os.path.join(os.path.dirname(PKG), "arm_camera")
sys.path.insert(0, os.path.join(PKG, "src"))
sys.path.insert(0, os.path.join(ARM_CAMERA, "src"))
sys.path.insert(0, os.path.join(ARM_CAMERA, "test"))

import json  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

import test_aruco as TA  # noqa: E402
from arm_camera.geometry import invert_transform, make_transform, quat_from_matrix, rot_x, rot_y, rot_z, rotation_error_deg  # noqa: E402

procs = []
MARKERS_YAML = os.path.join(tempfile.gettempdir(), "arm_perception_e2e_markers.yaml")
with open(MARKERS_YAML, "w") as _f:
    _f.write("dictionary: 4X4_1000\nmarkers:\n  - {id: 0, size: 0.100}\n  - {id: 984, size: 0.050, dictionary: ARUCO_ORIGINAL}\n")
T_BASE_CAM_A = make_transform(rot_z(0.4) @ rot_y(0.9), [0.20, -0.05, 0.50])
T_CAM_MARKER_A = TA.pose_facing_camera(0.3, 0.4, (0.02, -0.01, 0.45))
T_BASE_MARKER = T_BASE_CAM_A @ T_CAM_MARKER_A                  # fixed in the world
T_CAM_SMALL_A = TA.pose_facing_camera(-0.5, 0.35, (-0.10, -0.02, 0.40))   # the 50 mm marker (id 984), left of the big one
T_BASE_SMALL = T_BASE_CAM_A @ T_CAM_SMALL_A
T_CAM_UNLISTED_A = TA.pose_facing_camera(0.8, 0.3, (0.21, 0.08, 0.45))    # id 5, not in the table
T_BASE_UNLISTED = T_BASE_CAM_A @ T_CAM_UNLISTED_A
REL = (rot_y(0.06) @ rot_x(-0.04), np.array([0.02, -0.01, 0.01]))   # camera A -> camera B, in the camera frame
T_STILL_A, T_MOVE, T_STILL_B, T_GONE = 3.5, 6.0, 9.5, 11.5          # phase ends [s]


def cam_pose(t):
    s = 0.0 if t < T_STILL_A else (1.0 if t >= T_MOVE else (t - T_STILL_A) / (T_MOVE - T_STILL_A))
    return T_BASE_CAM_A @ make_transform(rot_y(0.06 * s) @ rot_x(-0.04 * s), REL[1] * s)


def start(cmd, name):
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([os.path.join(PKG, "src"), os.path.join(ARM_CAMERA, "src"), env.get("PYTHONPATH", "")])
    log = open(os.path.join(tempfile.gettempdir(), "arm_perception_e2e_%s.log" % name), "w")
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
        rospy.init_node("e2e_marker")
        start([sys.executable, os.path.join(PKG, "scripts", "marker_localizer_node.py"), "_markers_file:=" + MARKERS_YAML], "node")
        reports, poses = [], []
        rospy.Subscriber("/marker_localizer/report", String, lambda m: reports.append((time.time(), json.loads(m.data))), queue_size=50)
        rospy.Subscriber("/marker_localizer/marker_0", PoseStamped, lambda m: poses.append((time.time(), m)), queue_size=50)
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        pub_img = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1, latch=True)
        info = CameraInfo(width=1920, height=1080, K=list(TA.K.reshape(-1)), D=[0.0] * 5, distortion_model="plumb_bob")
        info.header.frame_id = "camera_optical_frame"
        pub_info.publish(info)
        wait_for(lambda: pub_img.get_num_connections() > 0, 30, "localizer subscribed to the image")
        br = tf2_ros.TransformBroadcaster()
        bridge = CvBridge()
        t0 = time.time()
        stop = threading.Event()

        def tf_loop():
            while not stop.is_set():
                t = time.time() - t0
                T = cam_pose(t)
                m = TransformStamped()
                m.header.stamp, m.header.frame_id, m.child_frame_id = rospy.Time.now(), "base_link", "camera_optical_frame"
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = [float(v) for v in T[:3, 3]]
                q = quat_from_matrix(T[:3, :3])
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = [float(v) for v in q]
                br.sendTransform(m)
                time.sleep(0.01)

        threading.Thread(target=tf_loop, daemon=True).start()
        cli_out = {}

        def run_cli():
            r = subprocess.run([sys.executable, os.path.join(PKG, "scripts", "marker_pose_cli.py"), "--timeout", "4"],
                               capture_output=True, text=True, env=dict(os.environ, PYTHONPATH=os.pathsep.join([os.path.join(PKG, "src"), os.path.join(ARM_CAMERA, "src"), os.environ.get("PYTHONPATH", "")])))
            cli_out.update(rc=r.returncode, out=r.stdout + r.stderr)

        cli_started = False
        while time.time() - t0 < T_GONE + 0.5:
            t = time.time() - t0
            if T_MOVE + 1.0 < t and not cli_started:
                cli_started = True
                threading.Thread(target=run_cli, daemon=True).start()
            if t < T_STILL_B:
                inv = invert_transform(cam_pose(t))
                img = TA.render_scene([("4X4_1000", 0, inv @ T_BASE_MARKER), ("4X4_1000", 5, inv @ T_BASE_UNLISTED)])
                TA.SIZE = 0.050                                       # render_scene draws every marker at TA.SIZE
                small = TA.render_scene([("ARUCO_ORIGINAL", 984, inv @ T_BASE_SMALL)])
                TA.SIZE = 0.100
                img = np.where(np.abs(small.astype(int) - 190) > 3, small, img).astype(np.uint8)
            else:
                img = np.full((1080, 1920), 190, np.uint8)         # the marker has gone
            msg = bridge.cv2_to_imgmsg(np.dstack([img] * 3), "bgr8")
            msg.header.stamp, msg.header.frame_id = rospy.Time.now(), "camera_optical_frame"
            pub_img.publish(msg)
            time.sleep(0.07)
        stop.set()

        def check(label, cond):
            nonlocal ok
            print("%-84s %s" % (label, "ok" if cond else "FAIL"))
            ok &= bool(cond)

        def valid_in(a, b, mid=0):
            return [m for (tw, r) in reports if t0 + a <= tw <= t0 + b for m in r["markers"] if m["valid"] and m["id"] == mid]

        def error(m):
            pos = 1000 * np.linalg.norm(np.array(m["position"]) - T_BASE_MARKER[:3, 3])
            from arm_camera.geometry import matrix_from_quat
            return pos, rotation_error_deg(matrix_from_quat(m["quaternion_xyzw"]), T_BASE_MARKER[:3, :3])

        print("-- still camera, pose A")
        va = valid_in(1.5, T_STILL_A)
        check("the marker is reported once the camera has been still", len(va) > 0)
        if va:
            e = [error(m) for m in va]
            check("position within 4 mm and orientation within 3 deg of the truth (worst %.1f mm, %.1f deg)" % (max(x[0] for x in e), max(x[1] for x in e)),
                  max(x[0] for x in e) < 4.0 and max(x[1] for x in e) < 3.0)
            check("the spread is reported (std x %.2f mm, %d samples)" % (va[-1]["std_mm"][0], va[-1]["n"]), va[-1]["n"] >= 3 and va[-1]["std_mm"][0] < 3.0)
        v984 = [m for (tw, r) in reports if t0 + 1.5 <= tw <= t0 + T_STILL_A for m in r["markers"] if m["id"] == 984 and m["valid"]]
        check("the 50 mm marker (id 984) is reported too, with its own size", bool(v984) and v984[-1].get("size") == 0.050)
        if v984:
            err984 = 1000 * np.linalg.norm(np.array(v984[-1]["position"]) - T_BASE_SMALL[:3, 3])
            check("its position is right within 6 mm using the 50 mm size (%.1f mm)" % err984, err984 < 6.0)
        seen = [r for (tw, r) in reports if t0 + 1.5 <= tw <= t0 + T_STILL_A]
        check("the unlisted marker is never reported but is named under ignored",
              seen and not any(m["id"] == 5 for r in seen for m in r["markers"]) and any(i["id"] == 5 for r in seen for i in r["ignored"]))
        print("-- camera moving")
        a, b = T_STILL_A + 0.8, T_MOVE - 0.2
        during = [(tw, r) for (tw, r) in reports if t0 + a <= tw <= t0 + b]
        check("no valid marker is reported while the camera moves", len(during) > 0 and not any(m["valid"] for _, r in during for m in r["markers"]))
        check("the report says the camera is moving", any(r["camera_moving"] for _, r in during))
        check("no PoseStamped is published while it moves", not [1 for (tw, _) in poses if t0 + a <= tw <= t0 + b])
        print("-- still camera, pose B (the marker has not moved in the world)")
        vb = valid_in(T_MOVE + 1.0, T_STILL_B - 0.3)
        check("the marker is reported again", len(vb) > 0)
        if vb:
            e = [error(m) for m in vb]
            check("same world pose from the new viewpoint (worst %.1f mm, %.1f deg)" % (max(x[0] for x in e), max(x[1] for x in e)),
                  max(x[0] for x in e) < 4.0 and max(x[1] for x in e) < 3.0)
        try:
            tf = buf.lookup_transform("base_link", "marker_0", rospy.Time(0), rospy.Duration(1.0)).transform.translation
            d = 1000 * np.linalg.norm([tf.x - T_BASE_MARKER[0, 3], tf.y - T_BASE_MARKER[1, 3], tf.z - T_BASE_MARKER[2, 3]])
            check("TF base_link -> marker_0 is published and matches (%.1f mm)" % d, d < 4.0)
        except Exception as ex:
            check("TF base_link -> marker_0 is published (%s)" % ex, False)
        check("PoseStamped in base_link is published", bool(poses) and poses[-1][1].header.frame_id == "base_link")
        print("-- CLI")
        wait_for(lambda: "rc" in cli_out, 8, "cli")
        check("marker_pose_cli prints the pose and exits 0", cli_out.get("rc") == 0 and "marker 0 (100 mm):" in cli_out.get("out", ""))
        print("-- marker gone")
        check("a vanished marker disappears from the report", reports and reports[-1][1]["markers"] == [])
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):
            p.send_signal(2)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
