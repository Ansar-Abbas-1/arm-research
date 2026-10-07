#!/usr/bin/env python3
"""Closed-loop check of the visual servo in SIMULATION on a private master (port 11312, never the real arm's 11311):
simulated arm (arm_sim_validator_node), a synthetic camera that renders the world markers from the arm's own TF,
the real marker_localizer and the real visual_servo node. Scenarios: converge on the CLOSEST 50 mm marker, e-stop
mid-run, marker lost, no target visible, no ground marker, operator stop. Starts its own roscore, kills only what it
started. Run with ROS sourced (after catkin_make):

    python3 src/arm_servo/test/e2e_sim_closed_loop.py
"""
import json
import os
import subprocess
import sys
import tempfile
import threading
import time

PORT = 11312
os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % PORT
HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(SRC, "arm_camera", "test"))

import numpy as np  # noqa: E402
import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

import test_aruco as TA  # noqa: E402
from arm_camera.geometry import invert_transform, make_transform, matrix_from_quat, rot_z  # noqa: E402

procs = []
FLOOR = -0.60
# world markers (base_link frame), lying flat on the floor, normal up: id -> (dictionary, size, x, y, yaw)
WORLD = {0: ("4X4_1000", 0.100, 0.46, 0.12, 0.3), 118: ("ARUCO_ORIGINAL", 0.050, 0.47, 0.02, 0.8), 94: ("ARUCO_ORIGINAL", 0.050, 0.38, -0.13, -0.5)}
PRE_APPROACH = [-0.0092, 1.8899, -1.2379, 2.7903, -1.7871]
visible = set(WORLD)


def T_world(mid):
    _, _, x, y, yaw = WORLD[mid]
    return make_transform(rot_z(yaw), [x, y, FLOOR])


def start(cmd, name):
    log = open(os.path.join(tempfile.gettempdir(), "arm_servo_e2e_%s.log" % name), "w")
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT)
    procs.append(p)
    return p


def wait_for(cond, timeout, what):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return
        time.sleep(0.1)
    raise RuntimeError("timeout waiting for " + what)


def render(T_base_cam):
    """Image of the visible world markers, grouped by size (render_scene draws every marker at TA.SIZE)."""
    img = np.full((1080, 1920), 190, np.uint8)
    for size in sorted({WORLD[m][1] for m in visible}, reverse=True):
        TA.SIZE = size
        layer = TA.render_scene([(WORLD[m][0], m, invert_transform(T_base_cam) @ T_world(m)) for m in sorted(visible) if WORLD[m][1] == size])
        img = np.where(np.abs(layer.astype(int) - 190) > 3, layer, img).astype(np.uint8)
    TA.SIZE = 0.100
    return img


def main():
    ok = True
    try:
        start(["roscore", "-p", str(PORT)], "roscore")
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        assert "/arm_interface" not in subprocess.check_output(["rosnode", "list"]).decode(), "arm_interface on this master"
        rospy.init_node("e2e_servo")
        start(["roslaunch", "arm_trajectory", "interactive_reach.launch", "use_hardware:=false", "use_rviz:=false", "presets:=false"], "sim")
        wait_for(lambda: "/arm_sim_validator_node" in subprocess.check_output(["rosnode", "list"]).decode(), 40, "simulated arm")
        start(["rosrun", "tf2_ros", "static_transform_publisher", "-0.025", "0", "-0.06", "1.5708", "0", "0", "end_effector", "camera_optical_frame"], "tf")
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        bridge = CvBridge()
        pub_img = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
        pub_info = rospy.Publisher("/usb_cam/camera_info", CameraInfo, queue_size=1, latch=True)
        info = CameraInfo(width=1920, height=1080, K=list(TA.K.reshape(-1)), D=[0.0] * 5, distortion_model="plumb_bob")
        info.header.frame_id = "camera_optical_frame"
        pub_info.publish(info)
        pub_out = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
        start(["roslaunch", "arm_perception", "marker_localizer.launch"], "localizer")
        start(["roslaunch", "arm_servo", "visual_servo.launch"], "servo")
        status = {"v": None}
        rospy.Subscriber("/visual_servo/status", String, lambda m: status.update(v=json.loads(m.data)), queue_size=5)
        servo_out = {"n": 0}
        joints = {"q": None}
        def on_out(m):
            servo_out["n"] += 1
        rospy.Subscriber("/joint_states_out", JointState, lambda m: on_out(m) if m.header.stamp.to_sec() > 0 else None, queue_size=50)
        rospy.Subscriber("/joint_states_in", JointState, lambda m: joints.update(q=dict(zip(m.name, m.position))), queue_size=1)
        stop = threading.Event()

        def camera_loop():
            while not stop.is_set():
                try:
                    t = buf.lookup_transform("base_link", "camera_optical_frame", rospy.Time(0), rospy.Duration(0.5)).transform
                    Tbc = make_transform(matrix_from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]), [t.translation.x, t.translation.y, t.translation.z])
                    msg = bridge.cv2_to_imgmsg(np.dstack([render(Tbc)] * 3), "bgr8")
                    msg.header.stamp, msg.header.frame_id = rospy.Time.now(), "camera_optical_frame"
                    pub_img.publish(msg)
                except Exception:
                    pass
                time.sleep(0.05)
        threading.Thread(target=camera_loop, daemon=True).start()
        srv_start = rospy.ServiceProxy("/visual_servo/start", Trigger)
        srv_stop = rospy.ServiceProxy("/visual_servo/stop", Trigger)
        estop = rospy.ServiceProxy("/arm_interface/estop", Trigger)
        estop_reset = rospy.ServiceProxy("/arm_interface/estop_reset", Trigger)
        wait_for(lambda: status["v"] is not None, 40, "servo status")

        def q5():
            return np.array([joints["q"]["joint_%d" % i] for i in range(1, 6)])

        def goto_pre_approach():
            t0 = time.time()
            while time.time() - t0 < 4.0:
                js = JointState()
                js.header.stamp = rospy.Time.now()
                js.name = ["joint_%d" % i for i in range(1, 6)]
                js.position = PRE_APPROACH
                pub_out.publish(js)
                time.sleep(0.05)
            time.sleep(2.0)

        def check(label, cond):
            nonlocal ok
            print("%-92s %s" % (label, "ok" if cond else "FAIL"))
            ok &= bool(cond)

        def state():
            return status["v"]["state"], status["v"]["reason"]

        def ground_ready():
            try:
                g = buf.lookup_transform("base_link", "marker_0", rospy.Time(0), rospy.Duration(0.1))
                return (rospy.Time.now() - g.header.stamp).to_sec() < 3.0
            except Exception:
                return False

        def to_idle():
            try:
                estop_reset()
            except Exception:
                pass
            srv_stop()
            time.sleep(0.5)

        goto_pre_approach()
        wait_for(ground_ready, 30, "the ground marker (id 0) localised from pre-approach")
        visible.discard(0)
        time.sleep(4.0)                                           # the TF of marker_0 ages out of the 3 s the check uses ...
        visible.add(0)

        print("-- scenario 1: converge on the closest 50 mm marker")
        goto_pre_approach()
        wait_for(ground_ready, 30, "ground marker")
        q_before = q5()
        r = srv_start()
        check("start accepted with the ground marker and 50 mm markers in view", r.success)
        wait_for(lambda: status["v"]["state"] == "SERVOING", 15, "SERVOING")
        check("the CLOSEST 50 mm marker (118) is the target, not 94", status["v"]["target_id"] == 118)
        wait_for(lambda: status["v"]["state"] in ("DONE", "ABORTED"), 90, "the run to end")
        check("the run ends DONE (state %s %s, left the view at the stand-off: %s)" % (state() + (status["v"]["left_view"],)), state()[0] == "DONE")
        tf_ee = buf.lookup_transform("base_link", "end_effector", rospy.Time(0), rospy.Duration(1.0)).transform
        Tbe = make_transform(matrix_from_quat([tf_ee.rotation.x, tf_ee.rotation.y, tf_ee.rotation.z, tf_ee.rotation.w]), [tf_ee.translation.x, tf_ee.translation.y, tf_ee.translation.z])
        p_e_true = (invert_transform(Tbe) @ np.append(T_world(118)[:3, 3], 1.0))[:3]
        err = np.linalg.norm(p_e_true - np.array([0.0, 0.0, 0.10]))
        check("the marker ends 10 cm ahead of the end effector on the tool axis (true error %.1f mm)" % (1000 * err), err < 0.012)
        down = -Tbe[:3, 2][2]
        check("the tool axis points down (angle from vertical %.1f deg)" % np.degrees(np.arccos(np.clip(down, -1, 1))), down > np.cos(np.radians(4.0)))
        check("the arm moved (it did not stay at the start)", np.linalg.norm(q5() - q_before) > 0.1)
        n_done = servo_out["n"]
        time.sleep(1.0)
        check("after DONE the node sends nothing more (the driver holds)", servo_out["n"] == n_done)

        print("-- scenario 2: e-stop in the middle of a run")
        to_idle(); goto_pre_approach(); wait_for(ground_ready, 30, "ground marker")
        srv_start(); wait_for(lambda: status["v"]["state"] == "SERVOING", 15, "SERVOING"); time.sleep(2.0)
        estop()
        wait_for(lambda: status["v"]["state"] == "ABORTED", 5, "abort on e-stop")
        check("e-stop aborts the run (reason: %s)" % state()[1], "e-stop" in state()[1])
        q_a = q5(); time.sleep(1.5)
        check("the arm stays where it was stopped", np.linalg.norm(q5() - q_a) < 0.003)

        print("-- scenario 3: the target marker disappears")
        to_idle(); goto_pre_approach(); wait_for(ground_ready, 30, "ground marker")
        srv_start(); wait_for(lambda: status["v"]["state"] == "SERVOING", 15, "SERVOING"); time.sleep(1.5)
        visible.discard(118)
        wait_for(lambda: status["v"]["state"] == "ABORTED", 8, "abort on lost marker")
        check("a lost marker aborts into hold (reason: %s)" % state()[1], "lost" in state()[1])
        n = servo_out["n"]; time.sleep(1.0)
        check("nothing more is commanded after the abort", servo_out["n"] == n)
        visible.add(118)

        print("-- scenario 4: no 50 mm marker visible at the start")
        to_idle(); goto_pre_approach(); wait_for(ground_ready, 30, "ground marker")
        visible.discard(118); visible.discard(94)
        n = servo_out["n"]
        srv_start(); time.sleep(0.4)
        wait_for(lambda: status["v"]["state"] == "ABORTED", 8, "abort: no target")
        check("with no target the run ends at once, as 'no target' (reason: %s)" % state()[1], "no 50 mm" in state()[1])
        check("and the arm was never commanded", servo_out["n"] == n)
        visible.update({118, 94})

        print("-- scenario 5: no ground marker")
        to_idle(); goto_pre_approach()
        visible.discard(0); time.sleep(13.0)                        # let marker_0's TF age beyond ground_max_age (10 s)
        r = None
        for _ in range(3):
            r = srv_start()
            if not r.success:
                break
            srv_stop(); time.sleep(0.5)
        check("start is refused without a recent ground marker (%s)" % (r.message[:70] if r else ""), r is not None and not r.success and "ground" in r.message)
        visible.add(0)

        print("-- scenario 6: the operator stops a run")
        to_idle(); goto_pre_approach(); wait_for(ground_ready, 30, "ground marker")
        srv_start(); wait_for(lambda: status["v"]["state"] == "SERVOING", 15, "SERVOING"); time.sleep(1.5)
        srv_stop()
        wait_for(lambda: status["v"]["state"] == "ABORTED", 3, "stop")
        check("~stop ends the run (reason: %s)" % state()[1], "operator" in state()[1])
        stop.set()
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):
            p.send_signal(2)
        for p in procs:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                p.kill()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
