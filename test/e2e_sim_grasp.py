#!/usr/bin/env python3
"""Closed-loop check of the grasp sequence in SIMULATION on a private master (port 11312, never the real arm's 11311):
simulated arm + the real planner and executor + the real grasp_sequencer_node, with a fake target TF. Scenarios: dry
run, unreachable target, a full grasp (open, above, descend, close, lift), e-stop in the descent, stale target, target
moved after planning, operator stop. Starts its own roscore, kills only what it started. After catkin_make:

    python3 src/arm_grasp/test/e2e_sim_grasp.py

To watch it: E2E_VISUAL=1 (needs X access from the container, `xhost +local:` on the host) also opens rviz and
rqt_py_trees on the private master and runs the simulated arm at velocity_scale 0.1 so the motion is watchable;
E2E_START_DELAY (default 25 s) is the time to arrange the windows before the scenarios start.
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

import numpy as np  # noqa: E402
import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from arm_grasp.srv import Grasp  # noqa: E402

procs = []
VISUAL = os.environ.get("E2E_VISUAL", "") not in ("", "0")
PRE_APPROACH = [-0.0092, 1.8899, -1.2379, 2.7903, -1.7871]
TARGET = {"x": 0.47, "y": 0.04, "z": float(os.environ.get("TARGET_Z", "-0.45")), "yaw": 0.5, "publish": True, "dz": 0.0}
SPEC = {"pre": 0.10, "grasp": 0.02, "lift": 0.15}


def start(cmd, name):
    log = open(os.path.join(tempfile.gettempdir(), "arm_grasp_e2e_%s.log" % name), "w")
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


def main():
    ok = True
    try:
        start(["roscore", "-p", str(PORT)], "roscore")
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        assert "/arm_interface" not in subprocess.check_output(["rosnode", "list"]).decode(), "arm_interface on this master"
        rospy.init_node("e2e_grasp")
        start(["roslaunch", "arm_trajectory", "interactive_reach.launch", "use_hardware:=false", "use_rviz:=%s" % ("true" if VISUAL else "false"),
               "presets:=false"] + (["velocity_scale:=0.1"] if VISUAL else []), "sim")
        wait_for(lambda: "/trajectory_executor_node" in subprocess.check_output(["rosnode", "list"]).decode(), 40, "the planner stack")
        start(["roslaunch", "arm_grasp", "gripper_control.launch"], "gripper")
        start(["roslaunch", "arm_grasp", "grasp.launch"], "grasp")
        wait_for(lambda: "/gripper_control/open" in subprocess.check_output(["rosservice", "list"]).decode(), 40, "the gripper_control services")
        wait_for(lambda: "/grasp_sequencer/grasp" in subprocess.check_output(["rosservice", "list"]).decode(), 40, "the grasp service")
        if VISUAL:
            start(["rqt_py_trees"], "rqt")
            delay = float(os.environ.get("E2E_START_DELAY", "25"))
            print("visual mode: rviz and rqt_py_trees are opening on the private master (port %d); the scenarios start in %.0f s" % (PORT, delay), flush=True)
            time.sleep(delay)
        br = tf2_ros.TransformBroadcaster()
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        stop = threading.Event()

        def target_loop():
            while not stop.is_set():
                if TARGET["publish"]:
                    c, s = np.cos(TARGET["yaw"] / 2), np.sin(TARGET["yaw"] / 2)
                    m = TransformStamped()
                    m.header.stamp, m.header.frame_id, m.child_frame_id = rospy.Time.now(), "base_link", "target"
                    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = TARGET["x"], TARGET["y"], TARGET["z"] + TARGET["dz"]
                    m.transform.rotation.z, m.transform.rotation.w = float(s), float(c)
                    br.sendTransform(m)
                time.sleep(0.05)
        threading.Thread(target=target_loop, daemon=True).start()

        status = {"v": None}
        history = []                                         # (time, state)
        rospy.Subscriber("/grasp_sequencer/status", String, lambda m: (status.update(v=json.loads(m.data)), history.append((time.time(), json.loads(m.data)["state"]))), queue_size=50)
        joints = {"q": None, "g": []}
        def on_joints(m):
            d = dict(zip(m.name, m.position))
            joints["q"] = np.array([d["joint_%d" % i] for i in range(1, 6)])
            if "joint_6" in d:
                joints["g"].append((time.time(), d["joint_6"]))
        rospy.Subscriber("/joint_states_in", JointState, on_joints, queue_size=1)
        pub_out = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
        grasp = rospy.ServiceProxy("/grasp_sequencer/grasp", Grasp)
        stop_srv = rospy.ServiceProxy("/grasp_sequencer/stop", Trigger)
        # The simulator has NO gripper load model (load behaviour is hardware only). This is a constant low-load STUB so the
        # sequencing checks can run: the close ends "fully closed below the target" ('thin'). Nothing here says anything
        # about real load behaviour.
        load_pub = rospy.Publisher("/arm_interface/motor_load", JointState, queue_size=1)

        def load_stub():
            while not stop.is_set():
                m = JointState(); m.header.stamp = rospy.Time.now(); m.name, m.effort = ["joint_6"], [2.5]
                load_pub.publish(m)
                time.sleep(0.1)
        threading.Thread(target=load_stub, daemon=True).start()
        estop = rospy.ServiceProxy("/arm_interface/estop", Trigger)
        estop_reset = rospy.ServiceProxy("/arm_interface/estop_reset", Trigger)
        wait_for(lambda: joints["q"] is not None and status["v"] is not None, 30, "joint states and status")
        trace = []                                           # (time, ee position)

        def ee():
            t = buf.lookup_transform("base_link", "end_effector", rospy.Time(0), rospy.Duration(0.5)).transform.translation
            return np.array([t.x, t.y, t.z])

        def tracer():
            while not stop.is_set():
                try:
                    trace.append((time.time(), ee()))
                except Exception:
                    pass
                time.sleep(0.05)
        threading.Thread(target=tracer, daemon=True).start()

        def goto_start():
            t0 = time.time()
            while time.time() - t0 < 4.0:
                js = JointState(); js.header.stamp = rospy.Time.now(); js.name = ["joint_%d" % i for i in range(1, 6)] + ["joint_6"]
                js.position = PRE_APPROACH + [0.0]
                pub_out.publish(js); time.sleep(0.05)
            time.sleep(1.0)

        def check(label, cond):
            nonlocal ok
            print("%-96s %s" % (label, "ok" if cond else "FAIL"))
            ok &= bool(cond)

        def reset():
            try:
                estop_reset()
            except Exception:
                pass
            TARGET.update(publish=True, dz=0.0, x=0.47, y=0.04)
            stop_srv(); time.sleep(0.5)

        def state():
            return status["v"]["state"], status["v"]["message"]

        def expected(h):
            return np.array([TARGET["x"], TARGET["y"], TARGET["z"] + h])

        print("-- scenario 1: dry run (nothing moves)")
        goto_start()
        q0 = joints["q"].copy()
        g0 = joints["g"][-1][1]
        r = grasp("target", True)
        print("   " + r.message[:200])
        check("the plan is reported feasible (z target %.2f)" % TARGET["z"], r.success)
        check("with the planned durations", r.plan_duration_s > 5.0 and r.max_cartesian_deviation_m < 0.005)
        time.sleep(1.0)
        check("the arm and the gripper did not move", np.linalg.norm(joints["q"] - q0) < 1e-3 and abs(joints["g"][-1][1] - g0) < 1e-6)
        if not r.success:
            raise RuntimeError("the test target is not reachable: " + r.message)

        print("-- scenario 2: an unreachable target is refused before anything moves")
        TARGET.update(x=1.6)
        time.sleep(0.5)
        r2 = grasp("target", True)
        check("dry run: not feasible (%s)" % r2.message[:80], not r2.success and "not reachable" in r2.message)
        r3 = grasp("target", False)
        check("a real run is refused too and the arm stays put", not r3.success and np.linalg.norm(joints["q"] - q0) < 1e-3)
        reset()

        print("-- scenario 3: a full grasp")
        goto_start()
        trace.clear(); joints["g"].clear(); history.clear()
        r = grasp("target", False)
        check("the sequence is accepted (%s)" % r.message[:60], r.success)
        paths = set()
        rospy.Subscriber("/grasp_sequencer/status", String, lambda m: paths.add(json.loads(m.data).get("active_path", "")), queue_size=50)
        wait_for(lambda: status["v"]["state"] in ("DONE", "ABORTED"), 240, "the sequence to end")
        check("the status carries the tree's active path (%s)" % sorted(p for p in paths if "DESCEND" in p), any(p.endswith("grasp/DESCEND") for p in paths) and any(p.endswith("grasp/open gripper") for p in paths))
        check("the tree is published for rqt_py_trees on ~log/tree", "/grasp_sequencer/log/tree" in subprocess.check_output(["rostopic", "list"]).decode())
        check("it ends DONE (%s %s)" % state(), state()[0] == "DONE")
        check("and the gripper node still holds the grasp afterwards, regulating (grip %s)" % status["v"]["gripper"].get("grip"), status["v"]["gripper"].get("grip") in ("thin", "held"))
        seen = []
        for _, st in history:
            if not seen or seen[-1] != st:
                seen.append(st)
        check("through OPENING, MOVING_PRE, CORRECTING, RECHECK, DESCENDING, CORRECTING, CLOSING, LIFTING, DONE (no sag in the sim: no re-planning) %s" % [s for s in seen if s not in ("PLANNING", "IDLE")],
              [s for s in seen if s not in ("PLANNING", "IDLE")] == ["OPENING", "MOVING_PRE", "CORRECTING", "RECHECK", "DESCENDING", "CORRECTING", "CLOSING", "LIFTING", "DONE"])
        pts = np.array([p for _, p in trace])
        zmin = pts[:, 2].min()
        i = int(np.argmin(pts[:, 2]))
        check("the lowest point is the grasp height above the target (z %.3f, expected %.3f)" % (zmin, expected(SPEC["grasp"])[2]), abs(zmin - expected(SPEC["grasp"])[2]) < 0.008)
        check("and it is straight above the target (xy error %.1f mm)" % (1000 * np.linalg.norm(pts[i, :2] - expected(0)[:2])), np.linalg.norm(pts[i, :2] - expected(0)[:2]) < 0.008)
        check("the final pose is the lift height (error %.1f mm)" % (1000 * np.linalg.norm(pts[-1] - expected(SPEC["lift"]))), np.linalg.norm(pts[-1] - expected(SPEC["lift"])) < 0.010)
        g = np.array([v for _, v in joints["g"]])
        check("the gripper opened and closed (max %.3f, final %.3f)" % (g.max(), g[-1]), g.max() > 0.024 and g[-1] < 0.002)
        check("fully closed below the load target (stub load) = 'thin' (%s)" % status["v"]["plan"].get("grasp_outcome"), status["v"]["plan"].get("grasp_outcome") == "thin")
        t_desc = [t for t, s in history if s == "DESCENDING"]
        t_close = [t for t, s in history if s == "CLOSING"]
        planned = [l for l in status["v"]["plan"]["legs"] if l["name"] == "DESCEND"][0]["duration"]
        dur = t_close[0] - t_desc[0]
        check("the descent was slowed: %.1f s against %.1f s planned (x3)" % (dur, planned), dur > 2.0 * planned)

        print("-- scenario 4: e-stop in the descent")
        reset(); goto_start()
        joints["g"].clear()
        grasp("target", False)
        wait_for(lambda: status["v"]["state"] == "DESCENDING", 120, "DESCENDING")
        wait_for(lambda: trace and trace[-1][1][2] < expected(SPEC["pre"])[2] - 0.02, 5, "2 cm of descent")   # the slowed descent is short
        estop()
        wait_for(lambda: status["v"]["state"] == "ABORTED", 10, "abort")
        check("the e-stop aborts in the descent (%s)" % state()[1], "e-stop" in state()[1] and status["v"]["state"] == "ABORTED")
        q_a = joints["q"].copy(); time.sleep(1.5)
        check("the arm stays where it stopped", np.linalg.norm(joints["q"] - q_a) < 0.003)
        gv = np.array([v for _, v in joints["g"]])
        i_open = int(np.argmax(gv > 0.024))                   # first sample at "open"; before it the gripper ramps up from closed
        check("the gripper opened and was never closed afterwards (min %.3f)" % gv[i_open:].min(), gv.max() > 0.024 and gv[i_open:].min() > 0.02)

        print("-- scenario 5: a stale target is refused")
        reset(); goto_start()
        TARGET["publish"] = False
        time.sleep(6.5)
        r = grasp("target", True)
        check("dry run: refused as stale (%s)" % r.message[:70], not r.success and "stale" in r.message)
        r = grasp("target", False)
        check("a real run is refused too", not r.success and "stale" in r.message)

        print("-- scenario 6: the target moves after planning")
        reset(); goto_start()
        trace.clear()
        grasp("target", False)
        wait_for(lambda: status["v"]["state"] == "MOVING_PRE", 30, "MOVING_PRE")
        TARGET["dz"] = 0.02                                   # 2 cm lower, with a fresh stamp
        wait_for(lambda: status["v"]["state"] in ("ABORTED", "DESCENDING", "DONE"), 120, "RECHECK outcome")
        check("the sequence aborts before descending (%s)" % state()[1], state()[0] == "ABORTED" and "moved" in state()[1])
        pts = np.array([p for _, p in trace])
        check("and it never went below the pre-grasp height (lowest z %.3f, pre %.3f)" % (pts[:, 2].min(), expected(SPEC["pre"])[2]), pts[:, 2].min() > expected(SPEC["pre"])[2] - 0.01)

        print("-- scenario 7: the operator stops a run")
        reset(); goto_start()
        grasp("target", False)
        wait_for(lambda: status["v"]["state"] == "MOVING_PRE", 30, "MOVING_PRE")
        time.sleep(1.0)
        stop_srv()
        wait_for(lambda: status["v"]["state"] == "ABORTED", 10, "stop")
        check("~stop ends the run (%s)" % state()[1], "operator" in state()[1])
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
