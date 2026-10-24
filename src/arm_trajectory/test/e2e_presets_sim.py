#!/usr/bin/env python3
"""End-to-end check of the preset backend and the services behind the rviz Arm Control panel, against the
SIMULATED arm on a private master (port 11312, never the real arm's 11311). The panel's own widgets are covered
by arm_gui's rostest; this checks that what its buttons call works against the real nodes.

    source /opt/ros/noetic/setup.bash && source /home/dev/devel/setup.bash
    python3 src/arm_trajectory/test/e2e_presets_sim.py
Starts its own roscore and roslaunch and stops only those.
"""
import json
import os
import subprocess
import sys
import tempfile
import time

PORT = 11312
os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % PORT
HERE = os.path.dirname(os.path.abspath(__file__))

import rospy  # noqa: E402
import yaml  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from arm_gui.srv import PresetCommand  # noqa: E402

procs = []
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]


def start(cmd, name):
    log = open(os.path.join(tempfile.gettempdir(), "arm_trajectory_e2e_%s.log" % name), "w")
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
    user_file = os.path.join(tempfile.mkdtemp(), "presets.yaml")
    try:
        start(["roscore", "-p", str(PORT)], "roscore")
        wait_for(lambda: subprocess.call(["rosnode", "list"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0, 20, "roscore")
        rospy.init_node("e2e_presets")
        rospy.set_param("/preset_manager/presets_file", user_file)
        rospy.set_param("/arm_sim_validator_node/sim_motor_fault_joint", 4)     # simulated overload on joint_4
        rospy.set_param("/arm_sim_validator_node/sim_motor_warn_joint", 2)      # simulated 47 C on joint_2
        start(["roslaunch", "arm_trajectory", "interactive_reach.launch", "use_hardware:=false", "use_rviz:=false", "velocity_scale:=0.1"], "launch")
        wait_for(lambda: "/preset_manager" in subprocess.check_output(["rosnode", "list"]).decode(), 40, "preset_manager node")
        nodes = subprocess.check_output(["rosnode", "list"]).decode()
        assert "/arm_interface" not in nodes and "/arm_sim_validator_node" in nodes, "expected only the simulator on this master: " + nodes
        joints = {}
        rospy.Subscriber("/joint_states_in", JointState, lambda m: joints.update(dict(zip(m.name, m.position))), queue_size=1)
        estop = {"v": None}
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: estop.update(v=m.data), queue_size=1)
        drive = {"v": None}
        rospy.Subscriber("/arm_interface/drive_state", String, lambda m: drive.update(v=m.data), queue_size=1)
        state = {"v": None}
        rospy.Subscriber("/preset_manager/state", String, lambda m: state.update(v=json.loads(m.data)), queue_size=5)
        status = {"v": None}
        rospy.Subscriber("/arm_interface/motor_status", DiagnosticArray, lambda m: status.update(v=m.status), queue_size=1)
        for srv in ("/preset_manager/command", "/arm_interface/estop", "/arm_interface/estop_reset", "/arm_interface/set_torque",
                    "/arm_interface/reenable_failed_motors"):
            rospy.wait_for_service(srv, 30)
        wait_for(lambda: len(joints) >= 5 and state["v"] and estop["v"] is not None and status["v"], 30, "joint states, preset state, driver topics")
        cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), "config", "presets.yaml")))["presets"]
        preset = rospy.ServiceProxy("/preset_manager/command", PresetCommand)
        trigger = lambda name: rospy.ServiceProxy(name, Trigger)()
        set_torque = lambda on: rospy.ServiceProxy("/arm_interface/set_torque", SetBool)(on)

        def q():
            return [joints[j] for j in JOINTS]

        def near(goal, tol=0.02):
            return max(abs(a - b) for a, b in zip(q(), goal)) < tol

        def check(label, cond):
            nonlocal ok
            print("%-76s %s" % (label, "ok" if cond else "FAIL"))
            ok &= bool(cond)

        def go(name):
            return preset("go", name)

        def wait_idle():
            """The preset backend accepts the next move only once it has reported the last one."""
            wait_for(lambda: state["v"]["move"]["state"] != "moving", 60, "move finished")

        print("-- presets")
        check("state lists the shipped presets in order", [p["name"] for p in state["v"]["presets"]] == ["default", "ready", "pre_grasp"])
        r = go("nonexistent")
        check("an unknown preset is refused", not r.success and "unknown" in r.message)
        r = go("ready")
        check("go 'ready' is accepted", r.success)
        wait_for(lambda: state["v"]["move"]["state"] == "moving", 5, "move state moving")
        r2 = go("default")
        check("a second go while moving is refused", not r2.success and "already" in r2.message)
        wait_for(lambda: near(cfg["ready"]["joints"]), 40, "arm at 'ready'")
        wait_for(lambda: state["v"]["move"]["state"] == "done", 5, "move state done")
        check("the arm reaches 'ready' and the state reports done", near(cfg["ready"]["joints"]) and state["v"]["move"] == {"name": "ready", "state": "done", "message": ""})

        r = go("default")
        time.sleep(0.7)
        r2 = preset("cancel", "")
        check("cancel is accepted mid-move", r.success and r2.success)
        wait_for(lambda: state["v"]["move"]["state"] in ("failed", "done"), 15, "move ended after cancel")
        check("a cancelled move is reported failed and the arm did not reach 'default'",
              state["v"]["move"]["state"] == "failed" and not near(cfg["default"]["joints"], 0.01))
        go("default")
        wait_for(lambda: near(cfg["default"]["joints"]), 40, "arm at 'default'")
        wait_for(lambda: state["v"]["move"]["state"] == "done", 5, "done")

        before = q()
        r = preset("save", "my pose")
        saved = yaml.safe_load(open(user_file))["presets"]["my pose"]["joints"]
        check("save stores the current joints in the user file", r.success and max(abs(a - b) for a, b in zip(saved, before)) < 0.001)
        wait_for(lambda: "my pose" in [p["name"] for p in state["v"]["presets"]], 5, "state updated")
        check("the new preset appears in the state, flagged as the user's",
              [p for p in state["v"]["presets"] if p["name"] == "my pose"][0]["user"] is True)
        for bad in ("", "1x", "../x"):
            check("save refuses the name %r" % bad, not preset("save", bad).success)
        r = preset("save", "pre_grasp")
        check("saving over a shipped name overrides it", r.success and [p for p in state["v"]["presets"] if p["name"] == "pre_grasp"][0]["user"])
        r = preset("frobnicate", "x")
        check("an unknown action is refused", not r.success)

        print("\n-- e-stop")
        r = trigger("/arm_interface/estop")
        wait_for(lambda: estop["v"] is True, 10, "e-stop engaged")
        wait_for(lambda: drive["v"] == "ESTOPPED", 5, "drive state ESTOPPED")
        check("the e-stop service engages it", r.success and estop["v"] is True and drive["v"] == "ESTOPPED")
        hold = q()
        go("ready")
        time.sleep(3.0)
        wait_for(lambda: state["v"]["move"]["state"] in ("failed", "done"), 15, "move ended under e-stop")
        check("with the e-stop engaged a preset does not move the arm", near(hold, 0.01) and state["v"]["move"]["state"] == "failed")
        r = set_torque(False)
        check("releasing torque is refused while the e-stop is engaged", not r.success and drive["v"] == "ESTOPPED")
        r = trigger("/arm_interface/estop_reset")
        wait_for(lambda: estop["v"] is False, 10, "e-stop released")
        check("the reset service releases the e-stop", r.success and estop["v"] is False)
        time.sleep(1.0)
        go("default")
        wait_for(lambda: near(cfg["default"]["joints"]), 40, "arm at 'default' after reset")
        wait_idle()
        check("after the reset the preset moves again", near(cfg["default"]["joints"]) and state["v"]["move"]["state"] == "done")

        print("\n-- torque and motors")
        r = set_torque(False)
        wait_for(lambda: drive["v"] == "TORQUE_OFF", 8, "torque off")
        check("set_torque(false) releases torque", r.success and drive["v"] == "TORQUE_OFF")
        r = go("ready")
        check("go from torque-off is accepted", r.success)
        wait_for(lambda: near(cfg["ready"]["joints"]), 40, "arm at ready from torque off")
        check("a preset from torque-off re-enables torque and reaches the pose", near(cfg["ready"]["joints"]) and drive["v"] in ("HOLDING", "TRACKING"))
        wait_idle()
        set_torque(False)
        wait_for(lambda: drive["v"] == "TORQUE_OFF", 8, "torque off again")
        r = set_torque(True)
        wait_for(lambda: drive["v"] in ("HOLDING", "TRACKING"), 8, "torque on")
        check("set_torque(true) enables it again", r.success and drive["v"] in ("HOLDING", "TRACKING"))
        by = {s.name.split("/")[-1]: s for s in status["v"]}
        check("motor status shows the overload on joint_4 and the warning on joint_2", by["joint_4"].level == 2 and by["joint_2"].level == 1)
        r = trigger("/arm_interface/reenable_failed_motors")
        wait_for(lambda: all(s.level < 2 for s in status["v"]), 5, "fault cleared")
        check("the reset-failed-motors service clears the failed motor", r.success and all(s.level < 2 for s in status["v"]))
        r = trigger("/arm_interface/reenable_failed_motors")
        check("with no failed motor it reports that and succeeds", r.success and "No motor" in r.message)

        def cli(*args):
            r = subprocess.run([os.path.join(os.path.dirname(os.path.dirname(HERE)), "arm_interface", "scripts", "estop_cli.py")] + list(args),
                               capture_output=True, text=True, env=dict(os.environ))
            return r.returncode, r.stdout + r.stderr

        rc, out_ = cli("status")
        check("CLI status reports e-stop released and the motors OK/WARN", rc == 0 and "e-stop: released" in out_)
        rc, out_ = cli("motors")
        check("CLI motors prints the table with joint_2 at 47/50", rc == 0 and "47/50" in out_)
        rc, out_ = cli("torque", "off", "--yes")
        wait_for(lambda: drive["v"] == "TORQUE_OFF", 5, "torque off via CLI")
        check("CLI torque off releases torque", rc == 0 and drive["v"] == "TORQUE_OFF")
        rc, out_ = cli("torque", "on")
        wait_for(lambda: drive["v"] in ("HOLDING", "TRACKING"), 5, "torque on via CLI")
        check("CLI torque on enables it again", rc == 0 and drive["v"] in ("HOLDING", "TRACKING"))
        rc, out_ = cli("trigger")
        wait_for(lambda: estop["v"] is True, 5, "e-stop via CLI")
        rc2, out2 = cli("torque", "off", "--yes")
        check("CLI trigger engages the e-stop and torque off is then refused", rc == 0 and estop["v"] is True and rc2 == 1 and "REFUSED" in out2)
        rc, out_ = cli("reset", "--yes")
        wait_for(lambda: estop["v"] is False, 5, "reset via CLI")
        check("CLI reset releases the e-stop", rc == 0 and estop["v"] is False)
        print("RESULT:", "PASS" if ok else "FAIL")
    finally:
        for p in reversed(procs):
            p.send_signal(2)   # SIGINT: lets roslaunch stop its nodes
        for p in procs:
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                p.kill()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
