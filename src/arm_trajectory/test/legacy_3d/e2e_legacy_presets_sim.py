#!/usr/bin/env python3
"""LEGACY (3D marker version, not enabled by default). End-to-end check of the rviz preset buttons against the SIMULATED arm, on a private master (port 11312,
never the real arm's 11311): the same InteractiveMarkerFeedback messages rviz sends on a click are published,
and the arm (arm_sim_validator_node) must move only after the confirming second click.

    source /opt/ros/noetic/setup.bash && source /home/dev/devel/setup.bash
    python3 src/arm_trajectory/test/e2e_presets_sim.py
Starts its own roscore and roslaunch and stops only those.
"""
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
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from visualization_msgs.msg import InteractiveMarkerFeedback as FB  # noqa: E402
from visualization_msgs.msg import InteractiveMarkerUpdate, MarkerArray  # noqa: E402
from std_msgs.msg import String  # noqa: E402

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
        rospy.set_param("/preset_buttons/presets_file", user_file)
        rospy.set_param("/arm_sim_validator_node/sim_motor_fault_joint", 4)     # simulated overload on joint_4
        rospy.set_param("/arm_sim_validator_node/sim_motor_warn_joint", 2)      # simulated 47 C on joint_2
        start(["roslaunch", "arm_trajectory", "legacy_preset_markers.launch", "use_hardware:=false", "use_rviz:=false"], "launch")
        wait_for(lambda: "/preset_buttons" in subprocess.check_output(["rosnode", "list"]).decode(), 40, "preset_buttons node")
        nodes = subprocess.check_output(["rosnode", "list"]).decode()
        assert "/arm_interface" not in nodes and "/arm_sim_validator_node" in nodes, "expected only the simulator on this master: " + nodes
        joints = {}
        rospy.Subscriber("/joint_states_in", JointState, lambda m: joints.update(dict(zip(m.name, m.position))), queue_size=1)
        estop = {"v": None}
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: estop.update(v=m.data), queue_size=1)
        descriptions = []
        desc = {}

        def on_update(m):
            for x in m.markers:
                desc[x.name] = x.description
                descriptions.append(x.description)

        rospy.Subscriber("/preset_buttons/update", InteractiveMarkerUpdate, on_update, queue_size=50)
        report = {"rows": None}
        rospy.Subscriber("/preset_buttons/motor_report", MarkerArray,
                         lambda m: report.update(rows=[(k.text, k.color.r, k.color.g) for k in m.markers if k.action == k.ADD]), queue_size=5)
        drive = {"v": None}
        rospy.Subscriber("/arm_interface/drive_state", String, lambda m: drive.update(v=m.data), queue_size=1)
        pub = rospy.Publisher("/preset_buttons/feedback", FB, queue_size=10)
        wait_for(lambda: pub.get_num_connections() > 0 and len(joints) >= 5, 30, "preset feedback subscriber and joint states")
        time.sleep(1.0)
        cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.dirname(HERE)), "config", "presets.yaml")))["presets"]

        def q():
            return [joints[j] for j in JOINTS]

        def near(goal, tol=0.02):
            return max(abs(a - b) for a, b in zip(q(), goal)) < tol

        def click(name):
            m = FB(marker_name="preset_" + name, client_id="e2e", event_type=FB.BUTTON_CLICK)
            m.header.frame_id = "base_link"
            pub.publish(m)

        def menu(name, entry):
            m = FB(marker_name="preset_" + name, client_id="e2e", event_type=FB.MENU_SELECT, menu_entry_id=entry)
            m.header.frame_id = "base_link"
            pub.publish(m)

        def check(label, cond):
            nonlocal ok
            print("%-72s %s" % (label, "ok" if cond else "FAIL"))
            ok &= bool(cond)

        start_q = q()
        click("ready")
        time.sleep(1.5)
        check("one click only arms the button: the arm has not moved", near(start_q, 0.005))
        check("the armed button shows 'click again to GO'", any("click again" in d for d in descriptions))
        click("ready")
        wait_for(lambda: near(cfg["ready"]["joints"]), 40, "arm at 'ready'")
        check("second click within the window moves the arm to 'ready'", near(cfg["ready"]["joints"]))

        click("default")
        time.sleep(4.0)                       # longer than confirm_timeout (3 s)
        click("default")                       # a late second click arms again instead of confirming
        time.sleep(1.5)
        check("a late second click does not move the arm", near(cfg["ready"]["joints"], 0.005))
        click("default")
        wait_for(lambda: near(cfg["default"]["joints"]), 40, "arm at 'default'")
        check("armed again then confirmed: the arm goes to 'default'", near(cfg["default"]["joints"]))

        before = q()
        menu("pre_grasp", 2)                   # right-click menu: Overwrite with current pose
        time.sleep(1.0)
        saved = yaml.safe_load(open(user_file))["presets"]["pre_grasp"]["joints"]
        check("'Overwrite with current pose' saved the current joints", max(abs(a - b) for a, b in zip(saved, before)) < 0.001)

        menu("ready", 1)                       # right-click menu: Go to this pose (no second click needed)
        wait_for(lambda: near(cfg["ready"]["joints"]), 40, "arm at 'ready' via the menu")
        check("menu 'Go to this pose' moves at once", near(cfg["ready"]["joints"]))

        estop_click = FB(marker_name="preset_ESTOP", client_id="e2e", event_type=FB.BUTTON_CLICK)
        pub.publish(estop_click)
        wait_for(lambda: estop["v"] is True, 10, "e-stop engaged")
        check("the E-STOP button engages the e-stop", estop["v"] is True)
        hold = q()
        click("default")
        click("default")
        time.sleep(3.0)
        check("with the e-stop engaged a preset click does not move the arm", near(hold, 0.01))
        menu("ESTOP", 1)                       # Reset e-stop
        wait_for(lambda: estop["v"] is False, 10, "e-stop released")
        check("the E-STOP menu resets the e-stop", estop["v"] is False)
        time.sleep(1.0)
        click("default")
        click("default")
        wait_for(lambda: near(cfg["default"]["joints"]), 40, "arm at 'default' after reset")
        check("after the reset the preset moves again", near(cfg["default"]["joints"]))

        # ---- e-stop management, torque and the motor report -------------------------------------------------
        print("\n-- status, motor report, torque, reset button, CLI")
        wait_for(lambda: "preset_ESTOP_STATUS" in desc and report["rows"], 10, "status marker and motor report")
        check("status marker shows released and the motor ERROR from the simulated overload", "released" in desc["preset_ESTOP_STATUS"] and "ERROR" in desc["preset_ESTOP_STATUS"])
        check("status marker names the failing motor", "joint_4" in desc["preset_ESTOP_STATUS"] and "overload" in desc["preset_ESTOP_STATUS"])
        rows = report["rows"]
        check("motor report has a header and six rows", len(rows) == 7)
        row2 = [r for r in rows if "joint_2" in r[0]][0]
        row4 = [r for r in rows if "joint_4" in r[0]][0]
        check("joint_2 row shows 47/50 C and is yellow", "47/50 C" in row2[0] and row2[1] > 0.9 and 0.7 < row2[2] < 0.95)
        check("joint_4 row shows the overload and is red", "overload" in row4[0] and row4[1] > 0.9 and row4[2] < 0.4)
        click_motors = FB(marker_name="preset_MOTORS", client_id="e2e", event_type=FB.BUTTON_CLICK)
        pub.publish(click_motors)
        wait_for(lambda: report["rows"] == [], 5, "report hidden")
        check("the MOTORS button hides the report", report["rows"] == [])
        pub.publish(click_motors)
        wait_for(lambda: report["rows"] and len(report["rows"]) == 7, 5, "report shown")
        check("and shows it again", len(report["rows"]) == 7)

        def torque_click():
            pub.publish(FB(marker_name="preset_TORQUE", client_id="e2e", event_type=FB.BUTTON_CLICK))

        wait_for(lambda: drive["v"] == "HOLDING" and "TORQUE: ON" in desc.get("preset_TORQUE", ""), 10, "torque ON display")
        check("torque button shows ON while holding", "TORQUE: ON" in desc["preset_TORQUE"])
        torque_click()
        time.sleep(1.0)
        check("one torque click only arms it (the arm keeps its torque)", drive["v"] == "HOLDING" and "SAG" in desc["preset_TORQUE"])
        torque_click()
        wait_for(lambda: drive["v"] == "TORQUE_OFF", 8, "torque off")
        time.sleep(1.0)
        check("the second click releases torque", drive["v"] == "TORQUE_OFF" and "TORQUE: OFF" in desc["preset_TORQUE"])
        click("ready")
        click("ready")
        wait_for(lambda: near(cfg["ready"]["joints"]), 40, "arm at ready from torque off")
        check("a preset move from torque-off re-enables torque and reaches the pose", near(cfg["ready"]["joints"]) and drive["v"] in ("HOLDING", "TRACKING"))
        time.sleep(1.0)
        torque_click()
        torque_click()
        wait_for(lambda: drive["v"] == "TORQUE_OFF", 8, "torque off again")
        torque_click()
        torque_click()
        wait_for(lambda: drive["v"] in ("HOLDING", "TRACKING"), 8, "torque on")
        check("two more clicks enable torque again", drive["v"] in ("HOLDING", "TRACKING") and "TORQUE: ON" in desc["preset_TORQUE"])

        pub.publish(FB(marker_name="preset_ESTOP", client_id="e2e", event_type=FB.BUTTON_CLICK))
        wait_for(lambda: estop["v"] is True and drive["v"] == "ESTOPPED", 10, "e-stop engaged")
        time.sleep(1.0)
        check("status marker shows ENGAGED and the drive state ESTOPPED", "ENGAGED" in desc["preset_ESTOP_STATUS"] and "ESTOPPED" in desc["preset_ESTOP_STATUS"])
        torque_click()
        torque_click()
        time.sleep(1.5)
        check("releasing torque is refused while the e-stop is engaged", "REFUSED" in desc["preset_TORQUE"] and drive["v"] == "ESTOPPED")
        reset_click = FB(marker_name="preset_ESTOP_RESET", client_id="e2e", event_type=FB.BUTTON_CLICK)
        pub.publish(reset_click)
        time.sleep(1.0)
        check("one click on RESET only arms it", estop["v"] is True and "click again" in desc["preset_ESTOP_RESET"])
        pub.publish(reset_click)
        wait_for(lambda: estop["v"] is False, 8, "reset by the button")
        time.sleep(1.0)
        check("the second click releases the e-stop", estop["v"] is False and "released" in desc["preset_ESTOP_STATUS"])

        def cli(*args):
            r = subprocess.run([os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(HERE))), "arm_interface", "scripts", "estop_cli.py")] + list(args),
                               capture_output=True, text=True, env=dict(os.environ))
            return r.returncode, r.stdout + r.stderr

        rc, out_ = cli("status")
        check("CLI status reports e-stop released and the motor ERROR", rc == 0 and "e-stop: released" in out_ and "ERROR joint_4" in out_)
        rc, out_ = cli("motors")
        check("CLI motors prints the table with 0x20 for joint_4", rc == 0 and "0x20" in out_ and "47/50" in out_)
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
