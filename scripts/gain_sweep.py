#!/usr/bin/env python3
"""MOTOR INTEGRAL-GAIN SWEEP in free air (it WRITES THE POSITION I GAIN OF THE MOTORS and COMMANDS THE ARM through the grasp sequencer's probe mode).

For every I-gain value of --values (raw Dynamixel register units, applied to joints 1 to 5, never the gripper, RAM only):
    1. write the gain through /arm_interface/apply_position_gains (parameter /arm_interface/position_i_gain) and read it back,
    2. watch the arm holding the viewing pose for --watch seconds: abort on oscillation (position std above --max-std-deg), load or temperature,
    3. run the free-air probe on a grid of points (PRE, GRASP, LIFT measured, no corrective move, the gripper untouched),
    4. read the residual of every leg's first measurement from the sequencer's log (sag_log_dir) and print the table.
The gain is put back to 0 at the end and on any abort. Start the stack with the sag model OFF and the log in its own directory so the real sag log stays clean:

    roslaunch arm_bringup bringup_grasp.launch use_hardware:=true mission_control:=none detector:=false sag_gain:=0 sag_log_dir:=/home/dev/recordings/gain_test
    rosrun arm_grasp gain_sweep.py --azimuth 0 --log-dir /home/dev/recordings/gain_test            # dry runs only: lists the points, nothing moves, no gain written
    rosrun arm_grasp gain_sweep.py --azimuth 0 --log-dir /home/dev/recordings/gain_test --go       # the real sweep (e-stop in reach, the area under every point clear)
    ... --no-marker --floor-z -0.596       # without a camera: points at fixed base coordinates (start the stack with camera:=false target:=false)

The values are heuristic and ascend: the sweep stops at the first sign of trouble. The conversion of the register to a physical gain is not verified here.
"""
import argparse
import glob
import threading
import json
import os
import sys
import time

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger

import sag_test as st
from arm_grasp import grasp_plan as gp
from arm_grasp import sag_model as sm
from arm_grasp import target_offset as to
from arm_grasp.srv import Grasp
from arm_gui.srv import PresetCommand

ARM_JOINTS = 5


def set_i_gain(value):
    """Write the I gain `value` to joints 1 to 5 (-1 = leave the gripper alone). Returns (success, message)."""
    rospy.set_param("/arm_interface/position_i_gain", [int(value)] * ARM_JOINTS + [-1])
    rospy.wait_for_service("/arm_interface/apply_position_gains", 5.0)
    r = rospy.ServiceProxy("/arm_interface/apply_position_gains", Trigger)()
    return r.success, r.message


def hold_check(seconds, max_std_deg, max_load, max_temp):
    """Watch the held arm: (ok, text). Position std per arm joint, the largest |load| and the hottest motor."""
    samples = []
    sub = rospy.Subscriber("/joint_states_in", JointState, lambda m: samples.append(list(m.position[:ARM_JOINTS])), queue_size=200)
    t0 = time.time()
    worst_load, hot = 0.0, None
    while time.time() - t0 < seconds and not rospy.is_shutdown():
        rospy.sleep(0.5)
        try:
            from diagnostic_msgs.msg import DiagnosticArray
            msg = rospy.wait_for_message("/arm_interface/motor_status", DiagnosticArray, 2.0)
            for s in msg.status:
                kv = {v.key: v.value for v in s.values}
                if "load_percent" in kv and s.name.endswith(tuple("12345")):
                    worst_load = max(worst_load, abs(float(kv["load_percent"])))
                if "temperature_c" in kv:
                    t = float(kv["temperature_c"])
                    hot = t if hot is None else max(hot, t)
        except rospy.ROSException:
            pass
    sub.unregister()
    if len(samples) < 10:
        return False, "no joint states while holding"
    std = np.degrees(np.std(np.array(samples), axis=0))
    text = "hold: position std %s deg, worst load %.0f %%, hottest %s C" % (np.round(std, 2).tolist(), worst_load, "?" if hot is None else "%.0f" % hot)
    if float(std.max()) > max_std_deg:
        return False, text + " -> OSCILLATION (limit %.2f deg)" % max_std_deg
    if worst_load > max_load:
        return False, text + " -> LOAD above %.0f %%" % max_load
    if hot is not None and hot >= max_temp:
        return False, text + " -> TEMPERATURE at or above %.0f C" % max_temp
    return True, text


class FixedTarget(object):
    """--no-marker: publishes the `grasp_target` frame (base_link) itself, at a fixed point of the base frame, with the frame of the approach angle
    (target_offset.angled_frame) and the current time stamp, so the sequencer's freshness check passes. Replaces marker_0 + the grasp_target node."""

    def __init__(self, angle):
        self.angle, self.p, self.lock = angle, None, threading.Lock()
        self.br = tf2_ros.TransformBroadcaster()
        threading.Thread(target=self.loop, daemon=True).start()

    def set(self, p):
        with self.lock:
            self.p = np.array(p, float)

    def loop(self):
        while not rospy.is_shutdown():
            with self.lock:
                p = None if self.p is None else self.p.copy()
            if p is not None:
                m = TransformStamped()
                m.header.stamp, m.header.frame_id, m.child_frame_id = rospy.Time.now(), "base_link", "grasp_target"
                m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = [float(v) for v in p]
                q = gp.quaternion_from_matrix(to.angled_frame(p, self.angle))
                m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
                self.br.sendTransform(m)
            rospy.sleep(0.1)


def read_rows(log_dir, since):
    rows = []
    for f in sorted(glob.glob(os.path.join(log_dir, "sag_*.jsonl"))):
        for line in open(f):
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if r.get("time", 0) >= since and r.get("mode") == "probe" and int(r.get("step", 0)) == 0:
                rows.append(r)
    return rows


def residual(r):
    """measured - goal of one logged leg, in the arm frame at the goal's azimuth: [radial, tangential, vertical] mm."""
    g, m = np.asarray(r["goal"], float), np.asarray(r["measured"], float)
    az = np.arctan2(g[1], g[0])
    u, w = np.array([np.cos(az), np.sin(az), 0.0]), np.array([-np.sin(az), np.cos(az), 0.0])
    e = m - g
    return 1000.0 * np.array([e @ u, e @ w, e[2]])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--azimuth", type=float, required=True)
    ap.add_argument("--values", default="0,16,32,64,128,256,512,0", help="I-gain values in order; the last 0 repeats the baseline to show drift")
    ap.add_argument("--radii", default="0.46,0.52")
    ap.add_argument("--heights", default="0.07,0.09", help="grasp-point heights above the floor [m]")
    ap.add_argument("--angle", type=float, default=45.0)
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--preset", default="Pre-approach-New", help="the viewing pose between points (front)")
    ap.add_argument("--log-dir", default="/home/dev/recordings/gain_test", help="the sequencer's sag_log_dir")
    ap.add_argument("--out", default="", help="JSON results file (default: <log-dir>/gain_sweep_<time>.json)")
    ap.add_argument("--watch", type=float, default=4.0, help="seconds of holding watched after every gain change")
    ap.add_argument("--max-std-deg", type=float, default=0.3)
    ap.add_argument("--max-load", type=float, default=80.0)
    ap.add_argument("--max-temp", type=float, default=70.0)
    ap.add_argument("--no-marker", action="store_true", help="no marker_0 (no camera): the points are fixed in the base frame at --floor-z + height and this script publishes the target frame")
    ap.add_argument("--floor-z", type=float, default=-0.605, help="with --no-marker: the floor z in base_link [m] (the lab floor was measured at -0.596, -0.605 and -0.646)")
    ap.add_argument("--go", action="store_true", help="WRITE THE GAINS AND MOVE THE ARM (without it: dry runs only)")
    a, _ = ap.parse_known_args(rospy.myargv()[1:])
    values = [int(v) for v in a.values.split(",")]
    rospy.init_node("gain_sweep")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.wait_for_service("/grasp_sequencer/grasp", 10.0)
    grasp = rospy.ServiceProxy("/grasp_sequencer/grasp", Grasp)
    preset = rospy.ServiceProxy("/preset_manager/command", PresetCommand)
    if rospy.get_param("/grasp_sequencer/sag_gain", 1.0) != 0.0:
        print("the sag model is ON (sag_gain %s): start the stack with sag_gain:=0 for this test" % rospy.get_param("/grasp_sequencer/sag_gain", 1.0))
        return 1
    if a.go and os.path.realpath(rospy.get_param("/grasp_sequencer/sag_log_dir", "")) != os.path.realpath(a.log_dir):
        print("the sequencer logs to %s, not %s: start it with sag_log_dir:=%s" % (rospy.get_param("/grasp_sequencer/sag_log_dir", "?"), a.log_dir, a.log_dir))
        return 1
    fixed = FixedTarget(a.angle) if a.no_marker else None
    if not a.no_marker:
        m = st.wait_marker(buf, 10.0)
        if m is None:
            print("marker_0 is not in view: put the arm in the viewing pose first (or use --no-marker)")
            return 1
    floor_z = a.floor_z if a.no_marker else m[0][2]
    points = sm.grid_points(a.azimuth, [float(v) for v in a.radii.split(",")], [float(v) for v in a.heights.split(",")], floor_z)
    print("%d points x %d repeats x %d gain values; approach %.0f deg; %s" % (len(points), a.repeats, len(values), a.angle, "WRITING GAINS AND MOVING THE ARM" if a.go else "dry runs only"))
    results = []
    status = 0
    try:
        for value in values:
            if a.go:
                ok, msg = set_i_gain(value)
                print("\n== I gain %d: %s" % (value, msg[:160]))
                if not ok:
                    print("gain write failed: stopping"); status = 2; break
                ok, text = hold_check(a.watch, a.max_std_deg, a.max_load, a.max_temp)
                print("   " + text)
                if not ok:
                    status = 3; break
            t_start = time.time()
            for rep in range(a.repeats if a.go else 1):
                for r, h, p in points:
                    tag = "I %d r %.2f h %.2f rep %d" % (value, r, h, rep + 1)
                    if a.go:
                        res = preset(action="go", name=a.preset)
                        if not res.success:
                            print(tag, "cannot go to the viewing pose:", res.message); status = 4; raise StopIteration
                        rospy.sleep(2.0)
                        st.wait_state(("IDLE", "DONE", "ABORTED"), 5.0)
                    if fixed is not None:
                        fixed.set(p)
                        rospy.sleep(1.5)
                    else:
                        m = st.wait_marker(buf, 40.0)
                        if m is None:
                            print(tag, "marker_0 not seen: stopping"); status = 5; raise StopIteration
                        offset = to.vertical_frame(m[1]).T.dot(p - m[0])
                        rospy.set_param("/grasp_target/approach", "%g" % a.angle)
                        rospy.set_param("/grasp_target/offset", [float(v) for v in offset])
                        rospy.sleep(3.0)
                    dry = grasp(target_frame="grasp_target", dry_run=True, probe=False)
                    if not dry.success:
                        print("%-28s SKIPPED: %s" % (tag, dry.message[:100])); continue
                    if not a.go:
                        print("%-28s feasible" % tag); continue
                    res = grasp(target_frame="grasp_target", dry_run=False, probe=True)
                    if not res.success:
                        print("%-28s REFUSED: %s" % (tag, res.message[:100])); status = 6; raise StopIteration
                    last = st.wait_state(("DONE", "ABORTED"), 90.0)
                    print("%-28s %s" % (tag, last.get("state")))
                    if last.get("state") != "DONE":
                        status = 7; raise StopIteration
                    hot = st.hottest_motor()
                    if hot is not None and hot[1] >= a.max_temp:
                        print("   %s at %.0f C: stopping" % hot); status = 8; raise StopIteration
            rows = read_rows(a.log_dir, t_start - 1.0)
            by_leg = {}
            for r in rows:
                by_leg.setdefault(r["key"], []).append(residual(r))
            summary = {leg: {"n": len(v), "mean_mm": np.mean(v, axis=0).round(1).tolist(), "norm_mean_mm": round(float(np.mean(np.linalg.norm(v, axis=1))), 1),
                             "norm_max_mm": round(float(np.max(np.linalg.norm(v, axis=1))), 1)} for leg, v in by_leg.items()}
            results.append({"i_gain": value, "legs": summary})
            for leg in sm.KEYS:
                if leg in summary:
                    s = summary[leg]
                    print("   %-5s n %d  mean r/t/v %s mm  norm %.1f (max %.1f)" % (leg, s["n"], s["mean_mm"], s["norm_mean_mm"], s["norm_max_mm"]))
    except StopIteration:
        pass
    finally:
        if a.go:
            ok, msg = set_i_gain(0)
            print("\nI gain back to 0: %s" % msg[:160])
        out = a.out or os.path.join(a.log_dir, "gain_sweep_%s.json" % time.strftime("%Y%m%d_%H%M%S"))
        if a.go and results:
            os.makedirs(os.path.dirname(out), exist_ok=True)
            json.dump({"azimuth": a.azimuth, "angle": a.angle, "radii": a.radii, "heights": a.heights, "results": results, "status": status}, open(out, "w"), indent=1)
            print("results in", out)
    return status


if __name__ == "__main__":
    sys.exit(main())
