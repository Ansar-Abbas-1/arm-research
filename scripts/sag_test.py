#!/usr/bin/env python3
"""The FREE-AIR SAG TEST: moves the arm through a grid of points in free air and logs how far it sags (it COMMANDS THE ARM through the grasp sequencer).

For every point of the grid (radius x height at one azimuth, relative to the base, see sag_model.grid_points) and repeat:
    1. go back to the viewing preset (preset_manager) so marker_0 is seen and the target is fresh,
    2. set the grasp_target (marker_0 + offset, upright, approach angle) on the free-air point,
    3. dry-run it; if feasible, run the grasp sequencer as a PROBE: PRE, GRASP and LIFT are moved to and measured, the gripper is never commanded, nothing is gripped.
The sequencer logs every measurement (sag_log_dir); fit them with scripts/sag_fit.py.

    rosrun arm_grasp sag_test.py --azimuth -40                  # dry runs only: lists the points and whether the planner can reach them, nothing moves
    rosrun arm_grasp sag_test.py --azimuth -40 --go             # the real test (e-stop in reach, the area under every point clear)

The area around every point must be free (the grasp point is a point in the air, PRE is 10 cm up and back along the approach line, the arm sweeps there).
"""
import argparse
import sys
import time

import numpy as np
import rospy
import tf2_ros
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from arm_grasp import sag_model as sm
from arm_grasp import target_offset as to
from arm_grasp.srv import Grasp
from arm_gui.srv import PresetCommand


def marker_pose(buf, max_age):
    t = buf.lookup_transform("base_link", "marker_0", rospy.Time(0), rospy.Duration(1.0))
    if (rospy.Time.now() - t.header.stamp).to_sec() > max_age:
        raise tf2_ros.ExtrapolationException("marker_0 is %.1f s old" % (rospy.Time.now() - t.header.stamp).to_sec())
    q = t.transform.rotation
    p = t.transform.translation
    return np.array([p.x, p.y, p.z]), [q.x, q.y, q.z, q.w]


def wait_marker(buf, timeout, max_age=1.5):
    t0 = time.time()
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        try:
            return marker_pose(buf, max_age)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.3)
    return None


def wait_state(done, timeout):
    state = {"s": ""}
    sub = rospy.Subscriber("/grasp_sequencer/status", String, lambda m: state.update(s=m.data), queue_size=1)
    import json
    t0 = time.time()
    last = {}
    while time.time() - t0 < timeout and not rospy.is_shutdown():
        try:
            last = json.loads(state["s"]) if state["s"] else {}
        except ValueError:
            last = {}
        if last.get("state") in done:
            break
        rospy.sleep(0.5)
    sub.unregister()
    return last


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--azimuth", type=float, required=True, help="degrees from base +x: a direction with free space under the points (no default: it depends on the scene)")
    ap.add_argument("--radii", default="0.40,0.46,0.52", help="horizontal distances from the base axis [m]")
    ap.add_argument("--heights", default="0.12,0.20", help="grasp-point heights above the floor [m]: keep them above anything in the way")
    ap.add_argument("--angle", type=float, default=45.0, help="approach angle below the horizontal (90 = from above)")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--preset", default="Pre-approach-New", help="the viewing pose to return to between points")
    ap.add_argument("--go", action="store_true", help="MOVE THE ARM (without it: dry runs only)")
    a, _ = ap.parse_known_args(rospy.myargv()[1:])
    rospy.init_node("sag_test")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.wait_for_service("/grasp_sequencer/grasp", 10.0)
    grasp = rospy.ServiceProxy("/grasp_sequencer/grasp", Grasp)
    preset = rospy.ServiceProxy("/preset_manager/command", PresetCommand)
    m = wait_marker(buf, 10.0)
    if m is None:
        print("marker_0 is not in view: put the arm in the viewing pose first")
        return 1
    floor_z = m[0][2]
    points = sm.grid_points(a.azimuth, [float(v) for v in a.radii.split(",")], [float(v) for v in a.heights.split(",")], floor_z)
    print("floor z %.3f (marker_0); %d points x %d repeats; approach %.0f deg; %s" % (floor_z, len(points), a.repeats, a.angle, "MOVING THE ARM" if a.go else "dry runs only"))
    results = []
    first = True
    for rep in range(a.repeats if a.go else 1):
        for r, h, p in points:
            tag = "r %.2f h %.2f rep %d" % (r, h, rep + 1)
            if a.go and not first:
                res = preset(action="go", name=a.preset)
                if not res.success:
                    print(tag, "cannot go to the viewing pose:", res.message)
                    return 2
                rospy.sleep(2.0)
                wait_state(("IDLE", "DONE", "ABORTED"), 5.0)
            first = False
            m = wait_marker(buf, 40.0)
            if m is None:
                print(tag, "marker_0 not seen after the move: stopping")
                return 3
            offset = to.vertical_frame(m[1]).T.dot(p - m[0])
            rospy.set_param("/grasp_target/approach", "%g" % a.angle)
            rospy.set_param("/grasp_target/offset", [float(v) for v in offset])
            rospy.sleep(3.0)                                                   # the target node re-reads its parameters at 2 Hz and re-publishes
            dry = grasp(target_frame="grasp_target", dry_run=True, probe=False)
            if not dry.success:
                print("%-24s SKIPPED: %s" % (tag, dry.message[:120]))
                results.append((tag, "skipped"))
                continue
            if not a.go:
                print("%-24s feasible (%s)" % (tag, dry.message[:90]))
                results.append((tag, "feasible"))
                continue
            res = grasp(target_frame="grasp_target", dry_run=False, probe=True)
            if not res.success:
                print("%-24s REFUSED: %s" % (tag, res.message[:120]))
                results.append((tag, "refused"))
                continue
            last = wait_state(("DONE", "ABORTED"), 90.0)
            print("%-24s %s: %s" % (tag, last.get("state"), str(last.get("message"))[:100]))
            results.append((tag, last.get("state")))
            if last.get("state") != "DONE":
                print("a probe aborted: stopping the test here (look at the arm first)")
                return 4
    print("done: %s" % ", ".join("%s=%s" % t for t in results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
