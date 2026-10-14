#!/usr/bin/env python3
"""TF-driven grasp sequence: open the gripper, move above the target, descend straight onto it, close the gripper, lift.
Executed as a behavior tree (py_trees, see arm_grasp/grasp_tree.py); view it live with `rqt_py_trees` (topic ~log/tree).

COMMANDS THE REAL ARM (through the trajectory executor and /joint_states_out). Safety:
  * the WHOLE sequence is planned first (each leg with /generate_trajectory, chained start configurations, nothing
    executed); any leg that is unreachable, or a straight leg that deviates too much, means nothing moves
  * ~grasp with dry_run:=true plans and reports only
  * the trajectories run through the existing executor, which refuses to move under an e-stop (fail closed); the
    tree's guard checks stop and e-stop before every tick, and any failure cancels the motion (the driver holds)
  * the descent and the lift are slowed down (descent_slow_factor, lift_slow_factor) beyond the planner's timing
  * the target must be a fresh TF (target_max_age); before the descent it is looked up again and the sequence aborts
    if it moved
  * correction_steps > 0: the arm sags under gravity (3.7 cm at the PRE pose on the hardware), so after PRE and after the descent
    the end effector is measured and re-aimed up to correction_steps times; the descent and the lift are then re-planned from the
    actual joints with the learned offset, and the sequence aborts before descending if that is infeasible or the pose is not
    reached within correction_max_error
  * the gripper belongs to the gripper_control node (roslaunch arm_grasp gripper_control.launch must be running for a real
    grasp): this node asks it to open and to close on load, and watches its grip status. Its load PI keeps regulating after
    the close, through the lift and after the sequence ends; a lost grip aborts the lift. Closing completely below the target
    load is 'thin' (counted as grasped; aborts if require_object)
The target frame is any TF in base_link whose +z is the surface normal (e.g. marker_<id> from marker_localizer).
Parameters: config/grasp.yaml (every default flagged there).
"""
import json
import threading
import time

import actionlib
import numpy as np
import py_trees
import py_trees_ros
import rospy
import tf2_ros
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from actionlib_msgs.msg import GoalStatus

from arm_grasp import correction as corr
from arm_grasp import grasp_plan as gp
from arm_grasp import grasp_tree as gt
from arm_grasp.srv import Grasp, GraspResponse
from arm_trajectory.srv import GenerateTrajectory, GenerateTrajectoryRequest

JOINTS = ["joint_%d" % i for i in range(1, 6)]


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Node(gt.GraspContext):
    def __init__(self):
        P = rospy.get_param
        d = gp.GraspSpec()
        self.spec = gp.GraspSpec(
            pre_height=float(P("~pre_height", d.pre_height)), grasp_height=float(P("~grasp_height", d.grasp_height)),
            lift_height=float(P("~lift_height", d.lift_height)), tool_offset=float(P("~tool_offset", d.tool_offset)),
            grasp_offset=tuple(P("~grasp_offset", list(d.grasp_offset))), gripper_open=float(P("~gripper_open", d.gripper_open)),
            gripper_closed=float(P("~gripper_closed", d.gripper_closed)), descent_slow_factor=float(P("~descent_slow_factor", d.descent_slow_factor)),
            lift_slow_factor=float(P("~lift_slow_factor", d.lift_slow_factor)),
            side_grasp_height=float(P("~side_grasp_height", d.side_grasp_height)), side_lift=str(P("~side_lift", d.side_lift)),
            max_cartesian_deviation=float(P("~max_cartesian_deviation", d.max_cartesian_deviation)))
        self.base = P("~base_frame", "base_link")
        self.target_max_age = float(P("~target_max_age", 5.0))
        self.recheck_mm = float(P("~recheck_translation_mm", 5.0))
        self.recheck_deg = float(P("~recheck_rotation_deg", 3.0))
        self.gripper_speed = float(P("~gripper_speed", 0.02))
        self.gripper_settle = float(P("~gripper_settle", 0.6))
        self.require_object = bool(P("~require_object", False))
        self.endpoint_tol = float(P("~endpoint_tolerance_rad", 0.03))
        steps = int(P("~correction_steps", 0))
        self.correction = corr.PositionCorrection(steps=steps, trigger=float(P("~correction_trigger", 0.006)), gain=float(P("~correction_gain", 1.0)),
                                                  max_total=float(P("~correction_max_total", 0.06)), max_error=float(P("~correction_max_error", 0.05)))
        if steps > 0:                                                  # the sag is corrected in Cartesian space: a joint-space endpoint check at 0.03 rad would abort first
            self.endpoint_tol = max(self.endpoint_tol, float(P("~correction_endpoint_tolerance_rad", 0.12)))
        self.tick_rate = float(P("~tick_rate", 20.0))
        d_close = gp.CloseParams()
        self.close = gp.CloseParams(**{k: type(getattr(d_close, k))(P("~close/" + k, getattr(d_close, k))) for k in vars(d_close)})
        self.params = gt.SequenceParams(recheck_mm=self.recheck_mm, recheck_deg=self.recheck_deg, require_object=self.require_object,
                                        correction=self.correction, correction_settle=float(P("~correction_settle", 0.8)))
        self.wps = []
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        self.q = None
        self.grip_status, self.grip_status_time = {}, 0.0             # the gripper_control node's latest ~status (JSON) and when it arrived
        self.gripper_call = {"state": "idle"}                         # the gripper_control call in a worker thread
        self.estop = None
        self.lock = threading.Lock()
        self.stop_requested = threading.Event()
        self.state = "IDLE"
        self.message = ""
        self.plan_info = {}
        self.target_frame = ""
        self.worker = None
        self.abort_reason = None
        self.plan_T, self.plan_stamp, self.trajs = None, None, {}
        self.goal_sent_at, self.goal_end, self.goal_duration = 0.0, None, 0.0
        self.outcome = None
        self.tree = py_trees_ros.trees.BehaviourTree(gt.build_tree(self, self.spec, self.params, lambda: self.outcome), record_rosbag=False)
        if not self.tree.setup(timeout=5.0):
            rospy.logwarn("grasp: the behavior tree's ROS interfaces did not set up in time (the tree still runs)")
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        rospy.Subscriber("/gripper_control/status", String, self.on_grip_status, queue_size=1)
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: setattr(self, "estop", bool(m.data)), queue_size=1)
        self.pub_status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.client = actionlib.SimpleActionClient("follow_joint_trajectory", FollowJointTrajectoryAction)
        self.plan_srv = rospy.ServiceProxy("/generate_trajectory", GenerateTrajectory)
        rospy.Service("~grasp", Grasp, self.on_grasp)
        rospy.Service("~stop", Trigger, self.on_stop)
        self.publish()
        rospy.logwarn("grasp sequencer ready (IDLE): pre %.3f / grasp %.3f / lift %.3f m above the target, gripper %.3f open %.3f closed "
                      "(FLAGGED DEFAULTS: see config/grasp.yaml). Nothing moves until ~grasp.", self.spec.pre_height, self.spec.grasp_height,
                      self.spec.lift_height, self.spec.gripper_open, self.spec.gripper_closed)

    # ---- inputs ---------------------------------------------------------------------------------------------
    def on_joints(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.q = [d[j] for j in JOINTS]

    def on_grip_status(self, m):
        try:
            self.grip_status, self.grip_status_time = json.loads(m.data), time.time()
        except ValueError:
            pass

    def estop_allows(self):
        return self.estop is False

    def publish(self):
        self.pub_status.publish(String(json.dumps({"state": self.state, "message": self.message, "target_frame": self.target_frame,
                                                   "plan": self.plan_info, "gripper": self.grip_status,
                                                   "active_path": gt.active_path(self.tree.root)})))

    def set_state(self, state, message=""):
        self.state, self.message = state, message
        rospy.logwarn("grasp: %s %s", state, message)
        self.publish()

    # ---- planning ---------------------------------------------------------------------------------------------
    def lookup_target(self):
        t = self.buf.lookup_transform(self.base, self.target_frame, rospy.Time(0), rospy.Duration(1.0))
        age = (rospy.Time.now() - t.header.stamp).to_sec()
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_matrix([q.x, q.y, q.z, q.w])
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        return T, t.header.stamp, age

    def plan_legs(self, wps):
        """Plan PRE (joint space), DESCEND and LIFT (straight lines) with chained start configurations. Returns
        (legs, trajectories)."""
        return self.plan_chain(list(self.q), (("PRE", wps[0].position, wps[0].R, False), ("DESCEND", wps[1].position, wps[1].R, True),
                                              ("LIFT", wps[2].position, wps[2].R, True)))

    def plan_chain(self, start, items):
        """Plan the (name, position, R, cartesian) items one after the other, each from the end of the previous one."""
        legs, trajs = [], {}
        for name, position, R, cartesian in items:
            if start is None:
                legs.append(gp.Leg(name, False, "not planned: an earlier leg failed", cartesian=cartesian))
                continue
            req = GenerateTrajectoryRequest()
            req.start_joint_positions = start
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [float(v) for v in position]
            qx, qy, qz, qw = gp.quaternion_from_matrix(R)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
            req.goal_pose = pose
            req.cartesian_path = cartesian
            try:
                res = self.plan_srv(req)
            except rospy.ServiceException as e:
                legs.append(gp.Leg(name, False, "planner call failed: %s" % e, cartesian=cartesian))
                start = None
                continue
            if not res.success or not res.trajectory.points:
                legs.append(gp.Leg(name, False, res.message, cartesian=cartesian))
                start = None
                continue
            dur = res.trajectory.points[-1].time_from_start.to_sec()
            legs.append(gp.Leg(name, True, duration=dur, deviation=res.peak_position_deviation if cartesian else 0.0, cartesian=cartesian))
            trajs[name] = res.trajectory
            start = list(res.trajectory.points[-1].positions)
        return legs, trajs

    def on_grasp(self, req):
        if self.state not in ("IDLE", "DONE", "ABORTED"):
            return GraspResponse(False, "refused: a grasp is already in progress (%s)" % self.state, 0.0, 0.0)
        if self.q is None:
            return GraspResponse(False, "refused: no /joint_states_in", 0.0, 0.0)
        if not req.dry_run and not self.estop_allows():
            return GraspResponse(False, "refused: e-stop engaged or its state unknown", 0.0, 0.0)
        if not req.dry_run and self.grip_state() is None:
            return GraspResponse(False, "refused: no /gripper_control/status (start it: roslaunch arm_grasp gripper_control.launch)", 0.0, 0.0)
        self.target_frame = req.target_frame
        try:
            T, stamp, age = self.lookup_target()
        except tf2_ros.TransformException as e:
            return GraspResponse(False, "refused: no TF %s -> %s (%s)" % (self.base, self.target_frame, e), 0.0, 0.0)
        if age > self.target_max_age:
            return GraspResponse(False, "refused: the target frame %s is stale (%.1f s old, limit %.1f s)" % (self.target_frame, age, self.target_max_age), 0.0, 0.0)
        self.set_state("PLANNING", "target %s at (%.3f, %.3f, %.3f)" % (self.target_frame, *T[:3, 3]))
        try:
            wps = gp.waypoints(T, self.spec)
        except ValueError as e:
            self.set_state("ABORTED", str(e))
            return GraspResponse(False, "refused: " + str(e), 0.0, 0.0)
        legs, trajs = self.plan_legs(wps)
        self.wps = wps
        problems = gp.check_plan(legs, self.spec)
        duration = gp.total_duration(legs, self.spec, gripper_time=(self.spec.gripper_open - self.spec.gripper_closed) * (1.0 / self.gripper_speed + 1.0 / self.close.close_speed) + self.gripper_settle + self.close.hold_time + 5.0)
        dev = max([l.deviation for l in legs if l.success] or [0.0])
        self.plan_info = {"legs": [{"name": l.name, "ok": l.success, "duration": l.duration, "deviation_mm": 1000 * l.deviation, "message": l.message} for l in legs],
                          "duration": duration, "problems": problems, "waypoints": {w.name: [float(v) for v in w.position] for w in wps}}
        if problems:
            self.set_state("ABORTED", "plan not feasible: " + "; ".join(problems))
            return GraspResponse(False, "plan not feasible: " + "; ".join(problems), duration, dev)
        if req.dry_run:
            self.set_state("IDLE", "dry run: the plan is feasible (%.1f s)" % duration)
            return GraspResponse(True, "plan feasible: PRE %.1f s, DESCEND %.1f s (x%.1f), LIFT %.1f s (x%.1f), total about %.1f s; nothing moved" % (
                legs[0].duration, legs[1].duration, self.spec.descent_slow_factor, legs[2].duration, self.spec.lift_slow_factor, duration), duration, dev)
        self.stop_requested.clear()
        self.plan_T, self.plan_stamp, self.trajs = T, stamp, trajs
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()
        return GraspResponse(True, "plan feasible, sequence started (about %.1f s)" % duration, duration, dev)

    def on_stop(self, _):
        if self.state not in ("IDLE", "DONE", "ABORTED"):
            self.stop_requested.set()
            self.client.cancel_all_goals()
        return TriggerResponse(True, "stop requested (the driver holds the last goal)")

    # ---- GraspContext (what the tree's leaves use) ----------------------------------------------------------
    def now(self):
        return time.time()

    def warn(self, message):
        rospy.logwarn("grasp: %s", message)

    def fail(self, reason):
        if self.abort_reason is None:
            self.abort_reason = reason

    def safety_violation(self):
        if self.stop_requested.is_set():
            return "stopped by the operator (during %s)" % self.state
        if not self.estop_allows():
            return "e-stop engaged or its state unknown (during %s)" % self.state
        return None

    def start_gripper(self, action):
        service = {"open": "/gripper_control/open", "close": "/gripper_control/close_on_load"}[action]
        call = {"state": "running", "message": "", "outcome": "", "load": 0.0}
        self.gripper_call = call

        def work():
            try:
                client = rospy.ServiceProxy(service, Trigger)
                rospy.wait_for_service(service, 3.0)
                res = client()
                call["message"] = res.message
                if res.success and action == "close":
                    head, _, rest = res.message.partition(": ")
                    call["outcome"] = head if head in ("object", "thin") else "object"
                    try:
                        call["load"] = float(rest.split("load")[1].split("%")[0]) if "load" in rest else float(self.grip_status.get("load_percent") or 0.0)
                    except (IndexError, ValueError):
                        call["load"] = float(self.grip_status.get("load_percent") or 0.0)
                call["state"] = "succeeded" if res.success else "failed"
            except (rospy.ServiceException, rospy.ROSException) as e:
                call["message"], call["state"] = "gripper_control call failed: %s" % e, "failed"

        threading.Thread(target=work, daemon=True).start()
        return None

    def poll_gripper(self):
        c = self.gripper_call
        return c["state"] if c["state"] in ("running", "succeeded", "failed") else "failed", c.get("message", ""), c.get("outcome", ""), c.get("load", 0.0)

    def cancel_gripper(self):
        def work():
            try:
                rospy.ServiceProxy("/gripper_control/stop", Trigger)()
            except (rospy.ServiceException, rospy.ROSException):
                pass
        threading.Thread(target=work, daemon=True).start()

    def grip_state(self):
        if time.time() - self.grip_status_time > 2.0:
            return None
        return self.grip_status.get("grip")

    def start_trajectory(self, name, slow):
        return self._send(name, self.trajs[name], slow)

    def _send(self, name, traj, slow):
        stretched = gp.stretch_times([(p.time_from_start.to_sec(), p.positions, p.velocities, p.accelerations) for p in traj.points], slow)
        out = JointTrajectory()
        out.joint_names = traj.joint_names
        for t, pos, vel, acc in stretched:
            jp = JointTrajectoryPoint(positions=pos, velocities=vel or [], accelerations=acc or [])
            jp.time_from_start = rospy.Duration(t)
            out.points.append(jp)
        if not self.client.wait_for_server(rospy.Duration(3.0)):
            return "follow_joint_trajectory server not available"
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = out
        self.goal_name, self.goal_end = name, np.array(out.points[-1].positions)
        self.goal_duration = out.points[-1].time_from_start.to_sec()
        self.goal_sent_at = time.time()
        self.client.send_goal(goal)
        return None

    def poll_trajectory(self):
        state = self.client.get_state()
        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING, GoalStatus.RECALLING):
            if time.time() - self.goal_sent_at > self.goal_duration + 15.0:
                self.client.cancel_all_goals()
                return "failed", "%s timed out" % self.goal_name
            return "running", ""
        if state != GoalStatus.SUCCEEDED:
            return "failed", "%s failed: %s" % (self.goal_name, self.client.get_goal_status_text() or "not completed")
        got = np.array(self.q)
        err = float(np.max(np.abs(self.goal_end - got[:len(self.goal_end)])))
        if err > self.endpoint_tol:
            return "failed", "%s ended %.3f rad away from its goal" % (self.goal_name, err)
        return "succeeded", ""

    def cancel_trajectory(self):
        self.client.cancel_all_goals()

    # ---- correction (measure, re-aim, re-plan) ---------------------------------------------------------------
    def measure_ee(self):
        try:
            t = self.buf.lookup_transform(self.base, "end_effector", rospy.Time(0), rospy.Duration(0.3))
        except tf2_ros.TransformException:
            return None
        if (rospy.Time.now() - t.header.stamp).to_sec() > 1.0:         # the joint states stopped: no measurement
            return None
        return np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])

    def waypoint(self, key):
        return np.array(self.wps[{"PRE": 0, "GRASP": 1, "LIFT": 2}[key]].position)

    def start_corrective(self, key, aim):
        wp = self.wps[{"PRE": 0, "GRASP": 1, "LIFT": 2}[key]]
        cartesian = key != "PRE"
        legs, trajs = self.plan_chain(list(self.q), (("CORR_" + key, aim, wp.R, cartesian),))
        problems = gp.check_plan(legs, self.spec)
        if problems:
            return "; ".join(problems)
        return self._send("CORR_" + key, trajs["CORR_" + key], self.spec.descent_slow_factor if cartesian else 1.0)

    def replan_legs(self, comp, legs):
        """Re-plan `legs` from the CURRENT joints with `comp` added to their waypoints; keep them only if they pass the same checks."""
        comp = np.asarray(comp, float)
        items = [(name, self.wps[{"DESCEND": 1, "LIFT": 2}[name]].position + comp, self.wps[{"DESCEND": 1, "LIFT": 2}[name]].R, True) for name in legs]
        planned, trajs = self.plan_chain(list(self.q), items)
        problems = gp.check_plan(planned, self.spec)
        if problems:
            return "; ".join(problems)
        self.trajs.update(trajs)
        return None

    def target_drift(self):
        T_now, stamp_now, _ = self.lookup_target()
        if stamp_now == self.plan_stamp:                              # nothing newer to compare with
            return None
        return gp.target_moved(self.plan_T, T_now)

    def record_outcome(self, outcome, load):
        self.outcome = "%s (load %.0f %%)" % (outcome, load or 0.0)
        self.plan_info.update(grasp_outcome=outcome, grasp_load_percent=load)

    # ---- execution: tick the tree ---------------------------------------------------------------------------
    def run(self):
        """Tick the behavior tree at tick_rate until it succeeds or fails. The leaves report the states."""
        self.abort_reason, self.outcome = None, None
        self.correction.reset()                                          # the offset learned in an earlier grasp does not apply to this one
        self.tree.root.stop(py_trees.common.Status.INVALID)
        rate = rospy.Rate(self.tick_rate)
        last_path = None
        try:
            while not rospy.is_shutdown():
                self.tree.tick()
                status = self.tree.root.status
                path = gt.active_path(self.tree.root)
                if path != last_path:                                 # leaves set the state before they are RUNNING: refresh the path
                    last_path = path
                    self.publish()
                if status == py_trees.common.Status.SUCCESS:
                    self.plan_info["correction_mm"] = [round(1000.0 * float(v), 1) for v in self.correction.comp]
                    self.set_state("DONE", "grasp outcome: %s" % self.outcome)
                    return
                if status == py_trees.common.Status.FAILURE:
                    self.set_state("ABORTED", self.abort_reason or "the behavior tree failed")
                    return
                rate.sleep()
            self.client.cancel_all_goals()
        except Exception as e:  # noqa: BLE001  (never leave the tree running)
            self.client.cancel_all_goals()
            self.set_state("ABORTED", "unexpected error: %s" % e)


def main():
    rospy.init_node("grasp_sequencer")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
