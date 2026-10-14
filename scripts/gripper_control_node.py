#!/usr/bin/env python3
"""Gripper control as its own ROS node: it OWNS joint_6 and keeps the load regulated until told otherwise.

A 20 Hz loop runs arm_grasp/gripper_controller.GripperController: open, slow load-limited close, then HOLDING, where the
same PI keeps the load at close/load_target for as long as the gripper is not opened or stopped (it does not end when the
close call returns). A load that collapses after the grasp settled is GRIP_LOST: the command freezes (no creeping closed)
and `~grip_lost` goes true. Services (all block until the action ends; the regulation continues afterwards):
    ~open            std_srvs/Trigger        ramp to gripper_open
    ~close_on_load   std_srvs/Trigger        the load-limited close (the rviz panel uses it)
    ~close           arm_grasp/GripperClose  the same with execute / open_first; execute:=false only watches (sends nothing)
    ~stop            std_srvs/Trigger        end the action and the regulation; the command stays where it is (the driver holds it)
    ~set_load_target arm_gui/SetLoadTarget  change the steady-state load target AT RUN TIME (clamped to [0, close/load_target_max]); the rviz panel slider uses it
Topics: ~status (JSON, latched: phase, message, grip held|thin|lost|none, command, load_percent, position, load_target, load_target_max), ~grip_lost (Bool, latched).

COMMANDS THE GRIPPER (joint_6 only, through /joint_states_out). Safety:
  * refused when the e-stop is engaged or unknown (fail closed); an e-stop during any action or hold freezes the gripper at once
  * refused without a joint_6 measurement and a fresh /arm_interface/motor_load; a load that goes stale freezes it (the driver
    holds the last goal)
  * the closing law is grasp_plan.LoadLimitedCloser; see its docstring and config/grasp.yaml (every value there is flagged)
Parameters: config/grasp.yaml.

    rosservice call /gripper_control/close "{execute: false, open_first: false}"    # watch only
    rosservice call /gripper_control/open
    rosservice call /gripper_control/close_on_load
    rosservice call /gripper_control/stop
"""
import json
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from arm_grasp import grasp_plan as gp
from arm_grasp import gripper_controller as gc
from arm_grasp.srv import GripperClose, GripperCloseResponse
from arm_gui.srv import SetLoadTarget, SetLoadTargetResponse

RATE_HZ = 20.0


class Node(object):
    def __init__(self):
        P = rospy.get_param
        d = gp.CloseParams()
        self.params = gp.CloseParams(**{k: type(getattr(d, k))(P("~close/" + k, getattr(d, k))) for k in vars(d)})
        self.controller = gc.GripperController(
            self.params, open_pos=float(P("~gripper_open", 0.025)), closed_pos=float(P("~gripper_closed", 0.0)),
            open_speed=float(P("~gripper_speed", 0.02)), open_settle=float(P("~gripper_settle", 0.6)),
            open_tol=float(P("~gripper_tolerance", 0.002)), timeout=float(P("~timeout", 40.0)))
        self.pos, self.load, self.load_time, self.estop = None, None, None, None
        self.lock = threading.RLock()
        self.action_lock = threading.Lock()                           # one open/close call at a time
        self.waiter = None                                            # (Event, [Result]) of the call waiting for its action to end
        self.last_published = None
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        rospy.Subscriber("/arm_interface/motor_load", JointState, self.on_load, queue_size=1)
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: setattr(self, "estop", bool(m.data)), queue_size=1)
        self.pub_out = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
        self.pub_status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.pub_lost = rospy.Publisher("~grip_lost", Bool, queue_size=1, latch=True)
        rospy.Service("~close", GripperClose, self.on_close)
        rospy.Service("~stop", Trigger, self.on_stop)
        rospy.Service("~set_load_target", SetLoadTarget, self.on_set_load_target)
        rospy.Service("~open", Trigger, self.on_open)
        rospy.Service("~close_on_load", Trigger, self.on_close_trigger)
        self.pub_lost.publish(Bool(False))
        self.publish(force=True)
        threading.Thread(target=self.loop, daemon=True).start()
        rospy.logwarn("gripper_control ready (IDLE): load target %.1f %%, peak limit %.0f %%, close speed %.4f m/s (config/grasp.yaml). "
                      "Nothing moves until ~open / ~close_on_load.", self.params.load_target, self.params.load_limit, self.params.close_speed)

    # ---- inputs -----------------------------------------------------------------------------------------------
    def on_joints(self, m):
        if "joint_6" in m.name and m.name.index("joint_6") < len(m.position):
            self.pos = m.position[m.name.index("joint_6")]

    def on_load(self, m):
        if "joint_6" in m.name and m.name.index("joint_6") < len(m.effort):
            self.load, self.load_time = abs(float(m.effort[m.name.index("joint_6")])), time.time()

    # ---- the control loop --------------------------------------------------------------------------------------
    def command(self, cmd):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name, js.position = ["joint_6"], [float(cmd)]
        self.pub_out.publish(js)

    def loop(self):
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown():
            try:
                with self.lock:
                    out = self.controller.step_safe(time.time(), self.pos, self.load, self.load_time, self.estop is False)
                    if out.cmd is not None:
                        self.command(out.cmd)
                    if out.result is not None and self.waiter is not None:
                        self.waiter[1].append(out.result)
                        self.waiter[0].set()
                    if self.controller.last_error:
                        rospy.logerr("gripper_control: internal error, the gripper is frozen (open or close again to recover):\n%s", self.controller.last_error)
                        self.controller.last_error = None
                self.publish()
            except Exception as e:  # noqa: BLE001  (this thread must never die: a dead loop cannot open the gripper)
                rospy.logerr_throttle(5.0, "gripper_control: loop error %s: %s", type(e).__name__, e)
            rate.sleep()

    def publish(self, force=False):
        c = self.controller
        state = (c.phase, c.message, c.grip, c.p.load_target)
        now = time.time()
        if not force and state == self.last_published and now - getattr(self, "t_pub", 0.0) < 0.5:
            return
        if state != self.last_published:
            self.pub_lost.publish(Bool(c.grip == "lost"))
            if self.last_published is not None:
                rospy.logwarn("gripper_control: %s %s (grip: %s)", c.phase, c.message, c.grip)
        self.last_published, self.t_pub = state, now
        self.pub_status.publish(String(json.dumps({"phase": c.phase, "message": c.message, "grip": c.grip, "command": c.cmd,
                                                   "load_percent": self.load, "position": self.pos,
                                                   "load_target": c.p.load_target, "load_target_max": c.p.load_target_max})))

    # ---- actions ----------------------------------------------------------------------------------------------
    def preflight(self):
        """None when it is safe to command the gripper, else the reason it is refused."""
        t0 = time.time()
        while (self.pos is None or self.load_time is None or self.estop is None) and time.time() - t0 < 4.0:
            time.sleep(0.05)
        if self.pos is None or self.load_time is None:
            return "no joint_6 measurement or no /arm_interface/motor_load (is the driver or the simulator running?)"
        if self.estop is not False:
            return "e-stop engaged or its state unknown"
        if time.time() - self.load_time > self.params.load_max_age:
            return "the gripper load is stale (%.1f s old)" % (time.time() - self.load_time)
        return None

    def run_action(self, request):
        """Starts an action on the controller and blocks until it ends. Returns a gripper_controller.Result."""
        ev, box = threading.Event(), []
        with self.lock:
            self.waiter = (ev, box)
            request()
        ev.wait(self.controller.timeout + 10.0)
        with self.lock:
            self.waiter = None
        return box[0] if box else gc.Result("?", False, "", "no result (timed out waiting)")

    def do_open(self):
        why = self.preflight()
        if why:
            return gc.Result("open", False, "", "refused: " + why)
        return self.run_action(lambda: self.controller.request_open(time.time(), self.pos))

    def do_close(self):
        why = self.preflight()
        if why:
            return gc.Result("close", False, "", "refused: " + why)
        return self.run_action(lambda: self.controller.request_close(time.time(), self.pos))

    def on_open(self, _):
        if not self.action_lock.acquire(False):
            return TriggerResponse(False, "refused: a gripper action is already in progress")
        try:
            r = self.do_open()
            return TriggerResponse(r.ok, r.message)
        finally:
            self.action_lock.release()

    def on_close_trigger(self, _):
        if not self.action_lock.acquire(False):
            return TriggerResponse(False, "refused: a gripper action is already in progress")
        try:
            r = self.do_close()
            return TriggerResponse(r.ok, ("%s: " % r.outcome if r.ok else "") + r.message)
        finally:
            self.action_lock.release()

    def on_close(self, req):
        if not self.action_lock.acquire(False):
            return GripperCloseResponse(False, "", "refused: a gripper action is already in progress", self.load or 0.0)
        try:
            if not req.execute:                                         # watch only: nothing is sent
                why = self.preflight()
                if why:
                    return GripperCloseResponse(False, "", "refused: " + why, self.load or 0.0)
                return GripperCloseResponse(True, "not executed", "watch only: the gripper was not commanded", self.load or 0.0)
            if req.open_first:
                r = self.do_open()
                if not r.ok:
                    return GripperCloseResponse(False, "", r.message, self.load or 0.0)
            r = self.do_close()
            return GripperCloseResponse(r.ok, r.outcome if r.ok else "", r.message, r.load if r.ok else (self.load or 0.0))
        finally:
            self.action_lock.release()

    def on_set_load_target(self, req):
        with self.lock:
            try:
                applied, contact = self.controller.set_load_target(req.load_target)
            except ValueError as e:
                return SetLoadTargetResponse(False, "refused: %s" % e, self.params.load_target, self.params.contact_load, self.params.load_target_max)
        rospy.logwarn("gripper_control: load target %.1f %% (requested %.1f %%), contact/loss level %.1f %%", applied, req.load_target, contact)
        self.publish(force=True)
        return SetLoadTargetResponse(True, "load target %.1f %% (max %.1f %%), contact level %.1f %%" % (applied, self.params.load_target_max, contact),
                                     applied, contact, self.params.load_target_max)

    def on_stop(self, _):
        with self.lock:
            result = self.controller.request_stop("stopped by the operator")
            if result is not None and self.waiter is not None:
                self.waiter[1].append(result)
                self.waiter[0].set()
        self.publish(force=True)
        return TriggerResponse(True, "stopped: no more commands are sent, the driver holds the last goal")


def main():
    rospy.init_node("gripper_control")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
