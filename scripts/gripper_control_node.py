#!/usr/bin/env python3
"""Gripper control as its own ROS node: services ~open (Trigger), ~close (arm_grasp/GripperClose, the load-limited close), ~stop, latched ~status.

COMMANDS THE GRIPPER (joint_6 only, through /joint_states_out) when ~close is called with execute:=true; with
execute:=false it only watches the load. No planner, trajectory executor or arm motion is involved. Safety:
  * refused when the e-stop is engaged or its state is unknown (/arm_interface/estop_state, fail closed), and an e-stop
    during the close ends it at once
  * refused without a joint_6 measurement and a fresh /arm_interface/motor_load; a load that goes stale while closing
    stops the commands (the driver holds the last goal) and the call fails
  * closing is the slow load-limited law of grasp_plan.LoadLimitedCloser (constant close_speed until the load reaches
    load_target, then it holds there); ~stop ends it, the gripper stays where the last command put it
The same law runs inside the grasp sequence (grasp_tree.CloseOnLoad). Parameters: config/grasp.yaml.

    rosservice call /gripper_control/close "{execute: false, open_first: false}"     # watch only
    rosservice call /gripper_control/close "{execute: true, open_first: false}"      # close from where it is
    rosservice call /gripper_control/open                                            # ramp to gripper_open (COMMANDS the gripper)
"""
import json
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from arm_grasp import grasp_plan as gp
from arm_grasp.srv import GripperClose, GripperCloseResponse


class Node(object):
    def __init__(self):
        P = rospy.get_param
        d = gp.CloseParams()
        self.params = gp.CloseParams(**{k: type(getattr(d, k))(P("~close/" + k, getattr(d, k))) for k in vars(d)})
        self.open = float(P("~gripper_open", 0.025))
        self.closed = float(P("~gripper_closed", 0.0))
        self.open_speed = float(P("~gripper_speed", 0.02))
        self.open_settle = float(P("~gripper_settle", 0.6))
        self.open_tol = float(P("~gripper_tolerance", 0.002))
        self.timeout = float(P("~timeout", 40.0))
        self.pos, self.load, self.load_time, self.estop = None, None, None, None
        self.busy = threading.Lock()
        self.stop_requested = threading.Event()
        rospy.Subscriber("/joint_states_in", JointState, self.on_joints, queue_size=1)
        rospy.Subscriber("/arm_interface/motor_load", JointState, self.on_load, queue_size=1)
        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: setattr(self, "estop", bool(m.data)), queue_size=1)
        self.pub_out = rospy.Publisher("/joint_states_out", JointState, queue_size=1)
        self.pub_status = rospy.Publisher("~status", String, queue_size=1, latch=True)
        rospy.Service("~close", GripperClose, self.on_close)
        rospy.Service("~stop", Trigger, self.on_stop)
        rospy.Service("~open", Trigger, self.on_open)
        self.publish("IDLE", "")
        rospy.logwarn("gripper_control ready (IDLE): load target %.0f %%, close speed %.4f m/s (config/grasp.yaml). Nothing moves until ~close with execute:=true.",
                      self.params.load_target, self.params.close_speed)

    def on_joints(self, m):
        if "joint_6" in m.name and m.name.index("joint_6") < len(m.position):
            self.pos = m.position[m.name.index("joint_6")]

    def on_load(self, m):
        if "joint_6" in m.name and m.name.index("joint_6") < len(m.effort):
            self.load, self.load_time = abs(float(m.effort[m.name.index("joint_6")])), time.time()

    def publish(self, phase, message, cmd=None):
        self.pub_status.publish(String(json.dumps({"phase": phase, "message": message, "command": cmd, "load_percent": self.load, "executing": phase not in ("IDLE",)})))

    def command(self, cmd):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name, js.position = ["joint_6"], [float(cmd)]
        self.pub_out.publish(js)

    def on_stop(self, _):
        self.stop_requested.set()
        return TriggerResponse(True, "stop requested (the driver holds the last goal)")

    def refuse(self, message):
        self.publish("IDLE", "refused: " + message)
        return GripperCloseResponse(False, "", "refused: " + message, self.load or 0.0)

    def on_close(self, req):
        if not self.busy.acquire(False):
            return GripperCloseResponse(False, "", "refused: a close is already in progress", self.load or 0.0)
        try:
            return self.close(req)
        finally:
            self.busy.release()

    def preflight(self):
        """None when it is safe to command the gripper, else the reason it is refused."""
        self.stop_requested.clear()
        t0 = time.time()
        while (self.pos is None or self.load_time is None or self.estop is None) and time.time() - t0 < 4.0:
            time.sleep(0.05)
        if self.pos is None or self.load_time is None:
            return "no joint_6 measurement or no /arm_interface/motor_load (is the driver or the simulator running?)"
        if self.estop is not False:
            return "e-stop engaged or its state unknown"
        return None

    def ramp_open(self):
        """Ramp the command to gripper_open at gripper_speed and wait for the measurement to settle there. (ok, message)."""
        rate, cmd, reached, t_start = rospy.Rate(20), self.pos, None, time.time()
        self.publish("OPENING", "to %.3f" % self.open)
        while not rospy.is_shutdown() and time.time() - t_start < self.timeout:
            if self.stop_requested.is_set() or self.estop is not False:
                return False, "stopped or e-stop while opening"
            cmd = gp.gripper_ramp(cmd, self.open, self.open_speed, 0.05)
            self.command(cmd)
            if cmd == self.open and abs(self.pos - self.open) <= self.open_tol:
                reached = reached or time.time()
                if time.time() - reached >= self.open_settle:
                    return True, "open (joint_6 %.4f)" % self.pos
            rate.sleep()
        return False, "timed out opening (joint_6 %.4f, target %.4f)" % (self.pos, self.open)

    def on_open(self, _):
        if not self.busy.acquire(False):
            return TriggerResponse(False, "refused: a gripper move is already in progress")
        try:
            why = self.preflight()
            if why:
                self.publish("IDLE", "refused: " + why)
                return TriggerResponse(False, "refused: " + why)
            ok, message = self.ramp_open()
            self.publish("DONE" if ok else "ABORTED", message)
            rospy.logwarn("gripper_control: open %s", message)
            return TriggerResponse(ok, message)
        finally:
            self.busy.release()

    def close(self, req):
        why = self.preflight()
        if why:
            return self.refuse(why)
        rate = rospy.Rate(20)
        if req.open_first and req.execute:
            ok, message = self.ramp_open()
            if not ok:
                return self.finish(False, "", message)
        closer = gp.LoadLimitedCloser(self.pos, self.closed, self.open, self.params)
        phase, t_close = None, time.time()
        while not rospy.is_shutdown() and time.time() - t_close < self.timeout:
            if self.stop_requested.is_set():
                return self.finish(False, "", "stopped by the operator")
            if self.estop is not False:
                return self.finish(False, "", "e-stop engaged or its state unknown while closing")
            step = closer.step(time.time(), self.pos, self.load, self.load_time)
            if step.fault:
                return self.finish(False, "", step.fault)
            if req.execute:
                self.command(step.cmd)
            if step.phase != phase:
                phase = step.phase
                self.publish(phase, "executing" if req.execute else "watch only", step.cmd)
            if step.done:
                return self.finish(True, step.outcome, "load %.0f %%, outcome %s" % (self.load, step.outcome))
            if not req.execute and time.time() - t_close > 2.0:
                return self.finish(True, "not executed", "watch only: the gripper was not commanded")
            rate.sleep()
        return self.finish(False, "", "timed out")

    def finish(self, ok, outcome, message):
        self.publish("DONE" if ok else "ABORTED", message)
        rospy.logwarn("gripper_control: %s %s", "done:" if ok else "failed:", message)
        return GripperCloseResponse(ok, outcome, message, self.load or 0.0)


def main():
    rospy.init_node("gripper_control")
    Node()
    rospy.spin()


if __name__ == "__main__":
    main()
