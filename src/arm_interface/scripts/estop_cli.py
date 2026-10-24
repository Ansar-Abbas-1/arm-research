#!/usr/bin/env python3
"""Command-line e-stop management for the arm (the same services and topics as the rviz buttons).

  rosrun arm_interface estop_cli.py status     e-stop state, drive state and the health of every motor
  rosrun arm_interface estop_cli.py trigger    engage the e-stop at once (the arm HOLDS its pose, torque on)
  rosrun arm_interface estop_cli.py reset      release it (asks first; refused while a motor has a latched
                                               hardware error, reboot that motor first)
  rosrun arm_interface estop_cli.py motors     per-motor table: temperature and limit, load, voltage, hardware error
  rosrun arm_interface estop_cli.py torque on|off   enable or release torque (/arm_interface/set_torque); releasing
                                               lets the arm SAG, so it asks first; refused while the e-stop is engaged
  rosrun arm_interface estop_cli.py watch      print every change until Ctrl-C

trigger and reset use /arm_interface/estop and /arm_interface/estop_reset; status reads the latched topics
/arm_interface/estop_state, /arm_interface/drive_state and /arm_interface/motor_status. Reset only releases the
latch: the arm keeps holding until the next command.
"""
import argparse
import sys

LEVELS = {0: "OK", 1: "WARN", 2: "ERROR"}


def format_status(estop, drive, statuses):
    """Lines describing the state. statuses: list of (name, level, message) or None if unavailable."""
    state = "UNKNOWN (senders treat it as engaged)" if estop is None else ("ENGAGED" if estop else "released")
    lines = ["e-stop: %s" % state, "drive state: %s" % (drive or "n/a")]
    if statuses is None:
        lines.append("motors: no motor_status topic (simulator, or driver without health monitoring)")
    else:
        bad = [s for s in statuses if s[1] >= 1]
        lines.append("motors: %s" % ("all OK" if not bad else "%d with a warning or error" % len(bad)))
        for name, level, message in bad:
            lines.append("  %s %s: %s" % (LEVELS.get(level, "?"), name, message))
    return lines


def format_motors(statuses):
    """Table lines from (name, level, message, values) tuples; None = no motor_status topic."""
    if statuses is None:
        return ["no motor_status topic (simulator, or a driver without health monitoring)"]
    lines = ["%-8s %-8s %-9s %-9s %-7s %-6s %s" % ("joint", "ID", "temp/limit", "load", "volts", "error", "status")]
    for name, level, message, v in statuses:
        def f(key, fmt, default="-"):
            try:
                return fmt % float(v[key])
            except (KeyError, ValueError):
                return default
        lines.append("%-8s %-8s %-9s %-9s %-7s %-6s %s %s" % (
            name.split("/")[-1], v.get("hardware_id", "") or "-", f("temperature_c", "%.0f") + "/" + f("temperature_limit_c", "%.0f"),
            f("load_percent", "%.1f%%"), f("input_voltage_v", "%.1f"), "0x%02X" % int(float(v.get("hardware_error", 0))),
            LEVELS.get(level, "?"), message))
    return lines


def main():
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["status", "trigger", "reset", "motors", "torque", "watch"])
    ap.add_argument("state", nargs="?", choices=["on", "off"], help="for the torque command")
    ap.add_argument("--yes", action="store_true", help="reset or release torque without asking")
    args = ap.parse_args(rospy.myargv()[1:])
    rospy.init_node("estop_cli", anonymous=True)

    def latest(topic, msg_type, timeout=2.0):
        try:
            return rospy.wait_for_message(topic, msg_type, timeout=timeout)
        except rospy.ROSException:
            return None

    def statuses(detailed=False):
        m = latest("/arm_interface/motor_status", DiagnosticArray, 2.5)
        if m is None:
            return None
        if detailed:
            return [(s.name, s.level, s.message, dict({kv.key: kv.value for kv in s.values}, hardware_id=s.hardware_id.replace("dynamixel_", ""))) for s in m.status]
        return [(s.name.split("/")[-1], s.level, s.message) for s in m.status]

    def call(service):
        try:
            rospy.wait_for_service(service, timeout=2.0)
            return rospy.ServiceProxy(service, Trigger)()
        except (rospy.ROSException, rospy.ServiceException) as e:
            print("%s: %s" % (service, e))
            sys.exit(2)

    if args.command == "status":
        e, d = latest("/arm_interface/estop_state", Bool), latest("/arm_interface/drive_state", String)
        print("\n".join(format_status(None if e is None else e.data, None if d is None else d.data, statuses())))
    elif args.command == "motors":
        print("\n".join(format_motors(statuses(detailed=True))))
    elif args.command == "torque":
        from std_srvs.srv import SetBool
        if args.state is None:
            sys.exit("usage: estop_cli.py torque on|off")
        if args.state == "off" and not args.yes and input("Release torque? The arm will SAG (a heavy pose can fall). Support it first. Type 'yes': ").strip() != "yes":
            print("torque unchanged")
            sys.exit(1)
        try:
            rospy.wait_for_service("/arm_interface/set_torque", timeout=2.0)
            res = rospy.ServiceProxy("/arm_interface/set_torque", SetBool)(args.state == "on")
        except (rospy.ROSException, rospy.ServiceException) as e:
            print("/arm_interface/set_torque: %s" % e)
            sys.exit(2)
        print(("torque %s: " % args.state if res.success else "REFUSED: ") + res.message)
        sys.exit(0 if res.success else 1)
    elif args.command == "trigger":
        res = call("/arm_interface/estop")
        print(("e-stop engaged: " if res.success else "NOT confirmed: ") + res.message)
        sys.exit(0 if res.success else 1)
    elif args.command == "reset":
        e = latest("/arm_interface/estop_state", Bool)
        print("\n".join(format_status(None if e is None else e.data, None, None)[:1]))
        if not args.yes and input("Release the e-stop? The arm keeps holding its pose until the next command. Type 'yes': ").strip() != "yes":
            print("not released")
            sys.exit(1)
        res = call("/arm_interface/estop_reset")
        print(("released: " if res.success else "REFUSED: ") + res.message)
        sys.exit(0 if res.success else 1)
    else:
        last = {}

        def show(key, text):
            if last.get(key) != text:
                last[key] = text
                print("%s  %s" % (rospy.get_time(), text))

        rospy.Subscriber("/arm_interface/estop_state", Bool, lambda m: show("e", "e-stop %s" % ("ENGAGED" if m.data else "released")))
        rospy.Subscriber("/arm_interface/drive_state", String, lambda m: show("d", "drive state %s" % m.data))
        rospy.Subscriber("/arm_interface/motor_status", DiagnosticArray,
                         lambda m: show("m", "motors: " + (", ".join("%s %s" % (s.name.split("/")[-1], LEVELS.get(s.level, "?")) for s in m.status if s.level) or "all OK")))
        print("watching (Ctrl-C to stop)")
        rospy.spin()


if __name__ == "__main__":
    main()
