"""rqt plugin: Mission Control. Run it standalone (rosrun arm_mission arm_mission_rqt) or from the rqt Plugins menu (Arm > Mission Control)."""
import json

import rospy
from python_qt_binding.QtCore import QObject, Signal
from rqt_gui_py.plugin import Plugin
from std_msgs.msg import String

from arm_gui.srv import MissionCommand
from arm_mission.mission_widget import MissionWidget

COMMAND_SERVICE = "/mission_control/command"
STATE_TOPIC = "/mission_control/state"


class _Bridge(QObject):
    state = Signal(object)                           # ROS callbacks arrive on a ROS thread: hand them to the GUI thread through a signal


class MissionPlugin(Plugin):
    def __init__(self, context):
        super(MissionPlugin, self).__init__(context)
        self.setObjectName("MissionControl")
        self.widget = MissionWidget(self.send)
        self.widget.setWindowTitle("Mission Control")
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + " (%d)" % context.serial_number())
        context.add_widget(self.widget)
        self.bridge = _Bridge()
        self.bridge.state.connect(self.widget.set_state)
        self.sub = rospy.Subscriber(STATE_TOPIC, String, self.on_state, queue_size=1)

    def on_state(self, msg):
        try:
            self.bridge.state.emit(json.loads(msg.data))
        except ValueError:
            pass

    def send(self, action, index=0, values=(), text=""):
        rospy.wait_for_service(COMMAND_SERVICE, timeout=1.0)
        r = rospy.ServiceProxy(COMMAND_SERVICE, MissionCommand)(action, index, list(values), text)
        return r.success, r.message

    def shutdown_plugin(self):
        self.sub.unregister()
