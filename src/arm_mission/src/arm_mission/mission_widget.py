"""The mission control widget (Qt, no ROS): shows the state dict published on /mission_control/state and sends commands through `send`.

send(action, index=0, values=(), text="") -> (success, message). The rqt plugin wires it to the /mission_control/command service. GO asks for
confirmation (replaceable: `confirm`), STOP never does. Whether GO is enabled comes from the backend's go gate (state['go_enabled']), not from here.
"""
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (QComboBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QListWidget, QMessageBox,
                                         QPushButton, QVBoxLayout, QWidget)

from arm_mission import mission_logic as ml

APPROACHES = ["down", "60", "45", "30", "forward"]
GO_STYLE = "QPushButton { background-color: #2e7d32; color: white; font-weight: bold; font-size: 16px; padding: 10px; } QPushButton:disabled { background-color: #9e9e9e; }"
STOP_STYLE = "QPushButton { background-color: #c62828; color: white; font-weight: bold; font-size: 16px; padding: 10px; }"


class MissionWidget(QWidget):
    def __init__(self, send, parent=None):
        super(MissionWidget, self).__init__(parent)
        self.send = send
        self.confirm = lambda title, text: QMessageBox.question(self, title, text, QMessageBox.Yes | QMessageBox.Cancel, QMessageBox.Cancel) == QMessageBox.Yes
        self.state = {}
        self.dirty = False                         # the operator edited a setting that is not applied yet: the state must not overwrite it
        self.listed = None

        layout = QVBoxLayout(self)
        self.status = QLabel("waiting for /mission_control/state")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        box = QGroupBox("Plants (nearest to the gripper axis first)")
        v = QVBoxLayout(box)
        self.plants = QListWidget()
        self.plants.setMinimumHeight(90)
        v.addWidget(self.plants)
        self.refresh_button = QPushButton("Refresh plants")
        v.addWidget(self.refresh_button)
        layout.addWidget(box)

        box = QGroupBox("Settings")
        g = QGridLayout(box)
        self.approach = QComboBox()
        self.approach.setEditable(True)
        self.approach.addItems(APPROACHES)
        g.addWidget(QLabel("Approach (down | forward | angle deg)"), 0, 0, 1, 3)
        g.addWidget(self.approach, 0, 3)
        self.bias = []
        for i, name in enumerate(("radial", "tangential", "vertical")):
            s = QDoubleSpinBox()
            s.setRange(-80.0, 80.0)
            s.setDecimals(1)
            s.setSingleStep(1.0)
            s.setSuffix(" mm")
            self.bias.append(s)
            g.addWidget(QLabel("Aim bias " + name), 1 + i, 0, 1, 3)
            g.addWidget(s, 1 + i, 3)
        self.height = QDoubleSpinBox()
        self.height.setRange(0.0, 50.0)
        self.height.setDecimals(1)
        self.height.setSingleStep(0.5)
        self.height.setSuffix(" cm")
        g.addWidget(QLabel("Pinch height above floor"), 4, 0, 1, 3)
        g.addWidget(self.height, 4, 3)
        self.apply_button = QPushButton("Apply settings")
        g.addWidget(self.apply_button, 5, 0, 1, 4)
        layout.addWidget(box)

        self.plan = QLabel("")
        self.plan.setWordWrap(True)
        layout.addWidget(self.plan)
        row = QHBoxLayout()
        self.dry_button = QPushButton("Dry run")
        self.go_button = QPushButton("GO")
        self.go_button.setStyleSheet(GO_STYLE)
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(STOP_STYLE)
        for b in (self.dry_button, self.go_button, self.stop_button):
            row.addWidget(b)
        layout.addLayout(row)
        self.message = QLabel("")
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        layout.addStretch(1)

        self.refresh_button.clicked.connect(lambda: self.command("refresh"))
        self.plants.currentRowChanged.connect(self.on_row)
        self.apply_button.clicked.connect(self.apply_settings)
        self.dry_button.clicked.connect(lambda: self.command("dry_run"))
        self.go_button.clicked.connect(self.click_go)
        self.stop_button.clicked.connect(lambda: self.command("stop"))
        for s in self.bias + [self.height]:
            s.valueChanged.connect(self.mark_dirty)
        self.approach.currentTextChanged.connect(self.mark_dirty)
        self.set_state({})

    # ---- operator actions --------------------------------------------------------------------------------------------------
    def say(self, text, error=False):
        self.message.setText(text)
        self.message.setStyleSheet("color: #c62828;" if error else "")

    def command(self, action, index=0, values=(), text=""):
        try:
            ok, msg = self.send(action, index, list(values), text)
        except Exception as e:                      # service missing / call failed
            ok, msg = False, "%s: %s" % (action, e)
        self.say(msg, not ok)
        return ok

    def mark_dirty(self, *_):
        self.dirty = True

    def on_row(self, row):
        if row < 0 or self.listed is None or row == self.state.get("selected", -1):
            return
        self.command("select", index=row)

    def apply_settings(self):
        ok = self.command("set_approach", text=self.approach.currentText().strip())
        ok = self.command("set_bias", values=[s.value() / 1000.0 for s in self.bias]) and ok
        ok = self.command("set_height", values=[self.height.value() / 100.0]) and ok
        if ok:
            self.dirty = False
            self.say("Settings applied. Refresh the plants (the height moves their plane) and repeat the dry run.")

    def click_go(self):
        s = self.state.get("settings", {})
        sel = self.state.get("selected", -1)
        text = "Run the REAL grasp of plant #%d?\napproach %s, bias %s mm, pinch height %.1f cm.\nThe arm and gripper will move." % (
            sel, s.get("approach", "?"), ", ".join("%.0f" % (1000 * v) for v in s.get("bias", [])), 100 * s.get("height", 0.0))
        if self.confirm("Run the grasp", text):
            self.command("go")

    # ---- state ---------------------------------------------------------------------------------------------------------------
    def set_state(self, state):
        self.state = state
        busy = state.get("busy", False)
        have = bool(state)
        self.status.setText(ml.run_text(state) + ("" if not have else "  |  marker %s, %s in view" % ("seen" if state.get("marker_visible") else "NOT seen", state.get("live_count"))) if have else
                            "waiting for /mission_control/state (is the backend running?)")
        self.plan.setText(ml.plan_text(state))
        plants = state.get("plants", [])
        labels = [ml.plant_label(p) for p in plants]
        if labels != self.listed:
            self.plants.blockSignals(True)
            self.plants.clear()
            self.plants.addItems(labels)
            self.plants.blockSignals(False)
            self.listed = labels
        sel = state.get("selected", -1)
        if self.plants.currentRow() != sel:
            self.plants.blockSignals(True)
            self.plants.setCurrentRow(sel)
            self.plants.blockSignals(False)
        if state.get("snapshot_stale"):
            self.plants.setStyleSheet("color: #9e9e9e;")
        else:
            self.plants.setStyleSheet("")
        st = state.get("settings")
        if st and not self.dirty:
            for s in self.bias + [self.height, self.approach]:
                s.blockSignals(True)
            for s, v in zip(self.bias, st.get("bias", [0, 0, 0])):
                s.setValue(1000.0 * v)
            self.height.setValue(100.0 * st.get("height", 0.0))
            idx = self.approach.findText(str(st.get("approach", "")))
            if idx >= 0:
                self.approach.setCurrentIndex(idx)
            else:
                self.approach.setEditText(str(st.get("approach", "")))
            for s in self.bias + [self.height, self.approach]:
                s.blockSignals(False)
        self.refresh_button.setEnabled(have and not busy)
        self.plants.setEnabled(have and not busy)
        self.apply_button.setEnabled(have and not busy)
        self.dry_button.setEnabled(have and not busy and sel >= 0 and not state.get("snapshot_stale", False))
        self.go_button.setEnabled(bool(state.get("go_enabled", False)))
        self.go_button.setToolTip(state.get("go_reason", ""))
        self.stop_button.setEnabled(True)               # STOP is always live
