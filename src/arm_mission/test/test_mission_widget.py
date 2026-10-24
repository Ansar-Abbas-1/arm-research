"""Widget test of MissionWidget, headless (QT_QPA_PLATFORM=offscreen), with a recording `send`."""
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "arm_grasp", "src"))

from python_qt_binding.QtWidgets import QApplication   # noqa: E402

from arm_mission.mission_widget import MissionWidget    # noqa: E402

APP = QApplication.instance() or QApplication(sys.argv)


def state(go=False, busy=False, stale=False, plan="none", run="IDLE", bias=(-0.01, 0.01, 0.0), height=0.09):
    return {"plants": [{"index": 0, "confidence": 0.91, "position": [0.3, 0.1, -0.5], "axis_mm": 12.0, "offset": [0.1, 0, 0.1]},
                       {"index": 1, "confidence": 0.8, "position": [0.2, -0.1, -0.5], "axis_mm": 80.0, "offset": [0, 0, 0]}],
            "selected": 0, "snapshot_stale": stale, "live_count": 2, "marker_visible": True,
            "settings": {"approach": "45", "bias": list(bias), "height": height, "offset": [0.1, 0, 0.1]},
            "plan": {"state": plan, "message": "", "duration_s": 21.5, "deviation_mm": 1.2, "current": True},
            "run": {"state": run, "message": "x"}, "estop": False, "go_enabled": go, "go_reason": "run the dry run first", "busy": busy}


class WidgetTest(unittest.TestCase):
    def setUp(self):
        self.calls, self.answer, self.confirms, self.confirm_answer = [], (True, "ok"), 0, True
        self.w = MissionWidget(self.send)
        self.w.confirm = self.confirm

    def send(self, action, index, values, text):
        self.calls.append((action, index, list(values), text))
        return self.answer

    def confirm(self, title, text):
        self.confirms += 1
        return self.confirm_answer

    def test_disabled_until_the_backend_publishes_except_stop(self):
        w = self.w
        self.assertFalse(w.refresh_button.isEnabled()); self.assertFalse(w.go_button.isEnabled()); self.assertFalse(w.dry_button.isEnabled())
        self.assertTrue(w.stop_button.isEnabled()); self.assertIn("waiting", w.status.text())

    def test_state_fills_the_widget(self):
        w = self.w
        w.set_state(state())
        self.assertEqual(w.plants.count(), 2); self.assertEqual(w.plants.currentRow(), 0)
        self.assertIn("12 mm from axis", w.plants.item(0).text())
        self.assertAlmostEqual(w.bias[0].value(), -10.0); self.assertAlmostEqual(w.bias[1].value(), 10.0); self.assertAlmostEqual(w.height.value(), 9.0)
        self.assertEqual(w.approach.currentText(), "45")
        self.assertIn("IDLE", w.status.text()); self.assertIn("2 in view", w.status.text())
        self.assertTrue(w.dry_button.isEnabled()); self.assertFalse(w.go_button.isEnabled())
        self.assertEqual(self.calls, [])                                  # filling from the state sends nothing

    def test_go_follows_the_gate_and_asks_for_confirmation(self):
        w = self.w
        w.set_state(state(go=True, plan="ok"))
        self.assertTrue(w.go_button.isEnabled()); self.assertIn("dry run ok", w.plan.text())
        self.confirm_answer = False
        w.go_button.click()
        self.assertEqual((self.confirms, self.calls), (1, []))
        self.confirm_answer = True
        w.go_button.click()
        self.assertEqual(self.calls, [("go", 0, [], "")])

    def test_buttons_send_their_commands_and_stop_needs_no_confirmation(self):
        w = self.w
        w.set_state(state())
        w.refresh_button.click(); w.dry_button.click(); w.stop_button.click(); w.plants.setCurrentRow(1)
        self.assertEqual([c[0] for c in self.calls], ["refresh", "dry_run", "stop", "select"])
        self.assertEqual(self.calls[3][1], 1); self.assertEqual(self.confirms, 0)

    def test_apply_sends_metres(self):
        w = self.w
        w.set_state(state())
        w.approach.setEditText("60"); w.bias[0].setValue(-20.0); w.bias[1].setValue(5.0); w.height.setValue(7.0)
        w.apply_button.click()
        self.assertEqual([c[0] for c in self.calls], ["set_approach", "set_bias", "set_height"])
        self.assertEqual(self.calls[0][3], "60")
        self.assertAlmostEqual(self.calls[1][2][0], -0.020); self.assertAlmostEqual(self.calls[1][2][1], 0.005)
        self.assertAlmostEqual(self.calls[2][2][0], 0.07)

    def test_an_unapplied_edit_survives_the_next_state(self):
        w = self.w
        w.set_state(state())
        w.height.setValue(6.0)
        w.set_state(state())
        self.assertAlmostEqual(w.height.value(), 6.0)
        w.apply_button.click()
        w.set_state(state())
        self.assertAlmostEqual(w.height.value(), 9.0)

    def test_busy_leaves_only_stop(self):
        w = self.w
        w.set_state(state(busy=True, run="EXECUTING"))
        for b in (w.refresh_button, w.apply_button, w.dry_button, w.go_button):
            self.assertFalse(b.isEnabled())
        self.assertTrue(w.stop_button.isEnabled())

    def test_stale_list_disables_dry_run_and_failed_plan_shows(self):
        w = self.w
        w.set_state(state(stale=True, plan="failed"))
        self.assertFalse(w.dry_button.isEnabled()); self.assertIn("FAILED", w.plan.text())

    def test_a_refusal_is_red_and_an_exception_is_reported(self):
        w = self.w
        w.set_state(state())
        self.answer = (False, "refused: busy")
        w.refresh_button.click()
        self.assertIn("refused", w.message.text()); self.assertIn("c62828", w.message.styleSheet())
        def boom(*a): raise RuntimeError("service missing")
        w.send = boom
        w.refresh_button.click()
        self.assertIn("service missing", w.message.text())


if __name__ == "__main__":
    unittest.main()
