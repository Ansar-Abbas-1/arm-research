"""Tests of the run recorder and report on synthetic status streams (no ROS)."""
import os
import tempfile
import unittest

import numpy as np

from arm_servo import run_log, run_report


def status(state, err=(0.0, 0.0, 0.0), reason="", target=118, v=(0.0, 0.0, 0.0), age=0.1, ee=0.3, cam=0.4, left=False):
    return {"state": state, "reason": reason, "left_view": left, "target_id": target, "error_ee": list(err),
            "error_norm": float(np.linalg.norm(err)), "pitch_error_rad": 0.01, "camera_distance": cam, "ee_distance": ee,
            "v_cmd": list(v), "measurement_age": age, "ground_known": True}


def synthetic_run(final="DONE", left=False):
    """A converging run at 10 Hz: error 0.25 m -> ~3 mm, closing on the standoff, then the end state."""
    recs = []
    rec = run_log.RunRecorder(tail_s=0.5)
    t = 100.0
    out = None
    for i in range(12):                                          # idle, then selecting
        out = rec.feed(t, status("IDLE" if i < 3 else "SELECTING"), [0.0] * 5) or out
        t += 0.1
    for k in range(120):                                         # servoing
        e = 0.25 * np.exp(-k * 0.1) + 0.003
        speed = min(0.04, e)
        out = rec.feed(t, status("SERVOING", err=(0, 0, e), v=(0, 0, speed), ee=0.10 + e, cam=0.16 + e), [0.0, 0.01 * k, -0.01 * k, 0.0, 0.0]) or out
        t += 0.1
    for k in range(8):                                           # the end state and its tail
        out = rec.feed(t, status(final, err=(0, 0, 0.003), reason="" if final == "DONE" else "target marker lost", left=left, ee=0.103, cam=0.163), [0.0] * 5) or out
        t += 0.1
    return out


class RecorderTest(unittest.TestCase):
    def test_a_run_is_cut_from_selecting_to_a_second_after_the_end(self):
        run = synthetic_run()
        self.assertIsNotNone(run)
        self.assertEqual(run[0]["state"], "SELECTING")           # idle records before the start are not kept
        self.assertEqual(run[-1]["state"], "DONE")
        self.assertGreaterEqual(run[-1]["t"] - [r for r in run if r["state"] == "DONE"][0]["t"], 0.5)

    def test_nothing_is_recorded_while_idle(self):
        rec = run_log.RunRecorder()
        for i in range(30):
            self.assertIsNone(rec.feed(i * 0.1, status("IDLE")))
        self.assertIsNone(rec.records)

    def test_jsonl_round_trip(self):
        run = synthetic_run()
        path = os.path.join(tempfile.mkdtemp(), "sub", "run.jsonl")
        run_log.save_run(path, run)
        back = run_log.load_run(path)
        self.assertEqual(len(back), len(run))
        self.assertEqual(back[5]["error_ee"], run[5]["error_ee"])


class ReportTest(unittest.TestCase):
    def test_summary_of_a_converging_run(self):
        s = run_report.summarize(synthetic_run())
        self.assertEqual(s["outcome"], "DONE")
        self.assertEqual(s["target_id"], 118)
        self.assertAlmostEqual(s["initial_error_mm"], 253.0, delta=2.0)
        self.assertLess(s["final_error_mm"], 5.0)
        self.assertIsNotNone(s["time_to_5mm_s"])
        self.assertLess(s["time_to_20mm_s"], s["time_to_10mm_s"])
        self.assertLess(s["time_to_10mm_s"], s["time_to_5mm_s"])
        self.assertAlmostEqual(s["max_commanded_speed_mm_s"], 40.0, delta=0.5)
        self.assertAlmostEqual(s["min_ee_distance_m"], 0.103, delta=0.001)
        self.assertGreater(s["max_joint_speed_rad_s"], 0.0)

    def test_an_aborted_run_and_the_left_view_flag_show_in_the_text(self):
        text = run_report.format_summary(run_report.summarize(synthetic_run(final="ABORTED")))
        self.assertIn("outcome: ABORTED (target marker lost)", text)
        text = run_report.format_summary(run_report.summarize(synthetic_run(final="DONE", left=True)))
        self.assertIn("the marker left the view at the stand-off", text)

    def test_the_plot_is_written(self):
        path = os.path.join(tempfile.mkdtemp(), "run.png")
        run_report.plot_run(synthetic_run(), path)
        self.assertGreater(os.path.getsize(path), 5000)


if __name__ == "__main__":
    unittest.main()
