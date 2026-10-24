"""Splits the stream of visual servo status messages into runs and keeps them as JSON lines (no ROS).

A run starts when the state becomes SELECTING or SERVOING and ends `tail_s` after DONE or ABORTED. Each record is one
status message (error in the end-effector frame, speeds, distances, measurement age, state) plus the joint positions
at that moment, so a run can be summarised and plotted later (run_report.py), on the real arm or in simulation.
"""
import json
import os

ACTIVE = ("SELECTING", "SERVOING")
ENDED = ("DONE", "ABORTED")


class RunRecorder:
    def __init__(self, tail_s=1.0):
        self.tail_s = tail_s
        self.records = None
        self.ended_at = None

    def feed(self, t, status, q=None):
        """Feed one status dict (parsed JSON of /visual_servo/status) at time t. Returns the finished run (a list of
        records) when one has just ended, else None."""
        state = status.get("state")
        rec = dict(status)
        rec["t"] = float(t)
        rec["q"] = list(q) if q is not None else None
        if self.records is None:
            if state in ACTIVE:
                self.records = [rec]
                self.ended_at = None
            return None
        self.records.append(rec)
        if state in ENDED and self.ended_at is None:
            self.ended_at = t
        if state in ACTIVE:
            self.ended_at = None
        if self.ended_at is not None and t - self.ended_at >= self.tail_s:
            run, self.records, self.ended_at = self.records, None, None
            return run
        return None


def save_run(path, records):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        for r in records:
            f.write(json.dumps(r) + "\n")


def load_run(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]
