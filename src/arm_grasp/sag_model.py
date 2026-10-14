"""Arm sag: what the logged measurements say and the feed-forward offset that compensates it (numpy and json only, no ROS).

The arm sags under gravity: a pose it was commanded to reaches the end effector some centimetres off (mostly lower). The grasp sequence measures the end
effector after each move (grasp_sequencer_node, CorrectPosition) and appends one JSON line per measurement to a log:

    {"time", "run", "mode" real|probe, "key" PRE|GRASP|LIFT, "step" 0 = first measurement of that position, "approach_deg", "goal" [x, y, z] (the true
     waypoint), "aim" [x, y, z] (where the arm was commanded to go), "measured" [x, y, z], "joints", ...}

The SAG of a move is s = measured - aim (the displacement the arm adds to what it was told): unlike measured - goal it does not depend on any
compensation that was already applied, so samples taken with and without the model agree. The model is the mean sag per (approach angle bucket, key);
to land on a waypoint the arm has to be told to go to waypoint - s, so the feed-forward offset is -gain * s. It is constant per leg and approach angle
(it ignores how the sag changes with the target position: a fit over position needs many more samples).
"""
import json

import numpy as np

KEYS = ("PRE", "GRASP", "LIFT")


def read_rows(lines):
    """Parses an iterable of JSON lines; lines that are not valid JSON objects are skipped (a log cut mid-write)."""
    rows = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if isinstance(row, dict):
            rows.append(row)
    return rows


def sag_samples(rows, modes=("real", "probe")):
    """[(approach_deg, key, sag_vector)] from the FIRST measurement (step 0) of every position of every run; rows without a valid measurement are
    skipped. step > 0 measurements follow corrective moves that were aimed at the already compensated point: they are samples too, but the
    first one is the cleanest (one move from a known start), so only those are used."""
    out = []
    for r in rows:
        try:
            if r.get("mode", "real") not in modes or int(r["step"]) != 0 or r["key"] not in KEYS:
                continue
            s = np.asarray(r["measured"], float) - np.asarray(r["aim"], float)
            if s.shape != (3,) or not np.all(np.isfinite(s)):
                continue
            out.append((float(r["approach_deg"]), r["key"], s))
        except (KeyError, TypeError, ValueError):
            continue
    return out


class SagModel(object):
    """Mean sag per approach-angle bucket and key. `buckets` maps a bucket centre (degrees) to {key: {"mean": [3], "std": [3], "n": int}}."""

    def __init__(self, buckets=None, bucket_deg=15.0):
        self.buckets = buckets or {}
        self.bucket_deg = float(bucket_deg)

    @classmethod
    def fit(cls, samples, bucket_deg=15.0):
        acc = {}
        for angle, key, s in samples:
            centre = round(angle / bucket_deg) * bucket_deg
            acc.setdefault(centre, {}).setdefault(key, []).append(s)
        buckets = {}
        for centre, keys in acc.items():
            buckets[centre] = {k: {"mean": np.mean(v, axis=0).tolist(), "std": (np.std(v, axis=0, ddof=1) if len(v) > 1 else np.zeros(3)).tolist(), "n": len(v)}
                               for k, v in keys.items()}
        return cls(buckets, bucket_deg)

    def to_dict(self):
        return {"bucket_deg": self.bucket_deg, "buckets": {("%g" % c): v for c, v in sorted(self.buckets.items())}}

    @classmethod
    def from_dict(cls, d):
        return cls({float(c): v for c, v in d.get("buckets", {}).items()}, d.get("bucket_deg", 15.0))

    def sag(self, key, approach_deg, min_n=2, max_distance=None):
        """(mean sag vector, n) for the bucket nearest the approach angle (within max_distance degrees, default one bucket width) that has at least
        min_n samples for `key`; (zeros, 0) otherwise."""
        max_distance = self.bucket_deg if max_distance is None else max_distance
        best = None
        for centre, keys in self.buckets.items():
            d = abs(centre - approach_deg)
            if key in keys and keys[key]["n"] >= min_n and d <= max_distance and (best is None or d < best[0]):
                best = (d, keys[key])
        if best is None:
            return np.zeros(3), 0
        return np.asarray(best[1]["mean"], float), int(best[1]["n"])

    def offsets(self, approach_deg, gain=1.0, min_n=2, max_offset=0.08):
        """{key: feed-forward offset [m] to ADD to that waypoint} = -gain x mean sag, each limited to max_offset in length. LIFT is not measured by a real
        run: it uses its own samples when a probe measured it, else the GRASP sag."""
        out = {}
        for key in KEYS:
            s, n = self.sag(key, approach_deg, min_n)
            if n == 0 and key == "LIFT":
                s, n = self.sag("GRASP", approach_deg, min_n)
            off = -float(gain) * s
            norm = float(np.linalg.norm(off))
            out[key] = off * (max_offset / norm) if norm > max_offset else off
        return out


def load(path):
    with open(path) as f:
        return SagModel.from_dict(json.load(f))


def save(model, path):
    with open(path, "w") as f:
        json.dump(model.to_dict(), f, indent=1)


def grid_points(azimuth_deg, radii, heights, floor_z):
    """Free-air test points in the base frame: for every radius [m] (horizontal distance from the base axis) and height [m above the floor] at the given
    azimuth: [(radius, height, np.array([x, y, z]))], nearest radius and lowest first. The sag depends on the radius and the height (gravity acts in the
    arm's own vertical plane), hardly on the azimuth, so a test row at a free azimuth stands for the same radius and height anywhere else."""
    a = np.radians(float(azimuth_deg))
    return [(float(r), float(h), np.array([r * np.cos(a), r * np.sin(a), floor_z + h])) for r in sorted(radii) for h in sorted(heights)]
