"""Arm sag: what the logged measurements say and the feed-forward offset that compensates it (numpy and json only, no ROS).

The arm sags under gravity: a pose it was commanded to reaches the end effector some centimetres off (mostly lower). The grasp sequence measures the end
effector after each move (grasp_sequencer_node, CorrectPosition) and appends one JSON line per measurement to a log:

    {"time", "run", "mode" real|probe, "key" PRE|GRASP|LIFT, "step" 0 = first measurement of that position, "approach_deg", "goal" [x, y, z] (the true
     waypoint), "aim" [x, y, z] (where the arm was commanded to go), "measured" [x, y, z], "joints", ...}

The SAG of a move is s = measured - aim (the displacement the arm adds to what it was told): unlike measured - goal it does not depend on any
compensation that was already applied, so samples taken with and without the model agree. The model is the mean sag per (approach angle bucket, key);
to land on a waypoint the arm has to be told to go to waypoint - s, so the feed-forward offset is -gain * s. It is constant per leg and approach angle
(it ignores how the sag changes with the target position: a fit over position needs many more samples).

The sag is stored in the ARM'S OWN FRAME, not the base frame: radial (outward from the base axis, at the waypoint's azimuth), tangential and vertical. The
tangential part is signed by the direction joint 1 travels (from the azimuth the run started at to the waypoint's): a joint that stops short of its goal
under-shoots in the direction of travel, so that error mirrors when the target is on the other side of the start. (Measured 2026-10-14 at azimuth -40 degrees:
+12 mm tangential at every leg, i.e. about 1.4 degrees of joint 1 short of its goal.) Heuristic: the tangential part ramps from 0 to full over TRAVEL_RAMP rad of
joint 1 travel, since a move too small to break the friction has no such error.
"""
import json

import numpy as np

KEYS = ("PRE", "GRASP", "LIFT")
TRAVEL_RAMP = 0.05      # rad of joint 1 travel over which the tangential sag ramps in (FLAGGED guess)


def arm_frame(azimuth, azimuth_start):
    """(radial, tangent, vertical) unit vectors of the arm's frame at `azimuth`; the tangent points along the joint 1 travel (sign and size of
    clip((azimuth - azimuth_start) / TRAVEL_RAMP, -1, 1) are folded in, so it fades to zero for a move that is too small to matter)."""
    u = np.array([np.cos(azimuth), np.sin(azimuth), 0.0])
    v = np.array([-np.sin(azimuth), np.cos(azimuth), 0.0])
    return u, v * float(np.clip((azimuth - azimuth_start) / TRAVEL_RAMP, -1.0, 1.0)), np.array([0.0, 0.0, 1.0])


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


def sag_samples(rows, modes=("real", "probe"), max_z=None):
    """[(approach_deg, key, local_sag [radial, tangential-along-travel, vertical], valid [3 bool])] from the FIRST measurement (step 0) of every position of every
    run; rows without a valid measurement are skipped. step > 0 measurements follow corrective moves that were aimed at the already compensated point: the first
    one is the cleanest (one move from a known start), so only those are used. The azimuth is the waypoint's (`goal`), the start azimuth is q_start[0] when the
    row has it, else 0 (the viewing pose). The tangential component is valid only where joint 1 travelled at least TRAVEL_RAMP. The last element of a sample is
    the target point's (radius from the base axis, base-frame z), which gives the model its validity range; max_z drops samples whose target is higher than that
    (a model fitted at grasp heights must not be built from samples far above them)."""
    out = []
    for r in rows:
        try:
            if r.get("mode", "real") not in modes or int(r["step"]) != 0 or r["key"] not in KEYS:
                continue
            s = np.asarray(r["measured"], float) - np.asarray(r["aim"], float)
            goal = np.asarray(r["goal"], float)
            if s.shape != (3,) or not np.all(np.isfinite(s)) or goal.shape != (3,):
                continue
            tgt = np.asarray(r["target"], float) if r.get("target") else goal
            if max_z is not None and tgt[2] > max_z:
                continue
            az = float(np.arctan2(goal[1], goal[0]))
            az0 = float(r["q_start"][0]) if r.get("q_start") else 0.0
            u, w, z = arm_frame(az, az0)
            travel = float(np.linalg.norm(w))
            v = np.array([-np.sin(az), np.cos(az), 0.0])
            sign = np.sign(az - az0)
            local = np.array([s @ u, (s @ v) * sign, s @ z])
            out.append((float(r["approach_deg"]), r["key"], local, np.array([True, travel >= 1.0 - 1e-9, True]), (float(np.hypot(tgt[0], tgt[1])), float(tgt[2]))))
        except (KeyError, TypeError, ValueError, IndexError):
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
        for angle, key, s, valid, pos in samples:
            centre = round(angle / bucket_deg) * bucket_deg
            acc.setdefault(centre, {}).setdefault(key, []).append((s, valid, pos))
        buckets = {}
        for centre, keys in acc.items():
            buckets[centre] = {}
            for k, items in keys.items():
                L, M = np.array([i[0] for i in items]), np.array([i[1] for i in items], bool)
                n_c = M.sum(axis=0)
                mean = [float(L[M[:, c], c].mean()) if n_c[c] else 0.0 for c in range(3)]
                std = [float(L[M[:, c], c].std(ddof=1)) if n_c[c] > 1 else 0.0 for c in range(3)]
                P = np.array([i[2] for i in items])
                buckets[centre][k] = {"mean": mean, "std": std, "n": len(items), "n_tangential": int(n_c[1]),
                                      "radius_range": [float(P[:, 0].min()), float(P[:, 0].max())], "z_range": [float(P[:, 1].min()), float(P[:, 1].max())]}
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

    def covers(self, key, approach_deg, radius, z, margin_radius=0.05, margin_z=0.04, min_n=2, max_distance=None):
        """True when (radius, z) of the target is inside the range the sag of `key` was measured over, widened by the margins. A model without recorded ranges
        (an old file) covers everything. The sag changes with the pose: measured 2026-10-14, the model fitted at grasp heights (0.05 to 0.09 m above the floor)
        made the error WORSE 0.25 m above the floor (23 mm instead of 17 mm at PRE), so it must not be applied there."""
        max_distance = self.bucket_deg if max_distance is None else max_distance
        best = None
        for centre, keys in self.buckets.items():
            d = abs(centre - approach_deg)
            if key in keys and keys[key]["n"] >= min_n and d <= max_distance and (best is None or d < best[0]):
                best = (d, keys[key])
        if best is None or "z_range" not in best[1]:
            return True
        (r0, r1), (z0, z1) = best[1]["radius_range"], best[1]["z_range"]
        return bool(r0 - margin_radius <= radius <= r1 + margin_radius and z0 - margin_z <= z <= z1 + margin_z)

    def offsets(self, approach_deg, azimuth=0.0, azimuth_start=0.0, gain=1.0, min_n=2, max_offset=0.08, target_radius=None, target_z=None, tangent_gain=1.0):
        """{key: feed-forward offset [m] in the base frame to ADD to that waypoint} = -gain x the mean sag turned from the arm's frame (at `azimuth`, joint 1
        travelling from `azimuth_start`) into the base frame, each limited to max_offset in length. LIFT is not measured by a real run: it uses its own samples
        when a probe measured it, else the GRASP sag. With target_radius and target_z a leg whose target lies outside the measured range (see covers) gets
        a zero offset. tangent_gain scales ONLY the tangential part: it is measured at azimuths -40, 0 and +40 degrees but not in between, and its dependence on the
        azimuth is not the simple sign-of-travel that the model assumes (+12 mm at -40, +8 mm at 0, -12 mm at +40, along +azimuth)."""
        out = {}
        u, w, z = arm_frame(azimuth, azimuth_start)
        for key in KEYS:
            if target_radius is not None and not self.covers("GRASP" if key == "LIFT" and self.sag("LIFT", approach_deg, min_n)[1] == 0 else key, approach_deg, target_radius, target_z, min_n=min_n):
                out[key] = np.zeros(3)
                continue
            s, n = self.sag(key, approach_deg, min_n)
            if n == 0 and key == "LIFT":
                s, n = self.sag("GRASP", approach_deg, min_n)
            off = -float(gain) * (s[0] * u + float(tangent_gain) * s[1] * w + s[2] * z)
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
