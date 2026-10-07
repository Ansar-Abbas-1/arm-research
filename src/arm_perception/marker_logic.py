"""Pose of a marker in the robot base frame, from camera detections and the camera pose (no ROS).

Each detection gives T_cam_marker (solvePnP). With the camera pose in the base frame AT THE IMAGE TIME,
T_base_marker = T_base_cam @ T_cam_marker. The camera is carried by the arm and an image reaches the
computer 50 to 100 ms after exposure, so a pose taken while the arm moves is wrong by that much motion:
the tracker therefore reports a marker only when the camera has been still over the whole window, and
averages the detections of the window (robustly, with outlier rejection) to report the spread too.

Marker frame: origin at the marker centre, x right and y up in the marker's picture, z out of the marker
toward the viewer (a marker lying face up on a table has z up).
"""
from collections import defaultdict, deque

import numpy as np

from arm_camera import charuco
from arm_camera.geometry import rotation_error_deg


def base_pose(T_base_cam, T_cam_marker):
    """T_base_marker from the camera pose in the base frame and the marker pose in the camera frame."""
    return np.asarray(T_base_cam) @ np.asarray(T_cam_marker)


def camera_motion(T_a, T_b):
    """(translation [mm], rotation [deg]) between two camera poses."""
    return 1000.0 * float(np.linalg.norm(T_a[:3, 3] - T_b[:3, 3])), rotation_error_deg(T_a[:3, :3], T_b[:3, :3])


class MarkerTracker:
    """Collects detections over a sliding window of image stamps and reports averaged poses.

    window_s               length of the window [s] (image stamps, not wall time)
    still_translation_mm   the camera must stay within this of its latest pose over the whole window
    still_rotation_deg     ... and within this rotation (heuristics: encoder quantisation alone jitters the
                           camera pose by about 1 mm / 0.1 deg; tighten them with a still arm)
    min_samples            detections needed in the window to report a marker
    """

    def __init__(self, window_s=0.5, still_translation_mm=2.0, still_rotation_deg=0.3, min_samples=3):
        self.window_s = window_s
        self.still_translation_mm = still_translation_mm
        self.still_rotation_deg = still_rotation_deg
        self.min_samples = min_samples
        self.camera = deque()                      # (stamp, T_base_cam)
        self.detections = defaultdict(deque)       # id -> (stamp, T_base_marker, ambiguity, rms, distance)

    def add_camera_pose(self, stamp, T_base_cam):
        self.camera.append((stamp, np.asarray(T_base_cam)))

    def add_detection(self, marker_id, stamp, T_base_marker, ambiguity=0.0, rms=0.0, distance=0.0):
        self.detections[marker_id].append((stamp, np.asarray(T_base_marker), ambiguity, rms, distance))

    def _purge(self, now):
        while self.camera and self.camera[0][0] < now - self.window_s:
            self.camera.popleft()
        for q in self.detections.values():
            while q and q[0][0] < now - self.window_s:
                q.popleft()

    def camera_moving(self):
        """True when the camera moved more than the still thresholds relative to its latest pose in the window."""
        if len(self.camera) < 2:
            return False
        latest = self.camera[-1][1]
        for _, T in self.camera:
            mm, deg = camera_motion(T, latest)
            if mm > self.still_translation_mm or deg > self.still_rotation_deg:
                return True
        return False

    def report(self, now):
        """One dict per marker seen within the window (sorted by id):
        id, valid, reason, T_base_marker (averaged), n, std_mm [x, y, z], max_dev_mm, rot_spread_deg,
        ambiguity (worst IPPE ratio of the window), distance (camera to marker, mean) and stamp (newest sample)."""
        self._purge(now)
        moving = self.camera_moving()
        out = []
        for mid in sorted(self.detections):
            q = self.detections[mid]
            if not q:
                continue
            item = {"id": mid, "valid": False, "reason": "", "T_base_marker": None, "n": len(q),
                    "std_mm": [0.0, 0.0, 0.0], "max_dev_mm": 0.0, "rot_spread_deg": 0.0,
                    "ambiguity": max(s[2] for s in q), "distance": float(np.mean([s[4] for s in q])), "stamp": q[-1][0]}
            if moving:
                item["reason"] = "camera moving"
            elif len(q) < self.min_samples:
                item["reason"] = "only %d of %d samples" % (len(q), self.min_samples)
            else:
                est = charuco.robust_mount([s[1] for s in q])      # robust average of transforms
                item.update(valid=True, T_base_marker=est.X, n=int(est.inliers.sum()), std_mm=[float(v) for v in est.translation_std_mm],
                            max_dev_mm=float(est.translation_spread_mm), rot_spread_deg=float(est.rotation_spread_deg))
                if not est.inliers.all():
                    item["reason"] = "%d of %d samples rejected as outliers" % ((~est.inliers).sum(), len(est.inliers))
            out.append(item)
        return out


# ---- report (JSON on a topic, text in the CLI) ----------------------------------------------------------------

def report_dict(items, base_frame, camera_moving, stamp, specs=None, ignored=None):
    """JSON-ready dict of a MarkerTracker.report: pose as position, quaternion and yaw/pitch/roll [deg].
    specs: id -> MarkerSpec (adds size and dictionary to each marker); ignored: list of (id, dictionary) seen but
    not in the marker table."""
    from arm_camera.geometry import quat_from_matrix, ypr_from_matrix
    markers = []
    for it in items:
        m = {k: it[k] for k in ("id", "valid", "reason", "n", "std_mm", "max_dev_mm", "rot_spread_deg", "ambiguity", "distance", "stamp")}
        if specs and it["id"] in specs:
            m["size"], m["dictionary"] = specs[it["id"]].size, specs[it["id"]].dictionary
        T = it["T_base_marker"]
        if T is not None:
            m["position"] = [float(v) for v in T[:3, 3]]
            m["quaternion_xyzw"] = [float(v) for v in quat_from_matrix(T[:3, :3])]
            m["ypr_deg"] = [float(np.degrees(v)) for v in ypr_from_matrix(T[:3, :3])]
        markers.append(m)
    return {"stamp": stamp, "base_frame": base_frame, "camera_moving": bool(camera_moving), "markers": markers,
            "ignored": [{"id": int(i), "dictionary": d} for i, d in sorted(ignored or [])]}


def format_report(rep):
    """Human-readable text of report_dict (used by marker_pose_cli.py)."""
    lines = ["pose in %s (camera %s)" % (rep["base_frame"], "MOVING" if rep["camera_moving"] else "still")]
    if not rep["markers"]:
        lines.append("  no marker seen in the last window")
    for m in rep["markers"]:
        if m["valid"]:
            x, y, z = m["position"]
            yaw, pitch, roll = m["ypr_deg"]
            size = " (%.0f mm)" % (1000 * m["size"]) if "size" in m else ""
            lines.append("  marker %d%s: x=%.3f y=%.3f z=%.3f m | yaw=%.1f pitch=%.1f roll=%.1f deg" % (m["id"], size, x, y, z, yaw, pitch, roll))
            lines.append("            %d samples, std x/y/z = %.1f/%.1f/%.1f mm, worst %.1f mm, rotation spread %.2f deg, "
                         "camera distance %.3f m%s" % (m["n"], m["std_mm"][0], m["std_mm"][1], m["std_mm"][2], m["max_dev_mm"],
                                                        m["rot_spread_deg"], m["distance"],
                                                        ", ORIENTATION AMBIGUOUS (ratio %.2f)" % m["ambiguity"] if m["ambiguity"] > 0.7 else ""))
            if m["reason"]:
                lines.append("            note: " + m["reason"])
        else:
            lines.append("  marker %d: not reported (%s)" % (m["id"], m["reason"]))
    if rep.get("ignored"):
        lines.append("  seen but not in the marker table (ignored): " + ", ".join("%d (%s)" % (i["id"], i["dictionary"]) for i in rep["ignored"]))
    return "\n".join(lines)
