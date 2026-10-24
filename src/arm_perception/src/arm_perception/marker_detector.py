"""Per-frame marker detection against the marker table (OpenCV + numpy, no ROS).

One detector pass per dictionary in the table, each marker posed with ITS OWN size; markers that are seen but not
in the table are returned separately (a wrong dictionary shows up as ignored ids).
"""
from dataclasses import dataclass

import cv2
import numpy as np

from arm_camera import aruco


@dataclass
class Detection:
    id: int
    dictionary: str
    size: float
    T_cam_marker: np.ndarray
    rms: float
    ambiguity: float
    distance: float


class MarkerDetector:
    def __init__(self, config, quad_decimate=2.0, max_rms=2.0):
        self.cfg = config
        self.params = aruco.detector_parameters(quad_decimate=quad_decimate)
        self.max_rms = max_rms

    def detect(self, gray, K, D):
        """(detections of listed markers with an acceptable fit, [(id, dictionary)] seen but not listed)."""
        if gray.ndim == 3:
            gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        found, ignored = [], []
        for name in self.cfg.dictionaries():
            for d in aruco.detect(gray, name, self.params):
                spec = self.cfg.spec(name, d.marker_id)
                if spec is None:
                    ignored.append((d.marker_id, name))
                    continue
                T, rms, ratio = aruco.estimate_pose(d.corners, spec.size, K, D)
                if T is None or rms > self.max_rms:
                    continue
                found.append(Detection(d.marker_id, name, spec.size, T, rms, ratio, float(np.linalg.norm(T[:3, 3]))))
        return found, ignored
