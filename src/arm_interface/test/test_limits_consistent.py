#!/usr/bin/env python3
"""URDF joint limits must lie within the hardware's encoder limits.

robot_interface clamps every command to config/ArmInterface.yaml's
encoder_min/encoder_max, while IK and trajectory generation plan against
urdf/arm.urdf's <limit> tags. If the URDF is looser than the encoder
bounds anywhere, the planner can produce poses the hardware silently
clips. The URDF may be tighter (safety margin), never looser.

Joints 1-5 only: joint_6 (gripper) is prismatic in the URDF, rotary in the
YAML, and related by gripper_conversion.h instead (see its own test).
"""
import math
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOL = 1e-9


def hardware_limits_rad(cfg, i):
    k = 2.0 * math.pi / cfg["dxl_resolution"]
    a = cfg["sign"][i] * (cfg["encoder_min"][i] - cfg["bias"][i]) * k
    b = cfg["sign"][i] * (cfg["encoder_max"][i] - cfg["bias"][i]) * k
    return min(a, b), max(a, b)


class LimitsConsistent(unittest.TestCase):
    def test_urdf_within_hardware_limits(self):
        with open(os.path.join(PKG, "config", "ArmInterface.yaml")) as f:
            cfg = yaml.safe_load(f)
        root = ET.parse(os.path.join(PKG, "urdf", "arm.urdf")).getroot()
        limits = {j.get("name"): j.find("limit") for j in root.iter("joint") if j.find("limit") is not None}
        for i in range(5):
            name = "joint_%d" % (i + 1)
            lo, hi = hardware_limits_rad(cfg, i)
            lim = limits[name]
            u_lo, u_hi = float(lim.get("lower")), float(lim.get("upper"))
            self.assertGreaterEqual(u_lo, lo - TOL, "%s URDF lower %.4f below hardware %.4f" % (name, u_lo, lo))
            self.assertLessEqual(u_hi, hi + TOL, "%s URDF upper %.4f above hardware %.4f" % (name, u_hi, hi))


if __name__ == "__main__":
    unittest.main()
