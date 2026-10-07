"""Tests of the marker table: parsing, per-marker sizes and dictionaries, validation, and the shipped config."""
import os
import unittest

from arm_camera import aruco
from arm_perception import marker_config as mc

HERE = os.path.dirname(os.path.abspath(__file__))
CAPACITY = lambda name: aruco.get_dictionary(name).bytesList.shape[0]
SHIPPED = os.path.join(os.path.dirname(HERE), "config", "markers.yaml")
IDS_50MM = [984, 952, 864, 786, 650, 398, 167, 267, 232, 161, 137, 118, 102, 94]


class ParseTest(unittest.TestCase):
    def test_sizes_per_marker_and_the_default_dictionary(self):
        cfg = mc.MarkerConfig.from_dict({"dictionary": "4X4_1000", "markers": [{"id": 0, "size": 0.1}, {"id": 984, "size": 0.05}]})
        self.assertEqual(cfg.spec("4X4_1000", 0), mc.MarkerSpec(0, 0.1, "4X4_1000"))
        self.assertEqual(cfg.spec("4X4_1000", 984).size, 0.05)
        self.assertEqual(cfg.dictionaries(), ["4X4_1000"])

    def test_an_unlisted_marker_or_one_from_another_dictionary_is_not_found(self):
        cfg = mc.MarkerConfig.from_dict({"dictionary": "4X4_1000", "markers": [{"id": 0, "size": 0.1}]})
        self.assertIsNone(cfg.spec("4X4_1000", 5))
        self.assertIsNone(cfg.spec("6X6_250", 0))

    def test_an_entry_may_name_its_own_dictionary(self):
        cfg = mc.MarkerConfig.from_dict({"dictionary": "4X4_1000", "markers": [
            {"id": 0, "size": 0.1}, {"id": 7, "size": 0.05, "dictionary": "ARUCO_ORIGINAL"}]})
        self.assertEqual(cfg.dictionaries(), ["4X4_1000", "ARUCO_ORIGINAL"])
        self.assertEqual(cfg.spec("ARUCO_ORIGINAL", 7).size, 0.05)
        self.assertIsNone(cfg.spec("4X4_1000", 7))

    def test_invalid_tables_are_refused(self):
        bad = [{"markers": [{"id": 0, "size": 0.1}]},                                                  # no dictionary at all
               {"dictionary": "4X4_50", "markers": []},                                                # empty
               {"dictionary": "4X4_50", "markers": [{"id": 0, "size": 0.1}, {"id": 0, "size": 0.05}]},  # duplicate id
               {"dictionary": "4X4_50", "markers": [{"id": 1, "size": 0.0}]},                           # zero size
               {"dictionary": "4X4_50", "markers": [{"id": 1}]}]                                       # no size
        for d in bad:
            with self.assertRaises(ValueError, msg=str(d)):
                mc.MarkerConfig.from_dict(d)

    def test_an_id_beyond_the_dictionary_capacity_is_refused(self):
        with self.assertRaises(ValueError) as cm:
            mc.MarkerConfig.from_dict({"dictionary": "4X4_50", "markers": [{"id": 984, "size": 0.05}]}, CAPACITY)
        self.assertIn("4X4_50", str(cm.exception))
        mc.MarkerConfig.from_dict({"dictionary": "4X4_1000", "markers": [{"id": 984, "size": 0.05}]}, CAPACITY)


class ShippedConfigTest(unittest.TestCase):
    def test_the_shipped_table_has_the_100_mm_4x4_marker_and_the_fourteen_50_mm_5x5_ones(self):
        cfg = mc.MarkerConfig.load(SHIPPED, CAPACITY)
        self.assertEqual(cfg.spec("4X4_1000", 0).size, 0.100)
        small = sorted(i for i, s in cfg.specs.items() if s.size == 0.050)
        self.assertEqual(small, sorted(IDS_50MM))
        self.assertTrue(all(cfg.spec("ARUCO_ORIGINAL", i) is not None for i in IDS_50MM))
        self.assertEqual(len(cfg.specs), 1 + len(IDS_50MM))
        self.assertEqual(cfg.dictionaries(), ["4X4_1000", "ARUCO_ORIGINAL"])

    def test_id_0_is_the_same_marker_in_the_50_and_1000_id_dictionaries(self):
        import numpy as np
        self.assertTrue(np.array_equal(aruco.render_marker("4X4_50", 0, 80), aruco.render_marker("4X4_1000", 0, 80)))


if __name__ == "__main__":
    unittest.main()
