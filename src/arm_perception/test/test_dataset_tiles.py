#!/usr/bin/env python3
"""Unit tests of dataset_tiles: polygon clipping, the crop grid and the label crops."""
import unittest

import numpy as np

from arm_perception import dataset_tiles as dt

SQUARE = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], float)


class ClipTest(unittest.TestCase):
    def test_area(self):
        self.assertAlmostEqual(dt.polygon_area(SQUARE), 1600.0)
        self.assertEqual(dt.polygon_area(SQUARE[:2]), 0.0)

    def test_inside_unchanged(self):
        c = dt.clip_polygon(SQUARE, 0, 0, 100, 100)
        self.assertAlmostEqual(dt.polygon_area(c), 1600.0)

    def test_half_cut(self):
        c = dt.clip_polygon(SQUARE, 30, 0, 100, 100)                 # keeps x in [30, 50]
        self.assertAlmostEqual(dt.polygon_area(c), 20 * 40)
        self.assertAlmostEqual(c[:, 0].min(), 30.0)

    def test_corner_cut(self):
        c = dt.clip_polygon(SQUARE, 30, 30, 100, 100)
        self.assertAlmostEqual(dt.polygon_area(c), 400.0)

    def test_outside_is_empty(self):
        self.assertEqual(len(dt.clip_polygon(SQUARE, 60, 60, 100, 100)), 0)

    def test_triangle_clipped_exactly(self):
        tri = np.array([[0, 0], [40, 0], [0, 40]], float)           # area 800; keep x <= 20: 800 - 200 = 600
        self.assertAlmostEqual(dt.polygon_area(dt.clip_polygon(tri, -5, -5, 20, 100)), 600.0)


class GridTest(unittest.TestCase):
    def test_covers_the_axis_with_the_last_tile_flush(self):
        s = dt.grid_origins(1000, 640, 320)
        self.assertEqual(s[0], 0)
        self.assertEqual(s[-1], 1000 - 640)
        self.assertTrue(all(b - a <= 320 for a, b in zip(s, s[1:])))

    def test_short_axis_is_one_tile(self):
        self.assertEqual(dt.grid_origins(500, 640, 320), [0])

    def test_exact_multiple_has_no_duplicate(self):
        self.assertEqual(dt.grid_origins(960, 640, 320), [0, 320])


class LabelsTest(unittest.TestCase):
    def test_scaled_and_shifted(self):
        out = dt.tile_labels([(1, SQUARE)], 2.0, 0, 0, 640)           # square at 20..100 after scaling
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(dt.polygon_area(out[0][1]), 6400.0)

    def test_origin_shift(self):
        out = dt.tile_labels([(1, SQUARE)], 1.0, 20, 20, 640)         # clipped to 20..50 -> 30 x 30 shifted to the crop origin
        self.assertAlmostEqual(dt.polygon_area(out[0][1]), 900.0)
        self.assertAlmostEqual(out[0][1][:, 0].min(), 0.0)

    def test_sliver_is_dropped(self):
        self.assertEqual(dt.tile_labels([(1, SQUARE)], 1.0, 45, 0, 640, min_ratio=0.25), [])   # 5 of 40 px inside: 12.5 % < 25 %

    def test_class_is_kept(self):
        self.assertEqual(dt.tile_labels([(0, SQUARE), (1, SQUARE + 100)], 1.0, 0, 0, 640)[1][0], 1)

    def test_yolo_line_is_normalised(self):
        line = dt.to_yolo_seg_line(1, np.array([[0, 0], [320, 0], [320, 640]], float), 640)
        v = line.split()
        self.assertEqual(v[0], "1")
        self.assertEqual([float(x) for x in v[1:]], [0.0, 0.0, 0.5, 0.0, 0.5, 1.0])


if __name__ == "__main__":
    unittest.main()
