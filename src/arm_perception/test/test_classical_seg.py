#!/usr/bin/env python3
"""Unit tests of the classical segmentation baselines on synthetic images."""
import unittest

import numpy as np

from arm_perception import classical_seg as cs


def scene():
    """Dark soil with two touching round 'leaves' (green), one separate leaf and a thin grass blade; plus a bluish-grey tile corner."""
    img = np.full((200, 300, 3), (30, 40, 45), np.uint8)             # soil (BGR)
    import cv2
    cv2.circle(img, (60, 100), 22, (40, 190, 80), -1)                # leaf 1
    cv2.circle(img, (95, 100), 22, (40, 190, 80), -1)                # leaf 2 touching leaf 1
    cv2.circle(img, (200, 60), 22, (40, 190, 80), -1)                # leaf 3
    cv2.line(img, (150, 20), (150, 180), (60, 170, 60), 3)           # a thin blade
    img[150:200, 230:300] = (170, 160, 150)                          # pavement
    return img


class ClassicalSegTest(unittest.TestCase):
    def test_exg_is_high_on_green_and_low_on_soil_and_pavement(self):
        e = cs.exg(scene())
        self.assertGreater(e[100, 60], 0.3)
        self.assertLess(e[10, 10], 0.1)
        self.assertLess(e[170, 260], 0.1)

    def test_near_black_pixels_do_not_produce_extreme_values(self):
        img = np.zeros((10, 10, 3), np.uint8)
        img[0, 0] = (0, 3, 0)                                           # almost black, but "all green" by ratio
        self.assertEqual(float(cs.exg(img)[0, 0]), 0.0)

    def test_otsu_separates_a_bimodal_array(self):
        v = np.concatenate([np.full(500, 0.1), np.full(500, 0.8)])
        t = cs.otsu(v)
        self.assertTrue(0.1 < t < 0.8)

    def test_exg_and_hsv_masks_find_the_leaves_and_the_blade(self):
        img = scene()
        for m in (cs.green_mask_exg(img), cs.green_mask_hsv(img)):
            self.assertTrue(m[100, 60] and m[100, 95] and m[60, 200])
            self.assertFalse(m[10, 10] or m[170, 260])
            self.assertTrue(m[100, 150])                              # the thin blade is green too

    def test_remove_thin_drops_the_blade_and_keeps_the_leaves(self):
        m = cs.remove_thin(cs.green_mask_hsv(scene()), 5)
        self.assertFalse(m[100, 150])
        self.assertTrue(m[100, 60] and m[60, 200])

    def test_watershed_splits_the_touching_leaves(self):
        m = cs.remove_thin(cs.green_mask_hsv(scene()), 5)
        inst = cs.split_instances(m, min_dist=0.75, min_area=200)
        self.assertEqual(len(inst), 3)
        xs = sorted(i["centroid"][0] for i in inst)
        self.assertAlmostEqual(xs[0], 60, delta=6)
        self.assertAlmostEqual(xs[1], 95, delta=6)

    def test_remove_small(self):
        m = np.zeros((50, 50), bool)
        m[5:8, 5:8] = True
        m[20:40, 20:40] = True
        out = cs.remove_small(m, 50)
        self.assertFalse(out[6, 6])
        self.assertTrue(out[30, 30])

    def test_pixel_metrics(self):
        gt = np.zeros((10, 10), bool); gt[:, :5] = True
        pred = np.zeros((10, 10), bool); pred[:, 2:7] = True
        r = cs.pixel_metrics(pred, gt)
        self.assertAlmostEqual(r["precision"], 0.6)
        self.assertAlmostEqual(r["recall"], 0.6)
        self.assertAlmostEqual(r["iou"], 30.0 / 70.0)

    def test_instance_hits(self):
        a = np.zeros((100, 100), bool); a[10:30, 10:30] = True
        b = np.zeros((100, 100), bool); b[60:80, 60:80] = True
        r = cs.instance_hits([(20, 20), (21, 22), (90, 90)], [a, b])
        self.assertAlmostEqual(r["recall"], 0.5)                        # only the first instance is covered
        self.assertAlmostEqual(r["precision"], 2.0 / 3.0)


    def test_instance_hits_labels(self):
        lab = np.zeros((100, 100), np.int32)
        lab[10:30, 10:30] = 1
        lab[60:80, 60:80] = 2
        r = cs.instance_hits_labels([(20, 20), (21, 22), (90, 90)], lab)
        self.assertAlmostEqual(r["recall"], 0.5)
        self.assertAlmostEqual(r["precision"], 2.0 / 3.0)


if __name__ == "__main__":
    unittest.main()
