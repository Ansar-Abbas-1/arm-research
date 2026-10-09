#!/usr/bin/env python3
"""Unit tests of the detector's pure logic: letterbox, the three YOLO output layouts, NMS and the floor-plane geometry."""
import unittest

import numpy as np

from arm_perception import object_logic as ol

K = np.array([[1500.0, 0, 960.0], [0, 1500.0, 540.0], [0, 0, 1.0]])


def camera_looking_down(height, x=0.0, y=0.0):
    """Optical frame pose in the base frame: above (x, y, height), optical z pointing DOWN (-z base), optical x along base x."""
    T = np.eye(4)
    T[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])        # x right = base x, y down = base -y, z forward = base -z
    T[:3, 3] = [x, y, height]
    return T


def project(T_base_camera, point, K):
    p = np.linalg.inv(T_base_camera).dot(np.append(point, 1.0))[:3]
    return K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]


class LetterboxTest(unittest.TestCase):
    def test_shape_scale_and_padding(self):
        img = np.zeros((1080, 1920, 3), np.uint8)
        blob, scale, pad = ol.letterbox(img, 640)
        self.assertEqual(blob.shape, (1, 3, 640, 640))
        self.assertEqual(blob.dtype, np.float32)
        self.assertAlmostEqual(scale, 640 / 1920.0)
        self.assertEqual(pad[0], 0)
        self.assertEqual(pad[1], (640 - 360) // 2)

    def test_the_channels_are_rgb_and_scaled(self):
        img = np.zeros((100, 100, 3), np.uint8)
        img[:, :, 0] = 255                                            # blue in BGR
        blob, _, _ = ol.letterbox(img, 100)
        self.assertAlmostEqual(float(blob[0, 2].mean()), 1.0)         # blue is the 3rd channel of RGB
        self.assertAlmostEqual(float(blob[0, 0].mean()), 0.0)

    def test_boxes_map_back_to_the_original_image(self):
        shape = (1080, 1920)
        _, scale, pad = ol.letterbox(np.zeros(shape + (3,), np.uint8), 640)
        orig = np.array([[300.0, 200.0, 900.0, 700.0]])
        boxed = orig.copy()
        boxed[:, [0, 2]] = boxed[:, [0, 2]] * scale + pad[0]
        boxed[:, [1, 3]] = boxed[:, [1, 3]] * scale + pad[1]
        np.testing.assert_allclose(ol.unletterbox(boxed, scale, pad, shape), orig, atol=1e-3)


class DecodeTest(unittest.TestCase):
    def test_v8_channels_first(self):
        out = np.zeros((1, 4 + 3, 50), np.float32)                    # 3 classes, 50 candidates
        out[0, :4, 0] = [100, 120, 40, 60]; out[0, 4 + 2, 0] = 0.9    # class 2
        out[0, :4, 1] = [300, 300, 20, 20]; out[0, 4 + 0, 1] = 0.1    # below the threshold
        boxes, scores, cls = ol.decode(out, "auto", conf=0.25, num_classes=3)
        self.assertEqual(len(boxes), 1)
        np.testing.assert_allclose(boxes[0], [80, 90, 120, 150])
        self.assertEqual((cls[0], round(float(scores[0]), 2)), (2, 0.9))

    def test_v8_segmentation_drops_the_mask_coefficients(self):
        out = np.zeros((1, 4 + 2 + 32, 50), np.float32)
        out[0, :4, 1] = [50, 50, 10, 10]; out[0, 4 + 1, 1] = 0.8
        out[0, 6:, 1] = 5.0                                           # mask coefficients must not be read as scores
        boxes, scores, cls = ol.decode(out, "v8", conf=0.25, num_classes=2)
        self.assertEqual((len(boxes), cls[0]), (1, 1))

    def test_v5_multiplies_objectness_and_class_score(self):
        out = np.zeros((1, 50, 5 + 2), np.float32)
        out[0, 0, :4] = [100, 100, 20, 20]; out[0, 0, 4] = 0.5; out[0, 0, 5 + 1] = 0.8
        boxes, scores, cls = ol.decode(out, "auto", conf=0.25, num_classes=2)
        self.assertEqual((len(boxes), cls[0]), (1, 1))
        self.assertAlmostEqual(float(scores[0]), 0.4, places=5)

    def test_end_to_end_layout_is_already_xyxy(self):
        out = np.zeros((1, 300, 6), np.float32)
        out[0, 0] = [10, 20, 110, 220, 0.7, 3]
        out[0, 1] = [0, 0, 5, 5, 0.1, 0]
        boxes, scores, cls = ol.decode(out, "auto", conf=0.25)
        np.testing.assert_allclose(boxes, [[10, 20, 110, 220]])
        self.assertEqual(cls[0], 3)

    def test_format_detection(self):
        self.assertEqual(ol.detect_format((1, 84, 8400)), "v8")
        self.assertEqual(ol.detect_format((1, 300, 6)), "e2e")
        self.assertEqual(ol.detect_format((1, 25200, 85), 80), "v5")
        self.assertEqual(ol.detect_format((1, 8400, 84), 80), "v8_t")
        self.assertEqual(ol.detect_format((1, 38, 8400), 2), "v8")                 # a segmentation head (4 + 2 + 32 channels)
        self.assertEqual(ol.detect_format((1, 7, 50), 3), "v8")


class NmsTest(unittest.TestCase):
    def test_overlapping_boxes_of_one_class_keep_the_best(self):
        boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105], [300, 300, 400, 400]], np.float32)
        keep = ol.nms(boxes, np.array([0.6, 0.9, 0.7], np.float32), np.array([0, 0, 0]))
        self.assertEqual(list(keep), [1, 2])

    def test_different_classes_do_not_suppress_each_other(self):
        boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105]], np.float32)
        keep = ol.nms(boxes, np.array([0.9, 0.8], np.float32), np.array([0, 1]))
        self.assertEqual(sorted(keep), [0, 1])

    def test_empty(self):
        self.assertEqual(len(ol.nms(np.zeros((0, 4), np.float32), np.zeros(0, np.float32), np.zeros(0, int))), 0)


class GeometryTest(unittest.TestCase):
    def test_the_principal_point_ray_is_the_optical_axis(self):
        np.testing.assert_allclose(ol.pixel_ray(960, 540, K), [0, 0, 1], atol=1e-9)

    def test_a_ray_looking_down_meets_the_floor_straight_below(self):
        T = camera_looking_down(0.40)
        p = ol.object_position([950, 530, 970, 550], T, K, None, 0.0)
        np.testing.assert_allclose(p, [0, 0, 0], atol=1e-9)

    def test_a_floor_point_is_recovered_from_its_pixel(self):
        T = camera_looking_down(0.55, x=0.30, y=-0.05)
        for target in ([0.42, -0.02, 0.0], [0.25, -0.12, 0.0], [0.30, 0.04, 0.0]):
            u, v = project(T, np.array(target), K)
            p = ol.object_position([u - 10, v - 10, u + 10, v + 10], T, K, None, 0.0)
            np.testing.assert_allclose(p, target, atol=1e-9)

    def test_a_raised_grasp_plane_is_used(self):
        T = camera_looking_down(0.55)
        target = np.array([0.05, 0.03, 0.04])                         # the grasp point 4 cm above the floor
        u, v = project(T, target, K)
        np.testing.assert_allclose(ol.object_position([u - 5, v - 5, u + 5, v + 5], T, K, None, 0.04), target, atol=1e-9)
        wrong = ol.object_position([u - 5, v - 5, u + 5, v + 5], T, K, None, 0.0)       # intersecting the floor instead is off
        self.assertGreater(np.linalg.norm(wrong - target), 0.003)

    def test_a_ray_that_does_not_reach_the_plane_gives_none(self):
        T = camera_looking_down(0.2)
        self.assertIsNone(ol.intersect_plane(T[:3, 3], [0, 0, 1.0], 0.0))      # looking up
        self.assertIsNone(ol.intersect_plane(T[:3, 3], [1.0, 0, -1e-9], 0.0))  # parallel to the plane
        self.assertIsNone(ol.intersect_plane([0, 0, 0.2], [0, 0, -1.0], -5.0, max_range=3.0))  # beyond the range

    def test_distortion_is_undistorted(self):
        D = np.array([-0.376, 0.173, -0.001, 0.0018, 0.0])
        d0 = ol.pixel_ray(100, 100, K, None)
        d1 = ol.pixel_ray(100, 100, K, D)
        self.assertGreater(np.linalg.norm(d0 - d1), 1e-3)                # the edge pixel moves when the distortion is applied


if __name__ == "__main__":
    unittest.main()
