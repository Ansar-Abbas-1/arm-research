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

    def test_end_to_end_segmentation_head_reads_only_the_box_score_and_class(self):
        out = np.zeros((1, 300, 38), np.float32)
        out[0, 0, :6] = [10, 20, 110, 220, 0.8, 0]
        out[0, 0, 6:] = 9.0                                                         # mask coefficients: not read as anything
        boxes, scores, cls = ol.decode(out, "auto", conf=0.25, num_classes=1)
        np.testing.assert_allclose(boxes, [[10, 20, 110, 220]])
        self.assertEqual((len(scores), cls[0]), (1, 0))

    def test_format_detection(self):
        self.assertEqual(ol.detect_format((1, 84, 8400)), "v8")
        self.assertEqual(ol.detect_format((1, 300, 6)), "e2e")
        self.assertEqual(ol.detect_format((1, 25200, 85), 80), "v5")
        self.assertEqual(ol.detect_format((1, 8400, 84), 80), "v8_t")
        self.assertEqual(ol.detect_format((1, 38, 8400), 2), "v8")                 # a segmentation head (4 + 2 + 32 channels)
        self.assertEqual(ol.detect_format((1, 7, 50), 3), "v8")
        self.assertEqual(ol.detect_format((1, 300, 38), 1), "e2e")                 # a YOLO26 -seg end-to-end head: box, score, class, 32 coefficients


class NmsTest(unittest.TestCase):
    def test_overlapping_boxes_of_one_class_keep_the_best(self):
        boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105], [300, 300, 400, 400]], np.float32)
        keep = ol.nms(boxes, np.array([0.6, 0.9, 0.7], np.float32), np.array([0, 0, 0]))
        self.assertEqual(list(keep), [1, 2])

    def test_different_classes_do_not_suppress_each_other(self):
        boxes = np.array([[0, 0, 100, 100], [5, 5, 105, 105]], np.float32)
        keep = ol.nms(boxes, np.array([0.9, 0.8], np.float32), np.array([0, 1]))
        self.assertEqual(sorted(keep), [0, 1])

    def test_ios_removes_a_small_box_inside_a_big_one_that_iou_keeps(self):
        boxes = np.array([[0, 0, 100, 100], [20, 20, 50, 50]], np.float32)       # the second lies inside the first: IoU 0.09, IOS 1.0
        scores = np.array([0.9, 0.8], np.float32)
        self.assertEqual(sorted(ol.nms(boxes, scores, np.array([0, 0]), 0.5, "iou")), [0, 1])
        self.assertEqual(list(ol.nms(boxes, scores, np.array([0, 0]), 0.5, "ios")), [0])

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


class TilingTest(unittest.TestCase):
    def test_a_small_image_is_one_tile(self):
        self.assertEqual(ol.tile_grid(400, 300, 512, 0.2), [(0, 0, 400, 300)])

    def test_tiles_cover_the_image_with_the_overlap_and_the_last_one_aligned_to_the_border(self):
        tiles = ol.tile_grid(1920, 1080, 512, 0.2)
        xs = sorted({t[0] for t in tiles})
        self.assertEqual(xs, [0, 409, 818, 1227, 1408])                  # step int(512 * 0.8) = 409, the last one at 1920 - 512
        self.assertEqual(sorted({t[1] for t in tiles}), [0, 409, 568])
        self.assertTrue(all(t[2] - t[0] == 512 and t[3] - t[1] == 512 for t in tiles))
        covered = np.zeros((1080, 1920), bool)
        for x0, y0, x1, y1 in tiles:
            covered[y0:y1, x0:x1] = True
        self.assertTrue(covered.all())

    def test_a_12_megapixel_photo(self):
        tiles = ol.tile_grid(3968, 2976, 512, 0.2)
        self.assertEqual(len(tiles), 10 * 8)                              # 10 columns x 8 rows
        self.assertEqual(max(t[2] for t in tiles), 3968)
        self.assertEqual(max(t[3] for t in tiles), 2976)


class MatchTest(unittest.TestCase):
    def test_iou(self):
        self.assertAlmostEqual(ol.box_iou([0, 0, 10, 10], [0, 0, 10, 10]), 1.0)
        self.assertAlmostEqual(ol.box_iou([0, 0, 10, 10], [5, 0, 15, 10]), 1.0 / 3.0)
        self.assertEqual(ol.box_iou([0, 0, 10, 10], [20, 20, 30, 30]), 0.0)

    def test_matching_counts_tp_fp_fn_one_to_one(self):
        gts = [[0, 0, 100, 100], [200, 200, 300, 300], [500, 500, 600, 600]]
        dets = [([2, 2, 102, 102], 0.9), ([198, 198, 302, 302], 0.8), ([205, 205, 305, 305], 0.7), ([900, 900, 950, 950], 0.6)]
        tp, fp, fn = ol.match_detections(dets, gts, 0.5)
        self.assertEqual((tp, fp, fn), (2, 2, 1))                         # the second box over the same ground truth and the stray one are false positives


class FloorMemoryTest(unittest.TestCase):
    def test_the_ground_marker_height_is_used_while_it_is_fresh_then_the_parameter(self):
        f = ol.FloorMemory(max_age=600.0)
        self.assertEqual(f.get(100.0, -0.60), (-0.60, "param", None))               # never seen: the parameter
        f.update(-0.598, 100.0)
        z, source, age = f.get(160.0, -0.60)
        self.assertEqual((z, source), (-0.598, "marker_0"))
        self.assertAlmostEqual(age, 60.0)
        self.assertEqual(f.get(100.0 + 601.0, -0.60)[:2], (-0.60, "param"))         # too old
        self.assertEqual(f.get(99.0, -0.60)[:2], (-0.60, "param"))                   # a clock that went backwards is not trusted

    def test_a_newer_observation_replaces_and_an_older_one_does_not(self):
        f = ol.FloorMemory()
        f.update(-0.60, 10.0)
        f.update(-0.59, 20.0)
        f.update(-0.70, 15.0)
        self.assertEqual(f.get(21.0, 0.0)[0], -0.59)


if __name__ == "__main__":
    unittest.main()
