"""Hardware-free tests of the ArUco library: markers rendered at known poses, detected, and
posed; the distance and its dependence on the marker size; dictionary auto-detection."""
import unittest

import cv2
import numpy as np

from arm_camera import aruco
from arm_camera.geometry import invert_transform, make_transform, rot_x, rot_y, rot_z, rotation_error_deg

K = np.array([[1521.4, 0.0, 951.9], [0.0, 1521.9, 532.1], [0.0, 0.0, 1.0]])
SIZE = 0.100   # metres, the black square


def pose_facing_camera(yaw=0.0, tilt=0.4, t=(0.02, -0.01, 0.50)):
    """T_cam_marker with the marker's z (out of the marker) pointing back at the camera."""
    return make_transform(rot_x(np.pi) @ rot_z(yaw) @ rot_y(tilt), t)


def render_scene(placements, size=(1920, 1080), px=400):
    """Image with markers [(dictionary, id, T_cam_marker)] under a pinhole camera K (no distortion)."""
    canvas = np.full((size[1], size[0]), 190, np.uint8)
    for name, mid, T in placements:
        m = aruco.render_marker(name, mid, px)
        pad = px // 4
        src = np.full((px + 2 * pad, px + 2 * pad), 255, np.uint8)
        src[pad:pad + px, pad:pad + px] = m
        s = SIZE / px
        c = pad + px / 2.0
        S = np.array([[s, 0, -c * s], [0, -s, c * s], [0, 0, 1]])        # source px -> marker plane (x right, y up)
        H = K @ np.column_stack([T[:3, 0], T[:3, 1], T[:3, 3]]) @ S
        warped = cv2.warpPerspective(src, H, size, flags=cv2.INTER_LINEAR, borderValue=190)
        mask = cv2.warpPerspective(np.full_like(src, 255), H, size, flags=cv2.INTER_NEAREST, borderValue=0) > 0
        canvas[mask] = warped[mask]
    return cv2.GaussianBlur(canvas, (3, 3), 0.7)


class ArucoPoseTest(unittest.TestCase):
    def test_pose_and_distance_from_a_rendered_marker(self):
        for yaw, tilt, z in ((0.0, 0.4, 0.50), (1.0, 0.5, 0.35), (-2.5, 0.3, 0.70)):
            T_true = pose_facing_camera(yaw, tilt, (0.03, -0.02, z))
            img = render_scene([("4X4_50", 7, T_true)])
            dets = aruco.detect(img, "4X4_50")
            self.assertEqual([d.marker_id for d in dets], [7], "marker not read at z=%.2f" % z)
            T, rms, ratio = aruco.estimate_pose(dets[0].corners, SIZE, K, np.zeros(5))
            self.assertLess(rms, 0.6)
            self.assertLess(abs(aruco.distance(T) - aruco.distance(T_true)) * 1000.0, 2.0, "distance error [mm] at z=%.2f" % z)
            self.assertLess(np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000.0, 2.5)
            self.assertLess(rotation_error_deg(T[:3, :3], T_true[:3, :3]), 2.0)

    def test_distance_is_proportional_to_the_marker_size_that_is_entered(self):
        # The most common source of error in a distance check: the printed size is not the
        # size typed in. A 5 % error in marker_size is a 5 % error in the distance.
        T_true = pose_facing_camera(0.3, 0.4, (0.0, 0.0, 0.50))
        det = aruco.detect(render_scene([("4X4_50", 3, T_true)]), "4X4_50")[0]
        d_ok = aruco.distance(aruco.estimate_pose(det.corners, SIZE, K, np.zeros(5))[0])
        d_big = aruco.distance(aruco.estimate_pose(det.corners, SIZE * 1.05, K, np.zeros(5))[0])
        self.assertAlmostEqual(d_big / d_ok, 1.05, places=3)

    def test_ambiguity_is_high_when_frontal_and_low_when_tilted(self):
        frontal = aruco.detect(render_scene([("4X4_50", 1, pose_facing_camera(0.0, 0.0, (0.0, 0.0, 0.8)))]), "4X4_50")[0]
        tilted = aruco.detect(render_scene([("4X4_50", 1, pose_facing_camera(0.0, 0.8, (0.0, 0.0, 0.4)))]), "4X4_50")[0]
        _, _, r_frontal = aruco.estimate_pose(frontal.corners, SIZE, K, np.zeros(5))
        _, _, r_tilted = aruco.estimate_pose(tilted.corners, SIZE, K, np.zeros(5))
        self.assertGreater(r_frontal, r_tilted)
        self.assertLess(r_tilted, 0.5)

    def test_two_markers_in_one_image_get_their_own_poses(self):
        Ta = pose_facing_camera(0.2, 0.4, (-0.12, 0.0, 0.5))
        Tb = pose_facing_camera(-0.4, 0.3, (0.14, 0.02, 0.6))
        dets = {d.marker_id: d for d in aruco.detect(render_scene([("4X4_50", 4, Ta), ("4X4_50", 9, Tb)]), "4X4_50")}
        self.assertEqual(sorted(dets), [4, 9])
        for mid, T_true in ((4, Ta), (9, Tb)):
            T, _, _ = aruco.estimate_pose(dets[mid].corners, SIZE, K, np.zeros(5))
            self.assertLess(np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000.0, 2.5, "marker %d" % mid)

    def test_a_marker_turned_by_90_degrees_keeps_its_id_and_turns_its_frame(self):
        T0 = pose_facing_camera(0.0, 0.3, (0.0, 0.0, 0.5))
        T90 = pose_facing_camera(np.pi / 2, 0.3, (0.0, 0.0, 0.5))
        d0 = aruco.detect(render_scene([("4X4_50", 2, T0)]), "4X4_50")[0]
        d90 = aruco.detect(render_scene([("4X4_50", 2, T90)]), "4X4_50")[0]
        self.assertEqual(d0.marker_id, d90.marker_id)
        A, _, _ = aruco.estimate_pose(d0.corners, SIZE, K, np.zeros(5))
        B, _, _ = aruco.estimate_pose(d90.corners, SIZE, K, np.zeros(5))
        self.assertAlmostEqual(rotation_error_deg(A[:3, :3], B[:3, :3]), 90.0, delta=2.0)
        self.assertLess(np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000.0, 3.0)   # same centre


class ArucoDictionaryTest(unittest.TestCase):
    def test_auto_detection_names_the_dictionary(self):
        T = pose_facing_camera(0.1, 0.4, (0.0, 0.0, 0.5))
        for name, mid in (("4X4_50", 3), ("5X5_100", 77), ("6X6_250", 120), ("ARUCO_ORIGINAL", 11)):
            found, dets = aruco.detect_auto(render_scene([(name, mid, T)]))
            self.assertEqual(found, name)
            self.assertEqual([d.marker_id for d in dets], [mid])

    def test_a_low_id_is_named_by_the_smallest_dictionary_that_reads_it(self):
        T = pose_facing_camera(0.1, 0.4, (0.0, 0.0, 0.5))
        found, dets = aruco.detect_auto(render_scene([("5X5_100", 7, T)]))
        self.assertEqual(found, "5X5_50")          # id 7 is the same marker in both
        self.assertEqual([d.marker_id for d in dets], [7])

    def test_nothing_found_in_an_empty_image(self):
        self.assertEqual(aruco.detect_auto(np.full((480, 640), 128, np.uint8)), (None, []))

    def test_svg_has_one_rect_per_black_cell_and_the_exact_size(self):
        svg = aruco.marker_svg("4X4_50", 5, 100.0, label=False)
        cells = aruco.marker_cells("4X4_50", 5)
        self.assertEqual(svg.count('fill="black"'), int(cells.sum()))
        self.assertIn('width="150.00mm"', svg)          # 100 mm square + 2 x 25 mm quiet margin
        self.assertEqual(cells.shape, (6, 6))           # 4x4 bits + the black border


if __name__ == "__main__":
    unittest.main()
