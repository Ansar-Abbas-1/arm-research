"""Hardware-free tests of the calibration library: geometry, chessboard detection on a
rendered image, and hand-eye recovery from synthetic arm poses with PnP-like noise."""
import unittest

import cv2
import numpy as np

from arm_camera import board as board_mod
from arm_camera import hand_eye
from arm_camera import overlay as overlay_mod
from arm_camera.geometry import (average_rotation, invert_transform, make_transform, matrix_from_quat,
                                 quat_from_matrix, rot_x, rot_y, rot_z, rotation_error_deg)

K = np.array([[1000.0, 0.0, 960.0], [0.0, 1000.0, 540.0], [0.0, 0.0, 1.0]])
BOARD = board_mod.Board(cols=9, rows=6, square_size=0.03)


class GeometryTest(unittest.TestCase):
    def test_quaternion_round_trip(self):
        rng = np.random.RandomState(1)
        for _ in range(50):
            R = rot_z(rng.uniform(-3, 3)) @ rot_y(rng.uniform(-1.5, 1.5)) @ rot_x(rng.uniform(-3, 3))
            self.assertLess(rotation_error_deg(R, matrix_from_quat(quat_from_matrix(R))), 1e-6)

    def test_inverse_composes_to_identity(self):
        T = make_transform(rot_z(0.4) @ rot_y(-0.7), [0.1, -0.2, 0.3])
        np.testing.assert_allclose(T @ invert_transform(T), np.eye(4), atol=1e-12)

    def test_average_rotation_of_identical_rotations(self):
        R = rot_x(0.5)
        np.testing.assert_allclose(average_rotation([R, R, R]), R, atol=1e-12)


def render_board(T_cam_board, size=(1920, 1080), px_sq=80, margin=40, black_parity=1):
    """Image of the chessboard seen by a pinhole camera with intrinsics K (no distortion)."""
    cols, rows, sq = BOARD.cols, BOARD.rows, BOARD.square_size
    w, h = (cols + 1) * px_sq + 2 * margin, (rows + 1) * px_sq + 2 * margin
    src = np.full((h, w), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == black_parity:
                x0, y0 = margin + c * px_sq, margin + r * px_sq
                src[y0:y0 + px_sq, x0:x0 + px_sq] = 0
    # board px (u, v) -> metric board plane (x, y): first inner corner is at (margin + px_sq).
    S = np.array([[sq / px_sq, 0, -(margin + px_sq) * sq / px_sq], [0, sq / px_sq, -(margin + px_sq) * sq / px_sq], [0, 0, 1]])
    P = K @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]])
    img = cv2.warpPerspective(src, P @ S, size, flags=cv2.INTER_LINEAR, borderValue=200)
    return cv2.GaussianBlur(img, (3, 3), 0.8)


class BoardPipelineTest(unittest.TestCase):
    def test_board_is_unambiguous_for_even_odd_corner_counts(self):
        self.assertTrue(board_mod.Board(9, 6, 0.03).is_unambiguous())
        self.assertFalse(board_mod.Board(7, 7, 0.03).is_unambiguous())

    def test_detect_and_recover_pose_from_a_rendered_image(self):
        # black_parity=1: white top-left square, the detector's order matches board.object_points().
        for yaw, tilt, dist in ((0.0, 0.3, 0.55), (0.4, 0.5, 0.65), (-0.3, 0.2, 0.5)):
            R = rot_z(yaw) @ rot_x(tilt)   # printed board: x right, y down, normal into the paper
            T_true = make_transform(R, [-0.10, -0.06, dist])
            img = render_board(T_true, black_parity=1)
            corners = board_mod.detect_corners(img, BOARD)
            self.assertIsNotNone(corners, "board not found at yaw=%.1f tilt=%.1f" % (yaw, tilt))
            T_est, rms = board_mod.estimate_pose(corners, BOARD, K, np.zeros(5))
            self.assertLess(rms, 0.6)
            self.assertLess(np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]) * 1000.0, 3.0)   # mm
            self.assertLess(rotation_error_deg(T_true[:3, :3], T_est[:3, :3]), 0.7)       # deg

    def test_black_top_left_board_is_labelled_from_the_far_end_but_consistently(self):
        # The common printed board (black top-left square): the detector starts at the other
        # end, so T_cam_board is the true pose composed with a fixed 180 degree in-plane turn.
        # Calibration only needs the labelling to be consistent, which it is.
        offsets = []
        for yaw, tilt in ((0.0, 0.3), (0.5, 0.4), (-0.4, 0.2)):
            T_true = make_transform(rot_z(yaw) @ rot_x(tilt), [-0.10, -0.06, 0.55])
            corners = board_mod.detect_corners(render_board(T_true, black_parity=0), BOARD)
            self.assertIsNotNone(corners)
            T_est, rms = board_mod.estimate_pose(corners, BOARD, K, np.zeros(5))
            self.assertLess(rms, 0.6)
            offsets.append(invert_transform(T_true) @ T_est)   # same fixed transform every frame
        for o in offsets[1:]:
            self.assertLess(np.linalg.norm(o[:3, 3] - offsets[0][:3, 3]) * 1000.0, 3.0)
            self.assertLess(rotation_error_deg(o[:3, :3], offsets[0][:3, :3]), 0.7)

    def test_first_square_colour_is_read_from_the_image(self):
        for parity in (0, 1):
            T_true = make_transform(rot_z(0.2) @ rot_x(0.3), [-0.10, -0.06, 0.55])
            img = render_board(T_true, black_parity=parity)
            corners = board_mod.detect_corners(img, BOARD)
            T_est, _ = board_mod.estimate_pose(corners, BOARD, K, np.zeros(5))
            # Independent check: sample the image at the projected centre of cell (0, 0) of the LABELLING frame.
            centre = board_mod.project(K, np.zeros(5), T_est, [[BOARD.square_size / 2, BOARD.square_size / 2, 0.0]])[0]
            dark_there = float(img[int(round(centre[1])) - 2:int(round(centre[1])) + 3,
                                   int(round(centre[0])) - 2:int(round(centre[0])) + 3].mean()) < 100.0
            self.assertEqual(board_mod.first_square_is_dark(img, corners, BOARD), dark_there)

    def test_pose_with_lens_distortion(self):
        D = np.array([-0.30, 0.10, 0.001, -0.001, 0.0])
        T_true = make_transform(rot_z(0.2) @ rot_x(0.35), [-0.10, -0.05, 0.5])
        px = board_mod.project(K, D, T_true, BOARD.object_points())
        rng = np.random.RandomState(3)
        corners = (px + rng.normal(0, 0.3, px.shape)).astype(np.float32).reshape(-1, 1, 2)
        T_est, rms = board_mod.estimate_pose(corners, BOARD, K, D)
        self.assertLess(rms, 0.6)
        self.assertLess(np.linalg.norm(T_est[:3, 3] - T_true[:3, 3]) * 1000.0, 2.0)

    def test_camera_matrix_from_zero_info_is_none(self):
        self.assertIsNone(board_mod.camera_matrix_from_info([0.0] * 9))
        self.assertIsNotNone(board_mod.camera_matrix_from_info(K.reshape(-1).tolist()))


X_TRUE = make_transform(rot_z(0.25) @ rot_y(0.12) @ rot_x(-0.08), [0.04, -0.025, 0.07])
B_TRUE = make_transform(rot_x(np.pi), [0.35, 0.02, 0.0])   # board lying flat on the table, facing up


def arm_pose(yaw, pitch, r, h):
    """end_effector pose of an arm that only has a yaw joint and a pitch (like this one:
    joint_1 about the vertical, joints 2 to 5 about a horizontal axis)."""
    R = rot_z(yaw) @ rot_y(pitch)
    return make_transform(R, rot_z(yaw) @ np.array([r, 0.0, h]))


def synthetic_samples(n, seed, rot_noise_deg=0.25, t_noise_mm=1.0, yaw_range=0.5, pitch_range=(1.9, 2.7)):
    """Arm poses from which the board is visible, with PnP-like noise on T_cam_board."""
    rng = np.random.RandomState(seed)
    samples = []
    while len(samples) < n:
        T = arm_pose(rng.uniform(-yaw_range, yaw_range), rng.uniform(*pitch_range),
                     rng.uniform(0.15, 0.40), rng.uniform(0.25, 0.50))
        T_cam = T @ X_TRUE
        T_cb = invert_transform(T_cam) @ B_TRUE
        centre = T_cb[:3, 3]
        if centre[2] < 0.15 or centre[2] > 0.8 or np.degrees(np.arctan2(np.hypot(centre[0], centre[1]), centre[2])) > 30:
            continue
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        ang = np.radians(rng.normal(0, rot_noise_deg))
        K_ = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        dR = np.eye(3) + np.sin(ang) * K_ + (1 - np.cos(ang)) * K_ @ K_
        T_cb_noisy = make_transform(dR @ T_cb[:3, :3], T_cb[:3, 3] + rng.normal(0, t_noise_mm / 1000.0, 3))
        samples.append(hand_eye.Sample(T_base_ee=T, T_cam_board=T_cb_noisy))
    return samples


class HandEyeTest(unittest.TestCase):
    def test_recovers_the_camera_pose_with_the_arms_two_rotation_axes(self):
        samples = synthetic_samples(15, seed=4)
        rot, axis = hand_eye.motion_diversity(samples)
        self.assertGreater(axis, 20.0, "fixture must have non-parallel rotation axes")
        results = hand_eye.solve_all(samples)
        self.assertGreaterEqual(len(results), 3)
        best = results[0]
        self.assertLess(np.linalg.norm(best.X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, 6.0, "translation error [mm] (%s)" % best.method)
        self.assertLess(rotation_error_deg(best.X[:3, :3], X_TRUE[:3, :3]), 1.0, "rotation error [deg] (%s)" % best.method)
        # The fixed board must come out fixed, and agree with the truth.
        self.assertLess(np.linalg.norm(best.B[:3, 3] - B_TRUE[:3, 3]) * 1000.0, 6.0)
        self.assertLess(best.position_scatter_mm, 10.0)

    def test_consistency_metric_detects_a_wrong_x(self):
        samples = synthetic_samples(12, seed=5, rot_noise_deg=0.0, t_noise_mm=0.0)
        _, pos_true, rot_true = hand_eye.consistency(samples, X_TRUE)
        wrong = X_TRUE @ make_transform(rot_z(0.05), [0.01, 0.0, 0.0])
        _, pos_wrong, rot_wrong = hand_eye.consistency(samples, wrong)
        self.assertLess(pos_true, 1e-6)
        self.assertGreater(pos_wrong, 5.0 * max(pos_true, 0.1))
        self.assertGreater(rot_wrong, rot_true)

    def test_reprojection_metric_is_small_for_the_true_x(self):
        samples = synthetic_samples(10, seed=6, rot_noise_deg=0.0, t_noise_mm=0.0)
        for s in samples:
            px = board_mod.project(K, np.zeros(5), s.T_cam_board, BOARD.object_points())
            s.corners = px.astype(np.float32).reshape(-1, 1, 2)
        per = hand_eye.reprojection_per_sample(samples, X_TRUE, B_TRUE, K, np.zeros(5), BOARD)
        self.assertLess(max(per), 1e-3)

    def test_diversity_flags_parallel_axes_and_no_rotation(self):
        # Yaw-only motion: every relative rotation has the same (vertical) axis.
        yaw_only = [hand_eye.Sample(arm_pose(y, 2.2, 0.3, 0.4), np.eye(4)) for y in (-0.4, -0.1, 0.2, 0.5)]
        rot, axis = hand_eye.motion_diversity(yaw_only)
        self.assertGreater(rot, 10.0)
        self.assertLess(axis, 1.0)
        # Pure translation: no rotation at all.
        translating = [hand_eye.Sample(make_transform(np.eye(3), [0.1 * i, 0.0, 0.3]), np.eye(4)) for i in range(4)]
        rot, axis = hand_eye.motion_diversity(translating)
        self.assertLess(rot, 1e-6)
        self.assertEqual(axis, 0.0)

    def test_too_few_samples_is_refused(self):
        with self.assertRaises(ValueError):
            hand_eye.solve(synthetic_samples(2, seed=7))


class OverlayTest(unittest.TestCase):
    def test_board_triangles_cover_the_printed_board_and_follow_the_colour_flag(self):
        pts, cols = overlay_mod.board_triangles(BOARD, first_dark=True)
        self.assertEqual(len(pts), 6 * (BOARD.cols + 1) * (BOARD.rows + 1))
        self.assertAlmostEqual(pts[:, 0].min(), -BOARD.square_size)
        self.assertAlmostEqual(pts[:, 0].max(), BOARD.cols * BOARD.square_size)
        self.assertAlmostEqual(pts[:, 1].max(), BOARD.rows * BOARD.square_size)
        # cell (0, 0) is the 2nd cell in each direction of the list (i = -1 comes first).
        first_cell = (1 * (BOARD.rows + 1) + 1) * 6
        self.assertLess(cols[first_cell][0], 0.5)                      # dark when first_dark
        _, cols2 = overlay_mod.board_triangles(BOARD, first_dark=False)
        self.assertGreater(cols2[first_cell][0], 0.5)                  # light otherwise

    def test_corner_errors_and_drawing_do_not_alter_the_input(self):
        img = np.zeros((200, 300, 3), np.uint8)
        det = np.array([[50.0, 50.0], [100.0, 100.0]])
        pred = det + np.array([3.0, 4.0])
        np.testing.assert_allclose(overlay_mod.corner_errors(det, pred), [5.0, 5.0])
        out = overlay_mod.draw_overlay(img, det, pred)
        self.assertEqual(int(img.sum()), 0)
        self.assertGreater(int(out.sum()), 0)
        self.assertGreater(int(overlay_mod.draw_overlay(img, None, pred).sum()), 0)

    def test_frustum_has_four_rays_and_four_edges(self):
        lines = overlay_mod.frustum_lines(K, (1920, 1080), depth=0.2)
        self.assertEqual(len(lines), 16)
        self.assertTrue(np.allclose(lines[:, 2][1::2], 0.2))


if __name__ == "__main__":
    unittest.main()
