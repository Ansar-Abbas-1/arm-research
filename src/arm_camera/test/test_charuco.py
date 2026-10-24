"""Hardware-free tests of the ChArUco library: detection and pose on rendered boards, partial views,
the frame conventions, recovery of the camera mount from a board with a known pose, and the
sensitivity of that recovery to errors in the board pose that the user measures."""
import unittest

import numpy as np

from arm_camera import board as board_mod
from arm_camera import charuco
from arm_camera import mount_calibration as mc
from arm_camera import session
from arm_camera.geometry import invert_transform, make_transform, rot_x, rot_y, rot_z, rotation_error_deg, ypr_from_matrix
from test_calibration_pipeline import X_TRUE, arm_pose

K = np.array([[1521.4, 0.0, 951.9], [0.0, 1521.9, 532.1], [0.0, 0.0, 1.0]])
SPEC = charuco.CharucoSpec(squares_x=8, squares_y=6, square_length=0.030, marker_length=0.022, dictionary="4X4_50")
ZERO_D = np.zeros(5)


def facing_pose(yaw, tilt, centre_depth, shift=(0.0, 0.0)):
    """T_cam_board for a board seen upright and facing the camera (board z toward the camera)."""
    R = rot_x(np.pi) @ rot_z(yaw) @ rot_y(tilt)
    # put the board CENTRE (W/2, H/2) on the optical axis, then shift
    centre_cam = R @ np.array([SPEC.width / 2.0, SPEC.height / 2.0, 0.0])
    t = np.array([shift[0], shift[1], centre_depth]) - centre_cam
    return make_transform(R, t)


class CharucoPoseTest(unittest.TestCase):
    def test_pose_from_a_rendered_board(self):
        for yaw, tilt, depth in ((0.0, 0.4, 0.45), (0.5, 0.5, 0.40), (-0.6, 0.3, 0.55)):
            T_true = facing_pose(yaw, tilt, depth)
            det = charuco.detect(charuco.render_board_view(SPEC, T_true, K), SPEC, K, ZERO_D)
            self.assertIsNotNone(det, "board not found at yaw=%.1f" % yaw)
            self.assertEqual(len(det.ids), SPEC.num_corners)
            T, rms, ratio = charuco.estimate_pose(det, SPEC, K, ZERO_D)
            self.assertLess(rms, 0.6)
            self.assertLess(np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000.0, 2.5)
            self.assertLess(rotation_error_deg(T[:3, :3], T_true[:3, :3]), 0.8)

    def test_a_board_partly_outside_the_image_still_gives_a_pose(self):
        # The point of ChArUco over a plain chessboard: ids make partial views usable.
        T_true = facing_pose(0.2, 0.5, 0.40, shift=(0.22, 0.0))     # the right part of the board is cut off by the image edge
        det = charuco.detect(charuco.render_board_view(SPEC, T_true, K), SPEC, K, ZERO_D)
        self.assertIsNotNone(det)
        self.assertLess(len(det.ids), SPEC.num_corners)
        self.assertGreaterEqual(len(det.ids), 8)
        T, rms, _ = charuco.estimate_pose(det, SPEC, K, ZERO_D)
        self.assertLess(rms, 0.8)
        self.assertLess(np.linalg.norm(T[:3, 3] - T_true[:3, 3]) * 1000.0, 6.0)

    def test_nothing_found_without_a_board(self):
        self.assertIsNone(charuco.detect(np.full((480, 640), 128, np.uint8), SPEC, K, ZERO_D))

    def test_ambiguity_is_high_when_frontal_and_low_when_tilted(self):
        frontal = charuco.detect(charuco.render_board_view(SPEC, facing_pose(0.0, 0.0, 0.5), K), SPEC, K, ZERO_D)
        tilted = charuco.detect(charuco.render_board_view(SPEC, facing_pose(0.0, 0.7, 0.45), K), SPEC, K, ZERO_D)
        _, _, r_frontal = charuco.estimate_pose(frontal, SPEC, K, ZERO_D)
        _, _, r_tilted = charuco.estimate_pose(tilted, SPEC, K, ZERO_D)
        self.assertGreater(r_frontal, r_tilted)
        self.assertGreater(charuco.tilt_deg(facing_pose(0.0, 0.7, 0.45)), 30.0)


class FrameConventionTest(unittest.TestCase):
    def test_yaw_pitch_roll_round_trip(self):
        for y, p, r in ((0.3, -0.5, 1.2), (-2.0, 0.9, -0.4), (3.0, 0.0, 0.0)):
            R = rot_z(y) @ rot_y(p) @ rot_x(r)
            yy, pp, rr = ypr_from_matrix(R)
            np.testing.assert_allclose(rot_z(yy) @ rot_y(pp) @ rot_x(rr), R, atol=1e-12)

    def test_board_frame_has_its_origin_at_the_bottom_left_corner_with_y_up(self):
        corners = SPEC.corner_points()
        self.assertEqual(len(corners), SPEC.num_corners)
        np.testing.assert_allclose(corners[0], [0.030, 0.030, 0.0])          # id 0: one square in from the corner
        np.testing.assert_allclose(corners[1], [0.060, 0.030, 0.0])          # ids run along x
        np.testing.assert_allclose(corners[SPEC.squares_x - 1], [0.030, 0.060, 0.0])   # then up in y

    def test_level_board_pose_corner_and_centre_conventions_agree(self):
        half = np.array([SPEC.width / 2, SPEC.height / 2, 0.0])
        corner = charuco.board_pose_in_base(0.40, 0.10, 0.0, 30.0, SPEC, origin="corner")
        centre = charuco.board_pose_in_base(0.40, 0.10, 0.0, 30.0, SPEC, origin="center")
        np.testing.assert_allclose(corner[:3, 3], [0.40, 0.10, 0.0])                       # the corner is where it was put
        np.testing.assert_allclose(centre[:3, :3] @ half + centre[:3, 3], [0.40, 0.10, 0.0], atol=1e-12)   # the centre is where it was put
        np.testing.assert_allclose(corner[:3, :3], centre[:3, :3])
        self.assertAlmostEqual(corner[2, 2], 1.0)                                          # face up: board z is base z
        # yaw 90 degrees: the board's x axis points along base y
        np.testing.assert_allclose(charuco.board_pose_in_base(0, 0, 0, 90.0, SPEC)[:3, 0], [0, 1, 0], atol=1e-12)
        with self.assertRaises(ValueError):
            charuco.board_pose_in_base(0, 0, 0, 0, SPEC, origin="middle")


def visible(T_cam_board, margin=60):
    corners = [(0, 0, 0), (SPEC.width, 0, 0), (SPEC.width, SPEC.height, 0), (0, SPEC.height, 0)]
    px = board_mod.project(K, ZERO_D, T_cam_board, corners)
    return T_cam_board[2, 2] < -0.5 and 0.2 < T_cam_board[2, 3] < 0.9 and \
        px[:, 0].min() > margin and px[:, 0].max() < 1920 - margin and px[:, 1].min() > margin and px[:, 1].max() < 1080 - margin


class MountFromKnownBoardTest(unittest.TestCase):
    BOARD_TRUE = charuco.board_pose_in_base(0.34, 0.03, 0.0, 12.0, SPEC, origin="center")

    def views(self, n, seed, fk_noise_mm=0.0):
        rng = np.random.RandomState(seed)
        out = []
        while len(out) < n:
            T_be = arm_pose(rng.uniform(-0.45, 0.45), rng.uniform(1.9, 2.7), rng.uniform(0.15, 0.40), rng.uniform(0.25, 0.50))
            T_cb = invert_transform(T_be @ X_TRUE) @ self.BOARD_TRUE
            if not visible(T_cb):
                continue
            det = charuco.detect(charuco.render_board_view(SPEC, T_cb, K), SPEC, K, ZERO_D)
            if det is None:
                continue
            T_cb_est, rms, _ = charuco.estimate_pose(det, SPEC, K, ZERO_D)
            T_be_noisy = T_be.copy()
            T_be_noisy[:3, 3] += rng.normal(0, fk_noise_mm / 1000.0, 3)
            out.append((T_be_noisy, T_cb_est))
        return out

    def test_a_single_view_recovers_the_mount_when_the_board_pose_is_known(self):
        T_be, T_cb = self.views(1, 1)[0]
        X = charuco.mount_from_known_board(T_be, T_cb, self.BOARD_TRUE)
        self.assertLess(np.linalg.norm(X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, 4.0)
        self.assertLess(rotation_error_deg(X[:3, :3], X_TRUE[:3, :3]), 1.0)

    def test_robust_average_over_views_with_arm_pose_noise(self):
        views = self.views(8, 2, fk_noise_mm=2.0)
        est = charuco.robust_mount([charuco.mount_from_known_board(a, b, self.BOARD_TRUE) for a, b in views])
        self.assertLess(np.linalg.norm(est.X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, 3.0)
        self.assertLess(rotation_error_deg(est.X[:3, :3], X_TRUE[:3, :3]), 1.0)
        self.assertGreater(est.translation_spread_mm, 0.0)       # the 2 mm arm noise shows up as spread
        self.assertTrue(est.inliers.all())

    def test_error_in_the_measured_board_yaw_goes_straight_into_the_mount(self):
        # Documents the sensitivity: the board pose you measure is the accuracy limit.
        views = self.views(6, 3)
        errors = {}
        for yaw_err in (0.0, 1.0, 3.0):
            wrong = charuco.board_pose_in_base(0.34, 0.03, 0.0, 12.0 + yaw_err, SPEC, origin="center")
            est = charuco.robust_mount([charuco.mount_from_known_board(a, b, wrong) for a, b in views])
            errors[yaw_err] = np.linalg.norm(est.X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, rotation_error_deg(est.X[:3, :3], X_TRUE[:3, :3])
        self.assertLess(errors[0.0][0], 3.0)
        self.assertGreater(errors[3.0][0], 3.0 * errors[1.0][0] * 0.6)     # grows with the yaw error
        self.assertAlmostEqual(errors[3.0][1], 3.0, delta=0.6)             # a 3 degree yaw error is a ~3 degree mount rotation error
        self.assertGreater(errors[3.0][0], 5.0)                            # millimetres: several mm for a 3 degree yaw error at ~0.4 m

    def test_robust_mount_rejects_an_outlier_view(self):
        good = [make_transform(rot_z(0.3), [0.05 + 0.0005 * i, -0.02, 0.07]) for i in range(7)]
        bad = make_transform(rot_z(0.3), [0.05 + 0.040, -0.02, 0.07])
        est = charuco.robust_mount(good + [bad])
        self.assertEqual(int(est.inliers.sum()), 7)
        self.assertFalse(est.inliers[-1])
        self.assertLess(abs(est.X[0, 3] - 0.0515) * 1000.0, 1.0)


class MountSolveTest(MountFromKnownBoardTest):
    def partial_views(self, n, seed):
        """Arm poses over a wide range, board possibly only partly in view (what ChArUco allows):
        needed for rotation diversity, since keeping the whole board in view limits the arm to
        about 12 degrees of rotation between views."""
        rng = np.random.RandomState(seed)
        out = []
        while len(out) < n:
            T_be = arm_pose(rng.uniform(-0.9, 0.9), rng.uniform(1.7, 2.8), rng.uniform(0.10, 0.50), rng.uniform(0.20, 0.60))
            T_cb = invert_transform(T_be @ X_TRUE) @ self.BOARD_TRUE
            centre = board_mod.project(K, ZERO_D, T_cb, [(SPEC.width / 2, SPEC.height / 2, 0.0)])[0]
            if T_cb[2, 2] > -0.5 or not 0.2 < T_cb[2, 3] < 0.9 or not (150 < centre[0] < 1770 and 150 < centre[1] < 930):
                continue
            det = charuco.detect(charuco.render_board_view(SPEC, T_cb, K), SPEC, K, ZERO_D, min_corners=12)
            if det is None:
                continue
            T_est, rms, ratio = charuco.estimate_pose(det, SPEC, K, ZERO_D)
            if rms < 1.0 and ratio < 0.8:
                out.append((T_be, T_est, len(det.ids)))
        return out

    def samples(self, n, seed):
        return [mc.MountSample(a, b, num_corners=c, rms=0.3) for a, b, c in self.partial_views(n, seed)]

    def test_known_pose_method_with_the_handeye_cross_check(self):
        res = mc.solve(self.samples(12, 4), T_base_board=self.BOARD_TRUE)
        self.assertEqual(res.method, "known")
        self.assertLess(np.linalg.norm(res.X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, 3.0)
        self.assertIn("board_pose_error_mm", res.compare)
        self.assertLess(res.compare["mount_difference_mm"], 8.0)            # the two methods agree
        self.assertLess(res.compare["board_pose_error_mm"], 10.0)           # the measured pose is right
        self.assertLess(res.compare["board_pose_error_deg"], 1.5)

    def test_cross_check_exposes_a_wrong_board_pose_measurement(self):
        wrong = charuco.board_pose_in_base(0.34, 0.03, 0.0, 12.0 + 3.0, SPEC, origin="center")      # yaw measured 3 degrees off
        res = mc.solve(self.samples(12, 4), T_base_board=wrong)
        self.assertAlmostEqual(res.compare["board_pose_error_deg"], 3.0, delta=1.5)
        self.assertGreater(res.compare["mount_difference_mm"], 4.0)

    def test_handeye_method_needs_no_board_pose(self):
        res = mc.solve(self.samples(12, 5), T_base_board=None)
        self.assertEqual(res.method, "handeye")
        self.assertLess(np.linalg.norm(res.X[:3, 3] - X_TRUE[:3, 3]) * 1000.0, 8.0)
        self.assertLess(rotation_error_deg(res.X[:3, :3], X_TRUE[:3, :3]), 1.5)
        self.assertLess(np.linalg.norm(res.B[:3, 3] - self.BOARD_TRUE[:3, 3]) * 1000.0, 10.0)   # and finds where the board is

    def test_too_few_views_for_handeye_is_refused_but_known_pose_still_works(self):
        few = self.samples(4, 6)
        with self.assertRaises(ValueError):
            mc.solve(few, T_base_board=None)
        res = mc.solve(few, T_base_board=self.BOARD_TRUE)
        self.assertEqual(res.method, "known")
        self.assertTrue(any("cross-check skipped" in n for n in res.notes))

    def test_output_file_round_trips_and_is_readable_by_the_tf_publisher_reader(self):
        import os, tempfile
        res = mc.solve(self.samples(10, 7), T_base_board=self.BOARD_TRUE)
        path = os.path.join(tempfile.mkdtemp(), "mount.yaml")
        mc.save_mount_yaml(path, res, SPEC, board_origin={"x": 0.34, "y": 0.03, "z": 0.0, "yaw_deg": 12.0, "origin": "center"})
        X, d = mc.load_mount_yaml(path)
        np.testing.assert_allclose(X, res.X, atol=1e-9)
        self.assertEqual(d["method"], "charuco_known")
        loaded = session.load_hand_eye(path)                  # what hand_eye_tf_publisher.py uses
        np.testing.assert_allclose(loaded["X"], res.X, atol=1e-9)
        self.assertIsNone(loaded["board"])


class PrintableTest(unittest.TestCase):
    def test_svg_has_the_exact_physical_size_and_the_png_marks_the_frame(self):
        svg = charuco.board_svg(SPEC, margin_mm=10.0)
        self.assertIn('width="260.0mm"', svg)                  # 240 mm board + 2 x 10 mm margin
        self.assertIn("data:image/png;base64", svg)
        png = charuco.annotated_board_png(SPEC)
        self.assertEqual(png.ndim, 3)
        self.assertGreater(int(png[..., 2].astype(int).sum() - png[..., 1].astype(int).sum()), 0)   # red x arrow drawn


if __name__ == "__main__":
    unittest.main()
