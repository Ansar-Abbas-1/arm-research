"""Hardware-free tests of the hand-measured camera mount: directions in room axes to T_end_effector_camera."""
import unittest

import numpy as np

from arm_camera import static_mount as sm
from arm_camera.geometry import invert_transform, make_transform, rot_x, rot_y, rot_z, ypr_from_matrix

# end_effector pose of the simulator at zero joints (read from TF): z forward, x down, y left
R_EE = np.array([[0.07, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0.07]])
R_EE, _ = np.linalg.qr(R_EE)           # make it exactly orthonormal
if np.linalg.det(R_EE) < 0:
    R_EE[:, 0] *= -1
T_BE = make_transform(R_EE, [0.039, 0.0, 0.452])


class DirectionTest(unittest.TestCase):
    def test_words_and_vectors(self):
        np.testing.assert_allclose(sm.parse_direction("forward"), [1, 0, 0])
        np.testing.assert_allclose(sm.parse_direction("Down"), [0, 0, -1])
        np.testing.assert_allclose(sm.parse_direction("1,0,-1"), np.array([1, 0, -1]) / np.sqrt(2))
        with self.assertRaises(Exception):
            sm.parse_direction("sideways")
        with self.assertRaises(ValueError):
            sm.parse_direction("0,0,0")


class RotationTest(unittest.TestCase):
    def test_right_handed_frame_from_two_directions(self):
        R, skew = sm.camera_rotation([1, 0, 0], [0, -1, 0])      # looks forward, image right = room right
        np.testing.assert_allclose(R[:, 2], [1, 0, 0])
        np.testing.assert_allclose(R[:, 0], [0, -1, 0])
        np.testing.assert_allclose(R[:, 1], [0, 0, -1])          # image down = room down
        self.assertAlmostEqual(np.linalg.det(R), 1.0)
        self.assertLess(skew, 1e-9)

    def test_small_skew_is_corrected_and_reported(self):
        R, skew = sm.camera_rotation([1, 0, 0], [0.05, -1, 0])   # image right 2.9 deg off perpendicular
        self.assertAlmostEqual(abs(R[:, 0] @ R[:, 2]), 0.0, places=12)
        self.assertAlmostEqual(skew, np.degrees(np.arctan(0.05)), places=6)
        self.assertAlmostEqual(np.linalg.det(R), 1.0)

    def test_a_large_skew_is_refused(self):
        with self.assertRaises(ValueError):
            sm.camera_rotation([1, 0, 0], [0.5, -1, 0])

    def test_a_mirrored_image_down_is_refused(self):
        with self.assertRaises(ValueError):
            sm.camera_rotation([1, 0, 0], [0, -1, 0], image_down=[0, 0, 1])      # down given as up
        sm.camera_rotation([1, 0, 0], [0, -1, 0], image_down=[0, 0, -1])          # consistent: fine


class ConversionTest(unittest.TestCase):
    def test_sensor_to_optical_centre_moves_along_the_axis(self):
        np.testing.assert_allclose(sm.optical_centre([0.06, 0, 0.025], [2, 0, 0], 0.004), [0.064, 0, 0.025])
        np.testing.assert_allclose(sm.optical_centre([0.06, 0, 0.025], [0, 0, 1], 0.0), [0.06, 0, 0.025])

    def test_link5_origin_to_end_effector_origin(self):
        np.testing.assert_allclose(sm.offset_from_end_effector([0.06, 0, 0.025]), [0.06, 0, 0.025 - 0.105])

    def test_directly_in_end_effector_axes(self):
        # camera 6 cm along ee x and 8 cm behind the origin, looking along ee +x, right = ee +y, down = ee +z
        T, _ = sm.mount_from_measurements(np.eye(4), [0.06, 0, -0.08], [1, 0, 0], [0, 1, 0], [0, 0, 1])
        np.testing.assert_allclose(T[:3, 3], [0.06, 0, -0.08])
        np.testing.assert_allclose(T[:3, 0], [0, 1, 0])      # image right
        np.testing.assert_allclose(T[:3, 1], [0, 0, 1])      # image down
        np.testing.assert_allclose(T[:3, 2], [1, 0, 0])      # optical axis


class MountTest(unittest.TestCase):
    def test_camera_looking_along_the_tool_axis(self):
        # tool axis (ee z) is nearly forward; camera looks along it, image right = room right
        z_e = R_EE[:, 2]
        right = np.cross([0, 0, 1], z_e)                      # horizontal, perpendicular to z_e
        right = -right / np.linalg.norm(right) if right[1] > 0 else right / np.linalg.norm(right)
        T, _ = sm.mount_from_measurements(T_BE, [0, 0, 0], z_e, right)
        np.testing.assert_allclose(T[:3, 3], 0.0, atol=1e-12)
        np.testing.assert_allclose(T[:3, :3] @ [0, 0, 1], [0, 0, 1], atol=1e-9)      # optical axis = ee z
        self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1.0)

    def test_offset_is_expressed_in_end_effector_axes(self):
        # lens 5 cm forward and 2 cm up of the origin, looking forward
        T, _ = sm.mount_from_measurements(T_BE, [0.05, 0.0, 0.02], [1, 0, 0], [0, -1, 0])
        expected = R_EE.T @ np.array([0.05, 0.0, 0.02])
        np.testing.assert_allclose(T[:3, 3], expected, atol=1e-12)

    def test_the_transform_reproduces_the_measured_camera_pose(self):
        T, _ = sm.mount_from_measurements(T_BE, [0.03, -0.01, 0.04], [1, 0, -1], [0, -1, 0])
        T_bc = T_BE @ T
        np.testing.assert_allclose(T_bc[:3, 3], T_BE[:3, 3] + [0.03, -0.01, 0.04], atol=1e-12)
        np.testing.assert_allclose(T_bc[:3, 2], np.array([1, 0, -1]) / np.sqrt(2), atol=1e-12)   # optical axis
        np.testing.assert_allclose(T_bc[:3, 0], [0, -1, 0], atol=1e-12)                           # image right

    def test_launch_arguments_round_trip_to_the_same_rotation(self):
        T, _ = sm.mount_from_measurements(T_BE, [0.03, 0.01, -0.02], [1, 0, -0.6], [0, -1, 0])
        yaw, pitch, roll = ypr_from_matrix(T[:3, :3])
        np.testing.assert_allclose(rot_z(yaw) @ rot_y(pitch) @ rot_x(roll), T[:3, :3], atol=1e-12)
        text = sm.launch_args(T)
        for key in ("x:=", "y:=", "z:=", "yaw:=", "pitch:=", "roll:="):
            self.assertIn(key, text)


if __name__ == "__main__":
    unittest.main()
