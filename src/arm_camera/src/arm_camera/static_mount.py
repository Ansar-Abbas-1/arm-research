"""Camera mount (T_end_effector_camera) from hand measurements made in room directions (no ROS).

The operator puts the arm in a pose, measures where the lens centre is and which way the camera looks, in
the directions of the base frame (forward = +x, left = +y, up = +z), and this converts them to the transform
that camera_static_tf.launch needs, using the end effector pose of that moment (forward kinematics / TF):

    T_ee_cam = inv(T_base_ee) @ T_base_cam,   T_base_cam = (p_ee + offset, [x_c y_c z_c])

Optical frame: z out of the lens, x right in the image, y down in the image (y = z cross x).
Typical accuracy of hand measurements is a few mm and a few degrees: a placeholder, not a calibration.
"""
import numpy as np

from arm_camera.geometry import invert_transform, make_transform, ypr_from_matrix

WORDS = {"forward": (1, 0, 0), "back": (-1, 0, 0), "backward": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0),
         "up": (0, 0, 1), "down": (0, 0, -1)}


def parse_direction(text):
    """'forward', 'down' ... or 'x,y,z' (any length, in forward/left/up axes) -> unit vector."""
    t = str(text).strip().lower()
    v = np.array(WORDS[t], float) if t in WORDS else np.array([float(c) for c in t.split(",")], float)
    if v.shape != (3,) or np.linalg.norm(v) < 1e-9:
        raise ValueError("direction %r must be a word (%s) or three numbers x,y,z" % (text, ", ".join(sorted(WORDS))))
    return v / np.linalg.norm(v)


def camera_rotation(optical_axis, image_right, image_down=None, max_skew_deg=5.0):
    """Rotation whose columns are the camera x (image right), y (image down), z (optical axis) in the measuring frame.

    image_right is made exactly perpendicular to the optical axis (the skew is reported). If image_down is also
    given it must agree with z cross x: a disagreement means a mirrored or mistaken direction."""
    z = np.asarray(optical_axis, float); z = z / np.linalg.norm(z)
    x = np.asarray(image_right, float); x = x / np.linalg.norm(x)
    skew = np.degrees(np.arccos(np.clip(abs(float(x @ z)), 0.0, 1.0)))
    skew = 90.0 - skew
    if skew > max_skew_deg:
        raise ValueError("image-right and the optical axis are %.1f deg away from perpendicular (limit %.1f): "
                         "check the two directions" % (skew, max_skew_deg))
    x = x - (x @ z) * z; x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    if image_down is not None:
        d = np.asarray(image_down, float); d = d / np.linalg.norm(d)
        if float(d @ y) < np.cos(np.radians(max_skew_deg + 10.0)):
            raise ValueError("image-down %s disagrees with optical axis x image-right = %s: one of the three directions "
                             "is wrong or mirrored" % (np.round(d, 2), np.round(y, 2)))
    return np.column_stack([x, y, z]), skew


def mount_from_measurements(T_base_ee, lens_offset, optical_axis, image_right, image_down=None):
    """T_ee_cam from the end effector pose and the measurements (offset of the lens centre from the end
    effector origin, in base axes). Returns (T_ee_cam, skew_deg)."""
    R_bc, skew = camera_rotation(optical_axis, image_right, image_down)
    T_base_cam = make_transform(R_bc, np.asarray(T_base_ee)[:3, 3] + np.asarray(lens_offset, float))
    return invert_transform(T_base_ee) @ T_base_cam, skew


LINK5_TO_END_EFFECTOR = 0.105   # m along link_5's z axis (end_effector_joint in arm.urdf); both frames share orientation


def optical_centre(point, optical_axis, advance):
    """Moves a measured point (for example on the sensor) `advance` metres along the optical axis, toward the
    scene: the pinhole of the camera model is at the lens, a few mm in front of the sensor."""
    a = np.asarray(optical_axis, float)
    return np.asarray(point, float) + advance * a / np.linalg.norm(a)


def offset_from_end_effector(offset_from_link5):
    """Offset of a point from the end_effector origin, given its offset from link_5's origin (both in the
    link_5 / end_effector axes)."""
    return np.asarray(offset_from_link5, float) - np.array([0.0, 0.0, LINK5_TO_END_EFFECTOR])


def launch_args(T_ee_cam):
    """The arguments of camera_static_tf.launch for a transform, as text."""
    yaw, pitch, roll = ypr_from_matrix(T_ee_cam[:3, :3])
    t = T_ee_cam[:3, 3]
    return "x:=%.4f y:=%.4f z:=%.4f yaw:=%.4f pitch:=%.4f roll:=%.4f" % (t[0], t[1], t[2], yaw, pitch, roll)
