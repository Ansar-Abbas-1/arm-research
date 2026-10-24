"""Camera-mount calibration (X = T_end_effector_camera) from ChArUco views (no ROS).

Two independent ways to get X from the same stored views:
  known   the board pose in base_link is measured by the user:  X_i = inv(T_be) T_bb inv(T_cb) per view,
          robust average over views. Fast, needs few views, accuracy limited by the measured pose.
  handeye the board pose is NOT needed: OpenCV calibrateHandEye on the relative arm / camera motions
          (arm_camera.hand_eye). Needs 8 or more views with rotation about two non-parallel axes (this
          arm: joint_1, and the pitch joints), accuracy limited by the arm.
When both are available the difference between them, and between the board pose hand-eye finds and
the one you measured, says how good your measurement was.
"""
import os
from dataclasses import dataclass, field

import numpy as np
import yaml

from arm_camera import charuco, hand_eye
from arm_camera.geometry import make_transform, matrix_from_quat, quat_from_matrix, rotation_error_deg, ypr_from_matrix


@dataclass
class MountSample:
    T_base_ee: np.ndarray
    T_cam_board: np.ndarray
    num_corners: int = 0
    rms: float = 0.0
    name: str = ""


@dataclass
class MountResult:
    method: str                          # "known" or "handeye"
    X: np.ndarray                        # T_ee_cam
    B: np.ndarray                        # board pose in base_link used or found
    spread_translation_mm: float
    spread_rotation_deg: float
    num_views: int
    notes: list = field(default_factory=list)
    compare: dict = field(default_factory=dict)
    known: object = None                 # charuco.MountEstimate
    handeye: object = None               # hand_eye.Result


def solve(samples, T_base_board=None, method="auto", min_handeye_views=8):
    """Solve from stored views. method: 'known' (needs T_base_board), 'handeye', or 'auto' = known when a
    board pose is given, else hand-eye. With a board pose AND enough diverse views, hand-eye is also run
    and the comparison filled in."""
    if not samples:
        raise ValueError("no samples")
    if method == "auto":
        method = "known" if T_base_board is not None else "handeye"
    if method == "known" and T_base_board is None:
        raise ValueError("method 'known' needs the board pose (x y z yaw)")

    known = None
    if T_base_board is not None:
        known = charuco.robust_mount([charuco.mount_from_known_board(s.T_base_ee, s.T_cam_board, T_base_board) for s in samples])

    he, notes = None, []
    he_samples = [hand_eye.Sample(T_base_ee=s.T_base_ee, T_cam_board=s.T_cam_board) for s in samples]
    rot, axis = hand_eye.motion_diversity(he_samples)
    if len(samples) >= min_handeye_views and rot >= 20.0 and axis >= 20.0:
        results = hand_eye.solve_all(he_samples)
        if results:
            he = results[0]
    elif method == "handeye":
        raise ValueError("hand-eye needs at least %d views with 20+ degrees of rotation about two different axes "
                         "(have %d views, %.0f deg rotation, %.0f deg between axes); move joint_1 AND the pitch joints"
                         % (min_handeye_views, len(samples), rot, axis))
    else:
        notes.append("hand-eye cross-check skipped: %d views, %.0f deg of rotation, %.0f deg between rotation axes "
                     "(needs %d views, 20 and 20)" % (len(samples), rot, axis, min_handeye_views))

    compare = {}
    if known is not None and he is not None:
        compare["mount_difference_mm"] = float(1000.0 * np.linalg.norm(known.X[:3, 3] - he.X[:3, 3]))
        compare["mount_difference_deg"] = rotation_error_deg(known.X[:3, :3], he.X[:3, :3])
        compare["board_pose_error_mm"] = float(1000.0 * np.linalg.norm(T_base_board[:3, 3] - he.B[:3, 3]))
        compare["board_pose_error_deg"] = rotation_error_deg(T_base_board[:3, :3], he.B[:3, :3])

    if method == "known":
        res = MountResult(method="known", X=known.X, B=T_base_board, spread_translation_mm=known.translation_spread_mm,
                          spread_rotation_deg=known.rotation_spread_deg, num_views=int(known.inliers.sum()), notes=notes, compare=compare,
                          known=known, handeye=he)
        if not known.inliers.all():
            res.notes.append("%d of %d views rejected as outliers" % ((~known.inliers).sum(), len(samples)))
    else:
        if he is None:
            raise ValueError("hand-eye did not converge")
        res = MountResult(method="handeye", X=he.X, B=he.B, spread_translation_mm=he.position_scatter_mm,
                          spread_rotation_deg=he.rotation_scatter_deg, num_views=len(samples), notes=notes, compare=compare,
                          known=known, handeye=he)
    return res



def report(res):
    """Human-readable result, ending with the arguments for camera_static_tf.launch."""
    X = res.X
    yaw, pitch, roll = ypr_from_matrix(X[:3, :3])
    lines = ["method: %s on %d views" % (res.method, res.num_views),
             "camera in end_effector: x=%.4f y=%.4f z=%.4f m, yaw=%.4f pitch=%.4f roll=%.4f rad" % (X[0, 3], X[1, 3], X[2, 3], yaw, pitch, roll),
             "view-to-view spread: %.1f mm / %.2f deg (arm accuracy and board-pose error show up here)" % (res.spread_translation_mm, res.spread_rotation_deg)]
    if res.compare:
        c = res.compare
        lines.append("cross-check with hand-eye (needs no board pose): mount differs by %.1f mm / %.2f deg; the board pose hand-eye finds "
                     "differs from the one you entered by %.1f mm / %.2f deg" % (c["mount_difference_mm"], c["mount_difference_deg"],
                                                                                  c["board_pose_error_mm"], c["board_pose_error_deg"]))
        lines.append("   -> a large board-pose difference means the measurement of the board (or the arm) is off by about that much")
    lines += ["note: " + n for n in res.notes]
    lines.append("static TF: roslaunch arm_camera camera_static_tf.launch x:=%.4f y:=%.4f z:=%.4f yaw:=%.4f pitch:=%.4f roll:=%.4f"
                 % (X[0, 3], X[1, 3], X[2, 3], yaw, pitch, roll))
    return "\n".join(lines)



# ---- storage -----------------------------------------------------------------------------------------------

def save_samples(directory, samples):
    os.makedirs(directory, exist_ok=True)
    for i, s in enumerate(samples):
        np.savez(os.path.join(directory, "view_%03d.npz" % i), T_base_ee=s.T_base_ee, T_cam_board=s.T_cam_board,
                 num_corners=s.num_corners, rms=s.rms)


def append_sample(directory, index, sample, image_bgr=None):
    os.makedirs(directory, exist_ok=True)
    stem = os.path.join(directory, "view_%03d" % index)
    np.savez(stem + ".npz", T_base_ee=sample.T_base_ee, T_cam_board=sample.T_cam_board, num_corners=sample.num_corners, rms=sample.rms)
    if image_bgr is not None:
        import cv2
        cv2.imwrite(stem + ".png", image_bgr)


def load_samples(directory):
    import glob
    out = []
    for path in sorted(glob.glob(os.path.join(directory, "view_*.npz"))):
        d = np.load(path)
        out.append(MountSample(d["T_base_ee"], d["T_cam_board"], int(d["num_corners"]), float(d["rms"]), os.path.basename(path)))
    return out


def _pose(T):
    return {"translation": [float(v) for v in T[:3, 3]], "quaternion_xyzw": [float(v) for v in quat_from_matrix(T[:3, :3])]}


def save_mount_yaml(path, result, spec, board_origin=None):
    """hand_eye.yaml-compatible output: parent end_effector, child camera_optical_frame, translation and
    quaternion_xyzw (what hand_eye_tf_publisher.py reads), plus how it was obtained."""
    import datetime
    data = {"parent_frame": "end_effector", "child_frame": "camera_optical_frame"}
    data.update(_pose(result.X))
    data["board_in_base"] = _pose(result.B)
    data["method"] = "charuco_" + result.method
    data["num_samples"] = int(result.num_views)
    data["metrics"] = {"position_scatter_mm": float(result.spread_translation_mm),
                       "rotation_scatter_deg": float(result.spread_rotation_deg),
                       "reprojection_rms_px": float("nan")}
    data["board"] = {"type": "charuco", "squares_x": spec.squares_x, "squares_y": spec.squares_y,
                     "square_length": spec.square_length, "marker_length": spec.marker_length, "dictionary": spec.dictionary,
                     "frame": "origin at the bottom-left outer corner as printed, x right, y up, z toward the viewer"}
    if board_origin:
        data["board_pose_input"] = board_origin
    if result.compare:
        data["cross_check"] = {k: float(v) for k, v in result.compare.items()}
    data["created"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)


def load_mount_yaml(path):
    with open(path) as f:
        d = yaml.safe_load(f)
    return make_transform(matrix_from_quat(d["quaternion_xyzw"]), d["translation"]), d
