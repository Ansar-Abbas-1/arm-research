"""Reading and writing calibration data: capture sessions and the hand-eye result."""
import datetime
import glob
import os

import numpy as np
import yaml

from arm_camera import hand_eye
from arm_camera.board import Board
from arm_camera.geometry import make_transform, matrix_from_quat, quat_from_matrix

PARENT_FRAME = "end_effector"
CHILD_FRAME = "camera_optical_frame"


def save_sample(directory, index, sample, image_bgr=None):
    """One capture: poses and corners in an .npz, plus the image for later re-checks."""
    os.makedirs(directory, exist_ok=True)
    stem = os.path.join(directory, "sample_%03d" % index)
    np.savez(stem + ".npz", T_base_ee=sample.T_base_ee, T_cam_board=sample.T_cam_board, corners=sample.corners)
    if image_bgr is not None:
        import cv2
        cv2.imwrite(stem + ".png", image_bgr)
    return stem


def load_samples(directory):
    samples = []
    for path in sorted(glob.glob(os.path.join(directory, "sample_*.npz"))):
        d = np.load(path)
        samples.append(hand_eye.Sample(T_base_ee=d["T_base_ee"], T_cam_board=d["T_cam_board"],
                                       corners=d["corners"], name=os.path.basename(path)))
    return samples


def write_session_meta(directory, board, K, D, image_size, first_square_dark=None):
    os.makedirs(directory, exist_ok=True)
    meta = {"board": {"cols": board.cols, "rows": board.rows, "square_size": board.square_size},
            "K": np.asarray(K).reshape(-1).tolist(), "D": np.asarray(D).reshape(-1).tolist(),
            "image_size": list(image_size), "first_square_dark": first_square_dark}
    with open(os.path.join(directory, "session.yaml"), "w") as f:
        yaml.safe_dump(meta, f)


def read_session_meta(directory):
    with open(os.path.join(directory, "session.yaml")) as f:
        meta = yaml.safe_load(f)
    board = Board(meta["board"]["cols"], meta["board"]["rows"], meta["board"]["square_size"])
    K = np.array(meta["K"], dtype=np.float64).reshape(3, 3)
    D = np.array(meta["D"], dtype=np.float64)
    return board, K, D, tuple(meta["image_size"]), meta.get("first_square_dark")


def _pose_dict(T):
    return {"translation": [float(v) for v in T[:3, 3]], "quaternion_xyzw": [float(v) for v in quat_from_matrix(T[:3, :3])]}


def _pose_from_dict(d):
    return make_transform(matrix_from_quat(d["quaternion_xyzw"]), d["translation"])


def save_hand_eye(path, result, board, num_samples, diversity, first_square_dark=None, image_size=None):
    data = {"parent_frame": PARENT_FRAME, "child_frame": CHILD_FRAME}
    data.update(_pose_dict(result.X))
    data["board_in_base"] = _pose_dict(result.B)
    data["method"] = result.method
    data["num_samples"] = int(num_samples)
    data["metrics"] = {"position_scatter_mm": float(result.position_scatter_mm),
                       "rotation_scatter_deg": float(result.rotation_scatter_deg),
                       "reprojection_rms_px": float(result.reprojection_rms_px),
                       "max_rotation_between_poses_deg": float(diversity[0]),
                       "max_rotation_axis_angle_deg": float(diversity[1])}
    data["board"] = {"cols": board.cols, "rows": board.rows, "square_size": board.square_size,
                     "first_square_dark": first_square_dark}
    if image_size is not None:
        data["image_size"] = [int(image_size[0]), int(image_size[1])]
    data["created"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)


def load_hand_eye(path):
    """dict with X (T_ee_cam), B (T_base_board), board, metrics, and the raw yaml."""
    with open(path) as f:
        d = yaml.safe_load(f)
    out = {"X": _pose_from_dict(d), "B": _pose_from_dict(d["board_in_base"]), "raw": d,
           "board": Board(d["board"]["cols"], d["board"]["rows"], d["board"]["square_size"])}
    return out
