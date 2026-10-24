"""Pure helpers of the frame capture node (no ROS): file names and the sidecar record that goes with each saved image.

The sidecar makes a saved frame usable offline for more than image tests: with the camera pose in the base frame, the camera model and the ground
marker's pose, a detection's pixel can still be turned into a floor-plane position (object_logic.object_position) without the robot.
"""
import time


def frame_name(prefix, stamp, index):
    """File stem 'prefix_YYYYmmdd_HHMMSS_mmm_iii' from a ROS time in seconds and a running index (distinct even within one millisecond)."""
    t = time.localtime(stamp)
    return "%s_%s_%03d_%03d" % (prefix, time.strftime("%Y%m%d_%H%M%S", t), int(round((stamp - int(stamp)) * 1000)) % 1000, index)


def pose_record(translation, quaternion_xyzw):
    """A pose as plain lists, or None when it is unknown."""
    if translation is None or quaternion_xyzw is None:
        return None
    return {"position": [float(v) for v in translation], "quaternion_xyzw": [float(v) for v in quaternion_xyzw]}


def sidecar(stamp, frame_id, width, height, K, D, camera_pose, marker_pose, joints=None, note=""):
    """Dictionary written next to a frame as JSON. camera_pose / marker_pose: pose_record() in the base frame (or None); joints: {name: position}."""
    return {"stamp": float(stamp), "frame_id": frame_id, "width": int(width), "height": int(height),
            "K": [float(v) for v in K], "D": [float(v) for v in D],
            "camera_pose_base": camera_pose, "marker_0_pose_base": marker_pose,
            "joints": {k: float(v) for k, v in (joints or {}).items()}, "note": note}
