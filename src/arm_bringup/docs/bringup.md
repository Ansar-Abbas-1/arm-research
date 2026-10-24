# Bringup and mission control

## 1. Launch sets

One file per functional set; each includes the one below it and repeats all its arguments (same name, same default), so the complete stack takes every parameter. `test/test_bringup.py` checks that no argument is forgotten.

| File | Adds | Nodes |
|---|---|---|
| `bringup_arm` | driver or simulator, planner, executor, reach, presets, gripper | `arm_interface` \| `arm_sim_validator_node`, `robot_state_publisher`, `trajectory_*`, `reach_target_pose_node`, `interactive_pose_marker_node`, `preset_manager`, `gripper_control`, rviz |
| `bringup_vision` | camera, camera TF, marker localizer | `usb_cam`, `camera_static_tf` (or `hand_eye_tf_publisher`), `marker_localizer` |
| `bringup_detect` | detector, grasp target | `object_detector`, `grasp_target` |
| `bringup_grasp` (complete) | grasp sequencer, mission control | `grasp_sequencer`, `mission_control` (+ `mission_control_ui` for rqt) |

```
roslaunch arm_bringup bringup_grasp.launch use_hardware:=true device:=/dev/video3      # complete stack, mission control in rviz
roslaunch arm_bringup bringup_grasp.launch mission_control:=rqt                        # same panel as an rqt window
roslaunch arm_bringup bringup_vision.launch use_hardware:=true                         # arm + camera + markers only
```

`use_hardware` defaults to **false**: a bringup never opens the serial port unless asked. Stopping the driver disables the torque: stop the stack only at the default pose.

## 2. Parameters

Precedence, lowest first: package yaml (`arm_perception/config/objects.yaml`, `arm_grasp/config/grasp.yaml`) < deployment profile (`config/detect.yaml`, loaded after the package yaml) < launch arguments. Defaults are the operator-confirmed working settings of 2026-10-14 (FLAGGED values stay flagged in the package yaml):

| Argument | Default | Meaning |
|---|---|---|
| `approach` | `45` | `down` \| `forward` \| degrees below horizontal; runtime `/grasp_target/approach` |
| `bias_r/_t/_z` | `-0.010 / 0.010 / 0` | aim bias in metres, arm frame at the target's azimuth; runtime `/grasp_target/bias` |
| `plant_class`, `grasp_height` | `Vigna`, `0.09` | pinch point above the floor; runtime `/object_detector/grasp_heights` |
| `load_target`, `contact_load` | `15`, `5` | gripper hold load, % (also the panel slider) |
| `model`, `device_ep` | profile: `vignax_small_v45A.onnx`, `auto` | override the profile when not empty |
| `camera_tf`, `x y z yaw pitch roll` | `static`, `-0.025 0 -0.06 1.5708 0 0` | hand-measured camera mount (see M6) |
| `device` | `/dev/video2` | camera node; renumbers after a replug |
| `rviz_config`, `use_rviz` | per set | `arm.rviz`, `vision.rviz`, `detect.rviz`, `mission.rviz` |

Mission control's own parameters: `arm_mission/config/mission_control.yaml` (`mission_config`), `plan_valid_s` (120 s) and `snapshot_valid_s` (60 s, FLAGGED).

## 3. Restart of crashed nodes

`respawn:=true` (default in the bringup files; `false` in the single launch files) restarts a node 2 s (`respawn_delay`) after it dies. Never respawned: rviz, `mission_control_ui`, `detector_view` (closing them is deliberate), and the hardware **driver** (`respawn_driver`, default false): a driver that dies on a motor error would re-enable the torque again and again, and a restarted driver re-seeds its goals from the present pose. `camera.launch` always respawns (the webcam drops off USB).
A restarted `grasp_sequencer` comes up IDLE (a run in progress is aborted; the driver holds the last goal), a restarted `gripper_control` stops regulating the load (the jaws stay where they were commanded), a restarted `grasp_target` stays silent until marker_0 is seen again.

## 4. Mission control

`mission_control:=rviz|rqt|none` of `bringup_grasp`:

- `rviz`: the **Mission Control** panel (`arm_gui/MissionPanel`) in rviz, config `mission.rviz`; needs `use_rviz:=true`.
- `rqt`: the same panel as an rqt plugin (`arm_mission`, also `rosrun arm_mission arm_mission_rqt`, or Plugins > Arm > Mission Control); rviz keeps `detect.rviz`.
- `none`: no interface and no backend.

Both front ends are thin clients of the backend node `mission_control` (`arm_mission/scripts/mission_control_node.py`):

- `/mission_control/state` (JSON, latched, 2 Hz): plants, selection, settings, dry-run result, sequencer state, go gate.
- `/mission_control/command` (`arm_gui/MissionCommand`): `refresh`, `select`, `set_approach`, `set_bias`, `set_height`, `dry_run`, `go`, `stop`.

Workflow: arm at the viewing pose, marker_0 seen. **Refresh plants** snapshots the Vigna in view, ranked by distance from the tool axis on the plant plane, selects the nearest and writes marker_0 + offset to `/grasp_target/offset`. Pick another plant in the list if wanted. **Dry run** plans the sequence (nothing moves). **GO** (confirmation dialog) is enabled only after a successful dry run of exactly the current plant, approach, bias and height, not older than `plan_valid_s`, with no e-stop and no run in progress; one dry run releases one go. Changing any setting, the plant or the height invalidates the plan; a height change also makes the plant list stale (the plant plane moves with it): refresh again. While a grasp runs only **STOP** is accepted. The e-stop button stays in the Arm Control panel. The hold-load slider is also there.

Not covered by the panel: the probe (sag measurement in free air), the gripper open/close (Arm Control panel), and the grasp parameters of `grasp.yaml`.
