# rviz "Arm Control" panel: preset poses, e-stop, torque, motors

A normal 2D panel docked in rviz (package `arm_gui`, class `arm_gui/ArmControlPanel`, a C++/Qt rviz plugin). It is in
`arm_trajectory/rviz/interactive_reach.rviz`, so `interactive_reach.launch` and `arm_camera/arm_with_camera.launch` show it.
In another rviz config: Panels > Add New Panel > `arm_gui/ArmControlPanel`.

| Control | What it does |
|---|---|
| Status bar | e-stop state, drive state and worst motor health, e.g. `e-stop: ENGAGED | drive: ESTOPPED | motors: ERROR | joint_4: overload`. Red when the e-stop is engaged or a motor is in error, amber on a warning, green otherwise. An unknown e-stop state is shown as engaged (senders treat it that way). |
| **E-STOP** (big red) | Engages the e-stop at once, **no confirmation**: both the `/arm_interface/estop` service and the `/arm_interface/estop_trigger` topic. The arm HOLDS its pose with torque on. |
| Reset e-stop... | Confirms, then `/arm_interface/estop_reset`. The driver's refusal reason is shown (a motor with a latched hardware error must be reset first). Releasing only lifts the latch: the arm keeps holding until the next command. |
| Preset buttons | One per preset (`arm_trajectory/config/presets.yaml`, plus the poses you saved). Confirms with the largest joint move, then `go`. Enabled only with the e-stop released and torque on. The running one is highlighted. Tooltip: note and joints. |
| Save current pose as... | Asks for a name, stores the arm's current joints in `~/.ros/arm_trajectory/presets.yaml` (overrides shipped poses by name; asks before overwriting). |
| Cancel move | Cancels the running preset move. |
| Torque: ON - release... / OFF - enable... | Confirms, then `/arm_interface/set_torque`. **Releasing torque lets the arm sag** (the dialog says so); the driver refuses while the e-stop is engaged. A command while torque is off re-enables it. |
| Motor report | Temperature / the motor's own limit, load, voltage and status per motor, coloured green, yellow or red. The box title shows `update #N, x.x s ago`; with no data for 3 s it reads `NO DATA`, the report greys out and the status bar says `motors: n/a`, so a frozen report cannot pass for live values. |
| **Reset failed motors (reboot)...** | For when a motor stopped with a hardware error (overheat, overload, ...) and **you judge it safe to re-engage it**. Enabled whenever a motor is in ERROR, also under e-stop. Confirms (the dialog names the failing motor and warns that the arm may have sagged), then `/arm_interface/reenable_failed_motors`: every motor with a latched error is rebooted, its goal is seeded to the pose it has now, and torque is enabled. A motor that fails again stops again; the error remains shown. Only latched hardware errors are reset: a motor that is merely hot is not touched. If the e-stop is also engaged, reset the motors first, then the e-stop. |

Buttons are disabled when what they need is missing (no driver, no preset backend, e-stop engaged, move running); the
bar says when the driver is not found. The last action result is shown at the bottom and logged.

## Structure

* `arm_gui` `panel_logic.h`: Qt/ROS-free rules (health roll-up, status text, motor rows, which buttons are enabled).
* `arm_gui` `ArmControlPanel`: widgets; ROS callbacks are forwarded to the GUI thread. Talks to the driver directly and to the
  preset backend through `/preset_manager/command` (`arm_gui/PresetCommand`: `go`, `save`, `cancel`) and the latched JSON
  `/preset_manager/state`.
* `arm_trajectory/scripts/preset_manager_node.py` (started by `interactive_reach.launch`; `presets:=false` leaves it out, the
  panel then has no preset buttons): holds the poses and runs the moves through exactly the path of `stage3_joint_move.py`
  (`/generate_trajectory`, then the `follow_joint_trajectory` action, whose executor refuses to move while the e-stop is
  engaged). Saved poses are checked against the URDF limits and one move runs at a time. An optional size guard
  (`preset_max_delta:=3.2` for example, a heuristic against typos in saved poses) refuses larger joint moves; it is off by default (0). Joint 6 (the gripper) is not touched.

## Command line (the same services)

```bash
rosrun arm_interface estop_cli.py status          # e-stop, drive state, motor health summary
rosrun arm_interface estop_cli.py motors          # per-motor table with hardware error codes
rosrun arm_interface estop_cli.py trigger         # engage the e-stop
rosrun arm_interface estop_cli.py reset           # release it (asks first)
rosrun arm_interface estop_cli.py torque off      # release torque (asks first); torque on enables it
rosrun arm_interface estop_cli.py watch           # print every change
```

## Simulator parity

`arm_sim_validator_node` offers the same `set_torque`, `drive_state`, `motor_status` and `reenable_failed_motors` as the hardware
driver (a command re-enables torque, `set_torque` is refused while the e-stop is latched, with torque off the simulated arm
holds still, the reset clears the injected fault). `_sim_motor_warn_joint:=N` and `_sim_motor_fault_joint:=N` inject a
temperature warning or an overload on joint N so a bad motor can be tried without touching the real arm.

## Tests

* `arm_gui/test/test_panel_logic.cpp` (16, gtest) and `test_arm_control_panel.cpp` (15, rostest, headless Qt against fake
  services: confirmations asked and honoured, E-STOP unconfirmed on both paths, enable rules, torque release/enable, motor
  reset, stale motor data, save, cancel). `rosrun arm_gui panel_probe [seconds]` runs a panel headless against the live
  master (it only subscribes) and prints what it shows: tells a panel problem from a display one.
* `arm_trajectory/test/test_preset_logic.py` (11), `arm_interface/test/test_estop_cli.py` (6).
* `arm_trajectory/test/e2e_presets_sim.py`: the services the panel calls against the simulated arm on a private master
  (11312), about 40 checks (go, busy refusal, cancel, save and its name checks, e-stop with refused moves and torque, reset,
  torque flow, motor reset, CLI).

Run in a real rviz against the simulator and on the real arm (presets, e-stop, live motor report); the failed-motor reset only in simulation.

## Limits

* An explicit torque release is overridden by the next command when a sender is streaming; the panel does not stream.
* The torque state is derived from the drive state: the driver has no separate torque topic.
* Presets are joint-space moves: no collision check along the straight joint-space path (as for every joint-space move
  here). The shipped `pre_grasp` is a placeholder (pose A): save your own over it.
* Display quirk found on the real machine (Wayland session, rviz in Docker through XWayland, software GL): a QTableWidget for the motor report did not repaint on updates until the window was resized, and translucent cell colours left a black bar. The report is therefore a rich-text label with opaque colours (`MotorTableHtml`). `-e QT_X11_NO_MITSHM=1` did not change it.
* A panel service call blocks the GUI for the duration of the call (milliseconds when the driver is up).

## The 3D version (kept, not enabled)

The first version drew these controls as interactive markers floating in the 3D scene. It is kept as a quirky alternative and
is not started by any default launch file:

```bash
roslaunch arm_trajectory legacy_preset_markers.launch use_hardware:=false
```

(`scripts/legacy_preset_buttons_node.py`, `rviz/legacy_preset_markers.rviz`, tests in `test/legacy_3d/`: two clicks to move,
red E-STOP, torque and motors buttons.) Use it instead of `interactive_reach.launch`, not next to it.
