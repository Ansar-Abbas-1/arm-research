# Launch files

One launch file per demo/tool in this package; each is self-contained (own
`robot_description`/`robot_state_publisher`/sim, no shared includes) so any
one of them runs standalone.

## `fk_visualize.launch`

- `robot_state_publisher`, `joint_state_publisher_gui` (sliders publish to
  `/joint_states_out`; the sim reads that and republishes `/joint_states_in`)
- `arm_sim_validator_node` (`arm_interface`) — kinematic sim, `sim_mode`/`tau` args
- `fk_node` — publishes `/computed_end_effector_pose` from `ForwardKinematicsSolver`
- `rviz` (optional, `use_rviz`)

Manual FK sanity check: jog joints via the GUI sliders, compare `fk_node`'s
pose overlay against the URDF's own `end_effector` TF frame in rviz — the
same comparison `test/fk_validation.test` makes numerically.

## `workspace_visualize.launch`

- `robot_state_publisher`, `joint_state_publisher_gui`
- `workspace_mapper_node` — one-shot Monte Carlo sample of the reachable
  workspace (`workspace_sampler.h`), publishes a latched `PointCloud2`
- `rviz` (optional)

Visualizes the static reachable-workspace point cloud against the robot
mesh. No live tracking: the workspace is a fixed property of the URDF's
joint limits alone, nothing to drive.

## `resolved_rate_demo.launch`

- `robot_state_publisher`
- `arm_sim_validator_node` (`sim_mode:=ideal`)
- `resolved_rate_demo_node` — moves to a below-base, downward-pointing start
  pose (max contrast for the null-space bias), then streams a continuous
  oscillating task-space jog (`vx`/`vy`/`vz`/`pitch_rate`) through
  `ComputeResolvedRateVelocity`, null-space bias active throughout
  (`nullspace_gain`, `nullspace_tip_link`)
- `rviz` (optional)

Visual demo of `resolved_rate.md`'s null-space secondary task: watch
`nullspace_tip_link` (default `link_4`) straighten toward vertical while the
arm jogs. The bias is gated to zero with no primary command by design, so a
real jog has to be running to see any effect.

## `position_pitch_ik.launch`

- `robot_state_publisher`
- `arm_sim_validator_node` (`sim_mode:=ideal`)
- `position_pitch_ik_node` — loads every `PositionPitchIkOptions` field from
  `config/position_pitch_ik.yaml` (`nullspace_gain` also overridable as a
  launch arg); advertises `/solve_position_pitch_ik`, subscribes
  `/joint_states_in` for the default seed when a request's
  `seed_joint_positions` is empty
- No rviz — service-only, exercised via `rosservice call`, not a visual demo

Stands up `PositionPitchIkSolver` (`position_pitch_ik.md`) as a
reconfigurable ROS service, including its optional null-space secondary
task, rather than only being usable from C++.

## Not covered here

Plain `ik_node` (full 6D pose IK) has no launch file of its own in this
package — it's launched from `arm_trajectory`'s `interactive_reach.launch`
and exercised by `test/ik_validation.test`.
