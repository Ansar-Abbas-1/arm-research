# Workspace identification — position-only reachable envelope

Characterizes the set of `end_effector` positions the 5-DOF chain can
reach, given its actual joint limits (`arm_interface/urdf/arm.urdf`) —
flagged as a known gap in [`inverse_kinematics.md`](inverse_kinematics.md)
/ the roadmap's M2 notes ("the position-only-reachable envelope ... never
separately characterized/mapped") after the `end_effector` offset fix
shifted it slightly from `link_5`'s. **Position only** — orientation
reachability is out of scope here (would be a 6D characterization, much
more expensive, and wasn't what was asked for).

Following this repo's own docs convention (topic-based derivation notes,
as in `forward_kinematics.md`/`inverse_kinematics.md`/`arm_teleop`'s
`teleop.md`) rather than the generic 5-section report template — a
deliberate, consistent deviation across every prior milestone doc here.

## Method: Monte Carlo over joint space, not a grid

`ForwardKinematicsSolver::ComputePose` is already exact for any given
joint configuration `q` (M1). The only approximation here is *which*
configurations get sampled. A uniform grid over the 5 actuated joints is
combinatorially infeasible: even a coarse 50 points/joint is `50^5` ≈
3.1×10⁸ combinations. Monte Carlo — draw `q` uniformly at random within
each joint's `[lower, upper]` limit (`GetActuatedJointLimits`, all five
finite for this arm, confirmed) and evaluate FK — costs exactly
`num_samples` FK calls regardless of DOF count, at the price of the usual
Monte Carlo tradeoff: coverage is statistical, not guaranteed, and dense
regions get resampled while thin ones may be missed.

`workspace_sampler.h/.cpp` (`arm_kinematics`, no new package — this is a
direct extension of M1's FK solver, no new external dependency):
- `SampleReachableWorkspace(solver, num_samples, voxel_size_m, rng_seed)` →
  the sampled point cloud plus `min_reach`/`max_reach` (over `‖p‖`),
  bounding box, and a volume estimate.
- `EstimateVolumeByVoxelCounting(points, voxel_size_m)` — bins the cloud
  into a uniform voxel grid and reports `occupied_voxel_count × voxel
  volume`. Split out as its own pure, RNG-free function specifically so
  it's independently testable against hand-picked points, not just as an
  opaque side effect of random sampling (see Testing below) — this is a
  standard Monte Carlo volume estimator: a voxel counts as "occupied" if
  *any* sample landed in it, which is not the same claim as "every point
  in that voxel is reachable."

`num_samples` and `voxel_size_m` are both heuristic accuracy/cost knobs
with no principled default — see Results below for how much this matters
in practice (it turned out to matter a lot more than expected).

## Implementation: node + launch + rviz

`workspace_mapper_node`: one-shot at startup (the workspace is a static
property of the URDF's joint limits, nothing to recompute on a timer),
samples once, logs the stats, and publishes the cloud as a **latched**
`sensor_msgs/PointCloud2` on `/reachable_workspace_points` — a subscriber
(rviz) opened after the (potentially tens-of-seconds-long) sampling pass
still sees it immediately, same latched-topic pattern `arm_teleop` used
for `/teleop_mode`. `~num_samples` (default 200000), `~voxel_size_m`
(default 0.01), `~rng_seed` (default 42, for reproducibility).

`launch/workspace_visualize.launch`: `robot_description` + a plain
`robot_state_publisher`/`joint_state_publisher_gui` (so the real robot
mesh can be jogged around for visual comparison against the cloud, no
simulator needed since nothing here is closed-loop) + the mapper node +
rviz (`rviz/workspace_visualize.rviz`, a `PointCloud2` display colored by
Z alongside the existing `RobotModel`/`TF` displays, same base as
`fk_visualize.rviz`). `use_rviz:=false` for headless runs.

```bash
roslaunch arm_kinematics workspace_visualize.launch
```

## Results

All measured live via `roslaunch ... use_rviz:=false` (headless container,
so RNG/compute results are real; only the rviz *display* is skipped).

**Reach and bounding box are stable and consistent across every sample
budget tested** (200k, 300k, 2M):

| Quantity | Value |
|---|---|
| Max reach `‖p‖` (sampled) | 1.188–1.189 m |
| Chain geometric upper bound (independent, see below) | 1.281 m |
| Min reach `‖p‖` (sampled) | ~0.0002–0.0005 m (arm can fold back almost onto its own base) |
| Bounding box X | [-1.02, 1.08] m |
| Bounding box Y | [-1.09, 1.08] m |
| Bounding box Z | [-0.72, 1.19] m |

The upper bound is an *independent* cross-check, computed by walking
`arm_interface/urdf/arm.urdf` directly (not via `ForwardKinematicsSolver`)
and summing each joint's origin-translation norm root→tip:
`0.03+0.065+0.34+0.029+0.376+0.336+0.105 = 1.281 m` — the triangle
inequality on a chain of rigid segments, true for *any* configuration
regardless of joint limits. Sampled max reach (1.189 m) sits comfortably
under it (93% of the theoretical max), consistent with joint limits
preventing every segment from aligning perfectly straight at once.

**Volume did not converge at any tested budget — this is the real
finding, not a clean result:**

| `num_samples` | `voxel_size_m` | occupied voxels | estimated volume |
|---|---|---|---|
| 200,000 | 0.01 | 177,804 | 0.178 m³ |
| 300,000 | 0.01 | 256,188 | 0.256 m³ |
| 2,000,000 | 0.01 | 1,170,955 | 1.171 m³ |
| 300,000 | 0.03 | 97,258 | 2.626 m³ |
| 1,000,000 | 0.03 | 143,170 | 3.866 m³ |

Occupied-voxel count keeps growing roughly in proportion to `num_samples`
at *every* budget tested, up to 2M samples (~142s at 1cm voxels) — no
sign of saturating. Two things are happening: (1) the reachable-position
set, seen from 5D joint space, is filled highly non-uniformly (regions
near the workspace boundary/shell are hit by a much smaller fraction of
random joint draws than the well-covered interior), so fully covering the
boundary needs far more samples than covering the bulk; (2) a coarser
voxel (3cm) reports a *larger* volume than a finer one (1cm) at a
comparable sample count (2.6 m³ vs 0.26 m³ at 300k) — coarse voxels pad
thin/boundary regions with dead space that isn't actually reachable,
while fine voxels underestimate because sampling hasn't filled every truly
-reachable cell yet. The true volume lies somewhere these bracket, and
neither setting tested here has converged close enough to quote a single
trustworthy number.

**Reach and bounding box did NOT have this problem** — extremal values
converge fast (a handful of samples near a limit combination is enough to
find approximately the true extreme), unlike volume, which needs the
*entire* set filled in.

## Known limitation (not solved here, same treatment as M3 Finding 4)

The volume estimate should be read as **order-of-magnitude only, likely an
underestimate at the default settings** (0.178–0.256 m³ at the shipped
200k/300k-sample, 1cm-voxel default), not a converged figure — flagged
rather than silently shipped as if it were precise. Reach bounds and the
bounding box (the table above) are the trustworthy, converged outputs of
this analysis. If a tighter volume number is ever needed: either accept
the multi-minute compute cost of pushing `num_samples` far higher at a
fixed voxel size until successive runs agree, or replace Monte Carlo
occupied-voxel counting with a method whose convergence can be bounded
analytically (e.g. rejection sampling against a reachability oracle with
known error bounds) — out of scope for what was asked here.

## Testing

- `test_workspace_sampler.cpp` (gtest): `EstimateVolumeByVoxelCounting`
  against hand-picked points with exactly known expected voxel counts
  (single point, same-voxel pair, different-voxel points, a negative-
  coordinate boundary-straddling case) — this piece has an actual ground
  truth, unlike the sampling-based volume above, so it's checked exactly,
  not just for plausibility. `SampleReachableWorkspace`: exact sample
  count returned; full reproducibility given the same `rng_seed`
  (needed so the numbers in this doc are regeneratable); different seeds
  actually differ; `min_reach`/`max_reach`/bbox/volume are internally
  consistent with the returned point cloud (recomputed independently in
  the test, not just trusted); and the chain-reach independent geometric
  bound above, enforced as `max_reach <= bound` for every sample —
  catches a real correctness bug (wrong joint, unit error), not just a
  repeat of the sampler's own math. All 12 new cases pass; full
  `arm_kinematics` gtest suite (26 tests across FK/Jacobian/IK/workspace)
  still green.
- No new rostest: nothing here is a control loop or has ROS-I/O-dependent
  behavior beyond what `workspace_mapper_node.cpp`'s thin wrapper does
  (load URDF, sample once, publish) — covered by the live headless runs
  above, and the existing `roslaunch_add_file_check` catches launch-file
  XML errors (caught and fixed one here: a literal `--` inside an XML
  comment, the same recurring gotcha noted elsewhere in this workspace).
