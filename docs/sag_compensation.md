# Arm sag: measurement, model and compensation (2026-10-14)

## 1. Experimental setup
- **Arm:** the 5-DOF Dynamixel arm with the eye-in-hand camera and the gripper; the floor is the plane of `marker_0` (z about -0.605 m in `base_link`).
- **Free-air probe:** the grasp sequencer in `probe` mode (`/grasp_sequencer/grasp` with `probe: true`) moves to PRE, GRASP and LIFT, waits 0.8 s, and measures the
  end effector (TF from the measured joints). It makes no corrective move and never commands the gripper, so nothing is touched. Every first measurement of a leg is
  appended to `recordings/sag/sag_YYYYMMDD.jsonl` (JSON lines; real grasps log the same way).
- **Quantity:** the sag `s = measured - aim`, where `aim` is the position the arm was commanded to (waypoint plus any feed-forward), so samples taken with and
  without the model agree.
- **Approach:** 45 degrees below the horizontal from the robot's side, jaws closing sideways; PRE 10 cm back along that line (joint-space move from the viewing
  pose), GRASP at the target point, LIFT 15 cm straight up.
- **Points:** target radius 0.46 to 0.55 m, grasp height 5 to 9 cm above the floor, azimuths -40, 0 and +40 degrees (the areas cleared by the operator). A
  boundary check at 25 to 30 cm was run at azimuth 0.
- **Runs:** 45 probes (one aborted at PRE: a corrective aim was unreachable, which is why probes now only measure), 133 first measurements, 109 at grasp heights.
  Order: trial at -40 degrees (12 probes, no model) -> fit -> validation with the model on (-40: 8 probes at held-out radii 0.49 / 0.55 m and heights 6 / 8 cm; +40: 4;
  0: 12 low and 8 high) -> refit.

## 2. Model (`src/arm_grasp/sag_model.py`, `config/sag_model.json`)
- The mean sag per approach-angle bucket (15 degrees wide; only 45 degrees is measured) and leg, in the arm's own frame: radial (outward), tangential (along the joint 1
  travel, signed by its direction with a 0.05 rad ramp: a heuristic) and vertical.
- Feed-forward `ff = -gain * (s_r u + g_t s_t w + s_z z)`, each leg limited to 8 cm; LIFT uses its own samples. Valid only within the measured radius (0.46 to 0.55 m, +-5 cm) and
  height (+-4 cm): outside it the offset is zero.
- Fitted values at grasp heights (n = 37 / 36 / 36), mm: PRE radial +29.9, tangential -12.0, vertical +1.8; GRASP +0.5, -13.1, -4.5; LIFT -0.9, -13.1, -22.6.

## 3. Control method
- The plan aims at `waypoint + ff`; the correction loop's running compensation starts at PRE's offset and later legs add their difference; the 2 correction steps (gain 0.7)
  only trim the residual. If the full offset makes the plan unreachable (inner edge of the workspace) it is backed off to 75, 50, 25 and 0 %.
- `sag_gain` (1.0) scales all, `sag_tangent_gain` (0.5, a flagged guess) only the tangential part, `sag_model: ""` turns it off. Both are in `config/grasp.yaml`.

## 4. Results (grasp heights only, residual to the waypoint, mm, mean (max))
| azimuth | leg | n | no model | model, tangent gain 1 | deployed, tangent gain 0.5 |
|---|---|---|---|---|---|
| -40 | PRE | 21 | 31.3 (37.9) | 4.8 (8.2) | 7.8 (10.0) |
| -40 | GRASP | 20 | 13.9 (17.1) | 2.1 (4.2) | 6.8 (10.0) |
| -40 | LIFT | 20 | 26.9 (34.9) | 6.1 (12.7) | 9.2 (14.5) |
| 0 | PRE | 12 | 32.3 (39.4) | 6.8 (9.9) | 7.8 (10.5) |
| 0 | GRASP | 12 | 9.8 (11.2) | 6.1 (7.6) | 7.4 (8.7) |
| 0 | LIFT | 12 | 24.4 (29.2) | 7.9 (10.5) | 8.9 (11.3) |
| +40 | PRE | 4 | 33.7 (38.6) | 5.9 (7.2) | 8.6 (9.0) |
| +40 | GRASP | 4 | 14.6 (16.8) | 3.3 (6.2) | 7.7 (10.0) |
| +40 | LIFT | 4 | 26.3 (30.9) | 6.5 (7.3) | 9.5 (10.0) |
| all | | 109 | 23.6 | 5.3 | 8.0 |

- Held-out validation at -40 degrees (radii 0.49 / 0.55 m, heights 6 / 8 cm, model on, live): PRE 4.7, GRASP 3.1, LIFT 8.5 mm.
- Cross-validation: refitting the same structure gains nothing (5.39 mm for the current model against 5.38 mm for an out-of-group refit); the model is at its structural limit.
- The sag is mostly radial at PRE (+30 mm: the arm lands further out), vertical at LIFT (-22 mm), small at GRASP (-4.5 mm vertical).
- The tangential part (joint 1) is **not** a simple sign of the travel: along +azimuth it is +12 mm at -40, **+8 mm at 0** (no travel), -12 mm at +40. At 0 the model leaves about 8 mm.
- Radius dependence (grasp heights): PRE radial slope -141 mm/m (spread 5.1 mm -> 2.2 mm if linear), LIFT vertical -125 mm/m (4.3 -> 1.3 mm), GRASP vertical -25 mm/m.
- At 25 to 30 cm the sag is different (PRE radial +9 mm): the model would make PRE worse (23.8 against 16.5 mm), hence the validity range.

## 5. Conclusions and limitations
- The feed-forward cuts the raw error at grasp heights about three times (23.6 -> 8.0 mm; 5.3 mm with the full tangential part where it is validated).
- Not yet tested with real grasps (a gripped plant adds load at LIFT). Only the 45 degree approach, only grasp heights up to 9 cm, azimuths -40, 0, +40, free air.
  Joint 2 reached 62 degrees C during the tests; temperature dependence was not examined.
- Next: (1) measure -20, -10, +10, +20 degrees at grasp heights (needs clear space there) and model the tangential part against the azimuth, then raise `sag_tangent_gain`;
  (2) linear-in-radius terms for PRE radial and LIFT vertical (would roughly halve the spread); (3) let real grasps feed the log and refit.
- Tools: `scripts/sag_test.py` (the test), `sag_fit.py`, `sag_report.py`, `sag_cv.py`; 21 model tests.

## Simulated runs are kept out of the log (2026-10-16)

The sequencer logs every measurement to `sag_log_dir` (`recordings/sag`), and `sag_fit.py` / `sag_report.py` / `sag_cv.py` read `sag_*.jsonl` there. Simulated grasps measure no sag, so their rows (error 0 at the 90 degree approach) would add a fake zero-sag bucket to a refit. 12 such rows from the e2e test (2026-10-15 and -16) were moved to `recordings/sag/sim/` (not read by the tools). `grasp.launch` has a `sag_log_dir` argument (empty = yaml); `e2e_sim_grasp.py` logs to the temp directory and `bringup_grasp.launch` with `use_hardware:=false` to `recordings/sag/sim`.
