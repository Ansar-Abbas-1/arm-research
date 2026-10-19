# Arm sag: measurement, model and compensation (2026-10-14; final real-grasp experiments 2026-10-16)

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

**Final experiments (2026-10-16, real grasps of a plant, not free air).** 15 detector-driven grasps with the sag model in the loop, started from Mission Control
(plant chosen by the detector, one dry run per grasp, then GO) from the *left* viewing pose (`viewing-left`, joint 1 +90 degrees):
- Same 45 degree approach; targets at azimuth 73 to 110 degrees (left of the arm, outside the -40 to +40 degrees of the fit), target radius 0.527 to 0.663 m.
- Pinch height 9 cm above the floor for the first 12 grasps, 11 cm for the last 3 (target z -0.556 and -0.535 m, floor about -0.646 m); aim bias
  -10 / +10 mm radial / tangential (-5 / +10 mm for the last grasp); hold load 15 %; correction steps 2, gain 0.7, **trigger 10 mm** (raised from 6 mm after the
  corrections below 10 mm only chased noise).
- After the lift the arm returned to `viewing-left` with the plant still gripped (new RETURNING leg of the behavior tree).
- Logged as before (first measurement of every leg plus every correction step): 57 rows in `recordings/sag/sag_20261016.jsonl`. 12 rows of simulated runs that had
  landed in the same directory were moved to `recordings/sag/sim/` and are not used.

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

### 4.1 Final experiments: real grasps at the left side (2026-10-16)
Residual to the waypoint at the first measurement of a leg (before any correction), arm frame at the waypoint's azimuth, mm:

| group | leg | n | radial | tangential | vertical | norm mean (max) |
|---|---|---|---|---|---|---|
| model on (target radius 0.527 to 0.586 m, azimuth 73 to 103) | PRE | 12 | -7.6 | +0.5 | -0.9 | 11.5 (24.0) |
| model on | GRASP | 12 | -1.6 | +0.3 | +1.0 | 7.7 (16.4) |
| no feed-forward (target radius 0.619 to 0.663 m, azimuth 109 to 110) | PRE | 3 | -6.2 | -16.3 | -6.7 | 20.0 (30.3) |
| no feed-forward | GRASP | 3 | -18.6 | +0.9 | -0.1 | 20.0 (23.2) |

- The three furthest targets are outside the model's radius range (0.46 to 0.55 m, +-5 cm), so it applied no offset to them: that is the "no feed-forward" group.
- **GRASP, the leg that matters, is as good as in the free-air fit at the new azimuth**: 7.7 mm mean with the model against 6.8 to 7.7 mm at -40, 0 and +40 degrees with the
  deployed tangent gain, 20.0 mm without it. The model transfers from -40..+40 to about 90 degrees and to a gripped-plant task (the PRE and GRASP legs do not carry the plant).
- PRE, target radius up to 0.55 m: radial residual within +-1.5 mm in 6 of 7 grasps (one -10.6 mm). Above 0.55 m the fixed radial offset (-30 mm) over-compensates:
  PRE radial -5, -23, -14, -18 and -19 mm at 0.563, 0.575, 0.582, 0.584 and 0.586 m. Without the offset (0.619, 0.622 m) the radial sag is about 0, and -19 mm at 0.663 m. Read
  as the radial sag of PRE falling from +30 mm (r <= 0.55 m) to about +11 mm (0.585 m), 0 (0.62 m) and -19 mm (0.663 m): a steeper slope (about -300 to -540 mm/m) than the
  -141 mm/m measured inside 0.46 to 0.55 m, i.e. not linear. Indicative only: 15 grasps, mixed azimuths and heights.
- Tangential residual at PRE from -14 to +13 mm without a clear dependence on the azimuth (+13 and +12 mm in the two grasps at 89 degrees, about -7 mm at 99 to 103): the tangential part stays the weakest;
  the guess `sag_tangent_gain` 0.5 is neither confirmed nor rejected.
- Correction loop (2 steps, gain 0.7, trigger 10 mm): PRE needed no correction in 5 of 15 grasps (residual below 10 mm). Of the 15 corrected legs 9 ended better than they started:
  GRASP in 4 of 5 (16 -> 10, 16 -> 11, 16 -> 10, 20 -> 7 mm; one 23 -> 24), PRE in 5 of 10 (13 -> 6, 13 -> 4, 24 -> 16, 20 -> 15, 30 -> 22; the others 13 -> 16, 15 -> 22, 18 -> 20, 15 -> 18, 15 -> 20).
  The PRE corrections often added error because the gravity sag at PRE changes with the pose of the correcting move.
- Outcomes: 13 of 15 grasps uprooted a plant (`object`, load 14 to 15 %) and returned to the viewing pose. 2 ended ABORTED by "e-stop engaged or its state unknown", one during CLOSING (r 0.663 m,
  azimuth 110, GRASP error 24 mm after 3 corrections) and one during LIFTING (r 0.622 m); both were targets outside the model's range. The log cannot say whether the e-stop was pressed
  or reported by the driver. Whether a grasp pulled the plant out is the operator's observation; the sequencer's `object` outcome only means the load target was reached.

## 5. Conclusions and limitations
- The feed-forward cuts the raw error at grasp heights about three times (23.6 -> 8.0 mm; 5.3 mm with the full tangential part where it is validated).
- Confirmed on 15 real grasps at azimuth 73 to 110 degrees (section 4.1): GRASP 7.7 mm with the model against 20.0 mm without, 13 of 15 grasps uprooted a plant. The LIFT leg, which
  carries the plant, is not covered by the first-measurement statistics above (the real grasps measure PRE and GRASP only).
- Limits: only the 45 degree approach, grasp heights 5 to 11 cm, azimuths -40 to +40 (free air) and 73 to 110 degrees (real grasps). The radius range 0.46 to 0.55 m is too narrow for the
  left side: targets at 0.575 to 0.586 m got an over-compensating PRE offset, and those at 0.62 to 0.66 m none, and both aborted grasps were in the latter group.
  Joint 2 reached 62 degrees C during the free-air tests; temperature dependence was not examined.
- Next: (1) refit with the 57 real rows (`sag_fit.py`; check the buckets at 45 degrees and extend the radius validity to about 0.67 m, or fit the radius dependence of PRE radial, which the
  new data show is steeper above 0.55 m); (2) measure the azimuths between -40 and 73 degrees (-20, 0 ... +40 are done, 40 to 73 is not) and model the tangential part against the azimuth, then
  decide `sag_tangent_gain`; (3) log the LIFT leg of real grasps (the correction loop does not measure it) to test the vertical sag with a plant in the gripper;
  (4) compare the PRE corrections with and without them: they improved only half of the legs.
- Tools: `scripts/sag_test.py` (the test), `sag_fit.py`, `sag_report.py` (`--since 20261016_000000` gives the table of 4.1), `sag_cv.py`; 21 model tests.

## Simulated runs are kept out of the log (2026-10-16)

The sequencer logs every measurement to `sag_log_dir` (`recordings/sag`), and `sag_fit.py` / `sag_report.py` / `sag_cv.py` read `sag_*.jsonl` there. Simulated grasps measure no sag, so their rows (error 0 at the 90 degree approach) would add a fake zero-sag bucket to a refit. 12 such rows from the e2e test (2026-10-15 and -16) were moved to `recordings/sag/sim/` (not read by the tools). `grasp.launch` has a `sag_log_dir` argument (empty = yaml); `e2e_sim_grasp.py` logs to the temp directory and `bringup_grasp.launch` with `use_hardware:=false` to `recordings/sag/sim`.

## Motor integral gain in free air (2026-10-19)

Question: can an integral term in the motors' position loop remove the sag instead of the feed-forward model? Test: `scripts/gain_sweep.py` writes the Position I gain (register 82, RAM; baseline P 850, I 0, D 0 on all motors) of joints 1 to 5 through the new driver service `/arm_interface/apply_position_gains`, then runs the free-air probe (PRE, GRASP, LIFT, no correction, sag model off) at azimuth 0, 45 degree approach, radii 0.46 and 0.52 m, heights 7 and 9 cm (4 points per value, no camera: fixed target frame, floor z -0.596 m). Raw files: `recordings/gain_test/`.

| I gain | PRE norm (radial, vertical) | GRASP norm (vertical) | LIFT norm (vertical) | mean of the three |
|---|---|---|---|---|
| 0 | 27.9 (27.4, +2.5) | 7.3 (-4.3) | 20.2 (-18.1) | 18.5 |
| 2 | 31.4 (30.1, +7.3) | 5.3 (-0.1) | 15.7 (-14.1) | 17.5 |
| 4 | 33.4 (30.5, +12.8) | 8.3 (+6.8) | 11.8 (-10.2) | 17.8 |
| 8 | 37.5 (33.4, +16.8) | 9.9 (+9.1) | 9.9 (-6.5) | 19.1 |
| 12 | 39.1 (34.5, +18.1) | 9.3 (+8.6) | 9.3 (-7.1) | 19.2 |
| 0 (repeat, end) | 29.4 (29.2, +2.9) | 6.9 (-4.7) | 20.2 (-18.7) | 18.8 |

(mm, residual to the waypoint, mean of 4 points; first run, I = 16 on joints 1 to 5 already made joint 2 oscillate: position std 0.33 deg against the 0.30 deg limit, joint 3 0.20 deg, joint 2 load 52 %. No other value tripped the checks; the hottest motor rose from 43 to 48 C during the run.)

- The integral term does lift the vertical droop of LIFT (-18 -> -10 -> -7 mm) but it shifts PRE and GRASP **upward** (+3 -> +18 mm at PRE, -4 -> +9 mm at GRASP), and PRE gets worse in total (radial sag grows too). The mean error over the three legs stays at 17.5 to 19.2 mm against 18.5 mm without it: no net gain. Only GRASP at I = 2 is better (5.3 against 7.3 and 6.9 mm, n = 4 each).
- Likely cause (a hypothesis, not tested): the probe measures 0.8 s after each move, long before the integrator has converged, and it carries the state of the previous pose (the viewing pose and PRE need different torques than LIFT), so the effect depends on the order and timing of the legs, not on the pose alone. A real steady-state test needs a settle of several seconds per leg.
- The baseline repeat agrees with the first one (LIFT 20.2 / 20.2, GRASP 7.3 / 6.9, PRE 27.9 / 29.4 mm), so the differences above are not drift.
- Conclusion: an integral gain in the motors is not a replacement for the feed-forward model (8.0 mm mean on the validated range). Limits: one azimuth, 4 points per value, uniform I on joints 1 to 5, no per-joint split (the loaded MX-106 joints 2 and 3 oscillate first), settle time of 0.8 s.
