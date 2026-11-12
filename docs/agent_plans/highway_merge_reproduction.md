# Highway merge reproduction (SenSys resubmission)

Started 2026-11-09. Tyler: the colleague's export (`~/Downloads/eval_run_v12/eval_run_v12`)
carries a `highway_merge_edge_late_fusion` geometry that no scenario in this sandbox
produces. Recreate the scenario and reproduce part of that data.

## What the export says the geometry is

- `runs.csv`: geom `highway_merge_edge_late_fusion`, speed 97 km/h, arms
  `late_fusion` off/on and `oracle` on, N=1, 24 configured-delay cells
  (0 to 1050 in 50 ms steps, then 1150 and 1250), 50 seeds per cell, 25,200 rows.
  kv carries the design row (`closing_speed_mps`, `initial_gap_m`, ...) plus
  `workload: highway_merge`, `motion: constant_velocity`, `prediction_steps: 60`,
  `behavior.curv_lat_accel: 8`, `self_exclusion_m: 1.0`. Cell names follow the
  test_runner convention with an `_experiment_highway_merge` suffix.
- `highway_merge_design.csv`: ego 27 m/s, closing speeds 4 to 16 m/s, bumper gap at
  the cut-in `3 + 1.9 * cs`, relative stop distance `cs^2 / 12`, kinematic budget
  `1.9 - 0.28 - cs / 12` s. `scripts/sensys_analysis/highway_merge_design.py --check`
  reproduces every value.
- Paper (5.1.5, 5.2.2, Figs. 16 and 17): a slower vehicle cuts into the CAV's lane at
  97 km/h; the swept variable is the closing speed at the cut-in; budgets fall from
  about 1.2 s at 4 m/s to 250 ms at 16 m/s; V2X2V without SBA self-ghosts from the
  earliest ages and never reaches 0.8 at 14 and 16 m/s.

## What was built (all new files, committed with explicit paths)

- `ecav/scenario_testing/scenarios/scenario_highway_merge.py` and `.xml`: Town06
  eastbound carriageway (lane centres y = 135.93 + 0.0041 x for lane -3, 3.5 m per
  lane, probed offline from the xodr). Ego (nissan.patrol) in lane -4 from x = 100 at
  its commanded speed; intruder (tesla.model3, so the per-tick logger's
  conflict-actor rule finds it) in lane -5 at ego speed minus the closing speed,
  placed so the bumper gap is `design gap + cs * PRE_TRIGGER_S` (4 s of cruise).
  `CutIn` behaviour: when the bumper gap reaches the design gap, the intruder's lane
  plan is replaced by a lane change into lane -4 over `CUTIN_DURATION_S * speed`
  metres (2 s). Both vehicles start at speed (`set_target_velocity`). Optional
  occluding truck (`OCCLUDER=1`) behind the intruder in lane -5. Knobs are
  environment variables (test_runner `--scenario-env`), listed in the file header.
- `ecav/scenario_testing/highway_merge_edge_late_fusion.py` (runner, from the blind
  overtake runner; `SCENARIO_MAX_STEP` honoured) and
  `config_yaml/highway_merge_edge_late_fusion.yaml` (from the blind overtake YAML:
  town06, two RSUs on the south shoulder at x 215 and 330, 7 m masts; the junction
  planner block; `per_tick_age_log` with `conflict_xy` [230, 140.4] and
  `cross_traffic_type: tesla`; destination lane -4 at x 600).
- `scripts/sensys_highway_merge.sh smoke|validate|pilot|full`: the closing speed
  travels as `--edge-kv closing_speed_mps=<cs>` (run directory and reducer kv, as in
  the export) and as `--scenario-env CLOSING_SPEED_MPS=<cs>` (what the scenario
  reads). `CS=10` restricts validate/full to one closing speed; `HM_ENV` adds scenario
  knobs.
- `scripts/sensys_analysis/highway_merge_check.py <stamp>`: outcome table per
  (closing speed, arm, delay) with the cut-in tick, gap and speeds parsed from the
  run log, plus the pre-launch gate (Tyler, Oct 30).

## Smoke run (2026-11-09, exact-state, cs 10, 0 ms, stamp sensys_hm_smoke)

Cut-in fired at a 21.7 m bumper gap (design 22.0), ego 25.8 m/s (the planner holds
`max_speed - speed_lim_dist`, 94 km/h), intruder 16.3 m/s. The ego braked to a full
stop (the planner's emergency-braking response), resumed, then stopped again on its
own lidar fragments (0.1 by 0.7 m anonymous tracks at 50 m) after the intruder left
the single RSU's lidar reach near x 350; the held stale track aged to 1.2 s. Outcome
s_op 1, no collision, no ghost, 51 s wall clock per run. Fixes: second RSU at x 330,
`EGO_DRIVE_DISTANCE` default 250 m (the run ends just after the stop), tick counter
primed at construction.

## Gate, closing speed 10 (2026-11-09, stamp sensys_hm_v1, two seeds per cell)

| arm | delay | s_op | collided | self-ghost | p99 age (ms) |
|---|---|---|---|---|---|
| exact state | 0 | 2/2 | 0 | 0 | 168 |
| exact state | 1000 | 2/2 | 0 | 0 | 1137 |
| V2X2V +SBA | 0 | 2/2 | 0 | 0 | 589 |
| V2X2V no SBA | 0 | 0/2 | 0 | 2 | 275 |
| no cooperation | 0 | 0/2 | 2 | 0 | |

Three of four gate conditions pass; the stale probe does not: the exact-state arm still
completes at 1000 ms. Two causes, both fixed before the pilot:

1. Braking authority. The emergency stop uses brake 1.0 and the ego sheds 26 m/s in about
   3 s (8.5 m/s2 average, 20 m/s2 at the onset). The design assumes 6 m/s2, and the
   kinematic budget (787 ms at 10 m/s) only holds at that deceleration. The controller's
   emergency path now honours `max_brake` (`ecav/core/actuation/pid_controller.py`, 1.0
   everywhere else, so no other geometry changes) and the highway YAML sets the value that
   calibrates to about 6 m/s2 (runs at 0.4 and 0.6 in stamps sensys_hm_cal_*).
2. Coverage. The exact-state arm publishes ground truth only within 50 m of a managed
   sensor (`edge_manager_oracle` DETECTION_RANGE), and the camera-lidar arms lose a car at
   a similar distance. With RSUs 15 m south of the lane the along-road reach is about 48 m
   each side, so the intruder's track froze 50 m past the last RSU and the held copy aged to
   1.2 s (the p99 of 950 to 1137 ms above). Three RSUs at x 165, 260 and 355 cover the
   intruder from its start to the end of the run.

The +SBA p99 of 589 ms also includes early stale anonymous duplicates of the intruder
(450 to 600 ms at ticks 27 to 58), which is the late-fusion tracker's own behaviour and
stays.

## Grid sizing (51 s per run on atlas)

| stage | runs | one host |
|---|---|---|
| validate (7 cs, 5 arms/probes, 2 seeds) | 70 | about 1 h |
| pilot (cs 4, 10, 16; 8 delay cells; 3 arms; 10 seeds) | 720 | about 10 h |
| full (export grid, 50 seeds) | 25,200 | about 360 h |

The full grid is not a single-host job. Reproduction target: pilot first, then widen
seeds on the cells the paper quotes (Fig. 16 panels at 4, 10, 16 m/s).

## Open points for Tyler

- Occlusion: the paper says every encounter has the conflict actor occluded from the
  CAV. The export's design row has no occlusion parameter. The scenario has the truck
  knob; the gate runs are without it.
- Emergency stop: the planner stops fully after the cut-in although the intruder pulls
  away. That is the paper's control response and the junction runs behave the same.
- Measured closing speed is about 0.5 m/s below the design (planner speed margin and
  follower undershoot). Reported from the log, not corrected.

## Pilot failed the gate; diagnosis and fixes (2026-11-11 23:24)

Pilot (stamp sensys_hm_pilot, 10 seeds, cs 10 and 16) failed at both closing speeds. Reference
behaviour (eval_run_v12, 50 seeds): the collision rate is the same for the three arms at each
(closing speed, delay) and follows the kinematic design, 50 % point near budget + 50 ms.

Defects in the scenario and planner setup, found from per-tick logs and runner_stdout:

1. Path horizon. buffer_size 16 / waypoint_update_freq 9 gave 40 to 72 m of path, 1.5 to 2.8 s at
   highway speed. The time-synchronised collision check only covers the path, so the conflict was
   seen at the next refill, 0.85 s after the lane change began. Now 28 / 22 (94 m minimum).
2. Brake. max_brake 0.5 is 4.4 m/s2 at 20 to 27 m/s. 0.7 is 5.5. Now 0.8 (design 6).
3. Cruise speed. speed_lim_dist 3 left the ego at 94 km/h. Now 0, ego 27.0 m/s at the cut-in.
4. Path not consumed under the brake hold (behavior_agent "[STOP] hazard without obstacle" branch
   skipped pop_buffer). On release the target was behind the car and the ego steered off the
   carriageway: the collisions and stalls of the V2X2V arms at low delays. Fixed, stalls are 0.
5. Gap reference. The design counts failure at the 3 m clearance, CARLA at bumper contact. The
   cut-in now fires at design gap minus clearance (GAP_REFERENCE=clearance).

Exact-state arm after the fixes (stamp sensys_hm_fix4, 2 seeds per cell):

| cs | completes through | collides from | reference 50 % point |
|---|---|---|---|
| 4 | 1050 ms (1150: 1 of 2) | 1250 ms | about 1200 ms |
| 10 | 700 ms | 800 ms | about 700 to 750 ms |
| 16 | 250 ms | 300 ms | about 225 to 250 ms |

The exact-state arm reproduces the reference within one 50 ms delay cell at the three speeds.

V2X2V arms do not reproduce it (same stamp): +SBA at cs 10 is 2/2, 1/2, 0/2, 1/2 at 0, 100, 200,
400 ms, and 0/2 at cs 16 from zero delay. Two causes, both in the late-fusion tracker's forecast:
- Lateral velocity lags a lane change by about 1 s. The forecast still points straight while the
  intruder is mid-change, so the hazard fires only when the position itself is within the 2.5 m
  proximity radius (first brake about 1.0 s after the cut-in began; exact state: 0.15 s).
- Speed and heading noise on an adjacent-lane car (17 m/s reported as 11 to 23 m/s, lateral drift
  of 1.3 m over the 3 s forecast) raises a hazard before the cut-in, now that the path is long
  enough to see it (proximity radius 2.5 m against 3.5 m lane spacing).
In the reference the camera-lidar arm follows exact state within about 50 ms.

Provenance fact: the reference rows carry kv keys workload=highway_merge and
motion=constant_velocity and a cell suffix _experiment_highway_merge. No file or commit in this
repository contains those keys, so the reference harness for this geometry is not in the repo.

Open (Tyler's call before any large launch): how the camera-lidar arms should be made to follow
exact state (tracker lateral velocity, proximity radius for multi-lane roads, or the cut-in
duration of the reference, which the export does not state).
