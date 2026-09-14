# Closed-loop state-depth block (Figure 4), design 2026-09-13

Tyler's ruling (via the paper side): the state question is answered in the
closed loop; the offline frontier leaves the paper. Vary ONLY the transferred
state at the handoff; everything else identical across arms. Build now; run
only on Tyler's word on the MTR fix shape (the block's fde5 is an MTR forecast,
so it runs under freeze-1q).

## Arms (campaign lineage: frozen base 71c9f37e + code_overlay; nothing new)

| representation | launch | record |
|---|---|---|
| none | MIGRATION_MODE=cold | nothing transferred; destination cold-starts |
| pos_vel | MIGRATION_MODE=kf | depth-1 box + vel_mps (freeze-1k export computes the velocity for every mode, pluggable_base_wired.py:232-259) |
| hist2 | MIGRATION_MODE=warm MIGRATION_HIST=2 | last 2 memo frames (0.4 s at 5 Hz) + velocity |
| hist5 | MIGRATION_MODE=warm MIGRATION_HIST=5 | last 5 frames (1.0 s) + velocity |
| hist10 | MIGRATION_MODE=warm | full record (2.0 s), today's Khonsu arm |
| no_handoff | -t openscenario_1_flow_single with runner_wired.py copied onto its runner | one locale, no crossing (the dashed reference) |

Depth reaches the forecast on this lineage: the freeze-1m edge rebuilds every
imported tracklet's predictor input from its memo bank every cycle
(edge_manager_merged_1l_epoch.py:1853-2010); maturity is record-depth only
(hist10 and hist5 mature at import; hist2 after three local frames; pos_vel
after four, coasting on its velocity meanwhile; _MIN_HIST_TICKS = 5).

Held identical across arms: scenario xml, road-user script, boundary
(x = 240-250), trigger (predictive, LOOKAHEAD_S 1, TRIGGER_DIST 300), network
(InterLocaleLink base 3 ms + 0.5 ms jitter + payload/1 Gbps + serialization
1e-5 ms/B), source and destination load (same actors), seeds (KHONSU_SEED
1..20, seed-major grid). Modelled transfer delay: the byte-dependent terms are
payload/1 Gbps + 1e-5 ms/B = 0.027 ms at 1462 B and 0.017 ms at 922 B; the
0.010 ms spread between the largest and smallest record is 50x below the
0.5 ms jitter, so every arm is charged the same delay to within 10 us with no
code change (stated in the rows' notes; an override is not needed).

## Outputs (same runs) and lander (scripts/khonsu_state_depth_land.py)

Schema: maneuver, representation, rep, collided, completed, fde5_m,
route_time_s, hard_brakes, min_ttc_s, kb (+ tag, fde3_m, window_anchor,
history_depth_env, migration_mode, dist_m, eval_tag, machine).
- collided / completed: the campaign rule (khonsu_design_extract: binary
  collided from the RUNROW episodes; completed = 90 m).
- fde5_m: the FDE block's rule (khonsu_1l_land.run_fde): min of the two
  highest-scoring modes at 5 s, mean over the first five refreshes at or after
  the window anchor (destination commit; first local forecast for none).
- route_time_s: time_alive_s from the planner metrics dict.
- hard_brakes: separate deceleration episodes of the focal CAV at or below
  -4.0 m/s^2 from the [EGO-DBG] speed samples (0.25 s); no runtime change.
- min_ttc_s: khonsu_ablation_diag.min_ttc from the [EGO-DBG] ttc samples.
- kb: the gating oncoming's first [OBSTACLE_HANDOFF] bytes / 1000.
- representation from [LAUNCHENV] (mode + MIGRATION_HIST); maneuver from
  MANEUVER=<name> written into [LAUNCHENV] by the runner script.

## Maneuvers (constant, ltap_od, scp, lane_change)

- constant: the flow scenario as run in every landed block (ONCOMING_SPEED 12,
  the oncoming in the adjacent lane, blocker at x = 278). Exists.
- ltap_od, scp, lane_change: feasibility read in progress; the report to the
  paper side precedes any scenario build. Known already: the road user's
  acceleration is scripted (ONCOMING_ACCEL, ONCOMING_ACCEL_ONSET,
  ConstantAccelWaypointFollower, scenario_1_1l.py:24-65, 215-312) on the
  overtake-lane geometry, not a crossing; deceleration is blocked by two
  `_accel_f > 0` guards (one-character change); a turning road user exists in
  the branching harness (scenario_1_branch.py, GlobalRoutePlanner plan=); no
  lane change is scripted anywhere.

## Runner (scripts/atlas_state_depth.sh, on the atlas_fde.sh pattern)

Freeze base assert, overlay copy table, PRED_OVERLAY = the 1q predictor after
the fix ruling, OUTDIR frozen_1r_state_depth, EVAL_TAG khonsu-eval-freeze-1r,
tag sd_<maneuver>_<representation>_r<seed>, MANEUVER=<name> and the arm envs
written into [LAUNCHENV]; the no_handoff arm launches
-t openscenario_1_flow_single. Seed-major grid, 20 seeds x 6 arms per
maneuver; hard-fail gates RUNROW + [MODESROW] + mode diversity (1q).

## Reading rules (Tyler) as invalidation conditions

1. History strongly improving constant-speed driving (fde5 of hist10 well below
   pos_vel on the constant maneuver, beyond the seed spread): the snapshot arm
   is broken or its velocity is poor; check the [EXPORT-DBG] endpoint velocity
   against the actor's speed before any figure.
2. hist2, hist5, hist10 with identical fde5 under maneuvering: a short history
   is sufficient and ten frames is only the full-input choice; said as such.
3. fde5 falling across arms with completion flat: the scenario is not at the
   decision boundary; the snapshot arm should fail often and the full record
   succeed often, else the operating point moves (ONCOMING_SHIFT_X) and the
   block reruns whole.
4. Ten frames worse than five: reported, not hidden.
Plus the block's own: no_handoff must show no [OBSTACLE_HANDOFF]; every warm
arm's [EXPORT-DBG] memo= must equal its depth; a run without [MODESROW] for the
gating oncoming is skipped and counted; mode-diversity gate as in 1q.

## Gates before launch

Tyler's fix-shape ruling (1q overlay + verify_1q_cell.sh passing every gate);
the maneuver feasibility report accepted; one-cell smoke per maneuver sent to
the paper side before the grid.
