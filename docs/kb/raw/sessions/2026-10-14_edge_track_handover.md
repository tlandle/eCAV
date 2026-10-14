# 2026-10-14 session: cross-traffic track loss at the RSU handover (offline)

Branch: `sensys/edge-track-handover` from develop 58200557 (d24b53d8 plus
one KB commit), worktree `.claude/worktrees/agent-a48b2d8bc84be210c`. No
CARLA, no simulation; everything from the gate logs and the code.

## Question

In the SenSys gate runs (scenario_3, hero turning left, Tesla crossing
westbound at 13 m/s, two RSUs, late-fusion edge with AB3DMOT) the edge
loses the Tesla for up to 16 ticks and publishes it at 0 to 6 m/s.

## What the logs show

- `per_tick_age.csv` carries `conflict_dist_m` against the live GT on every
  row of a tick, so three or more rows trilaterate the Tesla's GT per tick
  (residual 0.00). The Tesla starts at tick 26, accelerates 2.8 to 13.7 m/s
  by tick 60, then holds 13 m/s, and crosses the conflict point at tick 113.
- Both RSUs detect the Tesla on every tick (no dropout). rsu_-1 (at the
  junction, sees the front face) reports 0.7 m ahead of the GT centre;
  rsu_-2 (near the start, sees the rear) reports 2.5 m behind; 3.3 m apart
  on average, 4.5 m worst case. rsu_-2 repeats a stale box on 20 of 94 ticks.
  rsu_-1 loses the Tesla from tick 98 (within 8 m of the mast).
- The tracker received a measurement that alternated between the two faces:
  the edge's cross-source NMS (3 m, keeps the first-listed) let the pair
  through on 68 percent of ticks, and AB3DMOT's own NMS then merged it but
  keeps the last-listed of equal scores.
- matching.py's kinematic gate compared the KF per-frame speed (m/tick)
  against m/s constants: every track below 10 m/s was "stationary" with a
  1.0 m gate, and the speed term was at most 0.03 m. A 3 m face flip or a
  2 m stale-repeat jump was rejected, the track coasted (published speed 0
  by design in `output()`), died after max_age 6, and the Tesla was re-born
  under a new id, which the linear predictor ignores until it has been
  published in 10 of the last 30 frames.

## Fix (three edits, all arms)

- `AB3DMOT_libs/matching.py`: convert the KF speed to m/s before the gate;
  `_D_BASE` 1.0 to 3.0 m (the measured cross-view disagreement).
- `_cross_source_nms` default 3.0 to 5.0 m (one car length, the same value
  as the per-RSU cross-camera NMS); both call sites (late-fusion family and
  VIPS) use the default; the copy in `fusion/late_fusion_backend.py`
  (adaptive and SOTA pluggable edges) set to the same value.
- `time_since_update` semantics unchanged: a re-association is a real
  measurement, so obs_tick = source_tick - time_since_update stays valid.

## Offline verification

`scripts/diag/replay_edge_tracker.py` rebuilds every tracker frame from the
`[DET]` lines and drives the real AB3DMOT with the edge's config. Before
the fix it reproduces the logged tracks (same ids and positions to 0.1 m
through tick 63). gate3 run_4 and gate1 run_10, window ticks 26 to 136:
before 12 to 13 ids, published speed error 6.9 m/s, 27 coasting ticks at
speed 0, planner-visible gaps 41 to 48 ticks; after 1 id, no coasting,
speed error 2.2 m/s (KF lag during the acceleration and a dip at the tick
98 source switch), planner-visible gap 4 ticks. max_age 12 alone: 7.5 ids,
46 percent of planner-visible ticks stationary. Sweep: the allowance must
reach 3 m for one id. Result identical at both box-extent extremes.

All 212 complete gate runs (both gates, anchoring on and off, 0 to 500
ms): ids 10.5 to 1.8, published speed error 6.3 to 2.7 m/s, coasting ticks
28.7 to 3.5, planner-visible max gap 23.3 to 8.2 ticks mean. Residual 18 to
33 tick gaps in 36 runs start at the conflict point: the identity
exclusion zone culls the Tesla's anonymous track while it passes within
2 m cross-track of the stopped hero's beacon track (zone off in the replay:
tracker gap at most 5, planner gap at most 13). Not changed; reported.

Merged develop twice (1b2bcb88 lineage, 3919a5f5 coasting velocity).
tests/test_obs_tick_lineage.py passes on the merged tree.

## Files

- AB3DMOT_libs/matching.py
- ecav/core/application/edge/edge_manager/edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py
- ecav/core/application/edge/edge_manager/edge_manager_vips_temporal_alignment.py
- ecav/core/application/edge/fusion/late_fusion_backend.py
- scripts/diag/replay_edge_tracker.py (new)
- docs/kb/wiki/current_state.md

## Live check still needed

One gate cell (`test_runner.py -t openscenario_3_edge_late_fusion
--manager-types late_fusion --anchoring on --latencies 0 --speeds 70
--ego-counts 1 --repetitions 10`) and read per_tick_age.csv: one
conflict-actor track id from tick 26 to the crossing, no published speed
below 10 m/s after tick 60, no external_stale brake on the Tesla.
