# relay_eval_2026_08: Khonsu closed-loop data

One CSV row per run. Binary outcome fields: `collided` (any ego contact), `completed`. Report collided-or-not and completed-without-collision only; never mean episodes.

| File | What | Harness (develop) | Status |
|---|---|---|---|
| design_sweep_v3_rows.csv | six flow arms x 10 + burst arms x 5 on the corrected scenario (ONCOMING_SPEED=12, TRIGGER_DIST=300 as runner defaults) | scripts/khonsu_rebaseline_arms.sh | citable re-baseline |
| q5_scale_rows.csv | oncoming density N=2/4/8 (FLOW_N releases the first N of 8 parked oncoming vehicles at scenario start; all N cross the boundary; unconnected actors, one migration per crossing), arms cold / edgewarp (at-crossing one-frame snapshot, paper name "handover snapshot") / warm (forecast trigger 1 s, full history), 5 runs per cell | scripts/khonsu_q5_scale.sh; FLOW_N knob in ecav/scenario_testing/scenarios/scenario_1.py | citable (collided field); completion-field format under confirmation |
| design_sweep_v1_rows.csv | 150-run uniform design sweep on the unvalidated geometry | scripts/khonsu_design_sweep.sh | relative findings only, not citable absolutely |
| fault_injection_results.csv | eight ownership-epoch scenarios through the per-track state machine | unit harness | citable |
| canvas_latency.csv | fusion latency and memory vs BEV canvas side, 8 contributors | see current_state.md 2026-08-31 | citable (Fig. 1) |
| q3_lookahead_sweep.csv, q7_locale_sizing.csv | August pilots | superseded by s5 lookahead redo (scripts/khonsu_s5_lookahead_pinned.sh, worktree khonsu_v1_wt at 9d5e1883) and the sizing study | do not cite |

Retired: q4_flow_table.csv (canonical warm arm reproduces 4/4 under the env contract, but the reactive and edgewarp arms never transferred). Provenance history in docs/kb/wiki/current_state.md (2026-08-31, 2026-09-03, 2026-09-04) and raw/sessions.
