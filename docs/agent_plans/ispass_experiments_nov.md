# ISPASS 2027 experiments: readiness inventory (2026-11-09)

Companion to `paper_ispass/ispass2027_action_plan.md` (the four P0 experiments and the
9-page blueprint). This file records what exists on disk, what still has to run, and
what the draft's `[PLANNED]` paragraphs will consume. The Overleaf-linked repo is
`~/repos/ecav_ispass` (GitHub tlandle/ecav_ispass, master, fc5daea = the 2026-08-14
review copy, plus eight uncommitted bib entries from the action plan's citation list;
Tyler 2026-11-09). `~/repos/ecav_ispass_overleaf` (main, d49658e) is a Sep 20 import of
the same contents and is not the working repo.

## Experiment A: timing-model fidelity (draft 5 "Timing-model fidelity [PLANNED]")

- Figure built 2026-11-09: `scripts/ispass/plot_timing_fidelity.py` writes
  `paper_ispass/fig_timing_fidelity.pdf` (7 in by 1.55 in, 7 pt text): age p50/p95, update
  rate and superseded fraction against N for both services.

- Harness: `scripts/bench_timing_fidelity.py` (untracked, 2026-11-05 21:52). Loads
  PointPillarWorldFusion from `ecav/ml_manager/models/worldfusion_multiv2x_caronly_aug/
  net_epoch31.pth` plus the ns-3 LUT sampler, drives N in {1, 2, 4, 8, 12, 16, 24, 32}
  for 160 ticks, compares a continuous 20 Hz wall-clock service against the 50 ms
  barrier model (compute + UL + DL stamps).
- Output present: `paper_ispass/timing_fidelity_table.csv` and
  `timing_fidelity_results.json` (2026-11-05 21:50, atlas RTX 4080S). Mean-age error
  11 % at N=1, 33 % at N=2, 11 % at N=4, 13 % at N=8, 6 % at N=12, 5 % at N=16, under
  1 % at N=24, 3 % at N=32; update-rate error 3 to 38 %; superseded fraction tracks
  within 3 points from N=8 up. The N=2 point is the outlier (the 50 ms quantisation of
  a 52 ms compute).
- Still to do: a figure (Fig. 4 of the blueprint: p50/p95/p99 age, rate, superseded
  fraction, error vs N), the A10 repeat on the MEC-representative GPU (azure A10 per
  `reference_azure_a10_access`), and the prose. The harness accepts `--n-list` and
  `--ticks`.

## Experiment C: reproducibility and replay (draft 5 "Reproducibility [PLANNED]")

- Output present: `paper_ispass/reproducibility_results.json` for N in {4, 8, 16}, 10
  repeats each: live compute std 4.7 / 6.2 / 7.7 ms, boundary-shift fraction 5.0 / 0.0 /
  3.6 %, trajectory RMS 0.0 m in both modes. The replay mode is the `--timing-mode
  replay --timing-trace` path in the working-tree changes to `ecav.py`,
  `ecav/ecav2/arg_utils.py` and `edge_manager_worldfusion_ab3dmot_linear_predictor.py`
  (uncommitted, another session's work: do not commit from here).
- Still to do: the trajectory-RMS numbers are 0.0 for the live mode too, which means the
  closed-loop part of C.1 (fixed-seed CARLA repeats under live timing) has not run or
  the metric is not wired; confirm with the session that produced the file. C.3 (fork
  two policies from one snapshot) has `scripts/branching/` (daemon_branch.py,
  gen_branch_scenario.py, openscenario_1_branch_k{2,3,4}.yaml) from the NSDI work.

## Experiment B: design-conclusion reversal (draft 6, Fig. 5 and table)

- Data already on disk, no new runs: idealised and compute-only rankings from the SEC
  results (`~/repos/conductor_sec_short`, memory `project_sec_paper_progress`), radio
  rows from the ns-3 PC5/Uu measurements (`evaluation_outputs/canvas_latency.csv` and
  the SenSys export `architecture_age.csv`, `architecture_emulation.csv`), and the
  sensor-derived closed-loop rows from the SenSys export `runs.csv` (I2V, V2X2V,
  cooperative planning, feature fusion at 70 km/h).
- Verified pointers (2026-11-09): the export's `architecture_age.csv` gives the measured
  compute configuration, p99 age per architecture at N = 4, 8, 16, 24, 31 (exact state 150
  to 164 ms, I2V objects 169 to 212, V2X2V prediction 175 to 220, cooperative planning 175
  to 217, intermediate feature fusion 226 to 368); `architecture_emulation.csv` has the
  model against emulated p99 and update rate per architecture and N. The accuracy row
  (idealised configuration) comes from the SEC paper's `tab:pred_quality` (FDE/ADE at 1, 3,
  5 s per prediction method) and its AP@0.7 numbers in `~/repos/conductor_sec_short/contents`.
- Still to do: one assembly script that builds the four-configuration table and figure
  from those files (nothing exists yet).

## Experiment D: four-arm attribution (draft 5 "State-model divergence [PLANNED]")

- Table built 2026-11-09 from the export: `scripts/ispass/four_arm_attribution.py` writes
  `paper_ispass/four_arm_attribution.csv` (98 rows: both junctions at 70 km/h, exact state
  at 0 ms, exact state at every delay, cooperative stack at every delay, no cooperation).
  At 0 ms: exact state 1.00 on both junctions, cooperative stack 0.96 (left turn) and 0.92
  (straight crossing), no cooperation 0.00 (10 of 10 collide) on both.

- The SenSys export already holds three of the four arms at N=1 on two junctions:
  `oracle` at 0 ms (Arm A), `oracle` at every configured delay (C-lat, the
  latency-matched oracle), `late_fusion` +SBA (cooperative sensor arm), and
  `encounter_validation.csv` has the no-cooperation outcome per run (Arm B).
  First-detection distance is not in the export; the per-tick age logs of the local
  campaigns (`experiment_results/openscenario_3_edge_late_fusion/sensys_nov27_j6*`)
  carry `conflict_dist_m` per consumed object, from which it can be reduced.
- The August E6 ladder (`scripts/e6_sweep.sh`, `scripts/e6_divergence_metrics.py`,
  `docs/agent_plans/ispass_divergence_scenarios.md`) is the blind-overtake version of
  the same experiment with a contributor-count axis; it never ran (GPU gate) and its
  arm configs predate the Oct 16 blind-overtake geometry fix.
- Still to do: decide junction (export data, zero new runs) versus blind overtake
  (E6, needs runs); write the reduction either way.

## Draft and Overleaf

- Tyler said he changed the Overleaf text. Neither GitHub repo has anything after the
  2026-08-14 content (fetched 2026-11-09), and no Overleaf git credential exists here,
  so his edits reach this machine only through the Overleaf to GitHub sync into
  `tlandle/ecav_ispass`. Until then the rewrite works from the August copy.
- Prose fixes the action plan already lists: the 256 / 128 / 64 / 4 to 16 scaling
  numbers are conflated in 5 (control-only 256 on 4 hosts; per-vehicle perception 64;
  full edge stack 1 to 4 per locale), implementation still says LitServe (now grpc.aio
  servers), LTE-V2X Mode 4 vs NR-V2X Mode 2 naming, LUT accuracy separated from
  field-trace calibration, and the application papers (Conductor, SBA, Khonsu) framed as
  case studies of the instrument.
