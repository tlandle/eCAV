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

## Experiment 2 (architecture by fidelity): intermediate-fusion detector diagnosis (2026-11-12 00:55)

Symptom: the intermediate-fusion arm (plain `worldfusion` manager, `openscenario_3_edge_late_fusion_wf`)
tracked the crossing car on about 15 % of frames and collided under the full system.

Cause: the generated YAML inherited the object-sharing arms' 7 m RSU masts and used epoch 27 of
`worldfusion_multiv2x_translaug_finetune`. The configuration on record
(`openscenario_3_edge_worldfusion_mtr_live.yaml`) is epoch 45 on a 4.3 m mast, the Multi-V2X
training height. The model's voxel range is z in [-6, 2] m in the lidar frame, so from 7 m the
ground and all but the roof of a car fall below the range.

Attribution (probe `WF_RECALL_PROBE`, stamp wfdiag1 on cetus, left turn 70 km/h, zero delay, one
run per variant, fusion ticks with the car moving and within 60 m of the first RSU). Lidar coverage
is 100 % of ticks in every variant (occupied input cells within 2.5 m of the car), so the loss is
model output, not coverage, transform or association:

| mast | epoch | fused score at the car, median | ticks >= 0.15 | >= 0.20 | detector TP / FN / FP | conflict-actor tracked ticks |
|---|---|---|---|---|---|---|
| 7.0 m | 27 (was) | 0.090 | 30 % | 21 % | 24 / 99 / 1235 | 72 |
| 7.0 m | 45 | 0.037 | 0 % | 0 % | 0 / 0 / 0 | 8 |
| 4.3 m | 27 | 0.216 | 67 % | 51 % | 81 / 42 / 2921 | 119 |
| 4.3 m | 45 (now) | 0.267 | 81 % | 74 % | 70 / 0 / 90 | 200 |

At 4.3 m and epoch 45 the score clears 0.15 on 98 % of ticks at 15 to 30 m and 88 % at 30 to 45 m.
Past 45 m from the first RSU it is 8 %: that is after the conflict point. Epoch 27 produces 13 to
32 times the false positives of epoch 45 (the scene-overfit behaviour on record).

Fix: `scripts/ispass/gen_fidelity_wf_yaml.py` defaults to epoch 45 and a 4.3 m mast for the
intermediate-fusion arm (RSU x, y unchanged). `net_epoch45.pth` is not in git or LFS
(sha256 aa2563e791b49426...), copy it to each host.

Gate (stamp wfgate, 2 seeds): idealized 2/2, age at first brake 0 ms; full system (149 ms compute,
32 ms up, 20 ms down) 2/2, age at first brake 250 ms, yields 15 m short of the crossing lane;
published speed 0.98 to 0.99 of truth; coverage 0.95 to 1.00.

Finding outside this fix: an ego that receives nothing (7 m, epoch 45: no true brake) passes the
crossing car at 3.9 m centre distance without contact and counts as an operational success at
this cell. The outcome at zero added delay does not separate a blind arm from a seeing one.

### Added-delay axis and campaign launch (2026-11-12 02:27)

Gate 1 (stamp ispass_fid2_gate, conditions A and D, both geometries, added delay 0 and 200 ms, one
seed) showed the 200 ms added delay was not applied under table delivery for V2X2V and intermediate
fusion (age at brake 50 to 100 ms and 250 ms, the same as at 0 ms). With `use_ns3_lut` on, both
managers stamp arrival from the table sample only. New option `lut_plus_configured` (default off, so
existing campaigns are unchanged) adds the configured delay. Conditions C and D pass it.

Gate 2 (stamp ispass_fid3_gate, conditions C and D): age at brake at 200 ms added delay is 250 ms
(I2V), 250 to 300 ms (V2X2V), 300 ms (intermediate fusion, network only) and 450 ms (intermediate
fusion, full system: 145 ms compute, 32 ms up, 20 ms down). Every cell 1/1.

Campaign: stamp ispass_fid3 on cetus (`~/exp2_wt` at 1394d117), three architectures, conditions A to
D, left turn and straight crossing, added delay 0 and 200 ms, table at N=16, 10 seeds per cell then
30 (same stamp). 1,440 runs at 30 seeds. Started 02:27.
- status: `ssh tlandle@143.215.184.49 'cd ~/exp2_wt && scripts/ispass/fidelity_status.sh ispass_fid3 logs/fid3_full.log'`
- land: rsync `exp2_wt/experiment_results/{openscenario_3_edge_late_fusion,scp_edge_late_fusion}/ispass_fid3` (simulation_metrics.json, per_tick_age.csv, runner_stdout.txt), then
  `PYTHONPATH=.:ecav/protos python scripts/ispass/fidelity_reduce.py ispass_fid3 --csv paper_ispass/fidelity.csv --fig paper_ispass/fig_fidelity.pdf --fig-n 16 --fig-lat 200`

## Experiment 4, radio fidelity (2026-11-12 09:09, atlas, stamps rf_20261112 and ispass_rfcl)

Open loop (scripts/ispass/radio_fidelity_bench.py, 600 ticks, 3 seeds, 2 at N 24 and 31 with 16.9 KB): live
ns-3 NR Uu against the stored tables and the SEE-V2X trace, table in paper_ispass/radio_fidelity_table.md,
figure paper_ispass/fig_radio_fidelity.pdf. Table within 0.6 ms (uplink p50) and 1.7 ms (downlink p50) of
live through N 16 at every payload, except downlink p50 at N 8 with 16.9 KB (16.1 against 23.1 ms). At
16.9 KB and N 24 the table gives delivery 0.37 and p50 223 ms, live gives 0.80 and 122 ms. At N 31 the
table gives 0.22 and 401 ms to every sender, live gives 0.27 with 19 of 31 senders receiving nothing.
The table's N 24 and 31 feature cells hold 67 and 50 delivered samples (about 7 ticks of ns-3). Live ns-3
costs 0.16 to 6 wall s per simulated s at 300 B and 1 KB and 0.5 to 26 at 16.9 KB.
Closed loop (scripts/ispass/radio_fidelity_closed_loop.sh, left turn, intermediate fusion, 70 km/h,
measured compute, 16.9 KB both directions, 10 paired seeds): every arm completes 10 of 10 at both loads
(correction Nov 12: the no-cooperation check ispass_fid5gate collides 3 of 3, so the encounter needs cooperation; the arms do not separate because the encounter tolerates more than 1 s of added delay). p99 age at N 16: table 338, live 305 ms.
At N 24: table 536, table with loss 766, live 300 ms, because ns-3 serves the managed CAV (sender 0)
at 48 ms with full delivery while other senders starve. New edge options on the intermediate-fusion
manager, all default off: radio=live, lut_loss=true, msg_b=<bytes>. No 5G-MOBIX trace replay exists.
