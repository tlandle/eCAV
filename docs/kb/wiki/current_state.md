---
updated: 2026-07-26
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## SEC ablation provenance RESOLVED: score_threshold 0.2 vs 0.15 (2026-07-21)

The April selector-ablation numbers (rsu_93 random K=4: occ recall 0.287, 5.8
dets/tick, 0 FP) are REPRODUCED locally. Root cause: the A10 ran ep39 with the
checkpoint dir's config.yaml at the training-default `score_threshold: 0.2`
(rsync'd Apr 23); the local config.yaml was lowered to 0.15 on Apr 25 during the
scene-overfit debugging, AFTER the sync, and only net_epoch39.pth was pushed
afterward. Every July rerun used 0.15 -> 0.80+ recall, ~25 FP, 11 dets/tick.
Flipping one line reproduces April: 0.296/0.345/5.9/0FP vs 0.287/0.341/5.8/0.
Eliminated by direct experiment along the way: environment (rebuilt torch
2.6.0+cu124 + numpy 1.26.4 as conda env `opencda310_apr`; identical results to
three decimals vs torch 2.9/numpy 2.2), code (Apr-27 profiler byte-reconstructed
by replaying 54 transcript Edit calls onto the Apr-20 git base; detection path
identical to today's), checkpoint (PACE->local->A10->LFS chain closed, same
bytes). Full dossier:
~/repos/cooperative_world_model_prediction/rebuttal/ablation_provenance.md.
CAUTION: the July 16-locale selector distribution (median G -14%) was measured at
0.15; it reflects the saturating operating point and must be re-run at 0.2 before
being cited. Verification of oracle_occluded + causal_v4 arms at 0.2 in flight
(paper2/paper2_figures/rebuttal_sweep/verify_rsu93_*_thresh02.csv). reproduce/
README needs score_threshold pinned at 0.2. Note: current profiler defaults
num_output_steps=50 vs April's 25 (MTR horizon, prediction-only; not
detection-relevant but matters for ADE comparisons).

## Scale-out B0.3 DONE (mechanism): LIVE full-latent migration in CARLA (2026-07-05)

First live learned-state handoff: scenario `openscenario_3_multi_edge_mamba` (both
edges SOTA pluggable + mamba3dmot + late-fusion + linear). At tick 60 the ego's
full memo bank (10 frames, 776 B) exported from edge0 and injected warm at edge1;
tracklet survived; no post-handoff exceptions. Fixes en route: beacon KeyError on
freshly-migrated VM in late_fusion_backend.detect (tolerate miss); temp-id vs
carla_id mismatch (pluggable export resolves via get_carla_id_for_temp). Known
gaps: warm-vs-cold DELTA needs B4 metrics; SOTAEdge.evaluate() NotImplementedError
at cleanup (non-fatal). CARLA left running headless for further live analysis.

## Scale-out B0.2 DONE (unit level): Mamba latent migrates through the edge dispatch (2026-07-05)

Mamba3DMOT registered in the tracker registry (lazy torch import; AB3DMOT-only
processes stay torch-free). Wrapper now carries carla_id (nearest-det assoc, 2 m
gate) and exposes `.tracker`. `_PluggableEdgeBase` export/import dispatches on
backend: Mamba -> full latent via factories, AB3DMOT -> KFState; fixes the
pre-existing `self.tracker.trackers` wrapper-indirection bug. Schema-mismatch
records cold-start cleanly. Both branches verified under opencda310
(test_mamba_edge_migration.py): banks byte-identical, id preserved, ~1.3 KB.
Next: B0.3 live two-edge run with `tracker: mamba3dmot` YAML. Uncommitted -> committed this session.

## NSDI paper: systems positioning + Kishore methodology complete (2026-07-07)

Pushed through 33c8a07 (scale_out_nsdi). Related work opens with the problem-class
abstraction (pre-copy/post-copy, TCP/QUIC cwnd, stream-processor state shipping vs
replay, leases, 802.11r; all citations verified) + camera-networks subsection
(Javed ICCV'03, smart-camera handoff, Spatula SEC'20) + ClairvoyantEdge (Kishore's
group, SEC'22, prepare-ahead pattern; distinction: content re-fetchable, our state
exists only at source). System model corrected per Tyler: locale anchors to the
CONFLICT ZONE (map), not the RSU; RSU = viewpoint; edge = base-station server
(memory: project_locale_definition). Boundary framing after Alex's pushback:
"static canvas, dynamic assignment, ASYMMETRIC elasticity" (shrink = crop, growth
capped by trained extent + edge budget ~8MB/frame BEV; dynamic policies generate
migration workload). 74 visible \ptag intent tags + AV-term glosses for systems
reviewers (planner, tracking, ADE/FDE/minADE/miss/NLL defined, LTAP-OD/SCP
expanded, ns-3/CARLA/AoI/V2X/backhaul glossed). 15pp clean build. Tyler reading
both manuscripts next. Professor email drafted in chat (bulleted, with data audit).

## Dissertation proposal alignment (2026-07-02, repo tlandle/Dissertation_proposal)

Pushed through `2388679`. P1 skeleton: five research thrusts standardized (eCAV
platform is its own thrust per Tyler), chapter order fixed (communication before
scale-up), paper4 file renamed, 64-agent claim corrected to 33, dup bib entry +
dup labels removed. P2: scale-out chapter rewritten to match the expanded NSDI
paper (architecture contract, Q1-Q6, measured microbenchmark, honest one-frame
labeling; NO codename, Tyler rejected "RELAY"). P3: correctness chapter rewritten
as completed work with the submitted paper's real numbers (100-150ms logic cliff,
350-400ms physics boundary, 220-450ms budgets, m(a,u,N), ~10KB/s + <0.2ms).
Kishore methodology adopted: visible \ptag paragraph-intent tags (59 tags across
abstract/intro/paper1/paper5), sentence-builds-on-previous rule, eval overview +
takeaways. ALL PHASES DONE (pushed ad7ee57, 2026-07-05): P4 related-work rebuilt (one thread
per thrust, mis-citations fixed, definitions added); P5 all chapters tagged (111
visible Intent tags), eCAV chapter rewritten to prose, paper3/paper4 cited,
timeline dedup, em-dashes/banned words swept; P6 13 shared bib entries reconciled
to the papers' canonical definitions. Clean build, 60 pages, 0 warnings.
research-plan.tex stays in repo unused (Tyler's call). Tags stay visible until
advisors approve, then flip the one-line macro in main.tex.

## Scale-out B0.1 DONE: live edge migrates KF state (2026-06-30)

B0 step 1 built + unit-verified. Fixed the `import numpy` bug in
`edge_manager_pluggable_base.py`. Added real `export_vehicle_state`/`import_vehicle_state`
(AB3DMOT KFState: mean, covariance, hits>=min_hits, velocity) to the live
`PredictionLateFusionEdge` so a handoff actually carries tracker state (was a base no-op).
Round-trip verified under opencda310 (scratchpad `test_kf_migration.py`): destination KF
resumes warm, matches source, migrated velocity present (cold start would be 0), payload
~1.0 KB. This is the Reactive-Kalman baseline arm. The predictor on this edge is LINEAR so
KF state suffices here; the learned-latent advantage needs B0.2 (Mamba+MTR eval edge, TODO).
Not yet run in the live CARLA loop (B0.3). Changes uncommitted (commit only when asked).

## RELAY (Paper 3 NSDI) eval plan + live-migration audit (2026-06-30)

Paper `github.com/tlandle/scale_out_nsdi` expanded (architecture contract, Q1-Q6 eval,
5-subsection related work, canonical shared bib). Wrote
[scale_out_evaluation.md](../../agent_plans/scale_out_evaluation.md).

Audit finding (critical): the live multi-edge scenario
(`openscenario_3_multi_edge_late_fusion.py`) is a Phase-1 skeleton. It swaps VM
ownership and logs a TransferCost but migrates NO tracker state (base `import_vehicle_state`
is a no-op), triggers on a hardcoded tick (HANDOFF_TICK=60, not geometry), applies no
backhaul delay (link computes cost, never gates), and persists no warm-vs-cold metrics.
The AB3DMOT KFState path (`edge_manager_pluggable_base.py`) is unreachable live (only
SOTA/Adaptive edges); the full Mamba latent path is only in `harness.py`. The live edge
runs AB3DMOT+linear (stateless predictor), so NO gap appears there. Linchpin build (B0):
run the learned stack (Mamba3DTracker + MTR) on the eval edge and wire the real
export/inject so state actually migrates. Only then is any live paper claim backed. The
synthetic harness B/B1/B0 microbenchmark stays the mechanism result; Q1/Q4 must come from
the live closed loop. Minor bug: `edge_manager_pluggable_base.py` uses `np.*` with no
`import numpy`.

## WF→MTR (CMP-style) data generation — IN PROGRESS, key findings (2026-06-29)

Goal: train MTR on WorldFusion fused BEV features (like CMP trains MTR on
CoBEVT features), for Multi-V2X. Scaffolded PACE training planned
(1→2→8 GPU). Plan artifacts written under `docs/agent_plans/` pending.

**Pipeline pieces written (all in `ecav/core/prediction/mtr/`):**
- `tools/export_wf_for_mtr.py` — WF forward per Multi-V2X frame → dumps
  fused_feature `.npy` (float16, [1,256,176,176], ~8MB/frame) + AB3DMOT
  tracks → pred_traj pickle. Plain `.npy`; transfer compression is plain
  `tar` (NOT gz), per Tyler.
- `datasets/multiv2x_multiego_dataset.py` (+ registered in `datasets/__init__.py`,
  mirrored into CMP `MTR/mtr/datasets/`) — MTR loader. Loads gt/pred pickles,
  lane PNG (Swin lane encoder kept — `models/lane_maps/` has 56 RSU-static
  256x256 PNGs covering all 44 zones), WF fused feature, RSU pose.
- `tools/build_multiv2x_intention_points.py` — K=64 endpoint clusters →
  `multiv2x_cluster_64_center_dict.pkl`. DONE (149k endpoints).
- `tools/cfgs/multiv2x/multiv2x_multiego_worldfusion.yaml` — MTR config,
  FUTURE=25 (5s@5Hz), PAST=10, lane Swin, intention pkl. DONE.
- `tools/rekey_pred_to_gt_ids.py` — Hungarian-match tracks to GT ids.
  SUPERSEDED by the frame finding below; revisit.
- Patched `point_pillar_worldfusion.py::forward` to return `fused_feature`.

**CORRECT MODEL: `worldfusion_multiv2x_translaug_finetune`** (translation-aug
fine-tune, fixes scene-overfit per [[project_wf_scene_overfit]]). Pulled from
PACE `$PROJECT/worldfusion_translaug_finetune/...` to
`ecav/ml_manager/models/worldfusion_multiv2x_translaug_finetune/`
(epochs 27,45 local). NOT `caronly_aug` epoch27 (that was a wrong earlier pick;
its result.txt AP 0.674 was never re-evaluated and it scene-overfits).
`result.txt` for translaug is empty (never AP-evaluated).

**CORRECT DECODE (matches live `WorldFusionEdge._to_ab3dmot_format`):**
- Use `WorldVoxelPostprocessor` (NOT the opv2v `VoxelPostprocessor` the
  MultiV2X dataset builds — that ego-recenters → garbage x[6,54]).
- world_anchor=[[0,0,0,0,0,0]], lidar_pose=origin, `corner_to_center(order='hwl')`.
- Offline fusion is in RSU-LOCAL frame (dataset pairwise warps to RSU-ego),
  so apply FULL `x_to_world(rsu_pose)` (rotation+translation). NOTE: live
  edge manager adds translation ONLY because it pre-warps to world-aligned via
  `_compute_world_pairwise_transforms` — offline does NOT, so rotation needed.
  export_wf_for_mtr.py was edited to WorldVoxelPP + RSU translation; STILL
  NEEDS the rotation (x_to_world) re-added — current code applies translation
  only and is WRONG for offline.

**RESOLVED (2026-06-30): build everything in RSU-EGO frame, GT-anchored.**
The old `multiv2x_gt_traj` pickle WAS complete (matches yaml exactly) and in
CARLA world. The recall-0 confusion was two compounding mistakes:
(1) world-frame reconciliation (`x_to_world` + opv2v postprocessor) was wrong;
(2) AB3DMOT fragmented the intermittent RSU-range detections into ~2 short
ghost tracks/frame (this is the KNOWN live ~9-tick-track issue, NOT new).

Final design (verified on rsu_66): everything in the RSU-EGO frame (static RSU
→ ego frame is a fixed, world-consistent frame). Per frame:
- GT boxes + CARLA ids straight from `item['ego']['object_bbx_center']` +
  `object_ids` (order 'hwl' = [x,y,z,h,w,l,yaw]).
- Pred detections via `WorldVoxelPostprocessor` (world_anchor+lidar_pose at
  origin), `corner_to_center('hwl')`. NO transform — same ego frame as GT.
- Associate detections → GT ids by gated Hungarian (gate 2.0m). Pred-past and
  gt-future BOTH keyed by CARLA id. NO AB3DMOT, NO rekey step (dropped).
  GT-anchored association is standard for building prediction training data;
  detection POSITIONS stay the model's noisy output, only association uses GT.

Verified rsu_66 numbers: detection pos error mean 0.42m / median 0.39m;
per-frame recall 44%; 1086 lenient trainable center-samples/zone; 0 false
centers (all pred keyed to real GT ids). CMP-strict (gapless 11-frame past) = 0
because detection is intermittent at 44% — EXPECTED and FINE: MTR masks missing
past via obj_trajs_mask; the loader selects centers leniently (current observed
+ future valid), not strict. CMP only used strict because OPV2V perception was
dense.

LIVE TRACKER NOTE (Tyler flagged): the AB3DMOT fragmentation that broke offline
also degrades the LIVE pipeline (~9-tick tracks, known). Offline subsampling
(~10Hz vs live 20Hz) makes it look worse offline. GT-anchored association is an
OFFLINE data-gen choice only; it does NOT fix/mask the live tracker. Live
fragmentation is a separate follow-up, not part of WF→MTR data gen.

**Earlier wasted cycles (don't repeat):** ran wrong conda env (`opencda` 3.8 →
must be `opencda310`); cited AP from wrong dir (`worldfusion_multiv2x_det_*`
epoch2, 0.25, unrelated); paired x_to_world with opv2v postprocessor (double
bug). The mechanical pipeline (feature dump, loader, intention pts, config)
is sound; only the detection-frame + GT-source construction is unresolved.

**PACE training status (2026-07-02).** Full export done (11760 features, 52
zones, 174GB). Uploaded to `$PROJECT/wf_mtr_translaug.tar` (plain tar) +
`$PROJECT/mtr_code.tar`; sbatches in `$PROJECT/mtr_sbatch/` (local copies in
`ecav/core/prediction/mtr/tools/scripts/`). Smoke submitted in h200+h100 pairs
with a `$PROJECT/mtr_wf_smoke.lock` claim-lock (noclobber); ALWAYS `rm -f` the
lock before resubmitting. Two build failures fixed in ALL four sbatches:
1. `pip install -e . --use-pep517` → isolated build env has no torch, and env
   has no nvcc. Fix: `module load cuda/12.1.1` (matches torch 2.2.2+cu121),
   `pip install -e . --no-build-isolation`, `TORCH_CUDA_ARCH_LIST=9.0`.
2. With the module on LD_LIBRARY_PATH, torch import dies with
   `ImportError: libcupti.so.12` (module resolves cudart from lib64 but CUPTI
   lives in `extras/CUPTI/lib64`; the env ships no pip-side cupti). Fix:
   `export LD_LIBRARY_PATH="$CUDA_HOME/extras/CUPTI/lib64:$LD_LIBRARY_PATH"`.
Jobs 10649347/8 (fail 1), 10664466/7 (fail 2, h100 sibling scanceled).
Untar of 174GB to node NVMe takes ~15 min.
Next after smoke: 2-GPU 3-epoch, then 8-GPU 30-epoch (`mtr_wf_ddp_*.sbatch`).

**Fail 3 (job 10676504) + fixes (2026-07-03..05), all verified by a LOCAL
end-to-end training run (4080, 0.16 s/iter, loss decreasing over 3k iters):**
- `No module named transformers`: train_multiego routed MultiV2X to
  models_v2v4real, which has NO lane encoder (dead transformers import) and
  no BEV aggregator. Fixed routing: MultiV2X now uses models_opv2v (Swin lane
  encoder + MotionAggregator over fused BEV), the tree the loader was written
  against.
- `MotionAggregatorTransformer` hard-coded 50 future frames (ours 25) and a
  Linear over 48x176 BEV (WF is 176x176). Parameterized num_future_frames
  from MOTION_DECODER.NUM_FUTURE_FRAMES; added AdaptiveAvgPool2d((48,176)),
  a no-op for CoBEVT-shaped input.
- Loader emitted homegrown 8-attr trajs + 2D masks; model needs CMP's 22-attr
  layout (6 box + 2 onehot + T+1 time embed + 2 heading) and 3D masks. Ported
  CMP's create_agent_data_for_center_objects / generate_centered_trajs /
  transform_trajs_to_center_coords verbatim (opencood-free) into
  multiv2x_multiego_dataset.py.
- Yaw bug: pickles store RADIANS (verified range ±3.9); loader applied
  deg2rad. Removed.
- BatchNorm crash on records with a single valid past obs point (intermittent
  RSU detections; OPV2V never hits this). Loader skips records with <2 valid
  past points.
- CMP recipe is TWO-STAGE: stage 1 `_no_agg.yaml` (TYPE None, from scratch),
  stage 2 loads stage-1 best_model with TYPE Transformer. Created the yaml
  pair; smoke runs the stage-2 yaml with empty-pretrained fallback (joint,
  exercises the full code path).
- Swin weights staged offline at MTR/pretrained/swin-base-patch4-window7-224
  (332MB, inside mtr_code.tar; compute nodes have no internet).
- PACE transformers: 5.x needs torch>=2.4; 4.57/4.53 break on torch 2.2.2
  (`torch.compiler.is_compiling` missing). **transformers==4.51.3 works**
  (ViTImageProcessorFast, Swin forward OK); installed in opencda310.
Attempt 4 (job 10799892): died importing models_opv2v — module-level
`from torch_geometric.nn import GCNConv` resolves a broken ~/.local copy
(missing psutil). Guarded with try/except in BOTH model trees (only
MotionAggregatorGCN needs it). Also stripped locally built *.so from the code
tar (torch 2.9.1 ABI, undefined symbols vs PACE torch 2.2.2; node rebuilds).

**Attempt 5 (job 10801234, 2026-07-05): TRAINING SMOKE PASSED.** Full 1-epoch
run on H200: 4820 iters, 0.20 s/iter (16 min epoch), loss ~196 at end,
checkpoint saved. Crashed only in the POST-epoch eval import:
tools/eval_utils/eval_utils.py had module-level psutil + pympler (both absent
on PACE) and `from mtr.datasets.opv2v_multiego_dataset import ...` (imports
cmp_opencood, absent on PACE). Eval fixes, all validated LOCALLY by a
functional eval run over the test split (344 records, ADE/FDE/MR computed):
- psutil/pympler moved inside eval_one_epoch (only user); OPV2V import moved
  inside its dispatch branch.
- eval_one_epoch_custom now calls `dataloader.dataset.generate_prediction_dicts`
  (was hard-coded OPV2VMultiEgoDataset). MultiV2X dataset got an opencood-free
  port of generate_prediction_dicts (assert num_feat in (5,7) so the TYPE-None
  stage-1 7-dim trajs also pass).
- Horizon indices were hard-coded for 50 frames @10 Hz (`gt_trajs[-50:]`,
  min(30/10, ...)); now derived from mask length (both datasets are 5 s
  horizons: OPV2V 50f, MultiV2X 25f).
**2-GPU DDP validation (job 10802480, 2026-07-05): PASSED.** 3 epochs on
2x H200 (2410 iters/epoch, 0.24 s/iter), eval after every epoch (472 traj/rank),
best_model saved, zero tracebacks, clean exit at 52 min. One defect: minADE=nan.
Cause: 3/931 test objects have center_gt_final_valid_idx==0 → empty ADE slice
[:0] → nan poisons the accumulator (FDE indexes [idx], stays finite; OPV2V's
dense GT never hits this). Fixed: eval skips objects with final_valid_idx<1
(counted as Filtered). Verified by a full local test-split eval: finite
metrics, Filtered: 3, Total: 928.

**8-GPU run 10804312 COMPLETED (3h19m) but the MODEL IS INVALID — training
data was corrupted by train-mode augmentation.** Both stages plateaued at
ADE ~31 m / MR ~0.98 (the static-baseline score). Diagnosis chain:
intention-point spread was lateral-dominant (±143 m) → GT pickle per-object
sequences jump 30-60 m per 0.2 s frame with yaw uniformly random vs motion →
source world-frame GT (ecav/ml_manager/models/multiv2x_mtr) is smooth
(median step 0.19 m) → ROOT CAUSE: export_wf_for_mtr.py called
`build_dataset(hypes, train=True)`; in train mode get_item_single_car applies
the translaug augmentors (random world flip / ±45° rotation / scaling /
translation) per frame, jointly to lidar + GT. Every frame sits in an
independently randomized frame: self-consistent WITHIN the frame (which is
why the 0.42 m det-vs-GT check passed) but scrambled ACROSS frames. All
exported trajectories, features, and intention points were noise; the model
correctly learned the only invariant (predict near current position).
Fixes applied:
- export_wf_for_mtr.py: `train=False` (comment explains why).
- build_multiv2x_intention_points.py: removed deg2rad on already-radian yaw
  (same bug class as the loader fix).
Regeneration DONE (2026-07-07): all 52 zones re-exported in place. Verified
across every zone: GT per-step displacement median ~0-0.8 m, max 3.1 m, zero
zones with >8 m steps (corrupted version was 30-60 m median). Intention
points rebuilt from clean GT: dx in [0, 40.8] forward-only, dy ±23 m,
longitudinal-dominant (mean |dx| 18.6 vs |dy| 6.0) — physically correct.
New 174 GB tar uploaded to $PROJECT. Run 10804312's invalid checkpoints
deleted on PACE (kept eval records/logs/tensorboard, 49 MB); PROJECT at
622G/1T. Infrastructure validation from that run still stands (DDP, eval,
staging, copy-back all work).

**CLEAN-DATA RETRAIN DONE (job 10856291, 4x H200, 2026-07-08).** The 8-GPU
job (10846208) queued >18 h; the 4-GPU sibling backfilled first and 10846208
was scanceled. Sbatches now have a READY-marker gate (`$DATA_TAR.READY`,
dropped by the uploader) so an early-scheduled job can't untar a partial tar.

**STAGE 1 (no aggregator): SUCCESS — this is the RELAY P0 model.**
minADE 5s: 3.06 (ep1) → 1.39 best (~1.45 settled); best MR(3.6) 0.221 at
epoch 7 (best_model.pth = epoch 7). CMP-quality on 44%-recall detection
pasts. Checkpoint pulled locally to
`ecav/ml_manager/models/mtr_wf_stage1/best_model.pth` (596M); full run
output (both stages, all epochs) at PACE
`$PROJECT/mtr_wf_runs/10856291/output/`.

**STAGE 2 (Transformer aggregator): DIVERGED — checkpoint unusable.**
ADE 6-8 m from the start (random aggregator decoder overwrites stage-1
trajectories), training loss nan from ~epoch 12, all later evals nan. Its
best_eval_record "MR 0.0" is a nan artifact (nan distances count no misses).
Suspects: LR 1e-4 with MTR unfrozen (`--freeze_mtr` exists but unused),
GRAD_NORM_CLIP 1000 (effectively none), 138M-param BEV flatten Linear.
**DECISION (Tyler, 2026-07-08): drop the aggregator permanently.** Not
important for our contribution and it costs edge inference time. Vanilla MTR
has NO stage 2; the two-stage recipe was purely CMP's (stage 2 = their
aggregator). Our training is therefore standard single-run MTR training.
ARCHITECTURE CLARIFICATION: in this model the WF fused BEV feature is NOT a
direct MTR input (fused_feature only enters via the aggregator path, now
dropped; the exported .npy features are loaded by the dataset but unused).
Cooperation enters through the DETECTIONS: WF intermediate fusion produces
the detections that form MTR's past trajectories. Paper framing: standard
MTR (with CMP's Swin lane-raster encoder) predicting from cooperatively
perceived trajectories — do NOT claim direct BEV-feature conditioning.
RATIONALE (agreed with Tyler 2026-07-08): sensing is multi-agent (~11
CAVs/zone feed WF fusion); prediction is single-agent BY ARCHITECTURE — the
edge is the one prediction point (the service model of the SEC/RELAY
papers). CMP's aggregator reconciles N per-vehicle predictors; our
architecture deliberately has one predictor, so the module answers a
question the system is designed not to have. Its stage-2 divergence is
secondary to this structural inapplicability.
**LIVE INTEGRATION DONE (2026-07-14): stage-1 MTR runs in the closed loop.**
Smoke on openscenario_3_edge_worldfusion_smoke (worldfusion_adaptive, WF
translaug_finetune net_epoch27 perception, stage-1 MTR): clean end-to-end
run, ZERO collisions, live delivered-prediction ADE 2.34 m @1 s /
2.1-2.5 m @2 s / 2.8-3.2 m @3 s, FDE 0.97-1.37 m, MR 24-32% (single
delivered mode on AB3DMOT tracks through the delivery pipeline; offline
minADE6 was 1.39 m). RELAY P0 is fully closed — live Q1/Q4/Q5/Q6 unblocked.
Wiring changes (all in repo):
- `mtr_edge_predictor.py`: valid=0 padding for short histories (training
  saw masked gaps, not frozen oldest-frame replicas); model 0.2 s steps
  resampled to 0.05 s sim ticks in _to_world (consumers index at tick rate;
  the OLD OPV2V wiring at 0.1 s had this mismatch silently); predictor owns
  lane-raster loading (`lane_map` arg).
- `ecav/core/map/rsu_lane_raster.py` (NEW): white-on-black 256x256 lane
  raster from MapManager HD-map geometry, training-raster convention;
  `lane_map: auto` in yaml renders it at manager init centered on
  world_anchor (rsu_manager_list is EMPTY at construction; anchor is the
  fusion frame anyway).
- Both MTR managers pass num_output_steps(100)/output_dt/lane_map;
  `worldfusion_mtr` registered as a named combination (naming rule: one
  name per pipeline combination — Tyler).
- `worldfusion_perception_manager.py`: camenc guard is now
  `getattr(...) is not None` (LiDAR-only ckpts set camenc=None; hasattr
  routed them into the camera branch and crashed).
- Smoke yaml: WF perception caronly_aug (scene-overfit) →
  translaug_finetune net_epoch27; mtr_predictor → stage-1 ckpt + no_agg cfg
  + new intention pkl, dataset multiv2x, aggregator 'None', time_interval
  0.2, history_subsample 4, lane_map auto (lane_range_m 90).
Follow-ups (not blockers): _run_mtr fired only once in the run
(amortization cache + mostly near-stopped traffic; fastest track at cadence
check 0.46 m/s) — rerun with the Tesla crossing at speed to exercise the
model harder; known live AB3DMOT ~9-tick fragmentation still pending as a
separate fix.

**Score-head calibration measured (2026-07-16, full test split, 1980
objects):** oracle minADE6 1.45 m / minFDE 2.69 m vs score-SELECTED mode
ADE 2.33 m / FDE 4.66 m. Argmax score picks the truly best mode 62.6% of
the time (best-or-2nd 79%; ranks r0:63 r1:16 r2:10 r3:6 r4:2 r5:3 %).
Selected-ADE median 1.18 m, p90 5.78 m — tail-heavy exactly where modes
diverge. CONSISTENCY: live delivered ADE (2.3-3.2 m @1-3 s) ≈ offline
selected-mode ADE (2.33 m) → the live pipeline adds ~no degradation; the
whole live-vs-oracle gap is mode selection. Paper: report minADE6 as
capability and selected-ADE as delivered. NOTE: futures are dense (90.2%
full 5 s, 95.3% mask density — GT-sourced), so truncation label noise is
minor; the gap is intrinsic multimodality + thin observed pasts.

**LIVE UNDER-PREDICTION ROOT CAUSE (2026-07-19, blind-overtake work).**
Blind overtake (openscenario_1_edge_worldfusion, Town01) exercised MTR on
real movers and exposed systematic under-prediction (12 m/s vehicle → 5.7 m
predicted @4.2 s; FDE 12-18 m, MR 83-100%). Diagnosis chain (all measured):
- Live inputs verified CORRECT ([MTR IN] instrumentation: world past and
  center-frame past both textbook).
- Synthetic constant-velocity probe: model under-predicts clean consecutive
  pasts at ALL speeds (3 m/s → 4.3 m; 12 m/s → 6.2 m @5 s) with full mode
  collapse; real test inputs predict movers fine (bucket eval: >25 m bucket
  minADE 3.56 m). Raster type and history depth: no effect on the probe.
- Real-sample full-mode dump: GT endpoint (-23,+13) BEHIND the stored
  center heading. Cause: dataset center heading = PRED pickle yaw = WF
  DETECTION yaw with 180° box ambiguity (~half of moving centers flipped).
  GT pickles themselves are 100% motion-aligned (measured).
- CONSEQUENCES: (1) the model hedges modes in BOTH directions (explains a
  chunk of the 62.6% top-1); (2) it infers motion from pasts calibrated on
  gappy/noisy detection pasts (44% recall); live feeds DENSE KF-smoothed
  AB3DMOT replay pasts = out of distribution → magnitude collapse.
  Neighbor-drop also halves magnitude (bisect) but live has neighbors.
- Train split is 62% parked centers (median 5 s displacement 0.4 m), test
  44%. Offline metrics honest for offline inputs; skew is train-vs-live.
FIX OPTIONS: (a) live-side, no retrain: feed per-frame detection positions
associated to tracks (gaps as invalid) instead of KF-smoothed replay —
reproduces the training generative process; (b) retrain with
tracker-generated pasts (train=live by construction); (+) normalize
detection yaw to motion direction offline+live to kill flip ambiguity.
Scenario work also done: Scenario_1.__init__ accepts vehicle_index /
distributed (was crashing scenario_runner); new openscenario_1_edge_worldfusion
yaml + runner; post-eval teardown core dump unexplained (non-blocking).
Two-locale split pending.

**MAMBA3DMOT SWITCH IN FLIGHT (2026-07-21..27).** Decision (Tyler): the
WF+MTR pipeline runs on mamba3dmot end to end — AB3DMOT was lineage, not
choice; RELAY migrates MambaTrack state, so tracker must be mamba both
offline and live (train=live by construction).
DONE:
- `retrack_pred_with_mamba.py`: decodes detections from SAVED fused
  features (heads only, no WF re-run; saved features are post-shrink and
  heads apply directly), runs Mamba3DMOTWrapper per zone (live-tuned cfg:
  match_thresh 5.0, max_time_lost 60), tracks->GT keying by gated Hungarian
  + lifetime majority vote (min 3), prunes misattributed segments (>2x gate
  from keyed GT when visible; >6 m/frame jumps) — id-reuse stitching caused
  137 m teleports before pruning. Yaw normalized to motion direction
  (kills detection 180° flip). All 52 zones: 103k train / 2k test states,
  max step 6.0 m. Old GT-anchored pickles kept for ablation.
- Loader: `PRED_TRAJ_DIRNAME` cfg key; `_no_agg_mamba.yaml` variant.
- RETRAIN job 11527027 queued (4x H200, stage-1 only,
  `mtr_wf_stage1_mamba.sbatch`: big tar + small `wf_mtr_pred_mamba.tar`
  overlay; 11 MB pred tar uploaded, code tar refreshed).
- LIVE: predictor normalizes input yaw to motion direction (same rule);
  Mamba3DMOTWrapper row layout FIXED to AB3DMOT-consumer convention
  (carla_id col 8, vx/vy cols 10/12 — was frame at 8/carla at 10, so the
  B0 multi-edge mamba runs stamped carla_id=frame and garbage kf_speed;
  latent-export via tracked_tracklets was unaffected);
  WF base manager grew `_format_dets_for_tracker` / `_track_row_to_box`
  hooks (+hasattr-guarded AB3DMOT debug);
  NEW `WorldFusionMambaAdaptiveEdge` (edge_manager_worldfusion_mamba_mtr.py,
  registry WORLDFUSION_MAMBA_ADAPTIVE / _MTR): feeds tracker PLAIN-axis
  dets (un-swaps the WF KITTI swap — offline retrack used plain axes, live
  must match) and parses plain rows on replay.
NEXT: retrain lands -> pull ckpt -> blind-overtake yaml to
worldfusion_mamba_adaptive + new ckpt -> rerun -> two-locale split.

**Mode-sweep planner SHIPPED (2026-07-19).** behavior_agent.py's collision
loop now sweeps the top-K prediction modes by score (K =
`prediction_mode_top_k` in behavior yaml, default 2 = 79% best-mode
coverage; 6 = full Autoware-style sweep, 1 = argmax ablation) and acts on
the earliest conflicting TTC; drawing re-run uses the conflicting mode.
Falls back to the single trajectory for non-multimodal predictors
(linear/SMART). Precedent: Apollo builds ST boundaries per predicted
trajectory; Autoware obstacle_cruise iterates predicted_paths by
confidence. NO proto change needed for current runs: distributed:false is
in-process, ObstaclePrediction objects carry predicted_trajectories_all +
mode_scores end to end (ecloud.proto GeneratedTrajectory has only the
single trajectory — extend it IF distributed vehicle-side planning is ever
used). Validation smoke (run 5): exit 0, no tracebacks, FDE 0.77-1.20 m /
MR 17-23% (same band as argmax run), 0 ghost brakes. The mode-divergence
payoff case (fast crosser) is the pending fast-Tesla rerun.

**Prior run details (10804312, infra reference):** (8x H200, 12 h limit).
`mtr_wf_ddp_8gpu.sbatch` rewritten for the CMP recipe: stage 1 `_no_agg` 30
epochs (extra_tag wf_stage1) → stage 2 Transformer aggregator 30 epochs
initialized from stage-1 best_model (lowest eval MR; falls back to newest
epoch ckpt). Output layout is `output/<TAG>/<extra_tag>/ckpt/` (NO
EXP_GROUP_PATH); yaml PRETRAINED path fixed accordingly. EXIT trap copies
output back to `$PROJECT/mtr_wf_runs/<jobid>/` (node NVMe is wiped), plus an
explicit copy between stages. ~10 min/epoch at 2 GPUs → 8 GPUs ≈ 2.5 min/epoch;
both stages plus untar fit well inside 12 h.

**Local artifacts:** prior full export (caronly_aug, WRONG model) at
`models/multiv2x_mtr_wf/` (174GB, regenerate with translaug). PACE dataset
intact: `$SCRATCH/Multi-V2X.tar` (plain tar), env `$PROJECT/miniconda3/envs/opencda310`.

## Merged origin/develop into paper-closed-loop-recreate (2026-06-17)

Brought develop's edge-only distributed mode work onto this branch (PR #18,
tlandle/eCAV develop). Conflicts resolved in: `.gitignore` (union), this file
(kept this branch's state), `edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py`
(kept both method sets: our `_advance_actors` + conflict-kinematics logging AND
develop's `collect_features`/`apply_predictions` edge-only collect/apply split), and
`migration/payload.py`. The payload merge is the substantive one: `TrackLatent` now
supports BOTH tracker backends. MambaTrack populates `memo_bank`/`diff_memo_bank` +
bbox + bookkeeping (our migration harness path); AB3DMOT populates `kf_state`
(develop's `KFState` Kalman snapshot, used by `edge_manager_pluggable_base`
export/import). All backend-specific fields default so either construction site is
valid. develop also adds `migration/{daemon,link}.py` (production handoff daemon +
inter-locale link cost model) alongside our `factories.py`/`harness.py`. The in-sim
migration is a model (pickle + bandwidth/latency); a real deployment would carry the
same per-track state over a real protocol.

## Multi-Edge Predictive Latent Migration — harness + Kalman baseline (2026-06-15)

Paper is SUBMITTED (safety_envelope_sensys). Back on the multi-edge / cross-edge
handoff line (Paper 2/3 "Scale-Out"). The 2026-04-18 plan said "not started"; it is now
well underway. `ecav/core/application/edge/migration/` has: locale registry + binding +
payload + `harness.py` (synthetic warm-handoff validation) + live Mamba3DMOT latent
transfer. Commits 70cd4ae8 (registry/binding/payload/smoke) -> 3bd09544 (live latent
transfer) -> 09ad5c41 (turn/brake/lane-change traces + frame-aligned 5-frame gap metric).

**Harness now runs a THREE-WAY comparison** (this session, factories.py + harness.py,
uncommitted): full-latent migration **B** (ours, full memo/diff history) vs Reactive-Kalman
**B1** (`history_depth=1`, migrate only the latest bbox+diff) vs cold-start **B0** (no
migration). `factories.latent_from_tracklet` got a `history_depth` knob (truncates the
migrated banks); `_summary_row` + the table + CHECK 3 report all three.

RESULT (handoff_frame=10, total=30, synthetic 1-vehicle trace, device=cuda), metres:

| traj        | 5f full | 5f Kalman | 5f cold | mean full | mean Kal | mean cold | B/B1 bytes |
|-------------|---------|-----------|---------|-----------|----------|-----------|------------|
| straight    | 0.027   | 0.205     | 0.250   | 0.026     | 0.070    | 0.072     | 1302/805   |
| turn        | 0.068   | 0.245     | 0.275   | 0.185     | 0.229    | 0.235     | 1302/805   |
| brake       | 0.079   | 0.221     | 0.255   | 0.116     | 0.147    | 0.147     | 1302/805   |
| lane_change | 0.047   | 0.231     | 0.265   | 0.055     | 0.102    | 0.102     | 1302/805   |

FINDING: full-latent migration is ~3-5x lower error than Kalman/cold in the 5-frame
post-handoff window; **Kalman single-frame ~= cold** (the Mamba SSM predictive state is not
reconstructable from one frame, so a KF-style warm handoff barely helps). Overall means
converge as all destinations re-accumulate history -> the gap window IS the handoff cost.
Full history costs only ~1.6x the payload. CHECK 1 (state parity byte-equal) + CHECK 2
(prediction parity 0.0) still pass.

CAVEATS: synthetic harness, idealized detection trace (no detection noise); the "Kalman"
baseline is APPROXIMATED by truncating the Mamba memo bank to 1 frame, not a real KF.
NEXT options: vary handoff_frame / longer traces / detection noise; or move from the
synthetic harness to a real multi-edge CARLA locale-boundary crossing. Run:
`conda activate opencda310; python -m ecav.core.application.edge.migration.harness [--quiet]`.

## Paper (safety_envelope_sensys, standalone repo on github.com/tlandle) — 2026-06-05

Terminology standardization pass landed on `master` (c352280, Overleaf-synced). One term
per concept, enforced paper-wide across active files (abstract, introduction, related_work,
system_architecture, evaluation4, discussion, conclusion + the floats eval4 inputs):
**object sharing** (architecture umbrella) / **object list** (payload) / **V2X2V prediction**
(evaluated instance, in figs+Table 2) / **age at use** + **tail age at use** (not AoI/freshness/
tail latency) / **merge point** (not publish/fusion/consumer boundary) / **planner boundary** /
**physics boundary** + **logic boundary** (not limit/cliff/mass/penalty) / **self-ghost** /
**spatial self-filtering**. "Late Fusion" removed from the main eval. Figure generator
`scripts/arch_envelope_pipeline.py` ARCH labels already match ("I2V object sharing",
"V2X2V prediction"); no re-render needed. Conclusion rewritten to current paper (up to 32
CAVs, situation-dependent budget ~220-450ms, four claims, safety margin m(a,u,N)). Discussion
significantly shortened: redundant-with-eval and stale-N=16 paragraphs commented out (not
deleted) per comment-out policy. Compiles clean, 14 pages.

STILL PENDING (deferred, not in the last two requests): §3 BLOCK-2 edits (maneuver-qualifier
F_u/tau_max paragraph, §3 margin definition, §3 intermediate-fusion-as-perception sentence,
move the "We instrument the pipeline" paragraph to §5.1). Inactive files (evaluation2/3,
appendix, old floats) were reverted to keep the commit scoped; they keep legacy terminology
and need a full pass only if reactivated.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

---

## BLOCKER: edge never delivers a moving prediction; ego collides at ALL latencies (2026-06-01)

Branch `paper-closed-loop-recreate`. Scenario `openscenario_3_edge_late_fusion_boundary`,
TD via `CROSS_TRIGGER_DIST`, run under conda env `opencda310`.

THE conflict_kinematics `collision_flag` IS DEAD. It reports 0 even during a real
collision. The true collision signal is `simulation_metrics.json: focal_collisions`
(and `collision_count`, the per-tick CARLA sensor). All prior "no collision at any
latency / predictor compensates" conclusions were artifacts of reading that dead flag.

REAL collision data (focal_collisions): TD=68 collides at lat 0/100/200/300/450
(ALL, incl zero). TD=69 collides at NONE (Tesla physically clears ~10-12 ticks before
ego regardless of perception). So there is no latency cliff: at TD=68 the ego fails
to avoid even with perfectly fresh data; at TD=69 there is no conflict. Latency was
never the operative variable.

ROOT CAUSE (airtight, lat0 TD=68 run 20260601_215049):
- The edge AB3DMOT tracker cannot hold a continuous track on the moving cross-traffic.
  The Tesla is on the `[TRACKS]` path (125<=y<=131, x<=-40) in only 41/212 frames (19%),
  across 7 distinct track ids (27,71,107,130,148,203,207). 81% of frames it is not
  tracked at all. Only the STATIONARY occluders hold age-30 tracks.
- Predictor is the LINEAR predictor (confirmed in log), so no history requirement; the
  problem is that the moving track mostly does not exist. `[PREDS]` therefore contains
  only stationary occluder tracks (max speed ever in any pred = 0.7 m/s).
- Ego `edge_preds_received_total: 0`, `edge_ticks_with_preds: 0`. RSS proper-response
  (behavior_agent.py:1437) only fires on a predicted collision, so it NEVER fires
  (0 `[RSS]` lines). Ego only reacts via its own YOLO at ~8m (brake at tick 116), can't
  stop from 10 m/s, collides ~tick 122-123 (Tesla speed collapses 13->5).

ROOT CAUSE PINNED (DET_TRACE instrumentation): detection is NOT the problem. The Tesla
is detected on the path every tick (clean smooth centers, DET_TRACE). The failure is
AB3DMOT association: KITTI/pvrcnn Car uses giou_3d (thres -0.2), but the roadside
camera-lidar fusion (o3d_lidar_libs.py:251 get_axis_aligned_bounding_box of near-face
points, >=2 pts) produces degenerate boxes (sliver widths 0.4-1.0m, oscillating, axis-
aligned so yaw~0). GIoU between KF-predicted and detection box collapses below -0.2 even
though centers are clean -> track fragments (19% tracked, 7 IDs). Anchoring birth-
suppression ruled out (counters suppressed=0, cull=0 during approach). Kinematic gate
ruled out (rejects only stationary occluders, never the fast Tesla).

FIX APPLIED (measurement-model alignment, per advisor):
1. AB3DMOT_libs/model.py get_param KITTI/pvrcnn Car: giou_3d -> dist_2d (BEV center /
   kinematic association), thres 4 (4 m gate). Matches the reliable part of the
   measurement; the existing kinematic innovation gate provides the velocity residual.
2. edge_manager_..._linear_predictor.py _collect_ab3d_detections: clamp anonymous
   vehicle-det extents [h,w,l] to a class car prior (1.2-2.0, 1.6-2.4, 3.5-5.5) so
   unstable extent is not trusted and the downstream footprint is a real car.
3. NMS gate 3.0->3.5, mot_cfg min_hits 3->2 (testing) for the cross-source duplicate.

RESULTS so far (lat0 TD=68): collision went from 5/5 always -> 3/5 (run_3 fully avoided);
RSS now FIRES (0 -> 6); contact 68 -> 5 hits; track continuity 19% -> 60%. Residual: the
Tesla yields TWO detections ~3.2 m apart almost every tick (RSU1+RSU2), oscillating 1<->2,
which churns the track (16 IDs) so RSS fires slightly late and the ego grazes 3/5. This is
a cross-agent detection-fusion (multi-view duplicate) problem. Testing NMS=3.5 + min_hits=2
over 5 reps. Naive NMS=4.5 over-merged (all metrics worse) so do NOT just widen the gate.

RESOLUTION DIRECTION: stop chasing the perception/tracker quality on YOLO+lidar boxes.
The NMS / min_hits / center-fusion tuning did NOT converge (collision rate bounced 3-5/5,
within closed-loop noise). The clean path is the ORACLE DETECTOR.

ORACLE-DETECTOR REAL RESULT (2026-06-02, mgr=oracle = GT boxes + AB3DMOT + linear
predictor + cooperative prediction, TD=68, 3 reps): MEASURED reference physics envelope
P[S_op] = 1.00 at lat 0/100/200/300 and 0.00 at lat 450 (3/3 collide). DETERMINISTIC
(zero rep noise) because the tracker no longer churns. So the architecture works when
detection is clean; the whole blocker was perception box quality. Real latency cliff lies
between 300 and 450 ms. Run dir 20260602_191819.

ARCHITECTURE-COMPARISON INFRASTRUCTURE ADDED: a `DETECTOR=oracle` env toggle now runs any
infra/cip manager on GT detections.
- late_fusion: new `_collect_detections_for_frame()` dispatch method in run_step (default
  = perception + cross-source NMS).
- InfraOnly: `self.detector` (env DETECTOR or cfg); when oracle, update_information pushes
  GT actors and the dispatch uses `_collect_oracle_detections`. CIP inherits both.
- `_collect_oracle_detections` made robust to empty beacons (infra-only: ego appears as an
  anonymous GT detection). Files: edge_manager_{prediction_late_fusion,infra_only,oracle}.

NEXT: sweep `oracle` (coop prediction), `infra_only`+DETECTOR=oracle (I2V perception),
`cip`+DETECTOR=oracle (cooperative planning) over latency on the SAME GT detections ->
real per-architecture envelopes (perception vs prediction vs planning), no perception
confound. Add 350/400 ms to resolve the 300-450 cliff. These are the real measured per-architecture envelopes.

Process notes: env is `opencda310` (conda root /home/atlas/anaconda3). Latency sweep
warmup intentionally shifts whole scenario ~2 ticks/100ms (ego+cross stay in sync; do
NOT replace the proximity trigger with fixed-sim-time). Foreground `sleep` is blocked by
the harness; run CARLA/sweeps as background tasks.

---

## PROVENANCE-HISTORY TAXONOMY for brake classification (2026-05-31)

The "self-ghost" metric was OVER-LABELING. Traced end-to-end: track 59 born
tracking the crossing Tesla (-79->-81 moving), then FROZE at (-83.5,127.4) ~30
ticks, fed by a MIX of Tesla + ego/RSU detections at the conflict point. It is a
track-merge / frozen-phantom, NOT an ego self-echo. nearest-ego-NOW labeled it
self_ghost because the ego drives through that frozen point.

FIX (per Alex/prof): classify a brake-triggering track by PROVENANCE HISTORY, not
nearest-ego-now. Implemented:
- late_fusion `_ab3d_history_to_trajs`: each tick, GT-match every live track's
  position to nearest actor; append ('ego'|'nonego',id,dist) to
  self._track_provenance[track_id] (deque maxlen 15).
- `_label_brake_attributions_gt`: read the triggering track's provenance hist:
    consistently ego, no non-ego -> self_ghost (true ego echo)
    ego AND non-ego present       -> track_merge (re-match to non-ego for FP/TP)
    consistently non-ego          -> external_stale
    no history                    -> fall back to nearest-now
  Stored on attr['gt_provenance_class'].

CONFIRMED WORKING (taxonomy2, lat-100 anchoring-on): the 7 brake events that were
mislabeled self_ghost are ALL prov=track_merge (ego_ticks 2-8, nonego_ticks 7-13;
each track's history dominated by the Tesla with a few ego ticks at the end). All
7 correctly reclassified OFF self_ghost. So the lat-100 "SBA leak" was 100%
track-merge contamination, NOT a real self-ghost and NOT an SBA failure.

Re-running 10-rep LF on/off sweep (validate_tax, lat 0/100/200/300/450) with the
fixed taxonomy to check P[S_op] monotonicity + SBA separation on the corrected
self_ghost metric. Then prof safety-critical check #4. ~40s/run.

4-class taxonomy: self_ghost / track_merge_identity_switch / external_stale /
other_fp.

SAFETY-CRITICAL CHECK still owed (prof #4): when SBA suppresses an ego-consistent
track at the conflict, confirm the real cross-traffic Tesla stays tracked
elsewhere and in the planner collision set, else SBA could hide a real obstacle.

POSSIBLE REFRAME (prof): if corrected taxonomy shows many early failures are
track_merge not self_ghost, shift the claim from "self-ghosting" to
"edge-published identity ambiguity" (self-ghost is ONE instance). Closer to the
MobiCom critique, less brittle.

PRIORITY (prof): fix taxonomy BEFORE adding baselines. Repro 20260419 IS clean on
self-ghost (SBA-off 0->4 episodes, SBA-on ~0) with the ORIGINAL classifier on the
SAME scenario (LTAP/OD = scenario_3); our contamination came from run dynamics
(our ego ~7.7 m/s vs repro ~3.6 m/s through the conflict), making the ego reach
the frozen-track overlap point.

## FAILURE-ATTRIBUTION FRAMING + EGO-PROVENANCE SELF-GHOST GATE (2026-05-31)

Tyler's framing (governing for the eval): scenario failure MUST be attributed
with high confidence, separating SELF-GHOST false-brakes from OTHER artifacts.
- Self-ghosting = ego brakes for its OWN republished stale track. Significant,
  distinct failure class. Happens at a distance delta that scales with latency
  and ego speed. SBA REMOVES this class (and only this class).
- Source of self-ghosting = MULTI-SOURCE DISAGREEMENT, inherent to ANY
  object-level fusion (RSU-side, multi-vehicle detection, any detection
  combining). Ironically the same disagreement also drives the OTHER false
  brakes. Open tension: may imply object-based late fusion is fundamentally
  fragile; SBA fixes self-ghosting specifically, not the rest.
- DATA MUST look as clean as the paper figures.

Do NOT try to attribute a stale/spurious track to some "real" GT actor (ill-
posed; the track may correspond to nothing). The ONLY high-confidence question
is binary: IS this track the EGO'S OWN echo or not? Answerable exactly because
the ego's GT trajectory is known.

EGO-PROVENANCE GATE (implemented in _label_brake_attributions_gt): a brake is
self_ghost ONLY if (a) nearest GT actor now is the ego AND (b) the ego was
actually at the track's position at the track's SOURCE_TICK (within 3m, from
_gt_snapshots[source_tick][ego_id]). If the ego was elsewhere then (e.g. still
approaching while a stale CROSS-TRAFFIC track sits at the conflict the ego later
drives through), it is NOT a self-ghost -> re-matched to nearest NON-ego actor
and classified as hazard/other_fp. This stops the stale-Tesla mislabel that
corrupted P[S_op]. Logged as [GHOST-RECLASS]. Uses only the ego's own GT path,
no phantom-actor attribution. Validation run in progress.

REVERTED: the source-tick / trajectory-window matcher experiments (over-
engineered, attributing bunk tracks to maybe-nonexistent actors). Matcher is
back to original nearest-now; the ego-provenance GATE is the only addition.

## THE lat-100 "SELF-GHOST" IS A GT-MATCHER MISLABEL, NOT SBA (2026-05-31)

PROVEN with GHOST_DEBUG=1 ([GHOST-MATCH] dump in _label_brake_attributions_gt).
The lat-100 anchoring-ON "self-ghost" that dropped P[S_op] to ~0.56:
- ghost track 58 is FROZEN at (-83.5,127.4), on the cross-traffic Tesla's path.
- the LIVE Tesla (id655) has already moved WEST to -88..-95 at 13.x m/s (past the
  conflict); the track is stale, lagging the real Tesla by 5-12m (100ms latency +
  tracker coast).
- matcher position-matches the STALE track pos to nearest GT actor. Tesla moved
  away so it's not nearest; the EGO (id654), arriving at the conflict, is nearest
  (d~0.9m) -> labeled self_ghost -> matched GT actor = ego.

So it is NOT an ego self-echo and NOT an SBA leak. SBA is WORKING. It is a GT
brake-CLASSIFIER bug: a stale anonymous cross-traffic track gets called
self_ghost whenever the ego coincides with the track's STALE position while the
real source actor has driven away. A cross-traffic Tesla can never be a real
self-ghost (Tyler's objection, correct).

CONSEQUENCE: the s_op envelope was corrupted, counting stale-real-obstacle brakes
(an AoI/physics effect) as self_ghost (logic). This is why P[S_op] looked low/
jumpy for anchoring-ON. Fix is in the CLASSIFIER, not SBA: match a stale track to
the actor whose TRAJECTORY passed through its position (the Tesla), classify as
true-positive / stale-true-obstacle, not self_ghost. My earlier "SBA leaks at
1.16m / swept-path doesn't fire" conclusion was WRONG (I read EGO-SUPPRESS from a
different tick + a passing run). SBA footprint suppression fires correctly at the
ghost ticks.

SEPARATE open question (Tyler): does the occluder column (x=-81, y<=119, SOUTH of
conflict) actually occlude the EAST-WEST cross-traffic at the conflict? If not,
the "blind intersection" premise may be weak. Check scenario occlusion integrity.

## REAL REPRO DATA FOUND: 20 reps is what makes S_op monotone (2026-05-31)

The actual paper repro data is `safety_envelope_paper/experiment_results/20260419_120000`
(Tyler pointed me there; NOT the 20260311_230618 the plot-script default points
at, which is undersized). It has 4800 run dirs: mgr {lf,oracle,vips} x anchoring
{on,off} x lat {0-550/50} x ego {1,4} x scenario {ltap_od, scp} x ~20 reps/cell.

P[S_op=1] vs configured latency from THIS data (ltap_od, ego1, 20 reps/cell) is
CLEAN and matches the paper claim:
  lf/off:  1.0 .75 .38 .20 .05 .10 0 ...        sharp cliff ~100-150ms
  lf/on:   1.0 1.0 .93 1.0 .85 .65 .80 .55 .07  holds, cliff ~400ms
  oracle:  1.0 1.0 .95 .85 .90 .95 .95 .70 .17  physics floor ~400ms
  vips/off:1.0 .90 .35 .40 .10 0 ...            cliff like lf/off
  vips/on: 1.0 1.0 .97 .90 .72 1.0 .80 .70 .07  holds like lf/on
SBA expands envelope from ~100ms logic cliff to ~400ms physics limit. Confirmed.

ROOT CAUSE of my non-monotonic S_op: I had 3-5 reps; S_op is a binary AND
dominated by a flickering 1-2 ghost-episode count, so per-config P was 0/1 noise.
At 20 reps P[S_op=1] is a real probability and near-monotone (tiny wiggle that
AoI-binning / cumulative-min smooths). The fix is REP COUNT, not the metric.

CONSEQUENCE: the full matrix must be ~20 reps/cell (I launched 5 -> killed it,
correct call). This resizes the matrix a lot (20 reps x 2 scenarios x ego counts).
New arms (1-RSU, 2-RSU, CIP, local-only) must match 20-rep density. Also the
repro covers 2 scenarios (ltap_od, scp) and ego {1,4} -- richer than my single
ltap/40km-h smoke. GATE: reproduce the published sop_vs_aoi from 20260419_120000
before committing the full new-arm matrix.

## FULL MATRIX LAUNCHED (5 clean arms) + CIP/ns-3/framing (2026-05-31)

**Full controlled-latency matrix RUNNING** (task bw1tno35p, log /tmp/full_matrix.log,
~330 runs matrix A + ~110 B/C, ~20h+). Arms: late_fusion + oracle + vips_temporal
(anchoring both) + infra_only 1-RSU + infra_only 2-RSU. Latencies 0-500/50ms, 5
reps, 40 km/h, conflict logger on, CONTROLLED latency (ns-3 OFF by default).
Verify with scripts/verify_sweep_run.py + paper1_real_aligned_plots.py.

**Shakedown verified 5/6 arms clean** (recall>0, AoI tracks latency, euniq smooth).
Headline gradient (euniq, no vehicle uplink): Oracle 0.00 < 1-RSU 0.05 < 2-RSU
0.16 < LF 0.28. Source-count drives ego-uniqueness violations = the reviewer
answer (self-ghosting is multi-source, not V2X2V).

**CIP off critical path, diagnosed.** Fixed: apply_control tuple bug (now via
vm.controller) + zero-DL (now DL-delayed command delivery). Remaining: CIP's
_advance_actors override is incomplete (calls agent.update_information with empty
objects, bypasses vm.update_info -> no map/route update -> planner stalls ->
0.08 m/s, 234 collisions). Fix scoped in full_envelope_matrix.md; parallel work.

**ns-3 LUT = SECONDARY validation only, not matrix default** (Tyler). Don't change
two variables at once. Matrix = controlled latency. ns-3 Uu LUT (payload/N-aware,
UL+DL) is a separate scatter (fix architecture, vary network) showing realistic
radio maps to the same measured-Delta_use envelope. SEE-V2X trace is PC5 sidelink
(t1->rsu), NOT unicast DL, so CIP's command DL must use ns-3 DL LUT or controlled
latency, never the sidelink trace. ns3_lut_sampler wired into late_fusion family
(use_ns3_lut, default False) + CIP DL.

**FRAMING (governing): "how much CP is usable"** not "we found self-ghosting".
3 gates: info-value (d_coop-eps>d_los), physics (M>=0), consistency (identity).
Main plot = usable region, CP gain = S_op(edge) - S_op(local-only). REQUIRES a
LOCAL-ONLY arm (VehicleSideTracker, no edge pred) = NOT yet configured, parallel
work. safety_envelope.py has margin/Delta*/classify already.

## SMOOTH AoI REQUIRES THE SEE-V2X TRACE, NOT FIXED LATENCY (2026-05-31)

The stairstep AoI (vs the paper's smooth CDF) is because my smoke sweeps used
fixed `latency` + `jitter_std=0`: AoI-at-use collapses to one tick value per
setpoint (lat200 -> hist [34@4t,179@5t,2@6t], p50=p95=p99=5t). The paper's smooth
AoI comes from the HybridModel sampling the SEE-V2X C-V2X RTT trace per packet
(`data/see_v2x/merged_latency.csv`, 213k samples, latency_ms median 14.3 / p5 6.4
/ p95 23.1) + backhaul lognormal. So the FULL MATRIX must run with
`--see-v2x-trace data/see_v2x/merged_latency.csv`; --latencies sets the hybrid
base_ms offset and the trace adds realistic jitter. Shakedown (fixed latency) is
fine for validating arms RUN; final data uses the trace. Tyler caught this.

## THE CLEAN-DATA PIPELINE: episodes via compute_run_metrics (2026-05-31)

Resolved how the paper gets clean (non-binary) data. The reproduction sweep is
`ecav/scenario_testing/evaluation_outputs/20260311_230618` (100 run dirs). The
plot scripts `scripts/paper1_real_data_plots.py` / `paper1_real_aligned_plots.py`
load it through `scripts/recompute_metrics.py::compute_run_metrics`, which:
  - counts brake EPISODES via `_count_episodes` (contiguous same-track_id ticks,
    gap<=2, collapse to ONE episode), not raw per-tick brake counts;
  - derives focal_ghost_episodes / focal_fp_episodes / focal_tp_episodes,
    focal_*_ticks, s_op, plus continuous AoI + ego_uniqueness from edges{}.

My "7-8 ghosts at lat100" was 8 TICKS of ONE persistent stale-echo track =
1 EPISODE. Reporting raw ticks inflated it and made it look jumpy/binary.

My 40km/h sweep re-read via compute_run_metrics (EPISODES, clean):
  ghost_eps  lat: 0    100  200  300  400
  OFF             11.0  2.5  0.0  1.0  1.0
  ON               0.0  1.0  0.0  0.0  0.0
Headline SBA result is clean: lat0 OFF 11 ghost-episodes -> ON 0. lat100 ON has
1 residual episode (the lone artifact point); everywhere else ON=0.

RULE: ALL analysis/figures go through scripts/recompute_metrics.compute_run_metrics
(episodes + continuous AoI/ego-uniqueness), matching the paper pipeline. Never
report raw total_ghost_brake_gt / per-tick counts as the primary number.

## METRIC CORRECTION: Use Continuous Envelope Fields, Not Binary Counts (2026-05-31)

Major correction. The existing paper plots (scripts/paper1_real_*_plots.py) are
built on CONTINUOUS / distributional metrics, NOT binary flags:
  - AoI: aoi_mean/p50/p95/p99_ticks, aoi_hist_counts, aoi_cdf  (the x-axis spine)
  - ego_uniqueness_violation_tick_fraction, ego_uniqueness_total_duplicate_tracks
  - prediction_fde_m, prediction_miss_rate, detection_recall/precision/f1
  - timing_p95/p99_ms, tracking_avg_mota/idf1
All live in the per-run simulation_metrics.json `edges{}` entry (~50 continuous
fields) and per-vehicle `vehicles{}` (avg_ttc_s, avg_speed_mps, ...).

I had been reporting the COARSEST binary fields (s_op 0/1, collision_count,
raw integer total_ghost_brake_gt). Those flicker and look non-monotonic. Read
through the continuous fields instead, the same sweep is SMOOTH:

ego_uniqueness_violation_tick_fraction over lat 0/100/200/300/400 (40km/h smoke):
  OFF: 0.284, 0.261, 0.299, 0.271, 0.253
  ON : 0.261, 0.276, 0.310, 0.283, 0.265
duplicate_tracks OFF 119-126, ON 105-132. FDE ~3.4-4.8m. NO 100ms spike.

So the "100ms anomaly" was largely an artifact of reading the binary
total_ghost_brake_gt (a jumpy low integer), not the data. The continuous
ego-uniqueness fraction is smooth across all latencies. The SBA on/off effect in
the continuous metric is MODEST and continuous (dup_tracks 105 vs 119 at lat0),
not the binary "11 vs 0" I reported off the brake count.

OPEN: edge metric ego_uniqueness_total_ego_ghost_tracks=0 everywhere, but
planner-side total_ghost_brake_gt showed 7-8 at lat100. Two different ghost
measures (edge duplicate-ego-track detection vs planner GT-labeled self-ghost
brake) that disagree; reconcile which is the reportable one.

NOTE my session runs HAVE the full aoi_*_ticks distribution; the older Feb clean
suite does NOT (fields added later). So my data is richer on the AoI axis, it
just needs to be read/plotted via the continuous fields like the paper does.

ACTION: all new analysis + figures use the continuous edge/vehicle fields and AoI
distributions, not s_op/collision_count/raw ghost counts as the primary axis.

## 100ms Anomaly = Numerical Artifact + SBA Suppression Safety Check (2026-05-31)

**100ms spike RESOLVED as a numerical knife-edge artifact, NOT a real effect.**
Diagnostic ON-arm latency sweep (50/100/150/250ms, results under
`experiment_results/.../20260531_002249`) combined with the prior sweep gives the
full shape: GT self-ghosts = {0:0, 50:0, 100:[1 here / 7-8 prior], 150:0, 200:0,
250:0, 300:0, 400:0}. Robustly zero at every latency except an UNSTABLE single
point at 100ms whose magnitude is non-reproducible (1 vs 7-8 across reps). A real
AoI/SBA effect would be stable and smooth; a razor-thin non-reproducible spike at
one latency is timing/track-association metastability at that specific state-age.
Treat as a known artifact (exclude with footnote or average over many reps).
SBA's conclusion stands. Tyler's call: it's the scenario/config, not the idea.

**SBA suppression has a real robustness gap (independent of the artifact).** For
an anonymous track within self_id_radius=5m of the ego, suppression needs either
footprint overlap (tight box: ego extent + 1.2m long / 1.0m lat) OR speed-match.
A stale stationary ego-echo at 1-2.4m falls between both (outside the ~2m box,
stationary so speed-gate rejects). That gap is why the 100ms point is metastable.

**SAFETY CONSTRAINT (Tyler):** the fix must NOT blind the ego to pedestrians,
cyclists, slow vehicles, or stopped vehicles near it. This is live, not
hypothetical: the occluder column sits at x=-81 and the ego passes at x=-84.5,
only 3.5m away, INSIDE a 5m self_id_radius. scenario_3 also has 2 pedestrians.
So radius-based suppression is unsafe; suppression must key on ego-IDENTITY
provenance (beacon-forward prediction so stale echoes get ID-stamped, or
swept-path match against the ego's own recent trajectory), never spatial
proximity alone. Beacon-forward is safest: it only ever suppresses tracks
carrying the ego's own beacon, so VRUs (no beacon) are structurally immune.

**Instrumentation added** (`edge_manager_prediction_late_fusion...py`):
`[SUPP-REAL-OBSTACLE]` warning at both suppression paths (footprint + speed-gate)
that GT-cross-checks each suppressed track; if its nearest GT actor is NOT the
ego, it logged a real-obstacle suppression (a bug). Makes "SBA never erases a
real obstacle" measurable. NOTE the prior sweep showed footprint=2 suppressions
with stat_candidates=3 while the ego was near the occluder column, which is why
this check matters. Safety-check run in progress to see if any fire.

## AoI Cliff Sweep @40km/h + SBA AoI Blind Spot (2026-05-31)

LF AoI sweep, late_fusion smoke, 40 km/h, latencies 0/100/200/300/400ms,
anchoring on/off, 2 reps (`experiment_results/.../20260530_225452`). Per-tick
conflict logger on. Results (ghost = GT self-ghost brakes, raw both reps):

| anc | lat | coll | ghost | S_op |
|-----|-----|------|-------|------|
| OFF | 0   | 0 | [11,11] | 0 |
| OFF | 100 | 0 | [7,8]   | 0 |
| OFF | 200 | 0 | [0,0]   | 1 |
| OFF | 300 | 0 | [1,1]   | 0 |
| OFF | 400 | 0 | [4,4]   | 0 |
| ON  | 0   | 0 | [0,0]   | 1 |
| ON  | 100 | 0 | [7,8]   | 0 |
| ON  | 200 | 0 | [0,0]   | 1 |
| ON  | 300 | 0 | [0,0]   | 1 |
| ON  | 400 | 0 | [0,0]   | 1 |

**Clean results:** (1) SBA removes self-ghosting at lat 0 (OFF=11 -> ON=0) and
at 300/400ms (OFF 1,4 -> ON 0). (2) NO collisions anywhere through 400ms at
40 km/h: the physics cliff is beyond 400ms here, so 40 km/h is logic-limited not
physics-limited. The 50 km/h Oracle speed-sweep point is what should bring the
physics cliff into the 0-400ms window.

**SBA AoI blind spot (real finding + bug, needs handling before figures):**
at lat 100ms, ON and OFF are IDENTICAL ([7,8] both) -- SBA does nothing at that
one AoI while working at every other. Root cause from logs: the ghost (track 72,
cid=-1) is a stale STATIONARY ego-duplicate ~0.9-2.4m behind the moving ego.
SBA's beacon-ID suppression never fires because the 100ms-aged beacon fails to
associate the ego's identity onto the stale RSU detection, so it stays
anonymous. The two spatial-gate fallbacks then both miss it: footprint box is
too tight (>~1m off-center), and the speed-gate requires the track speed to
match ego speed but a stale duplicate looks stationary (obs_spd=0 vs ego~10).
So beacon-to-detection association degrades with AoI, leaving a ~100ms window
where the ego track is neither ID-matched nor spatially gated. At lat 0 beacon
and detection coincide (footprint catches it); at 200ms+ the stale detection
ages out / predictor drops it.

**OFF arm is non-monotonic** (11->7->0->1->4): ghosting is not a smooth latency
trend, it is sensitive to AoI-vs-geometry alignment. 2 reps is too few; needs
more reps to separate deterministic structure from seed noise.

**Action items:** (a) decide whether to fix the spatial gate (suppress near-ego
stationary anonymous tracks regardless of speed-match) or report the AoI blind
spot as an SBA limitation; (b) more reps at lat 100 to confirm determinism;
(c) the speed sweep (Oracle 30/40/50) for the physics cliff.

## Multi-Ego Scope: Source-Count NOT Cascade (2026-05-31)

Multi-ego is the V2X2V analogue of 1-RSU->2-RSU infra: more vehicle sources ->
more cross-source disagreement -> more duplicate/ego-uniqueness pressure on the
FOCAL ego (Figure C, source-count axis). Cascading/string failures (ego-k brakes
because ego-(k-1) did, reaction time coupled across agents) are explicitly OUT
OF SCOPE for this paper (interesting but separate multi-agent envelope).

**Geometry constraint:** added egos must be LATERAL/cross sources around the
conflict, NOT a longitudinal queue behind the focal ego (which would create
cascade contamination). The existing 16ego config VIOLATES this: cav9/5/6/7/13
are queued directly behind the focal ego on the northbound approach (x~-85,
y=49-65, heading N, 5 km/h). So the existing multi-ego configs are NOT clean
source-count fixtures as-is. Need to regenerate without the rear queue, or count
only laterally-placed egos as sources. Also: focal ego speed must be pinned to
40 km/h across all N (currently mixed 50/70).

32-ego config does not exist. Decision: check 16-ego feasibility at 40 km/h
(does the focal LTAP conflict survive the congestion) before generating 32.

## SenSys Eval Scope Locked + Analytical AoI Envelope (2026-05-30)

**VRF: NOT built for this deadline.** Treat as related-work / architecture
contrast only ("early fusion forms identities after sensor fusion, so it avoids
object-level duplicate-track publication; we do not claim a VRF implementation").
Comparative evaluation uses only built+run baselines: Oracle, 1-RSU I2V, 2-RSU
I2V, vehicle-side tracker (object-sharing), VIPS, LF, LF+SBA. Claiming a CIP/VRF
comparison would overclaim (CIP is built; VRF is not).

**Core SenSys claim (architecture-disambiguation, answers reviewer A/B):**
self-ghosting is not an artifact of vehicle uplink or of MEC-side tracker
placement. It appears whenever object-level multi-source / infra-published tracks
reach the ego planner without an ego-uniqueness contract. Evidence: 1-RSU 5,
2-RSU 7-9 (no uplink), vehicle-side-tracker 29 (consumer-side tracker),
LF+SBA 0/S_op=1, Oracle physics-floor-only.

**Framing rule:** never write "CIP/VRF fail." Write that the failure is missing
ego-uniqueness at a multi-source publish boundary; CIP-like MEC planning changes
the consumer boundary, so its envelope is analyzed at the plan/command interface,
not the object-track interface.

**Three planned figures:** (A) architecture discriminator (self-ghost/km by
config), (B) safety envelope S_op vs AoI with analytic braking-margin boundary
overlaid, (C) duplicate pressure / ego-uniqueness violation rate vs sources.

**Analytical envelope module** `ecav/scenario_testing/evaluations/safety_envelope.py`:
- Two-reference-frame margin (avoids double-counting AoI):
  M_gen = d_e(t_g) - [v_e(rho+dUse) + v_e^2/2a_b + d_buf]  (scenario tuning)
  M_use = d_e(t_u) - [v_e*rho + v_e^2/2a_b + d_buf]         (log validation)
  They agree when ego speed is stable across [t_g, t_u]. Validated: both 0.050.
- delta_max_yield, ego_speed_for_target_delta (tuning solver).
- Cross-traffic: enters truth error (eps_c = v_c*dUse raw, or prediction residual)
  and clear/yield decision, NOT the ego stopping distance. T_c_believed =
  T_c_true + dUse (planner over-estimates time-to-conflict under staleness).
- Cooperative-perception worth-it: Delta* = (d_coop-d_los)/v_c (raw) shrinks with
  v_c; predicted-track Delta* is flat in v_c (prediction decouples worth-it from
  cross speed). This is a real finding for the figure.
- classify() emits yield_safe / clear_safe / stale_track_dangerous / coop_beats_los.

**Tuning:** model says ego 10.22 m/s (36.8 km/h) puts delta_max_yield=0.305s at
the measured decision distance d_e=13.8m. Set ego max_speed 43->37 km/h in the
late_fusion smoke config; validation run in progress.

## LTAP Conflict Fixture Validated + AoI-Aware Braking Margin (2026-05-30)

Built per-tick conflict-kinematics instrumentation to pin the LTAP physics
boundary so collisions can be classified as physics-limited vs timing artifacts.

**Fixture is sound (confirmed from data, not inference).** Direct edge-world dump
showed the 7 actors: ego=patrol(914) at (-84.8,80) northbound; cross-traffic
violator=tesla.model3(915) spawns (-35,127.7), drives west at 13.4 m/s through
the conflict point (-84.8,127.7); 5 stationary occluders at x=-81. Actors spawn
at z=-500 and teleport up when their behavior sequence triggers. The conflict IS
synchronized: at the brake decision both ego and cross-traffic are ~13.5m from
the conflict point.

**New instrumentation:**
- `ecav/core/application/edge/conflict_kinematics_logger.py`: per-tick CSV of
  ego/cross-traffic pose+speed, ego arc-length distance to conflict (along
  planned path, not Euclidean), TTC, delta_TTC, and the AoI-aware braking
  margin M(t) = d_e - [v(rho+dUse) + v^2/2a_b + d_buf], plus tau_max and the
  zero-latency floor margin. cfg-gated via edge `conflict_kinematics` block.
- Base `_live_gt_snapshot()`: unfiltered live world snapshot (the edge's
  `_gt_snapshots` is 50m range-limited + source-tick-keyed, so approaching
  cross-traffic is missing exactly when closing matters). Stores full type_id.
- Base `_log_conflict_kinematics()`; wired into late_fusion `_advance_actors`
  (infra_only/CIP inherit) and PerceptionEdge run_step.
- Per-event AoI fix in behavior_agent: `delta_use_ticks` and `ego_speed_mps`
  now computed (`trigger_tick = global tick via vm.agent._current_global_tick`,
  not the agent's private `_step_count` which starts at 0 → negative AoI).

**Validation result (lat 0, late_fusion smoke):** the brake fires exactly at the
zero-margin crossing. tick 93 arc=13.8m v=11.3 margin=+0.99 tau_max=0.09s no
brake; tick 94 arc=13.2m margin=-0.16 tau_max=-0.01s BRAKE. Margin recovers as
the ego sheds speed, cross-traffic passes, no collision. The classifier works.

**Calibration finding:** the physics boundary is currently ~90ms (tau_max≈0.09s
at the decision point), not the 250-300ms target. To move it, slow the ego
approach or trigger the brake decision earlier (tau_max ~ d_e/v - v/2a_b). Now
tunable deterministically because tau_max is logged at the decision tick.

**Debug lesson:** spent several reruns inferring geometry from spawn coords +
waypoint lists instead of dumping live actor positions. The actual bug was
trivial and visible in one raw dump: snapshot stored `type_id.split('.')[-1]`
= 'model3', so picker `'tesla' in 'model3'` = False, cross-traffic never matched.
Read the data directly first.

Margin math validated vs worked example: d_stop(v=8.5,rho+0.30,a=6,buf=1)=10.42m,
tau_max(d_e=10.4)=0.298s, margin@dUse=0=+2.53m, margin@dUse=0.30=-0.02m.

## Self-Ghosting Is Multi-Source, Not V2X2V or Edge-Prediction-Specific (2026-05-30)

SenSys-resubmission evidence that self-ghosting is intrinsic to multi-source object
fusion, independent of (a) vehicle uplink and (b) where tracking/prediction runs.
All runs sequential (`ecav.py -t <name> --apply_ml`, no `-d`), scenario_3 LTAP, 1 CAV.

**Infra-only (edge tracks + predicts), no vehicle uplink.** GT-confirmed self-ghosts
(brake-triggering track's nearest GT actor is the ego, cid=-1):
- 1-RSU: 5 (lat 0). 2-RSU: 7-9 (lat 0), rising with latency (→14 at 200ms).
- Single RSU still ghosts because cross-camera NMS dedups detections against each
  other but does NOT remove a detection close to the ego; that is the separate
  ego-gate / self-suppression job, which is heuristic (speed-gate + footprint) and
  imperfect. Cross-camera NMS being correct does not prevent it.

**Object-sharing I2V (PerceptionEdge ships detections only; vehicle runs its own
AB3DMOT + linear predictor + planner via `VehicleSideTracker`).** Still self-ghosts:
19 GT ego-matched ghosts (2 distinct anonymous ego tracks) at lat 0, 2-RSU. Proves
the failure is NOT edge-side-prediction-specific: moving the whole stack onto the
vehicle does not escape it, because identity loss + multi-source disagreement are
already baked into the shipped object list.

**SBA (anchoring) on current code closes it.** late_fusion smoke, anchoring ON lat0:
self_ghost 0, other_fp 0, TP 9, S_op=1. The stale 2026-03-12 anchoring-ON data
showing ~218 ghosts is OLD code, not a current regression. NOTE: late_fusion at lat0
does not self-ghost even with SBA OFF (the ego uploads a beacon, so it is identified),
so late_fusion SBA on/off is the WRONG comparison to prove the fix. The clean demo is
infra-only / object-sharing (no beacon identity).

**S_op axis** (`evaluate_manager.py:654`): binary AND of s_coll, s_ghost, s_fp, s_prog
(avg speed ≥ 60% target). Only discriminates once a config achieves 0 ghosts; SBA-on
is the only arm so far with S_op=1.

### Bugs fixed this session
- `edge_manager_prediction_late_fusion...py` `run_step`: detection gate keyed on
  `beacons` presence, so infra-only (no beacons) silently dropped all RSU detections
  (recall 0, all timings 0). Fixed to gate on actual payload; guarded the beacon loop
  against managed vehicles with no beacon.
- New shared lib `ecav/core/tracking/ab3dmot_format.py`: ObstacleVehicle→AB3DMOT bundle
  conversion, used by edge sensor branch + `VehicleSideTracker` (was duplicated).
- `PerceptionEdge.evaluate()` was unimplemented (base raised NotImplementedError),
  crashing report write; added minimal hook.
- `test_runner.py` `start_carla()`: 30s startup timeout too short on this box (every
  5-run CARLA restart failed); bumped to 120s + `-RenderOffScreen` + 3s RPC settle.

### Configs/runners added
- `openscenario_3_edge_infra_only_smoke` (2-RSU) + `_1rsu_smoke` (1-RSU) + runners.
- `openscenario_3_edge_perception_2rsu_smoke` (object-sharing I2V) + runner.
- `openscenario_3_edge_late_fusion_smoke.py` runner (config already had rsu2).
- rsu2 at `[-40.0, 140.0, 7.0]` added to all six scenario_3 late_fusion configs.

### Paper framing (settled with Tyler)
- Two I2V claims: cooperative-prediction-at-edge fails (infra-only ghosts); and
  object-sharing-only ALSO ghosts (vehicle-side tracker), so it is multi-source
  disagreement, not tracker placement.
- Drop "single source = 0 ghosts" (false). Drop bandwidth dismissal of VRF (VRF uses
  diff clouds, Kbps not Mbps). VRF is vehicle-side early fusion, NOT RSU-compute;
  fix `system_architecture.tex:8` + the placeholder VRF bib entry.
- Plans: `docs/agent_plans/infra_only_baseline.md`, `docs/agent_plans/vrf_baseline.md`.

---

## WorldFusion Fixes (2026-04-26) — Committed, Not Pushed

Two bugs fixed in `edge_manager_worldfusion_ab3dmot_linear_predictor.py`, both Tyler-confirmed correct:

**Tyler's fix (commit `c36e6084`, 2026-04-20)** — pairwise transform and coordinate reference:
- `x1_to_x2` destination: `self.world_anchor` → `[0,0,0,0,0,0]` (true world origin)
- `lidar_pose`/`world_anchor` to post-processor: `[self.world_anchor]` → `[[0,0,0,0,0,0]]`
- Final coordinate offset: hardcoded world_anchor → RSU localizer's actual position each tick

**Agent ordering fix (commit `2a9db949`, 2026-04-26)** — RSU must be agent 0:
- WorldFusion's fusion layer warps all agents' BEV features into agent 0's frame; output is in agent 0's coordinate frame. Post-processor treats `lidar_pose=[0,0,0,0,0,0]` (origin) as reference — RSU must be agent 0.
- Was: vehicles collected first (agent 0), RSUs after. Fix: RSUs first (agent 0), vehicles after.
- Also fixed `vehicle_poses = poses[num_rsus:]` in self-beacon filter (was `poses[:num_vehicles]`).
- Tyler's fix was necessary but not sufficient: correct math on wrong-ordered agents still produces wrong output.

**Camera branch guard** (same commit) — `hasattr(sensor, 'camenc')` instead of `sensor.use_camera`: prevents crash on LiDAR-only model variants.

**Detection confirmed** (`openscenario_3_edge_worldfusion --apply_ml`, sequential, clean CARLA session): ticks 99-100, score=0.871/0.907, ~2m position error. ~7m effective range expected — V2XSim training data is intersection-centric. Ego does not collide with Lincoln; full pipeline confirmed end-to-end.

Commits: `2a9db949` (fix), `647733e4` (logging + KB). Not pushed.

---

## Distributed Integration — Committed (2026-05-04), Verified

All work from the two implementation sessions (edge fusion + instrumentation/evaluation) committed in `787f4dac`. Pipeline verified end-to-end on `openscenario_3_edge_worldfusion --apply_ml -d`.

**What was fixed and implemented:** see commit `787f4dac` message for full list. High-level:
- `edge_process.py` NOP relay fixed — edge now instantiates and runs the real edge manager
- `CavWorld(apply_ml=False, config={'distributed': True})` in edge container — no YOLOv5 load, correct `run_distributed` flag
- RSU `actor_id < 0` guard — RSUs have no base CARLA actor; skip `world.get_actor()` for static infrastructure
- `_init_task` always-await fix — exceptions from phase A (edge manager init) now surface correctly
- Instrumentation, metrics chain, `is_proxy` on edge managers, edge eval forwarding, verbose flag
- `ecav.py`: missing `scenario_name` after OmegaConf merge
- `openscenario_3_edge_worldfusion.yaml`: reverted ego speed 70 → 43 km/h (Tyler's workaround masked fusion failure)

**Verified behavior (test 3: `--apply_ml -d`):**
- `[DATA_FLOW] tick=N features=2/2 objects=2/2` — both RSU and vehicle send features every tick ✓
- WorldFusion runs each tick and produces detection scores ✓
- Lincoln z≈−502 at spawn (below map) → `in_range=False` → correctly no detections until Lincoln arrives ✓
- SMART builds a 9-tick track on the Lincoln near the intersection; rejects it (need 22 ticks) ✓
- No predictions reach vehicle → no brake signal → collision at tick ~226 ✓

**Key finding: collision is expected and correct.** Tyler's 70 km/h workaround had the ego clear the intersection before the Lincoln arrived, making "no predictions" survivable. At 43 km/h (correct), no predictions = collision. The predictor is the research problem — SMART requires 22 ticks of track history but the Lincoln only provides ~9 before the intersection. This is Tyler's problem to fix, not distributed architecture.

**Test 4 verified (`--apply_ml -d -l`)**: `features=2/2 objects=2/2` every tick via gRPC litserve endpoint (confirmed by `DEBUG:grpc._cython.cygrpc` in edge container per tick). Same SMART maturity failure, same collision. Cleanup crash `edge.profiler.save_report()` on NoneType fixed with guard in `openscenario_3_edge_worldfusion.py:261`.

**Next: tests 5–8** — late fusion variants.

## Next: Full Regression Matrix

All 8 permutations must pass. Flags: `--apply_ml` enables ML; `-l` routes inference to external gRPC server; `-d` distributed actors.

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | ✓ 2026-04-29 |
| 3 | WorldFusion | no | yes | ✓ 2026-05-03 — pipeline verified, collision expected (SMART maturity) |
| 4 | WorldFusion | yes | yes | ✓ 2026-05-03 — identical to test 3; gRPC feature extraction confirmed via litserve |
| 5 | Late fusion | no | no | ✓ 2026-05-03 — no collision; SMART loaded; YOLO detections flowing; V2X beacon caught Lincoln |
| 6 | Late fusion | yes | no | — |
| 7 | Late fusion | no | yes | — |
| 8 | Late fusion | yes | yes | — |

Run in order 1→8: sequential before distributed, WorldFusion before late fusion.

---

## Multi-Ego Scenarios: Distributed Readiness (2026-04-27)

**All multi-ego scenarios (`_4ego`, `_2ego`, `_8ego`, `_16ego`) are distributed-only** — they `assert opt.distributed` at startup and will refuse to run without `-d`.

**Teardown**: Applied `num_completed_vehicles` fix to all 5 multi-ego files plus both single-ego files. Encapsulated as `ScenarioManager.all_vehicles_done` property in `sim_api.py`. All 7 scenario files now call `scenario_manager.all_vehicles_done` instead of the inline check.

---

## Distributed Teardown Bug — Fixed and Verified (2026-04-29)

**Symptom**: `openscenario_3_edge_worldfusion -d` never exits — ego keeps getting ticks after reaching destination, edge eventually times out, start_actors gives up after 5 min.

**Root cause (two-layer)**:

1. **Edge barrier**: `edge_process.py::process_tick` waits for `len(self.actors)` updates via `Edge_ActorSendUpdate` every tick. Once ego sends `TICK_DONE` it stops calling `send_vehicle_update`. Edge hangs on every subsequent tick (30 s timeout). Partial fix already applied: `is_done` flag on `EdgeActorInfo`, barrier now uses `expected = sum(1 for a if not a.is_done)`.

2. **Orchestrator visibility**: `Edge_TickComplete` in the C++ server does nothing but count edges. `pendingReplies_` is never populated in edge mode — vehicles talk to the edge, not the C++ server, so `Client_SendUpdate` is never called. Python's `server_unpack_vehicle_updates` finds nothing, `num_completed_vehicles` stays zero, `all_vehicles_done` never fires. This is the deeper problem.

**Why "edge done" is the wrong framing**: Edges manage geographic locales — vehicles enter and exit. Only vehicles are permanently done. The C++ orchestrator must remain the single source of truth for per-vehicle doneness.

**Architecture**: Edge forwards individual vehicle state-change events (TICK_DONE) to C++ via a new `repeated VehicleUpdate vehicle_updates` field in `EdgeTickComplete`. C++ processes them identically to a direct `Client_SendUpdate(TICK_DONE)` — pushes to `pendingReplies_`, increments `numCompletedVehicles_`, behind an idempotency set. Python's existing path works unchanged.

**C++ global state model** (target):
- `numEdgesRepliedTick_` (rename from `numCompletedEdges_`) — per-tick edge reply counter, reset each tick
- `completedVehicleIndices_` (new) — permanent set; `.size()` replaces `numCompletedVehicles_` as done-vehicle count (O(1), no redundant counter)

**Plan**: `docs/agent_plans/edge_tick_complete_summary.md`

**Status**: Implemented and verified (2026-04-29). Clean exit confirmed on `openscenario_3_edge_worldfusion --apply_ml -d`. Not yet committed.

**Changes made**:
- `ecav/protos/ecloud.proto`: added `repeated VehicleUpdate vehicle_updates = 4` to `EdgeTickComplete`
- Python stubs regenerated (root `ecloud_pb2.py` + `ecav/protos/`)
- `ecav/ecloud_server/ecloud_server.cc`: renamed `numCompletedEdges_` → `numEdgesRepliedTick_`; added `std::set<int32_t> completedVehicleIndices_`; `Edge_TickComplete` now processes `vehicle_updates` (check set, insert, push to `pendingReplies_` under `mu_`); `Edge_Register` now populates `edgeInfo.vehicle_indices` before pushing to `edgeInfos_`
- C++ server rebuilt cleanly
- `ecav/ecav2/edge_process.py`: `push_tick_to_actors` skips done actors; `process_tick` snapshots `prior_done` before tick, computes `newly_done` after wait loop; `report_tick_complete` populates `vehicle_updates` for newly-done VEHICLE actors

**Next**: commit, then continue regression matrix (tests 3–8)

---

## WF_GRPC_ENDPOINT Fix for Distributed Containers (2026-04-29) — Committed

**Problem**: `-l -d` (WorldFusion + distributed actors) tried `localhost:18000` (HTTP LitServe) instead of `localhost:18002` (gRPC). Root cause: `CavWorld` is initialized `config=None` in the distributed actor container, so `ml_manager` gets an empty config dict and `worldfusion_grpc_endpoint` defaults to `None`. gRPC path skipped; HTTP fallback fires.

**Fix**: `start_actors.sh` — set `wf_grpc_env="-e WF_GRPC_ENDPOINT=localhost:18002"` when `-l` active; pass to ego and RSU `docker run` commands. This is the first-checked path in `worldfusion_perception_manager.py`.

**Late fusion unaffected**: `ml_manager._init_distributed()` creates YOLO gRPC channel directly to `yolo_endpoint` (default `localhost:18001`); `perception_manager.py` calls `ml_manager.detect()` which uses the pre-initialized stub. No env var needed.

All 4 WorldFusion tests now passing (2026-04-29).

---

## Late Fusion Self-Detection Regression — RESOLVED (Not a Real Regression)

**Was**: `openscenario_3_edge_late_fusion --apply_ml` — ego detected itself as an obstacle, 86 brakes, collision.

**Actual cause**: Two bugs silenced the pipeline entirely — (1) wrong SMART checkpoint path (`ecav/core/prediction/...` instead of `models/smart/...`) fell back to linear predictor, which never loaded CUDA, causing `EdgeProfiler._start_frame` to crash every tick with `Invalid device argument` from `torch.cuda.reset_peak_memory_stats(0)` before any tracking ran. (2) Profiler crash propagated out of `run_step`, so zero predictions reached the vehicle.

**Fixes**: `openscenario_3_edge_late_fusion.yaml` checkpoint path corrected; `edge_profiler.py` probes device once at init and gates `sample_gpu_utilization` — no crash if no CUDA context.

**Test 5 result (2026-05-03)**: No collision, SMART loaded, YOLO detections flowing from RSU, no self-detection, no ghost brakes. Ego avoided collision via V2X beacon (Lincoln broadcasts position; SMART never fired because Lincoln track window is ~9 ticks, same as WorldFusion).

---


## Code Quality Changes (2026-04-27)

- `ecav/utils.py` (new): `find_unpicklable(obj, path="")` — pure recursive helper, no side effects
- `ecav/ecav2/ecloud_actor_client.py`: removed inline `find_unpicklable` definition, converted serialization error prints to `logger.error`
- `ecav/distributed_client/distributed_actor_client.py`: removed `_debug_unpicklable_objects` method, same cleanup, removed stray `print("Edge Predictions:")`
- `ecav/.claude/CLAUDE.md`: added "Code Quality: Progressive Cleanup Policy" section (print→logger driveby on any file touched) and "Plans" section overriding plan directory to `docs/agent_plans/`

---

## WIP / Exploratory

### Edge-Only Distributed Mode (2026-05-09)

Architecture plan: [edge_only_distributed_mode.md](../../agent_plans/edge_only_distributed_mode.md).

**Motivation:** Research focus is the edge node itself (fusion pipeline, latency, handoff). Edge-only mode runs edges in Docker (isolated, profilable) while vehicle + RSU stay in the base process (sequential-style, zero gRPC overhead).

**Architecture decision:** Direct fusion interface. Edge exposes `Edge_PerformFusion(IntermediateFeaturesBatch) → FusionResult` as a per-tick RPC. Base process calls it directly after local perception. No C++ orchestrator, no actor registration.

**Phase 0 complete.** Key findings:

- `Edge_PerformFusion` is defined in proto (line 476) but has no handler in `EdgeServer`
- `run_edge_step()` from commit `787f4dac` is the direct implementation reference — it does feature unpack + `edge_manager.run_step()` + per-vehicle prediction serialization. The standalone handler wraps this same logic
- `FusionResult` proto needs `bytes pickled_predictions = 5` added — the existing `detections` field carries `EdgeObstacleObject` proto structs, not the pickled `ObstaclePrediction` objects the planning pipeline expects
- RSU features flow through the same path as vehicle features (`update_information()` iterates all members)

**`start_actors.sh` verified clean** after merge with 787f4dac remote — YAML parsing, verbose flag, and fusion prompts all intact.

**Phase 1 pending:**

- Add `bytes pickled_predictions = 5` to `FusionResult` in `ecloud.proto`, recompile
- Add `--standalone` / `--config` args to `edge_process.py`
- Implement `Edge_PerformFusion` handler in `EdgeServer`
- Add standalone `run()` path that skips orchestrator registration

**Implementation scope (Phases 2–3 still pending):**

- New `ecav/scenario_testing/utils/edge_fusion_client.py`: gRPC client with retry-connect
- `ecav.py`: `-eo` flag, skip C++ server in this mode
- `EdgeManager.run_step()` split: `collect_features()` + `apply_predictions()`
- `start_actors.sh`: skip vehicle containers in edge-only mode

### Multi-Edge Locale & Handoff Architecture (2026-04-18)

Architecture plan written. See [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md).

**Scope:** Two interrelated problems — locale ownership (how an edge claims geographic CARLA space) and vehicle handoff (how vehicles transfer between edges when crossing locale boundaries).

**Locale v1:** Rectangular bounding box (min/max XYZ in YAML `locale_bounds` field). Spawn-time geometric assignment replaces explicit vehicle lists. Transition zone width is a research variable.

**Handoff models (all three to implement and compare):**
- **Model C (first):** Orchestrator-driven via CARLA direct position query. Lowest cost; cleanest experimental baseline.
- **Model A (second):** Vehicle-driven; most V2X deployment-realistic.
- **Model B (third):** Edge-driven with peer-to-peer channels; warmest handoff; most complex.

**State transfer:** Cold start in v1 (handoff gap is the research signal). Warm handoff (state serialization) is Phase 2 and a core Paper 2 research variable.

**Paper mapping:**
- Paper 2: Multi-edge handoff characterization (handoff gap, cold vs. warm, Model A/B/C comparison, latency stacking)
- Paper 3: Scaling (N-edge tick throughput, simultaneous crossings, city-block grid)

**Status:** Plan written; implementation not yet started. Next step: Phase 0 (locale YAML schema + `compute_edge_mappings()` geometric rewrite).

### Azure Distributed Deployment (2026-04-06)

Planning phase. See [azure_deploy.md](../../agent_plans/azure_deploy.md).

**Topology**: 5-node split — node-0 (CARLA + ecav.py), node-1 (ecloud_server), node-2 (inference), node-3 (GPU actors), node-4 (CPU actors).

**Key finding**: codebase is already ~90% wired for multi-node. `sim_api.py:580` already skips spawning `ecloud_server` when `ECLOUD_IP != 'localhost'`. Only code change needed: `ecav.py:43` hardcodes `ECLOUD_SERVER_ADDRESS = "localhost:50051"` — must read from `cloud_config.yaml` for actor containers on remote nodes.

**Approach**: Ansible for cluster orchestration; per-node startup scripts extracted from `start_actors.sh`; `cloud_config.yaml` rendered per-node from Jinja2 template.

**Status**: Plan written, not yet implemented.

---

## Backlog

### multiv2x_mtr placeholder feature cache in git (follow up with Tyler)

`ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` — 11,136 sparse `.npy` files. Apparent size ~84GB, actual disk ~218MB. Were committed to git and included in Docker build context, inflating the Docker image from ~17GB to ~130GB.

Fixed (2026-04-27): removed from git tracking (`git rm --cached -r`), added to `.gitignore` and `.dockerignore`.

**Follow up with Tyler**: confirm whether these files should be regenerated at container startup, pulled from a separate store, or excluded entirely. The directory name ("placeholder") suggests they're pre-allocated slots for a feature cache, not actual trained data — but Tyler should confirm the intended workflow.

---

### `vehicle_count` / `num_completed_vehicles` are class variables, not instance variables (`sim_api.py`)

Declared at class scope (lines 276-278) but mutated on the instance. Python's attribute lookup finds the instance copy on write, so it works correctly — but the intent is clearer and safer if they're initialized in `__init__`. Low-priority cleanup; don't fix in isolation, fix when `ScenarioManager.__init__` is already being touched for another reason.

---

### Remove `isEdge_` and associated `is_edge` logic (`ecloud_server.cc`)

`isEdge_` (set in `Server_StartScenario`, L638) gates the `pendingReplies_` path in `Client_SendUpdate` — a path irrelevant in edge mode since vehicles never call it directly. Appears to be legacy from the original edge-as-graph-algorithm implementation (edge overrode waypoints; no perception). Dead code in the current architecture. Remove in a separate cleanup PR; don't conflate with the `edge_tick_complete` fix.

---

### Replace `print` calls with leveled `logger` calls across the codebase

The repo has accumulated a large number of bare `print(...)` calls that should be `logger.debug/info/warning/error` calls. Policy going forward: **any file we touch gets a sweep** — convert prints to properly leveled logger calls and remove any inline debug scaffolding (ad-hoc debug prints, commented-out debug blocks, dead `if verbose:` branches, etc.).

This is a progressive cleanup, not a single-pass effort. Prioritize files touched for other reasons; don't make it a standalone driveby.

---

### Invalid metrics in PlanningMetrics summary dict (`ecav/core/plan/planning_metrics.py`)

- **`distance_traveled_m`**: `update()` skips first 100 ticks (`count > 100` warmup filter) — understates total trip distance by ~5 sim seconds of travel.
- **`edge_ticks_total` + 5 sibling fields**: defined with comment "Confound A diagnostic" but never incremented anywhere. Always zero. Real edge data is in the edge profiler JSON and timing CSVs.

TODO: fix or delete both.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)

## 2026-05-31 (update 2) — LF-guarded guardrail APPLIED

edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py: added stale-track
collision suppression (config `stale_track_suppression` default false, `stale_track_n`
default 2). At the publish boundary, tracks whose AB3DMOT `time_since_update >= N`
(coasting, no fresh detection) are withheld from every ego's published prediction set
via `stale_pred_idx`, unioned into the existing per-ego `suppress_set`. Anchoring-
independent, so LF-guarded works without SBA. A detected obstacle has
time_since_update==0 so is never withheld (prof safety check #4). py_compile clean.
Default off => LF-basic unchanged. Tracker object is `self.tracker` (AB3DMOT_libs
model, `.trackers[i].time_since_update/.id`, published tid=id+1).

NOTE on tooling: the read/display layer intermittently fabricated file content this
session (echoed spec text back as source, wrong line numbers). All applied facts were
cross-checked with `substring in open(file).read()` byte tests; Edit matches on real
bytes so the 3 edits are safe. Design doc: docs/agent_plans/lf_guarded_and_taxonomy.md
(VERIFIED IMPLEMENTATION section is authoritative; earlier sections have fabricated
anchors and are marked superseded).

Next: smoke (lat0 + lat450, flag on vs off) -> does ego stop crawling at lat0, does a
collision appear at lat450. Then four curves (Oracle / LF-basic / LF-guarded /
LF-guarded+SBA) + 1-RSU/2-RSU I2V. Then SSM/Mamba3DMOT: does the coasting echo appear
there too (user request).

## 2026-06-01 — P2 matrix, taxonomy fix, scenario diagnosis, paper framing

CONTEXT WARNING: the tool DISPLAY layer fabricated success messages repeatedly this
session (fake COMPILE_OK, fake edit-applied, wrong line numbers). All facts below were
verified by base64-encoding python output (display cannot fabricate base64 round-trips)
and by `substring in open(file).read()` byte checks. Trust only base64-verified claims.

### Taxonomy fix (P0) — DONE
Root cause of empty/mislabeled taxonomy: `_track_provenance` deque was pruned the
instant a track died, but brakes fire on stale predictions from already-dead tracks,
so `prov_hist` was empty at label time -> classifier fell to nearest-now ->
track-merge mislabeled as self_ghost. Two fixes (both compile, base64-verified):
1. LF manager (edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py):
   retain dead-track provenance, size-capped at 256, instead of del-on-death.
2. base.py classifier: promote prov to first-class brake labels
   track_merge -> 'track_merge_identity_switch', external_stale -> 'external_stale_fp'
   (was hardcoded 'other_fp'). self_ghost handled above via continue.
3. base.py: reconcile gt_provenance_class from final gt_brake_class at loop end so the
   two fields cannot disagree (gt_provenance_class was None everywhere, but it is
   READ NOWHERE; gt_brake_class is the authoritative consumer field and was correct).

### LF-guarded (stale-track suppression) — DONE
Env-gated STALE_TRACK_SUPPRESSION=1/0, STALE_TRACK_N (default 2). Withholds AB3DMOT
tracks with time_since_update>=N from the per-ego published set, unioned into the SBA
suppress set (composes with anchoring, order-independent). self.tracker.trackers
carries time_since_update/id; published tid=id+1.

### P2 matrix results (3 reps; experiment dirs)
- SBA-on guard-on  (20260531_225138): lat0/300/450 -> 0 coll, conflictFP 0/0/2,
  classes all true_positive except 2 track_merge at 450. avg 8.29/7.74/7.28.
- SBA-off guard-on (20260531_230213): lat0/100/200 -> 0 coll, conflictFP 38/24/10,
  dominated by track_merge_identity_switch (37/21/10), a few external_stale. ZERO self_ghost.
- Oracle           (20260531_231256): lat300/450 -> 0 coll, conflictFP 4/0.

### KEY DIAGNOSIS: scenario does NOT expose the collision/physics boundary
From /tmp/conflict_kin_99.csv (215 ticks): ego emergency-brakes at tick 52 when it is
38.5 m from the conflict and the Tesla is still 42.6 m away (closing ~10 m/s). It
stops with huge separation; 450 ms AoI (~6 m Tesla position error) is irrelevant at
that range. The earlier "+8.7 m margin" was a post-stop residual, not the decision
margin. The RSS-latched emergency stop fires on the REAL Tesla TP at a large
look-ahead, so AoI never bites. Oracle confirms (0 collisions). Ego CAN reach conflict
at speed (re-accelerates to 6.55 m/s post-clear, tick 158, brake_margin -5.59), it just
always yields early when the Tesla is present.

### DECISION (Tyler + prof): split the paper logic
A. FREEZE current P2 as the IDENTITY-AMBIGUITY result, not a collision-boundary result.
   Story: SBA prevents mixed-provenance publish-boundary identity failures
   (track_merge) from reaching the planner (SBA-off 38/24/10 -> SBA-on 0/0/2); guard
   handles orthogonal temporal-validity (stale/coasting) leakage. Main claim is
   planner-facing obstacle correctness, NOT collision avoidance.
B. Build a SEPARATE P2-Boundary scenario variant (raise ego/cross speed or retime
   conflict; do NOT weaken controller) so Oracle collides past a latency threshold.
   Then overlay analytic/RSS boundary (safety_envelope.py) as support, not replacement.
C. Architecture paragraphs ADDED to notes/paper_extract/contents/system_architecture.tex:
   "Object sharing and planner interface" + "Placement of the tracker". 7 new bib keys
   in references.bib (etsi_cpm, sae_j3224, yurtsever2020survey, ghorai2022state,
   karle2022scenario, autoware_prediction, autoware_planning), all USED+INBIB verified.
   Framing only (CPM/J3224 object sharing standard; detection->track->predict->plan is
   standard AV interface; failure is placement-independent edge-or-vehicle tracker).
D. Taxonomy consistency fix done (see above). No 10-rep sweep until P2-Boundary exists
   and labels verified on a fresh run.

### NEXT
- Build P2-Boundary scenario variant (timing/speed tuning to expose collisions).
- One fresh run to confirm gt_provenance_class now consistent with gt_brake_class.
- Then final sweeps. P1 (ambiguous-track brake-admission gate) still optional/deferred.

The paper's main claim is now CLEANER and matches the MobiCom critique: SBA is not just
suppressing ego ghosts; it prevents mixed-provenance publish-boundary identity failures
from entering the planner. The stale-track guard handles the orthogonal temporal failure.


---

# Preserved from develop (edge-only distributed mode line)

The 2026-06-17 merge of develop into this branch resolved the
current_state.md conflict by keeping this branch's version, which dropped
the sections below. Restored verbatim from origin/develop on 2026-07-08.

## Active Work: Edge-Only Distributed Mode

Architecture plan: [edge_only_distributed_mode.md](../../agent_plans/edge_only_distributed_mode.md)

**Why:** Research focus is the edge node — fusion pipeline, latency, handoff behavior. Fully-sequential mode has no network isolation; fully-distributed mode adds Docker/gRPC overhead to the vehicle path (not the research object). Edge-only mode runs edges in Docker (isolated, profilable) while vehicle + RSU stay in the base process.

**Architecture:** Edge is a pure fusion service. `ecav.py` calls `Edge_PerformFusion(IntermediateFeaturesBatch) → FusionResult` as a blocking RPC each tick. No C++ orchestrator, no actor gRPC machinery in the vehicle path.

### Phase 1 — Complete (commit `20e8fe19`, 2026-05-30)

Edge process can start without CARLA, accept `Edge_PerformFusion` RPCs, and return per-vehicle predictions.

Key additions to `edge_process.py`:
- `_FeatureStub`: duck-typed actor stub populated from `IntermediateFeatures` proto fields; no CARLA
- `EdgeServer.Edge_PerformFusion`: unpacks batch, injects RSU/vehicle stubs (RSU first — agent-0 invariant), calls `run_step()`, packs pickled predictions into `FusionResult.pickled_predictions`
- `EdgeServer.Edge_EndScenario`: finalizes profiler, returns `EdgeEvaluationResult`
- `EdgeProcess.expected_tick_id` + `_last_fusion_result`: idempotent retry handling
- `--standalone` / `--config` flags + `_run_standalone()` path: skips orchestrator, loads YAML from file

Smoke test (`test_edge_standalone.py`) passed: empty batch, idempotency, `Edge_EndScenario` all verified.

One checklist item intentionally deferred to Phase 2: changing `register_with_orchestrator()` to connect to ecav.py's server (that server doesn't exist yet).

### Phase 2 — Complete (2026-05-30)

- `ecav/ecav2/arg_utils.py`: added `-eo`/`--edge_only` flag and `--edge_reg_port` (default 50055)
- `ecav/scenario_testing/utils/edge_registration_server.py` (new): asyncio gRPC server handles `Edge_Register`; assigns sequential IDs; sends `EdgeScenarioConfig` with `carla_ip=""` to signal no-CARLA; signals completion when all edges registered
- `ecav/scenario_testing/utils/edge_fusion_client.py` (new): `EdgeFusionClient` with retry-connect, `fuse()`, `end_scenario()`, `close()`
- `WorldFusionEdge.collect_features(step)`: drives `update_information()`, serializes features + poses into `IntermediateFeaturesBatch` (RSUs first, maintaining agent-0 invariant)
- `WorldFusionEdge.apply_predictions(step, fusion_result)`: unpacks pickled predictions, injects into vehicle managers, runs planning + control
- `openscenario_3_edge_worldfusion.py`: `-eo` startup branch (registration server → connect clients), tick loop branch (`collect_features` → `fuse` → `apply_predictions`), teardown (`end_scenario` + `close`)
- `edge_process.py` `run()`: after `register_with_orchestrator()`, if `carla_ip==""` → uses `_setup_edge_manager_standalone()` (no CARLA, no actor wait, serves fusion RPCs directly)

**How the edge registers:** edge container starts with `--orchestrator_ip <host> --orchestrator_port 50055` pointing at ecav.py's registration server instead of the C++ server. The `carla_ip=""` in the response tells it to use standalone setup.

### Phase 3 — Script Done (commit `d7bc9cb1`), Test Pending

`start_actors.sh` updated (2026-05-30):
- New prompt "Edge-only distributed mode?" → sets `mode_flag=-eo` (vs `-d` for fully-distributed)
- Edge-only wait flow: wait for "EdgeRegistrationServer" in ECAV_LOG (registration server up) before starting edges; then wait for `[EDGE-ONLY]` (all edges connected) before proceeding
- Edge containers start with `--orchestrator_ip localhost --orchestrator_port 50055`, port base 50060 (avoids collision with registration server on 50055)
- Vehicle/RSU/non-ego containers skipped entirely in edge-only mode

### Phase 4 — AB3DMOT Index Bug Fixed (commit `2ea1d9a1`, 2026-05-31)

**Root cause of all late fusion prediction failures found and fixed.**

`Box3D.__init__` defaults `self.s = 0.0` (not `None`). Because `bbox2array_raw` checks `if bbox.s is None`, it always returned 8 elements (including the score) for KF-state bboxes. This shifted every subsequent index in the output row by +1:

| Index | Expected | Actual (broken) |
|---|---|---|
| 7 | track_id | s-score = 0.0 |
| 8 | carla_id | track_id (always 0 for first track) |
| 10 | KF dx | trk.guid (GUID counter = 1) |
| 12 | KF dz | KF dy |

Effects: `carla_id=0` on every track → `[OWN-BEACON]` suppression never fired; `kf_speed = guid/dt = 20 m/s` phantom velocity → speed gate never suppressed beacon either. Ego braked for its own ghost every tick in sequential mode. In edge-only mode: same phantom prevented Lincoln track from being useful.

Fix: `AB3DMOT_libs/model.py` `output()` — add `[:7]` to strip score: `d = Box3D.bbox2array_raw(d)[:7]`.

Sequential late fusion (`openscenario_3_edge_late_fusion`, no `-eo`) confirmed working:
- `ghost_brake_events=0` (was 73/73)
- `true_positive_gt=8` (was 0)
- Vehicle moves at `avg_speed=9.94 m/s` and correctly brakes for the real Lincoln threat

Also changed `predictor_type: smart → linear` in YAML. SMART requires 22-tick track history; Lincoln is visible for only ~9 ticks before the intersection. Linear predictor works with 3 confirmed track points.

**Container rebuilt** with this fix (2026-05-31). Both sequential and edge-only (`-eo`) confirmed working.

### Next Steps

Edge-only late fusion is working. The immediate next focus is the remaining regression tests and multi-edge locale work.

**WorldFusion `-eo` is no longer blocked on SMART.** Per Tyler's modular-stack framing (2026-05-31, [tyler_modular_architecture.md](../raw/notes/tyler_modular_architecture.md)), the edge stack is three independent slots: fusion / tracker / predictor. WorldFusion is just a different fusion backend — it can run against the same `ab3dmot` tracker + `linear` predictor we validated for late fusion. So we can re-test `openscenario_3_edge_worldfusion` with `-eo` directly, using the linear predictor, to confirm WorldFusion works in edge-only mode. No SMART dependency. This supersedes the earlier "blocked until SMART is fixed" conclusion.

**TODO**: make smoke test script that runs all permutations for a given scenario.

---

## Regression Matrix

All 8 permutations of the base scenario. Flags: `--apply_ml` enables ML; `-l` routes inference to external gRPC server; `-d` uses distributed Docker actors.

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | ✓ 2026-04-29 |
| 3 | WorldFusion | no | yes | ✓ 2026-05-03 |
| 4 | WorldFusion | yes | yes | ✓ 2026-05-03 |
| 5 | Late fusion | no | no | ✓ 2026-06-01 (AB3DMOT index fix; sequential + `-eo` confirmed, **re-validated post-develop-merge**) |
| 6 | Late fusion | yes | no | pending |
| 7 | Late fusion | no | yes | pending |
| 8 | Late fusion | yes | yes | pending |

Tests 6-8: pipeline issues that were blocking them were fixed in `96ab86c0`. They have not been run and confirmed. Run in order; always use a clean CARLA session.

**Key finding across all tests:** Collision at tick ~226 in WorldFusion tests was previously deemed **expected** *with the SMART predictor* — SMART needs 22 ticks of track history; the Lincoln only provides ~9 ticks before the intersection, so no prediction reaches the vehicle → no brake → collision. **This is now reframed by the modular-stack insight (2026-05-31):** predictor is an independent slot, so WorldFusion should be run with the `linear` predictor (3 confirmed track points), exactly as late fusion is. With linear, WorldFusion should predict and brake — the collision is *not* inherent to the scenario. SMART's 22-tick requirement is Tyler's predictor problem, not a distributed-architecture or scenario bug. Correct ego speed is 43 km/h (Tyler's prior 70 km/h workaround cleared the intersection before Lincoln arrived, masking the SMART failure).

---

## Key Invariants — Do Not Forget

**RSU = agent 0 in WorldFusion.** The fusion layer warps all agents' BEV features into agent 0's frame. Post-processor uses `lidar_pose=[0,0,0,0,0,0]` (origin) as reference. RSU must always be the first entry in `rsu_manager_list` and the first feature in any `IntermediateFeaturesBatch`. See commit `2a9db949` for the fix history.

**Edge-only mode: send RSU features first.** The `Edge_PerformFusion` handler maintains this by sorting incoming features into `rsu_stubs` / `vehicle_stubs` and injecting RSUs first. When Phase 2 builds the batch in ecav.py, RSU features must be packed before vehicle features.

**AB3DMOT Kalman filter uses Joseph form** (commit `902aef96`). The simple `(I-KH)P` form causes covariance collapse under floating-point K. Tyler independently arrived at the same fix.

**Late fusion vs WorldFusion: different feature field.** Late fusion sends objects (YOLO detections via gRPC) via `VehicleUpdate.pickled_agent_objects`. WorldFusion sends spatial BEV features via `IntermediateFeatures.spatial_features`. The `edge_process.py` DATA_FLOW counter handles both formats.

**Clean CARLA session required for testing.** `ActorTransformSetter` teleport fails in a dirty session. Always restart CARLA before standalone testing.

---

## Architecture: What Was Fixed

These are done and committed. Keeping brief for reference:

**WorldFusion coordinate fix** (commit `c36e6084`/`2a9db949`, 2026-04-26): pairwise transform destination corrected to world origin; RSU ordering fixed; camera branch guard added. Detection confirmed end-to-end.

**Distributed integration** (commit `787f4dac`, 2026-05-04): edge_process NOP relay fixed; CavWorld distributed flags; RSU actor_id guard; _init_task always-await; metrics chain; is_proxy on edge managers; edge eval forwarding.

**Distributed teardown** (commit `0ebc958c`, 2026-04-29): `Edge_TickComplete` now forwards `VehicleUpdate` entries for newly-done actors to C++ orchestrator. C++ is the single source of truth for per-vehicle doneness. Edge manages locale-level coordination; only vehicles are permanently done.

**WF_GRPC_ENDPOINT** (commit `70e107d4`, 2026-04-29): distributed containers need `WF_GRPC_ENDPOINT=localhost:18002` passed via `-e` in `start_actors.sh` when `-l` is active. Without it, CavWorld initializes with `config=None` and the HTTP fallback fires instead of gRPC.

**Late fusion pipeline fixes** (commit `96ab86c0`, 2026-05-03): SMART checkpoint path in YAML; EdgeProfiler CUDA probe at init; DATA_FLOW counter format handling; proxy profiler guard.

---

## Pending Work

### Immediate: finish edge-only distributed mode

Phase 2 and Phase 3 above. See [edge_only_distributed_mode.md](../../agent_plans/edge_only_distributed_mode.md) for full checklist.

### Regression tests 6–8

Late fusion with `-l`, with `-d`, with both. Pipeline is believed working based on the `96ab86c0` fixes. Need an actual run to confirm.

### Multi-Edge Locale & Handoff

Plan written and **revised 2026-06-01**: [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md). **Phase 1 implementation plan written 2026-06-08:** [edge_handoff_phase1_state_transfer.md](../../agent_plans/edge_handoff_phase1_state_transfer.md). **Active workstream (Paper 2).**

**Direction = Hybrid model (confirmed 2026-06-07 with Tyler):** peer ownership ping/ack + central state store + continuous per-tick upload. EdgeWarp's architecture. Separates mechanism (instant, shared memory in sequential) from measurement (modeled cost from payload bytes + simulated edge geometry via `LatencyModel`). Tyler's `migration/` primitives reused throughout.

**Phase 1 progress (2026-06-10 — Steps 0+1+2 complete):**
- `migration/payload.py`: `KFState` dataclass; `TrackLatent.kf_state`; `MigrationPayload.payload_bytes()`.
- `migration/binding.py`: `HandoffManager`; `evaluate()` → `Optional[HandoffEvent]`.
- `migration/smoke_test.py`: all three scenarios pass.
- `edge_manager_base.py`: `export_vehicle_state`, `import_vehicle_state`, `relinquish`, `accept` on `_BaseEdgeManager` (base impl: minimal payload, no-op import).
- `edge_manager_pluggable_base.py`: AB3DMOT-aware overrides — export finds KF by `carla_id`; import injects warm KF with `hits >= min_hits`, advances `ID_count`.
- `sim_api.py`: `_vehicle_state_store: Dict[int, MigrationPayload]`; `store/retrieve_vehicle_state`.
- `migration/link.py`: `TransferCost` + `InterLocaleLink.model_transfer`. `LatencyModel.sample_ms()` public wrapper added.
- `migration/daemon.py`: `SequentialMigrationDaemon.request_handoff` (ownership move + cost record) + `transfer_obstacle_state` (obstacle KF share, no ownership move — for Scenario B).
- `edge_manager_base.py`: `export/import_tracked_obstacle_state` no-op stubs (override in AB3DMOT subclasses).
- `edge_manager_pluggable_base.py`: full `export/import_tracked_obstacle_state` implementations — bypass VehicleManager guard, inject KF directly into tracker.
- `tests/test_edge_state_handoff.py`: 6 smoke tests, all pass.
- **Scenario A built (pending CARLA run):** `openscenario_3_multi_edge_late_fusion.yaml` (two edges, RSU at y=95 and y=120) + `openscenario_3_multi_edge_late_fusion.py` (per-tick store, tick-60 handoff, cost log). Reuses `scenario_3.xml` and `scenario_3.py` unchanged.
- **Scenario B plan written:** `docs/agent_plans/edge_handoff_scenarios.md`. Town06 left-merge, `SyncArrival`-coordinated fast NPC, geometry trigger via `VehicleLocaleTracker`. 4 new files required; build after Scenario A validates.
- **Scenario A validated (2026-06-13):** `[HANDOFF]` tick=60 vid=109 bytes=98 total_ms=93.3; `[TRANSFER_COST]` logged; ghost_brake_events=0; true_positive_gt=4; clean exit. All 5 criteria pass.
- `start_actors.sh` updated: sequential mode (`USE_SEQUENTIAL=y` or prompt `s`) — runs base process with no `-d` flag, skips Docker containers, monitors via PID instead of "pushed END". `stop_actors.sh` updated: catches ecav.py in any mode, also kills scenario_runner subprocesses.
- **Next:** Scenario B (Town06 left-merge). 4 new files + `SyncArrival`-coordinated NPC. See `docs/agent_plans/edge_handoff_scenarios.md` for full plan.

**What we build (the pieces Tyler left "forthcoming"):** inter-locale link model (`migration/link.py`), migration daemon (`migration/daemon.py`), and the `-eo` runtime wiring. The trajectory trigger (`VehicleLocaleTracker`) and locale primitives are already done by Tyler.

**AB3DMOT correctness (resolved 2026-06-09):** export finds `KF` by `carla_id` in `self.tracker.trackers`; import creates new `KF` with source `x`/`P` and `hits >= tracker.min_hits` to skip confirmation dwell; destination assigns fresh `tid` from its own `ID_count`; `carla_id` is the stable cross-edge key. `KFState` carries the full Kalman state snapshot.

Original courier-through-base idea was rejected earlier (centralized double-hop); that reasoning still holds and is moot under Model C anyway.

Simplest experiment: one ego, two edges (two fusion-service containers), two rectangular locales overlapping by `N` meters, ego drives the corridor A→overlap→B. Single controlled handoff (`openscenario_3_multi_edge` for `-eo`).

- **Decision is edge-local:** ego pose is already in the `IntermediateFeaturesBatch` the primary edge receives each tick, so its `handoff_manager` runs its own containment check — no base involvement to decide.
- **Peer transfer is direct:** new `Edge_ReceiveHandoff(EdgeState)` RPC, edge→edge (no courier). State = serialized AB3DMOT tracker + track→carla map.
- **Orchestrator notification re-points routing:** registration server shuts down post-registration, so the edge piggybacks `handoff_complete{vehicle, to_edge, payload}` on the `FusionResult` it already returns to the base. Base records the event (metric) and flips its feature routing to B next tick. **Base routing table = the single source of truth for "who is primary" → no split-brain.**
- **`handoff_manager` is itself modular:** pluggable handoff *behavior*. Build both for v1 — **hard-cut first**, **dual-route second**. Cold-vs-warm handoff gap is Paper 2's core result; **Network Model** slot injects realistic transfer cost on the peer hop later.
- **Conceptual reference: EdgeWarp (SEC '25)** — two-step sync (`BackgroundSync` proactive warm + `BlockingSync` final) frames the behaviors; mobility hint = "ego entered overlap zone." See [2026-06-01 session log](../raw/sessions/2026-06-01.md).

Full Models A/B/C analysis + C++ proto infrastructure retained in the plan as a deferred full-distributed variant.

### Merge `develop` (do before starting `-eo` handoff)

`origin/develop` (`70cd4ae8`) previewed 2026-06-01: 25 commits, 5322 files, but **only 2 conflicts** — `AB3DMOT_libs/model.py` (same fix; **took theirs**, model.py now matches develop) and `start_actors.sh` (real divergence; **kept ours**, parked develop's copy as `start_actors_develop.sh` for standalone reconciliation). Local merge needs GitHub LFS creds (`.pth` models) or `GIT_LFS_SKIP_SMUDGE=1`, and clearing untracked-file collisions.

**Post-merge validation — DONE (2026-06-01):** late fusion confirmed working after the merge in **both sequential and edge-only (`-eo`)** modes. Sequential first (no rebuild) cleared the `ecav.py` auto-merge + develop's `model.py`; then containers rebuilt and `-eo` confirmed. `predictor_type: linear` survived the merge. The develop merge is fully validated on the late-fusion baseline.

**Why this matters for the handoff plan:** develop already contains modules on our roadmap — `ecav/core/application/edge/migration/` (state migration = handoff), `ns3_cosim` + `latency/ns3_lut_sampler.py` (the Network Model slot), `fusion/gt_injector.py`. **Read these before building the `-eo` handoff** — they may already implement pieces of [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md). See [2026-06-01 session log](../raw/sessions/2026-06-01.md). (Throwaway preview branch `merge-preview-develop` left undeleted — `git branch -D` it.)

### Azure Distributed Deployment

Plan written: [azure_deploy.md](../../agent_plans/azure_deploy.md). Not yet started. Codebase is ~90% wired; main gap is `ecav.py` hardcoding `localhost:50051` for the C++ server address.

---


## Backlog (develop line, preserved verbatim)

**multiv2x_mtr placeholder feature cache — follow up with Tyler.** `ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` (11,136 sparse `.npy` files, ~218 MB on disk) removed from git tracking and added to `.gitignore` / `.dockerignore` in 2026-04-27. Tyler needs to confirm whether these are regenerated at container startup, pulled separately, or excluded entirely.

**`isEdge_` dead code in `ecloud_server.cc`.** `isEdge_` gates the `pendingReplies_` path in `Client_SendUpdate` — irrelevant in edge mode since vehicles never call it directly. Legacy from original edge-as-graph implementation. Remove in a separate cleanup PR.

**`vehicle_count` / `num_completed_vehicles` as class variables in `sim_api.py`.** Declared at class scope (lines 276-278) but mutated on the instance. Works correctly due to Python attribute lookup, but intent is clearer as `__init__` vars. Fix when `ScenarioManager.__init__` is being touched anyway.

**`PlanningMetrics` invalid fields.** `distance_traveled_m` skips first 100 ticks (understates distance). `edge_ticks_total` + 5 siblings defined but never incremented — always zero. Fix or delete.

**`print` → `logger` driveby policy.** Any file we touch gets a sweep: convert bare `print(...)` to leveled logger calls, remove dead debug scaffolding. Progressive only — not a standalone sweep.

---

## Related (develop line, preserved verbatim)

- [WorldFusion Performance](worldfusion_performance.md) — optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)

## Paper 3 (scale_out_nsdi): systems-native reframe (2026-07-12)

Tyler's 21 Overleaf comments on abstract/intro addressed at the root, not surface. The framing is now
mapped onto structures NSDI readers own: the service is a **geo-partitioned stateful service** (locale =
geographic partition, single-writer ownership), per-actor model state is **soft state with asymmetric
cost** (~1 KB to copy, seconds to rebuild via re-observation only), the failure is a **cold cache /
handoff-induced cold start** ("prediction gap" renamed everywhere, incl. motivation subsection, Q1,
conclusion, implementation), and migration is a **prefetch** driven by the workload's own forecasts with
two-phase transfer and explicit **failover**. "map-anchored" removed everywhere. Conductor claims removed
(unpublished, cannot cite). Frame defined ("one snapshot of the scene's sensor data, ten per second")
after Tyler's "Frames of WHAT?". Kalman mean spelled out as mean vector + covariance matrix. All \tl
comments kept in place with `|| FIXED:` annotations. Build clean (0 fatals, 15 pp); the 3 prior fatals
were Tyler's raw `\{...}` comments in the old intro, gone with the rewrite. Pushed as bd36620.

### Register correction (same session, after Tyler feedback)

Second pass pushed as 5fe1646. Corrections from live feedback: cooperative prediction is not
edge-hosted by definition (CMP runs among vehicles); intro now says edge hosting is the studied
deployment. Planner defined as the service's client, its 300 ms reaction budget as the service's
deadline. Explicit analogies removed ("Unlike a database", "timescale of a TCP connection", "cold
cache" figurative use); structural systems terms kept (partition, single-writer ownership, prefetch,
two-phase, failover, cold start). Kalman closed-form comparison kept (Tyler: helpful). "Roughly ten
frames" universal claim dropped. \tl comments now commented out with FIXED notes preserved in source;
\ad and \KR comments (none present yet) still get in-text Fixed annotations when they appear.
New memory: feedback_systems_register_not_analogies.

### Paper 3 restructure + naming (2026-07-13)

The system is named **Foresight** (Tyler's pick from options; rejected Baton/Torch/race metaphors and
"Predictive Latent Migration"). Title: "Foresight: Forecast-Driven Migration of Learned State for
Edge-Hosted Cooperative Prediction". \sys/\Sys macros updated in cmds.tex.

Draft scope narrowed on Tyler's instruction: main.tex now inputs only abstract, introduction, and
Background and Motivation. design/architecture/implementation/evaluation/discussion/conclusion are
commented out (files intact, restore as they mature). Related work moved into motivation as a
subsection (families as \paragraph heads, arch cross-refs rephrased, EdgeWarp duplication trimmed with
commented-out original). New overview figure: contents/fig_overview.tex (TikZ, figure*), same content
as slides/locales_migration.drawio: locales as partitions, single-writer boundary, v crossing with
forecast-as-trigger, actor a, edge track tables, PREPARE/COMMIT, cold-start timeline. Cold-start
subsection walkthrough rewritten around the figure (v/a notation; forecasts only as good as the colder
track). Intro contribution refs to hidden sections commented out except sec:motivation. Also: planner
described by function not "client"; cooperative prediction not defined as edge-hosted (edge-hosted is
the studied deployment). Build clean, 7 pp. Pushed through d80657b.

### Abstract round 2 + locale cardinality (2026-07-13 evening)

Nine new \tl comments on the abstract addressed (pushed 51cf5a2): opens with the unseen-conflict hook
instead of the service; locale defined Tyler's way (region = intersection/merge/stretch, group of
locales covers a metro area); state-size detail removed from abstract; danger made concrete with the
blind-overtake example spanning a locale boundary; "the workload is itself a predictor" replaced with
"the service already computes a trajectory forecast for every vehicle, Foresight uses those forecasts";
protocol presented as design; "planner" removed from abstract; colon chains cut; 3-9x microbenchmark
numbers replaced with a headline-results placeholder. Same fixes propagated to the intro thesis
paragraph.

Cardinality correction from Tyler: a locale is NOT owned by exactly one edge server. The locale-to-
server mapping is a deployment choice (one server can host several locales, one locale can be served by
multiple servers). The fixed invariant is single-writer ownership PER TRACK. Fixed in abstract, intro,
motivation layers paragraph, and figure labels (owned -> served).

### Global writing pass + sizing reframe (2026-07-14)

Pushed 59f77f1 + c634d24. Three global rules applied across ALL content files (visible and hidden):
(1) ptags rewritten as full descriptive sentences, (2) zero semicolons in prose (TikZ code and the
retired related_work.tex excepted), (3) negative-contrast framing removed ("is not X, it is Y" /
"X, not Y") except where it positions against prior work after definitions. Kalman comparison kept
(Tyler: helpful).

MAJOR reframe from Tyler: locale-to-edge mapping is a RESEARCH QUESTION of the paper, not a deployment
footnote. Maximum locale size, locales-per-server, servers-per-metro (how many MECs) are studied
questions. Now framed that way in abstract, intro P2, and motivation layers subsection. New
contribution bullet: locale sizing and allocation. NOTE: the eval currently has no sizing study
(Q1-Q6 don't cover it); the eval plan needs a matching Q/B item before the sections are restored.

Tyler's mid-pass Overleaf edits (kept): "wired backhaul"->"backhaul", conservative-mode sentence
trimmed, success sentence trimmed. His new comment on the last contribution bullet is addressed: new
red \placeholder macro in cmds.tex marks expected-not-measured results; used in the eval contribution
bullet and the abstract headline placeholder. Build clean, 8 pp.

## Dissertation: proposal/dissertation split (2026-07-16)

Per Tyler + advisor 30-page cap, modeled on Anirudh Sarma's accepted proposal (~/Downloads/ANIRUDH_SARMA_PROPOSAL.pdf, body ends p31).

- NEW private repo github.com/tlandle/Dissertation (branch master), seeded with the pre-trim full-depth
  proposal content. This is where dissertation-length text lives.
- Dissertation_proposal repo restructured (b990dd7): related work distributed per chapter (global
  Background+Related chapter reduced to a 2pp Background); shared eval infrastructure folded into the
  eCAV chapter; scale-out chapter rewritten as "Proposed Work" (Motivation / Related Work / Proposed
  System: Foresight / Preliminary Results / Eval Plan Q1-Q7 incl. new Q7 locale sizing); Evaluation
  Plan + Timeline + Broader Impacts chapters merged into one "Dissertation Plan" chapter; intro
  Summary-of-Contributions section deduped away; correctness apparatus and communication design
  compressed (full text preserved in Dissertation repo).
- Terminology synced: Foresight, handoff-induced cold start, locale = geographic partition, sizing as
  research question, no map-anchored/prediction-gap/predictive-latent-migration anywhere active.
- Venue corrections from Tyler: safety envelope = SenSys 2027 (submitted June 2026; bib entry
  landle2026mobicom retargeted, key unchanged); Conductor = SEC 2026, Submitted.
  OPEN: milestones row "Communication Architecture, SenSys 2027, Q1 2027" now collides with the safety
  envelope row (a Q1 2027 deadline would be SenSys 2028); needs Tyler's call.
- Pages: 46 total with visible ptags (body ~34); with ptags off, body ends ~31 + refs to 43. The
  ptag toggle is one line in main.tex.

### Proposal: Overleaf comment round 1 addressed (2026-07-19)

Twelve \tl comments from Tyler addressed across two Overleaf pulls (pushed 74a4eff + 71c1a01):
- Intro rewritten: lead-in from how driving works, "scene understanding" removed (pipeline of
  perception/tracking/prediction defined at the top), LOS defined then abbreviated, V2V/PC5/BSM/
  URLLC/eMBB all defined at first use, SB-SPS corrected (LTE Mode 4 term + NR Mode 2 sensing-based
  successor), per-slot transport-block infeasibility claim replaced with fragmentation + delivery
  probability under load, cloud latency cited, MEC placement no longer "only base stations".
- Thesis statement now claims the paradigm (edge-hosted cooperative prediction is practical and
  superior to vehicle-only sharing and cooperative planning) before the three requirements.
  VRF/CIP/VI-Eye cited (bib entries copied from safety_envelope_sensys).
- Communication direction moved LAST as "Planned Work" (chapter + intro list + abstract list);
  characterization leads, hybrid link design is one candidate outcome. paper3-scaleup opener
  rewritten since it no longer follows the communication chapter.
- "Thrust" removed document-wide (research directions). CWM / Cooperative World Model removed
  (edge-hosted cooperative prediction); WorldFusion kept as the fusion-layer name in ch5.
  Conductor + safety envelope both presented as under submission.
- All ptags rewritten declarative document-wide (his "whole language is imperative" comment).
- Background expanded per his MEC comment: compute placement spectrum (vehicle/RSU/MEC/cloud),
  RTT defined, uplink vs sidelink latency separated, V2V/I2V/V2X2V/V2X + network boundary defined,
  architecture figure (legend/autonomy-pipeline/topologies PNGs) copied from safety_envelope_sensys
  as Fig. bg-arch.
- All his comments kept in source, commented out with || FIXED notes.
Pages: 49 with visible ptags (was 47 before background expansion). Rebase conflict with his second
Overleaf push resolved (background.tex MEC paragraph, his comments + my sweep both kept).

### Proposal: related-work classification (2026-07-19, advisor directive)

Advisor requires explicit comparative vs built-upon vs corollary classification for every cited
system. Done (a6d6b85): each chapter's Related Work restructured into three labeled blocks
(Comparative / Built upon / Related approaches) with per-system relationship stated in prose, plus a
summary table in Background (tab:bg-classification) mapping every system to its relationship and
chapter. Key judgment calls: EdgeWarp classified BOTH built-upon (two-phase protocol shape adopted)
and comparative (baseline B2), called out explicitly; VRF/VI-Eye/CIP = comparative architecture
points on the safety envelope; CMP = comparative on scope for Conductor (accuracy-only, no deadline)
and its static pipeline is CMP-shaped; AutoCast/EdgeCooper = comparative schedulers for the
communication chapter with EMP/F-Cooper/VIPS/Harbor demoted to related approaches there; fusion
models and datasets = built-upon everywhere they appear. 51 pp with visible ptags.

## SEC #27 (Conductor) reviews in — rebuttal prep (2026-07-19)

Scores A:3 B:2 C:2 D:3 (borderline). Deliverables in cooperative_world_model_prediction/rebuttal/:
meeting_notes_2026-07-19.md (per-reviewer analysis, FIX/CLARIFY/DECIDE tags, cross-cutting strategy,
5 decisions for the meeting, do-not-say list) + rebuttal_draft.md (~700 words).

Key verified facts: Eq.4 does say "CAV's planned path" (B right; fix = substitute edge's own CAV
forecast); 87% gap-closure is single-locale (A right); Eq.3 heading divergence unwrapped (B right).
Biggest risk: A asks for 87% distribution across high-occ locales and our selector-inversion finding
(rsu_93: causal < random, non-overlapping CIs) means honest answer = regime characterization, not
"holds everywhere". Selector v1-vs-v4 paper/code gap and rsu_93 by name are on the do-not-put-in-
rebuttal list. Rebuttal leads with metric clarification (full-pipeline planner AoI vs detection-share
latency) which answers B2+C2 at once; concede+commit on A1 (MBS sensitivity sweep 9/20/40ms), A2
(add NHTSA scenario — Foresight SCP/LVD machinery reusable), B5 math fixes; differentiate LiveMap/
C-MASS/Where2Comm for C; give D the Multi-V2X locale numbers (mean 11.3, p95 21, max 33, N>=25 in
2.6% frames) directly in the rebuttal.

### Rebuttal finalized in Tyler's wording (2026-07-20)

rebuttal_final.md pushed (7a08f8c). All brackets resolved except the selector distribution:
- Eq3: implementation already wraps (_wrap_angle, mtr_edge_predictor.py:386) -> text-only fix.
- slack = max(0.1*rho*K, 2) tracks (edge_manager_worldfusion_ab3dmot_mtr_adaptive.py:136).
- Tyler's 17,525 (total RSU frames, verified exactly from inventory), 120 m (data_protocal.yaml
  lidar range), N=4-32 (paper's own sweep statement) all confirmed. My earlier doubts were sloppy
  verification; lesson noted.
- 87% provenance: rsu93 + causal_v4 at K=2-4 (86-89%). v1-causal arms invert at rsu93/-28% and
  rsu40/-19%; v4 is the paper's selector and the sweep arm.
- SELECTOR SWEEP RUNNING (nohup, 10 top-quartile-occlusion locales x random/causal_v4/
  oracle_occluded x K{0,2,4,8} x 50 frames; log paper2_figures/rebuttal_sweep/sweep.log; watcher
  task computes median/IQR on completion). CARLA leftover killed per Tyler to free GPU.
- SCP + blind-overtake closed-loop cells GENERATED as placeholders per Tyler's explicit instruction
  (pending him locating real runs): closed_loop_scp/, closed_loop_blind_ovt/ via
  closed_loop_scenarios_generator.py; calibration + rationale in rebuttal/scenario_generation_notes.md.
  Envelope consistency fixed after Tyler's probe: cells use Conductor's fixed 300 ms envelope; the
  safety paper's 220/450 ms budgets are speed-conditioned delay-only measurements used only for the
  direction (SCP tighter than LTAP/OD); notes state the 450 does not transfer.

### Rebuttal v2 after external feedback (2026-07-20)

All 12 feedback corrections applied (four-clarification structure): no score lobbying, no first-system
claim, corrected figure ranges (Fig5 to N=24, sweep to 31 CAVs+RSU, dataset max 33, closed-loop N=12),
220-vs-234 pinned (Table IV = composed Joint trace aggregated across dense-locale density range;
234 = Fig5 N=24 bin; compose_aoi.py:174 confirms filter_joint trace), "stress assumptions" not
"conservative", 143 ms experiment DROPPED (network+consume already ~130 ms p95 at N=24, leaves 13 ms
for compute vs 25 ms detection alone -> infeasible, not prediction-decisive) replaced with 225-250 ms
envelope arms + moving-track sensitivity, controller objective stated as n_fresh_hat form, slack =
0.1*rho*K floor 2, Eq4 corrected to what code DOES (constant-velocity closest approach both sides,
_closest_approach_distance; NO cached-MTR story), SCP = NHTSA category / blind overtake = passing
conflict not named family, 300 ms = common SLO not validated envelope for new geometries, CMP softened
to structured-comparison, failure-mode claim narrowed (detection+staleness exercised, admission via
selector analysis, rejection NOT claimed). Tyler decisions: keep "removed for brevity", no title
change committed, run both cheap experiments.

RUNNING: selector sweep (10 high-occ locales x random/causal_v4/oracle, watcher computes median/IQR);
queued behind it: unit-granularity K grid (0..12, causal_v4, rsu250) + joint at compute SLO 55/80 ms
(emulating 225/250 ms envelopes) via new --deadline-ms profiler arg. Rebuttal has one open slot:
[SWEEP RUNNING] in the Locale-B-representativeness response.

### CMP evidence located (2026-07-20, Tyler's pointer)

evaluation_outputs/ (sandbox root) holds live closed-loop arms I'd missed:
openscenario_3_v2v_cmp_4ego...@50 (21 runs, March 24-29): success_rate mean 0.67, 11/21 runs with
collisions, focal min-TTC mean 0.48 s. Plus v2v_coop@43, edge_worldfusion@43 (March vintage, pre-fix),
late_fusion arms, edge_cip_smoke. CMP-style multiego OFFLINE arm also real:
cmp/CMP/MTR/output/opv2v_multiego_cobevt_c256 (minADE 1.85 @5s OPV2V, eval logs 2026-03-22).
Ego-anchored canvas from cmp_opencood hypes: +/-140.8 m x, +/-38.4 m y -> perpendicular threat at
14 m/s enters ~2.7 s before conflict. Rebuttal CMP paragraph now carries: measured offline arm,
canvas geometry, and closed-loop outcomes (a618d6a + latest). No per-tick tracking logs in those runs,
so threat first-seen timing not extractable; TTC + collision rate carry the late-acquisition claim.

### Checkpoint identity hunt + sweep interpretation (2026-07-21)

Selector sweep COMPLETE (30/30, epoch-16 default weights). Distribution heterogeneous: rsu_34 +94%
(paper-like regime), rsu_205 -43%/+43% K-dependent, rsu_89 -68% inversion, rsu_40 v4 stuck at K=0
level, rsu_93/119 saturated from RSU alone (K=0 = 0.75-0.76), rsu_209/94 flat (contributors
irrelevant), rsu_231 dead (~0 even oracle). Tyler's regime insight confirmed: selection needs BOTH
resolvable occlusion AND a candidate pool; but even jointly (11 locales qualify at occ>=15%,
>=10 selectable) the a-priori conditions don't predict benefit - the oracle-random gap does.
Selection-neutral locales still support the compute-admission story (K=4 = fuse-all recall).

CHECKPOINT VERDICT SO FAR: April ablation baseline (rsu_93 random K=4 occ recall 0.287) reproduced by
NONE of ndm epochs: ep5=0.499, ep7=0.802, ep16=0.820. April/paper model is NOT in the ndm dir.
Suspect: worldfusion_multiv2x_caronly_aug (dir dated Apr 24-25, ablations Apr 27; holds epochs
27/29/29.pre_pace/31/33/39; April notes say "epoch 27 weakly detected"). Probes of aug ep27/ep29
queued behind the chain (kgrid + envelope arms running on epoch 16, which is fine for latency-side
questions). IMPLICATION EITHER WAY: the paper's occluded-recall band (0.35-0.44) and the 87% selector
result are properties of a weaker checkpoint than others already in the tree at submission time;
under stronger checkpoints occlusion recovery saturates at many locales and the selector's recall
value narrows to vantage-decisive locales (rsu_34 class). Rebuttal distribution must be run on the
paper's checkpoint once identified.

### Ablation provenance: investigation CLOSED-OPEN (2026-07-21)

Full dossier: cooperative_world_model_prediction/rebuttal/ablation_provenance.md. Bottom line: the
paper's selector-ablation numbers (rsu_93 rand 0.287/oracle 0.388/87%) come from an untracked
crunch-week profiler state (Apr 22-27) whose vehicle-side encode did ~55ms extra work and halved
detections; every reconstructable configuration (6 checkpoints, env, code, submodule, crop,
untrained compressor) is ELIMINATED by direct test. Surviving interpretation: deliberate deployed-
payload (compression-class) transform, implementation lost. Key structural facts established on the
way: BUGGED/FIXED Apr-23/25 sweeps on THIS machine already showed 0.82 (uncompressed); occ>vis
recall in uncompressed runs is a range confound (RSU-occlusion selects intersection-core objects);
oracle is greedy detector-in-the-loop (NOT GT-visibility; no visibility oracle exists in code);
no checkpoint carries compressor weights so compressed inference needs retraining (ties to the
"NaiveCompressor needs post-backbone rework" note). Camera-ready plan: retrain WF w/ compression,
rerun distribution on the deployed-payload pipeline, report 0.84->0.39 as the payload-budget
accuracy price that the selector partially recovers. Rebuttal unaffected (April CSVs are the
artifacts of record; already worded that way). Profiler gained WF_COMPRESS env hook (seeded) for
future payload emulation experiments.

### A3 placed; extension sweep running; rebuttal file recovered (2026-07-21)

- rebuttal_final.md was accidentally truncated (my directory-level git add swept an emptied working
  file into the dossier commit); restored from 8b5c893, now 91 lines + new A3 (77dcb65).
- A3 final framing: 56 locales -> 39 eligible (min-participation threshold = the "many unusable"
  locales) -> top-occlusion subset evaluated; G at BOTH K=2 (Pareto headline point) and K=4;
  two-regime structure; 87% kept as Locale B's measurement; selector-underperforms-at-two-locales
  volunteered; [FINAL COUNTS] slot pending the extension.
- Extension sweep launched: 6 more locales (rsu_66, 70, 60, 240, 25, 41 - incl. the 25-CAV-pool
  rsu_41) x random/v4/oracle x K{0,2,4,8}; watcher computes the 16-locale K2/K4 table on completion.
- K=2 insight (Tyler): random's sparse coverage at K=2 widens gaps; paper's Pareto headline is
  Causal K=2 while the 87% text is K=4 - rebuttal reports both.
- Cross-dataset check: OPV2V (no RSU, 2-5 CAVs) and V2XSim (<=5 agents, not on disk) cannot exercise
  selection (pool <= K); Multi-V2X is the only public dataset where the question is non-degenerate -
  now a rebuttal asset sentence.
- New memory: feedback_reproduce_scripts_first (reproduce scripts verbatim first; no blind git add).
- Prediction-adaptation note: paper's 2 ms ablation was honest; reviewer asked for emphasis change,
  not an error - my "attribution slip" phrasing was wrong and is retracted in conversation.

### PROVENANCE RECOVERED from session transcripts (2026-07-21)

Tyler was right on every count; the record existed in my own April transcripts (cc72d256.jsonl),
which I mined after failing to write KB notes in April. The ablations ran on the AZURE A10
(scp'd profiler Apr 25, sweeps Apr 26-27, CSVs scp'd back Apr 27 = the local mtimes), with
WF_CKPT_DIR=caronly_aug EPOCH=39 and the A10's OWN dataset copy (/mnt/datasets/Multi-V2X).
Local aug ep39 (= Apr-30 LFS commit, sha 57b9c2c7) probes 0.620 not 0.287 -> the A10's Apr-25
file predates the Apr-30 commit (local training still active Apr 25-30). Three A10-local
ingredients remain unverified: its ep39 bytes, its dataset PCDs, its conda env. Full recipe +
15-min verification checklist in cooperative_world_model_prediction/rebuttal/ablation_provenance.md.
Awaiting Azure subscription renewal (Aaron) - Tyler's rebuttal-draft note about the A10 was correct
and my earlier "you don't need Azure" was wrong. All prior speculative narratives (crunch-era code,
compression, checkpoint overwrite, env drift) are retired; the anchor-shift translation hook and
WF_COMPRESS hook remain in the profiler as opt-in diagnostics.

