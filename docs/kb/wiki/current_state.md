---
updated: 2026-07-07
---
# Current State

Primary context-switching artifact. Read this first after a gap.

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

**Retrain job 10846208 submitted** (8x H200 two-stage, est start 17:55
07-07). Sbatch gained a READY-marker gate: it blocks (8 h max) on
`$DATA_TAR.READY`, which the uploader drops after the stream completes, so
an early-scheduled job can't untar a partial tar. Marker touched 15:38.
Success bar: eval minADE must land well under the ~31 m static-baseline
plateau of the corrupted run.

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
