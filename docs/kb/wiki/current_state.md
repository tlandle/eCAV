---
updated: 2026-07-27
---
# Current State

Primary context-switching artifact. Read this first after a gap.

---

# Follow Up (Tyler Sync)

Items to raise at next Tyler sync:

- **Phase 2 `-eo` distribution plan (Step 8)** — separate plan needed before building. Discuss scope and what Tyler has already scaffolded (registration server, edge-fusion client) vs. what needs wiring.
- **Curved-road suppression bug `behavior_agent.py:1555`** — `potential_curved_road` term in step-8 guard forces car-following even when `overtake_allowed=True` and `overtake_counter==0`. Fires when merge waypoints look curved to the local planner. Fix: remove `potential_curved_road` from that conditional. Tyler scope (flagged with `#TL`).
- **Track-birth timing findings** — Scenario B debugging established that AB3DMOT needs `min_hits=3` confirmed detections before a track is published. At 50ms world_dt that's 3 ticks. Relevant for Phase 2 advance-warning window claims.
- **Warm import gate (Phase 1.5)** — `handoff_warm_import = False` by default. When does Tyler want to enable this? Requires import-side reconciliation to avoid stale-duplicate ghosts.
- **`_PluggableEdgeBase` MRO change** — the 2026-07-27 develop merge removed `AB3DMOTStateTransferMixin` from `_PluggableEdgeBase`'s MRO and replaced it with inline dual-backend dispatch. Check whether any of Tyler's test fixtures do `isinstance(edge, AB3DMOTStateTransferMixin)` — those would now fail.
- **Open diagnostics in scenario file** — `[EGO-DBG]` and `[SCENB-DBG]` still at WARNING level in `openscenario_multi_edge_right_merge.py`. `[TRACK-DBG]` was resolved in the merge. Remove before Phase 2.
- **PACE MTR training status** — job 10846208 (8x H200, two-stage clean data) submitted 2026-07-07. Check if it completed and what minADE landed.
- **Docker cleanup** — `docker system df` showed build cache ~80GB with ~41GB reclaimable. Run `docker system prune` when convenient.
- **Scenario B MAX_STEP** — ego reaches x=384 at tick=695 but destination is x=600; scenario terminates before full route completion. Not a research blocker (merge sequence is the contribution) but worth noting.

---

## Active Branch

`develop` → PR target: `ecav_2_distributed`

Branch is 10 commits ahead of `origin/develop` (includes 2026-07-27 merge of origin/develop's multi-edge predictive latent migration work).

---

## DONE: Phase 1 Scenario B — Right-Merge Obstacle Handoff (2026-07-27)

**Result:** All Phase 1 Scenario B requirements met. Run 12 (commit `6cefda1a`).

| Event | Tick | Detail |
|---|---|---|
| Vehicle handoff | 63 | vid, 986 bytes, full KF state |
| Ambulance detected by edge predictions | ~305 | dist=42.3m (before local sensor range) |
| **Predictive obstacle handoff** | 161 | NPC at x=98.3, 16.7m before locale boundary |
| RSU1 first detects NPC | 243 | **advance-warning window = 82 ticks (4.1s)** |
| Ego merges right | ~500 | 4.3m lateral clearance, no crash |

**Three bugs fixed to reach this:**
1. `if not beacons:` guard in jitter buffer drain silently dropped all RSU detections after ego handoff (vehicle_manager_list empty → beacons dict empty → guard fired). Fixed: `if not beacons and not objects.get('vehicles'):`.
2. `KeyError` in `_collect_ab3d_detections` — VM in list but not in beacons snapshot (timing race at handoff boundary). Fixed: `if vm.vehicle.id not in beacons: continue`.
3. `_find_obstacle_kf` position fallback always returned None — used `kf.x[1]` (CARLA_z height ≈ 0) instead of `kf.x[2]` (CARLA_y lateral ≈ 140). KF state layout: `[x, y, z, theta, l, w, h, dx, dy, dz]` → `x[0]=CARLA_x`, `x[2]=CARLA_y`. Made NPC-to-KF distance ≈ 140m always >> `max_dist_m=15`. Fixed index to `[2]`.

**Key geometry:** Town06 right-merge (CARLA left-handed frame; +y is the vehicle's RIGHT heading east). Ego + ambulance in lane y≈137, NPC in lane y≈140. RSU0 at x=55 (moved from x=75 to clear near-field blind donut). Locale boundary x=115.

**Plan:** `docs/agent_plans/edge_handoff_phase1_state_transfer.md` (Steps 0–7 checked off).

---

## Next: Phase 2 `-eo` Distribution Plan (Step 8)

Separate plan needed. Pre-reads:
- `docs/agent_plans/edge_only_distributed_mode.md` — Phase 1–4 complete; registration server, fusion client, and standalone mode all built.
- Tyler's `migration/` primitives: `locale.py`, `registry.py`, `binding.py`, `payload.py` — these are the building blocks.

The research contribution is the advance-warning window (mechanism + cost). Phase 2 wires the mechanism into `-eo` so the handoff traverses a real gRPC hop with modeled transfer cost.

---

## NSDI Paper: scale_out_nsdi (2026-07-07)

Branch `scale_out_nsdi` (commit `33c8a07`). Systems positioning complete: related-work problem-class abstraction, camera-networks subsection, ClairvoyantEdge distinction, system model corrected (locale = CONFLICT ZONE not RSU). Boundary framing: "static canvas, dynamic assignment, ASYMMETRIC elasticity." 74 `\ptag` intent tags, AV-term glosses for systems reviewers. 15pp clean build.

Tyler reading both manuscripts. Professor email drafted.

---

## Dissertation Proposal (DONE, 2026-07-05)

Five research thrusts, chapter order fixed, P2–P6 complete, 60 pages clean build, 0 warnings. `\ptag` intent tags visible until advisors approve; flip one-line macro in `main.tex` to hide.

---

## Scale-out B0 (2026-07-05)

**B0.3 DONE — live full-latent migration in CARLA.** `openscenario_3_multi_edge_mamba`. At tick 60: full memo bank (10 frames, 776 B) exported from edge0, injected warm at edge1. Tracklet survived; no post-handoff exceptions. Known gaps: warm-vs-cold DELTA needs B4 metrics; `SOTAEdge.evaluate()` NotImplementedError at cleanup (non-fatal).

**B0.2 DONE — Mamba latent through edge dispatch.** Mamba3DMOT in tracker registry. `_PluggableEdgeBase` dispatches: Mamba → full latent via factories; AB3DMOT → KFState. Both verified under `opencda310` (`test_mamba_edge_migration.py`): banks byte-identical, id preserved, ~1.3 KB.

**B0.1 DONE — live edge migrates KF state.** `PredictionLateFusionEdge` export/import wired with real AB3DMOT KF snapshot (mean, covariance, hits≥min_hits, velocity). Round-trip verified.

---

## WF→MTR Training (PACE, status as of 2026-07-07)

Goal: train MTR on WorldFusion fused BEV features (CMP-style) for Multi-V2X.

**Current status:** Retrain job 10846208 submitted 2026-07-07 (8x H200, two-stage, ~12h). Used clean re-export (174 GB, 52 zones) after discovering the prior 8-GPU run (10804312) used `train=True` in export → per-frame random augmentation scrambled trajectories → model plateau at ADE ~31m (static baseline). Fixed: `export_wf_for_mtr.py` now uses `train=False`.

**Infrastructure validated** (2x H200 job 10802480): DDP, eval after every epoch, best_model save, clean exit, finite metrics (minADE=nan fix: skip objects with `final_valid_idx<1`).

**Correct model:** `worldfusion_multiv2x_translaug_finetune` (epoch 27/45 local at `ecav/ml_manager/models/`). NOT `caronly_aug` (wrong pick; scene-overfits).

**Key design decision:** everything in RSU-EGO frame. GT boxes + detection positions both in ego frame. No AB3DMOT, no rekey — GT-anchored Hungarian association (gate 2.0m) for building training data. Verified rsu_66: det pos error mean 0.42m, per-frame recall 44%, 1086 trainable samples/zone.

**PACE artifacts:** `$PROJECT/wf_mtr_translaug.tar` (174 GB, READY marker dropped after upload), `$PROJECT/mtr_code.tar`. Sbatches in `ecav/core/prediction/mtr/tools/scripts/`. Lock file pattern: `$PROJECT/mtr_wf_smoke.lock` — always `rm -f` before resubmitting.

**Known issue:** intermittent RSU detections (44% recall) give <2 valid past obs for some records — loader skips those (BatchNorm crash on single-point sequences). CMP-strict would drop these; MTR masks missing past via `obj_trajs_mask`, so lenient selection is correct.

---

## Phase 1 State Transfer — Steps 0–6 Reference

All committed. See `docs/agent_plans/edge_handoff_phase1_state_transfer.md` for full checklist.

- `migration/payload.py`: `KFState`, `TrackLatent`, `MigrationPayload`.
- `migration/binding.py`: `HandoffManager`, `evaluate()` → `Optional[HandoffEvent]`.
- `edge_manager_base.py`: `export/import_vehicle_state`, `relinquish`, `accept` stubs.
- `ab3dmot_state_transfer.py`: `AB3DMOTStateTransferMixin` — AB3DMOT-aware export/import with position fallback for unmanaged obstacles; mixed into `PredictionLateFusionEdge`.
- `edge_manager_pluggable_base.py`: dual-backend inline dispatch (Mamba + AB3DMOT) — replaces mixin inheritance as of 2026-07-27 merge.
- `sim_api.py`: `_vehicle_state_store`, `store/retrieve_vehicle_state`.
- `migration/link.py`: `TransferCost`, `InterLocaleLink.model_transfer`.
- `migration/daemon.py`: `SequentialMigrationDaemon.request_handoff`, `transfer_obstacle_state`.
- Metrics sink: `record_handoff_cost` on `ScenarioManager`; `handoff_eval` in `EvaluationManager`; `HAND-OFF MIGRATION` block in evaluation report.
- Scenario A validated (2026-06-13): tick=60, 986 bytes, ghost_brake_events=0.

**AB3DMOT warm import is OFF by default** (`handoff_warm_import = False`). Payloads are exported and logged; destination tracker is untouched until Phase 1.5.

---

## Paper 1 (safety_envelope_sensys) — SUBMITTED

Branch `paper-closed-loop-recreate`. Full analysis in session logs 2026-05-30 through 2026-06-02. Short summary: SBA (anchoring) expands safety envelope from ~100ms logic cliff to ~400ms physics limit. Oracle confirms architecture works when detection is clean. Detailed eval and taxonomy analysis archived in session logs; not recapped here.

---

## Key Invariants

**RSU = agent 0 in WorldFusion.** Fusion layer warps all agents' BEV features into agent 0's frame. RSU must always be first in `rsu_manager_list` and first in `IntermediateFeaturesBatch`. See commit `2a9db949`.

**AB3DMOT detection format = 8 columns.** `_collect_ab3d_detections` produces `[h,w,l,x,y,z,theta,confidence]` — 8 columns. Empty early-exit arrays must be `np.empty((0, 8), np.float32)`. `(0, 7)` is a recurring mistake (2026-05-31 root cause: `Box3D.__init__` defaults `s=0.0`; see session log). `array2bbox_raw` handles 8-column input gracefully (`data[-1]` → `bbox.s`); the `[:7]` strip in `model.py output()` is for the KF-state output path only.

**AB3DMOT KF state vector:** `[x, y, z, theta, l, w, h, dx, dy, dz]` (dim=10). KITTI camera convention: `x[0]=CARLA_x`, `x[1]=CARLA_z (height)`, `x[2]=CARLA_y (lateral)`. Position fallback in `_find_obstacle_kf` must use `x[0]` and `x[2]`, NOT `x[1]`.

**Clean CARLA session required for testing.** `ActorTransformSetter` teleport fails in a dirty session. Always restart CARLA before standalone testing.

**Late fusion vs WorldFusion: different feature field.** Late fusion: `VehicleUpdate.pickled_agent_objects` (YOLO detections). WorldFusion: `IntermediateFeatures.spatial_features` (BEV).

**AB3DMOT Kalman filter uses Joseph form** (commit `902aef96`). Simple `(I-KH)P` causes covariance collapse; Joseph form `(I-KH)P(I-KH)^T + KRK^T` is numerically stable.

**`collision_time_ahead: 4`** in `openscenario_multi_edge_right_merge.yaml`. At 2, hazard fires at ~22m → ego stops 5m from ambulance → collision on creep. At 4, fires at ~44m → ego stops with clearance.

---

## Regression Matrix

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | ✓ 2026-04-29 |
| 3 | WorldFusion | no | yes | ✓ 2026-05-03 |
| 4 | WorldFusion | yes | yes | ✓ 2026-05-03 |
| 5 | Late fusion | no | no | ✓ 2026-06-01 (re-validated post-develop-merge) |
| 6 | Late fusion | yes | no | pending |
| 7 | Late fusion | no | yes | pending |
| 8 | Late fusion | yes | yes | pending |

Tests 6–8: pipeline believed working from `96ab86c0` fixes; not yet run. Always use clean CARLA session.

---

## Backlog

- **`isEdge_` dead code** (`ecloud_server.cc`) — gates `pendingReplies_` path irrelevant in edge mode. Legacy. Remove in a cleanup PR.
- **`vehicle_count`/`num_completed_vehicles` as class vars** (`sim_api.py:276–278`) — mutated on instance; works but unclear. Fix when `ScenarioManager.__init__` is touched for another reason.
- **`PlanningMetrics` invalid fields** — `distance_traveled_m` skips first 100 ticks; `edge_ticks_total` + 5 siblings never incremented. Fix or delete.
- **`print` → `logger` driveby** — any file touched gets a sweep. Progressive only.
- **multiv2x_mtr placeholder cache** — `ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` removed from git; Tyler to confirm intended workflow.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md)
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
- [Phase 1 Plan](../../agent_plans/edge_handoff_phase1_state_transfer.md)
- [Scale-out Eval Plan](../../agent_plans/scale_out_evaluation.md)
