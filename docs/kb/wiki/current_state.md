---
updated: 2026-07-27
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

**`develop`** — the shared working branch (since 2026-06-01). All `distributed-integration` work was fast-forward-merged into `develop` and pushed; subsequent work commits directly on `develop`. In sync with `origin/develop` (pushed through 2026-06-02). New work goes on `develop`.

`distributed-integration` is retained as a redundant marker.

**Conda env: `ecav310`** (converged 2026-06-02, finishing the migration develop began; retired the `opencda`/`opencda310` mix). `environment.yml` `name:` is `ecav310`; all active scripts/docs use it. Recreate or `conda rename` your local env to match before running. `start_actors.sh` is reconciled (our `-eo` path + develop's `--auto`/env-override automation; conda env via `CONDA_ENV`/`CONDA_ROOT`). `.claude/settings.local.json` is now gitignored/untracked.

---

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

**Phase 1 progress (2026-07-19 — Steps 0–6 complete; Scenario B remaining):**
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
- **Step 6 metrics (2026-07-19):** hand-off cost sink on `ScenarioManager` (`record_handoff_cost`/`get_handoff_costs`, mirrors `sim_metrics`); pure `summarize_handoff_costs` in `evaluate_manager.py`; `EvaluationManager.handoff_eval` drains into `global_metrics['handoffs']`/`['handoff_summary']` + `HAND-OFF MIGRATION` section in `evaluation_report.txt`. Scenario loop records each cost; finally-block trace reads the sink (single source of truth).
- `tests/test_edge_state_handoff.py`: **8** smoke tests, all pass (added summary-shape + JSON round-trip). Step 6 live CARLA gate (block in a real `simulation_metrics.json`) folds into the next Scenario A/B run.
- **Scenario A validated (2026-06-13):** `[HANDOFF]` tick=60 vid=109 bytes=98 total_ms=93.3; `[TRANSFER_COST]` logged; ghost_brake_events=0; true_positive_gt=4; clean exit. All 5 criteria pass. (Validated before the Step 6 metric-sink wiring.)
- **Scenario B plan written:** `docs/agent_plans/edge_handoff_scenarios.md` + checklist added to Phase 1 plan Step 7. Town06 left-merge, `SyncArrival`-coordinated fast NPC, geometry trigger via `VehicleLocaleTracker`. 4 new files required; obstacle-export extension already built.
- `start_actors.sh` updated: sequential mode (`USE_SEQUENTIAL=y` or prompt `s`) — runs base process with no `-d` flag, skips Docker containers, monitors via PID instead of "pushed END". `stop_actors.sh` updated: catches ecav.py in any mode, also kills scenario_runner subprocesses.
- **Scenario B ego path VALIDATED live (2026-07-19, 9 runs; see session log):** renamed right_merge (CARLA left-handed frame — +y is the vehicle's RIGHT heading east); final geometry = ego+ambulance in leftmost lane -3, NPC in lane -4 on an explicit WaypointFollower plan; geometry-driven handoff for EVERY boundary crossing. Ego brakes on edge predictions (17m out, post-handoff via edge 1's RSU), waits for the NPC, merges right, continues. Handoff fires at the crossing with the full 986-byte KF payload; HAND-OFF MIGRATION block confirmed in the real eval report (**Step 6 live gate closed**). behavior_agent overtake fix required: plan lookahead floored at 15m (next(speed*6) degenerates at crawl speed; next(0) raises). AB3DMOTStateTransferMixin extracted — discovery: LateFusionEdge never inherited the transfer methods, so ALL prior live payloads (incl. Scenario A) were empty 98-byte stubs; identity now resolves via track_to_carla. Warm import gated OFF by default (handoff_warm_import; Phase 1.5). Commits: fd3a4e59, 64527db0.
- **Scenario B fully validated (2026-07-27, run 11; commit 6eef5c89):** predictive trigger implemented (`Locale.predicted_to_exit_within`, `OBSTACLE_HANDOFF_LOOKAHEAD_S=1.0`) + reactive fallback; `locale_by_id` added to `_build_locale_router` return. RSU0 moved x=75→55 (reacquisition zone shift). `collision_time_ahead: 2→4` fix (run-10: ego stopped only 5m from ambulance; run-11: first hazard fires at 42.3m). Full run-11 sequence: vehicle handoff tick=63 (986 bytes), ambulance detected at 42.3m via edge predictions, ego waits for NPC at 17 m/s (track_id=61 TTC=0.00s), overtake initiates tick=425, merge complete by tick=500 (y=141.2, 4.3m clear of ambulance at y=136.9), zero Safety Warnings in 700 ticks. MAX_STEP=700 reached with ego at x=384 of 600m destination — step budget runs out; the research-relevant sequence (detection → wait → merge) is complete. **OPEN:** NPC obstacle KF handoff still not firing — `[TRACK-DBG]` diagnostic confirmed `dets_in=0` at tick=183 on edge 0 (RSU0 blind donut persists; NPC transits reacquisition zone too fast). Root cause hypothesis: after ego handoff at tick=63, edge 0 has zero `vehicle_manager_list` entries, and RSU detection processing may gate on non-empty vehicle list — needs investigation. Curved-road suppression bug (`behavior_agent.py:1555`) documented in plan (Tyler scope). **OPEN:** `[TRACK-DBG]` and `[EGO-DBG]` / `[SCENB-DBG]` diagnostics still at WARNING level — remove after obstacle path resolves.
- **(superseded build notes:)** 4 new files — `scenarios/scenario_multi_edge_left_merge.xml`, `scenarios/scenario_multi_edge_left_merge.py` (runner; `SyncArrival` fixed to real 1-target API + wrapped in a `SUCCESS_ON_ONE` Parallel with a distance trigger so the tree advances past it), `config_yaml/openscenario_multi_edge_left_merge.yaml` (Town06, 2 edges w/ `locale` polygon blocks), `openscenario_multi_edge_left_merge.py` (geometry trigger via `VehicleLocaleTracker`; `daemon.transfer_obstacle_state` on `locale_1` crossing; geometric advance-warning proxy). Geometry trigger unit-verified vs. the real YAML polygons (crossing at x=156, inside overlap). All 4 files compile; XML/YAML parse.
- **Next:** run `python ecav.py -t openscenario_multi_edge_left_merge --apply_ml` on clean CARLA + YOLO litserve (:18001). Expect a tuning iteration on Town06 lane coords / `sync_target` / blueprint (plan flags these). After B validates: separate Phase 2 `-eo` distribution plan (placeholder = Step 8); actor distribution `-d` is a distinct Phase 3.

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

## Backlog

**multiv2x_mtr placeholder feature cache — follow up with Tyler.** `ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` (11,136 sparse `.npy` files, ~218 MB on disk) removed from git tracking and added to `.gitignore` / `.dockerignore` in 2026-04-27. Tyler needs to confirm whether these are regenerated at container startup, pulled separately, or excluded entirely.

**`isEdge_` dead code in `ecloud_server.cc`.** `isEdge_` gates the `pendingReplies_` path in `Client_SendUpdate` — irrelevant in edge mode since vehicles never call it directly. Legacy from original edge-as-graph implementation. Remove in a separate cleanup PR.

**`vehicle_count` / `num_completed_vehicles` as class variables in `sim_api.py`.** Declared at class scope (lines 276-278) but mutated on the instance. Works correctly due to Python attribute lookup, but intent is clearer as `__init__` vars. Fix when `ScenarioManager.__init__` is being touched anyway.

**`PlanningMetrics` invalid fields.** `distance_traveled_m` skips first 100 ticks (understates distance). `edge_ticks_total` + 5 siblings defined but never incremented — always zero. Fix or delete.

**`print` → `logger` driveby policy.** Any file we touch gets a sweep: convert bare `print(...)` to leveled logger calls, remove dead debug scaffolding. Progressive only — not a standalone sweep.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
