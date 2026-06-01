---
updated: 2026-06-01
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

**Unpushed commits:** `20e8fe19` (Phase 1: edge-only distributed mode — standalone edge fusion service)

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

Plan written and **revised 2026-06-01** for the `-eo` substrate: [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md). Not yet implemented. This is the next active workstream (Paper 2).

**Direction = Model C, building on Tyler's `migration/` module** (jrapp, 2026-06-01 PM; pending Tyler confirmation). **This supersedes the morning's Model B (edge-peer) decision.** Reason: develop already ships `ecav/core/application/edge/migration/` — polygon `Locale`, `LocaleRegistry`/`LocaleRouter` (orchestrator polls router → Model C), `VehicleLocaleTracker` (hysteresis binding, emits `HandoffEvent`), `MigrationPayload`/`TrackLatent` (latent-state migration). Tyler's router/binding is orchestrator-centric = Model C. We adopt it rather than diverge. The B-vs-C analysis and the morning's `-eo` peer-mesh rewrite in [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md) are retained for history but are now superseded by "build on Tyler's primitives."

**What we build (the pieces Tyler left "forthcoming"):** the trajectory trigger, the inter-locale link model (parametric, + the `ns3_cosim` 5G-LENA high-fidelity path = Network Model slot), the migration daemon, and the **`-eo` runtime wiring** (his primitives have zero runtime imports today). Our rectangular `locale_bounds` is superseded by his polygon `Locale`. Open question for Tyler: his `MigrationPayload` carries **neural latent state** (RNN hidden + attention cache) for the sequence-model stack; our validated `-eo` baseline is AB3DMOT + linear (KF state), so the cold-vs-warm baseline needs KF state mapped into `TrackLatent` or confirmation he's targeting the neural predictor.

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
