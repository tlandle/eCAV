---
updated: 2026-05-30
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

### Next Steps (stopping point 2026-05-30)

**Container must be rebuilt after any `ecav/` code change before running Docker-based tests.**
`docker build --network=host -f Dockerfile -t ecav-python310:latest .`

End-to-end test sequence — run in order:

1. Rebuild container if any code changed since last build (see above)
2. Start CARLA (headless or with display)
3. Start edge container (in a separate terminal, run without `-d` to see output):
   ```
   docker run --runtime=nvidia --gpus all --network=host \
       --name=edge_0 -e HOSTNAME=edge_0 -e IS_DOCKER=1 \
       -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
       ecav-python310:latest \
       python3.10 -u ecav/ecav2/edge_process.py \
           --orchestrator_ip localhost --orchestrator_port 50055 -P 50060
   ```
   Edge will print "edge-only ready" and wait for RPCs.
4. Run ecav.py:
   ```
   python -u ecav.py -t openscenario_3_edge_worldfusion -v 0.9.15 -eo --apply_ml
   ```

What to verify (Phase 4):
- Vehicle drives successfully using edge-fused predictions
- Edge profiler JSON written to `logs/edge_profiler_<ts>.json`
- No crashes during `collect_features` → `fuse` → `apply_predictions` per-tick loop

---

## Regression Matrix

All 8 permutations of the base scenario. Flags: `--apply_ml` enables ML; `-l` routes inference to external gRPC server; `-d` uses distributed Docker actors.

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | ✓ 2026-04-29 |
| 3 | WorldFusion | no | yes | ✓ 2026-05-03 |
| 4 | WorldFusion | yes | yes | ✓ 2026-05-03 |
| 5 | Late fusion | no | no | ✓ 2026-05-03 |
| 6 | Late fusion | yes | no | pending |
| 7 | Late fusion | no | yes | pending |
| 8 | Late fusion | yes | yes | pending |

Tests 6-8: pipeline issues that were blocking them were fixed in `96ab86c0`. They have not been run and confirmed. Run in order; always use a clean CARLA session.

**Key finding across all tests:** Collision at tick ~226 in WorldFusion tests is **expected and correct**. SMART needs 22 ticks of track history to confirm a track; the Lincoln only provides ~9 ticks before the intersection. No predictions reach the vehicle → no brake → collision. This is Tyler's predictor problem, not a distributed architecture bug. Tyler's prior workaround (70 km/h) cleared the intersection before Lincoln arrived, masking the failure. Correct ego speed is 43 km/h.

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

Plan written: [multi_edge_locale_handoff.md](../../agent_plans/multi_edge_locale_handoff.md). Not yet started. Depends on edge-only distributed mode (Phase 3) being solid first.

**Model C first** (orchestrator-driven via CARLA position query). Paper 2 focus. `EdgeFusionClient` from Phase 2 is the primitive this builds on — handoff just routes to a different client.

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
