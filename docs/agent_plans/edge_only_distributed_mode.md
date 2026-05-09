# Edge-Only Distributed Mode

**Branch:** `distributed-integration`  
**Status:** Planning  
**Created:** 2026-05-09  
**Relates to:** `docs/agent_plans/multi_edge_locale_handoff.md`

---

## Context

The primary research object is the edge node — its fusion pipeline, latency characteristics, and handoff behavior. The current simulation has two modes: fully-sequential (all actors in-process, no gRPC) and fully-distributed (all actors in Docker containers). Neither is right for edge-focused research:

- Fully-sequential: no network isolation, can't profile edge in realistic conditions
- Fully-distributed: adds Docker + gRPC overhead to the vehicle path, which is not the research focus

Edge-only distributed mode keeps edges in Docker containers (realistic, isolated, profilable) while the vehicle and RSU(s) run in the base process (sequential-style, zero extra overhead). This is also the structural prerequisite for Model C handoff (orchestrator-driven) in the multi-edge locale plan, where `ecav.py` directly queries CARLA positions and routes vehicle data to whichever edge owns the current locale.

---

## Architecture Decision: Direct Fusion Interface (Not Actor Protocol)

The current fully-distributed actor protocol is heavy: actor registration, push servers, tick synchronization via C++ orchestrator. None of that machinery is needed when the vehicle runs in the base process — the base process already owns the tick loop and has direct CARLA access.

Instead, the edge exposes a **per-tick fusion interface**: `Edge_PerformFusion(IntermediateFeaturesBatch) → FusionResult`. The base process calls this RPC directly after local perception, passing all agent feature data (vehicle + RSU cameras/lidar), and receives fused predictions to inject into planning.

This RPC is already defined in `ecav/protos/ecloud.proto` (line 445) and has generated stubs. It is **not currently implemented** in `ecav/ecav2/edge_process.py` — the `EdgeServer` class has no `Edge_PerformFusion` handler. That is the primary implementation gap.

Consequence: the C++ orchestrator (`ecloud_server`) is **not used** in edge-only mode. The edge reads its own scenario config from the YAML (already available in the Docker container via volume mount), starts its gRPC server, and waits for fusion calls. No actor registration, no push servers, no `Edge_TickComplete` back-channel.

This is also the right shape for the handoff plan: in Model C, `ecav.py` just switches which edge it sends `Edge_PerformFusion` calls to based on locale containment. No new infrastructure needed at handoff time.

---

## What Changes

### 1. `ecav/ecav2/edge_process.py`

Add standalone mode (`--standalone` flag):
- Skip `register_with_orchestrator()` — no C++ server
- Read scenario YAML from `--config <yaml_path>` arg instead
- Parse edge config from YAML (`scenario.edge_list[edge_index]`)
- Start gRPC server and wait for `Edge_PerformFusion` calls

Implement `Edge_PerformFusion` in `EdgeServer`:
- Receives `IntermediateFeaturesBatch` (tick_id, features from vehicle + RSU, pairwise_t_matrix)
- Calls the existing WorldFusion or late-fusion pipeline (whichever is configured)
- Returns `FusionResult` (fused detections)

The existing actor-protocol code (`Edge_ActorRegister`, `Edge_PushTick`, `Edge_TickComplete`, etc.) is **unchanged** — standalone mode just doesn't use it.

### 2. New `ecav/scenario_testing/utils/edge_fusion_client.py`

`EdgeFusionClient`:
- gRPC stub wrapping `Edge_PerformFusion`
- `connect(edge_ip, edge_port, retry_timeout_s)` — retry loop until edge is reachable
- `fuse(tick_id, features_list, pairwise_t_matrix) → FusionResult`
- One client per edge; for the handoff multi-edge case, the base process holds `{edge_index: EdgeFusionClient}`

### 3. Scenario `.py` files (new branch in `run_scenario()`)

When `opt.edge_only_distributed` is set, `run_scenario()` diverges at the point of tick execution:

```
# Sequential path (unchanged):
for step in range(MAX_STEP):
    edge.run_step(step)      # perception → local fusion → planning

# Edge-only distributed path (new):
for step in range(MAX_STEP):
    features = edge.collect_features(step)    # perception only, no fusion
    result   = fusion_client.fuse(step, features)
    edge.apply_predictions(step, result)      # planning + control using fused result
```

This requires splitting `EdgeManager.run_step()` into:
- `collect_features(step)` — runs perception pipeline, returns `IntermediateFeaturesBatch`
- `apply_predictions(step, fusion_result)` — injects predictions, runs planning/control

**Files to update:** `openscenario_3_edge_worldfusion.py`, `openscenario_3_edge_late_fusion.py` (and any future edge scenarios that opt into this mode).

### 4. `ecav/core/common/edge_manager.py` (or equivalent)

Add `collect_features()` and `apply_predictions()` methods (or refactor `run_step()` to accept an optional fusion callback). The exact split point depends on where the existing fusion code hooks in — **this is the riskiest part and needs careful reading during implementation**.

### 5. `ecav.py`

Add `--edge-only-distributed` / `-eo` flag. When set:
- Skip `run_comms()` (no C++ orchestrator)
- Create `EdgeFusionClient(s)` from YAML edge config
- Wait for edge containers to be reachable before starting tick loop
- Pass flag through to `opt` so `run_scenario()` branches correctly

### 6. `start_actors.sh`

In edge-only mode (detected from YAML `manager_type` or a new `edge_only: true` YAML field):
- Start base eCAV process with `-eo`
- Start edge Docker containers with `--standalone --config ecav/scenario_testing/config_yaml/${scenario_name}.yaml -e $edge_index`
- Do **not** start vehicle or RSU containers
- Wait for edge gRPC port to be reachable (replace current "pushed scenario start" wait with a retry-connect check)
- Rebuild container prompt still applies (edge still uses the Docker image)

---

## What Does NOT Change

- `ecav/ecav2/edge_process.py` actor protocol (non-standalone mode is untouched)
- `ecav/ecloud_server/` — C++ server unchanged
- `ecav/protos/ecloud.proto` — no new messages needed; `Edge_PerformFusion` already exists
- Existing fully-distributed scenarios (they still use the old code path)
- `stop_actors.sh`

---

## Risks & Open Questions

| Risk | Mitigation |
|------|-----------|
| `run_step()` split: perception and planning are interleaved in complex ways inside EdgeManager | Read the EdgeManager and WorldFusion/late-fusion perception manager code carefully before splitting. Start with worldfusion (cleaner model). |
| `IntermediateFeaturesBatch.pairwise_t_matrix` format: must match what the WorldFusion model expects | Confirm by reading `ecav/worldfusion/` inference code before building the serialization path |
| RSU perception data: RSU camera/lidar must be included in the fusion batch (not just vehicle data) | RSUManager runs locally in edge-only mode; its features must be collected in `collect_features()` alongside vehicle features |
| `Edge_PerformFusion` implementation in edge process must actually run the model | The existing `fuse_predictions()` is a passthrough placeholder — needs real implementation |

---

## Implementation Checklist

### Phase 0: Exploration
- [ ] Read `ecav/core/common/edge_manager.py` (or equivalent) — understand `run_step()` fully
- [ ] Read WorldFusion perception manager to understand feature extraction and the `IntermediateFeaturesBatch` fields
- [ ] Confirm RSU features are captured in the same path as vehicle features

### Phase 1: Edge Standalone Mode
- [ ] Add `--standalone` and `--config` args to `edge_process.py`
- [ ] Add `Edge_PerformFusion` handler to `EdgeServer` in `edge_process.py`
- [ ] Implement real fusion in `Edge_PerformFusion` (call WorldFusion model or late-fusion pipeline)
- [ ] Test: start edge process in standalone mode, call `Edge_PerformFusion` manually via grpcurl

### Phase 2: Base Process Client
- [ ] Write `ecav/scenario_testing/utils/edge_fusion_client.py` with retry-connect
- [ ] Add `-eo` flag to `ecav.py`
- [ ] Split `EdgeManager.run_step()` into `collect_features()` + `apply_predictions()`
- [ ] Add `edge_only_distributed` branch to `openscenario_3_edge_worldfusion.py`

### Phase 3: Launch Script
- [ ] Update `start_actors.sh`: skip vehicle containers in edge-only mode
- [ ] Replace "pushed scenario start" wait with edge gRPC health check
- [ ] Test: `openscenario_3_edge_worldfusion` end-to-end in edge-only distributed mode

### Phase 4: Verification
- [ ] Confirm vehicle drives successfully using edge-fused predictions
- [ ] Confirm edge profiler logs are written (`edge_profiler_<ts>.json`)
- [ ] Confirm late-fusion scenario also works (same code path, different edge config)

---

## Relationship to Handoff Plan

`EdgeFusionClient` is the primitive the handoff plan builds on. For Model C:

```python
# ecav.py per-tick (Model C handoff — future):
for vehicle_idx, vehicle in vehicles.items():
    edge_idx = locale_map.containing_edge(vehicle.get_transform().location)
    fusion_result = fusion_clients[edge_idx].fuse(step, collect_features(vehicle))
    apply_predictions(vehicle, fusion_result)
```

No new infrastructure at handoff time — just routing to a different `EdgeFusionClient`. The "handoff gap" (cold-start tracking state on the new edge) is directly observable.
