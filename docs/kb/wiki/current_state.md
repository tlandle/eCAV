---
updated: 2026-04-06
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

---

## Milestone: Distributed Perception Fast and Fully Operational (2026-04-06)

Both fusion modes confirmed working end-to-end in distributed mode, with production-quality latency.

### Late Fusion (BM2CP)
- Actor client → gRPC → C++ server → sim_api → edge_process → BM2CP model
- YOLO detection: `yolo_grpc_server.py` (port 18001), standalone, in-process inference
- Verified working.

### Intermediate Fusion (WorldFusion)
- Same actor path + feature extraction via `worldfusion_grpc_server.py` (port 18002)
- `grpc_ms ≈ server_total_ms ± 5-8ms` — IPC overhead fully eliminated
- Vehicle: ~130ms total e2e. RSU: ~140ms best case, ~200ms when GPU-contended with vehicle.
- RSU bimodal is pure CUDA serialization in distributed mode (two independent batch=1 calls). Not a transport issue.

### Infrastructure
- `start_actors.sh`: log-based container readiness (replaces static sleeps)
  - Edge: waits for `"registered successfully"` in docker logs
  - Vehicle/RSU: waits for `"Registered with"` (covers both edge and direct-to-orchestrator paths)

---

## Milestone: Sequential WorldFusion Unblocked (2026-04-06)

`openscenario_3_edge_worldfusion -l` (without `-d`) was deadlocked at "waiting for actors".

**Root cause**: `scenario_runner/scenario_runner.py:_prepare_ego_vehicles` only spawned the ego vehicle when `vehicle_index == 0` (the distributed-mode non-ego container convention). In sequential mode `vehicle_index = -2`, so the subprocess fell into the `else` branch and looped forever waiting for an ego nobody would spawn. This broke when `vehicle_index` was changed from `0` to `-2` in commit `3b9dcdb`.

**Fix**: Also spawn when `distributed=False`, regardless of `vehicle_index`. Distributed behavior unchanged. Committed to scenario_runner submodule (`34488e0`) and bumped in main repo (`925d526`).

**Status**: Proceeds past actor discovery. Secondary CARLA `rpc::timeout` errors on `_initialize_actors` appear on a dirty CARLA session (leftover state from the stuck run); clears on fresh CARLA restart.

**Verified**: `python ecav.py -t openscenario_3_edge_worldfusion --apply_ml` runs clean end-to-end. O5 batch=2 sequential path confirmed working.

---

## WIP / Exploratory

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

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
