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

## Backlog

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
