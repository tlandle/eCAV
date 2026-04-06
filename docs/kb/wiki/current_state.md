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

## Backlog

- **O5 sequential test**: Run `openscenario_3_edge_worldfusion` without `-d`. Expected: batch=2 fires, cutting RSU inference ~50% by merging both agents into a single forward pass. Low priority — distributed mode is already fast.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
