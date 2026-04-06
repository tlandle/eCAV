---
updated: 2026-04-05
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

## Milestone: Both Fusion Modes Working Distributed (2026-04-05)

Both intermediate fusion (WorldFusion) and late fusion (BM2CP) confirmed working end-to-end
in distributed mode on Scenario 3.

**What works:**
- Ego + RSU + non-ego containers all spawn, handshake, and tick correctly
- BM2CP late fusion: actor client → gRPC → C++ server → sim_api → edge_process → BM2CP model
- WorldFusion intermediate fusion: same path + feature extraction via LitServe HTTP, features forwarded via gRPC `pickled_features` field

---

## In-Progress Work

### WorldFusion Performance — LitServe Parallelism

Plan: [litserve_parallelism_plan.md](../../agent_plans/litserve_parallelism_plan.md)

**Root cause (diagnosed):** All GPU ops in `litserve_models.py` share CUDA stream 0. Two concurrent `asyncio.to_thread()` calls both submit GPU work, but it serializes on that stream. FastAPI/uvicorn does NOT block — the serialization is purely at the CUDA level. Result per tick:
- Vehicle (HOST process): served immediately → `http≈165ms`
- RSU (Docker container): queues behind vehicle → `http≈400ms` (~320ms CUDA wait + ~80ms inference)

**Key architectural finding:** YOLO is only used by the classic `PerceptionManager` (default backend). Neither `WorldFusionPerceptionManager` nor `BM2CPPerceptionManager` calls `ml_manager.detect()`. In a WorldFusion scenario, YOLO is loaded at LitServe startup and never called — it wastes GPU memory and, critically, initializes CUDA in the parent process before `server.run()`, which forces `api_server_worker_type="thread"` (single HTTP worker, no process-level parallelism). Stripping YOLO from `litserve_models.py` unblocks `workers_per_device=2` with no other structural changes.

**Parallelism plan — Phase 1 (per-thread CUDA streams):**
- Wrap `_extract_wf_features()` GPU ops in a `torch.cuda.Stream()` per call
- Minimal change; may not fully overlap if model is memory-bandwidth-bound (GPU has insufficient headroom)
- Verification: both callers should see `http≈100-120ms` instead of one at 165ms and one at 400ms

**Parallelism plan — Phase 2 (WorldFusion-only server + `workers_per_device=2`):**
- Strip `YOLOv5Server`, YOLO model loading, and gRPC startup from `litserve_models.py`
- Move WorldFusion model loading into `WorldFusionFeatureServer.setup()` (not parent `__main__`)
- Switch to `server.run()` default `"process"` mode + `workers_per_device=2`
- Two independent worker processes, each with their own CUDA context — no stream contention

See plan doc for full checklist including optional auto-batching (asyncio batch queue, no client changes needed).

### Previously Completed Payload Optimizations

- O1: `spatial_features` float16 response → −40% http_ms (291→175ms)
- O4: `imgs` uint8 → −53% request payload (8021→3803KB); lossless
- O5: Edge-coordinated batch inference — implemented; confirms `batch=1` in distributed mode (expected — vehicle WF PM lives in container, not root process). Sequential mode (`-d` off) gives `batch=2`.

### Bug Fixes Applied (2026-04-05)

- **RSUManager `run_distributed` parameter**: RSU was deriving `run_distributed` from `cav_world.run_distributed`, which reads the YAML `distributed:` key — always False even when `-d` is passed. Fixed by adding `run_distributed=False` explicit parameter to `RSUManager.__init__` and threading `self.run_distributed` from all three `sim_api.py` call sites (~L1013, ~L1383, ~L1495). Same pattern VehicleManager already used correctly.
- Proxy VM sensors: flat config key `camera_visualize` → nested `camera.visualize` in `vehicle_manager.py`
- Proxy RSU sensors: same nested key fix in `rsu_manager.py` + `activate=False`
- RSU 3-way LitServe contention: root-process RSU was calling LitServe as a third concurrent caller; fixed by `is_server_proxy = run_distributed` (was `run_distributed and not litserve`)

**Measured baseline (post O1+O4, LitServe hop only):**

| Config | request_KB | http_ms | total_e2e_ms |
|--------|-----------|---------|--------------|
| Baseline (float32) | 8021 | ~291ms | ~355ms |
| +O1+O4 | 3803 | ~175ms | ~233ms |
| O5 sequential (est.) | ~3803 | ~87ms/agent | TBD |

---

## Tyler Meeting — Resolved (2026-04-05)

Full notes in [`tyler_worldfusion_litserve_questions.md`](../../kb/raw/notes/tyler_worldfusion_litserve_questions.md).

**Key answers:**
- **Per-actor LitServe calls are correct and must stay.** Sending raw sensor data to the edge would be *early fusion* — a separate research area with known bandwidth problems. WorldFusion is intermediate fusion by design: actors run the sensor encoder locally, transmit only BEV features.
- **Custom FastAPI endpoint** was just a way to make it remote, not a deliberate design choice. Blocking behavior was untested in distributed mode.
- **`workers_per_device=N`** was never considered — distributed path was untested when built. It is the correct fix.
- **Compression bug** confirmed by Tyler: actor→LitServe HTTP path was missing `zlib`. Fixed (see below).

## Immediate Next Steps

1. **Phase 1**: Implement per-thread CUDA streams in `_extract_wf_features()` in `litserve_models.py`; add `stream_wait_ms` to log; run 30 ticks to verify both callers see ~100-120ms
2. **Phase 2**: Strip YOLO from `litserve_models.py`, move model load into `setup()`, enable `workers_per_device=N` where N = `min(gpu_headroom, num_actors_with_sensors)` — see design note in plan
3. **O5 sequential test**: Run `openscenario_3_edge_worldfusion` without `-d`; expect batch=2 and ~50% per-agent inference reduction

## TODO / Backlog

- **`start_actors.sh` determinism**: Replace timed delays between container startups with grep-able log signals from the main process. The main process should emit a known log line (e.g. `[sim_api] READY`) that `start_actors.sh` waits on before spinning up dependent containers.

---

## Recent Completions (this branch)

| Date | Item |
|------|------|
| 2026-04-05 | O8: zlib compression on actor→LitServe HTTP path (3 files: WF PM, edge manager O5, litserve server) |
| 2026-04-05 | Tyler meeting: architecture confirmed, compression bug confirmed, early fusion ruled out |
| 2026-04-05 | Diagnosed CUDA stream serialization as root cause of WorldFusion 400ms RSU latency |
| 2026-04-05 | Architectural finding: YOLO unused in WorldFusion/BM2CP; stripping it unblocks `workers_per_device=N` |
| 2026-04-05 | Created LitServe parallelism plan (Phase 1: CUDA streams, Phase 2: WorldFusion-only server) |
| 2026-04-05 | RSUManager `run_distributed` parameter fix — was reading YAML key (always False), now explicit from sim_api |
| 2026-04-05 | O5 implemented: edge-coordinated batch encoder in WorldFusionEdge + WF PM |
| 2026-04-05 | Fixed proxy VM + RSU sensor spawning (flat key → nested key) |
| 2026-04-05 | Fixed RSU `is_server_proxy` logic (3-way LitServe contention bug) |
| 2026-04-05 | **WorldFusion distributed working** — fixed missing zlib compression on gRPC pickled_features |
| 2026-04-05 | **BM2CP late fusion distributed confirmed** — full Scenario 3 smoke test passed |
| 2026-04-05 | Fixed edge logging silence, non-ego WDT, actors underground; scenario_3.py DRY cleanup |
| 2026-04-04 | Edge actor-ready handshake: gates tick loop on actor readiness |
| 2026-04-04 | Fixed proxy VM sensor crash, EcloudConfig.fatal_errors, perception_pb2 import |
| 2026-04-04 | Phase 6 (`sim_api.py`): `compute_edge_mappings()` + `Server_SetEdgeMappings` call |
| 2026-03-22 | O4: Send `imgs` as uint8 (−53% payload) |
| 2026-03-22 | O1: Send `spatial_features` as float16 |

## Uncommitted Modified Files

- `ecav/ecav2/ecloud_actor_client.py` — zlib.compress on pickled_features
- `ecav/scenario_testing/utils/sim_api.py` — zlib.decompress on pickled_features; RSUManager `run_distributed` param
- `ecav/core/common/vehicle_manager.py` — nested key fix for proxy VM sensors
- `ecav/core/common/rsu_manager.py` — nested key fix + activate=False + is_server_proxy fix + run_distributed param
- `ecav/core/sensing/perception/worldfusion_perception_manager.py` — O5: build_batch/apply_features; O8: zlib compress request
- `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py` — O5: batch encoder; O8: zlib compress request
- `ecav/ml_manager/litserve_models.py` — O8: zlib decompress on server
- `docs/kb/raw/sessions/2026-04-05.md` — session log

## Related

- [Plans Index](plans_index.md)
- [Research](research.md)
