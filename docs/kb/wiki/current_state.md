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

**Root cause (final):** `num_api_servers=2` spawns two uvicorn workers, but SO_REUSEPORT distributes connections by `hash(src_ip, src_port, dst_ip, dst_port)`. Both containers use `--network=host` → both appear as `127.0.0.1` to the kernel. Combined with `requests.Session` keep-alive (sticky TCP connection), both containers landed on the same worker at startup and remained there. Confirmed via `os.getpid()` in log: both requests show the same PID despite two workers being spawned.

**Prior root cause (also real):** All GPU ops share CUDA stream 0 within a single process. Two concurrent `asyncio.to_thread()` calls serialize at the GPU level, not the HTTP level. FastAPI/uvicorn accepts both concurrently; the bottleneck is CUDA.

**Background:** YOLO is only used by the classic `PerceptionManager`. Neither `WorldFusionPerceptionManager` nor `BM2CPPerceptionManager` calls `ml_manager.detect()`. Stripping YOLO from `litserve_models.py` eliminated the parent-process CUDA init that forced `api_server_worker_type="thread"`. Phase 2 complete (`num_api_servers=N`, `WF_NUM_ACTORS`/`WF_MAX_WORKERS`, `--profile` mode).

**Three options — to be implemented in order and evaluated:**

| Option | Approach | Status |
|--------|----------|--------|
| A | Per-thread CUDA streams | **DONE — no help.** `stream_wait=0ms`; both containers still sticky to same worker. |
| B | Port-per-actor (ports 18000/18001) | **DONE — partially.** Server-side both fast (~80ms). But: (1) OOM from double model load in spawn+uvicorn workers; (2) RSU tick still ~220ms slower due to `update_info()` blocking asyncio event loop in `ecloud_actor_client.py`. **Rolled back to single server.** |
| C | LitServe native path — `decode_request → predict → encode_response`; `workers_per_device=1, max_batch_size=2, batch_timeout=15ms` | **IMPLEMENTED — pending verification run** |

**RSU event loop fix**: `ecloud_actor_client.py` line 425 — `update_info()` was a blocking `requests.post()` inside `async def tick()`. Fixed: `await asyncio.to_thread(self.vehicle_manager.update_info)`. Eliminates ~220ms asyncio event loop starvation per RSU tick.

**LitServe spawn worker bug** (found during Option B): `WorldFusionFeatureServer.setup()` was loading the model into LitServe's inference spawn worker even though the custom `/extract_features` endpoint bypasses the native predict() pipeline entirely. This wasted ~400 MB × 2 servers. Fix: `setup()` is now a no-op. For Option C, `setup()` becomes the correct place to load the model again (inference workers handle predict()).

**Verification target:** Both callers see `http≈80-120ms` (no CUDA queue wait).

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

## Option C Outcome (2026-04-06)

Option C implemented and tested. batch=2 fires ~42% of ticks (110/261 requests). Findings:

- **batch=1 inference**: ~67ms server-side. **batch=2 inference**: ~128ms server-side (2 agents, single forward pass).
- **Client-observed http_ms**: vehicle ~185-218ms, consistently ~106ms above server total. RSU data from Option C run not yet confirmed (docker logs show pre-C data only).
- **Root cause of the gap**: `multiprocessing.Manager` queue IPC. At 861KB payload, pickle + queue write + queue read both ways ≈ 100-120ms. This is irreducible in LitServe's `workers_per_device` architecture.
- **Conclusion**: Option C eliminated CUDA serialization but exposed IPC as the new bottleneck. LitServe is the wrong tool for large binary payloads. **gRPC is the exit.**

## In-Progress: WorldFusion gRPC Migration (Phases 1–5 Done)

Plan: [`worldfusion_grpc_plan.md`](../../agent_plans/worldfusion_grpc_plan.md)

**All implementation complete — pending verification run (Phase 6).**

Files changed:
- `ecav/protos/perception.proto` — added `WfRequest`, `WfResponse`, `ExtractWfFeatures` RPC; recompiled
- `ecav/ml_manager/worldfusion_servicer.py` — NEW: `WorldFusionServicer` (in-process, no IPC)
- `ecav/ml_manager/worldfusion_grpc_server.py` — NEW: standalone entry point; port 18002, `ThreadPoolExecutor(4)`
- `ecav/core/sensing/perception/worldfusion_perception_manager.py` — gRPC endpoint detection (3-priority), `_extract_features_grpc()`, `use_grpc` branch in `run_step()`, `http_ms` → `grpc_ms`, channel close in `close()`
- `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py` — `_run_o5_batch_encoder()` uses `stub.ExtractWfFeatures()` instead of `session.post()`
- `ecav/ml_manager/ml_manager.py` — `worldfusion_grpc_endpoint` attribute from config
- `start_actors.sh` — starts `worldfusion_grpc_server.py` (port 18002, gRPC readiness probe)
- `ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml` — `worldfusion_grpc_endpoint: 'localhost:18002'` in ml_manager + both worldfusion_model sections

**Verified (2026-04-06):** `grpc_ms ≈ server_total_ms + 5-8ms` across all ticks. IPC overhead (~106ms) fully eliminated.

| Agent | grpc_ms | total_e2e_ms | vs LitServe |
|-------|---------|--------------|-------------|
| Vehicle | 73-81ms | 125-143ms | −55ms |
| RSU (fast) | 82-90ms | 130-140ms | −240ms |
| RSU (slow, GPU serial) | 130-153ms | 190-206ms | improved |

RSU bimodal = two independent batch=1 gRPC calls racing the GPU in distributed mode. Not a transport issue — pure CUDA serialization. Fix is O5 sequential batch=2, lower priority.

## Immediate Next Steps

1. **YOLO regression**: Run late fusion scenario to confirm YOLO gRPC still works after proto recompile.
2. **O5 sequential test**: Run `openscenario_3_edge_worldfusion` without `-d`; batch=2 should fire and cut RSU inference ~50% by merging both agents' forward passes.

## TODO / Backlog

- **`start_actors.sh` determinism**: Replace timed delays between container startups with grep-able log signals from the main process. The main process should emit a known log line (e.g. `[sim_api] READY`) that `start_actors.sh` waits on before spinning up dependent containers.

---

## Recent Completions (this branch)

| Date | Item |
|------|------|
| 2026-04-06 | WorldFusion gRPC migration: phases 1–5 complete (proto, servicer, grpc_server, PM client, edge O5, start_actors, YAML) — pending verification run |
| 2026-04-06 | Option C implemented; batch=2 fires ~42% of ticks; IPC overhead (~106ms) identified as new bottleneck; gRPC migration planned |
| 2026-04-06 | `asyncio.to_thread` fix for RSU event loop blocking (`ecloud_actor_client.py` line 425) |
| 2026-04-06 | voxel_coords reindex fix in `_merge_wf_batches` (batch=2 scatter slot assignment) |
| 2026-04-06 | LitServe `_prepare_request` monkey-patch for binary body (`application/octet-stream`) |
| 2026-04-05 | Phase 2: WorldFusion-only LitServe server; YOLO stripped; `workers_per_device=N`; `--profile` mode |
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
