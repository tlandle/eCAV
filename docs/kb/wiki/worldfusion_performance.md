---
updated: 2026-04-06
---
# WorldFusion Performance

Full record of the performance investigation and optimization of the WorldFusion intermediate
fusion perception path in distributed mode.

---

## Baseline

After initial distributed end-to-end working (2026-04-05), WorldFusion was functional but slow.

| Config | request_KB | transport_ms | total_e2e_ms |
|--------|-----------|-------------|--------------|
| Baseline (float32, LitServe HTTP) | 8021 | ~291ms | ~355ms |

Root cause: every actor tick sent the full float32 sensor batch over HTTP to LitServe.

---

## Payload Optimizations

### O1 — float16 spatial_features response
Return `spatial_features` as float16 instead of float32. Halves response payload (~10MB → ~5MB).
- Result: −40% http_ms (291→175ms). No detectable detection quality regression.

### O4 — uint8 imgs transport
CARLA captures uint8. `_build_batch()` was casting to float32 before sending. Send as uint8;
server casts back before camenc.
- Result: −53% request payload (8021→3803KB). Lossless.

### O5 — Edge-coordinated batch inference
In sequential mode, the edge can collect `build_batch()` outputs from all WF PMs before
sending a single merged request. Implemented in `_run_o5_batch_encoder()`.
- Result in **distributed mode**: `batch=1` — expected. Each actor lives in its own container;
  the edge process only holds proxy PMs (base PerceptionManager, not WF PM). O5 is
  architecturally a sequential-mode optimization.
- Result in **sequential mode**: `batch=2`; single forward pass for both agents.

### O8 — zlib compression on actor→server request
Applied `zlib.compress` on the msgpack payload before sending; `zlib.decompress` on receive.
- Result: reduces HTTP payload by ~30-50% on float16 activation maps.

After O1+O4+O8:

| Config | request_KB | transport_ms | total_e2e_ms |
|--------|-----------|-------------|--------------|
| +O1+O4+O8 | ~3803 | ~175ms | ~233ms |

---

## LitServe Parallelism Investigation (Options A/B/C)

Root cause of persistent high latency (~380ms RSU, ~175ms vehicle): identified as two problems:

**Problem 1 — SO_REUSEPORT sticky connections:** `num_api_servers=2` spawns two uvicorn workers.
With `--network=host`, both containers appear as `127.0.0.1` to the kernel. SO_REUSEPORT hashes
by `(src_ip, src_port, dst_ip, dst_port)` — combined with `requests.Session` keep-alive, both
containers landed on the same worker PID at startup and stayed there. Confirmed via `os.getpid()`
in logs.

**Problem 2 — CUDA stream serialization:** All GPU ops share stream 0 within a process.
Two concurrent `asyncio.to_thread()` calls serialized at the GPU level, not the HTTP level.

### Option A — Per-thread CUDA streams
Wrap all GPU ops in `with torch.cuda.stream(stream)`. Result: `stream_wait=0ms` in logs — GPU
was already serializing at the kernel level, not via stream contention. Sticky connection problem
remained. **No help.**

### Option B — Port-per-actor (ports 18000/18001)
Separate LitServe instances per actor. Server-side both fast (~80ms). Two problems:
1. OOM: LitServe `setup()` loaded model into spawn+uvicorn workers even though the custom
   `/extract_features` endpoint bypassed the native predict() pipeline. ~400MB × 2 wasted.
2. RSU still ~220ms slower due to `update_info()` blocking asyncio event loop.
   Fix applied: `await asyncio.to_thread(self.vehicle_manager.update_info)`.
**Rolled back to single server.**

### Option C — LitServe native pipeline with batching
`decode_request → batch → predict → unbatch → encode_response`; `max_batch_size=2`,
`batch_timeout=15ms`. batch=2 fired ~42% of ticks (110/261 requests).

- batch=1 inference: ~67ms server-side
- batch=2 inference: ~128ms server-side (2 agents, single forward pass)
- Client-observed http_ms: vehicle ~185-218ms — consistently ~106ms above server total

**Root cause of gap identified:** LitServe's `workers_per_device` uses `multiprocessing.Manager`
queues for IPC. Pickling 861KB payload + queue write + queue read both ways ≈ 100-120ms
irreducible overhead, larger than the actual inference time (~67ms).

**Conclusion:** LitServe is architecturally wrong for large binary payloads. The IPC hop
is irreducible. gRPC (same-process inference) is the exit.

---

## gRPC Migration (Final Solution)

Replaced LitServe HTTP transport with a standalone gRPC server. Inference runs directly
in the server process — no subprocess fork, no IPC queue.

### New files
- `ecav/ml_manager/worldfusion_servicer.py` — `WorldFusionServicer` implementing `ExtractWfFeatures`
- `ecav/ml_manager/worldfusion_grpc_server.py` — entry point; port 18002, `ThreadPoolExecutor(4)`

### Proto additions (`perception.proto`)
```proto
message WfRequest  { bytes payload = 1; int32 actor_id = 2; }
message WfResponse { bytes payload = 1; float unpack_ms = 2; float inference_ms = 3;
                     float pack_ms = 4; float total_ms = 5; }
rpc ExtractWfFeatures (WfRequest) returns (WfResponse);
```

### msgpack-in-bytes design
The batch dict has ~10 nested numpy arrays (voxel fields + optional image_inputs sub-dict).
Structured proto fields would require a `NumpyArray` message per field — significant proto churn.
The zlib+msgpack encoding was unchanged; only the transport layer changed.

### Verified results (2026-04-06, distributed mode)

| Agent | grpc_ms | srv_total_ms | gap | total_e2e_ms | vs LitServe |
|-------|---------|-------------|-----|--------------|-------------|
| Vehicle | 73-81ms | 68-77ms | 5-8ms | 125-143ms | −55ms |
| RSU (fast) | 82-90ms | 77-85ms | 5-8ms | 130-140ms | −240ms |
| RSU (slow) | 130-153ms | 126-148ms | 4-8ms | 190-206ms | improved |

Gap = pure gRPC framing overhead (no IPC). Hypothesis confirmed.

**RSU bimodal** explained: in distributed mode, each actor makes independent batch=1 gRPC calls.
When vehicle and RSU calls overlap, one waits for the GPU. This is pure CUDA serialization —
not a transport issue. Fix: O5 sequential mode (batch=2 combined forward pass).

### Dominant remaining cost
`pack_ms` (zlib+msgpack of 861KB) ≈ 50ms client-side. This is legitimate compute cost.
Potential future optimization: drop zlib compression on localhost (network integrity is handled
by gRPC framing; compression only helps on WAN). Not currently worth the complexity.

---

## Port Assignment

| Port  | Protocol | Service |
|-------|----------|---------|
| 18000 | HTTP/1.1 | LitServe (retained for local/debug only) |
| 18001 | gRPC | YOLO perception server |
| 18002 | gRPC | WorldFusion feature extraction server |

---

## Related

- [Decisions: D12](../decisions.md#d12-worldfusion-grpc-migration)
- [Plan: worldfusion_grpc_plan.md](../../agent_plans/worldfusion_grpc_plan.md)
- [Plan: litserve_parallelism_plan.md](../../agent_plans/litserve_parallelism_plan.md)
