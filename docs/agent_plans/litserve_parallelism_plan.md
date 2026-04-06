# LitServe WorldFusion Parallelism Plan

**Goal**: Enable true parallel inference for concurrent WorldFusion requests (vehicle + RSU), eliminating the ~400ms RSU latency (one caller always waits for the other).

**Root cause (final):** Two layers of serialization:
1. **CUDA stream**: all GPU ops in one process share stream 0; concurrent `asyncio.to_thread()` calls serialize at the GPU.
2. **SO_REUSEPORT + keep-alive**: `num_api_servers=2` spawns two uvicorn workers, but both containers use `--network=host` → both appear as `127.0.0.1`. SO_REUSEPORT hashes by `(src_ip, src_port, dst_ip, dst_port)`; same source IP means both can hash to the same worker. `requests.Session` keep-alive makes connections sticky. Confirmed: both callers share same PID in logs despite two workers running.

**Three options — implement in order, evaluate each:**

| Option | Approach | Hypothesis | Counter-hypothesis |
|--------|----------|------------|-------------------|
| A | Per-thread CUDA streams | GPU-level parallelism within single process; zero architecture change | WorldFusion is memory-bandwidth-bound; streams don't actually overlap |
| B | Port-per-actor | Guaranteed process isolation; no SO_REUSEPORT/keep-alive issue | 2× VRAM; scales linearly with actors |
| C | LitServe native path | Clean long-term; IPC queue routes to truly isolated worker processes | Most work; requires msgpack protocol restructuring |

**Verification target**: Both callers see `http_ms ≈ 80-120ms` (no queue wait).

---

## Background: Why the Bottleneck Exists

`/extract_features` is a custom FastAPI route using `asyncio.to_thread()`. Uvicorn accepts both incoming requests concurrently (async, no HTTP-level blocking). Both get submitted to Python's `ThreadPoolExecutor`. But PyTorch's default CUDA stream is **shared across all threads in one process** — GPU ops from both threads queue up on stream 0 and execute serially.

Result per tick:
- Caller A (vehicle): served immediately → `http=165ms` (80ms inference + 85ms overhead)
- Caller B (RSU): queues behind A → `http=400ms` (320ms CUDA wait + 80ms inference + some overhead)

`api_server_worker_type="thread"` was required because YOLO loads its model before `server.run()`, initializing CUDA in the parent. LitServe's default `"process"` mode forks uvicorn after CUDA init → "Cannot re-initialize CUDA in forked subprocess". Thread mode avoids this but means one HTTP worker and no process-level parallelism.

**Key finding**: YOLO is only used by the classic `PerceptionManager` (default backend). Neither WorldFusion nor BM2CP calls `ml_manager.detect()`. Stripping YOLO from `litserve_models.py` eliminated the parent-process CUDA init. Phase 2 is complete: `num_api_servers=N`, `WF_NUM_ACTORS`/`WF_MAX_WORKERS`, `--profile` mode.

**Why `num_api_servers=2` didn't fix it**: Both containers use `--network=host` Docker mode → both appear as source IP `127.0.0.1`. SO_REUSEPORT hashes connections to workers by `(src_ip, src_port, dst_ip, dst_port)`. With identical source IPs and `requests.Session` keep-alive (sticky TCP connections), both containers landed on the same uvicorn worker at startup and remained there. Confirmed via `os.getpid()` logging.

---

## Option A — Per-Thread CUDA Streams

Minimal change to `litserve_models.py`. Enables GPU-level parallelism within the current architecture. Tests the key unknown: is the WorldFusion encoder compute-bound (streams overlap → latency drops) or memory-bandwidth-bound (streams don't overlap → no improvement).

**Checklist:**

- [x] **A.1** In `_extract_wf_features()`, create `torch.cuda.Stream()`, wrap all CUDA ops in `with torch.cuda.stream(stream):`, time `wait_stream()`, return `(result, stream_wait_s)` tuple. Update `_process()` to unpack tuple; add `stream_wait_ms` to log and `X-Server-Stream-Wait-Ms` response header. Add `import time` at module level.

- [x] **A.2** `asyncio.to_thread()` default executor: `min(32, cpu_count+4)` — sufficient for 2 concurrent requests, no change needed.

- [x] **A.3** `stream_wait_ms` added to log line and response headers.

- [x] **A.4** Thread-safety audit: model in `.eval()` + `torch.inference_mode()`; BatchNorm read-only; in-place ReLU/depth_map mutations are on per-call tensors (not shared state). Safe for concurrent forward passes on same model instance.

- [x] **A.5** Result: RSU still ~400-500ms. `stream_wait=0ms` on all requests. Server log confirmed: PID 3223756 (worker B) received exactly 1 request total — one stray RSU hit caused by a different ephemeral port → different SO_REUSEPORT hash → keep-alive locked it back to worker A (PID 3223750). Both containers remain sticky to the same worker. Server `total` bimodal: 75ms (fast, no queue) vs 151ms (2× serial). **Option A does not fix the problem. Move to Option B.**

---

## Phase 2 — WorldFusion-Only Server (COMPLETE)

Stripped YOLO from `litserve_models.py`, eliminating the parent-process CUDA init that forced `api_server_worker_type="thread"`. Switched to `num_api_servers=N` for HTTP-level process parallelism. `workers_per_device` not used — the custom `/extract_features` endpoint bypasses LitServe's native predict() pipeline.

**Limitation discovered after Phase 2**: `num_api_servers=2` doesn't guarantee separate workers per actor when both containers use `--network=host`. See "Why `num_api_servers=2` didn't fix it" in Background above.

> **Note on YOLO**: If a future scenario requires both classic perception (YOLO gRPC) and WorldFusion HTTP simultaneously, create `ecav/ml_manager/litserve_yolo.py` as a standalone gRPC-only process. No current scenario requires this.

### `workers_per_device` Design

`N = min(WF_MAX_WORKERS, WF_NUM_ACTORS)`

**`WF_NUM_ACTORS`** (env var, set by `start_actors.sh`): number of actors that will call `/extract_features` per tick. Caps workers to what the scenario actually needs — no point loading 14 model copies for a 2-actor scenario.

**`WF_MAX_WORKERS`** (env var, optional): GPU headroom limit. If unset, defaults to `WF_NUM_ACTORS` (safe, no wasted VRAM). Set explicitly after profiling.

```python
# In __main__, before ls.LitServer():
num_actors  = int(os.environ.get('WF_NUM_ACTORS', '2'))
max_workers = int(os.environ.get('WF_MAX_WORKERS', str(num_actors)))
workers     = min(max_workers, num_actors)
server = ls.LitServer(api, workers_per_device=workers)
```

**VRAM estimates (WorldFusion model ~312MB, inference overhead ~2.5×):**

| GPU | Total VRAM | CARLA budget | Usable | Est. model+overhead | Max workers |
|-----|-----------|-------------|--------|---------------------|-------------|
| RTX 5080 | 16 GB | ~2-3 GB | ~13 GB | ~780 MB | ~14 (without CARLA) / ~10 (with CARLA) |
| A100 40GB | 40 GB | ~2-3 GB | ~37 GB | ~780 MB | ~47 |
| A100 80GB | 80 GB | ~2-3 GB | ~77 GB | ~780 MB | ~98 (compute-bound first) |

In practice for Scenario 3 (2 actors): `workers=2` regardless of GPU. Headroom matters when scaling to more actors or running multiple concurrent scenarios.

**`--profile` mode**: add a flag to `litserve_models.py` that spawns a subprocess to load the model, measure actual VRAM footprint, and print the `export WF_MAX_WORKERS=N` recommendation for the current GPU. The subprocess isolation is essential — CUDA init inside the subprocess dies with it, keeping the parent clean for the actual `server.run()`.

```bash
python ecav/ml_manager/litserve_models.py --profile
# [Profile] GPU: NVIDIA GeForce RTX 5080 (16376 MB)
# [Profile] WorldFusion model footprint: 312 MB
# [Profile] Per-worker estimate: 780 MB
# [Profile] Recommended WF_MAX_WORKERS: 14  (10 with CARLA running)
# [Profile] Export: export WF_MAX_WORKERS=10
```

### Step 1: Strip YOLO, clean CUDA init

- [x] **2.1** Remove `YOLOv5Server`, `_yolo_model`, gRPC startup (`PerceptionServicer`, `grpc.server(...)`) from `litserve_models.py`. The file becomes WorldFusion HTTP only.

- [x] **2.2** `WorldFusionFeatureServer.setup(device)` calls `load_wf_model()` — each spawned worker loads its own model instance after spawn. Global `_wf_model`/`_wf_device` remain but are per-process in spawned workers.

- [x] **2.3** `__main__` does NOT call `load_wf_model()`. No CUDA init in parent. Workers call `setup()` after spawn.

- [x] **2.4** `server.run(host=host, port=port)` — default `"process"` mode. No `api_server_worker_type="thread"`.

- [x] **2.5** Reads `WF_NUM_ACTORS` and `WF_MAX_WORKERS` env vars; `workers = min(max_workers, num_actors)`; passed to `server.run(num_api_servers=workers)`. Note: `workers_per_device` (LitServe inference worker queue) is NOT used — the custom `/extract_features` endpoint bypasses LitServe's native predict() pipeline. `num_api_servers` is the correct parameter for HTTP-level process parallelism.

- [x] **2.6** `--profile` mode: spawns `--profile-worker` subprocess, loads model, measures VRAM, prints `WF_MAX_WORKERS` recommendation for current GPU + with-CARLA estimate.

### Step 3: Auto-batching via custom `/extract_features` batch queue (optional but recommended)

Rather than migrating to LitServe's native request path (which requires restructuring the msgpack protocol to match LitServe's JSON pipeline), implement application-level batching in the custom route.

- [ ] **2.6** Add a shared batch collector to the FastAPI app state:
  ```python
  # In server setup:
  server.app.state.pending = asyncio.Queue()
  
  @server.app.post("/extract_features")
  async def extract_features(request: Request):
      body = await request.body()
      loop = asyncio.get_event_loop()
      future = loop.create_future()
      await request.app.state.pending.put((body, future))
      return await future  # waits for batch processor to resolve
  
  @asynccontextmanager
  async def lifespan(app):
      asyncio.create_task(_batch_loop(app))
      yield
  
  async def _batch_loop(app):
      BATCH_TIMEOUT = 0.015  # 15ms
      MAX_BATCH = 2
      while True:
          items = []
          try:
              item = await asyncio.wait_for(app.state.pending.get(), timeout=BATCH_TIMEOUT)
              items.append(item)
              # Drain any additional items without waiting
              while len(items) < MAX_BATCH:
                  try:
                      items.append(app.state.pending.get_nowait())
                  except asyncio.QueueEmpty:
                      break
          except asyncio.TimeoutError:
              continue
          
          # Process batch
          bodies = [i[0] for i in items]
          futures = [i[1] for i in items]
          payload, timing = await asyncio.to_thread(_process_batch, bodies)
          payloads = split_response(payload, len(items))  # split spatial_features on dim 0
          for f, p in zip(futures, payloads):
              f.set_result(Response(content=p, media_type="application/octet-stream", headers=timing))
  ```

- [ ] **2.7** Implement `_process_batch(bodies)`: unpack all bodies, merge via `_merge_wf_batches()` (already implemented in the edge manager — extract to shared utility), call `_extract_wf_features(merged)`, split `spatial_features[i:i+1]` per request.

- [ ] **2.8** No client changes needed — `/extract_features` endpoint and msgpack protocol unchanged.

### Step 4: Deployment

- [ ] **2.10** Update `start_actors.sh` to start `litserve_models.py` (now WorldFusion-only). Remove any YOLO gRPC readiness check that's no longer applicable.

- [ ] **2.11** Ensure `WF_HYPES_YAML` and `WF_CHECKPOINT` env vars are set for the WorldFusion server process (currently hardcoded in `load_wf_model()`, needs to move to `setup()` config).

---

## Option B — Port-Per-Actor (guaranteed separation)

Run two LitServe instances on different ports. Each actor's `WF_LITSERVE_ENDPOINT` env var points to its dedicated port. Bypasses SO_REUSEPORT/keep-alive issue entirely — each container has an exclusive server process.

**Tradeoff**: 2× VRAM (~1.3GB × 2 = ~2.6GB for 2 actors). Scales linearly. Fine for 2-4 actors on a 16GB GPU with CARLA.

**Checklist:**

- [x] **B.1** `start_actors.sh`: starts `PORT=18000 WF_NUM_ACTORS=1` for vehicles and `PORT=18001 WF_NUM_ACTORS=1` for RSUs (if num_rsu > 0). Waits for both to be ready. Cleanup handles both PIDs + pkill.

- [x] **B.2** `-e WF_LITSERVE_ENDPOINT=http://127.0.0.1:18000` on vehicle containers; `-e WF_LITSERVE_ENDPOINT=http://127.0.0.1:18001` on RSU containers.

- [x] **B.3** `worldfusion_perception_manager.py`: `WF_LITSERVE_ENDPOINT` env var is now priority 0 (before YAML config and -l flag). `edge_manager` not changed — in distributed mode actors call LitServe directly; edge manager only uses litserve in O5 sequential batch mode.

- [ ] **B.4** Run 30 ticks, compare `http_ms`. Expected: both callers ~80ms (each hits its own dedicated server process).

- [ ] **B.5** If both at ~80ms: Option B is the production fix. Skip Option C unless auto-batching becomes a goal.

---

## Option C — LitServe Native Path (clean long-term, most work)

Migrate `/extract_features` to LitServe's `decode_request → predict → encode_response` pipeline. Requests route through LitServe's internal IPC queue to inference workers — truly isolated processes regardless of source IP. Enables native `workers_per_device=N` and `max_batch_size` auto-batching.

**Key challenge**: LitServe's native path expects JSON-serializable input by default. Our clients send raw `application/octet-stream` msgpack. The decode/encode pipeline would need:
- `decode_request(bytes) → dict`: msgpack decode
- `batch(list[dict]) → merged_dict`: custom merge (override default — default `torch.stack` won't handle variable-length voxels)
- `predict(merged_dict) → output_dict`: calls `_extract_wf_features`
- `unbatch(output_dict) → list[dict]`: split `spatial_features` on dim 0
- `encode_response(dict) → bytes`: msgpack encode

The `batch()` override is critical — voxel_features are variable-length and must be `np.concatenate`'d, not stacked. `record_len` must be concatenated so PointPillarScatter knows which voxels belong to which agent.

**Checklist:**

- [x] **C.1** Override `decode_request(self, request: bytes)` — annotated as `bytes` so LitServe passes raw body (no JSON parse). zlib-decompress (try/except fallback) then msgpack.unpackb.

- [x] **C.2** Override `batch` calling `_merge_wf_batches()` — new module-level helper. Concatenates variable-length voxel arrays; no voxel_coords re-indexing needed (PointPillarScatter uses record_len to separate samples).

- [x] **C.3** `predict(self, x)` calls `_extract_wf_features(x)` directly — no `asyncio.to_thread()` needed; LitServe inference worker is a subprocess, blocking is fine.

- [x] **C.4** Override `unbatch` to split `spatial_features[i:i+1]` per request.

- [x] **C.5** Override `encode_response` returning `starlette.responses.Response(content=msgpack_bytes, media_type="application/octet-stream")`. Confirmed picklable across process boundary.

- [x] **C.6** Removed custom `@server.app.post` route and `@server.app.on_event("startup")` preload. `setup()` now loads model in inference worker.

- [x] **C.7** `LitAPI(max_batch_size=2, batch_timeout=0.015, api_path='/extract_features')` + `LitServer(api, workers_per_device=workers)`. Worker count formula: `ceil(num_actors / max_batch_size)` — ensures requests batch together rather than race to separate workers.

- [x] **C.8** No client changes — endpoint URL `/extract_features` and msgpack protocol unchanged. `WF_LITSERVE_ENDPOINT` env var still honoured.

- [x] **C.9** Ran ~261 requests (130 ticks). batch=2 fires 42% of ticks (110/261). Server: batch=1 inference ~67ms, batch=2 ~128ms. **Client http_ms: ~185-218ms** — consistently ~106ms above server total. IPC overhead from `multiprocessing.Manager` queue (pickle 861KB, both directions) is ~100-120ms and irreducible in this architecture. Option C eliminated CUDA serialization but the IPC cost now dominates. **Exit: gRPC migration. See [worldfusion_grpc_plan.md](worldfusion_grpc_plan.md).** Expected grpc_ms ≈ server_total_ms ≈ 75-85ms with no IPC hop.

---

## LitServe Native Path Migration (incorporated into Option C above)

---

## Auto-batching Merge Logic (for either approach)

When merging two agents' batches into batch=2:

```python
def _merge_wf_batches(batch_list):
    N = len(batch_list)
    pc_list = [b['processed_lidar'] for b in batch_list]
    merged_pc = {
        'voxel_features':    np.concatenate([pc['voxel_features'] for pc in pc_list]),
        'voxel_coords':      np.concatenate([pc['voxel_coords'] for pc in pc_list]),
        'voxel_num_points':  np.concatenate([pc['voxel_num_points'] for pc in pc_list]),
    }
    merged = {
        'processed_lidar': merged_pc,
        'record_len': np.concatenate([b['record_len'] for b in batch_list]),
    }
    if batch_list[0].get('image_inputs') is not None:
        merged['image_inputs'] = {
            k: np.concatenate([b['image_inputs'][k] for b in batch_list], axis=0)
            for k in batch_list[0]['image_inputs']
        }
    return merged
```

The output `spatial_features` will have shape `[N, C, H, W]`. Split with `features[i:i+1]` per request.

---

## Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| WorldFusion encoder is memory-bandwidth-bound; CUDA streams don't overlap (Option A) | Medium | Check `stream_wait_ms` in logs; if near-zero, move to Option B |
| `pillar_vfe` has thread-unsafe in-place ops (Option A) | Low | Audit before A.1; add per-call `threading.Lock` as fallback |
| LitServe spawn workers take 5-10s to load model | Certain | Expected; `server.run()` blocks until READY; use watchdog curl in `start_actors.sh` |
| `record_len` merge produces wrong scatter output (Option C) | Medium | Unit test: single batch vs concatenated batch should give identical per-agent spatial_features |
| asyncio batch loop race on future resolution (Option C) | Low | Use `loop.call_soon_threadsafe` if `_process_batch` runs in executor thread |
| SO_REUSEPORT hash collision persists even with different ports (Option B) | None | Different ports = different server processes entirely; hash collision not applicable |

---

## Related

- [worldfusion_litserve_plan.md](worldfusion_litserve_plan.md) — payload optimizations O1–O12
- [tyler_worldfusion_litserve_questions.md](../kb/raw/notes/tyler_worldfusion_litserve_questions.md) — architectural questions for Tyler
