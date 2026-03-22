# WorldFusion LitServe Enablement & Transport Optimization

**Goal**: Enable the `openscenario_3_edge_worldfusion` scenario to offload per-agent sensor encoding to the LitServe `/extract_features` endpoint, characterize transport latency, and apply applicable optimizations from the late-fusion LitServe investigation.

**Reference**: `docs/agent_plans/litserve_performance_plan.md` — late-fusion investigation; results and optimization decisions are directly referenced here where applicable.

---

## Implementation Checklist

**Phase 0 — Enable**
- [x] 1. Fix `ml_manager` YAML block — uncomment with correct `worldfusion_endpoint`
- [x] 2. Set camera capture resolution in YAML to match model `final_dim` (480×360, width×height)
- [x] 3. Refactor shared transport utilities into `ecav/ml_manager/litserve_transport.py` (`make_session`, `TimingRecorder`, numpy↔tensor helpers)
- [x] 4. Fix bare `requests.post()` in `_extract_features_remote()` — use `make_session()` from transport module
- [x] 5. Pre-load WorldFusion model at LitServe server startup (eliminate first-tick cold load)

**Phase 1 — Instrument**
- [x] 6. Consume all 5 server response headers in `_extract_features_remote()` (currently only reads 2 of 5: `Total` and `Inference`; missing `Read`, `Decode`, `Encode`)
- [x] 7. Accumulate per-tick timing rows in `WorldFusionPerceptionManager` — both distributed and local modes
- [x] 8. Add `close()` to `WorldFusionPerceptionManager`; call it explicitly from the scenario `finally` block (not via `CavWorld` — `scenario_manager.close()` does not propagate to perception managers)
- [x] 9. Call `edge.profiler.save_report()` from the scenario `finally` block — `EdgeProfiler` accumulates all edge-side metrics per tick but `save_report()` is never called from any scenario file; the JSON is never written
- [x] 10. Audit timing calculations for operator precedence bugs — none found
- [x] 11. Add `tools/analyze_wf_timing.py`

**Phase 1 — Baseline run**
- [x] 12. Measure: payload size, build_batch_ms, serialize_ms, http_ms, server sub-timings, total_e2e_ms per agent
- Observed: ~8MB request, ~10MB response (float32), ~291ms http_ms, ~355ms total_e2e_ms (n=168, vehicle)

**Phase 2 — Optimizations**
- [x] O1. Send spatial_features as float16 in response — **kept**; http_ms 291→175ms (-40%), total_e2e 355→234ms (-34%)
- [x] O2. ~~Eliminate zero `depth_map` from payload~~ — tried, no benefit (upload not bottleneck; reverted)
- [x] O3. ~~Cast `imgs` float32 → float16 before serialization~~ — tried, net negative (astype cost +2.5ms > http savings); reverted
- [x] O4. Send `imgs` as uint8 instead of float32 — **kept**; lossless, payload 8021→3803KB (-4.2MB, -53%). On loopback: total_e2e neutral (233→233ms), to_numpy_ms cost only 0.31ms. Kept because on Azure over real network links, 4MB × 2 agents × 20Hz = 160 MB/s avoided upload bandwidth per intersection.
- [ ] O5. Edge-coordinated batch inference — merge all agents' sensor data at the edge, send one HTTP request per tick instead of N sequential requests
- [ ] O6. Async/pipelined feature extraction — overlap build_batch + HTTP with vehicle tick loop
- [ ] O7. (Future) gRPC — `ExtractFeatures` RPC; see `grpc_perception_migration.md` Section 7

---

## Architecture Context

WorldFusion uses intermediate feature fusion: each agent (vehicle + RSU) runs the LiDAR/camera sensor encoder locally to extract `spatial_features` (before backbone), transmits those features to the edge, and the edge runs the Where2comm backbone + detection + tracking. The LitServe call is for the **sensor encoder** step only — the edge fusion path is always in-process and is not affected by LitServe transport.

This is distinct from the late-fusion YOLO path:
- YOLO: raw camera frames (JPEG-compressed) → LitServe → detection results
- WorldFusion: preprocessed voxel tensors + float32 camera tensors → LitServe → `spatial_features`

Both agents (vehicle and RSU) make independent LitServe calls per tick.

---

## Payload Structure and Size

`_build_batch()` constructs the following per agent per tick:

| Field | Shape | Dtype | Size |
|-------|-------|-------|------|
| `voxel_features` | (N_voxels, 32, 4) | float32 | variable (~sparse) |
| `voxel_coords` | (N_voxels, 4) | int32 | variable |
| `voxel_num_points` | (N_voxels,) | int32 | variable |
| `imgs` | (1, 4, 3, 360, 480) | float32 | **8.3 MB** |
| `depth_map` | (1, 4, 360, 480) | float32 | **2.8 MB** (all zeros — placeholder) |
| `rots`, `trans`, `intrins`, `post_rots`, `post_trans` | small geometry matrices | float32 | <1 KB total |
| `record_len` | scalar | int64 | negligible |

**Critical difference from YOLO**: the WorldFusion payload is dominated by float32 camera tensors (~11 MB per agent), not JPEG-compressed frames. With two agents (vehicle + RSU), the total inbound payload per tick is ~22 MB. YOLO after all optimizations was ~160 KB per actor.

The `depth_map` is currently a zero tensor (placeholder for a future learned depth estimation branch not yet implemented). It contributes 2.8 MB of constant-zero bytes to every request with no informational value.

Camera images are resized to (360, 480) in `_build_batch()` regardless of CARLA capture resolution (the model's `data_aug_conf.final_dim`). The CARLA camera resolution affects only local preprocessing quality, not LitServe payload size — unlike the YOLO scenario where native 640×480 capture was the key optimization.

---

## Phase 0: Enable

### Step 1 — YAML: Uncomment and fix `ml_manager` block

`ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml` currently has the ml_manager block commented out with stale pre-migration endpoints. Replace with:

```yaml
ml_manager:
  worldfusion_endpoint: 'http://localhost:18000'
```

The stale keys (`yolo_endpoint`, `worldfusion_vehicle_endpoint`, `worldfusion_edge_endpoint`) are from an abandoned distributed BM2CP design and should not be present.

**How the endpoint is consumed**: `CavWorld._init()` reads `ml_manager` config → `MLManager._init_distributed()` stores `self.worldfusion_endpoint`. `WorldFusionPerceptionManager.__init__` reads `cav_world.ml_manager.worldfusion_endpoint` when `-l` is active.

### Step 2 — YAML: Set camera capture resolution to match model `final_dim`

**File**: `ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml`

The WorldFusion YAML sets no `image_width`/`image_height` for either `vehicle_base` or `rsu_base` cameras, so CARLA captures at its sensor default (800×600). `_build_batch()` always resizes camera frames to the model's `final_dim` via `torch.nn.functional.interpolate` before packing the batch — currently [H=360, W=480] from the model config default. Capturing at 800×600 and resizing to 360×480 every tick wastes CPU work in `build_batch_ms` for no quality benefit.

Set cameras in both `vehicle_base` and `rsu_base` sensing blocks:

```yaml
camera:
  image_width: 480
  image_height: 360
```

**Impact**: Reduces `build_batch_ms` by eliminating the downscale from 800×600 → 360×480. Unlike the late-fusion case, this does **not** affect the LitServe payload size — `imgs` is always transmitted as a (1, 4, 3, 360, 480) float32 tensor regardless of capture resolution. The savings are purely local, on the client side.

**Constraint**: Confirm the model's actual `final_dim` from `config.yaml` when the WorldFusion checkpoint is available. The `_build_batch()` code hardcodes `[360, 480]` as the fallback default; the actual trained value must match. Using a wrong capture resolution here does not break correctness (the interpolate handles any input size) but will not achieve the intended cost reduction if `final_dim` differs.

This step is done before Phase 1 baseline measurement so the benchmark reflects the optimized-configuration cost, not the inflated resize overhead — same reasoning as the late-fusion 640×480 change.

### Step 4 — Fix bare `requests.post()` in `_extract_features_remote()`

**File**: `ecav/core/sensing/perception/worldfusion_perception_manager.py`

**Current state**: `_extract_features_remote()` calls `requests.post()` as a bare module function, importing `requests` locally on each call. This creates a new TCP connection per tick, paying a full handshake on every feature extraction request.

**Fix**: Initialize a `requests.Session` at construction time (same pattern O1 applied to ml_manager for YOLO before gRPC):

```python
# __init__: after self.use_litserve = True branch
import requests
self._session = requests.Session()

# _extract_features_remote(): replace
response = requests.post(...)
# with
response = self._session.post(...)
```

Add `self._session.close()` to a `close()` method or `__del__`.

This is the same fix that produced measurable gains in the YOLO path before gRPC migration. On the WorldFusion path with a 22 MB payload per request, the per-connection handshake overhead is proportionally smaller than for YOLO, but still wasteful.

### Step 5 — LitServe server: WorldFusion model pre-load

**File**: `ecav/ml_manager/litserve_models.py`

**Current state**: `load_wf_model()` is called lazily on the first `/extract_features` request. The first tick from each agent pays the full model load cost (~several seconds including checkpoint deserialization).

**Fix**: Call `load_wf_model()` at startup before `server.run()`, the same way the YOLO model is pre-loaded before the gRPC server starts:

```python
# In __main__, after YOLO model load:
load_wf_model()
print("[LitServe] WorldFusion model pre-loaded")
```

### Run command

```bash
python ecav.py -t openscenario_3_edge_worldfusion -l
```

`-l` implies `--apply_ml`. The LitServe server must be running first:

```bash
python ecav/ml_manager/litserve_models.py
```

---

## Phase 1: Measurement

**Goal**: Per-tick latency decomposition as a structured CSV for both distributed and local modes, enabling direct comparison and identification of the dominant cost.

### Current instrumentation state

**Client `_extract_features_remote()`** (`worldfusion_perception_manager.py`): Already has a detailed print covering 6 sub-timings per call — `to_numpy`, `pack`, `http`, `unpack`, `to_tensor`, `total` — plus inbound payload size in KB. However:
- Only reads 2 of 5 server headers: `X-Server-Total-Ms` and `X-Server-Inference-Ms`; `X-Server-Read-Ms`, `X-Server-Decode-Ms`, `X-Server-Encode-Ms` are sent by the server but not consumed
- Nothing is accumulated into a persistent record; all timing is lost at teardown

**Server `/extract_features`** (`litserve_models.py`): Already computes and returns all 5 sub-timing headers: `X-Server-Read-Ms`, `X-Server-Decode-Ms`, `X-Server-Inference-Ms`, `X-Server-Encode-Ms`, `X-Server-Total-Ms`. Also prints outgoing response payload size. No changes needed server-side.

**Client `run_step()`**: Prints `build_batch` and `extract_features` (total remote or local) ms. Not included in any persistent record.

**Edge `EdgeProfiler`**: Comprehensive — tracks `feature_collection`, `fusion`, `detection`, `tracking`, `prediction` with CUDA-synchronised `perf_counter` timing, GPU memory, and CPU utilization. Outputs JSON via `save_report()`. This is the edge-side counterpart; it runs in the edge manager process and is not connected to the client-side CSV.

### Step 4 — Add per-tick timing CSV to `_extract_features_remote()`

**File**: `ecav/core/sensing/perception/worldfusion_perception_manager.py`

All timestamps are already computed. The work is:

1. Read the remaining 3 server headers (`X-Server-Read-Ms`, `X-Server-Decode-Ms`, `X-Server-Encode-Ms`)
2. Accumulate a row per call into `self._timing_rows` (list of dicts), initialized in `__init__`
3. Dump to CSV at teardown

Target CSV schema:

```
tick, mode, agent_type, build_batch_ms, to_numpy_ms, pack_ms, http_ms,
server_read_ms, server_decode_ms, server_inference_ms, server_encode_ms,
server_total_ms, unpack_ms, to_tensor_ms, payload_bytes, response_bytes,
total_e2e_ms
```

Columns:
- `mode`: `'distributed'` or `'local'`
- `agent_type`: `'vehicle'` or `'rsu'` (already computed in `_extract_features_remote` and `run_step`)
- `build_batch_ms`: from `run_step()` — passed into the timing row at the `run_step` level, not inside `_extract_features_remote`
- `to_numpy_ms`, `pack_ms`: client-side serialization steps
- `payload_bytes`: `len(payload)` — inbound request size
- `response_bytes`: `len(response.content)` — spatial_features response size
- `server_*`: from response headers (all 5 already available)
- `unpack_ms`, `to_tensor_ms`: client-side deserialization steps
- `total_e2e_ms`: wall time from entry of `run_step()` to return, including `build_batch`

`tick` must come from `run_step()` (the perception manager doesn't currently track tick count — either accept it as a parameter from the caller or increment a counter on `self`).

### Step 5 — Add local mode rows to the same CSV

When `use_litserve=False`, `run_step()` currently prints `build_batch` and local `extract_features` timing and discards it. Add a timing row with the same schema, HTTP/server columns set to 0:

```
tick, mode='local', agent_type, build_batch_ms, to_numpy_ms=0, pack_ms=0,
http_ms=0, server_*=0, unpack_ms=0, to_tensor_ms=0,
payload_bytes=0, response_bytes=0, total_e2e_ms
```

This enables direct local vs. distributed comparison in the same CSV file, matching the pattern from late-fusion `ml_manager.py`.

### Step 6 — Dump CSV and EdgeProfiler report at teardown

**`WorldFusionPerceptionManager.close()`**

Add a `close()` method:

```python
def close(self):
    if self._timing_rows:
        import datetime
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs('logs', exist_ok=True)
        agent_tag = 'rsu' if self.vehicle is None else 'vehicle'
        path = os.path.join('logs', f'wf_timing_{agent_tag}_{ts}.csv')
        self._timing.dump_csv(path)  # via TimingRecorder
```

**Teardown path**: `scenario_manager.close()` does not propagate to perception managers — it only restores CARLA world settings. Call `close()` explicitly from the `finally` block in `openscenario_3_edge_worldfusion.py`, alongside the existing edge teardown loop:

```python
# In finally block, after the edge loop:
for edge in edge_list:
    for vm in edge.vehicle_manager_list:
        if hasattr(vm.perception_manager, 'close'):
            vm.perception_manager.close()
    for rsu in edge.rsu_list:
        if hasattr(rsu.perception_manager, 'close'):
            rsu.perception_manager.close()
```

**`EdgeProfiler.save_report()` — critical gap**

`EdgeProfiler` tracks all edge-side metrics per tick (feature_collection, fusion, detection, tracking, prediction timing; GPU memory; detection TP/FP/FN) but `save_report()` is never called from any scenario file. The accumulated JSON is silently discarded at teardown. Add to the `finally` block:

```python
for edge in edge_list:
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs('logs', exist_ok=True)
    edge.profiler.save_report(f'logs/edge_profiler_{ts}.json')
```

This is independent of the WorldFusion-specific client CSV — it is needed for all edge manager types and should be added to the base scenario teardown pattern.

### Step 7 — Timing bug check

Late-fusion Phase 0 found an operator precedence error in `perception_manager.py` invalidating all prior timing data. Audit `worldfusion_perception_manager.py` for the same class of bug before treating any measurement as valid. The existing timing computations in `run_step()` (lines 210–234) are correct as written, but verify `_extract_features_remote()` timestamps are all from the same `_t.time()` calls and not inadvertently multiplied before subtraction.

### Two output streams and how they relate

| Stream | Captures | Format | Granularity |
|--------|----------|--------|-------------|
| `wf_timing_{agent}_{ts}.csv` | Client feature extraction (per agent, per tick) | CSV | Per-tick |
| `EdgeProfiler` JSON | Edge fusion pipeline (feature collection, fusion, detection, tracking, prediction) | JSON | Per-edge-tick |

These are separate processes (or at minimum separate objects). Correlation is by `tick` value: the EdgeProfiler records `lag_steps` (AoI = edge_tick − source_tick). To reconstruct end-to-end latency from sensor capture to prediction distribution, join client CSV rows by tick to EdgeProfiler frames by `source_tick`.

### Analysis tooling

`tools/analyze_ml_timing.py` handles the YOLO CSV schema. WorldFusion uses a different schema (additional columns, two agents per tick). Either:
- Extend with a `--mode worldfusion` flag and separate column set, or
- Add `tools/analyze_wf_timing.py` as a parallel script

The EdgeProfiler JSON already has its own `get_summary()` / `save_report()` output path and does not need a new tool.

### Expected profile

Based on payload size, expected dominant costs (to be confirmed by measurement):

| Sub-step | Expected cost | Basis |
|----------|--------------|-------|
| `build_batch_ms` | 20–80ms | LiDAR voxelization + 4-camera resize to (360, 480) |
| `to_numpy_ms` | 5–20ms | Tensor copy + contiguous for 11 MB float32 |
| `pack_ms` | 10–50ms | msgpack serialization of 11 MB |
| `http_ms` (localhost) | 2–5ms | Loopback transfer of ~11 MB request + response |
| `server_decode_ms` | 10–50ms | msgpack unpack of 11 MB on server |
| `server_inference_ms` | unknown | Sensor encoder forward pass — first profiling target |
| `server_encode_ms` | 2–10ms | msgpack pack of spatial_features response (~smaller) |
| `unpack_ms` | 2–10ms | Client-side response unpack |

This is fundamentally different from YOLO (dominated by HTTP transport). WorldFusion serialization costs are expected to dominate due to float32 tensor payload volume.

---

## Phase 2: Optimizations

### O1 — Persistent `requests.Session` (covered in Phase 0 Step 2)

Direct application of late-fusion O1. Already required for correctness; listed here for completeness.

### O2 — Eliminate zero `depth_map` from payload ~~(tried; discarded)~~

**Implemented and measured. No observable benefit. Reverted.**

Stripped `depth_map` from the request on the client, reconstructed as zeros on the server from `imgs` shape. Payload dropped from ~8021KB to ~6146KB (-1875KB). Result:

- `http_ms`: 174.75 → 174.68ms (noise)
- `total_e2e_ms`: 233.54 → 233.74ms (noise)
- `pack_ms`: 1.34 → 0.91ms (real but immaterial)
- `server_read_ms`: 1.70 → 1.39ms (real but immaterial)

Root cause: the upload was never the bottleneck. Server reads the full request body in ~1.4ms regardless of whether it's 6MB or 8MB. `http_ms` is dominated by downloading the 5MB float16 response, which is unchanged. Adding client-strip + server-reconstruct complexity for zero e2e benefit is not justified. Change reverted.

### O3 — Cast `imgs` float32 → float16 before serialization ~~(tried; discarded)~~

**File**: `ecav/core/sensing/perception/worldfusion_perception_manager.py`

`imgs` is currently float32 (8.3 MB per agent). The model's camera encoder can accept float16 input (standard for mixed-precision inference). Cast before packing and cast back on the server before passing to the encoder:

```python
# client, before serialization:
batch_np['image_inputs']['imgs'] = batch_np['image_inputs']['imgs'].astype(np.float16)

# server, in _extract_wf_features, before encoder:
if 'imgs' in batch['image_inputs']:
    batch['image_inputs']['imgs'] = batch['image_inputs']['imgs'].float()
```

**Implemented and measured. Net negative. Reverted.**

Payload dropped from ~8021KB to ~5209KB (-2.75MB). Result:

- `to_numpy_ms`: 0.04 → 2.47ms (+2.43ms) — `astype(np.float16)` on an 8MB array costs ~2.5ms on the critical path
- `http_ms`: 174.75 → 173.25ms (noise — upload still not the bottleneck)
- `total_e2e_ms`: 233.54 → 238.33ms (slightly worse)

Same root cause as O2: the upload path costs ~1.4ms regardless of payload size. Paying a 2.5ms conversion cost to save time on a path that wasn't costing anything is net negative.

### O5 — Edge-coordinated batch inference

**Motivation**: Vehicle and RSU currently make independent sequential HTTP requests, each triggering a batch=1 forward pass through the sensor encoder. The GPU runs the full encoder twice, paying kernel launch overhead and weight cache misses twice. At batch=1 the 5080 is heavily underutilized — the encoder has many sequential layers making it latency-bound, not throughput-bound. A single batch=N forward pass costs roughly 80-90ms for N=2 vs 2×67=134ms for two sequential batch=1 passes (~35-45ms per agent effective cost). Scales further: 3 agents → one batch=3 pass instead of three sequential passes, and so on.

**Why Option B (edge-side aggregation) over server-side dynamic batching**: The edge already owns the coordination loop and calls all agents' `run_step()` sequentially. It has natural access to all vehicle and RSU perception managers. Server-side batching requires a request queue with a timeout that penalizes the first-arriving agent. Edge-side aggregation has no synchronization ambiguity.

**Required changes**:

*`WorldFusionPerceptionManager`*: Split `run_step()` into two phases:
- `build_batch()` — runs `_build_batch()`, stores result in `self._pending_batch`; pure CPU, no HTTP
- `apply_features(feature_dict)` — receives the spatial_features result from the edge, stores in `self.feature_dict` / `self.feature_map`

*Edge manager*: New method `_run_batched_encoder()`:
1. Call `build_batch()` on each vehicle and RSU perception manager
2. Merge batches — camera tensors stack directly; voxel data requires reindexing the batch_idx column of `voxel_coords` per agent (agent 0: batch_idx=0, agent 1: batch_idx=1, etc.); `record_len` = `[1] * N`
3. Send one HTTP POST to `/extract_features` with merged batch
4. Receive `(N, C, H, W)` spatial_features response
5. Split on dim 0 and call `apply_features()` on each agent

*Server `_extract_wf_features`*: No changes required. The model was trained on V2X-Sim with multi-agent batches and already handles `record_len=[1,1,...]` correctly. Output shape is `(N, C, H, W)` naturally.

**Voxel merging detail**: The only non-trivial piece. `voxel_features` and `voxel_num_points` concatenate directly. `voxel_coords` has shape `(K, 4)` where column 0 is the batch index (all zeros for a single-agent batch). For agent i, set `voxel_coords[:, 0] = i` before concatenation.

**Expected benefit**: ~50% inference cost reduction per agent at N=2 (current scenario). Scales sublinearly with N — batch=4 won't be 4× faster but will be significantly better than 4 sequential calls. This is the highest-leverage remaining optimization on the server-side compute path.

**Scaling note**: As the number of vehicles and RSUs per intersection grows, this optimization compounds. 4 vehicles + 1 RSU = 5 sequential 67ms calls (335ms) vs one batch=5 pass (~120ms estimated). The edge-side coordination cost is O(N) in build_batch (all CPU, parallelizable) and O(1) in HTTP calls and GPU inference.

---

### O4 — LiDAR-only mode (skip camera branch)

The WorldFusion model supports LiDAR-only input (the camera branch is guarded by `if 'image_inputs' in batch`). If the scene geometry at the blind intersection is well-covered by LiDAR alone, skipping image inputs eliminates the camera tensor payload entirely (saving 11 MB per agent per tick) and simplifies the server encoder path.

This is a research tradeoff, not a pure engineering optimization — detection quality with LiDAR-only vs. camera+LiDAR at the intersection needs evaluation. Document as an option to test after baseline measurements are available.

### O5 — gRPC transport (future)

As discussed for the YOLO path, gRPC provides HTTP/2 multiplexing and eliminates HTTP framing overhead. For WorldFusion the argument is weaker than for YOLO:

- WorldFusion runs at lower effective frequency (feature extraction is decoupled from the 20 Hz world tick via the jitter buffer)
- Two agents per scenario vs. N actors for late fusion — less concurrent pressure on the HTTP/1.1 connection
- Payload size means transport time dominates protocol overhead

The implementation path is documented in `grpc_perception_migration.md` Section 7 (`ExtractFeatures` RPC with `NumpyArray` message). Prioritize after O2/O3 are measured and if `http_ms` remains a significant fraction of e2e latency at the target deployment scale.

---

## DRY Opportunities: Shared Logic Between Late-Fusion and WorldFusion

The YOLO path (`ml_manager.py`) and WorldFusion path (`worldfusion_perception_manager.py`) share structural patterns that currently have duplicate implementations. These can be consolidated without major rearchitecting.

### Shared module: `ecav/ml_manager/litserve_transport.py`

Introduce a small module with three utilities used by both callers:

**1. Persistent session management**

Both callers need `requests.Session` lifecycle. Rather than duplicating the pattern, expose a shared factory and standard cleanup:

```python
def make_session() -> requests.Session:
    """Create a requests.Session with sensible defaults for LitServe calls."""
    s = requests.Session()
    adapter = requests.adapters.HTTPAdapter(max_retries=0)
    s.mount("http://", adapter)
    return s
```

Both `MLManager._init_distributed()` and `WorldFusionPerceptionManager.__init__` call `make_session()`. Behavior is identical; a future change (e.g., adding retry policy or auth headers) applies to both paths automatically.

**2. `TimingRecorder`**

Both paths accumulate per-tick timing rows and dump to CSV at teardown. Currently `MLManager` has `dump_timing_csv()` and the CSV schema hard-coded inline; `WorldFusionPerceptionManager` has only print statements. Extract into a shared class:

```python
class TimingRecorder:
    def __init__(self, fieldnames: list[str]):
        self.fieldnames = fieldnames
        self._rows: list[dict] = []

    def record(self, row: dict): ...
    def dump_csv(self, path: str): ...
```

`MLManager` replaces its inline `_dist_timing_rows` list + `dump_timing_csv()` with `self._timing = TimingRecorder(fieldnames=[...])`. `WorldFusionPerceptionManager` uses the same class with its own fieldnames. The schemas differ (different columns) but the record/dump lifecycle is identical.

**3. Numpy/tensor serialization helpers**

Both paths implement `_tensors_to_numpy` and its inverse. Currently:
- `_extract_features_remote()` defines `_tensors_to_numpy` as a local closure per call
- `_extract_wf_features()` on the server defines `_numpy_to_tensors` inline

Move to `litserve_transport.py`:

```python
def tensors_to_numpy(obj):
    """Recursively convert tensors in nested dicts to numpy arrays."""
    ...

def numpy_to_tensors(obj):
    """Recursively convert numpy arrays in nested dicts to tensors."""
    ...
```

Both client and server import from this module. Any future dtype handling (e.g., the float16 cast in O3) is applied in one place.

### Scope boundary

The transport utilities are limited to: session lifecycle, timing CSV, and numpy↔tensor conversion. They do not absorb inference logic, model loading, or protocol-specific details (msgpack framing, gRPC stubs). Each caller retains ownership of its serialization format and server interaction.

This is a modest refactor — three functions/classes extracted to one new file — that prevents the pattern from diverging further as both paths evolve.

---

## What Does NOT Apply from Late-Fusion Plan

| Late-fusion item | Applicability to WorldFusion |
|-----------------|------------------------------|
| O2 JPEG compression | Not applicable — payload is preprocessed float tensors, not raw camera frames |
| O3 msgpack response | Already done — `/extract_features` already returns msgpack |
| O4a Resize before encode | Not applicable — resize to (360, 480) already happens client-side before serialization |
| O4b TurboJPEG | Not applicable — no JPEG in WorldFusion transport path |
| O4c NVJPEG | Not applicable |
| Native 640×480 camera capture | Indirectly applicable (reduces local resize cost), but does not affect LitServe payload size |
| Phase 0 measurement bug fix | Check `worldfusion_perception_manager.py` timing calculations for operator precedence errors before trusting any measurement data |

---

## Risk Assessment

| Risk | Mitigation |
|------|-----------|
| First-tick model load latency | Pre-load WorldFusion model at LitServe startup (Step 3) |
| 22 MB payload saturates loopback on remote deployment | Measure payload size first; apply O2+O3 before declaring transport a bottleneck |
| float16 camera features degrade detection quality | Validate against local run on same scenario before enabling in production |
| LitServe `/extract_features` blocks uvicorn event loop | Already handled via `asyncio.to_thread()` in server; monitor for event loop starvation at high concurrency |
| Two agents × N ticks creates serialize/deserialize CPU pressure | Profile CPU utilization on client side; both agents share the same Python process |
