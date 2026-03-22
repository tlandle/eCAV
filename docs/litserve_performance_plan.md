# LitServe Distributed Perception Performance Investigation

**Goal**: Quantify, diagnose, and optimize the latency gap between local per-actor YOLO inference (`--apply_ml`) and distributed LitServe inference (`-l`).

**Deployment assumption**: LitServe runs on a separate host from the simulation process. Co-located optimizations (shared memory, Unix domain sockets) are out of scope.

---

## Phase 0: Fix Pre-existing Measurement Bug

**File**: `ecav/core/sensing/perception/perception_manager.py:641`

Operator precedence error invalidates all existing `detections_time_list` data:

```python
# wrong — multiplies start_time by 1000, then subtracts from end_time
detection_end_time - detection_start_time * 1000

# correct
(detection_end_time - detection_start_time) * 1000
```

Fix this before any measurement work. All timing data collected prior to this fix is invalid.

---

## Phase 1: Measurement — Per-tick Latency Decomposition

**Goal**: Produce a per-tick breakdown with columns:

```
tick, mode, serialize_ms, http_ms, server_decode_ms, server_inference_ms, server_encode_ms, deserialize_ms, total_e2e_ms
```

Compare local vs. distributed across 300 ticks.

### 1a. Client-side instrumentation (`ml_manager.py`, `_detect_yolo_distributed()`)

Insert timestamps around each sub-step:

- **t0**: entry
- **t1**: after `msgpack.packb` → `serialize_ms = (t1-t0)*1000`
- **t2**: after `requests.post` returns → `http_round_trip_ms = (t2-t1)*1000`
- Parse `X-Server-*` headers from response (see 1b)
- **t3**: after `msgpack.unpackb` → `client_deserialize_ms = (t3-t2)*1000`
- **t4**: after `_parse_yolo_response()` → `parse_ms = (t4-t3)*1000`

Accumulate rows in a list on the `MLManager` instance; dump to CSV via `MLManager.close()` at scenario teardown.

### 1b. Server-side instrumentation (`litserve_models.py`, `/predict_msgpack`)

Add `X-Server-*` response headers (same pattern already used by `/extract_features`):

- `X-Server-Read-Ms`: body read time
- `X-Server-Decode-Ms`: msgpack unpack time
- `X-Server-Inference-Ms`: `model(images)` wall time
- `X-Server-Encode-Ms`: `encode_response` time
- `X-Server-Total-Ms`: total server-side wall time

### 1c. Local inference baseline (`ml_manager.py`, `detect()`)

Wrap `self.object_detector(rgb_images)` with `time.time()` and record to the same CSV schema, with serialization/HTTP/server columns set to 0.

### 1d. Image payload profiling

Log `len(packed)` plus per-image dimensions and dtype. A single 800×600×3 uint8 frame is ~1.44 MB raw; with N cameras the msgpack payload is ~N×1.44 MB. Quantify whether payload size correlates with `http_round_trip_ms`.

---

## Phase 2: Hypotheses and Diagnostic Tests

| # | Hypothesis | Test |
|---|-----------|------|
| H1 | **Serialization dominates**: msgpack packing of raw numpy arrays (~1.44 MB/camera) is the primary cost | Measure `serialize_ms`. If >20% of e2e, confirmed. |
| H2 | **No connection reuse**: bare `requests.post()` creates a new TCP connection per call, paying handshake overhead every tick | Confirmed by code inspection. Switch to `requests.Session` and measure delta. |
| H3 | **Double-copy on server**: `await request.body()` copies full payload into Python, then `msgpack.unpackb` copies again during decode | Measure `server_decode_ms`. If >15ms, confirmed. |
| H4 | **JSON response overhead**: `encode_response` calls `.cpu().numpy().tolist()` then serializes to JSON; client calls `response.json()` to parse back | Measure `server_encode_ms + client_deserialize_ms`. If >10ms combined, switch to msgpack response. |
| H5 | **`asyncio.to_thread` dispatch latency**: thread-pool dispatch overhead; CUDA work serializes through GIL regardless | Measure server total wall time vs. inference time. |

---

## Phase 3: Optimizations

Ranked by expected impact and implementation cost.

### O1. `requests.Session` with keep-alive
**Effort**: 15 min | **Impact**: Medium

**File**: `ecav/ml_manager/ml_manager.py`

Create `self._session = requests.Session()` during `_init_distributed()`. Replace bare `requests.post(...)` with `self._session.post(...)` in `_detect_yolo_distributed()`. Eliminates per-request TCP handshake — particularly significant for remote hosts where RTT is non-trivial.

### O2. JPEG-compress images before transmission
**Effort**: 1 hr | **Impact**: High

**Files**: `ecav/ml_manager/ml_manager.py` (client), `ecav/ml_manager/litserve_models.py` (server)

Raw 800×600×3 uint8 = 1.44 MB per image. JPEG at quality 85 → ~50–100 KB (10–30× reduction). Compress client-side with `cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])`; decode server-side with `cv2.imdecode`. YOLOv5's `autoShape` is robust to JPEG artifacts at quality ≥ 80.

Alternative: PNG for lossless compression (~500 KB, ~3× reduction).

Tradeoff: lossy compression may marginally affect detection confidence. Quantify via mAP comparison if detection accuracy is a research variable.

### O3. msgpack response instead of JSON
**Effort**: 30 min | **Impact**: Medium

**Files**: `ecav/ml_manager/litserve_models.py` (server), `ecav/ml_manager/ml_manager.py` (client)

`/predict_msgpack` previously accepted msgpack but responded with JSON. Changed to return `msgpack.packb(result)` with `media_type=application/octet-stream`; client parses with `msgpack.unpackb(response.content)`. The `/extract_features` endpoint already used this pattern.

### O4. Image codec optimization
**Effort**: 30 min – 2 hr | **Impact**: Medium–High

JPEG encode (~18ms client) and decode (~19ms server) are now the dominant CPU costs after O2. Three options, in priority order:

**O4a. Resize to YOLO inference resolution before encoding**
YOLOv5m internally letterboxes input to 640×640 regardless. Resizing 800×600 → 640×480 client-side before `cv2.imencode` reduces pixel count ~36%, directly reducing both encode time and payload size. Zero new dependencies. Server decode is proportionally faster. Tradeoff: double resize (client + YOLO letterbox), but the gain is linear with pixel reduction.

**O4b. libjpeg-turbo via `PyTurboJPEG`**
Drop-in replacement for `cv2.imencode`/`cv2.imdecode` using SIMD-accelerated libjpeg-turbo. Typically 2–4× faster than OpenCV's JPEG implementation on the same CPU at equivalent quality. No change to payload format, server, or downstream logic.

```python
from turbojpeg import TurboJPEG
jpeg = TurboJPEG()
buf = jpeg.encode(bgr_img, quality=85)      # client
img = jpeg.decode(buf)                       # server
```

**O4c. NVJPEG (GPU-accelerated codec)**
Encode on the client GPU, decode on the server GPU via `torchvision.io.decode_jpeg` / `pynvjpeg`. Eliminates both CPU encode and decode costs entirely. The GPU is already loaded for YOLO inference; JPEG decode can pipeline on a separate CUDA stream ahead of the forward pass. Highest impact but requires CUDA context on both client and server.

```python
# server decode (torchvision)
import torchvision.io as tvio
img_tensor = tvio.decode_jpeg(torch.frombuffer(jpeg_bytes, dtype=torch.uint8), device='cuda')
```

### O5. gRPC transport
**Effort**: Multi-day | **Impact**: Medium (single client), High (multi-client at scale)

The project already uses gRPC extensively (`ecav/protos/ecloud.proto`). Adding a YOLO inference RPC eliminates HTTP framing overhead entirely and provides HTTP/2 multiplexing — N distributed actors share a single connection, removing per-actor TCP connection overhead. For a single actor on localhost the latency gain is ~3–5ms; the benefit compounds at scale.

See `docs/grpc_perception_migration.md` for the full implementation plan.

Target deployment has many distributed clients, making HTTP per-request overhead significant at scale. gRPC follows O4 regardless of residual transport gap.

---

## Measured Results

| Run | serialize_ms (avg) | http_ms (avg) | server_inference_ms (avg) | total_e2e_ms (avg) |
|-----|-------------------|---------------|--------------------------|-------------------|
| Baseline (O1+O3) | 15.09 | 67.15 | 16.05 | 82.58 |
| +O2 JPEG | 17.84 (+2.75) | 50.05 (-17.10) | ~16 (unchanged†) | 68.24 (-14.34) |

†Prior to timing fix, JPEG decode (~19ms) was incorrectly attributed to `server_inference_ms`. After fix, `server_img_prep_ms` captures JPEG decode separately.

Net: O2 saves ~14ms e2e (-17%), driven by payload reduction. Encode cost (+3ms) and server decode (~19ms) are the remaining CPU costs targeted by O4.

---

## Status

| Item | Status |
|------|--------|
| Phase 0 bug fix | ✅ Done |
| Phase 1 instrumentation | ✅ Done |
| O1 — Session keep-alive | ✅ Done |
| O3 — msgpack response | ✅ Done |
| O2 — JPEG compression | ✅ Done |
| O4a — Resize before encode | Pending |
| O4b — libjpeg-turbo | Pending |
| O4c — NVJPEG | Pending |
| O5 — gRPC transport | In progress |

## Recommended Next Steps

1. **O4a** (resize before encode) — zero dependencies, implement and measure first
2. **O4b** (libjpeg-turbo) — measure independently of O4a to isolate codec speedup
3. **O5** (gRPC) — replace HTTP transport; see `docs/grpc_perception_migration.md`
4. **O4c** (NVJPEG) — evaluate after gRPC is in place; pairs naturally with GPU pipeline on server
