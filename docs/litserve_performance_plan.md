# LitServe Distributed Perception Performance Investigation

**Goal**: Quantify, diagnose, and optimize the latency gap between local per-actor YOLO inference (`--apply_ml`) and distributed LitServe inference (`-l`).

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
- **t3**: after `response.json()` → `client_deserialize_ms = (t3-t2)*1000`
- **t4**: after `_parse_yolo_response()` → `parse_ms = (t4-t3)*1000`

Accumulate rows in a list on the `MLManager` instance; dump to CSV at scenario teardown.

### 1b. Server-side instrumentation (`litserve_models.py`, `/predict_msgpack`)

Add `X-Server-*` response headers (same pattern already used by `/extract_features`):

- `X-Server-Read-Ms`: body read time
- `X-Server-Decode-Ms`: msgpack unpack time
- `X-Server-Inference-Ms`: `model(images)` wall time
- `X-Server-Encode-Ms`: `encode_response` time
- `X-Server-Total-Ms`: total server-side wall time

Break `_yolo_process` into finer steps to separate decode, inference, and encode.

### 1c. Local inference baseline (`ml_manager.py`, `detect()`)

Wrap `self.object_detector(rgb_images)` with `time.time()` and record to the same CSV schema, with serialization/HTTP columns set to 0.

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
| H5 | **`asyncio.to_thread` dispatch latency**: thread-pool dispatch overhead; CUDA work serializes through GIL regardless | Measure server total wall time vs. inference time. If overhead visible at single-client load, remove `to_thread`. |
| H6 | **TCP loopback overhead**: even on localhost, TCP/IP stack processing (Nagle, segment ACK) adds fixed per-request latency | Compare against Unix domain socket transport (O4). |

---

## Phase 3: Optimizations

Ranked by expected impact and implementation cost.

### O1. `requests.Session` with keep-alive
**Effort**: 15 min | **Impact**: Medium

**File**: `ecav/ml_manager/ml_manager.py`

Create `self._session = requests.Session()` during `_init_distributed()`. Replace bare `requests.post(...)` with `self._session.post(...)` in `_detect_yolo_distributed()`. Eliminates per-request TCP handshake (~1–3ms on localhost, higher on remote hosts).

### O2. JPEG-compress images before transmission
**Effort**: 1 hr | **Impact**: High

**Files**: `ecav/ml_manager/ml_manager.py` (client), `ecav/ml_manager/litserve_models.py` (server)

Raw 800×600×3 uint8 = 1.44 MB per image. JPEG at quality 85 → ~50–100 KB (10–30× reduction). Compress client-side with `cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])`; decode server-side with `cv2.imdecode`. YOLOv5's `autoShape` is robust to JPEG artifacts at quality ≥ 80.

Alternative: PNG for lossless compression (~500 KB, ~3× reduction).

Tradeoff: lossy compression may marginally affect detection confidence. Quantify via mAP comparison if accuracy is a research variable.

### O3. msgpack response instead of JSON
**Effort**: 30 min | **Impact**: Medium

**Files**: `ecav/ml_manager/litserve_models.py` (server), `ecav/ml_manager/ml_manager.py` (client)

`/predict_msgpack` currently accepts msgpack but responds with JSON. Change `encode_response` to return msgpack-packed data; return `Response(content=packed, media_type="application/octet-stream")`. Client replaces `response.json()` with `msgpack.unpackb(response.content)`. The `/extract_features` endpoint already follows this pattern — replicate it.

### O4. Unix domain socket transport
**Effort**: 1 hr | **Impact**: Medium

**Files**: `ecav/ml_manager/litserve_models.py` (server startup), `ecav/ml_manager/ml_manager.py` (client endpoint config)

When server and client are co-located, replace TCP `localhost:18000` with a UDS path (e.g., `/tmp/ecav_litserve.sock`). Eliminates TCP/IP stack overhead: no Nagle delay, no loopback interface, no port allocation. Use `requests_unixsocket` or `httpx` with UDS support.

Degrades gracefully: if config specifies `host:port`, use TCP; if it specifies a socket path, use UDS. Remote deployments are unaffected.

### O5. Remove `asyncio.to_thread` for single-client case
**Effort**: 15 min | **Impact**: Low–Medium

**File**: `ecav/ml_manager/litserve_models.py`, `/predict_msgpack` handler

In the sequential (non-distributed actor) scenario there is exactly one client. The `to_thread` dispatch adds ~0.1–0.5ms and does not improve CUDA throughput since the GIL serializes CPU-side preprocessing regardless. Gate behind `LITSERVE_SINGLE_CLIENT=1` environment variable; call `_yolo_process` directly when set.

### O6. gRPC transport (longer term)
**Effort**: Multi-day | **Impact**: High

The project already uses gRPC extensively (`ecav/protos/ecloud.proto`). Adding a YOLO inference RPC eliminates HTTP overhead entirely, provides built-in flow control, and enables protobuf-based serialization. Combine with O2 (JPEG) for maximum gain.

Candidate proto schema:
```protobuf
service PerceptionService {
  rpc DetectYOLO (YOLORequest) returns (YOLOResponse);
}
message YOLORequest {
  repeated bytes jpeg_images = 1;
}
message YOLOResponse {
  repeated Detection detections = 1;
}
```

Pursue only if O1–O4 are insufficient.

### O7. Shared-memory IPC (longer term)
**Effort**: Multi-day | **Impact**: Highest (co-located only)

Write image frames to a `multiprocessing.shared_memory` segment; pass only the segment name and metadata over the control channel. Server maps the segment and reads at zero copy. Degrades gracefully to serialized transport when client and server are on different hosts (gated by config flag).

---

## Recommended Execution Order

1. **Fix Phase 0 bug** — prerequisite for valid data (2 min)
2. **Apply O1** (Session keep-alive) — cheap, do before first profiling run (15 min)
3. **Apply O3** (msgpack response) — cheap, eliminates a known inefficiency (30 min)
4. **Add Phase 1 instrumentation** — collect baseline numbers with O1+O3 already in place (1–2 hr)
5. **Analyze data** — determine whether H1 (serialization) or H6 (TCP) dominates
6. **Apply O2** (JPEG) if payload size dominates, **O4** (UDS) if transport overhead dominates
7. **Apply O5** (remove `to_thread`) — low risk, minor gain
8. **Evaluate** — if gap remains significant, proceed to O6 (gRPC)
