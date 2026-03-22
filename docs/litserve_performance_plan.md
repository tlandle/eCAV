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

### O4. Remove `asyncio.to_thread` for single-client case
**Effort**: 15 min | **Impact**: Low–Medium

**File**: `ecav/ml_manager/litserve_models.py`, `/predict_msgpack` handler

In the sequential (non-distributed actor) scenario there is exactly one client. The `to_thread` dispatch adds ~0.1–0.5ms and does not improve CUDA throughput since the GIL serializes CPU-side preprocessing regardless. Gate behind `LITSERVE_SINGLE_CLIENT=1` environment variable.

### O5. gRPC transport (longer term)
**Effort**: Multi-day | **Impact**: High

The project already uses gRPC extensively (`ecav/protos/ecloud.proto`). Adding a YOLO inference RPC eliminates HTTP framing overhead entirely, provides built-in flow control, and enables more efficient binary serialization. Combine with O2 (JPEG) for maximum gain.

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

Pursue only if O1–O4 are insufficient to close the gap.

---

## Status

| Item | Status |
|------|--------|
| Phase 0 bug fix | ✅ Done |
| Phase 1 instrumentation | ✅ Done |
| O1 — Session keep-alive | ✅ Done |
| O3 — msgpack response | ✅ Done |
| O2 — JPEG compression | Pending data analysis |
| O4 — Remove `to_thread` | Pending data analysis |
| O5 — gRPC transport | Longer term |

## Recommended Next Steps

1. **Analyze Phase 1 CSV** — identify which sub-component (`serialize_ms`, `http_ms`, `server_inference_ms`) dominates
2. **Apply O2** (JPEG) if payload serialization/transport dominates
3. **Apply O4** (remove `to_thread`) — low risk, evaluate independently
4. **Evaluate** — if gap remains significant, proceed to O5 (gRPC)
