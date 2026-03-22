# Migration Plan: Distributed Perception Transport from HTTP/msgpack to gRPC

## Implementation Checklist

- [x] 1. Define `ecav/protos/perception.proto`
- [x] 2. Compile proto → `perception_pb2.py`, `perception_pb2_grpc.py`
- [x] 3. Implement `ecav/ml_manager/perception_servicer.py`
- [x] 4. Update `ecav/ml_manager/litserve_models.py` — remove `/predict_msgpack`, add gRPC server startup
- [x] 5. Update `ecav/ml_manager/ml_manager.py` — gRPC channel, replace `_detect_yolo_distributed()`, remove `requests`/msgpack YOLO path, update `close()`
- [x] 6. Update `_parse_yolo_response()` to accept proto `YoloResponse`
- [x] 7. Rename `http_ms` → `rpc_ms` in timing CSV schema
- [x] 8. Update `tools/analyze_ml_timing.py` — rename `NUMERIC_COLS` entry
- [x] 9. Update `ecav/scenario_testing/config_yaml/*.yaml` — `yolo_endpoint: 'localhost:18001'`
- [x] 10. Update `opencda.py` — add `perception.proto` to `--build` step
- [x] 11. Integration test — single-actor scenario (21.14ms avg e2e, rpc_ms replaces http_ms, client_deserialize_ms ≈ 0)
- [ ] 12. Multi-actor test — 4+ actor scenario
- [ ] 13. (Future) Migrate WorldFusion `/extract_features` to `ExtractFeatures` RPC — see Section 7

---

## Motivation

The current distributed perception pipeline uses HTTP POST with msgpack serialization over a `requests.Session`. Each actor maintains a persistent HTTP/1.1 connection to the LitServe server. Under concurrent multi-actor load, HTTP/1.1 incurs head-of-line blocking per connection; actors that share a connection pool must serialize requests. gRPC over HTTP/2 provides native multiplexing, binary framing, and trailing metadata — removing the need for custom timing headers while improving throughput under concurrent actor workloads.

---

## 1. Proto Schema Design

**Decision: Create a new `ecav/protos/perception.proto` rather than extending `ecloud.proto`.**

`ecloud.proto` defines the orchestration control plane (ticks, registration, waypoints). Perception inference is a separate data-plane concern with different service endpoints and potentially a different host process. Keeping the proto files separate preserves modularity and avoids recompiling orchestration stubs when perception messages change.

```protobuf
syntax = "proto3";
package perception;

// Request: one or more JPEG-compressed camera frames
message YoloRequest {
  repeated bytes jpeg_frames = 1;   // Pre-compressed JPEG bytes (cv2.imencode)
  int32 actor_id = 2;               // For server-side logging/metrics
}

// A single detection box: [x1, y1, x2, y2, confidence, class_id]
message Detection {
  repeated float box = 1;
}

// Per-image detection results
message ImageDetections {
  repeated Detection detections = 1;
  map<int32, string> class_names = 2;
  string summary = 3;
}

// Response: detections per input image + server-side timing breakdown
message YoloResponse {
  repeated ImageDetections per_image = 1;
  float decode_ms = 2;
  float img_prep_ms = 3;
  float inference_ms = 4;
  float encode_ms = 5;
  float total_ms = 6;
}

service PerceptionService {
  rpc DetectYolo(YoloRequest) returns (YoloResponse);
}
```

**RPC pattern: Unary.** Each tick produces a single batch of frames requiring a single response before the actor can proceed. Client-streaming adds complexity without benefit since actors block on inference results before the next tick. Server-side batching across actors (if desired) can be implemented behind the unary interface via request queuing in the servicer.

**Message size:** A 4-camera actor at 640×480 JPEG quality 85 produces ~80–160 KB per request. Set `grpc.max_send_message_length` and `grpc.max_receive_message_length` to 16 MB on both client and server to accommodate future configuration changes.

---

## 2. Server Implementation

**Approach: Replace the HTTP YOLO endpoint entirely with a gRPC server running alongside uvicorn.**

The LitServe/FastAPI process loads the GPU model once. The gRPC servicer runs in a `ThreadPoolExecutor`-backed thread, reusing `load_global_model()` and `YOLOv5Server.encode_response()`. The `/predict_msgpack` HTTP endpoint is removed.

### New file: `ecav/ml_manager/perception_servicer.py`

Implement `PerceptionServiceServicer.DetectYolo()`:
1. JPEG-decode `request.jpeg_frames` via `cv2.imdecode`
2. Run `model(images)` under `torch.no_grad()`
3. Encode results into `YoloResponse`, populating timing fields directly in the message
4. Return response

### Modification to `litserve_models.py`

Remove `/predict_msgpack` and `load_global_model()`. Start the gRPC server alongside uvicorn:

```python
grpc_port = int(os.getenv("GRPC_PORT", "18001"))
grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
perception_pb2_grpc.add_PerceptionServiceServicer_to_server(servicer, grpc_server)
grpc_server.add_insecure_port(f"[::]:{grpc_port}")
grpc_server.start()
```

gRPC server on port 18001. uvicorn (LitServe) remains on port 18000 for the WorldFusion `/extract_features` endpoint only.

---

## 3. Client Implementation

### `ecav/ml_manager/ml_manager.py` — `_init_distributed()`

Replace `requests.Session` with a persistent gRPC channel:

```python
self._grpc_channel = grpc.insecure_channel(
    target=self.yolo_grpc_endpoint,
    options=[
        ("grpc.max_send_message_length", 16 * 1024 * 1024),
        ("grpc.max_receive_message_length", 16 * 1024 * 1024),
        ("grpc.keepalive_timeout_ms", 10000),
    ],
)
self._perception_stub = perception_pb2_grpc.PerceptionServiceStub(self._grpc_channel)
```

Remove `self._session`, `requests` import, and `msgpack` usage from the YOLO path.

### `_detect_yolo_distributed()`

JPEG encoding via `cv2.imencode` is unchanged. Replace msgpack pack + HTTP POST with:

```python
request = perception_pb2.YoloRequest(jpeg_frames=jpeg_images, actor_id=self.rank)
response = self._perception_stub.DetectYolo(request, timeout=30)
```

Extract timing from `response.decode_ms`, `response.inference_ms`, etc.

### `close()`

Replace `self._session.close()` with `self._grpc_channel.close()`.

---

## 4. Timing Instrumentation

Server-side timing is embedded in `YoloResponse` proto fields, eliminating custom HTTP headers. CSV schema changes:

- Rename `http_ms` → `rpc_ms` (wall-clock duration of the `DetectYolo` stub call)
- All `server_*` columns remain, sourced from proto fields
- `serialize_ms` measures proto message construction
- `client_deserialize_ms` measures proto response parsing + `YOLOResult` conversion

Update `fieldnames` in `dump_timing_csv` and `NUMERIC_COLS` in `tools/analyze_ml_timing.py`.

---

## 5. YAML Config

```yaml
ml_manager:
  yolo_endpoint: 'localhost:18001'
```

---

## 6. Proto Compilation

```bash
python -m grpc_tools.protoc \
    -I./ecav/protos \
    --python_out=./ecav/protos \
    --grpc_python_out=./ecav/protos \
    ./ecav/protos/perception.proto
```

Produces `ecav/protos/perception_pb2.py` and `ecav/protos/perception_pb2_grpc.py`. Follow the existing import convention (bare module names, `sys.path` includes `ecav/protos/`):

```python
import perception_pb2 as perception
import perception_pb2_grpc as perception_rpc
```

Add `perception.proto` to the existing `--build` flag compilation step in `opencda.py` so both protos compile in a single pass.

---

## 7. WorldFusion `/extract_features` Endpoint

**Current state: HTTP/msgpack. Future: migrate to gRPC (deferred).**

WorldFusion feature extraction is technically migratable to gRPC — the pattern is the same as YOLO. The blocking complexity is tensor serialization: `ExtractFeaturesRequest` must encode a nested dict of arbitrary-shape numpy arrays (`voxel_features`, `voxel_coords`, `voxel_num_points`, and optionally `imgs`, `depth_map`, geometry tensors). Each field requires `bytes data + repeated int64 shape + string dtype` in proto — the `CompressedTensor` pattern already present in `ecloud.proto`. This is more message types and more serialization/deserialization code than the YOLO migration.

The latency argument is weaker than for YOLO: WorldFusion runs at lower frequency and is not on the per-tick critical path for the primary blind-intersection scenario. Payload size is large regardless of transport (spatial features are dense tensors). The primary gain would be protocol consistency — one transport for all ML inference — rather than latency reduction.

### What the migration would require

**Proto additions to `perception.proto`:**

```protobuf
message NumpyArray {
  repeated int64 shape = 1;
  string dtype = 2;
  bytes data = 3;   // numpy.tobytes()
}

message ExtractFeaturesRequest {
  NumpyArray voxel_features = 1;
  NumpyArray voxel_coords = 2;
  NumpyArray voxel_num_points = 3;
  NumpyArray record_len = 4;
  bool has_image_inputs = 5;
  NumpyArray imgs = 6;
  NumpyArray depth_map = 7;
  // additional geometry fields as needed
}

message ExtractFeaturesResponse {
  NumpyArray spatial_features = 1;
}

// Add to PerceptionService:
//   rpc ExtractFeatures(ExtractFeaturesRequest) returns (ExtractFeaturesResponse);
```

**Other changes:**
- `perception_servicer.py`: add `ExtractFeatures()` method calling `_extract_wf_features()`
- `litserve_models.py`: remove `/extract_features` HTTP endpoint; gRPC servicer handles it
- `ml_manager.py`: replace `requests.post(worldfusion_endpoint/extract_features)` with stub call; remove `worldfusion_endpoint` config key
- YAML configs: remove `worldfusion_endpoint`; WorldFusion uses the same `yolo_endpoint` channel

---

## 8. Implementation Steps

| Step | File(s) | Description |
|------|---------|-------------|
| 1 | `ecav/protos/perception.proto` | Define `YoloRequest`, `YoloResponse`, `PerceptionService` |
| 2 | `ecav/protos/` (generated) | Compile proto, verify `perception_pb2.py` and `perception_pb2_grpc.py` |
| 3 | `ecav/ml_manager/perception_servicer.py` | Implement `PerceptionServiceServicer` with YOLO inference and timing |
| 4 | `ecav/ml_manager/litserve_models.py` | Remove `/predict_msgpack`; add gRPC server startup |
| 5 | `ecav/ml_manager/ml_manager.py` | gRPC channel init, replace `_detect_yolo_distributed()`, remove `requests`/msgpack YOLO path, update `close()` |
| 6 | `ecav/ml_manager/ml_manager.py` | Update `_parse_yolo_response()` to accept proto `YoloResponse` |
| 7 | `ecav/ml_manager/ml_manager.py` | Rename `http_ms` → `rpc_ms` in timing CSV schema |
| 8 | `tools/analyze_ml_timing.py` | Update `NUMERIC_COLS` for renamed column |
| 9 | `ecav/scenario_testing/config_yaml/*.yaml` | Replace `yolo_endpoint` with `yolo_grpc_endpoint` |
| 10 | `opencda.py` | Add `perception.proto` to `--build` compilation step |
| 11 | Integration test | Single-actor scenario; compare timing CSV and detections against HTTP baseline |
| 12 | Multi-actor test | 4+ actor scenario; verify concurrent gRPC multiplexing |

---

## 9. Risk Assessment

| Risk | Mitigation |
|------|-----------|
| gRPC message size exceeds default 4 MB limit | Set 16 MB max on both channel and server at init time |
| Thread contention between gRPC servicer ThreadPool and uvicorn event loop | GPU inference is GIL-bound; `ThreadPoolExecutor` serializes GPU access naturally. Monitor for event loop starvation under high concurrency. |
| Proto not compiled before first run | Add `perception.proto` to existing `--build` step; document in `INSTALL.md` |
