# Migration Plan: Distributed Perception Transport from HTTP/msgpack to gRPC

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

**Message size:** A 3-camera actor at JPEG quality 85 produces ~200–400 KB per request. Set `grpc.max_send_message_length` and `grpc.max_receive_message_length` to 16 MB on both client and server to accommodate high-resolution or many-camera configurations.

---

## 2. Server Implementation

**Approach: Run the gRPC server in a background thread within the existing LitServe process.**

The LitServe/FastAPI process loads the GPU model once. Adding a gRPC servicer in a `ThreadPoolExecutor`-backed thread avoids spawning a separate process while reusing the loaded model. The servicer calls the same `load_global_model()` function and reuses `YOLOv5Server.encode_response()`.

### New file: `ecav/ml_manager/perception_servicer.py`

Implement `PerceptionServiceServicer.DetectYolo()`:
1. JPEG-decode `request.jpeg_frames` via `cv2.imdecode` (identical to current `_yolo_process` logic)
2. Run `model(images)` under `torch.no_grad()`
3. Encode results into `YoloResponse`, populating timing fields directly in the message (replaces HTTP headers)
4. Return response

### Modification to `litserve_models.py`

After LitServe server setup, start the gRPC server alongside uvicorn:

```python
grpc_port = int(os.getenv("GRPC_PORT", "18001"))
grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
perception_pb2_grpc.add_PerceptionServiceServicer_to_server(servicer, grpc_server)
grpc_server.add_insecure_port(f"[::]:{grpc_port}")
grpc_server.start()
```

HTTP server stays on port 18000; gRPC server on port 18001. Both share `_global_model`.

---

## 3. Client Implementation

### `ecav/ml_manager/ml_manager.py` — `_init_distributed()`

Create a persistent channel replacing `requests.Session`:

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

The persistent channel multiplexes concurrent RPCs from multiple actor instances sharing the same endpoint — equivalent benefit to `requests.Session` keep-alive but with HTTP/2 multiplexing instead of HTTP/1.1 head-of-line.

### `_detect_yolo_distributed()`

JPEG encoding is unchanged. Replace msgpack pack + HTTP POST with:

```python
request = perception_pb2.YoloRequest(jpeg_frames=jpeg_images, actor_id=self.rank)
response = self._perception_stub.DetectYolo(request, timeout=30)
```

Extract timing from `response.decode_ms`, `response.inference_ms`, etc., replacing `response.headers.get('X-Server-*')`.

### `close()`

Replace `self._session.close()` with `self._grpc_channel.close()`.

---

## 4. Timing Instrumentation

Server-side timing is embedded in `YoloResponse` proto fields, eliminating custom HTTP headers. The CSV schema changes minimally:

- Rename `http_ms` → `rpc_ms` (wall-clock duration of the `DetectYolo` stub call, inclusive of network + server processing)
- All `server_*` columns remain, sourced from proto fields instead of HTTP headers
- `serialize_ms` measures proto message construction (faster than msgpack packing)
- `client_deserialize_ms` measures proto response parsing + `YOLOResult` conversion

Update `fieldnames` in `dump_timing_csv` and `NUMERIC_COLS` in `tools/analyze_ml_timing.py` accordingly.

---

## 5. Proto Compilation

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

## 6. Migration Path

HTTP and gRPC coexist during transition. The YAML config gains new keys under `ml_manager`:

```yaml
ml_manager:
  yolo_endpoint: 'http://localhost:18000'   # HTTP (existing)
  yolo_grpc_endpoint: 'localhost:18001'      # gRPC (new)
  perception_transport: 'grpc'               # 'http' or 'grpc'; default 'http'
```

`MLManager._init_distributed()` checks `perception_transport` and initializes the appropriate transport. `_detect_yolo_distributed()` dispatches accordingly. The LitServe server continues serving both endpoints simultaneously.

Once all scenario configs are validated on gRPC, the HTTP YOLO codepath, msgpack dependency for YOLO, and `requests.Session` are removed.

---

## 7. WorldFusion `/extract_features` Endpoint

**Leave as HTTP for now.**

WorldFusion feature extraction runs at lower frequency and involves large, arbitrary-shape numpy tensors. The existing msgpack-numpy encoding handles these payloads well; encoding equivalent tensors in protobuf requires `bytes` fields with manual numpy serialization (duplicating the `CompressedTensor` pattern already in `ecloud.proto`), adding complexity without a clear latency benefit. WorldFusion is also not on the per-tick critical path for the primary blind-intersection scenario. If profiling later identifies it as a bottleneck, add an `ExtractFeatures` RPC to `PerceptionService` following the same migration pattern.

---

## 8. Implementation Steps

| Step | File(s) | Description |
|------|---------|-------------|
| 1 | `ecav/protos/perception.proto` | Define `YoloRequest`, `YoloResponse`, `PerceptionService` |
| 2 | `ecav/protos/` (generated) | Compile proto, verify `perception_pb2.py` and `perception_pb2_grpc.py` |
| 3 | `ecav/ml_manager/perception_servicer.py` | Implement `PerceptionServiceServicer` with YOLO inference and timing |
| 4 | `ecav/ml_manager/litserve_models.py` | Add gRPC server startup alongside existing uvicorn |
| 5 | `ecav/ml_manager/ml_manager.py` | gRPC channel init in `_init_distributed()`, gRPC path in `_detect_yolo_distributed()`, update `close()` |
| 6 | `ecav/ml_manager/ml_manager.py` | Update `_parse_yolo_response()` to accept proto `YoloResponse` |
| 7 | `ecav/ml_manager/ml_manager.py` | Rename `http_ms` → `rpc_ms` in timing CSV schema |
| 8 | `tools/analyze_ml_timing.py` | Update `NUMERIC_COLS` for renamed column |
| 9 | `ecav/scenario_testing/config_yaml/*.yaml` | Add `yolo_grpc_endpoint` and `perception_transport` to relevant configs |
| 10 | `opencda.py` | Add `perception.proto` to `--build` compilation step |
| 11 | Integration test | Single-actor scenario with `perception_transport: grpc`; compare timing CSV and detections against HTTP baseline |
| 12 | Multi-actor test | 4+ actor scenario; verify concurrent gRPC multiplexing |
| 13 | Cleanup (deferred) | Remove HTTP YOLO codepath, `requests` dependency for YOLO, `requests.Session` |

---

## 9. Risk Assessment

| Risk | Mitigation |
|------|-----------|
| gRPC message size exceeds default 4 MB limit | Set 16 MB max on both channel and server at init time |
| Thread contention between gRPC servicer ThreadPool and uvicorn event loop | GPU inference is GIL-bound; `ThreadPoolExecutor` serializes GPU access naturally. Monitor for event loop starvation under high concurrency. |
| Proto not compiled before first run | Add `perception.proto` to existing `--build` step; document in `INSTALL.md` |
| Backward compatibility with existing YAML configs | `perception_transport` defaults to `'http'`; existing configs are unaffected until explicitly opted in |
