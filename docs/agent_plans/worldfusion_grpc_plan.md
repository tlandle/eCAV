# WorldFusion gRPC Migration Plan

**Continues item 13 from [grpc_perception_migration.md](grpc_perception_migration.md)**

## Hypothesis

LitServe's `workers_per_device` IPC (via `multiprocessing.Manager` queues) adds ~100-120ms of overhead
per request for 861KB payloads — larger than the actual inference time (~67ms). This was confirmed
empirically: server logs show `total=79ms` but client sees `http=185ms`, a consistent ~106ms gap
attributable to pickle + IPC both ways through the manager queue.

A standalone gRPC servicer (same process, no IPC) should bring client-observed latency to
`grpc_ms ≈ server total_ms`, i.e., ~75-85ms total. This is the same gain gRPC gave YOLO.

**Counter-hypothesis**: The bottleneck is actually msgpack serialization of large numpy arrays.
Falsified if `pack_ms` in logs is <15ms (it is — observed ~12ms in current runs).

## Verification target

`grpc_ms ≈ server_total_ms ± 5ms` (localhost, IPC overhead gone).

Expected: vehicle and RSU both at ~75-85ms, down from ~185-218ms.

---

## Proto design: msgpack-in-bytes vs structured fields

Section 7 of `grpc_perception_migration.md` proposed `ExtractFeaturesRequest` with individual
`NumpyArray` fields. This plan uses **msgpack-in-bytes** instead.

Rationale:
- The batch dict has ~10 nested numpy arrays with variable shapes (including optional `image_inputs`
  sub-dict with imgs, rots, trans, intrins, post_rots, post_trans, depth_map). Structured proto
  fields would require a `NumpyArray` message per field + repeated for variable-length arrays.
- The client already produces `zlib.compress(msgpack.packb(batch_np))` — zero client-side change.
- The server already has `zlib.decompress` + `msgpack.unpackb` — zero logic change.
- Transport correctness is identical; only the transport layer changes.
- Structured fields remain a future optimization (O6) if proto-native serialization is warranted.

---

## Checklist

### Phase 1 — Proto

- [x] **1.1** Add `WfRequest` and `WfResponse` to `perception.proto`; add `ExtractWfFeatures` RPC to `PerceptionService`
- [x] **1.2** Recompile: `conda run -n opencda python -m grpc_tools.protoc -I./ecav/protos --python_out=./ecav/protos --grpc_python_out=./ecav/protos ./ecav/protos/perception.proto`
- [x] **1.3** Smoke-check: `python -c "import perception_pb2; print(perception_pb2.WfRequest())"` succeeds with no import errors

### Phase 2 — Server

- [x] **2.1** Create `ecav/ml_manager/worldfusion_servicer.py` — `WorldFusionServicer(PerceptionServiceServicer)` implementing `ExtractWfFeatures`
- [x] **2.2** Create `ecav/ml_manager/worldfusion_grpc_server.py` — standalone entry point; loads model once, starts gRPC server on port 18002
- [ ] **2.3** Standalone smoke test: start server, run synthetic gRPC client ping, confirm model loads and RPC responds (even empty payload → unpack error confirms connectivity)

### Phase 3 — Client (`worldfusion_perception_manager.py`)

- [x] **3.1** Add endpoint resolution: `WF_GRPC_ENDPOINT` env var → `grpc_endpoint` YAML key → `ml_manager.worldfusion_grpc_endpoint` → fallback to None (local model)
- [x] **3.2** On `use_grpc=True`: open `grpc.insecure_channel`, create stub; keep `use_litserve` fallback path untouched
- [x] **3.3** Add `_extract_features_grpc()` method (stub call, same timing log format as HTTP path but `grpc_ms` instead of `http_ms`, server timing from response fields)
- [x] **3.4** Add `use_grpc` branch in `run_step()` before `use_litserve` branch
- [x] **3.5** Update `close()` to close channel if `use_grpc`
- [x] **3.6** Rename `http_ms` → `grpc_ms` in `_WF_TIMING_FIELDS` list

### Phase 4 — Edge manager O5 batch path

- [x] **4.1** In `_run_o5_batch_encoder()`: replace `requests.Session.post()` with stub call via `ready[0]._wf_stub`
- [x] **4.2** Update timing dict keys (`grpc_ms`, read server fields from `WfResponse`)
- [x] **4.3** Update PM eligibility filter: `pm.use_litserve` → `pm.use_grpc` (final)
- [x] **4.4** Add `import perception_pb2` at top of method

### Phase 5 — Infrastructure

- [x] **5.1** `start_actors.sh`: replace LitServe startup block with WorldFusion gRPC startup block (port 18002, gRPC readiness probe via `grpc.channel_ready_future`)
- [x] **5.2** `stop_actors.sh`: `pkill worldfusion_grpc_server` + retain `pkill litserve_models` for cleanup
- [x] **5.3** Scenario YAML (`openscenario_3_edge_worldfusion.yaml`): added `worldfusion_grpc_endpoint: 'localhost:18002'` to `ml_manager:` and both `worldfusion_model:` sections (RSU + vehicle)
- [x] **5.4** `ml_manager.py`: added `worldfusion_grpc_endpoint` attribute from config

### Phase 6 — Verification

- [x] **6.1** Full scenario run: confirmed `grpc_ms ≈ server_total_ms ± 5-8ms` in logs
- [ ] **6.2** Confirm detection quality unchanged (same bounding box count per tick vs LitServe baseline)
- [ ] **6.3** Run late fusion scenario: confirm YOLO gRPC still works after proto recompile
- [x] **6.4** Update `current_state.md` with findings

**Measured results (distributed mode, 2026-04-06):**

| Agent | grpc_ms | srv_total_ms | gap | total_e2e_ms | vs LitServe |
|-------|---------|-------------|-----|--------------|-------------|
| Vehicle | 73-81ms | 68-77ms | 5-8ms ✓ | 125-143ms | −55ms |
| RSU (fast) | 82-90ms | 77-85ms | 5-8ms ✓ | 130-140ms | −240ms |
| RSU (slow) | 130-153ms | 126-148ms | 4-8ms ✓ | 190-206ms | (GPU serial) |

RSU bimodal is pure CUDA serialization (two independent batch=1 calls racing the GPU in distributed mode) — not a transport issue. IPC overhead (~106ms) is eliminated.

**Remaining items:**
- Sequential O5 (batch=2) test — one combined forward pass eliminates GPU serialization; lower priority given current perf
- YOLO regression run

---

## New Files

### `ecav/protos/perception.proto` additions

```proto
message WfRequest {
  bytes payload   = 1;  // zlib(msgpack(batch_numpy_dict)) — same encoding as current HTTP path
  int32 actor_id  = 2;  // for server-side logging
}

message WfResponse {
  bytes payload       = 1;  // msgpack({'spatial_features': np.ndarray float16})
  float unpack_ms     = 2;
  float inference_ms  = 3;
  float pack_ms       = 4;
  float total_ms      = 5;
}

// Add to PerceptionService:
//   rpc ExtractWfFeatures(WfRequest) returns (WfResponse);
```

### `ecav/ml_manager/worldfusion_servicer.py`

```python
class WorldFusionServicer(perception_pb2_grpc.PerceptionServiceServicer):
    def __init__(self, extract_fn):
        self._extract = extract_fn   # _extract_wf_features from litserve_models.py

    def ExtractWfFeatures(self, request, context):
        t0 = time.time()
        try:
            body = zlib.decompress(request.payload)
        except zlib.error:
            body = request.payload
        batch = msgpack.unpackb(body, raw=False)
        t1 = time.time()

        result = self._extract(batch)
        t2 = time.time()

        packed = msgpack.packb(result, use_bin_type=True)
        t3 = time.time()

        print(f"[WorldFusionServicer] actor={request.actor_id} "
              f"batch={result['spatial_features'].shape[0]} "
              f"unpack={int((t1-t0)*1000)}ms inference={int((t2-t1)*1000)}ms "
              f"pack={int((t3-t2)*1000)}ms total={int((t3-t0)*1000)}ms")

        return perception_pb2.WfResponse(
            payload=packed,
            unpack_ms=(t1-t0)*1000, inference_ms=(t2-t1)*1000,
            pack_ms=(t3-t2)*1000,   total_ms=(t3-t0)*1000,
        )
    # DetectYolo: inherited UNIMPLEMENTED — correct for standalone WF server
```

### `ecav/ml_manager/worldfusion_grpc_server.py`

```python
def serve():
    load_wf_model()                              # warm up once; no subprocess fork
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=4),
        options=[("grpc.max_send_message_length",    32*1024*1024),
                 ("grpc.max_receive_message_length", 32*1024*1024)],
    )
    perception_pb2_grpc.add_PerceptionServiceServicer_to_server(
        WorldFusionServicer(_extract_wf_features), server
    )
    server.add_insecure_port(f"{host}:{port}")
    server.start()
    server.wait_for_termination()
```

---

## Port assignment

| Port  | Protocol | Service             |
|-------|----------|---------------------|
| 18000 | HTTP/1.1 | LitServe (retained for local/debug) |
| 18001 | gRPC     | YOLO (existing)     |
| 18002 | gRPC     | WorldFusion (new)   |

18002 avoids collision with LitServe (18000) and YOLO (18001). Distinct ports make
`start_actors.sh` readiness probes unambiguous.

---

## Known risks

| Risk | Mitigation |
|------|-----------|
| `DetectYolo` UNIMPLEMENTED on WF server (port 18002) | Correct — no YOLO client ever hits 18002. Port separation makes this impossible. |
| O5 batch encoder borrows stub from `ready[0]._wf_stub` | All PMs connect same endpoint; gRPC channels are thread-safe. Note in code. |
| Proto recompile breaks YOLO | Proto3 is additive; YOLO messages/RPC unchanged. Run YOLO regression after recompile. |
| `grpc_tools` version mismatch vs installed gRPC | Compile in `opencda` env to match. Check stub file header after compile. |
| `use_litserve` kept as fallback during transition | Remove after verification complete. Don't let two paths live indefinitely. |

---

## Related

- [grpc_perception_migration.md](grpc_perception_migration.md) — YOLO gRPC migration (complete); item 13 is this plan
- [litserve_parallelism_plan.md](litserve_parallelism_plan.md) — Options A/B/C (complete); gRPC is the exit
- [worldfusion_litserve_plan.md](worldfusion_litserve_plan.md) — payload optimizations O1–O8 (carried forward; O5 batch path adapts to gRPC in step 4)
