---
updated: 2026-04-04
---
# gRPC

## What It Is

gRPC is a high-performance RPC framework using Protocol Buffers for message serialization and HTTP/2 for transport. It supports unary and streaming RPCs with code generation for typed clients and servers in multiple languages.

## Role in eCAV

gRPC is the IPC layer for all simulation communication:

| Link | Service | Proto |
|------|---------|-------|
| Actors ↔ C++ comms server | `EcloudServer` | `ecloud.proto` |
| Actors ↔ LitServe (YOLO) | `PerceptionService` | `perception.proto` |
| Orchestrator ↔ C++ server (hooks) | `sim_api.py` bindings | `ecloud.proto` |

WorldFusion (LitServe port 18000) still uses HTTP; gRPC migration is planned.

## Proto Files

**`ecav/protos/ecloud.proto`** — Simulation orchestration:
- `VehicleState` enum (REGISTERING, TICK_OK, TICK_DONE, etc.)
- `Vehicle`, `RSU`, `Edge` message types
- `EcloudServer` service with `SendUpdate`, `GetScenario`, `Tick` RPCs
- `ScenarioParameters` — serialized YAML config passed at registration

**`ecav/protos/perception.proto`** — ML inference:
- `YoloRequest` — JPEG-encoded image bytes
- `YoloResponse` — bounding boxes with class and confidence
- `PerceptionService.Detect` unary RPC

Generated stubs (`*_pb2.py`, `*_pb2_grpc.py`) live alongside the `.proto` files and are added to `sys.path` at startup.

## Recompile Stubs

```bash
python ecav.py --build
```

Or directly:
```bash
python -m grpc_tools.protoc \
  -I./ecav/protos \
  --python_out=./ecav/protos \
  --grpc_python_out=./ecav/protos \
  ./ecav/protos/ecloud.proto \
  ./ecav/protos/perception.proto
```

## Ports

| Service | Port | Protocol |
|---------|------|----------|
| C++ comms server | 50051 | gRPC |
| LitServe YOLO | 18001 | gRPC |
| LitServe WorldFusion | 18000 | HTTP (not gRPC yet) |

## Implementation Notes

- C++ server uses grpc++ with native threads (no GIL)
- Python clients use `grpc.insecure_channel` with persistent connection (`grpc.channel_ready_future`)
- YOLO gRPC channel is kept alive across ticks (`ml_manager.py`); was previously re-created per request
- Message size limit: 16 MB (set on both client and server options) — relevant for large perception payloads

## Related

- [Architecture](../architecture.md)
- [LitServe dependency](litserve.md)
- [YOLOv5 dependency](yolov5.md)
- [grpc_perception_migration.md](../../../agent_plans/grpc_perception_migration.md)
