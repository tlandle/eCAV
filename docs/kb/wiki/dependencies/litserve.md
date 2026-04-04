---
updated: 2026-04-04
---
# LitServe

## What It Is

LitServe is a model serving framework from Lightning AI. It wraps ML models in a FastAPI/uvicorn HTTP server with built-in batching, async dispatch, and multiple worker support. It is designed to be simpler to configure than TorchServe or Triton while supporting production-grade throughput.

**Upstream:** https://github.com/Lightning-AI/LitServe

## Role in eCAV

LitServe hosts the ML perception models so that actor processes don't each load their own copy. With N actor containers each loading a ~600MB YOLO model, GPU memory becomes the constraint. LitServe runs once, serves all actors.

Two independent services:

| Service | Port | Transport | Model |
|---------|------|-----------|-------|
| YOLO detection | 18001 | gRPC | YOLOv5 |
| WorldFusion feature extraction | 18000 | HTTP | WorldFusion |

## Server Implementation

**`ecav/ml_manager/litserve_models.py`** — LitServe server definitions:
- `YoloLitAPI` — wraps YOLOv5; accepts JPEG bytes via gRPC, returns bounding boxes
- `WorldFusionLitAPI` — wraps WorldFusion; accepts feature tensors via HTTP, returns fused detections

**`ecav/ml_manager/perception_servicer.py`** — gRPC servicer for YOLO:
- Implements `PerceptionService.Detect`
- JPEG decode → YOLOv5 inference → encode bounding boxes → return

## WorldFusion HTTP Endpoint

`POST /extract_features` with JSON payload:
```json
{
  "voxel_features": ...,     // sparse tensor (float16)
  "voxel_coords": ...,
  "voxel_num_points": ...,
  "imgs": ...,               // uint8 (after O4 optimization)
  "depth_map": ...,          // zero placeholder
  "geometry": ...
}
```

Response: fused detections + intermediate features.

Payload per tick (2-agent scenario): ~3.8 MB request, ~5 MB response (after O1+O4 optimizations).

## Startup

LitServe server is started separately from actor processes:
```bash
# YOLO server (gRPC on 18001)
python ecav/ml_manager/ml_manager.py --yolo

# WorldFusion server (HTTP on 18000)
python ecav/ml_manager/ml_manager.py --worldfusion
```

Model is pre-loaded at startup (not on first request). This was O1 of the WorldFusion optimization: first-tick cold-load caused a ~2s spike on tick 1.

## Performance Notes

- YOLO e2e latency (640×480 camera): ~22ms over loopback gRPC
- WorldFusion e2e latency (2 agents): ~233ms over loopback HTTP (after O1+O4)
- WorldFusion O5 (batch inference): ~87ms target http_ms; pending implementation

See [worldfusion_litserve_plan.md](../../../agent_plans/worldfusion_litserve_plan.md) for full measurement data.

## Related

- [YOLOv5 dependency](yolov5.md)
- [WorldFusion dependency](worldfusion.md)
- [gRPC dependency](grpc.md)
- [Collaborative Perception concept](../concepts/collaborative_perception.md)
- [litserve_performance_plan.md](../../../agent_plans/litserve_performance_plan.md)
