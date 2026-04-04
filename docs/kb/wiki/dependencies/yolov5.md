---
updated: 2026-04-04
---
# YOLOv5

## What It Is

YOLOv5 is a single-stage object detection model from Ultralytics. It runs a convolutional backbone (CSP-based) followed by a detection head, producing bounding boxes with class and confidence scores in a single forward pass.

**Repo (local):** `yolov5/` (submodule, included directly in eCAV repo)
**License:** GPL-3.0

## Role in eCAV

YOLOv5 is the perception model for the **late fusion** path. Each vehicle actor (or the LitServe server, in `-l` mode) runs YOLOv5 on its front-facing CARLA camera feed to detect other vehicles.

Late fusion flow:
```
CARLA camera (640×480) → JPEG encode → gRPC → LitServe → YOLOv5 → bounding boxes → actor
```

In local (non-LitServe) mode:
```
CARLA camera (640×480) → in-process YOLOv5 → bounding boxes
```

## Inference Configuration

- **Input resolution:** 640×480 (matching native CARLA camera resolution after D4 decision)
- **Model weights:** pre-trained on COCO; fine-tuned weights may be in `ecav/assets/` or loaded from `yolov5/weights/`
- **NMS threshold:** configured in scenario YAML
- **Classes:** vehicle detection only (car, truck, bus); pedestrians not used in current scenarios

## Integration

**LitServe path:**
- `ecav/ml_manager/litserve_models.py` — `YoloLitAPI` class
- `ecav/ml_manager/perception_servicer.py` — gRPC servicer; decodes JPEG, runs inference, returns boxes
- `ecav/ml_manager/ml_manager.py` — client; sends gRPC request, receives response

**Local path:**
- `ecav/core/sensing/perception/` — perception manager; loads model locally per actor

## Transport (after gRPC migration)

- Client sends: JPEG-encoded frame (uint8, 640×480, typically 20–40 KB)
- Server returns: bounding box list (class, confidence, x1, y1, x2, y2)
- Transport: gRPC on port 18001, 16 MB message limit
- Connection: persistent channel; created once at actor startup

## Performance

- Inference: ~3–5ms on GPU (GTX 3080 class hardware)
- E2e over loopback gRPC: ~22ms (dominated by transport overhead, not inference)
- At full camera resolution: ~82ms e2e (dominated by encode/decode before D4)

## Related

- [LitServe dependency](litserve.md)
- [gRPC dependency](grpc.md)
- [Collaborative Perception concept](../concepts/collaborative_perception.md)
- [Decisions: D4 native camera resolution](../decisions.md)
- [litserve_performance_plan.md](../../../agent_plans/litserve_performance_plan.md)
