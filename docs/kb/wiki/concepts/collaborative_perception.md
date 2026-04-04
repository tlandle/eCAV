---
updated: 2026-04-04
---
# Collaborative Perception

## Definition

Collaborative perception is the sharing of perceptual information between agents (vehicles, edge cameras, RSUs) to collectively detect objects that no single agent could reliably detect alone. In eCAV, collaboration is between edge-mounted cameras and ego vehicles approaching a blind intersection.

## Fusion Architectures

### Late Fusion

Each agent independently completes perception (raw sensor → bounding box). The edge collects each agent's bounding box list and merges them.

**Advantages:**
- Simple: each agent runs standard detection independently
- Fault-tolerant: one agent's failure doesn't corrupt others' outputs
- Low coupling: agents don't need to share intermediate representations

**Disadvantages:**
- Loses spatial correlation between views; fusion happens at the most compressed representation
- Duplicate detections require NMS (non-maximum suppression) across agents
- No improvement for objects partially visible in multiple views (bounding box can't encode uncertainty)

**eCAV implementation:** Each vehicle runs YOLO (locally or via LitServe gRPC); edge runs YOLO on its own camera; edge merges bounding box lists. Tracking via AB3DMOT or VIPS.

### Intermediate Fusion

Agents share intermediate feature maps (output of CNN backbone, before detection head). A fusion module merges the shared features; the detection head runs once on the fused representation.

**Advantages:**
- Preserves spatial context across views; can detect objects partially visible in any individual view
- Learns cross-agent spatial relationships (via attention or learned fusion weights)
- Typically outperforms late fusion in perception accuracy

**Disadvantages:**
- Bandwidth: sharing feature tensors is orders of magnitude larger than sharing bounding boxes
- Tight coupling: all agents must use compatible model architectures
- Harder to scale: N×M agent-pair combinations; batch management required

**eCAV implementations:**
- **BM2CP**: OpenCOOD-based point-cloud intermediate feature fusion
- **WorldFusion**: camera-based intermediate fusion with world-model reconciliation and feature reranking

### Early Fusion (not implemented)

Sharing raw sensor data (pixels, point clouds) before any processing. Maximum information retention, but impractical bandwidth requirements (a single 1080p camera frame is ~6 MB).

## Fusion in eCAV's Architecture

### Late Fusion Data Path

```
Vehicle camera → YOLO (local or LitServe gRPC:18001) → bounding boxes
                                                              ↓
Edge camera → YOLO → bounding boxes ──────────────────→ merge → publish to vehicle
```

### Intermediate Fusion Data Path (WorldFusion)

```
Vehicle camera → backbone → feature tensor ─────────────────────────────────────┐
RSU camera → backbone → feature tensor ────────────────────────────────────────→ edge
                                                                                   ↓
                                                               edge calls LitServe HTTP:18000
                                                               /extract_features (batch)
                                                                                   ↓
                                                               detection head → bounding boxes
                                                                                   ↓
                                                               publish to vehicles
```

O5 optimization (pending): edge batches all agents' feature tensors into a single LitServe call (batch=N) rather than N sequential calls, reducing inference overhead per agent by ~50%.

## NMS Across Agents

Late fusion requires non-maximum suppression (NMS) across the merged bounding box lists to eliminate duplicates. WorldFusion's enablement included an NMS fix — the original implementation was running NMS before merging agent detections, not after, which caused valid cross-agent detections to be suppressed incorrectly.

## Related

- [Edge-Assisted Perception concept](edge_assisted_perception.md)
- [WorldFusion dependency](../dependencies/worldfusion.md)
- [BM2CP dependency](../dependencies/bm2cp.md)
- [YOLOv5 dependency](../dependencies/yolov5.md)
- [LitServe dependency](../dependencies/litserve.md)
- [Research](../research.md)
