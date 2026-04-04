---
updated: 2026-04-04
---
# SORT and AB3DMOT

## SORT

**What it is:** Simple Online and Realtime Tracking. 2D Kalman filter–based multi-object tracker. Associates detections across frames using IoU-based Hungarian algorithm assignment.

**Submodule:** `sort/` (ICGog/sort fork)
**Use in eCAV:** Baseline 2D tracker for vehicle actors in late fusion path. Runs on YOLOv5 bounding box outputs to maintain vehicle tracks across ticks.

## AB3DMOT

**What it is:** A Baseline for 3D Multi-Object Tracking (Weng et al., CVPR Workshop 2020). Extends Kalman filtering to 3D: tracks (x, y, z, θ, l, w, h) — position, heading, and dimensions. Hungarian matching on 3D IoU.

**Integration:** `AB3DMOT_libs/` (local, not a submodule)
**Citation:** Weng et al., CVPR Workshops 2020

**Use in eCAV:** Edge-side tracking algorithm. Receives 3D bounding box detections from the edge camera perception, maintains tracks across ticks, and publishes tracked obstacle state to vehicles.

## VIPS

VIPS (Velocity-based Temporal Alignment) is an eCAV-specific extension of AB3DMOT that compensates for network latency by projecting tracked obstacle positions forward in time based on their estimated velocity and the known/estimated latency.

Not a separate dependency — implemented within eCAV's tracking module.

## Tracking Algorithm Comparison

| Algorithm | Dimensionality | Latency Compensation | Role |
|-----------|---------------|---------------------|------|
| SORT | 2D (image plane) | None | Vehicle-side, local |
| AB3DMOT | 3D (world coords) | None | Edge-side baseline |
| VIPS | 3D (world coords) | Velocity projection | Edge-side, research |
| Oracle | 3D (ground truth) | Perfect | Upper bound |

## Related

- [Edge-Assisted Perception concept](../concepts/edge_assisted_perception.md)
- [Latency Modeling concept](../concepts/latency_modeling.md)
- [YOLOv5 dependency](yolov5.md)
- [Research](../research.md)
