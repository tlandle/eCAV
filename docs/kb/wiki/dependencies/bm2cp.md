---
updated: 2026-04-04
---
# BM2CP

## What It Is

BM2CP (Boosting Multi-Camera Collaborative Perception) is an intermediate feature fusion model from OpenCOOD. It processes LiDAR point clouds from multiple agents, extracts intermediate features, and fuses them before running the detection head.

**Submodule:** `ecav/BM2CP/` (fork: `tlandle/BM2CP`)
**Upstream:** OpenCOOD — https://github.com/ucla-mobility/OpenCOOD
**Citation:** Xu et al., ICRA 2022 (OPV2V paper for OpenCOOD)

## Role in eCAV

BM2CP is an alternative to WorldFusion for intermediate fusion. While WorldFusion focuses on camera-based fusion, BM2CP uses point-cloud (LiDAR) intermediate features.

BM2CP is not currently the active focus of optimization work (WorldFusion is). It represents a different modality (LiDAR-based vs. camera-based) for the same fusion objective.

## Relationship to WorldFusion

| Property | BM2CP | WorldFusion |
|----------|-------|-------------|
| Sensor modality | LiDAR | Camera |
| Fusion approach | Intermediate feature | Intermediate feature with reranking |
| Implementation status | Available | Active (optimization in progress) |
| LitServe endpoint | Not yet served | HTTP port 18000 |
| Python env | Python 3.7 (sequential) | Python 3.10 (distributed) |

BM2CP's Python 3.7 requirement historically constrained the sequential mode. WorldFusion's 3.10 compatibility is part of why it is preferred for the distributed architecture.

## Status

BM2CP integration exists but is not under active development. The `ecav/BM2CP/` submodule is the eCAV-specific fork.

## Related

- [WorldFusion dependency](worldfusion.md)
- [Collaborative Perception concept](../concepts/collaborative_perception.md)
- [LitServe dependency](litserve.md)
