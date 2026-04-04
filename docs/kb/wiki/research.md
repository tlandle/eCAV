---
updated: 2026-04-04
---
# Research

## Research Question

What is the operational safety envelope of edge-assisted autonomous vehicles under realistic network conditions?

Specifically: how do network latency, jitter, and multi-source state inconsistency affect closed-loop safety when a vehicle relies on edge-fused perception to navigate an occluded intersection?

## Publication

> Tyler Landle et al. "eCAV: An Edge-Assisted Evaluation Platform for Connected Autonomous Vehicles." arXiv:2506.16535, 2025.
> https://arxiv.org/abs/2506.16535

## Core Contributions

### Self-Beacon Anchoring (SBA)
Enforces an ego-uniqueness invariant at the edge publish boundary. Eliminates self-ghosting: the failure mode where a vehicle's own detected bounding box is reflected back from the edge as an external obstacle. Without SBA, edge-published state can include the ego vehicle as a phantom obstacle, causing erroneous avoidance maneuvers.

### Latency-Aware Tracking
Three edge tracking architectures:
- **AB3DMOT** — 3D Kalman filter baseline; no latency compensation
- **VIPS** — velocity-based temporal alignment; compensates for known latency by projecting state forward
- **Oracle** — ground-truth position baseline; upper bound on achievable performance

### Network Modeling
Trace-driven latency models from real network captures:
- **C-V2X PC5** (vehicle-to-vehicle radio) — SEE-V2X dataset
- **5G backhaul** (vehicle-to-edge) — 5G-MOBIX dataset
- MAC-layer model: C-V2X PC5 Mode 4 Sensing-Based Semi-Persistent Scheduling (3GPP TS 36.321)

### Collaborative Perception
Two intermediate fusion approaches for multi-vehicle feature sharing:
- **BM2CP** (OpenCOOD-based) — point-cloud intermediate feature fusion
- **WorldFusion** — camera-based intermediate fusion with world-model reconciliation and feature reranking

## Evaluation Setup

**Simulator:** CARLA 0.9.12+
**Scenario:** Occluded left-hand turn at blind intersection (NHTSA Pre-Crash Typology)
**Test matrix** (from `test_runner.py`):

```bash
python test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types late_fusion \
  --latencies 0 100 200 400 \
  --ego-counts 1 4 8 16 \
  --anchoring both \
  --repetitions 3
```

## Measured Performance: LitServe Optimization Work

### YOLO Late Fusion (gRPC, 640×480 camera)

| Configuration | e2e latency | Notes |
|---------------|-------------|-------|
| HTTP baseline (full-res camera) | ~82ms | Original implementation |
| +O1 requests.Session keep-alive | ~68ms | Connection reuse |
| +O2 JPEG compression | ~54ms | 10–30× size reduction |
| Native 640×480 camera resolution | ~22ms | Dominant win: eliminated resize overhead |
| +O5 gRPC transport | TBD | Transport migration complete; timing pending |

Key insight: the camera was set to a higher resolution than YOLO's inference resolution. Downsampling at capture (640×480) rather than at encode eliminated ~87% of encode/decode cost and cut total e2e by ~73%.

### WorldFusion Intermediate Fusion (HTTP, 2-agent scenario)

| Configuration | request_KB | response_KB | http_ms | total_e2e_ms |
|---------------|-----------|-------------|---------|--------------|
| Baseline (all float32) | 8021 | ~10000 | ~291 | ~355 |
| +O1: float16 response | 8021 | ~5000 | ~175 | ~234 |
| +O4: uint8 imgs | 3803 | ~5000 | ~175 | ~233 |
| O5: batch inference (pending) | ~3803 | ~5000 | ~87 est. | TBD |

O4 (uint8 imgs) is neutral on loopback but saves ~4 MB/s bandwidth per intersection on Azure, where network cost matters.

O5 targets ~50% inference cost reduction by merging all agents into a single batch=N LitServe call instead of N sequential calls.

## Research Relationship to eCAV Development

The research depends on the simulation infrastructure being reliable and instrumented. The LitServe optimization work (plans O1–O5) is not research per se — it reduces the simulation overhead so that network latency, not perception-pipeline latency, dominates the timing budget. If LitServe inference takes 350ms per tick, we cannot meaningfully model 100ms network latency.

The edge architecture work enables true distributed edge simulation, which is required to evaluate the multi-edge scaling scenarios described in the paper.

## Related

- [Edge-Assisted Perception concept](concepts/edge_assisted_perception.md)
- [Collaborative Perception concept](concepts/collaborative_perception.md)
- [Latency Modeling concept](concepts/latency_modeling.md)
- [WorldFusion dependency](dependencies/worldfusion.md)
- [BM2CP dependency](dependencies/bm2cp.md)
- [Plans Index](plans_index.md)
