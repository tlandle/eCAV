---
updated: 2026-04-04
---
# WorldFusion

## What It Is

WorldFusion is a collaborative perception model for intermediate feature fusion. It takes camera images and associated spatial metadata from multiple agents (vehicles, RSUs), fuses them via a learned attention mechanism with world-model reconciliation, and outputs a unified set of detections.

**Submodule:** `ecav/worldfusion/` (fork: `tlandle/WorldFusion`)
**Upstream:** `tlandle/WorldFusion` (eCAV-specific fork of original WorldFusion)

## Role in eCAV

WorldFusion implements the **intermediate fusion** path for the `openscenario_3_edge_worldfusion` scenario. The edge collects feature tensors from all actors (vehicle + RSU cameras), sends them in a batch to LitServe, and receives fused detections.

Intermediate fusion flow:
```
Vehicle camera → backbone → feature tensor ─┐
RSU camera → backbone → feature tensor ──────→ edge → LitServe HTTP:18000 → WorldFusion → detections
```

This allows detection of occluded objects that neither agent could detect alone, by fusing complementary viewpoints before the detection head runs.

## Input Payload (per LitServe call)

| Field | Type | Size (2 agents) | Notes |
|-------|------|-----------------|-------|
| `voxel_features` | float16 | small | Sparse voxel features |
| `voxel_coords` | int | small | Voxel coordinate indices |
| `voxel_num_points` | int | small | Points per voxel |
| `imgs` | uint8 | ~3.8 MB | Camera images (after O4 optimization) |
| `depth_map` | float32 | ~2.8 MB | Zero placeholder (depth not available) |
| `geometry` | mixed | small | Camera intrinsics/extrinsics |

Total inbound per call: ~3.8 MB (after O1+O4). Original float32 baseline: ~8 MB.

## LitServe Endpoint

- **Port:** 18000 (HTTP)
- **Endpoint:** `POST /extract_features`
- **Response:** fused detections + spatial features (~5 MB, float16 after O1)

## Optimization History

| Optimization | Status | Impact |
|-------------|--------|--------|
| O1: Pre-load model at startup | ✅ done | Eliminates first-tick ~2s cold-load spike |
| O2: Eliminate zero depth_map | ❌ reverted | No measurable benefit |
| O3: float32→float16 imgs at encode | ❌ reverted | astype cost > transport savings |
| O4: uint8 imgs | ✅ done | −53% request payload; neutral on loopback |
| O1-resp: float16 response spatial_features | ✅ done | −40% response payload; −40% http_ms |
| O5: Edge-coordinated batch inference | ⏳ pending | Target ~50% inference cost reduction |
| O6: Async/pipelined extraction | ⏳ future | |
| O7: gRPC transport | ⏳ future | Complex tensor serialization |

## NMS Fix

The original WorldFusion integration ran NMS *before* merging agent detections, which incorrectly suppressed cross-agent duplicate detections (i.e., valid overlapping boxes from different viewpoints). Fixed during enablement (commit 88351ff): NMS now runs *after* merging.

## Scenario

**`openscenario_3_edge_worldfusion`** — the scenario that uses WorldFusion intermediate fusion.
- YAML: `ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml`
- Python: `ecav/scenario_testing/openscenario_3_edge_worldfusion.py`

WorldFusion endpoint configured in YAML:
```yaml
ml_manager:
  worldfusion_endpoint: 'http://localhost:18000'
```

Camera resolution set to WorldFusion's expected `final_dim` (480H × 640W).

## Related

- [LitServe dependency](litserve.md)
- [BM2CP dependency](bm2cp.md)
- [Collaborative Perception concept](../concepts/collaborative_perception.md)
- [worldfusion_litserve_plan.md](../../../agent_plans/worldfusion_litserve_plan.md)
- [Decisions: D5, D6, D9](../decisions.md)
