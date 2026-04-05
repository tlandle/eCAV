---
updated: 2026-04-06
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

## Milestone: Both Fusion Modes Working Distributed (2026-04-06)

Both intermediate fusion (WorldFusion) and late fusion (BM2CP) are confirmed working end-to-end
in distributed mode on Scenario 3. This is the primary milestone for this branch.

**What works:**
- Ego + RSU + non-ego containers all spawn, handshake, and tick correctly
- BM2CP late fusion: actor client → gRPC → C++ server → sim_api → edge_process → BM2CP model
- WorldFusion intermediate fusion: same path + feature extraction via LitServe HTTP, features forwarded via gRPC `pickled_features` field

**Known issue:** WorldFusion distributed is functionally correct but slow. Next work is O5.

## In-Progress Work

### WorldFusion Performance: O5 — Edge-Coordinated Batch Inference

Plan: [worldfusion_litserve_plan.md](../../agent_plans/worldfusion_litserve_plan.md)

**Completed payload optimizations:**
- O1: `spatial_features` sent as float16 → −40% response payload (~10 MB → ~5 MB)
- O4: `imgs` sent as uint8 → −53% request payload (8021 → 3803 KB); lossless; 0.31ms encode cost
- **Fix (2026-04-06)**: `pickled_features` gRPC field was uncompressed despite proto comment saying "Compressed". Added `zlib.compress/decompress` on both ends (~5 MB → ~1-2 MB on the actor→edge hop).

**Pending:**
- **O5**: Batch N agents' feature tensors into a single LitServe call (batch=N). Expected ~50% inference cost reduction per agent. Natural point: edge process collects all actor data before calling LitServe.

**Measured baseline (Phase 1 instrumentation, LitServe hop only):**

| Config | request_KB | http_ms | total_e2e_ms |
|--------|-----------|---------|--------------|
| Baseline (float32) | 8021 | ~291ms | ~355ms |
| +O1 (float16 response) | 8021 | ~175ms | ~234ms |
| +O4 (uint8 imgs) | 3803 | ~175ms | ~233ms |
| O5 (pending) | ~3803 | ~87ms est. | TBD |

## Immediate Next Steps

1. **WorldFusion O5**: Implement `_run_batched_encoder()` in `EdgeProcess.fuse_predictions()` — batch N agents' tensors into single LitServe call. Profile full distributed tick to find other bottlenecks.
2. **Phase 0 YAML cleanup** (non-blocking): Remove `distributed:` field from ~75 YAML files.

## Recent Completions (this branch)

| Date | Item |
|------|------|
| 2026-04-06 | **WorldFusion distributed working** — fixed missing zlib compression on gRPC pickled_features |
| 2026-04-06 | **BM2CP late fusion distributed confirmed** — full Scenario 3 smoke test passed |
| 2026-04-06 | Fixed edge logging silence, non-ego WDT, actors underground; scenario_3.py DRY cleanup |
| 2026-04-04 | Edge actor-ready handshake: gates tick loop on actor readiness |
| 2026-04-04 | Fixed proxy VM sensor crash, EcloudConfig.fatal_errors, perception_pb2 import |
| 2026-04-04 | Phase 6 (`sim_api.py`): `compute_edge_mappings()` + `Server_SetEdgeMappings` call |
| 2026-04-04 | Built knowledge base (`docs/kb/`); full edge architecture codebase audit |
| 2026-03-22 | O4: Send `imgs` as uint8 (−53% payload) |
| 2026-03-22 | O1: Send `spatial_features` as float16 |
| 2026-03-18 | WorldFusion LitServe enablement + NMS fix |
| 2026-03-17 | gRPC migration: YOLO distributed perception (HTTP → gRPC) |

## Uncommitted Modified Files

- `ecav/ecav2/ecloud_actor_client.py` — zlib.compress on pickled_features
- `ecav/scenario_testing/utils/sim_api.py` — zlib.decompress on pickled_features
- `docs/kb/raw/sessions/2026-04-06.md` — session log

## Related

- [Plans Index](plans_index.md)
- [Research](research.md)
