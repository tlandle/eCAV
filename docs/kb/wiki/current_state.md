---
updated: 2026-04-04
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

## In-Progress Work

### WorldFusion LitServe: O5 — Edge-Coordinated Batch Inference

Plan: [worldfusion_litserve_plan.md](../../agent_plans/worldfusion_litserve_plan.md)

**Completed on this branch:**
- O1: `spatial_features` sent as float16 → −40% response payload (float32 response: ~10 MB → ~5 MB)
- O4: `imgs` sent as uint8 → −53% request payload (8021 → 3803 KB); lossless on CARLA camera output; 0.31ms encode cost

**Pending:**
- **O5**: Merge N agents' feature tensors into a single batch=N HTTP call to LitServe. Expected ~50% inference cost reduction per agent. Natural implementation point: edge process collects all actor data before calling LitServe.

**Measured baseline (Phase 1 instrumentation):**

| Config | request_KB | http_ms | total_e2e_ms |
|--------|-----------|---------|--------------|
| Baseline (float32) | 8021 | ~291ms | ~355ms |
| +O1 (float16 response) | 8021 | ~175ms | ~234ms |
| +O4 (uint8 imgs) | 3803 | ~175ms | ~233ms |
| O5 (pending) | ~3803 | ~87ms est. | TBD |

### Edge Architecture — Phase 0 Pre-requisites

Plan: [edge_architecture_proposal.md](../../agent_plans/edge_architecture_proposal.md)

8-phase rollout. **Phase 0 not yet started.** Key Phase 0 items:
- Add `--litserve` (`-l`) CLI flag; remove `distributed` field from YAML files
- Rename `distributed` → `litserve` in MLManager
- Clarify the two distinct "distributed" concepts (simulation vs. ML inference)

## Immediate Next Steps

1. Implement WorldFusion O5: edge-coordinated batch inference
2. Begin edge architecture Phase 0 flag/YAML cleanup

## Recent Completions (this branch)

| Date | Item |
|------|------|
| 2026-03-23 | Remove stale cmake artifacts |
| 2026-03-22 | O5 plan doc: edge-coordinated batch inference design |
| 2026-03-22 | O4: Send `imgs` as uint8 (−53% payload) |
| 2026-03-22 | O1: Send `spatial_features` as float16 |
| 2026-03-18 | WorldFusion LitServe enablement + NMS fix |
| 2026-03-18 | WorldFusion registered as git submodule |
| 2026-03-17 | gRPC migration: YOLO distributed perception (HTTP → gRPC) |
| 2026-03-17 | Move agent planning docs to `docs/agent_plans/` |

## Uncommitted Modified Files

- `.claude/ARCHITECTURE.md` — documentation updates
- `docs/agent_plans/worldfusion_litserve_plan.md` — plan updates
- `ecav.py` — entry point updates
- `.gitignore` — exclusion updates
- (deleted) `.claude/EDGE_ARCHITECTURE_PROPOSAL.md` — content moved to `docs/agent_plans/`

## Related

- [Plans Index](plans_index.md)
- [Research](research.md)
