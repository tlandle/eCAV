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

### Edge Architecture — Handshake implemented; smoke test pending

Plans: [edge_architecture_proposal.md](../../agent_plans/edge_architecture_proposal.md), [edge_actor_ready_handshake.md](../../agent_plans/edge_actor_ready_handshake.md)

**Complete:** Phases 1–7 plus edge actor-ready handshake (new protocol to gate tick loop on actor readiness)

**Handshake (new, 2026-04-04):**
- New RPCs: `Edge_ActorReady` (actor→edge), `Edge_ActorsReady` (edge→orchestrator)
- New C++ handler: `numEdgesActorReady_` atomic; fires second `TICK_ID_INVALID` push to sim_api when all edges ready
- `run_comms()` second `push_q.get()` unblocks only after all edges confirm actors ready
- Eliminates "duplicate tick_id:-1": `send_carla_data_to_ecav()` in edge mode calls `Edge_ActorReady` instead of `Client_RegisterVehicle`

**Also fixed (2026-04-04):**
- Proxy VM sensor crash: `localization.activate = False` for orchestrator-side distributed VMs
- `EcloudConfig.fatal_errors` missing attribute
- `perception_pb2` import error in actor containers (missing protos path)

**Pending:**
- **Smoke test**: rebuild Docker image (Y to "Rebuild containers" in `start_actors.sh`), run edge scenario, confirm "All edges actor-ready" in root log before first tick

**Also present but not blocking:**
- Phase 0 partial: `distributed:` field still in ~75 YAML files (all `false`, overwritten by CLI)
- `fuse_predictions()` in `edge_process.py` is placeholder — needed for WorldFusion O5

## Immediate Next Steps

1. **Smoke test**: Rebuild Docker image, run `start_actors.sh` with edge scenario. Verify handshake log sequence before first tick.
2. **WorldFusion O5**: Implement `_run_batched_encoder()` in `EdgeProcess.fuse_predictions()` — batch N agents' tensors into single LitServe call.
3. **Phase 0 YAML cleanup** (non-blocking): Remove `distributed:` field from ~75 YAML files.

## Recent Completions (this branch)

| Date | Item |
|------|------|
| 2026-04-04 | Edge actor-ready handshake: 6-file implementation gates tick loop on actor readiness |
| 2026-04-04 | Fixed proxy VM sensor crash, EcloudConfig.fatal_errors, perception_pb2 import |
| 2026-04-04 | Phase 6 (`sim_api.py`): `compute_edge_mappings()` + `Server_SetEdgeMappings` call |
| 2026-04-04 | Built knowledge base (`docs/kb/`); full edge architecture codebase audit |
| 2026-03-23 | Remove stale cmake artifacts |
| 2026-03-22 | O5 plan doc: edge-coordinated batch inference design |
| 2026-03-22 | O4: Send `imgs` as uint8 (−53% payload) |
| 2026-03-22 | O1: Send `spatial_features` as float16 |
| 2026-03-18 | WorldFusion LitServe enablement + NMS fix |
| 2026-03-18 | WorldFusion registered as git submodule |
| 2026-03-17 | gRPC migration: YOLO distributed perception (HTTP → gRPC) |
| 2026-03-17 | Move agent planning docs to `docs/agent_plans/` |

## Uncommitted Modified Files

Simulation code (from last commit):
- `ecav/scenario_testing/utils/sim_api.py` — Phase 6: `compute_edge_mappings()`, `server_set_edge_mappings()`, call in `run_comms()`
- `start_actors.sh` — 4 fixes: LitServe path, `opencda.py` → `ecav.py` (base/ego/non-ego)
- `Dockerfile` — torch pre-install, PyG wheel index, `opencda/BM2CP` → `ecav/BM2CP`
- `requirements_3_10.txt` — removed stale version pins on `torch_cluster`/`torch_scatter`
- `.claude/ARCHITECTURE.md` — documentation updates
- `docs/agent_plans/worldfusion_litserve_plan.md` — plan updates (O5/O7 fix, distributed section rewrite)
- `docs/agent_plans/edge_architecture_proposal.md` — phase status annotations
- `ecav.py` — entry point updates
- `.gitignore` — exclusion updates
- (deleted) `.claude/EDGE_ARCHITECTURE_PROPOSAL.md` — moved to `docs/agent_plans/`

KB and docs (new this session, untracked):
- `docs/kb/` — entire knowledge base (new)
- `docs/agent_plans/kb_plan.md` — KB implementation plan (new)
- `.claude/CLAUDE.md` — added KB section

## Related

- [Plans Index](plans_index.md)
- [Research](research.md)
