---
updated: 2026-04-27
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

---

## WorldFusion Fixes (2026-04-26) — Committed, Not Pushed

Two bugs fixed in `edge_manager_worldfusion_ab3dmot_linear_predictor.py`, both Tyler-confirmed correct:

**Tyler's fix (commit `c36e6084`, 2026-04-20)** — pairwise transform and coordinate reference:
- `x1_to_x2` destination: `self.world_anchor` → `[0,0,0,0,0,0]` (true world origin)
- `lidar_pose`/`world_anchor` to post-processor: `[self.world_anchor]` → `[[0,0,0,0,0,0]]`
- Final coordinate offset: hardcoded world_anchor → RSU localizer's actual position each tick

**Agent ordering fix (commit `2a9db949`, 2026-04-26)** — RSU must be agent 0:
- WorldFusion's fusion layer warps all agents' BEV features into agent 0's frame; output is in agent 0's coordinate frame. Post-processor treats `lidar_pose=[0,0,0,0,0,0]` (origin) as reference — RSU must be agent 0.
- Was: vehicles collected first (agent 0), RSUs after. Fix: RSUs first (agent 0), vehicles after.
- Also fixed `vehicle_poses = poses[num_rsus:]` in self-beacon filter (was `poses[:num_vehicles]`).
- Tyler's fix was necessary but not sufficient: correct math on wrong-ordered agents still produces wrong output.

**Camera branch guard** (same commit) — `hasattr(sensor, 'camenc')` instead of `sensor.use_camera`: prevents crash on LiDAR-only model variants.

**Detection confirmed** (`openscenario_3_edge_worldfusion --apply_ml`, sequential, clean CARLA session): ticks 99-100, score=0.871/0.907, ~2m position error. ~7m effective range expected — V2XSim training data is intersection-centric. Ego does not collide with Lincoln; full pipeline confirmed end-to-end.

Commits: `2a9db949` (fix), `647733e4` (logging + KB). Not pushed.

---

## Next: Full Regression Matrix

All 8 permutations must pass before moving to multi-ego (`openscenario_3_edge_worldfusion_4ego`). Cannot assume prior results hold — agent-ordering fix touches the edge manager in every mode.

Flags: `--apply_ml` enables ML; `-l` routes inference to external gRPC server; `-d` distributed actors.

| # | Fusion | `-l` | `-d` | Status |
|---|---|---|---|---|
| 1 | WorldFusion | no | no | ✓ 2026-04-26 |
| 2 | WorldFusion | yes | no | — |
| 3 | WorldFusion | no | yes | ✓ 2026-04-29 |
| 4 | WorldFusion | yes | yes | — |
| 5 | Late fusion | no | no | — |
| 6 | Late fusion | yes | no | — |
| 7 | Late fusion | no | yes | — |
| 8 | Late fusion | yes | yes | — |

Run in order 1→8: sequential before distributed, WorldFusion before late fusion.

---

## Multi-Ego Scenarios: Distributed Readiness (2026-04-27)

**All multi-ego scenarios (`_4ego`, `_2ego`, `_8ego`, `_16ego`) are distributed-only** — they `assert opt.distributed` at startup and will refuse to run without `-d`.

**Teardown**: Applied `num_completed_vehicles` fix to all 5 multi-ego files plus both single-ego files. Encapsulated as `ScenarioManager.all_vehicles_done` property in `sim_api.py`. All 7 scenario files now call `scenario_manager.all_vehicles_done` instead of the inline check.

---

## Distributed Teardown Bug — Fixed and Verified (2026-04-29)

**Symptom**: `openscenario_3_edge_worldfusion -d` never exits — ego keeps getting ticks after reaching destination, edge eventually times out, start_actors gives up after 5 min.

**Root cause (two-layer)**:

1. **Edge barrier**: `edge_process.py::process_tick` waits for `len(self.actors)` updates via `Edge_ActorSendUpdate` every tick. Once ego sends `TICK_DONE` it stops calling `send_vehicle_update`. Edge hangs on every subsequent tick (30 s timeout). Partial fix already applied: `is_done` flag on `EdgeActorInfo`, barrier now uses `expected = sum(1 for a if not a.is_done)`.

2. **Orchestrator visibility**: `Edge_TickComplete` in the C++ server does nothing but count edges. `pendingReplies_` is never populated in edge mode — vehicles talk to the edge, not the C++ server, so `Client_SendUpdate` is never called. Python's `server_unpack_vehicle_updates` finds nothing, `num_completed_vehicles` stays zero, `all_vehicles_done` never fires. This is the deeper problem.

**Why "edge done" is the wrong framing**: Edges manage geographic locales — vehicles enter and exit. Only vehicles are permanently done. The C++ orchestrator must remain the single source of truth for per-vehicle doneness.

**Architecture**: Edge forwards individual vehicle state-change events (TICK_DONE) to C++ via a new `repeated VehicleUpdate vehicle_updates` field in `EdgeTickComplete`. C++ processes them identically to a direct `Client_SendUpdate(TICK_DONE)` — pushes to `pendingReplies_`, increments `numCompletedVehicles_`, behind an idempotency set. Python's existing path works unchanged.

**C++ global state model** (target):
- `numEdgesRepliedTick_` (rename from `numCompletedEdges_`) — per-tick edge reply counter, reset each tick
- `completedVehicleIndices_` (new) — permanent set; `.size()` replaces `numCompletedVehicles_` as done-vehicle count (O(1), no redundant counter)

**Plan**: `docs/agent_plans/edge_tick_complete_summary.md`

**Status**: Implemented and verified (2026-04-29). Clean exit confirmed on `openscenario_3_edge_worldfusion --apply_ml -d`. Not yet committed.

**Changes made**:
- `ecav/protos/ecloud.proto`: added `repeated VehicleUpdate vehicle_updates = 4` to `EdgeTickComplete`
- Python stubs regenerated (root `ecloud_pb2.py` + `ecav/protos/`)
- `ecav/ecloud_server/ecloud_server.cc`: renamed `numCompletedEdges_` → `numEdgesRepliedTick_`; added `std::set<int32_t> completedVehicleIndices_`; `Edge_TickComplete` now processes `vehicle_updates` (check set, insert, push to `pendingReplies_` under `mu_`); `Edge_Register` now populates `edgeInfo.vehicle_indices` before pushing to `edgeInfos_`
- C++ server rebuilt cleanly
- `ecav/ecav2/edge_process.py`: `push_tick_to_actors` skips done actors; `process_tick` snapshots `prior_done` before tick, computes `newly_done` after wait loop; `report_tick_complete` populates `vehicle_updates` for newly-done VEHICLE actors

**Next**: commit, then continue regression matrix (tests 3–8)

---

## Code Quality Changes (2026-04-27)

- `ecav/utils.py` (new): `find_unpicklable(obj, path="")` — pure recursive helper, no side effects
- `ecav/ecav2/ecloud_actor_client.py`: removed inline `find_unpicklable` definition, converted serialization error prints to `logger.error`
- `ecav/distributed_client/distributed_actor_client.py`: removed `_debug_unpicklable_objects` method, same cleanup, removed stray `print("Edge Predictions:")`
- `ecav/.claude/CLAUDE.md`: added "Code Quality: Progressive Cleanup Policy" section (print→logger driveby on any file touched) and "Plans" section overriding plan directory to `docs/agent_plans/`

---

## WIP / Exploratory

### Azure Distributed Deployment (2026-04-06)

Planning phase. See [azure_deploy.md](../../agent_plans/azure_deploy.md).

**Topology**: 5-node split — node-0 (CARLA + ecav.py), node-1 (ecloud_server), node-2 (inference), node-3 (GPU actors), node-4 (CPU actors).

**Key finding**: codebase is already ~90% wired for multi-node. `sim_api.py:580` already skips spawning `ecloud_server` when `ECLOUD_IP != 'localhost'`. Only code change needed: `ecav.py:43` hardcodes `ECLOUD_SERVER_ADDRESS = "localhost:50051"` — must read from `cloud_config.yaml` for actor containers on remote nodes.

**Approach**: Ansible for cluster orchestration; per-node startup scripts extracted from `start_actors.sh`; `cloud_config.yaml` rendered per-node from Jinja2 template.

**Status**: Plan written, not yet implemented.

---

## Backlog

### multiv2x_mtr placeholder feature cache in git (follow up with Tyler)

`ecav/ml_manager/models/multiv2x_mtr/multiv2x_fused_features_placeholder/` — 11,136 sparse `.npy` files. Apparent size ~84GB, actual disk ~218MB. Were committed to git and included in Docker build context, inflating the Docker image from ~17GB to ~130GB.

Fixed (2026-04-27): removed from git tracking (`git rm --cached -r`), added to `.gitignore` and `.dockerignore`.

**Follow up with Tyler**: confirm whether these files should be regenerated at container startup, pulled from a separate store, or excluded entirely. The directory name ("placeholder") suggests they're pre-allocated slots for a feature cache, not actual trained data — but Tyler should confirm the intended workflow.

---

### `vehicle_count` / `num_completed_vehicles` are class variables, not instance variables (`sim_api.py`)

Declared at class scope (lines 276-278) but mutated on the instance. Python's attribute lookup finds the instance copy on write, so it works correctly — but the intent is clearer and safer if they're initialized in `__init__`. Low-priority cleanup; don't fix in isolation, fix when `ScenarioManager.__init__` is already being touched for another reason.

---

### Remove `isEdge_` and associated `is_edge` logic (`ecloud_server.cc`)

`isEdge_` (set in `Server_StartScenario`, L638) gates the `pendingReplies_` path in `Client_SendUpdate` — a path irrelevant in edge mode since vehicles never call it directly. Appears to be legacy from the original edge-as-graph-algorithm implementation (edge overrode waypoints; no perception). Dead code in the current architecture. Remove in a separate cleanup PR; don't conflate with the `edge_tick_complete` fix.

---

### Replace `print` calls with leveled `logger` calls across the codebase

The repo has accumulated a large number of bare `print(...)` calls that should be `logger.debug/info/warning/error` calls. Policy going forward: **any file we touch gets a sweep** — convert prints to properly leveled logger calls and remove any inline debug scaffolding (ad-hoc debug prints, commented-out debug blocks, dead `if verbose:` branches, etc.).

This is a progressive cleanup, not a single-pass effort. Prioritize files touched for other reasons; don't make it a standalone driveby.

---

### Invalid metrics in PlanningMetrics summary dict (`ecav/core/plan/planning_metrics.py`)

- **`distance_traveled_m`**: `update()` skips first 100 ticks (`count > 100` warmup filter) — understates total trip distance by ~5 sim seconds of travel.
- **`edge_ticks_total` + 5 sibling fields**: defined with comment "Confound A diagnostic" but never incremented anywhere. Always zero. Real edge data is in the edge profiler JSON and timing CSVs.

TODO: fix or delete both.

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
