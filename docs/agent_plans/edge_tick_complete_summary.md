# Edge Tick Complete: Vehicle State Propagation to Orchestrator

## Problem

The edge process hangs after ego sends `TICK_DONE`.

**Root cause**: `edge_process.py::process_tick` waits for `len(self.actors)` updates via
`Edge_ActorSendUpdate` every tick. Once ego sends `TICK_DONE` it stops calling
`send_vehicle_update`. Edge times out (30 s) every subsequent tick.

Deeper: `Edge_TickComplete` in the C++ server currently does nothing but count edges.
`pendingReplies_` is never populated in edge mode (`Client_SendUpdate` is never called —
vehicles talk to the edge, not the C++ server). Python's `server_unpack_vehicle_updates`
reads `pendingReplies_`, finds nothing, `num_completed_vehicles` stays zero,
`all_vehicles_done` never fires.

## Why "edge done" is the wrong framing

An edge manages a geographic locale. Vehicles enter and exit. Only **vehicles** are done —
permanently and globally. The C++ orchestrator must remain the single source of truth for
per-vehicle doneness. The edge is an intermediary, not an owner of done state.

## Correct architecture

Edge forwards individual vehicle state-change events to C++ via
`EdgeTickComplete.vehicle_updates`. On the tick a vehicle sends TICK_DONE, the edge
includes that vehicle's `VehicleUpdate` in the `EdgeTickComplete`. C++ processes it
identically to a direct `Client_SendUpdate(TICK_DONE)`: pushes to `pendingReplies_`,
increments `numCompletedVehicles_`. Python's existing path works unchanged.

## C++ findings (from reading `ecloud_server.cc`)

| Item | Finding |
|------|---------|
| `Edge_TickComplete` (L765) | Increments `numCompletedEdges_`, fires `PushTick` when all edges report. Nothing else. |
| `pendingReplies_` (L171) | Populated only by `Client_SendUpdate`. Never touched in edge mode. |
| `numCompletedVehicles_` (L122) | Atomic. Never reset. **Replaced by `completedVehicleIndices_.size()`** — set size is O(1), redundant counter eliminated. |
| `Server_SetEdgeMappings` (L588) | Populates `vehicleToEdgeMapping_` + `rsuToEdgeMapping_`. |
| `Edge_Register` (L734) | Reads those maps to send `vehicle_indices` back to edge. Does NOT write them into `EdgeInfo.vehicle_indices` — gap. |
| `numCars_` (L637) | Set to `vehicle_count + rsu_count` from Python. Only used in the direct `Client_SendUpdate` barrier — irrelevant in edge mode. |
| `isEdge_` (L638) | Set in `Server_StartScenario`. Controls `pendingReplies_` in `Client_SendUpdate` — irrelevant in edge mode since that path is never called. |

## Idempotency

Add `std::set<int32_t> completedVehicleIndices_` in the C++ server. Insert on first
TICK_DONE; skip if already present (guard against retry/restart double-counting).
`completedVehicleIndices_.size()` replaces `numCompletedVehicles_` as the done-vehicle
count — no separate atomic counter needed; set size is O(1).

The set operations (find + insert) go inside the same `mu_` lock already held for
`pendingReplies_.push_back` — one lock acquisition covers both. If we ever needed a
lockless alternative, `bool completedCars_[MAX_CARS]` would work: single-element bool
write is atomic by alignment, count trues on each update, no CAS or mutex required.
We don't need it here because we're already paying for `mu_`.

## Verification

`openscenario_3_edge_worldfusion -d` exits cleanly after ego reaches destination — no
5-min hang, no edge timeout warnings, eval runs, containers shut down.

---

## Scope

Four layers in lockstep:

1. **Proto** — add `repeated VehicleUpdate vehicle_updates` to `EdgeTickComplete`
2. **C++ orchestrator** — process `vehicle_updates` in `Edge_TickComplete`; add idempotency set; populate `EdgeInfo.vehicle_indices` at registration
3. **Edge process** — populate `vehicle_updates` for newly-done vehicles; fix local barrier (already partially done)
4. **Python `ScenarioManager`** — no change needed; existing path works once C++ is fixed

---

## Checklist

### 1. Proto: extend `EdgeTickComplete`

- [x] Add `repeated VehicleUpdate vehicle_updates = 4` to `EdgeTickComplete` in `ecav/protos/ecloud.proto`
- [x] Recompile stubs: `python ecav.py --build`

```proto
message EdgeTickComplete {
  int32 edge_index = 1;
  int32 tick_id = 2;
  int32 num_actors_processed = 3;
  repeated VehicleUpdate vehicle_updates = 4;  // state-change events only (e.g. TICK_DONE)
}
```

Note: summary counts (`num_vehicles`, `num_vehicles_done`) intentionally omitted — orchestrator
derives counts from its own state, not from edge summaries.

### 2. C++ orchestrator: `Edge_TickComplete` + global state

- [x] Rename `numCompletedEdges_` → `numEdgesRepliedTick_` everywhere it appears (global decl, constructor, `Server_DoTick` reset, `Edge_TickComplete` handler) — driveby cleanup; parallel to `numRepliedVehicles_`
- [x] Add `std::set<int32_t> completedVehicleIndices_` to globals (edge-path idempotency guard; `numCompletedVehicles_` retained for direct-path barrier — the two paths are disjoint at runtime)
- [x] Add to `EcloudServiceImpl` constructor init: `completedVehicleIndices_.clear()`
- [x] In `Edge_TickComplete` handler, after incrementing `numEdgesRepliedTick_`:
  - Iterate `request->vehicle_updates()`
  - For each update where `vehicle_state() == VehicleState::TICK_DONE`:
    - If `vehicle_index()` NOT in `completedVehicleIndices_`:
      - Insert into `completedVehicleIndices_`
      - Serialize update, lock `mu_`, push to `pendingReplies_`, unlock
- [x] In `Edge_Register`, after building `edgeInfo`: populate `edgeInfo.vehicle_indices`
  from `vehicleToEdgeMapping_` (same loop that builds the reply — store in `edgeInfo` too)

### 3. Edge process: populate `vehicle_updates` + fix local barrier

- [x] `EdgeActorInfo.is_done`: already added — keep
- [x] `Edge_ActorSendUpdate`: `is_done = True` on TICK_DONE — already added — keep
- [x] `process_tick` barrier: `expected = sum(1 for a in self.actors.values() if not a.is_done)` — already added — keep
- [x] `push_tick_to_actors`: skip done actors (fixes push_q assertion on done ego)
- [x] In `process_tick`: track which actors became newly done this tick
  - Before pushing tick: snapshot current done set (`prior_done`)
  - After waiting loop: diff against snapshot to find `newly_done`
  - Pass `newly_done` to `report_tick_complete`
- [x] In `report_tick_complete`: populate `request.vehicle_updates` with `last_update`
  for each newly-done actor (only vehicle type; skip RSUs)

### 4. Python `ScenarioManager`

- [x] No changes needed — `server_unpack_vehicle_updates` already reads `pendingReplies_`
  and increments `num_completed_vehicles` on TICK_DONE
- [x] `all_vehicles_done` property already in place
- [ ] Verify `vehicle_count` == number of vehicles in edge (currently 1 for worldfusion) — should already be correct from `edge_list[0]['vehicles']` path

---

## Backlog / Out of Scope

- **`isEdge_` cleanup**: `isEdge_` (set in `Server_StartScenario`, L638) and all associated `is_edge` logic appears to be legacy from the original edge-as-graph-algorithm implementation, where the edge overrode waypoints rather than doing perception. It gates the `pendingReplies_` path in `Client_SendUpdate` — a path that's irrelevant in edge mode since vehicles never call it directly. Remove `isEdge_`, the conditional it guards, and all call sites. Captures latent dead code; defer to a separate cleanup PR.

---

## Files Touched

| File | Change |
|------|--------|
| `ecav/protos/ecloud.proto` | Add `vehicle_updates` to `EdgeTickComplete` |
| `ecav/protos/ecloud_pb2.py` / `*_pb2_grpc.py` | Regenerated stubs |
| `ecav/ecloud_server/ecloud_server.cc` | `completedVehicleIndices_`; process `vehicle_updates` in `Edge_TickComplete`; populate `EdgeInfo.vehicle_indices` |
| `ecav/ecav2/edge_process.py` | Track `newly_done`; populate `vehicle_updates` in `EdgeTickComplete` |
| `ecav/scenario_testing/utils/sim_api.py` | No change expected |
