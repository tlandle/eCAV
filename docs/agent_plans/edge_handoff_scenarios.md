# Multi-Edge Sequential Handoff: Integration Scenarios

**Plan path (final):** `docs/agent_plans/edge_handoff_scenarios.md`  
**Phase 1 plan ref:** `docs/agent_plans/edge_handoff_phase1_state_transfer.md` (Steps 0–4 done)  
**Branch:** `develop`

---

## Context

Steps 0–4 of Phase 1 are complete: `KFState`, `HandoffManager`, `SequentialMigrationDaemon`,
`InterLocaleLink`, `TransferCost`, state store in `sim_api.py`, AB3DMOT export/import on both
`_BaseEdgeManager` and `_PluggableEdgeBase`, and the no-CARLA smoke test (6 tests passing).

What remains is live CARLA scenarios that exercise the daemon in a real simulation loop.
Two scenarios are needed, with a small code extension required for the second:

- **Scenario A (Town03)** — ego crosses a locale boundary; proves daemon plumbing (ownership
  move + cost record) in a real CARLA tick loop. Tick-based trigger. Reuses existing
  `Scenario_3` infrastructure unchanged.

- **Scenario B (Town06)** — the research scenario. Fast NPC approaching from behind in locale 0
  crosses into locale 1 where ego is attempting a forced left merge (emergency vehicle blocks
  right lane). State transfer fires when the NPC crosses the locale boundary; locale 1 gets the
  NPC's KF tracking history early, before its RSU can directly see the NPC. Geometry-based
  trigger via `VehicleLocaleTracker`. Requires a small obstacle-export extension to the edge
  manager and daemon.

Build A first. Do not start B until A passes all verification criteria.

---

## Code Extension Required for Scenario B

The current `export_vehicle_state(vid)` guards against vehicles with no `VehicleManager`.
For the NPC obstacle, there is no VehicleManager — it's tracked only in AB3DMOT, not
managed as a CAV. Three small additions are needed before Scenario B can be built:

### 1. `_PluggableEdgeBase.export_tracked_obstacle_state(carla_id)` — new method

```python
def export_tracked_obstacle_state(self, carla_id: int) -> Optional[MigrationPayload]:
    """Export KF state for any AB3DMOT-tracked obstacle (no VehicleManager required)."""
    kf_obj = next(
        (t for t in self.tracker.trackers if t.carla_id == carla_id), None
    )
    if kf_obj is None:
        return None
    ks = KFState(
        state_vector=kf_obj.kf.x.flatten().copy(),
        covariance=kf_obj.kf.P.copy(),
        hits=kf_obj.hits,
        anchoring_age=kf_obj.anchoring_age,
    )
    tid = next((t for t, c in self.track_to_carla.items() if c == carla_id), -1)
    track = TrackLatent(
        track_id=tid,
        persistent_vehicle_id=carla_id,
        hidden_state=np.zeros(1, dtype=np.float16),
        kf_state=ks,
    )
    return MigrationPayload(
        source_locale_id="", destination_locale_id="",
        trigger_time_s=0.0, tracks=[track],
    )
```

Also add a stub on `_BaseEdgeManager` that returns `None` (no-op for non-AB3DMOT backends).

### 2. `_PluggableEdgeBase.import_tracked_obstacle_state(carla_id, payload)` — new method

Same logic as `import_vehicle_state`, but no relinquish/accept — obstacle stays tracked in
locale 0 concurrently. Locale 1 gets a warm start; both edges can see the NPC once it enters
locale 1's direct RSU range.

```python
def import_tracked_obstacle_state(self, carla_id: int, payload: MigrationPayload) -> None:
    """Inject a warm KF for an obstacle into this edge's AB3DMOT tracker."""
    track = next(
        (t for t in payload.tracks if t.persistent_vehicle_id == carla_id), None
    )
    if track is None or track.kf_state is None:
        logger.warning("import_tracked_obstacle_state: no KF for carla_id %d", carla_id)
        return
    ks = track.kf_state
    new_tid = self.tracker.ID_count[0]
    self.tracker.ID_count[0] += 1
    info = np.array([0, -1, carla_id])
    new_kf = _AB3DMOT_KF(ks.state_vector[:7], info, new_tid)
    new_kf.kf.x = ks.state_vector.reshape(10, 1).copy()
    new_kf.kf.P = ks.covariance.copy()
    new_kf.carla_id = carla_id
    new_kf.hits = max(ks.hits, self.tracker.min_hits)
    new_kf.time_since_update = 0
    new_kf.anchoring_age = ks.anchoring_age
    self.tracker.trackers.append(new_kf)
    self.track_to_carla[new_tid] = carla_id
    logger.info(
        "import_tracked_obstacle_state: carla_id=%d -> tid=%d (hits=%d)",
        carla_id, new_tid, new_kf.hits,
    )
```

Also add a no-op stub on `_BaseEdgeManager`.

### 3. `SequentialMigrationDaemon.transfer_obstacle_state(...)` — new method

Like `request_handoff` but without relinquish/accept (no VehicleManager ownership move).
Calls `export_tracked_obstacle_state` on src, `import_tracked_obstacle_state` on dst,
records `TransferCost` via `InterLocaleLink.model_transfer`.

```python
def transfer_obstacle_state(
    self,
    carla_id: int,
    src_edge: "_BaseEdgeManager",
    dst_edge: "_BaseEdgeManager",
    link: InterLocaleLink,
    tick: int,
) -> Optional[TransferCost]:
    payload = src_edge.export_tracked_obstacle_state(carla_id)
    if payload is None:
        logger.warning("transfer_obstacle_state: no payload for carla_id %d", carla_id)
        return None
    dst_edge.import_tracked_obstacle_state(carla_id, payload)
    cost = link.model_transfer(payload, src_edge, dst_edge, tick)
    logger.info(
        "[OBSTACLE_HANDOFF] carla_id=%d %s->%s tick=%d bytes=%d total_ms=%.3f",
        carla_id, src_edge.edgeid, dst_edge.edgeid, tick, cost.payload_bytes, cost.total_ms,
    )
    return cost
```

Note: no `store` parameter — obstacle state is not persisted in the vehicle_state_store
(which is keyed by managed vehicle ID). The transfer is a one-shot push.

**Files modified for extension:**
- `ecav/core/application/edge/edge_manager/edge_manager_base.py` — add two no-op stubs
- `ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py` — add two implementations
- `ecav/core/application/edge/migration/daemon.py` — add `transfer_obstacle_state`

These additions are independent of Scenario A and can be built in the same PR.

---

## Scenario A — Town03 Locale Traversal

### What it proves
- Daemon runs in a real CARLA tick loop without error.
- Ownership moves: `edge_list[0].vehicle_manager_list` empties; `edge_list[1]` gains cav1.
- `TransferCost` record emitted with correct fields.
- Ego continues to receive edge predictions after handoff.

### Files

| File | Action |
|---|---|
| `ecav/scenario_testing/scenarios/scenario_3.xml` | Unchanged |
| `ecav/scenario_testing/scenarios/scenario_3.py` | Unchanged |
| `ecav/scenario_testing/config_yaml/openscenario_3_multi_edge_late_fusion.yaml` | **New** |
| `ecav/scenario_testing/openscenario_3_multi_edge_late_fusion.py` | **New** |

### YAML — `openscenario_3_multi_edge_late_fusion.yaml`

Copy `openscenario_3_edge_late_fusion.yaml` verbatim. Change only `scenario_name` and
`scenario.edge_list` — add a second edge with RSU but no vehicles:

```yaml
scenario_name: openscenario_3_multi_edge_late_fusion

scenario:
  edge_list:
    - <<: *edge_base                         # Edge 0 — lower corridor (y ≈ 80–108)
      manager_type: late_fusion
      predictor_type: linear
      rsus:
        - <<: *rsu_base
          name: rsu0
          spawn_position: [-82.0, 95.0, 7.0]
          id: 0
      vehicles:
        - <<: *vehicle_base
          name: cav1
          destination: [-28.8, 134.7, 0.5]

    - <<: *edge_base                         # Edge 1 — upper corridor / intersection (y ≈ 98–135)
      manager_type: late_fusion
      predictor_type: linear
      rsus:
        - <<: *rsu_base
          name: rsu1
          spawn_position: [-82.0, 120.0, 7.0]
          id: 1
      # No vehicles — cav1 migrates here at tick HANDOFF_TICK
```

Reference locale geometry (not wired — Phase A uses tick-based trigger):
- Locale 0: y ∈ [75, 108], x ∈ [−90, −78]
- Locale 1: y ∈ [98, 135], x ∈ [−90, −78]
- Overlap: y ∈ [98, 108]

### ecav `.py` — `openscenario_3_multi_edge_late_fusion.py`

Copy `openscenario_3_edge_late_fusion.py`. Changes:

**New imports:**
```python
from ecav.core.application.edge.migration import (
    HandoffManager, SequentialMigrationDaemon, InterLocaleLink,
)
```

**Pre-loop (after `edge_list` is created):**
```python
HANDOFF_TICK = 60   # ego at y≈116 at 43 km/h, well into locale 1 overlap
handoff_manager = HandoffManager(trigger_tick=HANDOFF_TICK)
daemon = SequentialMigrationDaemon()
link = InterLocaleLink(edge_list[0].latency_model)
handoff_done = False
transfer_costs = []
vid = None
```

**In the tick loop (after `edge.run_step(step)` calls):**
```python
# Per-tick snapshot upload
for edge in edge_list:
    for vm in edge.vehicle_manager_list:
        payload = edge.export_vehicle_state(vm.vehicle.id)
        if payload:
            scenario_manager.store_vehicle_state(vm.vehicle.id, payload)

# Tick-based handoff trigger
if not handoff_done:
    if vid is None and edge_list[0].vehicle_manager_list:
        vid = edge_list[0].vehicle_manager_list[0].vehicle.id
    if vid is not None:
        event = handoff_manager.evaluate(vid, step, step * world_dt)
        if event:
            cost = daemon.request_handoff(
                vid, edge_list[0], edge_list[1], scenario_manager, link, step
            )
            transfer_costs.append(cost)
            handoff_done = True
            logger.info(
                "[HANDOFF] tick=%d vid=%d bytes=%d total_ms=%.3f",
                step, vid, cost.payload_bytes, cost.total_ms,
            )
```

**In `finally` block:**
```python
for cost in transfer_costs:
    logger.info(
        "[TRANSFER_COST] vid=%d tick=%d bytes=%d "
        "serialize_ms=%.4f network_ms=%.4f total_ms=%.4f",
        cost.vehicle_id, cost.tick, cost.payload_bytes,
        cost.sim_serialize_ms, cost.sim_network_ms, cost.total_ms,
    )
```

### Verification (Scenario A)

`python ecav.py -t openscenario_3_multi_edge_late_fusion`

1. `[HANDOFF]` log appears exactly once near tick 60.
2. After tick 60: `edge_list[0].vehicle_manager_list` empty; `edge_list[1]` has 1 vehicle.
3. `[TRANSFER_COST]` line with `bytes > 0`, `network_ms > 0`.
4. Scenario runs to completion (≤ 250 ticks) without crash.
5. `ghost_brake_events=0` in final metrics (regression check).

---

## Scenario B — Town06 Left-Merge / Emergency Vehicle (Obstacle Handoff)

### What it proves

- `VehicleLocaleTracker` geometry trigger fires correctly in a live CARLA scenario.
- Obstacle-triggered handoff: locale 1 receives the fast NPC's KF state from locale 0 BEFORE
  locale 1's RSU can directly detect the NPC.
- The advance warning window: compare tick T_handoff (NPC crosses locale boundary) vs.
  T_direct_detect (NPC first enters locale 1's RSU detection range).
- Ego does not merge into an unsafe gap because locale 1 already has the NPC's predicted
  trajectory at the time of the merge decision.

### Geometry (Town06 east-west highway, y ≈ 134–150)

Town06 eastbound lanes: right (inner) at y ≈ 139.5, left (outer) at y ≈ 143.5.

```
x:   0        75      140  160      230     310
     |         |       |OVL|        |       |
     [----------- Locale 0 ---------]
                              [--------- Locale 1 ----------]
     fast NPC  RSU0                  RSU1
     (left lane, 80km/h east)
                      ego (right lane, 43km/h east)
                                              emrg vehicle (right lane, stationary)
```

**Actor spawns:**

| Actor | x | y | yaw | Speed |
|---|---|---|---|---|
| Ego (CAV) | 100 | 139.5 | 0 | 43 km/h |
| Emergency vehicle (stationary) | 280 | 139.5 | 0 | 0 |
| Fast NPC | 0 | 143.5 | 0 | 80 km/h (22.2 m/s) |

**Locale 0:** x ∈ [0, 160], y ∈ [134, 150] — RSU at (75, 141, 7)  
**Locale 1:** x ∈ [140, 310], y ∈ [134, 150] — RSU at (230, 141, 7)  
**Overlap:** x ∈ [140, 160]

**Advance warning calculation (with SyncArrival):**
- `SyncArrival` ensures: NPC at x=220 and ego at x=250 simultaneously.
- Ego travel from x=100 to x=250 at 11.9 m/s ≈ 12.6s = 252 ticks.
- During SyncArrival, NPC travels 220m in 12.6s → average speed ≈ 57 km/h.
- NPC crosses locale boundary (x=140) at t = 140/57km/h ≈ 8.8s = 176 ticks → locale tracker fires → handoff at tick ≈ 180.
- Locale 1 RSU (at x=230, effective camera detection ~60m) first directly sees NPC when NPC reaches x≈170 at t ≈ 10.7s = 214 ticks.
- **Advance warning = 214 − 180 = 34 ticks (1.7 s)**
- At tick 180: ego at x=100+11.9×9=206, emergency vehicle at x=280, gap=74m. Ego is approaching the merge zone but has not yet initiated it — locale 1 gets the NPC's KF state with room to warn ego before the merge is attempted.

### Files to create (Scenario B)

| File | Purpose |
|---|---|
| `ecav/scenario_testing/scenarios/scenario_multi_edge_left_merge.xml` | Actor spawns |
| `ecav/scenario_testing/scenarios/scenario_multi_edge_left_merge.py` | Behavior tree |
| `ecav/scenario_testing/config_yaml/openscenario_multi_edge_left_merge.yaml` | Two-edge config + locale bounds |
| `ecav/scenario_testing/openscenario_multi_edge_left_merge.py` | Scenario loop with geometry trigger |

### XML — `scenario_multi_edge_left_merge.xml`

```xml
<scenarios>
  <scenario name="Scenario_MultiEdgeLeftMerge"
            type="Scenario_MultiEdgeLeftMerge"
            town="Town06">
    <ego_vehicle x="100.0" y="139.5" z="1.0" yaw="0"
                 model="vehicle.nissan.patrol" />
    <!-- Emergency vehicle: underground until teleported -->
    <other_actor x="280.0" y="139.5" z="-500" yaw="0"
                 model="vehicle.ford.ambulance" />
    <!-- Fast NPC: underground until teleported -->
    <other_actor x="0.0"   y="143.5" z="-500" yaw="0"
                 model="vehicle.tesla.model3" />
  </scenario>
</scenarios>
```

### Scenario runner `.py` — `scenario_multi_edge_left_merge.py`

New `BasicScenario` subclass `Scenario_MultiEdgeLeftMerge`.

`_initialize_actors`: same underground → visible transform pattern as `scenario_3.py`.
`self.other_actors[0]` = emergency vehicle, `self.other_actors[1]` = fast NPC.

`_create_behavior`: Uses `SyncArrival` to coordinate NPC arrival at the locale boundary
with ego's merge-decision point. Without it, timing is sensitive to ego's acceleration
profile; `SyncArrival` makes it deterministic.

```python
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import SyncArrival

# Sync targets: NPC reaches x=220 (left lane) at the same time
# ego reaches x=250 (right lane, 30m from emergency vehicle at x=280).
# Gap = 30m in adjacent lane, NPC at 22 m/s, ego at ~12 m/s → TTC ≈ 3s → merge unsafe.
npc_sync_wp = self._map.get_waypoint(carla.Location(x=220, y=143.5, z=1))
ego_sync_wp = self._map.get_waypoint(carla.Location(x=250, y=139.5, z=1))

root = Parallel("Root", policy=SUCCESS_ON_ONE)

# Emergency vehicle: teleport, stay idle (blocks right lane)
emrg_seq = Sequence("EmrgVehicle")
emrg_seq.add_child(ActorTransformSetter(emrg, emrg_visible))
emrg_seq.add_child(Idle())

# Fast NPC:
#   1. Teleport to x=0, left lane
#   2. SyncArrival: PID-controlled speed so NPC reaches x=220 when ego reaches x=250
#      (during sync, NPC speed ≈ 57 km/h; fast enough to feel like an oncoming vehicle)
#   3. WaypointFollower at 22 m/s (80 km/h) for the remainder — unsafe speed post-sync
fast_seq = Sequence("FastNPC")
fast_seq.add_child(ActorTransformSetter(npc, npc_visible))
fast_seq.add_child(SyncArrival(npc, self.ego_vehicles[0], npc_sync_wp, ego_sync_wp))
fast_seq.add_child(WaypointFollower(npc, 22.0))
fast_seq.add_child(Idle())

# Terminate after ego drives 300m or timeout
root.add_child(emrg_seq)
root.add_child(fast_seq)
root.add_child(DriveDistance(ego_vehicles[0], 300))
return root
```

**Sync target tuning note:** the exact NPC sync waypoint (x=220) and ego sync waypoint (x=250)
will need one iteration after a test run to verify TTC ≈ 3s at the merge decision moment.
Move the ego target closer to x=280 to tighten the gap, or farther to relax it.

`_create_test_criteria`: `CollisionTest(ego_vehicles[0])`.

### YAML — `openscenario_multi_edge_left_merge.yaml`

Key differences from Scenario A:
- `scenario_runner.town: Town06`
- `scenario_runner.scenario: Scenario_MultiEdgeLeftMerge`
- `scenario_runner.num_actors: 3`
- Each edge has a `locale` block (polygon vertices, parsed in ecav `.py`):

```yaml
scenario:
  edge_list:
    - <<: *edge_base
      manager_type: late_fusion
      predictor_type: linear
      locale:
        id: "locale_0"
        polygon: [[0,134],[160,134],[160,150],[0,150]]
      rsus:
        - <<: *rsu_base
          name: rsu0
          spawn_position: [75.0, 141.0, 7.0]
          id: 0
      vehicles:
        - <<: *vehicle_base
          name: cav1
          destination: [600.0, 139.5, 0.3]

    - <<: *edge_base
      manager_type: late_fusion
      predictor_type: linear
      locale:
        id: "locale_1"
        polygon: [[140,134],[310,134],[310,150],[140,150]]
      rsus:
        - <<: *rsu_base
          name: rsu1
          spawn_position: [230.0, 141.0, 7.0]
          id: 1
      # No vehicles initially
```

### ecav `.py` — `openscenario_multi_edge_left_merge.py`

**New imports:**
```python
import numpy as np
from ecav.core.application.edge.migration import (
    Locale, LocaleRegistry, LocaleRouter,
    VehicleLocaleTracker, SequentialMigrationDaemon, InterLocaleLink,
)
```

**Pre-loop (build locale registry from YAML, set up tracking):**
```python
registry = LocaleRegistry()
for i, edge_cfg in enumerate(scenario_params['scenario']['edge_list']):
    if 'locale' in edge_cfg:
        lc = edge_cfg['locale']
        registry.register(Locale(
            locale_id=lc['id'],
            polygon=np.array(lc['polygon'], dtype=np.float64),
            edge_host_id=edge_list[i].edgeid,
        ))

router = LocaleRouter(registry)
locale_tracker = VehicleLocaleTracker(router, min_dwell_ticks=4)

daemon = SequentialMigrationDaemon()
link = InterLocaleLink(edge_list[0].latency_model)
obstacle_handoff_done = False
transfer_costs = []
npc_carla_id = None   # resolved after scene populates
```

**In the tick loop:**
```python
# Per-tick ego snapshot upload (same as Scenario A)
for edge in edge_list:
    for vm in edge.vehicle_manager_list:
        payload = edge.export_vehicle_state(vm.vehicle.id)
        if payload:
            scenario_manager.store_vehicle_state(vm.vehicle.id, payload)

# Resolve fast NPC carla_id (moving non-hero vehicle)
if npc_carla_id is None and step > 10:
    for v in world.get_actors().filter('vehicle.*'):
        if v.attributes.get('role_name') != 'hero':
            vel = v.get_velocity()
            if (vel.x**2 + vel.y**2)**0.5 > 2.0:
                npc_carla_id = v.id
                logger.info("[INIT] Fast NPC carla_id=%d", npc_carla_id)
                break

# Obstacle-triggered locale handoff
if npc_carla_id is not None and not obstacle_handoff_done:
    npc_actor = world.get_actor(npc_carla_id)
    if npc_actor:
        loc = npc_actor.get_location()
        sim_t = step * world_dt
        event = locale_tracker.update(npc_carla_id, (loc.x, loc.y), step, sim_t)
        if event and event.destination_locale_id == 'locale_1':
            cost = daemon.transfer_obstacle_state(
                npc_carla_id, edge_list[0], edge_list[1], link, step
            )
            if cost:
                transfer_costs.append(cost)
                obstacle_handoff_done = True
```

### Verification (Scenario B)

`python ecav.py -t openscenario_multi_edge_left_merge`

1. NPC carla_id resolved by tick 15.
2. `locale_tracker.update()` fires `HandoffEvent` for NPC around tick 126 ± 10 (as NPC crosses x=140 with 4-tick dwell).
3. `edge_list[1].tracker.trackers` contains an entry with `carla_id == npc_carla_id` immediately after handoff (KF injected).
4. That injected track has `hits >= tracker.min_hits` (warm start, no confirmation dwell).
5. `TransferCost` emitted; `payload_bytes > 0`, `total_ms > 0`.
6. No collision event between ego and fast NPC.
7. Log line: tick at which locale 1's RSU first publishes a fresh detection of the NPC (via the YOLO pipeline) should be ≥ tick 153 — confirming the handoff gave locale 1 advance awareness.

---

## Build Sequence

| Step | Work | Files |
|---|---|---|
| 1 | Extension: obstacle export/import + daemon method | `edge_manager_base.py`, `edge_manager_pluggable_base.py`, `daemon.py` |
| 2 | Scenario A YAML | `openscenario_3_multi_edge_late_fusion.yaml` |
| 3 | Scenario A ecav .py | `openscenario_3_multi_edge_late_fusion.py` |
| 4 | **Validate Scenario A** (all 5 criteria pass) | — |
| 5 | Scenario B XML | `scenario_multi_edge_left_merge.xml` |
| 6 | Scenario B scenario runner .py | `scenario_multi_edge_left_merge.py` |
| 7 | Scenario B YAML | `openscenario_multi_edge_left_merge.yaml` |
| 8 | Scenario B ecav .py | `openscenario_multi_edge_left_merge.py` |
| 9 | **Validate Scenario B** (all 7 criteria pass) | — |

Steps 1–4 ship together (one PR or one commit block). Steps 5–9 ship after Step 4 is green.

---

## Key Reuse

- `SequentialMigrationDaemon` — `ecav/core/application/edge/migration/daemon.py`
- `InterLocaleLink`, `TransferCost` — `ecav/core/application/edge/migration/link.py`
- `HandoffManager` — `ecav/core/application/edge/migration/binding.py` (Scenario A tick trigger)
- `VehicleLocaleTracker`, `LocaleRouter`, `LocaleRegistry` — same module (Scenario B geometry)
- `Locale` — `ecav/core/application/edge/migration/locale.py`
- `export_vehicle_state`, `relinquish`, `accept` — `ecav/core/application/edge/edge_manager/edge_manager_base.py`
- `scenario_manager.store_vehicle_state` — `ecav/scenario_testing/utils/sim_api.py:754`
- Template ecav `.py` — `ecav/scenario_testing/openscenario_3_edge_late_fusion.py`
- Template scenario runner — `ecav/scenario_testing/scenarios/scenario_3.py` (Scenario A, unchanged)

---

## Open Items (resolve during implementation)

- **HANDOFF_TICK=60 (Scenario A):** based on 43 km/h, dt=0.05s. If ego accelerates slowly
  from rest, tick 60 may still be in locale 0. Add a log of ego Y-position at tick 55-60 to
  confirm. Adjust if needed.
- **NPC detection before locale crossing (Scenario B):** locale 0's RSU (x=75, range 120m)
  should see the NPC from tick 1 (NPC at x=0, within range). Confirm via `tracker.trackers`
  count > 0 on tick 15.
- **Locale tracker `min_dwell_ticks=4`:** at 22.2 m/s and 4 ticks (0.2s), the NPC travels 4.4m
  inside locale 1 before the event fires. This is intentional hysteresis; adjust if handoff
  fires too late.
- **Emergency vehicle in Town06:** `vehicle.ford.ambulance` is the CARLA ambulance model.
  If not in the blueprint library, fall back to `vehicle.dodge.charger_2020`.
- **`vid` resolution guard (Scenario A):** `vehicle_manager_list` may be empty for 1–2 ticks
  after scenario start. The `if not handoff_done and vid is None and edge_list[0]...` guard
  handles this.
