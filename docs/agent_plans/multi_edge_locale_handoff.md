# Multi-Edge Architecture: Locale Ownership & Vehicle Handoff

**Branch:** `distributed-integration`  
**Status:** Architecture plan — no implementation yet  
**Created:** 2026-04-18  
**Audience:** jrapp + Tyler (PhD research, Georgia Tech)

---

## Context

The current edge architecture supports a single edge that owns a static, pre-declared set of vehicles and RSUs (defined in YAML at scenario start, immutable at runtime). This works for a single-intersection scenario but cannot model multi-intersection scenarios — the primary scaling challenge for Tyler's research.

This plan covers two interrelated problems:

1. **Locale architecture** — how edges claim geographic ownership of CARLA space, and what topologies to support (1:1, many:1, 1:many)
2. **Vehicle handoff** — how vehicles transfer from one edge to another as they cross locale boundaries, and who initiates/mediates that transfer

These are not merely implementation questions. The choice of handoff model directly determines the research question that can be asked, the metrics that can be measured, and how results translate to real-world V2X deployment.

---

## Existing Infrastructure (What We Can Build On)

- **VehicleUpdate already carries position**: `transform.location.{x,y,z}` sent every tick from each actor — but in edge mode this goes to the *edge*, not the orchestrator.
- **Orchestrator has direct CARLA access**: `ecav.py` holds the CARLA world reference and can query all actor ground-truth positions directly from CARLA each tick — critical enabling fact for Model C.
- **`vehicleToEdgeMapping_` (C++ server)**: Maps vehicle_index → edge_index. Set once via `Server_SetEdgeMappings` before scenario start. **No runtime update path exists today.**
- **`Client_GetConnectionInfo`**: Called once per actor at startup. Actor pins to its edge channel for the entire scenario. No re-query mechanism.
- **Edge process tracks per-actor state**: `self.actors` dict with `EdgeActorInfo` (last_update, push_stub, etc.) — this state would need to be serialized and transferred during handoff.
- **`fuse_predictions()` is still a placeholder**: passthrough only. Whatever real fusion logic goes here becomes the "state" that must transfer on handoff.

---

## Part 1: Locale Architecture

### v1: 1-Edge-1-Locale (Rectangular Grid Partition)

**Definition:** A locale is an axis-aligned rectangular bounding box in CARLA world XYZ space. The full map is partitioned into non-overlapping, gap-free tiles. Each edge owns exactly one tile.

**YAML representation (new field `locale_bounds`):**

```yaml
scenario:
  edge_list:
    - <<: *edge_base
      locale_bounds:
        min: [-200.0, 0.0, -5.0]    # [x, y, z]
        max: [0.0, 300.0, 50.0]     # [x, y, z]
      transition_zone_width: 10.0   # meters from locale boundary; research variable
      manager_type: bm2cp
      rsus:
        - <<: *rsu_base
          name: rsu1
          spawn_position: [-88.0, 140.0, 5.0]
```

**Containment check (2D — z ignored for ground-level vehicles):**
```python
def in_locale(vehicle_transform, locale_bounds) -> bool:
    x = vehicle_transform.location.x
    y = vehicle_transform.location.y
    return (locale_bounds.min[0] <= x <= locale_bounds.max[0] and
            locale_bounds.min[1] <= y <= locale_bounds.max[1])
```

**Assignment at scenario start:** `sim_api.py` assigns vehicles to edges based on spawn position (geometric lookup). `locale_bounds` replaces the explicit vehicle list as ownership source of truth. Initial `vehicleToEdgeMapping_` is computed geometrically.

**Decision D-L1:** Locale bounds are defined in YAML per edge. Spawn-time assignment is geometric. Explicit vehicle lists under edge_list are retained as an override for fixed scenarios but deprecated as the primary ownership mechanism.

**Assumption A-L1:** No vehicle spawns outside all locale bounds. Fatal config error if violated.

**Assumption A-L2:** Locales are non-overlapping and gap-free in v1. Enforced at scenario startup.

---

### v2a (Forward-Looking): Multiple Edges Per Locale

**Motivation:** Load balancing, redundancy, research into distributed consensus at a single locale.

**Topology:** Two or more edge processes share the same locale bounds. Vehicles distributed among them (round-robin, load-based, or capability-based).

**Key challenges:**
- **Consistency**: Two edges produce independent tracking states for the same vehicles. Which is ground truth?
- **Assignment protocol**: Who decides primary edge per vehicle? Options: orchestrator assigns at entry, edges elect via consensus, vehicle selects lowest-latency edge.
- **Research value**: Does multi-edge coverage at a single locale improve or degrade perception quality?

**Forward-looking design note:** `EdgeMappingSetup` currently has flat vehicle→edge mapping. v2a requires vehicle→[edge_list] with primary + secondaries. `ActorConnectionInfo` would return primary + optional secondary list.

---

### v2b (Forward-Looking): Multiple Locales Per Edge

**Motivation:** A single powerful edge node manages several nearby intersections. Relevant for sparse deployments.

**Topology:** One edge process owns several locale bounds. Its `locale_bounds` becomes `locale_list: [{...}, {...}]`.

**Key challenges:**
- **Fusion scope**: Does the edge fuse across all its locales, or independently per locale? Cross-locale fusion likely not physically meaningful for distant intersections.
- **Handoff within edge**: Vehicle moving from locale A to locale B (same edge) requires no edge handoff — but requires updating which RSU camera the edge uses for that vehicle.

---

### Non-Grid Locale Types (Forward-Looking)

**Overlapping locales (cell-network style):**
- Vehicle can be simultaneously "in" two locales → soft handoff (dual-reporting transition period)
- Containment check returns a list, not a bool
- Research variable: how does overlap radius affect handoff quality vs. bandwidth?

**Non-contiguous / gap locales:**
- Vehicle exits all locales → enters "unassisted" state, falls back to local perception
- Research value: the gap is the most safety-critical region. How long? What safety degradation?
- `in_locale()` returns None → vehicle uses local perception only

---

## Part 2: Handoff Architecture

The choice of handoff model determines the research question. All three should ultimately be implemented and compared — **the comparison is the contribution**.

---

### Model A: Vehicle-Driven Handoff

**Who decides:** Vehicle monitors its own position against the locale map.

**Mechanism:**
```
Tick N:   Vehicle detects it has entered transition zone
Tick N+k: Vehicle queries orchestrator for new edge info (mid-scenario Client_GetConnectionInfo)
          Vehicle pre-registers with new edge; new edge begins receiving duplicate updates
Tick N+k+T: Vehicle confirms handoff complete to old edge
            Old edge removes vehicle, optionally transfers state to new edge
            New edge takes over as primary
```

**Infrastructure needed:** locale_map at vehicle startup; mid-scenario Client_GetConnectionInfo; vehicle HANDOFF state machine; dynamic edge actor counts; Edge_ActorDeparting RPC.

**Pros:** Distributed, no central bottleneck; vehicle has ground-truth position; most faithful to real V2X deployment (cellular handover pattern).

**Cons:** Vehicle-side state machine complexity; split-brain risk; edge actor count becomes dynamic.

**Real-world fidelity:** HIGH

---

### Model B: Edge-Driven Handoff

**Who decides:** Current edge monitors vehicle positions from VehicleUpdate messages.

**Mechanism:**
```
Tick N:   Edge detects vehicle V in transition zone (near locale boundary)
          Edge identifies next edge from locale topology
          Edge contacts next edge directly: "Vehicle V inbound, here is its tracking state"
Tick N+k: Next edge confirms readiness; old edge sends HANDOFF command to vehicle
Tick N+k+1: Vehicle switches connection; new edge notifies orchestrator
```

**Infrastructure needed:** Edge-to-edge gRPC channels (new); locale topology at edge registration; Edge_PeerHandoff RPC; dynamic edge actor counts; Server_UpdateEdgeMappings (runtime).

**Pros:** Edge already has vehicle positions; warmest possible handoff (state pre-loaded); vehicle-side changes minimal (just respond to HANDOFF command).

**Cons:** Requires edge-to-edge communication (new infrastructure); edge crash = no handoff initiator; edges must know about neighbors.

**Real-world fidelity:** MEDIUM (matches ETSI ITS RSU-to-RSU coordination)

---

### Model C: Orchestrator-Driven Handoff

**Who decides:** CARLA orchestrator (`ecav.py`) via direct CARLA position queries — ground truth, no network delay.

**Mechanism:**
```
Every tick: ecav.py queries CARLA for all actor positions (direct API call)
            Containment check against locale bounds
            On boundary crossing:
              → Server_UpdateEdgeMapping (delta RPC, new)
              → C++ relays HANDOFF_PENDING to old edge, Edge_ActorInbound to new edge
              → Edges coordinate; C++ sends HANDOFF_COMPLETE
              → Vehicle receives SWITCH_EDGE command in next push tick
```

**Key insight:** `ecav.py` already holds CARLA world reference. Position query is an O(1) CARLA API call. Detection is free — only coordination requires new infrastructure.

**Infrastructure needed:** Server_UpdateEdgeMapping (delta RPC); HANDOFF_PENDING/COMPLETE commands; Edge_ActorInbound/Departing RPCs; dynamic edge actor counts; SWITCH_EDGE command to vehicle.

**Pros:** Orchestrator has CARLA ground truth — no position forwarding needed; centralized coordination → no split-brain; `vehicleToEdgeMapping_` already canonical source of truth; cleanest experimental baseline.

**Cons:** Adds to orchestrator per-tick work; central bottleneck for simultaneous handoffs; no real-world equivalent to "omniscient orchestrator."

**Real-world fidelity:** LOW (but best for controlled research experiments)

---

### Comparison Table

| Dimension | Model A (Vehicle) | Model B (Edge) | Model C (Orchestrator) |
|-----------|-----------------|----------------|----------------------|
| **Initiator** | Vehicle | Current edge | CARLA orchestrator |
| **Detection source** | Vehicle self-position | VehicleUpdate.transform | CARLA direct query |
| **New edge-to-edge channel** | No | Yes | No |
| **Locale map at vehicle** | Yes | No | No |
| **State transfer path** | Edge→Edge (or cold) | Edge→Edge (warm) | Via orchestrator relay |
| **Tick barrier impact** | High (dynamic actor count) | High | Medium (serialized) |
| **Real-world fidelity** | High | Medium | Low |
| **Implementation complexity** | High | Very high | Medium |
| **Research value** | V2X deployment model | RSU-to-RSU coordination | Clean experimental baseline |

---

### Recommended Implementation Order

1. **Model C first** — lowest infrastructure cost; cleanest baseline; CARLA position access already available
2. **Model A second** — most deployment-realistic; self-contained in `ecloud_actor_client.py`; reuses Model C infrastructure
3. **Model B third** — most complex (peer channels); warmest handoff; best for ETSI ITS comparison

---

## Part 3: State Transfer

**Cold Start (v1):** New edge starts fresh. Vehicle receives no edge-fused prediction for 2-5 ticks. This IS the research signal — it quantifies the "handoff gap."

**Warm Handoff (Phase 2):** Old edge serializes Kalman filter state, track IDs, prediction history. New edge deserializes before vehicle arrives. Requires edge state to be serializable (AB3DMOT state is structured but non-trivial).

**Decision D-H1:** Cold start is v1. Warm vs. cold comparison is Paper 2's core result.

---

## Part 4: Required Infrastructure

### New Proto Messages/RPCs (minimum for Model C)

```protobuf
message EdgeMappingDelta {
  int32 vehicle_index = 1;
  int32 old_edge_index = 2;
  int32 new_edge_index = 3;
  bytes transferred_state = 4;  // Optional: warm handoff
}
rpc Server_UpdateEdgeMapping(EdgeMappingDelta) returns (Empty);

message EdgeActorDeparting { int32 vehicle_index = 1; int32 destination_edge = 2; }
rpc Edge_ActorDeparting(EdgeActorDeparting) returns (Empty);

message EdgeActorInbound { int32 vehicle_index = 1; bytes state = 2; }
rpc Edge_ActorInbound(EdgeActorInbound) returns (Empty);

// New Command enum value: HANDOFF (vehicle switches edge)
```

### Files to Modify (Model C)

| File | Change |
|------|--------|
| `ecav/protos/ecloud.proto` | Add EdgeMappingDelta, new RPCs, HANDOFF command |
| `ecav/ecloud_server/ecloud_server.cc` | Server_UpdateEdgeMapping (thread-safe delta); relay Departing/Inbound |
| `ecav/ecav2/edge_process.py` | Dynamic expected_num_actors; Inbound/Departing handlers |
| `ecav/ecav2/ecloud_actor_client.py` | Handle HANDOFF command in tick loop |
| `ecav/scenario_testing/utils/sim_api.py` | Load locale_bounds; call Server_UpdateEdgeMapping on crossing |
| `ecav.py` | Per-tick CARLA position query + containment check |

---

## Part 5: Evaluation Framework

### Metrics

| Metric | Capture Point |
|--------|--------------|
| Handoff latency (ticks: crossing → first new-edge update) | Orchestrator |
| Perception gap (ticks with no edge_predictions) | Vehicle |
| Tracking error at handoff (first fused output vs. CARLA ground truth) | New edge |
| Safety events during handoff (near-miss, collision) | safety_manager.py; correlate with handoff events |
| Tick cycle time inflation vs. steady state | Orchestrator tick duration log |
| State transfer overhead (warm handoff: bytes + ms) | Edge_ActorDeparting path |

### Experimental Scenarios

| Scenario | Purpose |
|----------|---------|
| `openscenario_3_multi_edge` | Two locales, one vehicle, single controlled handoff — baseline |
| `openscenario_3_multi_concurrent` | N vehicles crossing simultaneously — tick barrier stress test |
| `openscenario_3_multi_oscillate` | Vehicle oscillates near boundary — debounce/hysteresis edge case |
| `openscenario_3_multi_latency` | Handoff under C-V2X / 5G latency model — worst-case stacking |
| `openscenario_3_multi_gap` | Non-adjacent locales with gap — unassisted mode characterization |

### Research Questions → Tyler's Papers

**Paper 2 (multi-edge handoff):**
- Q1: What is the handoff gap in ticks? How does it compare to base latency budget (130-200ms)?
- Q2: Cold start vs. warm handoff: does state transfer eliminate the gap?
- Q3: Model A vs. B vs. C: which detection model minimizes safety impact?
- Q4: Handoff latency × network latency: does the stack add linearly?
- Key result: "handoff safety envelope" — (speed, gap_width, latency) parameter space for safe vs. dangerous handoff

**Paper 3 (scaling):**
- Q5: Does N-edge tick time scale sub-linearly?
- Q6: At what vehicle density does simultaneous handoff degrade throughput?
- Q7: v2a (multiple edges per locale): does redundant coverage help or hurt?
- Q8: Operational envelope for city-block grid (4+ intersections, 8+ vehicles)?

---

## Part 6: Explicit Decisions & Assumptions

### v1 Assumptions

| ID | Assumption | Future direction |
|----|-----------|-----------------|
| A-L1 | All vehicles spawn within a locale | Support unassisted spawn; nearest locale on-demand |
| A-L2 | Locales are non-overlapping, gap-free | Overlapping (cell-net): soft handoff; gap: unassisted mode |
| A-L3 | Locale bounds are rectangular, axis-aligned | Polygon, waypoint-set, radius-from-RSU |
| A-L4 | One RSU per locale | Multiple RSUs per locale already supported in YAML |
| A-H1 | Cold start on handoff (no state transfer) | Warm handoff is the Paper 2 research variable |
| A-H2 | Vehicle route monotonic through locale sequence | Debounce/hysteresis guard for oscillation |
| A-H3 | Handoff atomic from orchestrator perspective | Grey-period dual-edge counting is a future option |

### Design Decisions

| ID | Decision | Rationale |
|----|---------|-----------|
| D-L1 | Locale bounds in YAML; spawn assignment geometric | Explicit vehicle lists too static for research |
| D-L2 | z-axis ignored in containment check | All vehicles are ground-level |
| D-H1 | Model C implemented first | Lowest infrastructure cost; best experimental baseline |
| D-H2 | Cold start is v1 | Need a baseline to compare warm handoff against |
| D-H3 | transition_zone_width is a research variable (configurable) | Narrow → less pre-registration time; wide → more bandwidth overhead |
| D-H4 | Server_UpdateEdgeMapping is a delta RPC, not full reset | Full reset disrupts non-handoff vehicles |

---

## Implementation Checklist

### Phase 0: Locale Definition
- [ ] Add `locale_bounds` field to edge YAML schema (min/max XYZ)
- [ ] Add `transition_zone_width` to YAML (default: 10m)
- [ ] Update `compute_edge_mappings()` in `sim_api.py` to geometric assignment
- [ ] Validate locale coverage at startup (no gaps, no overlaps, all spawns contained)
- [ ] Build `openscenario_3_multi_edge.yaml` (two locales, linear corridor)

### Phase 1: Model C Handoff (Orchestrator-Driven)
- [ ] `ecav.py`: per-tick containment check (CARLA position query)
- [ ] `ecloud.proto`: add EdgeMappingDelta, Server_UpdateEdgeMapping, Edge_ActorDeparting, Edge_ActorInbound, HANDOFF command
- [ ] Regenerate proto stubs; rebuild C++ server
- [ ] `ecloud_server.cc`: Server_UpdateEdgeMapping (thread-safe delta); relay Departing/Inbound to edges
- [ ] `edge_process.py`: dynamic expected_num_actors; Inbound/Departing handlers
- [ ] `ecloud_actor_client.py`: handle HANDOFF command in tick loop
- [ ] Instrument: handoff latency, perception gap
- [ ] Run openscenario_3_multi_edge end-to-end; verify handoff completes

### Phase 2: Warm Handoff (State Transfer)
- [ ] Define serialization format for EdgeManager + AB3DMOT state
- [ ] Old edge serializes on Edge_ActorDeparting
- [ ] New edge deserializes on Edge_ActorInbound
- [ ] Instrument: tracking error (warm vs. cold)
- [ ] Run comparative experiment

### Phase 3: Model A (Vehicle-Driven)
- [ ] Distribute locale map to vehicles at registration (EdgeScenarioConfig.locale_map)
- [ ] `ecloud_actor_client.py`: locale-awareness state machine + mid-scenario connection re-query
- [ ] Edge: handle mid-scenario actor arrival
- [ ] Run comparative experiment: Model A vs. Model C

### Phase 4: Model B (Edge-Driven)
- [ ] Design edge-to-edge channel infrastructure
- [ ] Distribute locale topology to edges at registration
- [ ] Edge_PeerHandoff RPC
- [ ] Run comparative experiment: Model B vs. Models A and C

### Phase 5: v2a/v2b Scenarios
- [ ] Multiple edges per locale: primary/secondary assignment
- [ ] Multiple locales per edge: per-locale fusion scope
- [ ] Overlapping locales: soft handoff (dual-reporting transition)
