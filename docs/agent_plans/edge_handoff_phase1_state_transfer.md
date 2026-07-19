# Edge Hand-Off — Phase 1: State Transfer (Sequential / Single-Process)

**Branch:** `develop`
**Status:** Steps 0–6 complete (Step 6 live gate closed); Step 7 ego path VALIDATED live (runs 5/7/8/9); obstacle handoff blocked on a track-birth bug — see Step 7 open items
**Created:** 2026-06-08
**Updated:** 2026-07-19
**Audience:** jrapp + Tyler (PhD research, Georgia Tech)
**Source of truth:** [tyler_edge_handoff_architecture.md](../kb/raw/notes/tyler_edge_handoff_architecture.md) (2026-06-07 meeting)

---

## Context

We are building edge-to-edge vehicle hand-off in stages: **sequential (this plan) →
edge-only `-eo` (RPC) → fully distributed (C++ buffer)**. This plan covers only the
sequential, single-process case.

Tyler's architecture adopts the **hybrid** model: a central **state store** that
edges write to and pull from, with edge-to-edge transfer reduced to a thin
**ownership ping/ack**. The store lives in `sim_api.py` (sequential), the C++
server (distributed). Edges retrieve state from the store; the peer message only
transfers *ownership*, not bytes.

### The governing principle — separate mechanism from measurement

> "The actual cost within the simulation to serialize-transfer-deserialize the data
> must not be conflated with the actual simulated measurement of that same process."

- **Mechanism (simulator):** in a single process, transfer is *instant* — a dict
  read + a member-function call. Memory is shared; nothing is actually sent.
- **Measurement (simulation/research):** we *model* serialization cost (from payload
  byte size) and network latency (from the **simulated** edge locations / latency
  model) and record them as metrics — entirely decoupled from the instant mechanism.

This separation is the architectural spine of the whole feature and must be built in
from line one. State storage/transfer is an artifact of *simulator* design; the
modeled cost is the *simulated* research quantity.

---

## Hypothesis

We can implement sequential edge-to-edge vehicle-state hand-off such that:
1. **(mechanism)** the destination edge gains the source edge's per-vehicle state via
   an instant store-pull + ownership ping/ack — no real serialization required;
2. **(measurement)** the simulated serialization + network cost of that transfer is
   modeled from payload size and simulated edge geometry, and recorded as a metric,
   independent of (1);
3. **(composition)** this completes the `link` + `daemon` pieces Tyler left
   "forthcoming" in `ecav/core/application/edge/migration/`, reusing his
   `MigrationPayload`/`TrackLatent` as the snapshot unit.

**Counter-hypothesis / risks to watch:** (a) the per-vehicle "state" for AB3DMOT +
linear is entangled in shared edge structures (one `tracker`, one `track_to_carla`),
so a clean per-vehicle export/import requires careful AB3DMOT injection — **Phase 1
targets correctness from the start** (see Step 0). The key insight: `track_to_carla`
is `tid → cid`; each `KF.carla_id` also holds `cid` directly. Export finds the KF by
`carla_id`; import injects a new KF with the source's `x`, `P`, and `hits >= min_hits`
so the track appears immediately in tracker output without a confirmation dwell. The
destination assigns a fresh `tid` from its own `ID_count` counter; `carla_id` is the
stable cross-edge key. (b) Measuring host pickle time instead of modeling it would
conflate mechanism with measurement — the plan explicitly models cost from byte size,
never from `perf_counter` around the real pickle.

### How we verify
- **Smoke test (no CARLA):** construct the store + two stub edges + a snapshot,
  trigger a hand-off, assert store/retrieve, ownership moved, cost recorded. Mirrors
  `migration/smoke_test.py`. This is the fast iteration loop.
- **Integration (2-edge sequential run):** a scripted hand-off (at tick N, move
  `vehicle_0` from `edge_0` → `edge_1`) inside a running sequential scenario; assert
  the destination edge holds the state and a hand-off cost record is emitted.

### How we evaluate
Not a perf result — a *mechanism + instrumentation* result. Success = the transfer
happens instantly, the destination edge has the state, and a per-hand-off cost record
(`bytes`, `sim_serialize_ms`, `sim_network_ms`, `total_ms`) is produced and is
sensitive to the configured edge geometry / latency model.

---

## Scope

### In scope (Phase 1)
- Central **state store** in `sim_api.ScenarioManager` (dict; full snapshot per
  vehicle per tick; instant retrieval).
- Per-vehicle **snapshot export/import** on the edge manager, using
  `migration.MigrationPayload` as the unit.
- **Ownership transfer** as a member-function ping/ack between two edge instances.
- **Cost model** (`migration/link.py`): serialization + network cost, modeled and
  recorded, decoupled from the instant mechanism; reuses `LatencyModel`.
- **Hand-off coordinator** (`migration/daemon.py`, sequential variant): wires
  trigger → ownership ping/ack → store-pull → cost record.
- **`HandoffManager`** in `migration/binding.py` (alongside `HandoffEvent`): owns the handoff decision and event emission. Call site: `event = handoff_manager.evaluate(vid, tick, sim_time_s, ...); if event: daemon.request_handoff(event)`. Phase 1 trigger is tick-based; `source_locale_id`, `destination_locale_id`, `position` kwargs are wired for Phase 2 without a signature change.
- **Smoke + integration tests** to verify plumbing and cost recording.

### Explicitly OUT (later phases)
- Locale-based trigger (`VehicleLocaleTracker`/`HandoffEvent`) → **Phase 2**.
- `-eo` RPC transfer + state array in the do-tick message → **Phase 2**.
- C++ server state buffer → **Phase 3**.
- Warm-handoff *correctness* (destination resumes tracking with no quality dip) →
  **follow-on (Phase 1.5)**; Phase 1 proves plumbing + cost, not resume fidelity.
- Partial-snapshot / cadence optimization — v1 is **full snapshot every tick**.

---

## Architecture

```
                    ┌─────────────────────────────────────────────┐
                    │ ScenarioManager (sim_api.py)                 │
   each tick ──────▶│   _vehicle_state_store: {vehicle_id: bytes}  │◀──── pull (instant)
   store full       │   store_vehicle_state() / retrieve_*()       │
   snapshot         └─────────────────────────────────────────────┘
        │                         ▲                       │
        │ export_vehicle_state()  │                       │ retrieve_vehicle_state()
        ▼                         │                       ▼
   ┌─────────┐   ownership ping   │                  ┌─────────┐
   │ edge_0  │ ───────────────────┼─────────────────▶│ edge_1  │
   │ (src)   │ ◀───── ack ────────┼──────────────────│ (dst)   │  import_vehicle_state()
   └─────────┘                    │                  └─────────┘
        │                         │
        └──── MigrationDaemon ────┘   records TransferCost (sim serialize+network ms)
              (migration/daemon.py)   via InterLocaleLink (migration/link.py)
```

- **Mechanism path (instant):** `edge_1` pulls `vehicle_0`'s snapshot from the store
  and `import_vehicle_state`; `edge_0` drops it, `edge_1` adds it (ownership =
  membership in `vehicle_manager_list`). The peer message is a bare ping/ack.
- **Measurement path (decoupled):** `InterLocaleLink.model_transfer(payload,
  src_edge, dst_edge)` returns a `TransferCost` from `payload.payload_bytes()` +
  simulated geometry/latency; the daemon records it. Never gates the instant path.

---

## File-by-file changes

| File | Change | Status |
|---|---|---|
| `ecav/core/application/edge/migration/payload.py` | Added `KFState` dataclass (`state_vector`, `covariance`, `hits`, `anchoring_age`); `TrackLatent.kf_state: Optional[KFState]` — separate slot, `hidden_state` reserved for recurrent/neural trackers; `nbytes()` updated. | **Done** |
| `ecav/core/application/edge/migration/binding.py` | Added `HandoffManager` alongside `HandoffEvent`; owns `_emit` + subscriber list; `evaluate(vid, tick, sim_time_s, *, source_locale_id, destination_locale_id, position) -> Optional[HandoffEvent]`. | **Done** |
| `ecav/core/application/edge/migration/smoke_test.py` | Integrated `HandoffManager` into all three scenarios; `evaluate()` called each tick (once per vehicle); manager events collected via `subscribe`; assertions verify manager and locale tracker fire on the same tick. | **Done** |
| `ecav/scenario_testing/utils/sim_api.py` | Added `_vehicle_state_store: Dict[int, MigrationPayload]` to `ScenarioManager`; `store_vehicle_state(vid, payload)`, `retrieve_vehicle_state(vid)`, `retrieve_all_vehicle_states()`. Stores objects directly (no real pickle — avoids mechanism/measurement conflation); API shaped to mirror future gRPC `Store/Retrieve` for `-eo`/C++ upgrade. | **Done** |
| `ecav/core/application/edge/edge_manager/edge_manager_base.py` | Add `export_vehicle_state(vehicle_id) -> MigrationPayload` / `import_vehicle_state(vehicle_id, payload)`; `relinquish(vehicle_id)` / `accept(vm)` for ownership move; obstacle no-op stubs. Base impl exports coarse slice; pluggable subclass overrides for AB3DMOT. | **Done** |
| `ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py` | AB3DMOT-aware export: find `KF` by `carla_id` in `self.tracker.trackers`; extract `kf.x`, `kf.P`, `hits`, `anchoring_age` into `KFState`; import injects new `KF` with source state + `hits >= min_hits`, advances `ID_count`, updates `track_to_carla`. Plus `export/import_tracked_obstacle_state`. | **Done** |
| `ecav/core/application/edge/migration/link.py` *(new)* | `InterLocaleLink` + `TransferCost`. `model_transfer(payload, src, dst, tick) -> TransferCost`: serialize cost = `payload_bytes() × rate`; network cost from `LatencyModel`. Pure model — no real I/O. | **Done** |
| `ecav/core/application/edge/migration/daemon.py` *(new)* | `SequentialMigrationDaemon.request_handoff(vid, src_edge, dst_edge, store, link, tick)`: `src.relinquish(vid)` → `dst.accept(vm)` → store-pull import → `InterLocaleLink.model_transfer()` → record `TransferCost`. Plus `transfer_obstacle_state` (no ownership move). | **Done** |
| `ecav/scenario_testing/…/openscenario_3_multi_edge_late_fusion.py` *(new)* + YAML | 2-edge sequential scenario (Scenario A); per-tick `store_vehicle_state`; scripted hand-off at `HANDOFF_TICK` via `daemon.request_handoff`; `record_handoff_cost`. | **Done** |
| `tests/test_edge_state_handoff.py` *(new)* | No-CARLA smoke test: store round-trip; export→store→retrieve→import; ownership moved; `TransferCost` recorded and sensitive to rate; metric-summary shape + JSON round-trip. 8/8 pass. | **Done** |
| Metrics (`sim_api.ScenarioManager` + `EvaluationManager`) | `record_handoff_cost`/`get_handoff_costs` sink on the manager; `summarize_handoff_costs` (pure); `handoff_eval` drains into `global_metrics` + `evaluation_report.txt` `HAND-OFF MIGRATION` section. | **Done** |

### Per-tick snapshot write (where)
The sequential loop lives in the scenario `.py` (`while flag: tick_world(); tick();
edge.run_step(step)`). After the edges step each tick, call
`scenario_manager.store_vehicle_state(vid, edge.export_vehicle_state(vid))` for every
owned vehicle. (Tyler v1: full snapshot, every vehicle, every tick.) Keep the call
site in the loop so the store reflects post-`run_step` state.

---

## Cost-model design (the crucial separation)

`InterLocaleLink.model_transfer` returns:

```
TransferCost(
    vehicle_id, src_edge_id, dst_edge_id, tick,
    payload_bytes,            # from MigrationPayload.payload_bytes()
    sim_serialize_ms,         # payload_bytes * serialize_rate_ms_per_byte  (config)
    sim_network_ms,           # LatencyModel sample, f(simulated edge geometry)
    total_ms = serialize + network + deserialize,
)
```

- **Serialization:** modeled from byte size × a configured rate, **not** measured from
  the host's real pickle time (that would be simulator-host-dependent, the conflation
  Tyler warns against). `payload_bytes()` already exists for exactly this.
- **Network:** reuse `create_latency_model(edge_cfg, dt)` (already on every edge as
  `self.latency_model`); for edge↔edge, sample a latency parameterized by the
  **simulated** locations of the two edges. v1 may use a per-link distribution / the
  C-V2X trace; later the ns-3 LUT (`ns3_lut_sampler.py`) gives high fidelity.
- **Decoupling invariant:** the daemon records `TransferCost` but the actual
  destination-import happens instantly regardless. The cost never blocks or delays the
  mechanism in Phase 1 (whether modeled latency later *gates* application of state is a
  Phase 2 research question, not a Phase 1 mechanism concern).

---

## Implementation checklist

### Step 0 — Snapshot contract
- [x] Add `KFState` dataclass to `payload.py` (`state_vector`, `covariance`, `hits`, `anchoring_age`); `TrackLatent.kf_state: Optional[KFState]` — separate slot, not overloading `hidden_state`.
- [x] Add `HandoffManager` to `binding.py`; `evaluate()` returns `Optional[HandoffEvent]`; owns `_emit` + subscribers.
- [x] Integrate `HandoffManager` into `migration/smoke_test.py`; all three scenarios pass.
- [x] Define `export_vehicle_state(vid) -> MigrationPayload` / `import_vehicle_state(vid, payload)` on `_BaseEdgeManager` (coarse base impl; pluggable subclass overrides for AB3DMOT). Also `relinquish(vid)` / `accept(vm)`.
- [x] AB3DMOT export: find `KF` by `carla_id`; extract `kf.x`, `kf.P`, `hits`, `anchoring_age` into `KFState`. (`_PluggableEdgeBase.export_vehicle_state`)
- [x] AB3DMOT import: create new `KF` with source `x`/`P`; set `hits >= tracker.min_hits`; advance `ID_count`; update `track_to_carla[new_tid] = cid`. (`_PluggableEdgeBase.import_vehicle_state`)

### Step 1 — State store (sim_api)
- [x] Add `_vehicle_state_store: Dict[int, MigrationPayload]` + `store_/retrieve_vehicle_state` + `retrieve_all_vehicle_states` to `ScenarioManager`.
- [x] Stores `MigrationPayload` objects directly (no real pickle in mechanism path). API mirrors future gRPC store/retrieve for forward-compat.

### Step 2 — Cost model (`migration/link.py`)
- [x] `TransferCost` dataclass + `InterLocaleLink.model_transfer(payload, src, dst, tick)`.
- [x] Serialization cost from `payload_bytes()` × configured rate; network cost from `LatencyModel.sample_ms()` (added public wrapper). Added `InterLocaleLink.from_cfg()` factory. `__init__.py` updated.

### Step 3 — Hand-off daemon (`migration/daemon.py`)
- [x] `SequentialMigrationDaemon.request_handoff(vid, src_edge, dst_edge, store, link, tick)`: ping/ack ownership move (member calls) + store-pull import + record `TransferCost`.
- [x] Ownership move: `src.relinquish(vid)` / `dst.accept(vm)`. Fallback: direct export from src when store is empty.

### Step 4 — Smoke test (no CARLA) — fast loop
- [x] `tests/test_edge_state_handoff.py`: store round-trip; export→store→retrieve→import; ownership moved; `TransferCost` recorded and sensitive to rate; fallback export path. All 6 assertions pass.

### Step 5 — Integration (2-edge sequential)
- [x] `openscenario_3_multi_edge_late_fusion.yaml` — two edges in `scenario.edge_list`; RSU 0 at y=95, RSU 1 at y=120; cav1 in edge 0 only.
- [x] `openscenario_3_multi_edge_late_fusion.py` — per-tick `store_vehicle_state` loop; `HandoffManager(trigger_tick=60)`; `daemon.request_handoff` at tick 60; cost logged in finally.
- [x] Obstacle-export extension: `export/import_tracked_obstacle_state` on `_BaseEdgeManager` (no-op stubs) + `_PluggableEdgeBase` (AB3DMOT implementations); `daemon.transfer_obstacle_state` — no ownership move, one-shot KF share. (Required for Scenario B; built alongside Scenario A.)
- [x] Run end-to-end (clean CARLA, 2026-06-13): `[HANDOFF]` tick=60 vid=109 bytes=98 total_ms=93.3; `[TRANSFER_COST]` logged; ghost_brake_events=0; true_positive_gt=4; SystemExit(0). All 5 criteria pass.

### Step 6 — Metrics
- [x] Hand-off cost sink on `ScenarioManager`: `record_handoff_cost(cost)` / `get_handoff_costs()` (mirrors `sim_metrics`; `_handoff_costs: List[TransferCost]`). Scenario loop records each cost; finally-block trace reads the sink (single source of truth).
- [x] `summarize_handoff_costs(costs)` — pure, JSON-serializable aggregation (per-hand-off records + count/totals/mean); unit-tested empty + populated.
- [x] `EvaluationManager.handoff_eval()` drains the sink into `global_metrics['handoffs']` + `['handoff_summary']`, one `lprint` line per hand-off; `HAND-OFF MIGRATION` section added to `evaluation_report.txt`. Wired into `evaluate()` after `edge_eval`.
- [x] Unit validation: `tests/test_edge_state_handoff.py` extended (8/8 pass) — summary shape, JSON round-trip, daemon→cost→summary chain.
- [x] **Live CARLA gate closed (2026-07-19, Scenario B run 9):** `HAND-OFF MIGRATION` block present in the real `evaluation_report.txt` + `handoff_summary`/`handoffs` in `simulation_metrics.json` (1 hand-off, 986 bytes, serialize 0.0099 ms + network 88.03 ms).

### Step 7 — Scenario B (Town06 left-merge / obstacle handoff)

Detailed design: [edge_handoff_scenarios.md](edge_handoff_scenarios.md) (§ Scenario B). The
research scenario: a fast NPC crosses from locale 0 into locale 1 (where ego is forced into a
left merge by a stationary emergency vehicle); locale 1 receives the NPC's KF history *before*
its own RSU can see it. Geometry-based trigger via `VehicleLocaleTracker`; obstacle transfer
(no ownership move) via `daemon.transfer_obstacle_state`.

- [x] Obstacle-export extension (`export/import_tracked_obstacle_state` on base + pluggable; `daemon.transfer_obstacle_state`) — **already built in Step 5** alongside Scenario A; no VehicleManager required, one-shot KF share, no relinquish/accept.
- [x] Scenario B XML — `scenario_multi_edge_left_merge.xml` (Town06; ego + emergency vehicle + fast NPC; underground→visible teleport pattern from `scenario_3.xml`).
- [x] Scenario B runner `.py` — `scenario_multi_edge_left_merge.py` (`BasicScenario` subclass; `CollisionTest` criteria). **Two fixes vs. the sketch in `edge_handoff_scenarios.md`:** (1) real `SyncArrival(actor, reference, target_location, gain)` takes ONE shared target, not two waypoints; (2) `SyncArrival.update()` never returns SUCCESS, so it is wrapped in a `SUCCESS_ON_ONE` Parallel with an `InTriggerDistanceToLocation` that ends the sync phase before `WaypointFollower(22 m/s)`.
- [x] Scenario B YAML — `openscenario_multi_edge_left_merge.yaml` (Town06; `num_actors: 3`; two edges each with a `locale` polygon block; RSU0 at (75,141), RSU1 at (230,141); cav1 in edge 0 only).
- [x] Scenario B ecav `.py` — `openscenario_multi_edge_left_merge.py` (`_build_locale_router` from YAML `locale` blocks; `VehicleLocaleTracker(min_dwell_ticks=4)`; resolve fast-NPC `carla_id` by velocity; on `locale_1` crossing → `daemon.transfer_obstacle_state` → `record_handoff_cost`; geometric advance-warning proxy = first tick NPC enters RSU1's `RSU1_DETECT_RANGE_M` radius vs. hand-off tick).
- [x] Geometry-trigger unit check (no CARLA): with the real YAML polygons the NPC binds `locale_0` at spawn and fires exactly one `locale_0→locale_1` crossing inside the overlap (x=156). Router nearest-centroid tiebreak confirmed at the overlap boundaries.
- [x] **Scenario redesigned during live tuning (2026-07-19; 9 runs).** Renamed `left_merge` → `right_merge` (CARLA's left-handed frame: heading east, +y is the vehicle's RIGHT — every original lane comment was backwards). Final geometry: ego + emergency vehicle in the LEFTMOST eastbound lane -3 (solid line to shoulder — no left escape, no blocker actor needed); NPC in adjacent lane -4 on an explicit `WaypointFollower` plan (plan-less follower stalls; `SyncArrival` steers 0 and drifts out of lane). Hand-off is geometry-driven for EVERY boundary crossing: managed vehicles → `request_handoff`, obstacles → `transfer_obstacle_state` (pending-retry).
- [x] **Ego path VALIDATED live (runs 5/7/8/9):** ego brakes on edge predictions of the blockage (17 m out, post-handoff via edge 1's RSU), waits for the NPC to pass, merges right, continues to destination. Geometry handoff fires at the crossing (tick 63 with final polygons) with the full KF payload (986 bytes) and cost recorded. Robust across seeds; run 6's visible failure was MAX_STEP=500 truncating a slow-approach merge (→ 700).
- [x] **`behavior_agent.py` overtake fix (required for the merge):** plan lookahead floored at 15 m — speed-proportional `next(speed*6)` degenerates to 3–6 m at crawl speed (and `next(0)` raises), wedging ego mid-merge forever.
- [x] **Warm import gated OFF by default** (`handoff_warm_import` attr; Phase 1.5): injecting a snapshot into a tracker that also sees the object risks stale-duplicate ghosts until import-side reconciliation exists. Payloads + costs recorded; destination tracker untouched.
- [ ] **OPEN — obstacle handoff never fires live: the track-birth bug.** Full derivation (2026-07-19, runs 8+9) so next session starts at the trace, not the rederivation:

  **Expected flow.** For the obstacle handoff to fire, edge 0 must hold a live KF
  track of the NPC at the crossing. Pipeline: RSU0 camera → YOLO detect (`[DET]`
  logged here) → jitter buffer → `tracker.track(dets)` → AB3DMOT birth. AB3DMOT's
  contract: any unmatched detection **births a tentative track immediately** — it
  sits in `tracker.trackers` from that instant (`min_hits` only gates *output*,
  not existence) and survives `max_age` update cycles unseen. Detections at ticks
  180–188 ⇒ a track should exist through ~tick 200.

  **Established facts.** (1) Detections exist: 9 `[DET]` entries from RSU0 in the
  NPC's lane at x=110–130, source ticks 180–188 — consistent across runs 8 and 9.
  (2) The tracker is empty: `[SCENB-DBG]` sampled `edge0.tracker.trackers` at
  ticks 190…280 → `n_trk=0` every time — a zero-length list, not "nearest too
  far". (3) The export logic (identity → `track_to_carla` → 15 m position
  fallback) is unit-tested and sound; it has nothing to search. So: detections
  in, no tracks out — something between `[DET]` and a surviving `trackers` entry
  eats them.

  **Hypotheses, ranked.**
  - **(a) Detections never reach `tracker.track()`.** `[DET]` logs inside
    `fusion.detect()`, upstream of the tracker call. Untraced between them:
    confidence gates, NMS, the beacon-fusion merge (the NPC has NO beacon — a
    merge step may drop unbeaconed detections), format conversion. Suspicious
    wrinkle: after ego's handoff (tick 63) **edge 0 has zero vehicles** — if any
    part of the drain/track path gates on `vehicle_manager_list`, RSU detections
    get computed-and-logged but discarded.
  - **(b) Births happen but die faster than the sampling.** If `max_age` counts
    *world ticks* rather than edge frames — or edge 0 calls `track([])` every
    tick instead of every 4th once it has no vehicles — a fragment born at 185 is
    dead by 187–189. `[SCENB-DBG]` sampled every 10th failed tick; a chain of
    1–2-tick fragments could slip entirely between samples. Not excluded.
  - **(c) Birth is gated by a local modification.** This AB3DMOT is modified
    (anchoring, `carla_id`/`guid` plumbing); a gate requiring beacon-associated /
    qualified detections would pass ego's and silently reject the anonymous NPC's.

  **Decisive trace (cheap — do this first).** One log line at the mouth of
  `tracker.track()` in
  `edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py` (~line 446):
  per edge frame log `n_dets_in, n_trackers_before, n_trackers_after`. One run
  discriminates: dets_in=0 at ticks 184–196 → **(a)**, walk detect→track for the
  dropping filter; dets_in>0 and trackers_after>0 but gone next frame → **(b)**,
  fix `max_age` semantics / empty-frame cadence; dets_in>0, trackers_after=0 →
  **(c)**, find the birth gate.

  **Research implication regardless of fix.** By the time a fast target exits
  locale 0, the source edge's knowledge of it has already decayed (RSU0's
  near-field blind donut x≈20–110 means the crossing sits in a marginal
  reacquisition zone). Fixing the birth path makes the *reactive* boundary
  handoff work; the architecturally-robust answer is Tyler's Phase 2
  **predictive trigger** (`Locale.predicted_to_exit_within`): hand off while the
  track is alive and fresh, *before* exit. Also on the table: RSU
  placement/aiming for continuous NPC-lane coverage. Bring both to the Tyler
  sync.
- [ ] Remove TEMP diagnostics (`[EGO-DBG]`, `[SCENB-DBG]` in the scenario loop) once the obstacle path validates.

### Step 8 — Phase 2 `-eo` distribution (PLANNING PLACEHOLDER — not yet planned)

> **Note only — no implementation here.** After Scenario B validates, we need a *separate*
> plan to lift **both** Scenarios A & B into **Phase 2: edge-only (`-eo`) distribution**.
> The lift swaps the in-process store + member-call ping/ack for the registration-server
> do-tick state array + a peer RPC; the cost model (`InterLocaleLink`/`TransferCost`) and the
> snapshot contract (`MigrationPayload`/`KFState`) are unchanged across the boundary. Write that
> plan when we get there — do not fold it into this doc.
>
> **Actor distribution** (`-d`, Docker-containerized vehicle clients) is a **distinct Phase 3**,
> planned separately again. `-eo` (edge distribution) and `-d` (actor distribution) are
> orthogonal axes; Phase 2 covers only the former.

---

## Decisions & open questions (for jrapp + Tyler)

| ID | Question | Decision |
|----|----------|----------|
| D-1 | Snapshot granularity + AB3DMOT correctness? | **Per-vehicle** `MigrationPayload`; correctness required from Phase 1. Export finds `KF` by `carla_id`; import creates new `KF` with source `x`/`P` and `hits >= min_hits`. `carla_id` is the stable cross-edge key; `tid` is ephemeral and reassigned at destination. **Resolved.** |
| D-2 | KF state vs neural latent in `TrackLatent`? | `KFState` is its own dataclass (`state_vector`, `covariance`, `hits`, `anchoring_age`) stored in `TrackLatent.kf_state`. `hidden_state` reserved for sequence-model trackers (Mamba etc.). **Resolved; implemented.** |
| D-3 | Where does the hand-off coordinator live? | New `migration/daemon.py` (completes Tyler's stubbed daemon), invoked from the sequential loop. **Resolved; implemented.** |
| D-4 | Single vs double barrier for retrieval? | **Single barrier** (retrieve on subsequent tick) per Tyler's default. **Resolved.** |
| D-5 | Trigger for Phase 1? | **`HandoffManager.evaluate()`** (tick-based stub) in `migration/binding.py` alongside `HandoffEvent`. Extension path is clear: replace tick check with locale geometry without changing the call site. **Resolved; implemented.** |
| D-6 | New 2-edge sequential scenario, or smoke-test-only for Phase 1? | Build both — no-CARLA smoke test for fast iteration, 2-edge scenario for end-to-end integration. **Resolved.** |
| D-7 | Does modeled latency *gate* state application? | **No** in Phase 1 — record-only. Gating is a Phase 2 research lever. **Resolved.** |

---

## Why this is the right Phase 1

It builds the smallest end-to-end slice that exercises the governing principle
(instant mechanism + modeled measurement), on the simplest substrate (one process,
shared memory), reusing Tyler's existing primitives and *completing* the two modules
he explicitly left forthcoming (`link`, `daemon`). Everything here lifts directly into
`-eo` (Phase 2) by swapping the in-process store/ping for the registration-server
do-tick array + a peer RPC — with the cost model and snapshot contract unchanged.
