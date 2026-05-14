# Plan: Distributed Instrumentation and Evaluation

## Context

Edge fusion was a NOP relay for an extended period; regression tests passed only because Tyler changed vehicle speeds, masking the failure entirely. The root structural problem: no process independently verifies that data is flowing through the pipeline at runtime, and `eval_manager.evaluate()` silently produces empty metrics in distributed mode because actor-side data never reaches the orchestrator.

Two independent gaps:

1. **Instrumentation gap**: nothing asserts that predictions are actually flowing each tick. A single `if not fused_predictions: logger.warning(...)` would have caught the NOP relay immediately.

2. **Evaluation gap**: `eval_manager.evaluate()` reads `vm.agent.planning_metrics`, `vm.client_metrics`, and `edge_manager.evaluate()` profiler data. In distributed mode, all of this lives on actor/edge processes — the orchestrator's proxy objects are empty shells.

**Target state**: symmetric proxy architecture at every level. Orchestrator holds `is_proxy=True` edge managers containing `is_proxy=True` VehicleManagers. Actors serialize their metrics on TICK_DONE and push them upstream. `eval_manager.evaluate()` runs identically in distributed and sequential modes. Runtime assertions catch NOP pipelines within the first tick.

---

## Hypothesis

Symmetric `is_proxy` at every level + per-TICK_DONE metrics serialization + boundary-level logging is sufficient to:
- Detect NOP pipelines immediately at runtime
- Make `eval_manager.evaluate()` produce correct output in distributed mode
- Give the orchestrator a complete audit trail of what each actor sent each tick

**Counter-hypothesis**: GT labeling (`_label_brake_attributions_gt`) calls `vm.vehicle.get_location()` / `vm.vehicle.get_velocity()` on live CARLA actors. Orchestrator proxy VMs get their CARLA actor reference from `create_edge_manager_from_scenario_runner()` (ego_vehicle + other_vehicles args). If actor containers spawn their own vehicles independently, the orchestrator may not have valid actor references for GT labeling post-scenario. **Mitigation**: GT labeling runs at `eval_manager.evaluate()` time while CARLA is still up; the orchestrator already connects to CARLA for world simulation.

---

## Implementation Checklist

### 1. CARLA actor ID passthrough — proxy VMs at every level hold a valid actor reference

`RegistrationInfo.actor_id` (field 4) already carries the CARLA actor ID — actors send it when registering with the edge. Edge stores it in `EdgeActorInfo.actor_id`. The gap is that it never reaches the orchestrator.

**`ecav/protos/ecloud.proto`:**
- [x] Add `ActorCarlaInfo` message, `ActorCarlaInfoResponse`, extend `EdgeReadyNotification` with `actor_infos`, add `carla_actor_id = 14` and `pickled_actor_metrics = 13` to `VehicleUpdate`, add `Server_GetAllActorCarlaInfos` RPC; stubs regenerated and copied to `ecav/protos/`

**`ecav/ecav2/edge_process.py` — `notify_actors_ready()`:**
- [x] Populate `request.actor_infos` from `self.actors.values()` before sending `Edge_ActorsReady`

**`ecav/ecloud_server/ecloud_server.cc`:**
- [x] Add `actor_carla_infos` to `EdgeInfo` struct; populate in `Edge_ActorsReady` under lock; implement `Server_GetAllActorCarlaInfos`; rebuilt cleanly

**`ecav/scenario_testing/utils/sim_api.py`:**
- [x] `run_comms()`: after ACTORS_READY call `Server_GetAllActorCarlaInfos` and log (observability only — proxy VMs not yet created)
- [x] `server_unpack_vehicle_updates()`: on first `carla_actor_id` per vehicle, call `world.get_actor()` and assign `manager_proxy.vehicle`; set `_carla_actor_linked` sentinel to avoid repeated lookups

**`ecav/ecav2/ecloud_actor_client.py`:**
- [x] Populate `vehicle_update.carla_actor_id = self.vehicle_manager.vehicle.id` for VEHICLE actors in every update

---

### 2. Instrumentation — data boundary logging and assertions

**`ecav/ecav2/edge_process.py` — run_edge_step():**
- [x] `FUSION_WARMUP_TICKS = 3` class constant
- [x] Per-actor DEBUG log: `[DATA_FLOW] tick=N actor=K features=yes/no(Nb) objects=yes/no(Nb)`
- [x] Per-tick INFO summary: `[DATA_FLOW] tick=N features=K/M objects=K/M predictions=P vehicles_updated=V`
- [x] WARNING if no fused predictions after warmup
- [x] WARNING if zero actors sent features after warmup
- [x] Verbose assertion: assert at least one actor had data after warmup

**`ecav/ecav2/ecloud_actor_client.py` — `send_vehicle_update()`:**
- [x] `[DATA_FLOW]` DEBUG log on outgoing update: state, features/objects bytes, carla_actor_id
- [x] `[DATA_FLOW]` DEBUG log on response: N objects from edge predictions, or "empty"
- [x] WARNING if `connected_to_edge` and `opt.apply_ml` and predictions empty after tick > 3

**All logs use `[DATA_FLOW]` prefix for greppability ✓**

---

### 3. Proto: add `pickled_actor_metrics` to VehicleUpdate

**`ecav/protos/ecloud.proto`:**
- [x] `bytes pickled_actor_metrics = 13` added to `VehicleUpdate` (done with section 1 proto pass)
- [x] Stubs regenerated and copied

---

### 4. Actor: serialize metrics on TICK_DONE

**`ecav/ecav2/ecloud_actor_client.py` — where TICK_DONE is set (in `tick()`):**
- [x] When `vehicle_state == TICK_DONE`, serialize and attach metrics:
  ```python
  metrics = {
      'planning_metrics': self.vehicle_manager.agent.planning_metrics,
      'client_metrics': self.vehicle_manager.client_metrics,
      'vehicles_detected': self.vehicle_manager.vehicles_detected,
  }
  vehicle_update.pickled_actor_metrics = pickle.dumps(metrics)
  ```
- [x] Only for `actor_type == VEHICLE` (RSUs don't have planning_metrics / agent)
- [x] Log at INFO: `[DATA_FLOW] Serialized actor metrics on TICK_DONE: planning_metrics brake_attrs=N, client_metrics collisions=K`

---

### 5. Edge_process: store metrics in proxy VM, forward in EdgeTickComplete

**`ecav/ecav2/edge_process.py` — `Edge_ActorSendUpdate` handler:**
- [x] On TICK_DONE with `pickled_actor_metrics` and VEHICLE: unpack and store in `actor_info.manager.agent.planning_metrics / client_metrics / vehicles_detected`
- [x] `EdgeTickComplete.vehicle_updates` carries the full `last_update` (including `pickled_actor_metrics`) — flows automatically

---

### 6. Orchestrator: unpack metrics and populate proxy VMs

**`ecav/scenario_testing/utils/sim_api.py` — `server_unpack_vehicle_updates()`:**
- [x] Added metrics unpack block after features/objects unpacking — guarded by `not is_rsu` and `pickled_actor_metrics` present; populates `manager_proxy.agent.planning_metrics`, `.client_metrics`, `.vehicles_detected`; logs `[DATA_FLOW]`

---

### 7. `is_proxy` on BaseEdgeManager

**`ecav/core/application/edge/edge_manager/edge_manager_base.py`:**
- [x] Add `is_proxy=False` parameter to `BaseEdgeManager.__init__`
- [x] Store as `self.is_proxy = is_proxy`
- [x] Guard with `if not is_proxy:`:
  - `create_latency_model()` call
  - `create_mac_model()` call
  - `self.debug = EdgeMetrics(...)` (keep instance as None or empty stub)
- [x] In `start_edge()` implementations: concrete subclasses guard model loading with `if not self.is_proxy: return` (raise NotImplementedError stays only for non-proxy path)
- [x] `evaluate()` and `_label_brake_attributions_gt()` must work when `is_proxy=True` (they only read from `vehicle_manager_list`, not profiler — check each subclass)
- [x] Add `run_step()` guard in base: if `self.is_proxy: return None`

**`ecav/scenario_testing/utils/sim_api.py` — `create_edge_manager_from_scenario_runner()`:**
- [x] Pass `is_proxy=distributed` to edge manager constructor in distributed mode
- [x] Also pass `is_proxy=distributed` to VehicleManager/RSUManager calls within this function in distributed mode (they're created as proxies anyway; flag makes intent explicit and skips sensor spawning)
- [x] Skip `edge_manager.start_edge()` when `is_proxy=True`

Note: `WorldFusionAdaptiveEdge`, `LateFusionEdge`, and others each have their own `__init__` that calls `super().__init__()` and then does model loading. Each must be audited — the `is_proxy` guard belongs in each subclass' `__init__` around the model load block.

---

### 8. Edge evaluation forwarding (completes the chain)

The orchestrator's proxy edge manager `evaluate()` works once VM metrics are populated (step 5). The remaining gap is the edge profiler data (`edge_manager.evaluate()` on the orchestrator side calls into the EdgeMetrics profiler, which is empty in proxy mode).

**Near-term (unblocks testing)**: edge_process already writes EdgeProfiler JSON independently. Orchestrator's `edge.evaluate()` returns empty metrics — this is a known, acceptable gap logged as a TODO.

**Full solution**:
- [x] Add `EdgeEvaluationResult` message to `ecloud.proto`; also added `EdgeEvaluationResultList` and `Server_GetAllEdgeEvaluations` RPC
- [x] Add `Edge_SendEvaluationData(EdgeEvaluationResult) returns (Empty)` RPC
- [x] In edge_process `process_tick(END)`: serialize profiler and call `Edge_SendEvaluationData` before returning False (sends while C++ server is still alive — moved from `cleanup()` to avoid race with server teardown)
- [x] In orchestrator `end()`: poll `Server_GetAllEdgeEvaluations` until all edges report (30s timeout); unpack and set `proxy_edge_manager._proxy_metrics`; proxy `evaluate()` returns `self._proxy_metrics` instead of empty dict
- [x] C++ server: `Edge_SendEvaluationData` stores `pickled_edge_profiler` per edge; `Server_GetAllEdgeEvaluations` returns all results

---

### 9. `start_actors.sh` — thread verbose flag to all containers

`--verbose` is already a defined flag in `ecav/ecav2/arg_utils.py` (used by all processes that call `build_arg_parser`). It just needs to be prompted and threaded through the shell script, following the same pattern as `ml_flag` and `litserve_flag`.

**`start_actors.sh`:**
- [x] Add prompt after existing prompts (around line 10):
  ```bash
  read -p "Enable verbose/debug mode (y/N)? " use_verbose
  ```
- [x] Set flag variable (alongside `ml_flag`/`litserve_flag` setup):
- [x] Thread `$verbose_flag` to all 4 active spawn sites:
  - Orchestrator `ecav.py` (line 280)
  - Edge container `ecav.py` (line 382)
  - Vehicle actor `ecloud_actor_client.py` (line 407)
  - Non-ego / edge-less `ecav.py` (line 430)
- [x] Added `Verbose: $use_verbose` to echo summary block

---

## Key Files

| File | Change |
|------|--------|
| `ecav/protos/ecloud.proto` | Add `ActorCarlaInfo` message; extend `EdgeReadyNotification`; add `carla_actor_id = 14` and `pickled_actor_metrics = 13` to `VehicleUpdate`; add `EdgeEvaluationResult` + RPC |
| `ecav/ecloud_server/ecloud_server.cc` | Store and forward `actor_infos` from `Edge_ActorsReady`; expose via Python-facing path |
| `ecav/ecav2/edge_process.py` | Populate `actor_infos` in `notify_actors_ready()`; instrumentation in `run_edge_step()`; metrics unpack on TICK_DONE; store in proxy VM |
| `ecav/ecav2/ecloud_actor_client.py` | Populate `carla_actor_id` in every update; serialize metrics on TICK_DONE; log edge_predictions receipt |
| `ecav/scenario_testing/utils/sim_api.py` | Link proxy VMs to CARLA actors after `Edge_ActorsReady`; `server_unpack_vehicle_updates()` unpack metrics; `create_edge_manager_from_scenario_runner()` pass `is_proxy=distributed` |
| `ecav/core/application/edge/edge_manager/edge_manager_base.py` | Add `is_proxy` flag; guard model loading |
| Each edge manager subclass (`worldfusion_adaptive`, `late_fusion`, etc.) | Guard model load in `__init__` and `start_edge()` with `is_proxy` |
| `start_actors.sh` | Add `use_verbose` prompt; set `$verbose_flag`; thread to all 5 spawn sites |

---

## Sequencing

**Recommended order for a testing-unblocked state:**
1 → 2 → 3–6 as a unit → 7 → 8 → 9

- Items 1 (CARLA ID) and 2 (instrumentation) are independent of each other but both precede the metrics chain
- Items 3–6 (proto + actor + edge + orchestrator) form a single atomic unit — partial states will cause crashes
- Item 7 (`is_proxy` on edge manager) is independent; do it before item 8
- Item 8 (edge evaluation forwarding) depends on item 7
- Item 9 (`start_actors.sh`) is independent; can be done at any point but should be verified last against a full distributed run

---

## Verification

1. Run `openscenario_3_edge_worldfusion --apply_ml -d` — confirm `[DATA_FLOW]` lines show features flowing each tick, no WARNING for empty predictions after warmup
2. Run same scenario to completion — confirm `simulation_metrics.json` contains non-zero brake_attributions with `gt_matched_actor_id` populated (proves GT labeling ran on real data)
3. Diff `simulation_metrics.json` against a sequential run — structures should match, values will differ (different timing, but same schema)
4. Deliberately break feature serialization in actor — confirm WARNING fires within 3 ticks
5. Run with `--verbose` — confirm hard assertion catches the broken state
