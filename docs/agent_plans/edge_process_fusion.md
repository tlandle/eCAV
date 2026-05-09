# Plan: Edge Process Fusion — Make edge_process.py Run Actual Fusion

## Context

The distributed simulation architecture requires that the edge process (`ecav/ecav2/edge_process.py`) runs the fusion logic (WorldFusion or late fusion), not the orchestrator. The correct data flow is:

```
orchestrator → ecloud_server → edge_process → actors
actors → edge_process → ecloud_server → orchestrator
```

**What is broken today:**

1. `edge_process.fuse_predictions()` is a placeholder that passes actor detections through unmodified. No edge manager is ever instantiated. The edge container is a NOP relay.

2. `fused_predictions` key type bug: `fuse_predictions()` stores under the string key `f"{actor_type}_{vehicle_index}"` (e.g. `"2_0"`), but `Edge_ActorSendUpdate` looks up by integer `vehicle_index`. The lookup never matches. Predictions are never returned to actors even in the passthrough case.

3. The orchestrator's `run_scenario()` calls `edge.run_step()` unconditionally in distributed mode. The orchestrator's proxy VMs have no data (features are not forwarded through EdgeTickComplete), so fusion produces nothing.

4. The orchestrator sends `PULL_OBJECTS_AND_TICK` every tick, causing actors to call `Client_GetObjects` from the orchestrator. This would overwrite any predictions received from the edge's `Edge_ActorSendUpdate` response with empty orchestrator predictions.

5. Regression tests 3 and 4 (`WorldFusion -d`, `WorldFusion -d -l`) are false positives. Tyler changed vehicle speeds so the ego clears the intersection before the oncoming vehicle arrives. Detection was never required; the scenario passes regardless of whether fusion produces output.

**Target state:** edge_process instantiates a real edge manager, runs fusion per tick, distributes predictions back to actors. Orchestrator sends `TICK` only and does not run `edge.run_step()` in distributed mode.

---

## Hypothesis

Moving the edge manager instantiation and `run_step()` call into edge_process, fixing the key type bug, and stopping the orchestrator from issuing `PULL_OBJECTS_AND_TICK` in distributed mode is sufficient to restore correct distributed fusion for the single-ego WorldFusion scenario.

**Verification:** Run `openscenario_3_edge_worldfusion --apply_ml -d -l` with the scenario restored to its original vehicle speeds (so the oncoming Lincoln actually arrives at the intersection while the ego is still approaching). Confirm edge_process logs show `[WorldFusion Edge]` fusion output and that the ego applies at least one brake event attributed to an edge prediction.

---

## Implementation Checklist

### 1. Move `_select_edge_manager()` to the shared registry

- [ ] In `ecav/scenario_testing/utils/sim_api.py`: replace the `_select_edge_manager()` body with a call to `get_edge_class()` from the shared registry
- [ ] Confirm both call sites in sim_api (`create_edge_manager_from_scenario_runner`, `create_single_cav_world`) now go through `get_edge_class()`
- [ ] `get_edge_class()` is already defined and exported from `ecav/core/application/edge/edge_manager/__init__.py` — no changes needed there

### 2. Add `is_proxy` flag to `VehicleManager` and `RSUManager`

Using real manager instances (not new proxy classes) keeps the interface guaranteed-compatible with the edge manager. The `is_proxy` flag gates initialization paths that are irrelevant in the edge process.

- [ ] Add `is_proxy=False` parameter to `VehicleManager.__init__` (`ecav/core/common/vehicle_manager.py`)
- [ ] Audit `VehicleManager.__init__` and add `if not self.is_proxy:` guards around: sensor spawning, behavior agent creation, ML manager initialization, and any other paths not already gated by `perception_active=False` or `run_distributed=True`
- [ ] Add `is_proxy=False` parameter to `RSUManager.__init__` (`ecav/core/common/rsu_manager.py`)
- [ ] Audit `RSUManager.__init__` and add corresponding guards

The `localizer` must still be created in proxy mode (edge manager reads `localizer.get_ego_pos()`), but it should not run its internal GNSS/estimation pipeline. Determine during implementation whether `get_ego_pos()` can have its return value overridden directly (e.g., a `_proxy_pos` attribute the proxy path sets) or whether a lightweight `set_ego_pos()` setter is needed.

### 3. Add `setup_edge_manager()` to `EdgeProcess`

Called in two phases:

**Phase A — on receipt of scenario config** (end of `register_with_orchestrator()`): start model loading asynchronously so the heavy PyTorch init runs while actors are registering.

```python
# at end of register_with_orchestrator(), after self.scenario_ready.set():
asyncio.create_task(self._init_edge_manager_model())
```

```python
async def _init_edge_manager_model(self):
    """Phase A: resolve and store edge manager class + config. Model loads in Phase B."""
    import json
    from ecav.core.common.cav_world import CavWorld
    from ecav.core.application.edge.edge_manager import get_edge_class

    scenario = json.loads(self.scenario_yaml_str)
    edge_cfg = scenario['scenario']['edge_list'][self.edge_index]
    world_dt = scenario.get('world', {}).get('fixed_delta_seconds', 0.05)
    edge_dt   = scenario.get('edge_base', {}).get('edge_dt', 0.2)

    self._edge_manager_cls  = get_edge_class(edge_cfg.get('manager_type', 'late_fusion'))
    self._edge_manager_cfg  = edge_cfg
    self._edge_manager_args = dict(world_dt=world_dt, edge_dt=edge_dt,
                                   run_distributed=True)
    self._cav_world = CavWorld(apply_ml=True, config=None)
    logger.info("Edge manager class resolved: %s", self._edge_manager_cls.__name__)
```

**Phase B — after all actors ready** (called from `notify_actors_ready()` before notifying orchestrator):

```python
async def setup_edge_manager(self):
    """Phase B: connect to CARLA, instantiate real VehicleManager/RSUManager proxies,
    instantiate edge manager (loads model here)."""
    import carla as _carla
    import json
    from ecav.core.common.vehicle_manager import VehicleManager
    from ecav.core.common.rsu_manager import RSUManager

    scenario = json.loads(self.scenario_yaml_str)
    client = _carla.Client(self.carla_ip, 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    carla_map = world.get_map()
    edge_cfg = self._edge_manager_cfg
    current_time = scenario.get('current_time', '')

    # Instantiate edge manager (loads PyTorch model)
    self.edge_manager = self._edge_manager_cls(
        world, edge_cfg, self._cav_world,
        carla_client=client,
        **self._edge_manager_args
    )

    # RSUs first — WorldFusion requires agent 0 to be the RSU
    rsu_cfgs = edge_cfg.get('rsus', [])
    rsu_keys = sorted(k for k, a in self.actors.items()
                      if a.actor_type == ecloud.ActorType.RSU)
    for idx, key in enumerate(rsu_keys):
        info = self.actors[key]
        carla_actor = world.get_actor(info.actor_id)
        rsu_manager = RSUManager(
            world, rsu_cfgs[idx], carla_map, self._cav_world,
            current_time, is_proxy=True, run_distributed=True
        )
        # Override CARLA actor with the live one from this simulation
        rsu_manager.rsu = carla_actor
        info.manager = rsu_manager
        self.edge_manager.add_rsu(rsu_manager)

    vehicle_cfgs = edge_cfg.get('vehicles', [])
    vehicle_keys = sorted(k for k, a in self.actors.items()
                          if a.actor_type == ecloud.ActorType.VEHICLE)
    for idx, key in enumerate(vehicle_keys):
        info = self.actors[key]
        carla_actor = world.get_actor(info.actor_id)
        vm = VehicleManager(
            vehicle=carla_actor,
            vehicle_index=info.vehicle_index,
            config_yaml=scenario,
            application=self.application,
            carla_world=world,
            carla_map=carla_map,
            cav_world=self._cav_world,
            current_time=current_time,
            is_proxy=True,
            run_distributed=True,
            perception_active=False
        )
        info.manager = vm
        self.edge_manager.add_member(vm)

    self.edge_manager.start_edge()
    logger.info("Edge manager ready: %s, %d vehicles, %d RSUs",
                type(self.edge_manager).__name__,
                len(self.edge_manager.vehicle_manager_list),
                len(self.edge_manager.rsu_manager_list))
```

Update `notify_actors_ready()`:
```python
async def notify_actors_ready(self):
    await self.setup_edge_manager()   # must complete before orchestrator starts ticking
    request = ecloud.EdgeReadyNotification()
    request.edge_index = self.edge_index
    request.num_actors = self.num_actors_ready
    await self.orchestrator_stub.Edge_ActorsReady(request)
    logger.info("Edge %s: notified orchestrator — %d actors ready",
                self.edge_index, self.num_actors_ready)
```

Add instance variables to `EdgeProcess.__init__`:
```python
self.edge_manager = None
self._edge_manager_cls  = None
self._edge_manager_cfg  = None
self._edge_manager_args = {}
self._cav_world = None
```

Add `manager = None` to `EdgeActorInfo.__init__` to hold the VehicleManager/RSUManager reference.

### 4. Add `run_edge_step()` to `EdgeProcess`; fix key bug

```python
def run_edge_step(self, tick_id: int):
    """Push actor data into proxy managers then run the edge manager step."""
    if self.edge_manager is None:
        logger.warning("run_edge_step called before edge_manager ready")
        return

    import pickle, zlib, msgpack, torch
    import msgpack_numpy as m_np
    import carla as _carla
    m_np.patch()

    for key, info in self.actors.items():
        if info.last_update is None or info.manager is None:
            continue
        upd = info.last_update
        mgr = info.manager

        # Pose — set on localizer so get_ego_pos() returns updated position
        if upd.HasField('transform'):
            t = upd.transform
            tf = _carla.Transform(
                _carla.Location(t.location.x, t.location.y, t.location.z),
                _carla.Rotation(roll=t.rotation.roll,
                                pitch=t.rotation.pitch,
                                yaw=t.rotation.yaw)
            )
            mgr.localizer.set_proxy_pos(tf)   # exact method TBD during implementation

        # Intermediate features (WorldFusion)
        if upd.pickled_features:
            try:
                feat_np = msgpack.unpackb(
                    zlib.decompress(upd.pickled_features), raw=False)
                mgr.perception_manager.feature_dict = {
                    k: torch.from_numpy(v) for k, v in feat_np.items()
                }
            except Exception as e:
                logger.warning("Feature unpack failed for %s: %s", key, e)

        # Detection objects (late fusion)
        if upd.pickled_agent_objects:
            try:
                objects = pickle.loads(upd.pickled_agent_objects)
                if info.actor_type == ecloud.ActorType.RSU:
                    mgr.objects = objects
                else:
                    mgr.agent.objects = objects
            except Exception as e:
                logger.warning("Objects unpack failed for %s: %s", key, e)

    try:
        serialized_preds = self.edge_manager.run_step(tick_id)
        if serialized_preds is not None:
            for obj_buf in serialized_preds.all_object_buffers:
                vm_idx = obj_buf.vehicle_id  # index in vehicle_manager_list
                actor_key = f"{int(ecloud.ActorType.VEHICLE)}_{vm_idx}"
                if actor_key in self.actors:
                    self.fused_predictions[actor_key] = obj_buf.pickled_edge_predictions
    except Exception as e:
        logger.exception("edge_manager.run_step failed at tick %s: %s", tick_id, e)
```

Replace `fuse_predictions()` call in `process_tick()`:
```python
# was: self.fuse_predictions()
self.run_edge_step(tick_id)
```

Fix the key lookup bug in `Edge_ActorSendUpdate`:
```python
# was: if vehicle_index in self.edge_process.fused_predictions:
actor_key = f"{int(actor_type)}_{vehicle_index}"
if actor_key in self.edge_process.fused_predictions:
    reply.pickled_edge_predictions = self.edge_process.fused_predictions[actor_key]
```

Remove the now-dead `fuse_predictions()` method.

### 5. Fix `run_scenario()` in `openscenario_3_edge_worldfusion.py`

**Change tick command** — distributed mode always uses `TICK`; predictions come from edge_process via `Edge_ActorSendUpdate` response, not from orchestrator pull:

```python
if opt.distributed:
    command = ecloud.Command.TICK  # edge_process distributes predictions directly
    flag = scenario_manager.broadcast_message(command)
    scenario_manager.tick_world()
else:
    scenario_manager.tick()
```

**Skip `edge.run_step()` in distributed mode** — fusion belongs to edge_process:

```python
if not opt.distributed:
    for edge in edge_list:
        serialized_predictions = edge.run_step(step)
        # sequential mode: _update_agents() drives vehicles directly; no push needed
```

The `edge_list` object is still created by `create_edge_manager_from_scenario_runner()` for end-of-scenario evaluation structure (vehicle_manager_list count, `eval_manager`). It runs no per-tick logic in distributed mode.

### 6. Apply same run_scenario() fix to all other edge scenario files

The same two changes (TICK command, skip edge.run_step) must be applied to every scenario file that has an edge:

- [ ] `openscenario_3_edge_worldfusion.py` (primary target)
- [ ] `openscenario_3_edge_late_fusion.py`
- [ ] `openscenario_3_edge_worldfusion_4ego.py`
- [ ] `openscenario_3_edge_late_fusion_4ego.py`
- [ ] Any other `*_edge_*.py` scenario files

---

## Key Files

| File | Change |
|------|--------|
| `ecav/ecav2/edge_process.py` | Proxy classes; `setup_edge_manager()`; `_init_edge_manager_model()`; `run_edge_step()`; fix key type bug in `Edge_ActorSendUpdate`; remove `fuse_predictions()` |
| `ecav/scenario_testing/openscenario_3_edge_worldfusion.py` | Send `TICK` not `PULL_OBJECTS_AND_TICK`; skip `edge.run_step()` in distributed mode |
| `ecav/scenario_testing/utils/sim_api.py` | Replace `_select_edge_manager()` body with call to `get_edge_class()` |
| All other `*_edge_*.py` scenario files | Same run_scenario() fix as worldfusion |

**Shared utility already in place:** `get_edge_class()` in `ecav/core/application/edge/edge_manager/__init__.py` — no changes needed.

---

## Known Deferral

`eval_manager.evaluate()` at scenario end calls `edge.evaluate()` on the orchestrator's edge manager object. In distributed mode this object has no per-tick profiler data (it never ran `run_step()`). The edge_process's EdgeProfiler writes its own JSON independently. Cross-process evaluation aggregation is out of scope for this fix. The orchestrator's `edge.evaluate()` call will return empty metrics; this is acceptable for now.

---

## Verification

1. Restore original vehicle speeds in the scenario (revert Tyler's timing change) so the Lincoln arrives at the intersection while the ego is still approaching.
2. Start WF gRPC server: `start_actors.sh` with `scenario=openscenario_3_edge_worldfusion`, `num_ego=1`, `num_rsu=1`, `num_edges=1`, `use_ml=Y`, `use_litserve=Y`.
3. In edge container logs: confirm `[WorldFusion Edge] Initialization complete` and per-tick `[WorldFusion Edge] Collected N feature_dicts`.
4. In actor container logs: confirm `Edge_ActorSendUpdate` responses contain `pickled_edge_predictions` (non-empty bytes).
5. In actor container logs: confirm `self.vehicle_manager.agent.edge_predictions` is set (check `[CLIENT VEHICLE TICK]` timing lines for brake events).
6. In orchestrator logs: confirm no `PULL_OBJECTS_AND_TICK` commands and no `push_edge_objects` calls.
7. Ego should apply brake events attributable to edge predictions before reaching the intersection.
8. Re-run regression tests 3 and 4 with the restored scenario to get real pass/fail results.
