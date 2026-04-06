# Distributed Actor Simulation Plan: WorldFusion and LateFusion

**Goal**: Run both WorldFusion (`openscenario_3_edge_worldfusion`) and LateFusion (`openscenario_3_edge`) scenarios in full Mode 4 distributed deployment: containerized actors, containerized edge processes running the actual EdgeManager, and an optional LitServe inference server — all communicating via gRPC, deployable across any number of nodes.

**References**:
- `docs/agent_plans/EDGE_ARCHITECTURE_PROPOSAL.md` — distributed edge architecture (basis for this plan)
- `docs/agent_plans/worldfusion_litserve_plan.md` — LitServe transport optimization and measurement
- `docs/agent_plans/grpc_perception_migration.md` — gRPC perception migration history

---

## Deployment Mode Matrix

| Mode | Actors | EdgeManager | Target |
|------|--------|-------------|--------|
| 1 — Sequential | In orchestrator process | In orchestrator process | Dev/debug; must not break |
| 2 — Distributed, co-located processes | Separate host processes | In orchestrator process | Stepping stone only |
| **4 — Full distributed** | **Containerized** | **Containerized edge_process** | **Research target** |

Mode 3 (distributed actors, co-located EdgeManager) is not of research interest and is not a goal of this plan. Mode 1 must not be broken. Mode 4 is the primary target.

---

## Process Topology (Mode 4)

For `openscenario_3_edge` (LateFusion) and `openscenario_3_edge_worldfusion` (WorldFusion):

```
┌─────────────────────────────────────────────────────────────────────┐
│ HOST (or Orchestrator Node)                                          │
│                                                                      │
│  CarlaUE4 process                                                    │
│  ecav.py process [orchestrator]                                      │
│    - Loads YAML, determines topology                                 │
│    - Spawns C++ ecloud_server                                        │
│    - Holds ProxyEdgeManagers (data containers, not fusion logic)     │
│    - Holds ProxyVehicleManagers (for evaluation/debug only)          │
│    - Tick loop: broadcast_message() only, no edge.run_step()         │
│  C++ ecloud_server (port 50051)                                      │
│    - When edges registered: pushes EdgeTick to edge containers       │
│    - When no edges: pushes Tick to individual actor push servers     │
│    - Receives Edge_TickComplete from edges; unblocks orchestrator    │
└─────────────────────────────────────────────────────────────────────┘
         ↕ gRPC (Edge_Register, Edge_TickComplete, Edge_PushTick)

┌─────────────────────┐   ┌─────────────────────┐
│ edge_0 container    │   │ edge_N container     │
│                     │   │                      │
│ edge_process.py     │   │ edge_process.py      │
│  - EdgeServer gRPC  │   │  - EdgeServer gRPC   │
│  - ProxyVehicleMs   │   │  - ProxyVehicleMs    │
│  - ProxyRSUMs       │   │  - ProxyRSUMs        │
│  - EdgeManager      │   │  - EdgeManager       │
│    (actual fusion)  │   │    (actual fusion)   │
│  - LitServe client  │   │  - LitServe client   │
│    (WorldFusion)    │   │    (WorldFusion)     │
└─────────────────────┘   └─────────────────────┘
         ↕ gRPC (Edge_ActorRegister, Edge_ActorSendUpdate, PushTick)

┌──────────────────────┐  ┌─────────────────────┐  ┌──────────────────┐
│ ego_vehicle_0        │  │ rsu_0 container     │  │ non_ego_vehicles │
│ container            │  │                     │  │ container        │
│                      │  │ ecloud_actor_       │  │                  │
│ Process 1:           │  │ client.py           │  │ ScenarioRunner   │
│  ScenarioRunner      │  │  - RSUManager       │  │ (-i -1)          │
│  (CARLA behavior,    │  │  - Sensors          │  │ No gRPC          │
│   goal following)    │  │  - Perception       │  │                  │
│                      │  │  - gRPC to edge     │  │                  │
│ Process 2:           │  │                     │  │                  │
│  ecloud_actor_       │  │ [LateFusion]        │  │                  │
│  client.py           │  │ YOLO → detections   │  │                  │
│  - VehicleManager    │  │  → pickled_objects  │  │                  │
│  - Sensors           │  │  → edge             │  │                  │
│  - Perception        │  │                     │  │                  │
│  - gRPC to edge      │  │ [WorldFusion]       │  │                  │
│                      │  │ voxels + camera     │  │                  │
│ [LateFusion]         │  │  → pickled_features │  │                  │
│ YOLO → detections    │  │  → edge             │  │                  │
│  → pickled_objects   │  │  (edge calls        │  │                  │
│  → edge              │  │   LitServe)         │  │                  │
│                      │  │                     │  │                  │
│ [WorldFusion]        │  │                     │  │                  │
│ voxels + camera      │  │                     │  │                  │
│  → pickled_features  │  │                     │  │                  │
│  → edge (edge calls  │  │                     │  │                  │
│    LitServe)         │  │                     │  │                  │
└──────────────────────┘  └─────────────────────┘  └──────────────────┘

Optional:
┌───────────────────────────────────────────────────────┐
│ LitServe server (litserve_models.py)                  │
│  - /extract_features HTTP (port 18000)                │
│    ← called by edge process (WorldFusion)             │
│  - gRPC PerceptionService (port 18001)                │
│    ← called by actor containers (LateFusion YOLO)     │
└───────────────────────────────────────────────────────┘
```

---

## Data Flow Per Fusion Type

### LateFusion (`openscenario_3_edge`)

Each actor (vehicle, RSU) runs YOLO perception locally or via LitServe gRPC (`-l`). The actor sends bounding box detections to the edge. The edge runs late fusion (BM2CP or simple association), tracking, and prediction.

```
Actor tick:
  perception_manager.run_step()
    → If -l: gRPC call to LitServe PerceptionService.DetectYolo
    → If local: YOLOv5 local inference
    → agent.objects = bounding boxes
  VehicleUpdate.pickled_agent_objects = pickle(agent.objects)
  → Edge_ActorSendUpdate(VehicleUpdate) to edge
  ← ObjectBuffer.pickled_edge_predictions (previous tick's fused predictions)

Edge tick:
  Receive VehicleUpdate from all actors
  ProxyVehicleManager.agent.objects = unpickle(pickled_agent_objects) per actor
  EdgeManager.run_step()  [BM2CP or late_fusion]
    reads .agent.objects from all ProxyVehicleManagers + ProxyRSUManagers
    produces fused detections + tracks + predictions
  fused_predictions stored per actor_index
    → returned as ObjectBuffer on next Edge_ActorSendUpdate response
```

### WorldFusion (`openscenario_3_edge_worldfusion`)

Each actor runs sensor preprocessing only (voxelization, camera resize) — no LitServe call. The actor sends pre-LitServe sensor data to the edge. The edge batches all actors' data, calls LitServe once (batch=N), gets spatial features, runs Where2comm fusion + detection + tracking + prediction.

```
Actor tick:
  perception_manager.run_step()
    → _build_batch(): LiDAR → voxels, camera → (1,4,3,360,480) float32
    → Does NOT call LitServe (edge does that in Mode 4)
    → Stores batch in self._pending_batch
  VehicleUpdate.pickled_features = msgpack(_pending_batch)  ← pre-LitServe sensor data
  → Edge_ActorSendUpdate(VehicleUpdate) to edge
  ← ObjectBuffer.pickled_edge_predictions (previous tick's fused predictions)

Edge tick:
  Receive VehicleUpdate from all actors (1 vehicle + 1 RSU for openscenario_3)
  Unpack pickled_features from each actor → sensor batches
  Merge batches: stack camera tensors, merge voxel coords (reindex batch_idx per actor)
  POST /extract_features to LitServe (merged batch, batch_size=N)
    → spatial_features shape: (N, C, H, W)
  Split features on batch dim → ProxyVehicleManager.perception_manager.feature_dict per actor
  EdgeManager.run_step()  [WorldFusion Where2comm]
    reads .perception_manager.feature_dict from all proxies
    produces fused detections + tracks + predictions
  fused_predictions stored per actor_index
    → returned as ObjectBuffer on next Edge_ActorSendUpdate response
```

**LitServe is mandatory for WorldFusion in Mode 4.** Local feature extraction on the edge container is not planned and not supported. Running `openscenario_3_edge_worldfusion` in distributed mode requires a LitServe server (`-l` flag on both orchestrator and edge process). Attempting to run WorldFusion Mode 4 without LitServe will fail at the `_extract_features_batched()` call in `run_edge_step()`.

**Critical semantic note**: In WorldFusion Mode 4, `VehicleUpdate.pickled_features` contains pre-LitServe sensor batch data (voxels + camera). In sequential mode, `pickled_features` would contain post-LitServe spatial features if they were transmitted (they are not — sequential mode has no gRPC). The field is only populated in distributed mode, so the semantic change does not break sequential operation. Documentation must be clear about this distinction.

---

## Proxy Pattern

### Motivation

The original eCAV research used "proxy" instances of manager classes to allow seamless switching between sequential and distributed modes. The same approach is central to this plan. Without a proxy mechanism, every access to `vehicle_manager.vehicle`, `agent.objects`, `perception_manager.feature_dict` requires an `is_distributed` branch. Evaluation code, profilers, and EdgeManagers then have dual code paths that diverge over time.

With the proxy flag, the same attribute names hold in both modes. The proxy instance stores gRPC-received data in identical attributes to the real instance. The EdgeManager, evaluation manager, and profiler are unaware of the difference — they call the same API surface.

### Implementation approach

**No distinct proxy classes are created.** The `proxy=True` flag is added to the existing class constructors: `VehicleManager`, `RSUManager`, and `EdgeManagerBase`. When `proxy=True`, the constructor early-exits before sensor creation, CARLA attachment, model loading, and map access. The same attribute names are populated as simple data containers instead.

This keeps a single class per manager type. Downstream code needs no type checks. The `proxy` attribute is visible in debugger output and log traces.

When `proxy=True`:

- **`VehicleManager` / `RSUManager`**: Skip sensor creation, CARLA attachment, localization init. Expose same attribute names (`agent`, `perception_manager`, `transform`, `velocity`, `vehicles_detected`) initialized as simple data containers. Add `update_from_grpc(vehicle_update: VehicleUpdate)` method that unpickles/unpacks proto fields into those attributes.

- **`EdgeManagerBase`**: Skip model loading, CARLA map access. Hold only gRPC metadata fields (`edge_index`, `edge_ip`, `edge_port`, `vehicle_manager_list`, `rsu_manager_list`, `profiler`). All subclass `run_step()` / `update_information()` methods are no-ops when `proxy=True`.

The manager factory (`_select_edge_manager()` in `sim_api.py`) passes `proxy=True` when `is_distributed=True` and edges are standalone.

### Proxy instances in the orchestrator (`ecav.py`)

When Mode 4 is detected, the orchestrator creates `proxy=True` manager instances:

| Class | `proxy=True` attributes held | Purpose |
|-------|------------------------------|---------|
| `EdgeManagerBase` | `edge_index`, `edge_ip`, `edge_port`, `vehicle_manager_list`, `rsu_manager_list`, `profiler` | Evaluation aggregation; tick coordination metadata |
| `VehicleManager` | `vehicle_index`, `transform`, `velocity`, `agent.objects`, `perception_manager.feature_dict`, `vehicles_detected` | Evaluation and debug data only |

The orchestrator's main loop calls `broadcast_message()` only — no `edge.run_step()`, no `edge.update_information()`. The proxy `EdgeManagerBase` instances exist solely for teardown, evaluation, and debug output.

### Proxy instances in the edge process (`edge_process.py`)

The edge process holds `proxy=True` actor manager instances with the same API surface as live managers:

| Class | `proxy=True` attributes held | Purpose |
|-------|------------------------------|---------|
| `VehicleManager` | `vehicle_index`, `push_port`, `push_stub`, `last_update`, `agent.objects`, `perception_manager.feature_dict` | Holds gRPC-received data; EdgeManager reads from these |
| `RSUManager` | Same as above | Same |

The EdgeManager calls `vm.agent.objects` and `vm.perception_manager.feature_dict` in exactly the same way it would with a live instance. The EdgeManager code path is identical in sequential and distributed modes.

---

## Current Implementation Status

Based on static analysis of `ecloud_server.cc`, `edge_process.py`, `ecloud_actor_client.py`, and `ecloud.proto`:

### Done (may have bugs; needs end-to-end testing)

- **`ecloud_server.cc`**: Two-path tick dispatch — when edges are registered, pushes `EdgeTick` to edges (not individual actors). When no edges, pushes `Tick` directly to actor push servers. Controlled by `hasEdges_` flag.
- **`ecloud_server.cc`**: `Edge_Register` handler stores edge IP/port, creates `PushClient` per edge.
- **`ecloud_server.cc`**: `Edge_TickComplete` barrier — waits for all edges to complete, then notifies orchestrator via `simAPIClient_->PushTick()`.
- **`ecloud.proto`**: All required RPCs defined: `Edge_Register`, `Edge_TickComplete`, `Edge_ActorRegister`, `Edge_ActorSendUpdate`, `Edge_PushTick`, `Edge_SendIntermediateFeatures`, `Edge_PerformFusion`.
- **`edge_process.py`**: Registers with orchestrator; receives `Edge_PushTick`; pushes `PushTick` to actors; collects `Edge_ActorSendUpdate`; reports `Edge_TickComplete`. END command propagation to actors is implemented.
- **`ecloud_actor_client.py`**: `Client_GetConnectionInfo` routing — connects to edge if `has_edge=true`, otherwise to orchestrator. `Edge_ActorSendUpdate` path sends VehicleUpdate and receives ObjectBuffer with fused predictions.

### Not done / broken (primary work for this plan)

1. **`edge_process.py:fuse_predictions()`** — stub that passes actor objects through unchanged. No EdgeManager instantiated. This is the central missing piece.
2. **WorldFusion actor path** — actor currently calls LitServe itself (when `-l`) and sends post-LitServe features. For Mode 4, actor should send pre-LitServe sensor batch; edge calls LitServe.
3. **`ecav.py` main loop** — still creates a full EdgeManager and calls `edge.run_step()` in distributed mode. In Mode 4 this is wrong; fusion happens in edge containers.
4. **Proxy class pattern** — not implemented. No `proxy=False` constructor parameter exists on any manager class.
5. **`start_actors.sh`** — invokes `opencda.py` and `opencda/ml_manager/litserve_models.py`. Neither path exists (`opencda/` contains only CMake build artifacts; all Python source is in `ecav/`). The ego container launch command structure (`ecav.py -d -i $i`) is architecturally correct once the path is fixed — ScenarioRunner spawns `Ecav2ActorClient` in-process; no separate actor client process is needed.
6. **`ecav.py`** — missing `-T / --trafficManagerPort` argument (present in `arg_utils.py` but not in `ecav.py`'s inline parser).
7. **`sim_api.py`** — never calls `Server_SetEdgeMappings`. The C++ server never learns which actors belong to which edge, so `Client_GetConnectionInfo` cannot return correct edge routing, and `Edge_Register` responses cannot populate `vehicle_indices` / `rsu_indices`. Must be called before edge containers start.
8. **`edge_process.py`** — `push_target = "localhost:{port}"` hardcoded. Breaks multi-host deployment.
9. **`edge_process.py`** — missing `-l / --litserve` argument in `arg_parse()`. Required for WorldFusion Mode 4; without it the edge cannot know to call LitServe for feature extraction.
10. **WorldFusion dependencies in Dockerfile** — `ecav/worldfusion/` (opencood) not installed. Edge containers need it for the Where2comm fusion stage that runs locally after LitServe returns spatial features. Feature extraction itself runs on the LitServe server, not in the edge container.
11. **Model weight access** — no strategy for making weights available inside edge containers.

---

## Implementation Plan

### Phase 0: Fix Blocking Infrastructure Bugs

#### 0.1 — `ecav.py`: Replace inline `arg_parse()` with shared `build_arg_parser`

`ecav.py` has an inline `arg_parse()` that duplicates `ecav/ecav2/arg_utils.py:build_arg_parser()` but is missing `-T / --trafficManagerPort`. `opencda.py` already imports and uses `build_arg_parser`. Apply the same change to `ecav.py`:

```python
# Remove inline arg_parse() and replace with:
from ecav.ecav2.arg_utils import build_arg_parser

def arg_parse():
    parser = build_arg_parser("eCAV scenario runner.")
    opt = parser.parse_args()
    return opt
```

All arguments including `-T / --trafficManagerPort` are then available via the shared parser. Propagate before `vehicle_runner()`:
```python
if opt.trafficManagerPort is not None:
    scene_dict.scenario_runner.trafficManagerPort = str(opt.trafficManagerPort)
```

#### 0.2 — `start_actors.sh`: Fix binary paths

- Line 105: `opencda/ml_manager/litserve_models.py` → `ecav/ml_manager/litserve_models.py`
- Lines 203, 307, 356: `opencda.py` → `ecav.py`

#### 0.3 — `sim_api.py`: Call `Server_SetEdgeMappings`

After `Server_StartScenario` in `run_comms()`, send edge-actor mappings so the C++ server can route `Client_GetConnectionInfo` correctly:

```python
async def _send_edge_mappings(self):
    if 'edge_list' not in self.scenario_params.get('scenario', {}):
        return
    request = ecloud.EdgeMappingSetup()
    request.num_edges = len(self.scenario_params['scenario']['edge_list'])
    global_vid, global_rid = 0, 0
    for edge_idx, edge in enumerate(self.scenario_params['scenario']['edge_list']):
        m = request.mappings.add()
        m.edge_index = edge_idx
        for _ in edge.get('vehicles', []):
            m.vehicle_indices.append(global_vid); global_vid += 1
        for _ in edge.get('rsus', []):
            m.rsu_indices.append(global_rid); global_rid += 1
    await self.ecloud_server.Server_SetEdgeMappings(request)
```

This must be called BEFORE edge containers start. The `start_actors.sh` "pushed scenario start" barrier enforces this ordering.

---

### Phase 1: Proxy Class Pattern

#### 1.1 — `VehicleManager` and `RSUManager`: Add `proxy=False`

```python
class VehicleManager:
    def __init__(self, vehicle, config_yaml, application, carla_map,
                 cav_world, current_time, data_dumping, proxy=False):
        self.proxy = proxy
        self.vehicle_index = config_yaml.get('vehicle_index', -1)
        if proxy:
            self.vehicle = vehicle  # CARLA actor reference (may be None)
            self.agent = SimpleNamespace(objects=None, edge_predictions=None)
            self.perception_manager = SimpleNamespace(
                feature_dict=None, _pending_batch=None)
            self.localizer = SimpleNamespace(get_ego_pos=lambda: None)
            self.vehicles_detected = {}
            return
        # existing full initialization unchanged ...

    def update_from_grpc(self, vehicle_update):
        """Populate proxy attributes from a received VehicleUpdate proto."""
        assert self.proxy, "update_from_grpc is only for proxy managers"
        if vehicle_update.pickled_agent_objects:
            self.agent.objects = pickle.loads(vehicle_update.pickled_agent_objects)
        if vehicle_update.pickled_features:
            self.perception_manager.feature_dict = msgpack.unpackb(
                vehicle_update.pickled_features, raw=False)
        self.transform = vehicle_update.transform
        self.velocity = vehicle_update.velocity
```

Apply same pattern to `RSUManager`.

#### 1.2 — `EdgeManagerBase`: Add `proxy=False`

```python
class EdgeManagerBase:
    def __init__(self, config, carla_world, edge_dt, world_dt, proxy=False):
        self.proxy = proxy
        self.edge_index = config.get('edge_index', 0)
        self.edge_ip = config.get('edge_ip', 'localhost')
        self.edge_port = config.get('edge_port', 50054)
        self.vehicle_manager_list = []
        self.rsu_manager_list = []
        self.profiler = EdgeProfiler()  # keep for evaluation
        if proxy:
            return
        # existing initialization...

    def run_step(self, step):
        if self.proxy:
            return None  # fusion happens in edge container
        # existing logic...

    def update_information(self, step):
        if self.proxy:
            return
        # existing logic...
```

Any method that calls `self.carla_world.get_map()` must assert first:
```python
assert not self.proxy, \
    "get_map() must not be called in distributed edge mode — " \
    "this path belongs to legacy route-planning code, not fusion"
```

Do NOT guard with `if self.carla_world is not None`. Assert and fail loud. These call sites are from legacy route-planning code paths that should not execute in Mode 4; a silent guard would mask incorrect execution, not prevent it.
```

#### 1.3 — `sim_api.py` and scenario files: Use proxy managers in Mode 4

Add factory function to `sim_api.py`:
```python
def create_proxy_edge_managers(scenario_params, cav_world):
    """Create lightweight proxy EdgeManagers for distributed Mode 4."""
    edge_list = []
    for edge_cfg in scenario_params['scenario'].get('edge_list', []):
        em = EdgeManagerBase(edge_cfg, carla_world=None,
                             edge_dt=edge_cfg.get('edge_dt', 0.2),
                             world_dt=0.05, proxy=True)
        # Populate proxy vehicle/RSU managers (no CARLA, no sensors)
        for vm_cfg in edge_cfg.get('vehicles', []):
            vm = VehicleManager(vehicle=None, config_yaml=vm_cfg, ..., proxy=True)
            em.vehicle_manager_list.append(vm)
        for rm_cfg in edge_cfg.get('rsus', []):
            rm = RSUManager(rsu=None, config_yaml=rm_cfg, ..., proxy=True)
            em.rsu_manager_list.append(rm)
        edge_list.append(em)
    return edge_list
```

Helper to detect Mode 4 deployment:
```python
def has_standalone_edges(scenario_params):
    """True when edge_list entries define edge_port (standalone edge containers)."""
    edges = scenario_params.get('scenario', {}).get('edge_list', [])
    return bool(edges) and 'edge_port' in edges[0]
```

In `openscenario_3_edge.py` and `openscenario_3_edge_worldfusion.py`:
```python
if opt.distributed and has_standalone_edges(scenario_params):
    edge_list = sim_api.create_proxy_edge_managers(scenario_params, cav_world)
    # Main loop: TICK only, no edge.run_step().
    # In Mode 4 actors receive fused predictions directly from the edge via
    # Edge_ActorSendUpdate response — no separate PULL_OBJECTS_AND_TICK cycle needed.
    while flag:
        flag = scenario_manager.broadcast_message(ecloud.Command.TICK)
        scenario_manager.tick_world()
        step += 1
else:
    # Sequential or Mode 2: existing full EdgeManager path
    edge_list = scenario_manager.create_edge_manager_from_scenario_runner(...)
    while flag:
        # existing loop with edge.run_step() ...
```

#### 1.4 — `openscenario_3_edge.py`: Pass `litserve` to CavWorld

```python
cav_world = CavWorld(
    apply_ml=opt.apply_ml,
    config=scenario_params,
    litserve=getattr(opt, 'litserve', False)
)
```

---

### Phase 2: Edge Process — Wire Actual EdgeManager

This is the central implementation phase. `fuse_predictions()` is replaced with a real EdgeManager.

#### 2.1 — `edge_process.py`: Add EdgeManager initialization

After `Edge_Register` response is received:

```python
def _init_edge_manager(self, config):
    import yaml
    from ecav.scenario_testing.utils.sim_api import _select_edge_manager
    from ecav.core.common.cav_world import CavWorld

    scenario_dict = yaml.safe_load(config.edge_config_yaml)
    edge_cfg = scenario_dict['scenario']['edge_list'][self.edge_index]
    manager_type = edge_cfg.get('manager_type', 'late_fusion')

    cav_world = CavWorld(
        apply_ml=True,
        litserve=self.opt.litserve,
        config=OmegaConf.create(scenario_dict)
    )
    world_dt = scenario_dict.get('world', {}).get('fixed_delta_seconds', 0.05)
    edge_dt = edge_cfg.get('edge_dt', 0.2)

    # EdgeManager runs fully here; carla_world=None (no CARLA in edge container)
    self.edge_manager = _select_edge_manager(manager_type)(
        edge_cfg, carla_world=None, edge_dt=edge_dt, world_dt=world_dt
    )
    self.manager_type = manager_type

    # Initialize proxy managers for all assigned actors
    for vid in config.vehicle_indices:
        key = f"{ecloud.ActorType.VEHICLE}_{vid}"
        if key not in self.actors:
            self.actors[key] = EdgeActorInfo(
                vehicle_index=vid, actor_id=-1, vid=-1,
                actor_type=ecloud.ActorType.VEHICLE)

    for rid in config.rsu_indices:
        key = f"{ecloud.ActorType.RSU}_{rid}"
        if key not in self.actors:
            self.actors[key] = EdgeActorInfo(
                vehicle_index=rid, actor_id=-1, vid=-1,
                actor_type=ecloud.ActorType.RSU)
```

#### 2.2 — `edge_process.py`: Replace `fuse_predictions()` with `run_edge_step()`

```python
def run_edge_step(self, tick_id):
    """Run one EdgeManager step from gRPC-received actor data."""
    # 1. Populate proxy managers from actor updates
    for key, actor in self.actors.items():
        if actor.last_update is None:
            continue
        pm = self._get_proxy_manager_for_actor(actor)
        if pm is not None:
            pm.update_from_grpc(actor.last_update)

    # 2. WorldFusion: batched LitServe call before fusion.
    # LitServe is mandatory for WorldFusion in Mode 4; assert rather than branch.
    if self.manager_type == 'worldfusion':
        assert self.opt.litserve, \
            "WorldFusion distributed edge requires LitServe (-l). Local feature extraction not supported."
        self._extract_features_batched()

    # 3. Run EdgeManager (fusion + tracking + prediction)
    predictions = self.edge_manager.run_step(tick_id)

    # 4. Store per-actor predictions for return on next tick
    if predictions:
        for vehicle_index, pred in predictions.items():
            self.fused_predictions[vehicle_index] = pickle.dumps(pred)
```

Call `run_edge_step(tick_id)` from `process_tick()` in place of `fuse_predictions()`.

#### 2.3 — `edge_process.py`: WorldFusion batched LitServe call

```python
def _extract_features_batched(self):
    """
    Merge all actors' pre-LitServe sensor batches, call LitServe once (batch=N),
    split spatial_features back per actor.
    """
    import msgpack, msgpack_numpy as m
    m.patch()

    ordered = sorted(self.actors.values(), key=lambda a: a.vehicle_index)
    batches = []
    proxy_managers = []
    for actor in ordered:
        if actor.last_update and actor.last_update.pickled_features:
            batch = msgpack.unpackb(actor.last_update.pickled_features, raw=False)
            batches.append(batch)
            proxy_managers.append(self._get_proxy_manager_for_actor(actor))

    if not batches:
        return

    merged = _merge_sensor_batches(batches)  # reindex voxel batch_idx, stack cameras
    body = msgpack.packb(_tensors_to_numpy(merged), use_bin_type=True)
    response = self._session.post(
        self.worldfusion_endpoint + '/extract_features',
        data=body,
        headers={'Content-Type': 'application/octet-stream'},
    )
    result = msgpack.unpackb(response.content, raw=False)
    spatial = result['spatial_features']  # (N, C, H, W) float16 numpy

    for i, pm in enumerate(proxy_managers):
        pm.perception_manager.feature_dict = {
            'spatial_features': torch.from_numpy(
                spatial[i:i+1].copy()).float()
        }
```

`_merge_sensor_batches` implements the O5 voxel batch merging from `worldfusion_litserve_plan.md`: stack camera tensors on dim 0, concatenate voxel arrays while setting `voxel_coords[:, 0] = agent_index`, update `record_len = [1] * N`.

#### 2.4 — `edge_process.py`: Fix actor push IP (multi-host prep)

```python
# In Edge_ActorRegister handler — replace:
push_target = f"localhost:{request.vehicle_port}"
# With:
actor_ip = getattr(request, 'vehicle_ip', '') or 'localhost'
push_target = f"{actor_ip}:{request.vehicle_port}"
```

In `ecloud_actor_client.py`, add to the `Edge_ActorRegister` request:
```python
request.vehicle_ip = VEHICLE_IP  # from cloud_config.yaml
```

---

### Phase 3: Actor Containers — WorldFusion Mode Change

#### 3.1 — `WorldFusionPerceptionManager`: `distributed_edge_mode` flag

Add to `__init__`:
```python
self.distributed_edge_mode = (
    getattr(cav_world, 'distributed_edge_mode', False)
)
```

`CavWorld` sets `distributed_edge_mode = distributed and has_standalone_edges(config)`.

In `run_step()`:
```python
batch = self._build_batch()  # CPU only: always runs

if self.distributed_edge_mode:
    # Mode 4: store batch; edge calls LitServe
    self._pending_batch = batch
    self.feature_dict = None
elif self.use_litserve:
    # Sequential/-l without standalone edge: actor calls LitServe
    features, timing = self._extract_features_remote(batch)
    self.feature_dict = features
    self._pending_batch = None
else:
    # Local inference
    features = self._extract_features_local(batch)
    self.feature_dict = features
    self._pending_batch = None
```

#### 3.2 — `ecloud_actor_client.py`: Pack `_pending_batch` for WorldFusion Mode 4

```python
pm = self.vehicle_manager.perception_manager
if hasattr(pm, '_pending_batch') and pm._pending_batch is not None:
    # WorldFusion Mode 4: pre-LitServe sensor data
    vehicle_update.pickled_features = msgpack.packb(
        _tensors_to_numpy(pm._pending_batch), use_bin_type=True)
elif hasattr(pm, 'feature_dict') and pm.feature_dict is not None:
    # Sequential/Mode 2 WorldFusion: post-LitServe features
    feat = {k: v.half().cpu().numpy() for k, v in pm.feature_dict.items()}
    vehicle_update.pickled_features = msgpack.packb(feat, use_bin_type=True)
```

#### 3.3 — Ego vehicle containers: Single-process entrypoint

No separate actor client process is needed for ego vehicles. `load_scenario()` in `scenario_runner/srunner/scenariomanager/scenario_manager.py` already spawns `Ecav2ActorClient` in-process when `ecav_vehicle_index >= 0`:

```python
if ecav_vehicle_index >= 0:
    self._ecav_client = Ecav2ActorClient(vehicle=self.ego_vehicles[0],
                                          vehicle_index=ecav_vehicle_index)
    asyncio.get_event_loop().run_until_complete(self._ecav_client.run())
```

`Ecav2ActorClient` receives the CARLA actor reference directly at construction — no hero vehicle lookup by role_name is needed. Ego containers therefore run a single command via `start_actors.sh`:

```bash
docker run $gpu_flag -d \
    --network=host \
    --name="$container_name" \
    -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
    ecav-python310:latest \
    python3.10 ecav.py -d -i $i -t "$scenario_name" $ml_flag $litserve_flag -T $((8000 + i))
```

RSU containers remain standalone `ecloud_actor_client.py` processes — ScenarioRunner has no concept of RSUs.

**Multi-ego vehicle naming**: The `hero_0`/`hero_1`/`hero_N` convention applies to the XML `role_name` used by ScenarioRunner to spawn vehicles in CARLA. Each ego container runs its own ScenarioRunner with its vehicle index, and each spawns its vehicle with `role_name=hero_N`. No actor client lookup by role_name is required since `Ecav2ActorClient` receives the CARLA actor reference directly from ScenarioRunner.

---

### Phase 4: Scenario Files and YAML Updates

#### 4.1 — YAMLs: Add `edge_index`, `edge_port`, `edge_ip`

Both `openscenario_3_edge.yaml` and `openscenario_3_edge_worldfusion.yaml`:

```yaml
scenario:
  edge_list:
    - <<: *edge_base
      edge_index: 0
      edge_port: 50054
      edge_ip: localhost    # override in cloud_config.yaml for multi-host
      manager_type: worldfusion  # or late_fusion
      ...
```

The `edge_port` field is what `has_standalone_edges()` checks to distinguish Mode 4 from sequential/Mode 2.

#### 4.2 — `openscenario_3_edge_worldfusion.yaml`: Remove `distributed: false`

Remove line 11. The `-d` CLI flag is the sole determinant.

#### 4.3 — `sim_api.py`: Add single-edge assertion

```python
assert len(scenario_params['scenario']['edge_list']) == 1, \
    "Multiple edges not yet supported. See distributed_actor_plan.md."
```

---

### Phase 5: Dockerfile for Edge Containers

#### 5.1 — Add WorldFusion (opencood) dependencies

```dockerfile
# Verify worldfusion submodule is initialized
RUN test -d ecav/worldfusion/opencood || \
    (echo "ERROR: ecav/worldfusion submodule missing. Run: git submodule update --init ecav/worldfusion" && exit 1)

RUN pip install -e ecav/worldfusion/ --no-deps 2>/dev/null || true
RUN pip install -r ecav/worldfusion/requirements.txt 2>/dev/null || true
```

Prerequisite check in `start_actors.sh` before `docker build`:

```bash
if [[ ! -f ecav/worldfusion/opencood/__init__.py ]]; then
    echo "ERROR: WorldFusion submodule not initialized."
    echo "Run: git submodule update --init ecav/worldfusion"
    exit 1
fi
```

#### 5.2 — CUDA extensions for edge containers

Edge containers require `iou3d_nms`, `roiaware_pool3d`, and `pointnet2` for the Where2comm fusion stage (post-feature-extraction). These are needed regardless of LitServe — feature extraction goes to LitServe, but the fusion itself runs locally on the edge container.

```dockerfile
RUN cd ecav/worldfusion && \
    python setup.py build_ext --inplace 2>&1 | tail -10
```

**Note**: WorldFusion Mode 4 does not support local feature extraction on edge containers. The CUDA extensions here cover Where2comm fusion only, not the sensor encoder. Feature extraction always goes to LitServe (`-l` mandatory).

Actor containers running WorldFusion Mode 4 do NOT need CUDA extensions — they only run voxelization and camera resize (CPU).

#### 5.3 — Model weight volumes for edge containers

Mount in `start_actors.sh` edge container launch:

```bash
docker run $gpu_flag -d \
    --network=host \
    --name="$container_name" \
    -v $(pwd)/ecav/worldfusion/opencood/logs:/app/ecav/worldfusion/opencood/logs:ro \
    -v $(pwd)/models:/app/models:ro \
    ecav-python310:latest \
    python3.10 ecav/ecav2/edge_process.py -e $e -P $edge_port $litserve_flag
```

---

### Phase 6: `start_actors.sh` Updates (consolidated)

All changes to `start_actors.sh`:

| Change | Location | Phase |
|--------|----------|-------|
| `opencda.py` → `ecav.py` | Lines 203, 307, 356 | 0.2 |
| `opencda/ml_manager/litserve_models.py` → `ecav/ml_manager/litserve_models.py` | Line 105 | 0.2 |
| Fix LitServe readiness check (port probe) | Lines 113–116 | 0.2 |
| Add worldfusion submodule check | Before docker build | 5.1 |
| Add model weight volume mounts to edge containers | Edge container launch | 5.3 |
| Update ego container launch to use `ego_vehicle_entrypoint.sh` | Ego container launch | 3.3 |

---

### Phase 7: `ecloud_server.cc` Verification

#### 7.1 — Verify `Server_SetEdgeMappings` → `Edge_Register` response chain

The C++ `Edge_Register` handler must populate `EdgeScenarioConfig.vehicle_indices` and `rsu_indices` from the mappings set by `Server_SetEdgeMappings`. If `Server_SetEdgeMappings` is called after edge containers start (and thus after `Edge_Register` is already received), the mappings arrive too late.

**Enforced ordering**: `start_actors.sh` already waits for "pushed scenario start" before launching edge containers. `Server_SetEdgeMappings` must be called inside `run_comms()` (Phase 0.3), which completes before the "pushed scenario start" log line is emitted. This ordering is correct — verify it holds.

#### 7.2 — Verify `Client_GetConnectionInfo` routing

When an actor calls `Client_GetConnectionInfo(vehicle_index=i)`, the C++ server must:
1. Look up `i` in `vehicleToEdgeMapping_` / `rsuToEdgeMapping_`
2. Return the matching edge's IP/port from `edgeClients_[edge_idx]`
3. Return `has_edge=false` if actor is not in any edge mapping (edge-less scenarios)

Trace and confirm this logic in the C++ handler.

#### 7.3 — Verify `Edge_PushTick` vs `PushTick` dispatch

The C++ server pushes to edges using `PushClient`. Confirm it calls `PushTick` (not `Edge_PushTick`) on the edge's gRPC port. `edge_process.py:EdgeServer` handles both via `_handle_tick()` dispatch, so either works — but the message type differs (`Tick` vs `EdgeTick`). Verify the C++ server sends a `Tick` message (not `EdgeTick`) to avoid a proto mismatch.

---

### Phase 8: Integration Testing Protocol

#### Test 8.1 — Sequential mode, both fusion types (regression)

```bash
python ecav.py -t openscenario_3_edge --apply_ml
python ecav.py -t openscenario_3_edge_worldfusion --apply_ml -l
```

Must complete without errors. This is the smoke test that confirms Phase 1 proxy changes did not break sequential mode.

#### Test 8.2 — Mode 4, LateFusion, no containers

Run all processes on host (no Docker) to validate gRPC wiring:

```bash
# T1: Orchestrator (Mode 4 detected via edge_port in YAML)
python ecav.py -t openscenario_3_edge -d --apply_ml

# T2: Edge process
python ecav/ecav2/edge_process.py -e 0 -P 50054 --apply_ml

# T3: Ego ScenarioRunner
python ecav.py -d -i 0 -t openscenario_3_edge

# T4: Ego actor client
python ecav/ecav2/ecloud_actor_client.py --apply_ml -i 0

# T5: RSU
python ecav/ecav2/ecloud_actor_client.py --apply_ml -i 0

# T6: Non-ego
python ecav.py -d -i -1 -t openscenario_3_edge
```

Expected: Edge process receives ticks from C++ server (not orchestrator directly). Actors register with edge. LateFusion EdgeManager runs in edge process. Fused predictions returned to actors.

#### Test 8.3 — Mode 4, WorldFusion, no containers

```bash
# T1: LitServe
python ecav/ml_manager/litserve_models.py

# T2: Orchestrator
python ecav.py -t openscenario_3_edge_worldfusion -d --apply_ml -l

# T3: Edge process (with LitServe endpoint configured)
python ecav/ecav2/edge_process.py -e 0 -P 50054 --apply_ml -l

# T4+T5: Ego vehicle (both processes)
python ecav.py -d -i 0 -t openscenario_3_edge_worldfusion &
python ecav/ecav2/ecloud_actor_client.py --apply_ml -l -i 0

# T6: RSU
python ecav/ecav2/ecloud_actor_client.py --apply_ml -l -i 0

# T7: Non-ego
python ecav.py -d -i -1 -t openscenario_3_edge_worldfusion
```

Expected: Actors send pre-LitServe sensor batches to edge. Edge makes single batched LitServe call. WorldFusion EdgeManager runs in edge process. Timing CSV written at teardown.

#### Test 8.4 — Docker containers, LateFusion

```bash
./start_actors.sh
# Inputs: openscenario_3_edge, 1 ego, 1 RSU, 1 edge, ML: Y, LitServe: Y
```

#### Test 8.5 — Docker containers, WorldFusion

```bash
./start_actors.sh
# Inputs: openscenario_3_edge_worldfusion, 1 ego, 1 RSU, 1 edge, ML: Y, LitServe: Y
```

---

## File Change Summary

| File | Change | Phase |
|------|--------|-------|
| `ecav.py` | Add `-T/--trafficManagerPort` | 0.1 |
| `start_actors.sh` | Fix paths; ego dual-process; edge weight mounts; LitServe probe; worldfusion check | 0.2, 3.3, 5.1, 5.3, 6 |
| `start_actors.sh` ego container launch | Simplify to single `ecav.py -d -i $i` command; remove entrypoint script | 3.3 |
| `ecav/scenario_testing/utils/sim_api.py` | Add `_send_edge_mappings()`; `create_proxy_edge_managers()`; `has_standalone_edges()`; single-edge assertion | 0.3, 1.3, 4.3 |
| `ecav/core/common/vehicle_manager.py` | Add `proxy=False`; add `update_from_grpc()` | 1.1 |
| `ecav/core/sensing/perception/rsu_manager.py` | Add `proxy=False`; add `update_from_grpc()` | 1.1 |
| `ecav/core/application/edge/edge_manager/edge_manager_base.py` | Add `proxy=False`; no-op `run_step`/`update_information` when proxy | 1.2 |
| `ecav/scenario_testing/openscenario_3_edge.py` | Pass `litserve`; add Mode 4 branch | 1.3, 1.4 |
| `ecav/scenario_testing/openscenario_3_edge_worldfusion.py` | Add Mode 4 branch | 1.3 |
| `ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml` | Add `edge_index`, `edge_port`, `edge_ip`; remove `distributed: false` | 4.1, 4.2 |
| `ecav/scenario_testing/config_yaml/openscenario_3_edge.yaml` | Add `edge_index`, `edge_port`, `edge_ip` | 4.1 |
| `ecav/ecav2/edge_process.py` | Replace `fuse_predictions()` with `run_edge_step()`; add EdgeManager init; batched LitServe; fix actor push IP; add `-l/--litserve` to `arg_parse()` | 2.1–2.4 |
| `ecav/ecav2/ecloud_actor_client.py` | Pack `_pending_batch` for WF Mode 4; add `vehicle_ip` to registration | 3.2, 2.4 |
| `ecav/core/sensing/perception/worldfusion_perception_manager.py` | Add `distributed_edge_mode`; `_pending_batch` path in `run_step()` | 3.1 |
| `ecav/core/common/cav_world.py` | Set `distributed_edge_mode` from config | 3.1 |
| `Dockerfile` | Add opencood; CUDA extensions; submodule check | 5.1–5.2 |
| `ecav/ecloud_server/ecloud_server.cc` | Verify edge tick dispatch, `Client_GetConnectionInfo`, `Edge_Register` response | 7.1–7.3 |

---

## Design Decisions (Resolved)

### 1 — Multi-ego `role_name` convention

**Resolved**: Use `hero_0`, `hero_1`, ..., `hero_N` in scenario XML and in `ecloud_actor_client.py`. See Phase 3.3. No separate CARLA actor ID transmission is needed.

### 2 — `Edge_ActorSendUpdate` return type: `ObjectBuffer` vs `Empty`

**Resolved**: Return `ObjectBuffer` containing previous tick's fused predictions.

The design question was whether returning predictions in the `Edge_ActorSendUpdate` response (blocking, combined call) is a significant departure from the original `Client_SendUpdate → Empty` + separate `PULL_OBJECTS_AND_TICK` / `Client_GetObjects` pull cycle.

Analysis: the response is **not blocking on fusion**. The edge returns predictions immediately when it receives `Edge_ActorSendUpdate` — it returns `fused_predictions[vehicle_index]` from the *previous* tick's fusion output (already computed and stored). The actor does not wait for other actors' updates or for the current tick's fusion to complete. Fusion runs after all actors have submitted updates (triggered by `Edge_PushTick`), and its output is stored for the *next* tick's responses.

This is a double-buffering pattern: actors submit tick N data and receive tick N-1 predictions in the same call. Functionally identical to the original `Client_GetObjects` → `Client_SendUpdate` sequence, except both operations are combined into one RPC. The actor-side synchronization model is unchanged; only the number of round trips is reduced (2 → 1 per tick per actor).

The orchestrator in Mode 4 always issues `TICK` (not `PULL_OBJECTS_AND_TICK`), since actors do not call `Client_GetObjects` — they get predictions from the edge directly. The C++ server's dispatch to edge containers is not affected by the command type when `hasEdges_=true`.

### 3 — `get_map()` calls in edge process

**Resolved**: Assert, do not guard. Any call path that reaches `self.carla_world.get_map()` in the edge process belongs to legacy route-planning code that must never execute in distributed mode. Assert `not self.proxy` at the entry of those methods, not `if self.carla_world is not None`. Fail loud; a silent guard masks a wrong code path being taken.

### 4 — Evaluation data in Mode 4

**Resolved**: Assert on unavailable data at runtime; do not proactively sandbox evaluation paths. `EvaluationManager` will hit assertions when it tries to access CARLA-backed data (live camera frames, ground-truth error from CARLA sensors) that is not available through proxy managers. Each assertion hit identifies a concrete metric that needs to be either: (a) computed from gRPC-available proxy data only, or (b) sandboxed for distributed mode. Do this reactively as assertions are hit during testing, not speculatively.

### 5 — `edge_process.py` `-l` / `--litserve` flag

**Resolved**: Add `-l/--litserve` to `edge_process.py:arg_parse()`. Confirmed. Edge must know whether to call LitServe for WorldFusion batched feature extraction. Already reflected in Phase 2 implementation (see `self.opt.litserve` in `run_edge_step()`). Update `start_actors.sh` edge container launch to pass `$litserve_flag`.

---

## Known Behavioral Artifacts

### Behavior Tree / Actor Client Tick Ordering

In the ScenarioRunner tick loop (`scenario_manager.run_scenario()`), `_tick_scenario()` executes before `ecav_client.tick()`:

```python
while self._running:
    self._tick_scenario(timestamp)   # behavior tree: waypoints, agent ego_action
    pong = run_until_complete(self._ecav_client.tick())  # update_info, run_step, send update
```

The behavior tree computes `ego_action` from the previous tick's CARLA world snapshot. `update_info()` inside `ecav_client.tick()` then refreshes state from the current snapshot before `run_step()`. In the sequential (non-distributed) path, both operations happen within the same process tick against the same snapshot, so there is no lag. In the distributed path, the behavior tree and actor client tick are computed from world states that are one snapshot apart.

**Empirical basis**: The current ordering was chosen because it produced behavior equivalent to the sequential path in the scenarios tested (occluded intersection, low relative velocities). The ordering was not derived from first principles — it was validated empirically.

**Conditions where this matters**: High-speed or high-density scenarios where a one-tick stale behavior tree state produces a materially different control decision than a current-state computation. At `fixed_delta_seconds=0.05s`, the window is narrow but not zero.

**Action**: No change warranted. Document as a known artifact. If sequential vs. distributed behavioral divergence is ever observed and cannot be explained by network latency or fusion lag, revisit this ordering as a candidate cause.

---

## Appendix: Azure Deployment Notes

**Azure deployment is not in scope for this plan.** Local co-located testing is the immediate target. When moving to Azure, the following items built into this plan avoid blocking Azure deployment later:

| Item | Status after this plan |
|------|----------------------|
| All IPs sourced from `cloud_config.yaml` | Done (edge uses `EDGE_IP`; actor push IP fix in Phase 2.4) |
| Actor IP sent in gRPC registration | Done in Phase 2.4 |
| Edge IP sent in `EdgeRegistrationInfo` | Already done |
| LitServe endpoint configurable per YAML | Already done |
| Model weights via volume mount | Done in Phase 5.3 |
| Port conventions documented | Done in this plan |

**Azure-specific NSG rules required**:

| Traffic | Port | Purpose |
|---------|------|---------|
| Any → Orchestrator | 50051 TCP | C++ ecloud_server |
| Orchestrator → Actor nodes | 50101–50200 TCP | Actor push servers |
| Orchestrator → Edge nodes | 50054–50063 TCP | Edge tick push |
| Actor nodes → Edge nodes | 50054–50063 TCP | Actor registration + updates |
| Edge nodes → LitServe node | 18000 TCP | WorldFusion HTTP |
| Actor nodes → LitServe node | 18001 TCP | LateFusion YOLO gRPC |
| Any → CARLA node | 2000–2002 TCP | CARLA client API |

Azure deployment scripts (ACI, AKS, or SSH-based orchestration) are a separate artifact to be developed once local distributed testing is validated.
