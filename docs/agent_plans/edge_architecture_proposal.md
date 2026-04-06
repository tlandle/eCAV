# Edge Standalone Process Architecture Proposal

## Executive Summary

This document proposes architectural changes to make the edge a standalone process that orchestrates its own vehicles and RSUs, while maintaining compatibility with edge-less scenarios.

---

## Current Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    CURRENT FLOW                                  │
└─────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────┐
                    │   Orchestrator      │
                    │   (sim_api.py +     │
                    │    ecloud_server.cc)│
                    └──────────┬──────────┘
                               │
        ┌──────────────────────┼──────────────────────┐
        │                      │                      │
        v                      v                      v
   ┌─────────┐           ┌─────────┐           ┌─────────┐
   │Vehicle 0│           │Vehicle 1│           │  RSU 0  │
   └────┬────┘           └────┬────┘           └────┬────┘
        │                      │                      │
        └──────────────────────┼──────────────────────┘
                               │
                               v
                    ┌─────────────────────┐
                    │   EdgeManager       │
                    │   (co-located with  │
                    │    orchestrator)    │
                    └─────────────────────┘

Problems:
1. EdgeManager runs inside orchestrator process (sim_api.py)
2. All vehicles/RSUs communicate directly with orchestrator
3. Edge processing is a function call, not a distributed service
4. No support for multiple edges as independent processes
```

---

## Proposed Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    NEW FLOW                                      │
└─────────────────────────────────────────────────────────────────┘

                    ┌─────────────────────┐
                    │   Orchestrator      │
                    │   (ecloud_server.cc)│
                    │                     │
                    │ - Tick coordination │
                    │ - Edge registration │
                    │ - Scenario config   │
                    └──────────┬──────────┘
                               │
            ┌──────────────────┴──────────────────┐
            │ Edge_TickComplete / Edge_GetTick    │
            │                                     │
            v                                     v
     ┌─────────────┐                       ┌─────────────┐
     │   Edge 0    │                       │   Edge 1    │
     │  (process)  │                       │  (process)  │
     │             │                       │             │
     │- Fusion     │                       │- Fusion     │
     │- Prediction │                       │- Prediction │
     └──────┬──────┘                       └──────┬──────┘
            │                                     │
     ┌──────┴──────┐                       ┌──────┴──────┐
     │             │                       │             │
     v             v                       v             v
┌─────────┐  ┌─────────┐             ┌─────────┐  ┌─────────┐
│Vehicle 0│  │  RSU 0  │             │Vehicle 2│  │  RSU 1  │
└─────────┘  └─────────┘             └─────────┘  └─────────┘
│Vehicle 1│
└─────────┘

Key Changes:
1. Edge becomes a standalone process with its own gRPC server
2. Vehicles/RSUs owned by an edge communicate with THAT edge
3. Each edge has a client connection to the orchestrator
4. Orchestrator only coordinates edges (not individual vehicles)
5. Edge-less scenarios: vehicles communicate directly with orchestrator (unchanged)
```

---

## Detailed Task List

### Phase 0: Pre-requisite Cleanup (Terminology & YAML) — ⚠️ PARTIAL

- 0.1 `--litserve` flag: ✅ implemented (`ecav/arg_utils.py`, wired through to `ecloud_actor_client.py` → `CavWorld`)
- 0.2 Wire through to MLManager: ✅ done
- 0.3 Remove `distributed:` from YAMLs: ❌ still present in ~75 files (all `false`; `sim_api.py` overwrites with CLI flag so functionally inert)
- 0.4 Remove `scenario_params['distributed']` reads: ❌ still in ~31 scenario `.py` files (reads the CLI-overwritten value, so correct but should be cleaned up)

This phase cleans up the naming confusion and removes the `distributed` field from YAML files.

#### 0.1 Add `--litserve` Command-Line Argument

**File: `/home/jordan/eCAV/ecav.py`**

```python
parser.add_argument('-l', "--litserve", action='store_true',
                    help='Use LitServe for distributed ML inference (requires LitServe server on port 18000)')
```

**File: `/home/jordan/eCAV/ecav/ecav2/ecloud_actor_client.py`**

```python
# In arg_parse():
parser.add_argument('-l', "--litserve", action='store_true',
                    help='Use LitServe for distributed ML inference')

# Pass to VehicleManager/CavWorld:
# self.opt.litserve → run_distributed parameter in MLManager
```

#### 0.2 Wire `--litserve` Through to MLManager

**File: `/home/jordan/eCAV/ecav/core/common/cav_world.py`**

```python
def __init__(self, apply_ml=False, litserve=False):
    # ...
    if apply_ml:
        self.ml_manager = MLManager(apply_ml=apply_ml, run_distributed=litserve)
```

**File: `/home/jordan/eCAV/ecav/scenario_testing/utils/sim_api.py`**

```python
# In ScenarioManager.__init__:
self.litserve = litserve  # New parameter

# Pass to CavWorld:
self.cav_world = CavWorld(self.apply_ml, litserve=self.litserve)
```

#### 0.3 Remove `distributed` from YAML Files

Remove the `distributed: true/false` line from all YAML files:

```bash
# These files contain 'distributed:' that should be removed:
ecav/scenario_testing/config_yaml/ecloud_edge_scenario.yaml
ecav/scenario_testing/config_yaml/ecloud_edge_4_car.yaml
ecav/scenario_testing/config_yaml/ecloud_edge_8_car.yaml
ecav/scenario_testing/config_yaml/ecloud_edge_16_car.yaml
ecav/scenario_testing/config_yaml/ecloud_4lane_*.yaml
ecav/scenario_testing/config_yaml/openscenario_*.yaml
# ... and others
```

#### 0.4 Update Scenario Scripts to Use `opt.distributed`

**Files: `/home/jordan/eCAV/ecav/scenario_testing/*.py`**

Change from:
```python
run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else False
```

To:
```python
run_distributed = opt.distributed  # From command-line -d flag
```

This affects ~30 scenario test files.

---

### Phase 1: Protocol Buffer Updates (`ecloud.proto`) — ✅ COMPLETE

**File: `/home/jordan/eCAV/ecav/protos/ecloud.proto`**

#### 1.1 New Message Types

```protobuf
// Edge registration with orchestrator
message EdgeRegistrationInfo {
  int32 edge_index = 1;
  string edge_ip = 2;
  int32 edge_port = 3;
  int32 num_vehicles = 4;
  int32 num_rsus = 5;
  string container_name = 6;
}

// Edge scenario configuration (returned to edge on registration)
message EdgeScenarioConfig {
  int32 edge_index = 1;
  bytes edge_config_yaml = 2;  // JSON of edge-specific YAML section
  int32 num_vehicles = 3;
  int32 num_rsus = 4;
  repeated int32 vehicle_indices = 5;  // Global vehicle indices owned by this edge
  repeated int32 rsu_indices = 6;      // Global RSU indices owned by this edge
}

// Edge tick completion notification
message EdgeTickComplete {
  int32 edge_index = 1;
  int32 tick_id = 2;
  bytes fused_predictions = 3;  // Optional: edge can send its fused data
}

// Tick notification to edge
message EdgeTick {
  int32 tick_id = 1;
  Command command = 2;
}

// Actor (vehicle/RSU) connection info - tells actor where to connect
message ActorConnectionInfo {
  bool has_edge = 1;
  string edge_ip = 2;
  int32 edge_port = 3;
  int32 edge_index = 4;
  // If no edge, actor connects to orchestrator (existing behavior)
}
```

#### 1.2 New RPC Methods

```protobuf
service Ecloud {
  // Existing RPCs (keep all)...

  // === NEW: Edge-to-Orchestrator RPCs ===

  // Edge registers with orchestrator, gets its scenario config
  rpc Edge_Register(EdgeRegistrationInfo) returns (EdgeScenarioConfig);

  // Edge notifies orchestrator it completed processing for a tick
  rpc Edge_TickComplete(EdgeTickComplete) returns (Empty);

  // Edge fetches the current tick (called after being notified)
  rpc Edge_GetTick(EdgeIndex) returns (EdgeTick);

  // === NEW: Actor-to-Edge RPCs (mirror existing Actor-to-Orchestrator) ===

  // Actor sends update to edge, receives previous tick's fused data
  rpc Edge_ActorSendUpdate(VehicleUpdate) returns (ObjectBuffer);

  // Actor registers with edge
  rpc Edge_ActorRegister(RegistrationInfo) returns (SimulationInfo);

  // === MODIFIED: Orchestrator tells actor where to connect ===

  // Modify Client_RegisterVehicle response OR add new RPC
  // Actor first connects to orchestrator, which tells it if there's an edge
  rpc Client_GetConnectionInfo(RegistrationInfo) returns (ActorConnectionInfo);
}

message EdgeIndex {
  int32 edge_index = 1;
}
```

---

### Phase 2: Scenario YAML Schema Updates — ✅ NOT NEEDED

`edge_index` and `edge_port` are dynamically assigned at runtime (sequential in `start_actors.sh`), not declared in YAML. This is the actual implementation and is a better design (execution-time properties belong in the shell script, not scenario files).

**Files: All scenario YAML files with edges**

#### 2.1 Add Edge Identification

```yaml
scenario:
  edge_list:
    - <<: *edge_base
      edge_index: 0  # NEW: explicit edge index
      edge_port: 50151  # NEW: port for this edge's gRPC server
      vehicles:
        - <<: *vehicle_base
          # vehicles inherit edge_index from parent
      rsus:
        - <<: *rsu_base
          # RSUs inherit edge_index from parent

    - <<: *edge_base
      edge_index: 1
      edge_port: 50152
      vehicles:
        # ...
```

#### 2.2 Add Edge Connection Info to Actor Configs (Optional)

Could be derived at runtime from YAML structure, but explicit is clearer:

```yaml
# Alternative: add to each actor explicitly
vehicles:
  - <<: *vehicle_base
    edge_index: 0  # Which edge owns this vehicle
```

---

### Phase 3: C++ Orchestrator Updates (`ecloud_server.cc`) — ✅ COMPLETE

All RPCs implemented: `Edge_Register`, `Edge_TickComplete`, `Client_GetConnectionInfo`, `Server_SetEdgeMappings`. Edge state tracking (`edgeInfos_`, `vehicleToEdgeMapping_`, `rsuToEdgeMapping_`, `numRegisteredEdges_`, `numCompletedEdges_`, `hasEdges_`) all present. Tick barrier correctly switches between edge-based and vehicle-based barrier depending on `hasEdges_`.

**File: `/home/jordan/eCAV/ecav/ecloud_server/ecloud_server.cc`**

#### 3.1 New State Variables

```cpp
// Edge tracking
std::atomic<int> numEdges_{0};
std::atomic<int> numCompletedEdges_{0};
std::map<int, EdgeInfo> edgeClients_;  // edge_index -> connection info
std::map<int, int> vehicleToEdge_;     // vehicle_index -> edge_index
std::map<int, int> rsuToEdge_;         // rsu_index -> edge_index

struct EdgeInfo {
  std::string ip;
  int port;
  int numVehicles;
  int numRsus;
  std::vector<int> vehicleIndices;
  std::vector<int> rsuIndices;
  std::unique_ptr<PushClient> pushClient;  // For pushing ticks to edge
};
```

#### 3.2 New RPC Implementations

```cpp
// Edge_Register: Edge registers and receives its configuration
ServerUnaryReactor* Edge_Register(CallbackServerContext* context,
                                  const EdgeRegistrationInfo* request,
                                  EdgeScenarioConfig* reply) override {
  // 1. Store edge connection info
  // 2. Parse scenario YAML to find this edge's vehicles/RSUs
  // 3. Compute global vehicle indices for this edge
  // 4. Return edge-specific config
}

// Edge_TickComplete: Edge signals it finished processing
ServerUnaryReactor* Edge_TickComplete(CallbackServerContext* context,
                                      const EdgeTickComplete* request,
                                      Empty* reply) override {
  // 1. Increment numCompletedEdges_
  // 2. If all edges complete: signal sim_api via PushTick
}

// Client_GetConnectionInfo: Actor asks where to connect
ServerUnaryReactor* Client_GetConnectionInfo(CallbackServerContext* context,
                                             const RegistrationInfo* request,
                                             ActorConnectionInfo* reply) override {
  // 1. Look up vehicle/RSU index in vehicleToEdge_ or rsuToEdge_
  // 2. If found: return edge IP/port
  // 3. If not found (edge-less): return has_edge=false
}
```

#### 3.3 Modified Tick Logic

```cpp
// In Server_DoTick: change from pushing to vehicles to pushing to edges
void broadcastTickToEdges(int tick_id, Command command) {
  for (auto& [edge_idx, edge_info] : edgeClients_) {
    // Async push tick to edge
    edge_info.pushClient->PushTick(tick_id, command);
  }
}

// Barrier changes: wait for edges instead of vehicles
// numCompletedEdges_ == numEdges_ triggers next tick
```

#### 3.4 Edge-less Fallback

```cpp
// If no edges registered, fall back to existing behavior
// (vehicles communicate directly with orchestrator)
bool hasEdges() { return numEdges_.load() > 0; }
```

---

### Phase 4: Edge Standalone Process — ✅ COMPLETE

`ecav/ecav2/edge_process.py` exists and is fully implemented: `EdgeProcess` (registration, tick loop, actor coordination, push-to-actors, report-to-orchestrator) and `EdgeServer` (gRPC servicer for actor `Edge_ActorRegister` and `Edge_ActorSendUpdate`). `fuse_predictions()` is a placeholder — it passes each actor's `pickled_agent_objects` back unchanged. Implementing WorldFusion O5 (`_run_batched_encoder()`) is what replaces this placeholder.

**File: `/home/jordan/eCAV/ecav/ecav2/edge_process.py`** (exists)

#### 4.1 Edge Process Main Components

```python
class EdgeProcess:
    """
    Standalone edge process that:
    1. Registers with orchestrator
    2. Runs its own gRPC server for actors
    3. Performs fusion/prediction
    4. Reports completion to orchestrator
    """

    def __init__(self, edge_index: int, orchestrator_address: str):
        self.edge_index = edge_index
        self.orchestrator_address = orchestrator_address

        # gRPC connections
        self.orchestrator_stub = None  # Client to orchestrator
        self.actor_server = None       # Server for actors

        # State
        self.edge_config = None
        self.vehicle_managers = {}
        self.rsu_managers = {}
        self.current_tick = 0
        self.actor_updates = {}  # tick -> {actor_idx: update}
        self.fused_predictions = {}  # tick -> predictions

        # EdgeManager for fusion/prediction
        self.edge_manager = None

    async def run(self):
        # 1. Connect to orchestrator
        await self.register_with_orchestrator()

        # 2. Start actor gRPC server
        await self.start_actor_server()

        # 3. Main loop: wait for tick, process, report complete
        await self.tick_loop()

    async def register_with_orchestrator(self):
        """Register and get scenario config"""
        request = EdgeRegistrationInfo(
            edge_index=self.edge_index,
            edge_ip=self.my_ip,
            edge_port=self.my_port,
            # ...
        )
        self.edge_config = await self.orchestrator_stub.Edge_Register(request)

        # Initialize EdgeManager with config
        self.edge_manager = EdgeManager(self.edge_config, ...)

    async def tick_loop(self):
        """Main processing loop"""
        while True:
            # Wait for tick from orchestrator
            tick = await self.wait_for_tick()

            if tick.command == Command.END:
                break

            # Wait for all actors to send updates
            await self.wait_for_actor_updates(tick.tick_id)

            # Run fusion/prediction
            predictions = self.edge_manager.run_step(tick.tick_id)

            # Store predictions for actors to pull
            self.fused_predictions[tick.tick_id] = predictions

            # Notify orchestrator we're done
            await self.notify_tick_complete(tick.tick_id)
```

#### 4.2 Edge gRPC Server (for actors)

```python
class EdgeActorService(ecloud_rpc.EcloudServicer):
    """gRPC service that actors connect to"""

    def __init__(self, edge_process: EdgeProcess):
        self.edge = edge_process

    async def Edge_ActorRegister(self, request, context):
        """Actor registers with this edge"""
        # Validate actor belongs to this edge
        # Return actor's config
        pass

    async def Edge_ActorSendUpdate(self, request, context):
        """Actor sends update, receives previous tick's fused data"""
        actor_idx = request.vehicle_index
        tick_id = request.tick_id

        # Store the update
        self.edge.actor_updates[tick_id][actor_idx] = request

        # Return PREVIOUS tick's fused predictions
        prev_tick = tick_id - 1
        if prev_tick in self.edge.fused_predictions:
            return self.edge.fused_predictions[prev_tick]
        else:
            return ObjectBuffer()  # Empty for first tick
```

---

### Phase 5: Actor Client Updates (`ecloud_actor_client.py`) — ✅ COMPLETE

`Client_GetConnectionInfo` RPC call implemented with graceful fallback. Edge-vs-orchestrator routing fully implemented: creates separate `edge_channel`/`edge_stub`, calls `Edge_ActorRegister`, routes tick updates through `Edge_ActorSendUpdate` when `connected_to_edge=True`. Fused predictions received in `Edge_ActorSendUpdate` response and applied to `vehicle_manager.agent.edge_predictions`.

**File: `/home/jordan/eCAV/ecav/ecav2/ecloud_actor_client.py`**

#### 5.1 Connection Discovery

```python
class Ecav2ActorClient:
    async def connect(self):
        # Step 1: Ask orchestrator where to connect
        conn_info = await self.get_connection_info()

        if conn_info.has_edge:
            # Connect to edge instead of orchestrator
            self.target_address = f"{conn_info.edge_ip}:{conn_info.edge_port}"
            self.connected_to_edge = True
        else:
            # Edge-less scenario: use orchestrator directly
            self.target_address = ECLOUD_SERVER_ADDRESS
            self.connected_to_edge = False

        # Step 2: Register with target (edge or orchestrator)
        await self.register()

    async def get_connection_info(self):
        """Ask orchestrator where this actor should connect"""
        # Initial connection is always to orchestrator
        channel = grpc.aio.insecure_channel(ECLOUD_SERVER_ADDRESS)
        stub = ecloud_rpc.EcloudStub(channel)

        request = RegistrationInfo(
            vehicle_index=self.vehicle_index,
            actor_type=self.actor_type,
            # ...
        )
        return await stub.Client_GetConnectionInfo(request)
```

#### 5.2 Modified Tick Loop

```python
async def tick(self):
    if self.connected_to_edge:
        # Send update to edge, receive fused data in response
        response = await self.edge_stub.Edge_ActorSendUpdate(update)
        if response.pickled_edge_predictions:
            self.agent.edge_predictions = pickle.loads(
                response.pickled_edge_predictions
            )
    else:
        # Edge-less: existing behavior with orchestrator
        await self.orchestrator_stub.Client_SendUpdate(update)
```

---

### Phase 6: Scenario Configuration Parsing — ❌ MISSING — BLOCKING GAP

`sim_api.py` does not call `Server_SetEdgeMappings`. The C++ orchestrator's `vehicleToEdgeMapping_` is never populated, so `Client_GetConnectionInfo` always returns `has_edge=false`. Every actor falls back to direct orchestrator connection and the edge process sits idle. All other phases are complete; this is the only thing preventing end-to-end distributed edge operation.

**Required**: Add `compute_edge_mappings()` (parse `edge_list` from YAML, assign global vehicle/RSU indices) and call `Server_SetEdgeMappings(EdgeMappingSetup)` before `Server_StartScenario`. See Phase 6 design below.

**File: `/home/jordan/eCAV/ecav/scenario_testing/utils/sim_api.py`**

#### 6.1 Compute Edge-Vehicle Mappings

```python
def compute_edge_mappings(scenario_params):
    """
    Parse scenario YAML and compute:
    - Which vehicles belong to which edge
    - Global vehicle indices
    - RSU assignments
    """
    edge_mappings = {}
    global_vehicle_idx = 0
    global_rsu_idx = 0

    for edge_idx, edge in enumerate(scenario_params['scenario']['edge_list']):
        edge_mappings[edge_idx] = {
            'vehicle_indices': [],
            'rsu_indices': [],
            'config': edge
        }

        if 'vehicles' in edge:
            for _ in edge['vehicles']:
                edge_mappings[edge_idx]['vehicle_indices'].append(global_vehicle_idx)
                global_vehicle_idx += 1

        if 'rsus' in edge:
            for _ in edge['rsus']:
                edge_mappings[edge_idx]['rsu_indices'].append(global_rsu_idx)
                global_rsu_idx += 1

    return edge_mappings
```

#### 6.2 Send Mappings to Server

```python
async def start_scenario(self):
    # Compute mappings
    edge_mappings = compute_edge_mappings(self.scenario_params)

    # Send to orchestrator with scenario start
    request = SimulationInfo(
        # ... existing fields ...
        edge_mappings_json=json.dumps(edge_mappings)
    )
    await self.ecloud_server.Server_StartScenario(request)
```

---

### Phase 7: start_actors.sh Updates — ✅ COMPLETE

Edge containers launched before vehicle containers. Loop over `num_edges`, runs `edge_process.py -e $e -P $edge_port` (base port 50054, sequential). `stop_actors.sh` generic cleanup covers edge containers. No edge-specific teardown logic needed.

**File: `/home/jordan/eCAV/start_actors.sh`**

#### 7.1 New Edge Container Launch

```bash
# Launch edge containers
echo "Starting $num_edges edge container(s)..."
for ((e=0; e<$num_edges; e++)); do
    edge_port=$((50151 + e))
    container_name="edge_$e"

    docker run $gpu_flag -d \
        --network=host \
        --name="$container_name" \
        -e "HOSTNAME=$container_name" \
        -e IS_DOCKER=1 \
        -e EDGE_INDEX=$e \
        -e EDGE_PORT=$edge_port \
        ecav-python310:latest \
        python3.10 ecav/ecav2/edge_process.py \
            --edge_index $e \
            --edge_port $edge_port \
            -d  # Distributed mode

    echo "  ✓ $container_name started (port $edge_port)"
    sleep 2
done

# Wait for edges to register before starting actors
echo "Waiting for edges to register..."
sleep 5
```

#### 7.2 Actor Launch Changes

```bash
# Actors no longer need to know about edges at launch time
# They discover their edge via Client_GetConnectionInfo RPC
docker run $gpu_flag -d \
    --network=host \
    --name="$container_name" \
    -e IS_DOCKER=1 \
    ecav-python310:latest \
    python3.10 ecav.py $ml_flag -v 0.9.15 -d -i $i
    # No change needed - actor discovers edge at runtime
```

---

### Phase 8: Build, Test, and Bug-Fix — ❌ NOT STARTED (blocked by Phase 6)

This phase validates the new edge architecture by building all components and running an end-to-end test.

#### 8.1 Rebuild Protocol Buffers

```bash
# Regenerate Python protobuf files
cd /home/jordan/eCAV
python -m grpc_tools.protoc -I./ecav/protos \
    --python_out=. \
    --grpc_python_out=. \
    ./ecav/protos/ecloud.proto

# Regenerate C++ protobuf files
cd /home/jordan/eCAV/ecav/ecloud_server/build
cmake ..
make -j$(nproc)

# Copy generated C++ files to source directory
cp ecloud.pb.h ecloud.pb.cc ecloud.grpc.pb.h ecloud.grpc.pb.cc ../
```

#### 8.2 Recompile C++ Server

```bash
cd /home/jordan/eCAV/ecav/ecloud_server/build
cmake ..
make -j$(nproc)
```

#### 8.3 Run End-to-End Test

Use `start_actors.sh` to run a scenario with edges:

```bash
# Test with openscenario_3_edge (has edge_list defined)
./start_actors.sh
# Inputs:
#   - Scenario: openscenario_3_edge
#   - 2 ego vehicles
#   - 1 RSU
#   - ML: y
#   - Rebuild containers: y (first time after code changes)
#   - Start Carla: y
#   - Headless: y
```

#### 8.4 Validation Checklist

- [ ] Edge container starts and registers with orchestrator
- [ ] Edge receives scenario config (vehicle/RSU assignments)
- [ ] Actors connect to edge (not orchestrator)
- [ ] Actors send updates to edge, receive fused predictions
- [ ] Edge reports tick completion to orchestrator
- [ ] Orchestrator advances world tick after all edges complete
- [ ] Scenario completes successfully ("pushed END" message)

#### 8.5 Bug-Fix Iteration

For any failures:
1. Check container logs: `docker logs edge_0`, `docker logs ego_vehicle_0`
2. Check orchestrator logs in `/tmp/ecav_base.*.log`
3. Fix issues and repeat from 8.1 or 8.2 as needed

---

## Implementation Order

0. **Phase 0**: Pre-requisite cleanup — ⚠️ **PARTIAL**
   - `-l`/`--litserve` flag: ✅ done (`ecav/arg_utils.py`, wired through `ecloud_actor_client.py` → `CavWorld`)
   - `distributed:` removed from YAMLs: ❌ still present in ~75 files (all `false`); sim_api.py overwrites with CLI flag so functionally inert, but messy
   - `scenario_params['distributed']` reads: ❌ still in ~31 scenario .py files; reads the CLI-overwritten value, so correct but should be cleaned up
1. **Phase 1**: Protocol buffers — ✅ **COMPLETE** (`ecav/protos/ecloud.proto`)
2. **Phase 2**: YAML schema — ✅ **NOT NEEDED** (edge_index/edge_port assigned dynamically; sequential in start_actors.sh, not in YAML — this is the actual design)
3. **Phase 3**: C++ server — ✅ **COMPLETE** (`ecav/ecloud_server/ecloud_server.cc`: `Edge_Register`, `Edge_TickComplete`, `Client_GetConnectionInfo`, `Server_SetEdgeMappings` all implemented with full edge state tracking)
4. **Phase 4**: Edge process — ✅ **COMPLETE** (`ecav/ecav2/edge_process.py`; `fuse_predictions()` is a placeholder pending WorldFusion O5 integration)
5. **Phase 5**: Actor client — ✅ **COMPLETE** (`ecav/ecav2/ecloud_actor_client.py`: `Client_GetConnectionInfo` call, edge-vs-orchestrator routing, `Edge_ActorRegister`, `Edge_ActorSendUpdate` in tick loop)
6. **Phase 6**: sim_api.py edge mappings — ❌ **MISSING — THIS IS THE BLOCKING GAP**
   - `compute_edge_mappings()`: not implemented
   - `Server_SetEdgeMappings` RPC call: not called
   - Without this, `vehicleToEdgeMapping_` in the C++ server is never populated, so `Client_GetConnectionInfo` always returns `has_edge=false` and all actors fall back to direct orchestrator connection
   - Only handles `edge_list[0]` — multi-edge support is a TODO
7. **Phase 7**: Shell scripts — ✅ **COMPLETE** (`start_actors.sh` launches edge containers via `edge_process.py -e $e -P $edge_port`, edges start before vehicles)
8. **Phase 8**: Build, test, and bug-fix — ❌ **NOT STARTED**

---

## Data Flow Summary

### With Edges (New)

```
Tick N:
1. Orchestrator broadcasts EdgeTick(N) to all edges
2. Each edge waits for all its actors to send updates
3. Actors send VehicleUpdate, receive predictions from tick N-1
4. Edge runs fusion/prediction on tick N data
5. Edge sends Edge_TickComplete(N) to orchestrator
6. When all edges complete: orchestrator ticks CARLA world
7. Repeat with tick N+1
```

### Without Edges (Existing, Preserved)

```
Tick N:
1. Orchestrator broadcasts Tick(N) to all vehicles
2. Vehicles process, send updates
3. Orchestrator collects all updates
4. Orchestrator ticks CARLA world
5. Repeat with tick N+1
```

---

## Key Design Decisions

1. **Actors discover edge at runtime** - Simplifies deployment, YAML doesn't need per-actor edge info
2. **Predictions returned with update response** - Single round-trip, previous tick's predictions
3. **Edge manages its own tick barrier** - Waits for all its actors before reporting complete
4. **Orchestrator tracks edges, not vehicles (when edges exist)** - Simpler barrier logic
5. **Edge-less fallback is automatic** - Based on scenario config, no code path changes needed

---

## Design Decisions (Confirmed)

1. **Edge-to-edge communication** - Not in scope for current research. Each edge owns all vehicles and RSUs listed as its children in the YAML. No handoff between edges.
2. **Edge failure handling** - If an edge crashes, the scenario test is considered a failure and should exit. No recovery mechanism needed.
3. **Dynamic edge assignment** - Not supported. Vehicle/RSU to edge assignment is static per scenario.
4. **RSU handling** - RSUs are children of a specific edge, as defined in the scenario YAML structure (see `openscenario_3_edge.yaml` for example).

---

## Terminology Clarification

**IMPORTANT**: The codebase has two distinct concepts of "distributed" that need to be clarified:

### 1. Distributed Simulation (`-d` / `--distributed`)
- Actors (vehicles, RSUs, edges) run in separate processes/containers
- Communication via gRPC to orchestrator
- Controlled via **command-line flag only** (not YAML)
- The `distributed` field should be **removed from all YAML files**

### 2. Distributed ML Inference (`-l` / `--litserve`)
- ML inference (YOLO, BM2CP) offloaded to LitServe service on port 18000
- Reduces GPU memory requirements per container
- Currently hardcoded in `MLManager` - needs to become a command-line flag
- Controlled via **new command-line flag**: `-l` / `--litserve`

### Command-Line Flags (After Refactor)

| Flag | Long Form | Purpose |
|------|-----------|---------|
| `-d` | `--distributed` | Run actors in separate processes/containers |
| `-l` | `--litserve` | Use LitServe for distributed ML inference |
| `--apply_ml` | | Enable ML models (YOLO, etc.) |

### Pre-requisite Task: Remove `distributed` from YAML

Before implementing the edge architecture, we should clean up the YAML files:

```bash
# Files to update (remove 'distributed: true/false' line):
ecav/scenario_testing/config_yaml/*.yaml
```

The distributed mode is determined by:
- Command-line `-d` flag → `opt.distributed`
- Passed to `ScenarioManager` → `self.run_distributed`

---

## Scenario Mode Matrix

The system must support four combinations based on **command-line flags** and **YAML structure**:

| `-d` flag | `edge_list` in YAML | Behavior |
|-----------|---------------------|----------|
| No | No | Traditional eCAV - everything in one process |
| No | Yes | Edge co-located with orchestrator (current behavior) |
| Yes | No | Distributed vehicles, no edge - vehicles talk to orchestrator directly |
| Yes | Yes | **NEW: Distributed edge** - edge as standalone process, vehicles talk to edge |

The new architecture only changes the **`-d` + edge_list exists** case. All other modes remain unchanged.

### Detection Logic

```python
# In orchestrator/actor startup
# opt.distributed comes from command-line -d flag (NOT from YAML)
has_edges = 'edge_list' in scenario_params['scenario'] and len(scenario_params['scenario']['edge_list']) > 0
is_distributed = opt.distributed  # Command-line flag

if is_distributed and has_edges:
    # NEW: Edge as standalone process
    # Actors connect to edge, edge connects to orchestrator
elif is_distributed and not has_edges:
    # Existing: Actors connect directly to orchestrator
elif not is_distributed and has_edges:
    # Existing: Edge co-located with orchestrator (sim_api.py)
else:
    # Existing: Traditional single-process eCAV
```

---

## Files to Create/Modify

### Phase 0: Pre-requisite Cleanup

| File | Action | Description |
|------|--------|-------------|
| `ecav.py` | MODIFY | Add `-l`/`--litserve` argument |
| `ecav/ecav2/ecloud_actor_client.py` | MODIFY | Add `-l`/`--litserve` argument, pass to MLManager |
| `ecav/ml_manager/ml_manager.py` | MODIFY | Accept litserve flag from caller (already supports `run_distributed`) |
| `ecav/core/common/cav_world.py` | MODIFY | Pass litserve flag to MLManager |
| `ecav/scenario_testing/config_yaml/*.yaml` | MODIFY | Remove `distributed: true/false` lines |
| `ecav/scenario_testing/*.py` | MODIFY | Remove `scenario_params['distributed']` checks, use `opt.distributed` |

### Phase 1-8: Edge Architecture

| File | Action | Description |
|------|--------|-------------|
| `ecav/protos/ecloud.proto` | MODIFY | Add edge messages and RPCs |
| `ecav/ecloud_server/ecloud_server.cc` | MODIFY | Add edge registration, routing, tick logic |
| `ecav/ecav2/edge_process.py` | CREATE | Standalone edge process |
| `ecav/ecav2/ecloud_actor_client.py` | MODIFY | Connection discovery, edge communication |
| `ecav/scenario_testing/utils/sim_api.py` | MODIFY | Edge mapping computation, remove YAML distributed checks |
| `start_actors.sh` | MODIFY | Edge container launching |
| `stop_actors.sh` | MODIFY | Edge container cleanup |
| Scenario YAML files | MODIFY | Add edge_index, edge_port fields |
| `ecav/ecloud_server/ecloud_comms.py` | MODIFY | Add edge client/server classes |
