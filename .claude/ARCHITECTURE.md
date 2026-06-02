# eCAV Architecture

## 1. System Overview
<!-- High-level description of the system and its research goals -->

eCAV is an extension of the [eCAV simulation tool](https://github.com/ucla-mobility/eCAV). The following features are added as an extension of eCAV:

- Distrbuted/Asynchronous communication between eCAV(Edge)/Carla and Vehicle clients using gRPC
- Containerization of vehicle clients using Nvidia Docker 2 (supports local vehicle planning/perception)
- Plugable Algorithm for vehicle autonomous driving
- Support for Propagation Models
- Metric/Evaluation gathering for simulation performance
- Integration of the [CARLA](https://carla.org/) Scenario Runner framework for advanced scenario development and testing
- Distributed perception using [LitServe](https://github.com/Lightning-AI/LitServe) 

### Research Focus

The primary goal of our research is evaluation of edge-assisted control planes. In a representative scenario, there is a camera at a blind intersection. The edge fuses perception data from this camera with perception data from a vehicle allowing the vehicles approaching the intersection to detect obstacle vehicles the otherwise would not be able to detect.

## 2. Process Topology
<!-- Diagram or description of all running processes and how they relate -->

The simplest scenario - a sequential, non-distributed scenario running prediction locally, will have three processes:

- CarlaUE4 - run inside of `opt/carla_simulator` by running the `CarlaUE4.sh` shell script
- `ecav.py`, running the entirety of given scenario.
    - this process will spawn a subprocess of the CARLA scenario_runner.py (in folder `/scenario_runner`)
    - all vehicles and RSUs - all _"actors"_ - will be created with the single process.

All actors within a given scenario correspond to CARLA actors inside of the CARLA world

For distributed *compute* scenarios, things are more complex. `ecav.py` cannot be containerized - at least not easily - because it requires access to the CARLA map files.

Individual actors are then containerized. Ego vehicles are spawned by calls to additional instances of `ecav.py`. When a specific vehicle index - e.g. `-i 0` - is passed to `ecav.py`, this tells the scenario_runner to spawn a object of the class `Ecav2ActorClient` from `ecav/ecav2/ecloud_actor_client.py`. RSUs (Road Side Units - fixed-position actors; typically cameras, but that is not a _hard_ requirement) are spawned by creating containers that run the `ecloud_actor_client.py` file directly. *All* non-ego vehicles are spawned by a single call to `ecav.py` with a vehicle index of -1 - `-i -1`. 

The first ego vehicle process to run - which should be vehicle 0, but that's again not a requirement; it's whichever one registers first - will spawn all ego vehicle actors _within CARLA._

When running a distributed (compute) scenario, communication runs through a central orchestration server - `ecav/ecloud_server/ecloud_server` - that is written in C++ to allow for true multithreading. This comms server is spawned by the root ecav process. 

All individual ego vehicles and RSUs come online, they register with the comms server using gRPC calls. Distributed actors register their connection info - port and IP and actor type - and then receive the scenario file (JSON dictionary) as part of the initial payload. These process then create the actual distinct vehicle_manager class objects - `ecav/core/common/vehicle_manager.py` (and the first process creates the associated CARLA actors) along with their associated perception stacks. It is possible to run vehicles without perception - in this case, they just receive localization data (actual XYZ and roll/pitch/yaw coordinates of all other actors in the simulation), but this is not generally something we use for research purposes.

When running without distributed perception - the default case, which does not use a litserve process - each process will spawn its own perception stack. The perception stack is a mix of LiDAR and cameras - defined in the YAML for a given scenario. Perception stack modules are located in `ecav/core/sensing`.


### 2.1 Root Orchestrator (`ecav.py`)
<!-- Role, responsibilities, what it owns across the simulation lifetime -->
Parses the command line for the scenario - `-t` - and then calls the given scenario's `run_scenario` method (or `run_vehicle` method for distributed ego vehicles)

### 2.2 Central Orchestration Server (C++)
<!-- Location, build process, why C++, what it owns vs. sim_api.py -->
designed to avoid the GIL by allowing true multithreaded responses. This allows multiple vehicles to contact the orchestration layer at once and then - when all vehicles have responded for a given tick - a single payload with all responses can then be pushed down the root orchestrator - via gRPC hooks in `sim_api.py`

This avoids needing to use a python asyncio process to respond to each actor during:

- registration
- ticks

By centralizing this in a C++ process which just aggregates response, we saw significant speedups over a prior instance where the python process had to also do scenario communication orchestration where the GIL became a significant limiter.

### 2.3 Scenario Runner (`scenario_runner/scenario_runner.py`)
<!-- How it is spawned, what it manages, relationship to CARLA -->
Uses CARLA's scenario_runner architecture - note that this is a git-submodule - with modifications. The core pytrees behavior tree still works as originally designed. Scenario_runner scenarios are similar to opencda where there is a .py file and then an associated XML (rather than YAML) file. The scenario_runner is spun up a standalone process by the opencda/ecav scenario process via a `Process` call. The scenario runner executes the actual logic of the scenario itself - e.g. occluded left-hand turn. In the case of distributed actor scenarios, the scenario runner process itself spawns the ecloud_actor_client instance for each ego vehicle.

### 2.4 Vehicle Actors
<!-- Standalone processes vs. Docker containers, lifecycle -->
run as vehicle manager class instances in sequential scenarios. run as ecloud actor client class instances in distributed scenarios, spawned by the scenario manager.

### 2.5 Edge / RSU
<!-- How the edge node differs from vehicle actors, its role in perception fusion -->
edge and rsu are both actor classes in sequential scenarios. And standalone processes in distributed scenarios. When running distributed, they are typically run inside docker containers to avoid the GIL.

#### 2.5.1 Modular Edge Stack

The edge manager is not a monolith. It is composed of three independently replaceable stages, each of which is a distinct axis of research. A given driving scenario (occluded left, overtake, etc. — e.g. `openscenario_3`) should be runnable against *any* permutation of these stages:

- **Fusion** — how perception inputs from the RSU and vehicle(s) are combined. Backends: `worldfusion` (intermediate / BEV feature fusion), `late_fusion` (YOLO bounding-box / object-level fusion), `oracle` (ground-truth), with `early`/point-cloud fusion (raw LiDAR, VRF-style) as a future slot. Selected by `fusion_backend` in YAML; instantiated via `get_fusion()` in `ecav/core/application/edge/fusion/`.
- **Tracker** — multi-object tracking over fused detections. Currently `ab3dmot` (the one to rely on for our tests); `mamba` and others are future slots. Selected by `tracker` in YAML; instantiated via `get_tracker()` in `ecav/core/tracking/`.
- **Predictor** — trajectory prediction over confirmed tracks. `linear` (KF-velocity extrapolation) or `smart` (SMART, NeurIPS 2024). Selected by `predictor_type` in YAML.

`_PluggableEdgeBase` (`edge_manager_pluggable_base.py`) composes fusion + tracker from config and runs the common collect → track → advance loop; subclasses supply the prediction strategy in `run_step()`. The concrete edge manager filenames encode their composition — e.g. `edge_manager_worldfusion_ab3dmot_mtr.py`, `edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py`.

Other components follow the same removable/addable pattern — notably the **Network Model** (latency/jitter/packet-loss applied to the uplink/downlink, configured in the `edge_base` YAML block), which is its own research axis.

**Current testing posture (2026-06):** SMART is broken, so tests use the `linear` predictor; `ab3dmot` is the tracker of record. Fusion is the variable under test — exercise each fusion backend against `ab3dmot` + `linear`, then layer scenario-specific permutations on top. See `docs/kb/raw/notes/tyler_modular_architecture.md`.

### 2.6 LitServe Perception Server
<!-- Standalone process, model served (YOLOv5), how actors communicate with it -->
Standalone process. Communication is done via HTTP and gRPC. Ego vehicles communicate directly with it for late fusion. For intermediate fusion, the edge communicates with it. 

## 3. Communication & IPC
<!-- All inter-process communication mechanisms -->
Carla world - communication via HTTP
All scenario communications - gRPC
Worldfusion still relies on HTTP for communication with Litserve server. But plans to move to gRPC for consistency.

### 3.1 gRPC & Protobuf Schema
<!-- Location of .proto file, key message types, service definitions -->
ecloud.proto - simulation messages
perception.proto - communication of actors with Litserve server

### 3.2 Message Flow Diagram
<!-- Which processes send/receive which messages -->
scenario orchetrator (.py) file sends the scenario file (serialized to JSON copy of the scenario params dictionary, which derives from the scenario YAML along with the default config). The C++ comms server stores this and then responds to registration messages from individual actor clients, initially sending them the serialized scenario dict provided by the orchestrator. Vehicles will receive a vehicle index; RSU receives an RSU index. This allows them to index into the scenario dict to get the particular vehicle info - e.g. spawn point, vehicle characteristics, etc. Vehicles will then update their state each tick to the comms server, which will collate all responses until all actors have responded and then push them down the simulation orchestrator. Once the orchestrator has processed all responses it will then execute a new call to do a tick.

When the edge is a standalone process, tick updates flow from actors to the edge that controls them. When all actors have responded to a given edge, that edge then updates the C++ comms server. And in this case, the comms server updates the orchestrator. Once all edges have responded, the comms server notifies the orchestrator. Once the orchestrator process all edge updates, the next tick is signaled from the orchestrator to the comms server, which then pushes it down to each edge. Each edge then pushes it down to the actors they manage.

## 4. Simulation Tick Lifecycle
<!-- The most important section — step-by-step sequence of a single tick --> 
Mostly covered in the message flow diagram - 3.2. 

Orchestrator ticks the world. All actions from the previous step are then executed by the various actors - vehicle braking/acceleration/steering - within the carla world. Orchestrator notifies comms server. Comms server notifies all actors - or, when running with edge, edges. In the case of an edge scenario, edges then communicate to their actors. Actors then `run_step`, which means updating perception, updating planning, and applying control. They then respond back to the comms server (or their edge) which then responds back to the orchestrator which processes the response. If the scenario is complete, we exit. Otherwise the cycle repeats, starting with the next tick of the CARLA world.

### 4.1 Tick Initiation
<!-- Who initiates, what triggers the tick -->
root process - ecav.py / opencda.py - manages the actual ticking of the CARLA world itself.

Tick is triggered once all actors have run their per-tick steps and - for distributed scenarios - responded back to the scenario orchestrator.

### 4.2 Actor Tick Sequence
<!-- Order of operations across vehicle actors, RSU, perception server -->
- `update_info()`
- `run_step()`
- `apply_control()`
  - vehicles only

### 4.3 Perception Data Flow Within a Tick
<!-- When/how camera+lidar data is sent to LitServe and results returned -->
Depends on whether it's late or intermediate fusion. If the actor is calling litserve - late fusion, this happens between the perception of data and the passing of an update back to their edge or the comms server. In the case of intemediate fusion, the actor sends the image data to the edge which then calls litserve with that data.

In all cases, the actors received obstacle vehicle data that is pushed down to them at the start of each tick.

### 4.4 Edge Fusion
<!-- Where edge camera perception is injected, how it merges with vehicle-local perception -->
Edges send back serialized vehicle manager obstacle vehicle dictionaries, which we then overwrite into the dicts on their vehicle manager instances.

### 4.5 Tick Completion
<!-- What constitutes a completed tick, synchronization mechanism -->
The comms server waits for either all actors - vehicles, RSUs - or all edges - to ack. It then pushes the summary data down to the orchestrator to process. Once the orchestrator is done processing updates, that constitutes the end of a tick.

## 5. Actor State Machine
<!-- States a vehicle actor moves through and the gRPC calls driving transitions -->
<!-- e.g., spawning → ready → ticking → done -->
moves through states outlined in `ecloud.proto`

```
enum VehicleState {
  REGISTERING = 0;
  CARLA_UPDATE = 1;
  UPDATE_INFO_OK = 2;
  GET_DESTINATION = 3;
  TICK_OK = 4; // regular OK / ack
  TICK_DONE = 5; // simulation ended --> include all Debug Info with this update
  OK = 6;
  ERROR = 7;
  DEBUG_INFO_UPDATE = 8;
}
```

`DEBUG_INFO_UPDATE` is a special state that indicates the vehicle has dumped all stored vehicle debug diagnostics back to the comms server at scenario conclusion.

Expected flow is:

- Registering - register with comms server (or edge)
- Carla Update - vehicle has received actor info from CARLA
- Tick_ok - common response
- Tick_done - vehicle has reached destination

## 6. Scenario Configuration
<!-- How scenarios are specified and what drives simulation behavior -->
we make use of CARLA's scenario runner scenarios - which use pytrees behavior trees to execute scenario logic (e.g. occluded left hand turn).

The ecav scenario defines world properties and scenario ownership structure - e.g. which edge(s) own which actor(s)

The scenario runner defines the actual execution logic. Scenario runners rely on scenario classes - e.g a given scenario is a Scenario_3 instance, which is a derived class of a BasicScenario.

### 6.1 File Structure
<!-- .py + .yaml + .xml/.py triplet for each scenario, where each lives -->
ecav scenario:

- .py file
- .yaml file
  - also inherits from default yaml
- located in `ecav/scenario_testing`
  - yaml in `/config_yaml` directory

scenario_runner scenario:

- .py file in `ecav/scenario_testing/scenarios`
- .xml file in `ecav/scenario_testing/scenarios`

### 6.2 YAML Schema
<!-- Key top-level fields: ecloud block, scenario block, perception_is_active, etc. -->

TODO - high priority

### 6.3 Distributed Mode Configuration
<!-- YAML fields that enable/disable distributed actors and/or distributed perception -->

TODO - high priority

## 7. Distributed Perception
<!-- End-to-end description of the LitServe-based perception path -->

TODO - high priority

### 7.1 Local vs. Distributed Perception
<!-- How the system selects which path to use -->

TODO - high priority

### 7.2 LitServe Server Setup
<!-- Python 3.12 env, startup, model loading -->

TODO - high priority

### 7.3 Data Pipeline
<!-- Camera/lidar serialization, transport, inference, result return -->

TODO - high priority

## 8. Deployment
<!-- How to run the system locally and in the cloud -->

### 8.1 Local Execution
<!-- conda env, start_actors.sh, headless vs. visual CARLA -->
When running automated tests or running in a cloud (e.g. Azure) environment, run CARLA in headless mode.

When running locally, we typically run Carla normally so we can see the scenario.

We use the `ecav310` conda environment for the orchestrator process.

### 8.2 Docker-Based Actors
<!-- Building the image, start_vehicles.sh / stop_vehicles.sh -->
One Dockerfile for all actors. 

`start_actors.sh` and `stop_actors.sh` to help facilitate rapidly running local scenarios.

### 8.3 Cloud Deployment
<!-- Pointer to ansible/README.md, high-level description -->
Ansible is unused currently. 

For Cloud deployment, we just run the same shell scripts in cloud - Azure - environment(s). 

Litserve server needs a GPU if running shared perception. 

If individual actors have perception, actor node(s) need GPU.

## 9. Metrics & Evaluation
<!-- What is measured, where data is collected, output format -->

## 10. Known Limitations & Open Issues
<!-- Architectural debt, TODOs with architectural implications -->
