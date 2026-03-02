# eCAV Architecture

## 1. System Overview
<!-- High-level description of the system and its research goals -->

eCAV is an extension of the [OpenCDA simulation tool](https://github.com/ucla-mobility/OpenCDA). The following features are added as an extension of OpenCDA:

- Distrbuted/Asynchronous communication between OpenCDA(Edge)/Carla and Vehicle clients using gRPC
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
- `opencda.py`, running the entirety of given scenario.
    - this process will spawn a subprocess of the CARLA scenario_runner.py (in folder `/scenario_runner`)
    - all vehicles and RSUs - all _"actors"_ - will be created with the single process.

All actors within a given scenario correspond to CARLA actors inside of the CARLA world

For distributed *compute* scenarios, things are more complex. `opencda.py` cannot be containerized - at least not easily - because it requires access to the CARLA map files.

Individual actors are then containerized. Ego vehicles are spawned by calls to additional instances of `opencda.py`. When a specific vehicle index - e.g. `-i 0` - is passed to `opencda.py`, this tells the scenario_runner to spawn a object of the class `Ecav2ActorClient` from `opencda/ecav2/ecloud_actor_client.py`. RSUs (Road Side Units - fixed-position actors; typically cameras, but that is not a _hard_ requirement) are spawned by creating containers that run the `ecloud_actor_client.py` file directly. *All* non-ego vehicles are spawned by a single call to `opencda.py` with a vehicle index of -1 - `-i -1`. 

The first ego vehicle process to run - which should be vehicle 0, but that's again not a requirement; it's whichever one registers first - will spawn all ego vehicle actors _within CARLA._

When running a distributed (compute) scenario, communication runs through a central orchestration server - `opencda/ecloud_server/ecloud_server` - that is written in C++ to allow for true multithreading. This comms server is spawned by the root opencda process. 

All individual ego vehicles and RSUs come online, they register with the comms server using gRPC calls. Distributed actors register their connection info - port and IP and actor type - and then receive the scenario file (JSON dictionary) as part of the initial payload. These process then create the actual distinct vehicle_manager class objects - `opencda/core/common/vehicle_manager.py` (and the first process creates the associated CARLA actors) along with their associated perception stacks. It is possible to run vehicles without perception - in this case, they just receive localization data (actual XYZ and roll/pitch/yaw coordinates of all other actors in the simulation), but this is not generally something we use for research purposes.

Typically each process will spawn its own perception stack. The perception stack is a mix of LiDAR and cameras - defined in the YAML for a given scenario. Perception stack modules are located in `opencda/core/sensing`.

### 2.1 Root Orchestrator (`opencda.py`)
<!-- Role, responsibilities, what it owns across the simulation lifetime -->

### 2.2 Central Orchestration Server (C++)
<!-- Location, build process, why C++, what it owns vs. sim_api.py -->

### 2.3 Scenario Runner (`scenario_runner/scenario_runner.py`)
<!-- How it is spawned, what it manages, relationship to CARLA -->

### 2.4 Vehicle Actors
<!-- Standalone processes vs. Docker containers, lifecycle -->

### 2.5 Edge / RSU
<!-- How the edge node differs from vehicle actors, its role in perception fusion -->

### 2.6 LitServe Perception Server
<!-- Python 3.12 process, model served (YOLOv5), how actors communicate with it -->

## 3. Communication & IPC
<!-- All inter-process communication mechanisms -->

### 3.1 gRPC & Protobuf Schema
<!-- Location of .proto file, key message types, service definitions -->

### 3.2 Message Flow Diagram
<!-- Which processes send/receive which messages -->

## 4. Simulation Tick Lifecycle
<!-- The most important section — step-by-step sequence of a single tick -->

### 4.1 Tick Initiation
<!-- Who initiates, what triggers the tick -->

### 4.2 Actor Tick Sequence
<!-- Order of operations across vehicle actors, RSU, perception server -->

### 4.3 Perception Data Flow Within a Tick
<!-- When/how camera+lidar data is sent to LitServe and results returned -->

### 4.4 Edge Fusion
<!-- Where edge camera perception is injected, how it merges with vehicle-local perception -->

### 4.5 Tick Completion
<!-- What constitutes a completed tick, synchronization mechanism -->

## 5. Actor State Machine
<!-- States a vehicle actor moves through and the gRPC calls driving transitions -->
<!-- e.g., spawning → ready → ticking → done -->

## 6. Scenario Configuration
<!-- How scenarios are specified and what drives simulation behavior -->

### 6.1 File Structure
<!-- .py + .yaml + .xml/.py triplet for each scenario, where each lives -->

### 6.2 YAML Schema
<!-- Key top-level fields: ecloud block, scenario block, perception_is_active, etc. -->

### 6.3 Distributed Mode Configuration
<!-- YAML fields that enable/disable distributed actors and/or distributed perception -->

## 7. Distributed Perception
<!-- End-to-end description of the LitServe-based perception path -->

### 7.1 Local vs. Distributed Perception
<!-- How the system selects which path to use -->

### 7.2 LitServe Server Setup
<!-- Python 3.12 env, startup, model loading -->

### 7.3 Data Pipeline
<!-- Camera/lidar serialization, transport, inference, result return -->

## 8. Deployment
<!-- How to run the system locally and in the cloud -->

### 8.1 Local Execution
<!-- conda env, start_actors.sh, headless vs. visual CARLA -->

### 8.2 Docker-Based Actors
<!-- Building the image, start_vehicles.sh / stop_vehicles.sh -->

### 8.3 Cloud Deployment (Ansible)
<!-- Pointer to ansible/README.md, high-level description -->

## 9. Metrics & Evaluation
<!-- What is measured, where data is collected, output format -->

## 10. Known Limitations & Open Issues
<!-- Architectural debt, TODOs with architectural implications -->
