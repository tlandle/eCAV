# eCAV 2.0

This repository contains **eCAV 2.0**, a cooperative driving simulator designed for large‑scale (100+ vehicles), cloud‑hosted experiments.
## Key Features

The conductor branch builds on the core functionality of OpenCDA and adds several features tailored for cloud‑native experiments:

- **Asynchronous communication between clients and server:** vehicle and edge clients interact with the simulator via gRPC, enabling non‑blocking data exchange and scalable distributed deployment:contentReference[oaicite:0]{index=0}.
- **Containerised vehicle clients:** each vehicle runs in its own Docker container using Nvidia Docker 2 for GPU‑accelerated perception:contentReference[oaicite:1]{index=1}.  Containers make it easy to scale to hundreds of vehicles and isolate experiments.
- **Pluggable driving algorithms:** vehicle control and perception modules can be swapped out to test new planners or sensors:contentReference[oaicite:2]{index=2}.  For example, you can integrate the BM2CP LiDAR–camera fusion model or other perception networks.
- **Propagation models:** support for radio‑propagation models to emulate realistic V2X communication channels:contentReference[oaicite:3]{index=3}.
- **Automation scripts for cloud deployment:** Ansible scripts automate provisioning of hosts and deployment of simulation components:contentReference[oaicite:4]{index=4}.
- **Metrics and evaluation gathering:** built‑in logging collects performance metrics to analyse perception latency, network throughput and control accuracy:contentReference[oaicite:5]{index=5}.

## Installation

### Prerequisites

1. **CARLA & OpenCDA:** Follow the [OpenCDA installation guide](https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html) to install CARLA and OpenCDA:contentReference[oaicite:6]{index=6}.
2. **Dependencies:** You will need additional Python packages such as `ortools` and `k-means-constrained`.  They can be installed with

   ```bash
   pip install --user ortools==9.3.10497  # optimisation library:contentReference[oaicite:7]{index=7}
   pip install k-means-constrained==0.7.0 # clustering algorithm:contentReference[oaicite:8]{index=8}
   python -c "from k_means_constrained import KMeansConstrained"

Create gRPC stubs

```bash
python -m grpc_tools.protoc -I./opencda/protos --python_out=. --grpc_python_out=. ./opencda//protos/ecloud.proto
```

For perception, install [Nvidia Docker 2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Usage

Activate the conda enviroment

```bash
conda activate opencda
```

Start the Carla server

```bash
./CarlaUE4.sh
./CarlaUE4.sh -RenderOffScreen # to run headless
```

Run opencda vehicle test

```bash
python opencda.py -t single_2lanefree_carla -v 0.9.12
python opencda.py -t multi_2lanefree_carla -v 0.9.12
python opencda.py -t ecloud_edge_scenario -v 0.9.12
```

Build Docker image for vehicle clients
```bash
sudo docker build -t vehicle-sim .
```

Run vehicle containers
```bash
sudo bash start_vehicles.sh
```

Stop and remove vehicle containers
```bash
sudo bash stop_vehicles.sh
```

Docs to run Simulation in the cloud using Ansible: [README](ansible/README.md)

Scratchpad

```bash
# number of running Docker containers (+1)
docker ps -a | wc -l

# dump *all* logs
docker ps -q | xargs -L 1 docker logs

# create symlink to file
ln -s <source> <destination>
```

## ToDo List

- gRPC server should reject old packets

- use `CHECK` in logs more to simplify and optimize logging logic for perf

- move to completion queue & threadpool with servers on client & sim API

- is maphelper required for 2 Lane Free?

Top Level

```yaml
# eCloud perception
define: &perception_is_active false
...
# eCloud
ecloud:
  num_servers: 2 # % num_cars to choose which port to connect to. 2nd - nth server port: p = 50053 + ( n - 1 )
  server_ping_time_s: 0.005 # 5ms
  client_world_time_factor: 0.9 # what percentage of last world time to wait initially
  client_ping_spawn_s: 0.05 # sleep to wait between pings after spawn
  client_ping_tick_s: 0.01 # minimum sleep to wait between pings after spawn
```

Scenario

```yaml
# define scenario.
scenario:
  ecloud: 
    num_cars: 128
    location_type: random # random || explicit - applies to Spawn & 
    done_behavior: destroy # destroy || control
  single_cav_list: 
    - <<: *vehicle_base
      destination: [606.87, 145.39, 0]
      behavior: # overrides
        <<: *base_behavior
        max_speed: 100 # maximum speed, km/h
        tailgate_speed: 111
        overtake_allowed: false
        local_planner:
          <<: *base_local_planner
          debug_trajectory: true
          debug: true
```

```yaml
 #define the platoon basic characteristics
edge_base: &edge_base
  max_capacity: 10
  inter_gap: 0.6 # desired time gap
  open_gap: 1.2 # open gap
  warm_up_speed: 55 # required speed before cooperative merging
  change_leader_speed: true # whether to assign leader multiple speed to follow
  leader_speeds_profile: [ 85, 95 ] # different speed for leader to follow
  stage_duration: 10 # how long should the leader keeps in the current velocity stag
  target_speed: 55 # kph
  num_lanes: 4
  edge_dt: 0.200 # use this and base dt to figure out how often to request updates of WP
  search_dt: 2.00
  edge_sets_destination: true # otherwise, edge sets WP
```
