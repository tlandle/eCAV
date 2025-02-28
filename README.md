# eCAV

This software is an extension of the [OpenCDA simulation tool](https://github.com/ucla-mobility/OpenCDA). The following features are added as an extension of OpenCDA:

- Distrbuted/Asynchronous communication between OpenCDA(Edge)/Carla and Vehicle clients using gRPC
- Containerization of vehicle clients using Nvidia Docker 2 (supports local vehicle planning/perception)
- Plugable Algorithm for vehicle autonomous driving
- Support for Propagation Models
- Automation scripts for Cloud deployment of simulation using Ansible
- Metric/Evaluation gathering for simulation performance


## Installation

Install Carla and OpenCDA:
https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html


Install Ortools

```bash
pip install --user ortools==9.3.10497 
```

Install k-means-constrained

```bash
pip install k-means-constrained==0.7.0
python -c "from k_means_constrained import KMeansConstrained"
```

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
