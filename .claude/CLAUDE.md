# eCAV: An Open Source Framework For Testing Of Connected Autonomous Vehicles 

eCAV is an extension of the [eCAV simulation tool](https://github.com/ucla-mobility/eCAV). The following features are added as an extension of eCAV:

- Distrbuted/Asynchronous communication between eCAV(Edge)/Carla and Vehicle clients using gRPC
- Containerization of vehicle clients using Nvidia Docker 2 (supports local vehicle planning/perception)
- Plugable Algorithm for vehicle autonomous driving
- Support for Propagation Models
- Metric/Evaluation gathering for simulation performance
- Integration of the [CARLA](https://carla.org/) Scenario Runner framework for advanced scenario development and testing
- Distributed perception using [LitServe](https://github.com/Lightning-AI/LitServe) 

## Research Focus

The primary goal of our research is evaluation of edge-assisted control planes. In a representative scenario, there is a camera at a blind intersection. The edge fuses perception data from this camera with perception data from a vehicle allowing the vehicles approaching the intersection to detect obstacle vehicles the otherwise would not be able to detect.

## Project Description & Architecture

eCAV supports optional distribution and parallelization to two core aspects of the simulation itself:

- distributed actors, where individual actor(s) run in standalone processes, optionally containerized in Docker containers
- distributed perception, where rather than running a local perception stack, perception data - camera and lidar - is fed to a litserve instance running the core perception stack; this avoids each individual actor from running the same PyTorch model - YOLOv5 - on its own.

With the exception of the central orchestration comms server, which is written in C++ for true multithreaded operation, the project is written entirely in Python. All of the project is written using Python 3.10 environment. We use a Conda environment - invoked by `conda activate opencda` - defined by the file `requirements_3_10.txt`

### Code Organization

The codebase is split across two top-level directories:

- **`ecav/`** — active development target. All new work goes here. Contains the core simulation logic, perception, edge managers, ML manager, scenario files, and protobufs.
- **`opencda/`** — legacy upstream code. Much of it has been superseded by `ecav/` equivalents. Before editing anything in `opencda/`, verify whether an `ecav/` counterpart exists and should be edited instead.

When in doubt about which directory a file belongs to, check `ecav/` first.

### Implementation

Individual scenarios are invoked through the root `ecav.py` entry point. The test scenario is passed as a command line arg - e.g. `-t openscenario_3_edge`. This name must correspond to both a `.py` and a `.yaml` file:

- the PY file is in `ecav/scenario_testing/` (scenario runner files)
- the YAML file is in `ecav/scenario_testing/config_yaml/`
    - the YAML file contains a section `scenario_runner` which provides the name of an associated CARLA scenario, e.g. `scenario_3`, which corresponds to both a `.py` and `.xml` file in `ecav/scenario_testing/scenarios/`

`ecav.py` calls the associated scenario `.py` file, which spawns a separate scenario runner process — `scenario_runner/scenario_runner.py` — based on the arguments specified in the YAML and XML files. It then calls `ecav/scenario_testing/utils/sim_api.py` which spawns the central orchestration server (a C++ executable).

All inter-process communication uses gRPC with protobufs. Proto files are in `ecav/protos/` (`ecloud.proto` for orchestration, `perception.proto` for distributed perception). Generated stubs (`*_pb2.py`, `*_pb2_grpc.py`) live alongside the `.proto` files and are added to `sys.path` at startup by `ecav.py`. Recompile with `python ecav.py --build`.

`ecav.py` handles each time step (_"tick"_) as well as metric gathering and evaluation.

### Usage

For simplicity in starting up multiple actors, use the shell script `start_actors.sh`. For cleanup, use `stop_actors.sh`

This shell script will also start the CarlaUE4 instance that is required for world simulation. We typically run this in headless mode, but it can also run _"normally"_ for visual debugging - what are the cars actually doing?
  
## Claude-Specific Guidelines 

### Model Prefences

- Use Opus 4.6 for planning and architecture work.
- Use Sonnet 4.6 by default and for implementation work.
- Use Haiku (latest) for running unit and integration tests. 

### Behavior Guidelines

- Be curious. This matters more than anything else.
- We are collaborators on this project. Use these guidelines to tailor how you approach working together and to guide the way in which you both approach problems and interact with your collaborators. 
- If I tell you that you are wrong, think about whether or not you think that's true and respond with facts.
- Avoid apologizing or making conciliatory statements.
- It is not necessary to agree with me with statements such as "You're right" or "Yes".
- Avoid hyperbole and excitement, stick to the task at hand and complete it pragmatically.
- Always ensure responses are relevant to the context of the code provided.
- Avoid unnecessary detail and keep responses concise.
- Revalidate before responding. Think step by step.
- Be direct and neutral in your statements.
- Recheck your data and provide sources as much as possible. 
- Always respond as a technical reviewer or researcher tone. 
- Use technical, concise language at all times, based off of tone from papers submitted to Usenix OSDI, SOSP, Sensys, and Mobisys.
- Do not write any sentences that contain a dangling 'this.'
 
