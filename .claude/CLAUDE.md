# eCAV: An Open Source Framework For Testing Of Connected Autonomous Vehicles 

eCAV is an extension of the [OpenCDA simulation tool](https://github.com/ucla-mobility/OpenCDA). The following features are added as an extension of OpenCDA:

- Distrbuted/Asynchronous communication between OpenCDA(Edge)/Carla and Vehicle clients using gRPC
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

With the exception of the central orchestration comms server, which is written in C++ for true multithreaded operation, the project is written entirely in Python. Allt of the project is written using Python 3.10 environment. We use a Conda environment - invoked by `conda activate opencda` - defined by the file `requirements_3_10.txt`

### Implementation

Individual scenarios are invoked through the root `openda.py` process. The test scenario is passed as a command line arg - e.g. `-t openscenario_3_edge`. This name must correspond to both a `.py` and a `.yaml` file

- the PY file is in `open_scenario_testing/`
- the YAML file is in `opencda/scenario_testing/config_yaml/`
    - the YAML file contains a section `scenario_runner` which provides the name an associated CARLA scenario, e.g. `scenario_3` which corresponds to both a `.py` and `.xml` file which are contained in `opencda/scenario_testing/scenarios/`

The root `opencda.py` process will call the associated `.py` file - e.g. `openscenario_3_edge.py` - which spawns a separate scenario runner process - `scenario_runner/scenario_runner.py` based on the arguments specified in the YAML and XML files. It will then call `opencda/scenario_testing/utils/sim_api.py` which spawns the central orchestration server - a CPP executable.

All communication between processes are handled via gRPC with messages written using protobufs. The `.proto` file for the project is located at `opencda/protos/ecloud.proto`

The root `opencda.py` process handles each time step - or _"tick"_ - as well as metric gathering and evaluation.

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
 
