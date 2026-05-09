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

## Plans

**Override**: Plans for this project live in `docs/agent_plans/` (not `~/Documents/agent_plans`). Use snake_case filenames as per global convention.

## Knowledge Base

A Karpathy-style knowledge base lives at `docs/kb/`.

**Update `docs/kb/wiki/current_state.md` after any meaningful block of work** — a diagnosis confirmed, a plan written, a bug fixed, an architectural finding established. Do not wait until end of session. This is the primary artifact for context recovery after a gap.

At the end of every conversation where meaningful work was done:

1. Write `docs/kb/raw/sessions/YYYY-MM-DD.md` — branch, what was touched, key decisions, files changed
2. Verify `docs/kb/wiki/current_state.md` reflects the full session (not just the last block)

The compaction scratch file (`YYYY-MM-DD_scratch.md`) has been removed as a directive — there is no reliable way to know when compaction is imminent, so it is not actionable. Keeping `current_state.md` up to date throughout the session is the mitigation.

This is automatic; whomever is working with you in this workspace does not need to ask. See [docs/agent_plans/kb_plan.md](../docs/agent_plans/kb_plan.md) for full KB structure.

### Code Search

Use the Semble MCP tools (`mcp__semble__search`, `mcp__semble__find_related`) for all codebase searches instead of Grep, Glob, or Read for exploratory queries. Always pass `repo` as `C:\Users\jorda\eCAV` on the first search of a session to build the index — subsequent searches in the same session are cached and fast. Fall back to Grep/Glob only for exact literal matches where Semble would be overkill (e.g. finding a specific known file path).

### Planning — Project Override

When editing any file for a functional reason (bug fix, feature, refactor), also sweep that file for:

- Plans for this project live in `/docs/agent_plans` (not `~/agent_plans`).
- You have explicit permission to read and write `.md` files under `/docs/` and all subfolders.
- Do not delete any file or folder from `/docs/` without asking.

## Code Quality: Progressive Cleanup Policy

1. **`print` → `logger` conversion** — bare `print(...)` calls should become `logger.debug/info/warning/error(...)` at the appropriate level. Use an existing logger if the module already has one; add `logger = logging.getLogger(__name__)` if not. Don't convert prints that are intentional user-facing CLI output.

2. **Debug cruft removal** — commented-out debug blocks, dead `if verbose:` branches, ad-hoc inline debug prints that predate the logger, and similar scaffolding.

This is a driveby policy, not a standalone sweep. Don't open a file purely to clean it. Don't clean files adjacent to the one being edited. The scope is exactly: the file being modified for a functional reason.

## Housekeeping

At the close of sessions, run `docker system df` to check current container usage. If the size of `Images` is over 100GB, prompt whomever you're working with to run `docker system prune` to free up space.
