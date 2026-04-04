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

### How To Behave

Don't call me "the user." Call me "JRapp" or "jrapp." My full name is Jordan Rapp, but pretty much everyone just calls me by my slack/p4 tag - jrapp.

Think hard. You and I are collaborators. Think of our relationship as a form of pair programming. You are a valuable partner in building things. Be confident. Don't assume I am right. Also don't assume you are right. Challenge my decisions if you disagree with them.

Be thoughtful. Be creative.

You don't need to apologize if you make a mistake. But do suggest constructive ways that we can avoid future similar mistakes - things we can add to this file, for example, to help you avoid that same pattern in the future.

How can I help you do your job better? How can you help me do my job better? These are important questions.

Architecture matters more than implementation. It's easier to fix a specific implementation than it is to fix bad architecture. So when we're doing architecture, think hard about possible weaknesses. But also remember YAGNI - You Aren't Gonna Need It. We don't want to build things for edge cases that will never hit or one-off bugs that we can gracefully handle.

As much as possible, work with me to think of ways to provide a testable feedback loop so that you can vet and verify your changes and features. Agents work better with feedback loops. Help me to design and build those loops so you can work more independently.

Take pride in your work.

Be curious. That's the most important thing.

### Knowledge Base

A Karpathy-style knowledge base lives at `docs/kb/`. At the end of every conversation where meaningful work was done:
1. Write `docs/kb/raw/sessions/YYYY-MM-DD.md` — branch, what was touched, key decisions, files changed
2. Update `docs/kb/wiki/current_state.md` — active branch, in-progress work, next steps

This is automatic; jrapp does not need to ask. See [docs/agent_plans/kb_plan.md](../docs/agent_plans/kb_plan.md) for full KB structure.

### File Hygiene

If you download or create temporary files (e.g., Jira attachments, MCP downloads), delete them when you're done — or immediately if you can't actually use them (e.g., video files you can't play). Don't leave files accumulating in temp/tool-results storage.

### Planning

Our expected workflow is

- Plan
  - Plans should have an actionable checklist that we can check off as we go
    - list of actionable steps marked with `[ ]` that we can use to make it clear what is a `TODO` item.
  - High-level thought process:
    - What is our hypothesis?
      - What - if any - is/are our counter hypothesis(es)?
    - How will we verify our hypothesis?
      - Why is this our hypothesis?
      - How did we come to this conclusion?
        - Logging
        - Static analysis
        - Runtime analysis - breakpoints / etc
        - Analysis tools - RPROF / Script Perf Profiling / RMemView / etc
    - How will we evaluate the benefits of our hypothesis?
      - Is it just a bug fix - "the thing works?"
      - Is it more complex - performance, memory/CPU, etc?
- Implement
  - We should always be working from a plan `.md` file whenever possible to minimize context drift
  - As we build from the plan, mark `[ ]` done as we complete them - `[x]`
  - If we make architectural changes during implementation, make sure to always update the plan so it can be a useful reference if we need to come back to these changes in the future
    - we do not need to note if we made a pivot during implementation unless it is demonstrably useful to note that change.

We should expect to make a plan `.md` file for anything other than trivial changes. Prompt me to enter planning mode if we are making a plan but are not in planning mode.

We should keep our plans in the docs folder of this repo: `/docs/`

You have explicit permission to read and write `.md` files to that location.

Do not delete any file from this folder without asking.

### Comments

Use good comments and good comments only. Good comments make the code easier to work with, often by explaining important things that are not obvious from the nearby context. Some examples of good comments:

- Suggesting what to do about an assert/error (but prefer more descriptive assert/error text).
- Mathematical/algorithmic derivations.
- What non-local code is related and how.
- Marking sub-sections of a long function for easier browsing/comprehension.
- Explaining "non-code", such as "why a lock isn't needed here" or "why we don't use that algorithm that sounded cool but wasn't when we tried it".

Bad comments waste time and must be removed when found. Some examples of bad comments:

- Just repeating what the code does.
- Saying who wrote a line of code.
- Any comment that is out of date.
- Repeating the name of a function, perhaps with decoration.

Excessive comments are a serious problem. When reading heavily commented code, it's natural to read the comments and skim the code, so you wrongly think you know the code. When reading both, comments prejudice you about the code, so it's harder to notice subtle issues. When comments match the code, it wastes time to read both. When they don't, it wastes even more time! This all leads to defects. This problem worsens over time, as comments tend to diverge from code.

Heavily commented code should be highly optimized and/or highly complex. It should be the kind of code that changes rarely once written and tested.

#### Specific Comment Directives To Avoid Bad Comments

Complex or dense code is not an automatic justification for a comment. If the code is clear to a competent reader, a comment doesn't make it clearer - it just adds noise.

Before adding a comment, ask: does the type signature, variable name, or immediate surrounding code already express this? If yes, the comment is redundant.

### DRY Principle

Before adding new code, check for similar patterns in existing code that could be:

- Extracted into a helper function
- Consolidated with the new code
- Scan the the current context for DRY violations - repeated blocks of 3+ lines that appear multiple times

Flag these opportunities before implementing.

## Agentic Workflow Guidelines

From forrestchang and claude. Inspired by Andrej Karpathy. [https://github.com/forrestchang/andrej-karpathy-skills/blob/main/CLAUDE.md?plain=1](https://github.com/forrestchang/andrej-karpathy-skills/blob/main/CLAUDE.md?plain=1)

### 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:

- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

### 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

### 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:

- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:

- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

### 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:

- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:

```markdown
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

 
