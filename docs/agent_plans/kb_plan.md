# Knowledge Base: Plan

## Context

eCAV is a side project supporting a PhD candidate (Tyler Landle). Work happens in bursts with significant gaps between sessions. The primary friction is context restoration: "where were we, what's in flight, what's next." A secondary need is structured capture of research discussions with Tyler and reference-grade documentation on the external systems the project depends on.

Follows Karpathy's raw → wiki model: jrapp contributes raw material (notes, clipped references); Claude maintains the compiled wiki. jrapp does not write wiki articles directly.

## Design Decisions

**Vault at whole repo, not `docs/kb/` only.**
Existing plan docs (`docs/agent_plans/`) and `.claude/ARCHITECTURE.md` become vault-navigable. Wiki articles link directly to them without duplicating content.

**`current_state.md` is the primary artifact.**
Every session ends with an updated `wiki/current_state.md`. This answers "where were we?" in one read.

**Session-end is automatic.**
Claude writes `raw/sessions/YYYY-MM-DD.md` and updates `wiki/current_state.md` at the end of every session where meaningful work was done. jrapp does not need to request this.

**Notes intake via conversation.**
jrapp pastes research discussion notes into the conversation. Claude writes them to `raw/notes/YYYY-MM-DD-topic.md` and updates relevant wiki articles.

## Directory Structure

```
docs/kb/
  raw/
    sessions/       # End-of-session logs (Claude writes)
    notes/          # Research discussion notes (jrapp contributes)
    papers/         # Paper/reference markdown (jrapp drops, Claude summarizes)
  wiki/
    index.md                       # Master index
    current_state.md               # Active work + next steps
    architecture.md                # Compiled from .claude/ARCHITECTURE.md
    research.md                    # Research direction, hypotheses, data
    decisions.md                   # Architectural decision log
    plans_index.md                 # Status of all docs/agent_plans/ files
    concepts/
      edge_assisted_perception.md
      distributed_simulation.md
      collaborative_perception.md
      latency_modeling.md
    dependencies/
      carla.md
      opencda.md
      grpc.md
      litserve.md
      yolov5.md
      worldfusion.md
      bm2cp.md
      sort_ab3dmot.md
      scenario_runner.md
```

## Obsidian Setup

**Vault:** `/home/jordan/eCAV` (whole repo)

**Excluded folders** (Settings → Files & Links → Excluded files):
```
build
yolov5
sort
waymo-open-dataset
ecav/BM2CP
ecav/worldfusion
scenario_runner
evaluation_outputs
see-v2x-input
logs
AB3DMOT_libs
lazy
```

**Plugins to consider:**
- Dataview — query by frontmatter (e.g., articles updated before a date)
- Calendar — navigate session logs by date

## Implementation Checklist

### Phase 1: Foundation
- [x] Create `docs/kb/` directory structure
- [x] Create `wiki/index.md`
- [x] Create `wiki/current_state.md`
- [x] Create `wiki/plans_index.md`

### Phase 2: Initial Wiki Compilation
- [x] `wiki/architecture.md`
- [x] `wiki/research.md`
- [x] `wiki/decisions.md`
- [x] `wiki/concepts/edge_assisted_perception.md`
- [x] `wiki/concepts/distributed_simulation.md`
- [x] `wiki/concepts/collaborative_perception.md`
- [x] `wiki/concepts/latency_modeling.md`
- [x] `wiki/dependencies/carla.md`
- [x] `wiki/dependencies/opencda.md`
- [x] `wiki/dependencies/grpc.md`
- [x] `wiki/dependencies/litserve.md`
- [x] `wiki/dependencies/yolov5.md`
- [x] `wiki/dependencies/worldfusion.md`
- [x] `wiki/dependencies/bm2cp.md`
- [x] `wiki/dependencies/sort_ab3dmot.md`
- [x] `wiki/dependencies/scenario_runner.md`

### Phase 3: Workflow Integration
- [x] Save session-end protocol to Claude memory
- [x] Write this plan to `docs/agent_plans/kb_plan.md`
- [ ] Add session-end reminder to `.claude/CLAUDE.md`

### Phase 4: Obsidian Setup (jrapp action)
- [ ] Install Obsidian (https://obsidian.md)
- [ ] Open vault at `/home/jordan/eCAV`
- [ ] Configure excluded folders (see above)

## Future Extensions

- `scripts/kb_search.py` — simple CLI search over wiki (~20+ articles threshold)
- Health check: stale articles, broken links, inconsistencies
- Experiment log: structured latency measurement table
- Marp slide generation for research presentations
