## Paper 2 parking note

**Working title**
**Resource-Aware Cooperative World Models at the Edge**

**Venue**
ACM/IEEE SEC 2026

**Core thesis**
A centralized edge-hosted cooperative world-model pipeline built from intermediate fusion, learned tracking, and multimodal prediction encounters a dense-scale deadline cliff under bounded compute; a runtime resource-aware controller shifts that cliff right by adapting service rate, model fidelity, and model tier while preserving planner-relevant utility.

**Why Paper 2 is distinct from Paper 1**
Paper 1 established correctness limits for a deliberately simple stack: late fusion + AB3DMOT + linear prediction, with the focus on state inconsistency and self-ghosting.
Paper 2 studies a richer, more realistic edge workload: intermediate fusion + SSM tracker + multimodal predictor, with the focus on compute, queueing, and deadline failure under dense participation.

**Candidate contributions**

1. **Dense-scale characterization.**
   First characterization of the deadline cliff for a centralized edge-hosted cooperative world-model stack under dense participation ((25)–(50+) agents).
2. **Planner-aware utility objective.**
   Define deadline-qualified utility at the planner boundary, rather than generic perception metrics alone.
3. **Runtime adaptation controller.**
   Jointly adapt update rate, fusion/prediction fidelity, and model tier to keep the pipeline within a planner-feasible latency region.

**System stack for Paper 2**

* Intermediate fusion perception
* SSM-based multi-object tracker
* Multimodal trajectory predictor
* Centralized edge execution
* SBA enabled throughout to remove identity-confusion confounds

**Main claim to prove**

* The failure boundary is caused by the **joint pipeline** rather than one module in isolation.
* As participation grows, cross-stage queueing and compute pressure push publish/use-time state beyond deadline.
* Resource-aware adaptation preserves the most safety-relevant outputs and improves closed-loop success under load.

**Evaluation matrix**

* Agents: 4, 8, 16, 24, 32, 40, 48, 64
* Hardware budgets: at least 2 edge resource tiers
* Policies: static full stack, rate-only, fidelity-only, model-tier-only, full controller
* Scenarios: varying occlusion, density, and network jitter

**First five figures**

1. Stage-wise latency breakdown vs number of agents
2. End-to-end p99/p999 latency and AoI-at-use vs number of agents
3. Deadline miss rate vs number of agents across hardware tiers
4. Planner-aware utility vs load for static stack and adaptive controller
5. Closed-loop success / intervention / collision rate vs number of agents

**Immediate implementation priorities**

1. Finalize exact intermediate-fusion / tracker / predictor stack
2. Instrument per-stage timing, queue depth, and memory
3. Define planner-aware utility metric
4. Implement simplest useful controller with three knobs only
5. Run first sweep to locate the initial cliff

**Non-goals for Paper 2**

* No state migration
* No MARL coordination
* No VLM/security work
* No attempt to solve all dissertation phases in one paper

**One-sentence abstract seed**
We present a centralized edge-hosted cooperative world-model pipeline for dense vehicular participation and show that fusion, tracking, and prediction jointly induce a compute-driven deadline cliff; we then design a runtime controller that preserves planner-relevant utility and closed-loop performance under bounded edge resources.

**Hard stop**
Paper 2 planning resumes tomorrow for one bounded work session. Tonight is only for recovery.
