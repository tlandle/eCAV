---
updated: 2026-04-04
---
# Latency Modeling

## Why Latency Modeling Matters

Edge-assisted perception introduces a round-trip delay between a vehicle's event and its corrected perception. A vehicle 50 meters from an intersection at 30 mph takes ~3.7 seconds to reach it. A 400ms edge latency means the vehicle acts on state that is 12+ meters stale. Whether this is safe depends on the scenario geometry, the tracking algorithm, and whether latency compensation is applied.

eCAV's research goal is to characterize this tradeoff empirically — at what latency does safety degrade? How does this interact with vehicle count, tracking algorithm, and anchoring protocol?

## Latency Sources

In a deployed edge-assisted system, total round-trip latency has three components:

1. **Radio access** (V2X uplink) — vehicle sensor data to edge
2. **Edge processing** — perception inference, fusion, state publication
3. **Radio access** (V2X downlink) — edge state back to vehicle

eCAV models components 1 and 3 via trace-driven network models. Component 2 is the LitServe optimization target (O1–O5 work).

## Network Models

### C-V2X PC5 (Vehicle-to-Vehicle Direct)

**Standard:** 3GPP TS 36.321 (LTE-V2X)
**Mode:** PC5 Mode 4 — decentralized, no base station required; vehicles select transmission resources using Sensing-Based Semi-Persistent Scheduling (SBSP)

**Dataset:** SEE-V2X (empirical C-V2X measurements)

**Characteristics:**
- Short range (~300–500m reliable range)
- Low latency when channel is clear
- Contention-based; degrades under high vehicle density
- Half-duplex: transmission and sensing compete for resources

**MAC-layer model:** eCAV implements SBSP: each vehicle probabilistically selects resource blocks based on sensed channel occupancy, modeling packet collision rates as a function of vehicle density and transmission interval.

### 5G Backhaul (Vehicle-to-Infrastructure)

**Dataset:** 5G-MOBIX (empirical 5G NR measurements from vehicle-mounted UE)

**Characteristics:**
- Requires base station coverage (infrastructure-dependent)
- Higher peak bandwidth than C-V2X
- Latency varies significantly by load and handover state
- Measured traces capture real-world jitter distribution

### Latency Injection

eCAV injects latency by delaying state publication from the edge to the vehicle. The delay is sampled from the trace distribution rather than fixed, capturing jitter effects on tracking algorithm stability.

## Tracking Algorithms Under Latency

### AB3DMOT (Baseline)
3D Kalman filter tracking. Treats received state as if it arrived instantaneously. Under latency, the published obstacle positions lag real positions; tracking quality degrades proportionally to latency × obstacle velocity.

### VIPS (Velocity-based Temporal Alignment)
Extends AB3DMOT with explicit latency compensation. Known or estimated latency is used to project tracked obstacle positions forward in time before publishing to the vehicle. Reduces effective position error at the cost of requiring a latency estimate.

### Oracle
Ground-truth obstacle positions from the CARLA simulator. Represents the upper bound on achievable tracking performance — what a perfect, zero-latency edge would provide.

## Test Parameter Matrix

```python
latencies    = [0, 100, 200, 400]   # ms
ego_counts   = [1, 4, 8, 16]
manager_types = ["late_fusion", "vips", "oracle"]
anchoring    = ["sba", "no_sba"]
repetitions  = 3
```

Total: 4 × 4 × 3 × 2 × 3 = 288 simulation runs for a full sweep.

## Simulation vs. Research Latency

A practical concern: the LitServe perception pipeline itself adds latency to each simulation tick. If WorldFusion intermediate fusion takes 350ms per tick, we cannot model 100ms network latency — the simulation latency swamps the experimental variable.

The O1–O5 optimization work (bringing WorldFusion e2e from ~355ms → target <100ms) is therefore a prerequisite for the intermediate fusion research experiments, not just an engineering nicety.

## Related

- [Edge-Assisted Perception concept](edge_assisted_perception.md)
- [Research](../research.md)
- [SORT / AB3DMOT dependency](../dependencies/sort_ab3dmot.md)
- [worldfusion_litserve_plan.md](../../../agent_plans/worldfusion_litserve_plan.md)
