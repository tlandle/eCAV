# Paper 2 Contributions Summary
## For Professor Discussion / Literature Check

---

## Title (working)
**Resource-Aware Cooperative World Models at the Edge: Network Saturation, Deadline Cliffs, and Runtime Adaptation**

---

## One-paragraph summary

We present the first closed-loop evaluation of cooperative driving perception that integrates a full 3GPP NR V2X MAC simulation with cooperative perception, tracking, and multimodal prediction in a unified system. We show that decentralized V2V cooperative perception hits a communication-induced scaling knee where planner-usable state degrades, that centralizing the pipeline on an edge server avoids the communication bottleneck but introduces a bounded-compute deadline cliff, and that a lightweight runtime rate controller shifts this cliff to sustain planner-qualified utility under dense participation.

---

## Claimed contributions

### C1: First closed-loop cooperative perception evaluation with realistic 3GPP networking

No prior cooperative perception system (V2X-ViT, Where2comm, CMP, BM2CP, AutoCast, V2XVerse, Harbor) evaluates through a real MAC simulation. They assume either perfect communication or use toy models (fixed delay, fixed drop rate, distance-only reachability). Our system runs 5G-LENA NR V2X Mode 2 (the 3GPP Release 16 sidelink MAC) co-simulated with CARLA via shared memory, producing per-link SINR and delivery decisions that directly affect cooperative perception quality.

**What we built:** ns-3 5G-LENA NR V2X co-simulation bridge with shared memory IPC. Validated against full ns-3 simulation at N={9,15,21,33} with PRR measurements.

**What prior work does:** CMP assumes 27 Mbps fixed rate + synthetic async. AutoCast uses distance-only at 7.2 Mbps. V2XVerse uses 802.11g WiFi. Harbor uses real cellular traces but not MAC simulation.

**Literature gap:** The cooperative perception community and the V2X networking community have not been connected. We bridge them.

### C2: Communication-induced scaling knee characterization

We show empirically (via ns-3 NR V2X) that V2V cooperative perception at different payload sizes hits different failure boundaries:
- BSM (300B): PRR >0.97 at N=33. V2V works fine.
- Late fusion (1-5KB): PRR >0.97 at N=33. V2V still works.
- Intermediate fusion (200KB): exceeds sidelink transport block capacity. V2V fails at N=1.
- Even with 5G NR's 40 MHz bandwidth and numerology 2, intermediate feature exchange saturates the sidelink.

**Key finding:** The V2V networking bottleneck is payload-size dependent, not just N-dependent. Small payloads (BSMs, bounding boxes) scale well. Large payloads (intermediate fusion features) cannot use sidelink at all. This is not about quadratic airtime. It's about the mismatch between cooperative perception payload requirements and sidelink transport block capacity.

**Implication:** V2V can support late fusion and object-level sharing. V2V cannot support intermediate fusion. The edge advantage is specifically for rich cooperative world models that require large feature exchange.

### C3: Edge-hosted global world model as the architecturally correct placement

The edge is not a crutch for weak vehicles. It is the correct placement for a shared multi-agent workload:
- Cooperative world models are inherently multi-agent. N vehicles each reconstructing overlapping world state = O(N^2) aggregate duplicated work.
- The edge computes one shared world model: world-aligned fusion, global tracking, scene-level multi-agent prediction (SMART), per-vehicle filtered output.
- Uu scheduled uplink (5G NR) provides contention-free upload to the edge. No MAC collision. Graceful bandwidth sharing.
- Compact downlink (objects + trajectories, ~5KB vs 200KB features) is feasible even at high N.

**What we built:** SOTAEdge manager with pluggable fusion (WorldFusion/late fusion), SBA-enabled AB3DMOT tracker, SMART scene-level predictor, per-stage profiling.

### C4: Dense-scale deadline cliff under bounded edge compute

When the edge runs the full pipeline (intermediate fusion + tracking + multimodal prediction) for increasing N, the total compute time eventually exceeds the planner deadline (100ms). The transition is steep: at N=24 the pipeline may finish in 95ms, at N=28 it needs 105ms. Because the arrival rate is fixed at 10Hz, a 5ms deficit per frame accumulates into unbounded queue depth and AoI divergence within seconds.

**What we measure:** Per-stage latency (fusion, tracking, prediction) vs N on NVIDIA A10 edge hardware. p99 end-to-end latency, deadline miss rate, AoI at the planner boundary.

**This is the main technical result of the paper.**

### C5: Runtime rate controller shifts the cliff

A simple controller monitors p95 latency and queue depth, and adjusts the edge processing rate (edge_dt). When load increases, the controller reduces update frequency to stay within the deadline. This trades temporal resolution for deadline compliance. The planner receives less frequent but timely predictions rather than frequent but stale ones.

**What we show:** Static pipeline vs rate-adaptive pipeline. The controller reduces deadline miss rate, maintains lower AoI, and improves closed-loop safety outcomes.

### C6: Planner-qualified utility metric

Traditional perception metrics (mAP, ADE) don't capture the systems reality. A prediction that arrives after the planner deadline has zero utility regardless of accuracy. We define planner-qualified utility as: U = 1[latency <= deadline] * weighted_recall_of_safety_critical_objects. This connects the systems behavior to the planner's actual needs.

---

## What we do NOT claim

- Novel perception/tracking/prediction architectures (we use existing models)
- Universal failure of V2V (V2V works for small payloads)
- First edge cooperative perception system (EMP, Harbor exist)
- First cooperative prediction system (CMP exists)
- First V2X networking evaluation (the ns-3 community has extensive work)

---

## What IS new

The integration. No prior work connects:
1. Realistic 3GPP MAC simulation
2. Cooperative perception models
3. Multi-object tracking
4. Scene-level multimodal prediction
5. Closed-loop vehicle control
6. Dense participation scaling
7. Runtime adaptation

in a single evaluated system.

---

## Comparison set (peer-reviewed only)

| System | Venue | Perception | Tracking | Prediction | Networking | Closed-loop | Scale |
|--------|-------|-----------|----------|------------|------------|-------------|-------|
| V2X-ViT | ECCV 2022 | Transformer coop perception | None | None | Algorithm-level | No | Small |
| Where2comm | NeurIPS 2022 | Communication-efficient IF | None | None | Sparse feature exchange | No | Small |
| AutoCast | MobiSys 2022 | Occupancy grid + GT enrich | None | Extrapolation | Simplified V2V | Yes | ~10 |
| BM2CP | CoRL 2023 | Multimodal coop perception | None | None | Algorithmic reduction | No | Small |
| EMP | MobiCom 2021 | Edge-assisted multi-vehicle | None | None | Wireless system model | Planner integration | Yes |
| Harbor | SenSys 2024 | PointPillars at edge | None | None | Real cellular traces | Not full stack | 2-100 |
| CMP | IEEE RA-L 2025 | CoBEVT (OpenCOOD) | AB3DMOT | MTR | 27 Mbps fixed + async | No | OPV2V/V2V4Real |
| **Ours** | **SEC 2026** | **WorldFusion/LF** | **AB3DMOT+SBA** | **SMART** | **ns-3 NR V2X MAC** | **Yes** | **4-64 agents** |

CMP is the closest prior work. It has cooperative perception + tracking + prediction. But it does not give: NR V2X MAC-in-the-loop networking, edge-hosted deadline cliff characterization, runtime rate control, or dense-scale (4-64 agent) evaluation.

---

## Key figures needed

1. PRR vs N at different payload sizes (ns-3 NR V2X validated)
2. Visibility-transfer failure timeline (ego-centric BEV crop discards cooperative data)
3. System architecture diagram (perception -> tracking -> prediction -> planner, with network and controller hooks)
4. Stage-wise latency vs N (where compute grows)
5. End-to-end p99 latency / AoI vs N (the cliff)
6. Deadline miss / planner utility vs N (static vs adaptive)
7. Closed-loop safety outcome vs N (collision rate)

---

## Open questions for professors

1. Is the ns-3 co-simulation strong enough for the networking claim, or do we need real hardware measurements?
2. Should we include the V2N2V relay architecture as a third comparison point, or is V2V sidelink vs edge sufficient?
3. Is the "payload size determines V2V viability" finding interesting enough on its own, or is it just motivation?
4. Should SMART be retrained on V2X-Sim (in progress) or is AB3DMOT + linear predictor sufficient for the systems argument?
5. Is SEC the right venue, or is this more MobiSys/SenSys given the networking component?
