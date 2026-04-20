# Weekly Update: March 19, 2026
## Paper 2: Resource-Aware Cooperative World Models at the Edge

---

## Slide 1: Paper 2 Thesis (1 min)

**Title:** Resource-Aware Cooperative World Models at the Edge

**Core thesis:** A centralized edge-hosted cooperative world-model pipeline (intermediate fusion + tracking + multimodal prediction) hits a dense-scale deadline cliff under bounded compute. A runtime controller shifts it.

**Target venue:** ACM/IEEE SEC 2026 (May 1 deadline)

---

## Slide 2: What Changed Since Paper 1 (2 min)

Paper 1 (MobiCom): safety envelope characterization with a deliberately simple stack (late fusion + AB3DMOT + linear predictor)

Paper 2 advances to a realistic 2026-era edge workload:
- **Intermediate fusion** (BM2CP / WorldFusion) instead of late fusion
- **Scene-level multimodal predictor** (SMART, NeurIPS 2024) instead of linear
- **Realistic C-V2X channel model** instead of idealized networking
- **V2V baselines** for comparative evidence
- **Runtime adaptation** to manage the compute cliff

---

## Slide 3: V2V Networking Infrastructure Built (3 min)

**C-V2X Channel Engine** (new, this week)
- Standalone C++ engine with Python fallback (pybind11)
- WINNER+ B1 propagation model extracted from ns-3 (3GPP TR 36.885)
- SB-SPS MAC with per-link SINR and capture effect
- 3-level occlusion model via CARLA ray casting (clear / vehicle-blocked / building-blocked)

**Validated PRR degradation curve:**

| N vehicles | PRR   | CBR   | Mean SINR |
|-----------|-------|-------|-----------|
| 4         | 0.917 | 0.000 | +8.9 dB   |
| 8         | 0.857 | 0.129 | +15.3 dB  |
| 16        | 0.667 | 0.294 | +10.1 dB  |
| 32        | 0.433 | 0.713 | -2.9 dB   |
| 64        | 0.269 | 0.823 | -10.3 dB  |

This is the network saturation cliff. Show this curve.

---

## Slide 4: Three V2V Baselines Implemented (3 min)

All three implemented with realistic C-V2X channel model:

### 1. Intermediate Fusion V2V (BM2CP)
- Each vehicle shares learned BEV features over PC5 sidelink
- Per-vehicle fusion + detection + tracking + prediction
- **Finding: visibility-transfer failure** (next slide)

### 2. Early Fusion V2V / AutoCast-style
- Vehicles share detected object point cloud segments
- MCKP bandwidth-aware scheduler (greedy knapsack under 100ms budget)
- Avoids BEV crop problem (points fused before voxelization)

### 3. Harbor-style Hybrid V2I+V2V
- V2I-primary with V2V relay for poor-connectivity vehicles
- Edge server (through backhaul, NOT at RSU) does point cloud merge + detection
- Helper/helpee assignment based on V2I bandwidth quality

---

## Slide 5: Key Finding - Visibility Transfer Failure (3 min)

**Show the timeline figure here**

Standard ego-centric BEV cooperative perception fails at the architecture level:
- RSU at intersection acquires Lincoln's features at frame 55 (Lincoln enters RSU's 32m BEV)
- Features transmitted to ego successfully (channel delivers)
- BM2CP fusion processes them correctly
- **VoxelPostprocessor's 32m ego-centric crop discards the Lincoln** because it's >32m from ego
- Lincoln first detected at frame 102, collision 18 frames later
- Communication cost paid, zero planner utility

**This is NOT a detection quality problem.** When the Lincoln enters the ego's crop, it's detected with score=1.000 at 3.6m match distance.

**The architecture discards cooperative information before it reaches the planner.**

---

## Slide 6: AutoCast Code Audit (2 min)

Analyzed the AutoCast codebase (Qian et al., arXiv:2112.14947):

- All non-passive vehicles get real CARLA LiDAR sensors (confirmed in `scenario_manager.py` line 251)
- LiDAR-based occupancy grid detection runs on each cooperating vehicle
- **But:** GT actor state (velocity, acceleration, position, actor ID) injected post-detection (`PCProcess.py` lines 1092-1142)
- **And:** detections NOT matching a GT actor within 3m are **silently dropped** (GT-assisted false positive filtering)
- Parked cars at the intersection ARE participating cooperators

**Precise characterization:** AutoCast performs real LiDAR sensing but enriches detections with oracle simulator state and eliminates false positives via GT matching.

---

## Slide 7: Harbor Architecture (from paper, SenSys 2024) (2 min)

Key finding from reading the paper:
- Edge is NOT at the RSU. Path: Vehicle -> V2V relay -> Helper -> V2I uplink -> cellular backhaul -> edge server -> detection -> backhaul -> downlink -> Vehicle
- This matches OUR edge topology exactly
- Harbor uses early fusion (raw point clouds) at edge, we use intermediate fusion
- 500ms E2E deadline, helper/helpee assignment every 100ms
- Real-world eval at Mcity testbed with Lincoln MKZ vehicles

**Implication:** Harbor validates the V2I+V2V hybrid approach. Our edge system is the next step: richer world model (intermediate fusion + tracking + prediction) at the edge.

---

## Slide 8: CMP Analysis (2 min)

Confirmed from CMP codebase (tasl-lab/CMP on GitHub):
- Two-stage pipeline: OpenCOOD perception -> AB3DMOT tracking -> MTR prediction
- **MTR retrained on OPV2V data** (not using Waymo weights directly)
- `transform_trajs_to_center_coords` normalizes to agent-centric coordinates during training
- Multi-ego cooperative pipeline on OPV2V/V2V4Real

**Key gap CMP does NOT address:** centralized edge-native world-model service. CMP is still a per-ego cooperative pipeline.

---

## Slide 9: SMART Predictor Integration Challenges (2 min)

SMART (NeurIPS 2024) is the right scene-level multi-agent predictor:
- Processes ALL agents in one forward pass (scene-centric, not per-agent)
- Token-based autoregressive decoder
- 7.2M parameters, 28MB model

**Problem discovered:** SMART was trained on Waymo world coordinates. CARLA coordinates are completely out of distribution. The model predicts the Lincoln moving EAST when it's actually moving WEST. No coordinate conversion fixes this.

**Solution in progress:** Retraining SMART on V2X-Sim 2.0
- V2X-Sim data converted to SMART format (160 train / 20 val scenarios)
- Training script written, currently debugging (running on local RTX 4080 SUPER)
- CMP/MTR code and configs copied to our repo for parallel MTR training

---

## Slide 10: Correct Edge Architecture (2 min)

**The key insight this week:**

The edge stack must be **scene-centric**, not ego-centric:

1. **Cooperative perception ingestion** -- RSU + CAV data arrives at edge
2. **Global world-model construction** -- world-frame tracks (AB3DMOT + SBA)
3. **Scene-centric multi-agent prediction** -- SMART runs ONCE on the whole scene
4. **Per-vehicle planner interface** -- filter global predictions to each ego's ROI

Standard ego-centric intermediate fusion (BM2CP) is the wrong architectural primitive for an edge node serving multiple vehicles. The edge should be a global world-model publisher.

---

## Slide 11: Infrastructure Status (1 min)

### Built and working:
- C-V2X channel engine (C++ + Python fallback)
- CARLA ray-cast occlusion model (3-level)
- 3 V2V managers (intermediate, early/AutoCast, Harbor hybrid)
- V2V metrics (PRR, CBR, AoI, feature delivery fraction)
- MCKP bandwidth scheduler
- SOTA edge manager with pluggable fusion + tracker + predictor
- Edge profiler with per-stage timing
- V2X-Sim 2.0 dataset loaders for both MTR and SMART training
- Scenario configs for all manager types

### Azure A10 edge server provisioned:
- Standard NV36ads A10 v5 (36 vCPUs, 440 GiB RAM, NVIDIA A10 24GB)
- East US 2, ready for N-sweep experiments

---

## Slide 12: What's Blocking (1 min)

**Blocker 1:** SMART retrained on CARLA/V2X-Sim data
- Waymo-trained checkpoint produces wrong predictions on CARLA geometry
- V2X-Sim training data converted, training script running
- Need: successful training run, then validate predictions on Scenario 3

**Blocker 2:** OPV2V dataset for CMP validation
- Test split downloaded (25GB), extracting
- Train split downloading
- Need: verify CMP's reported numbers, then train MTR on V2X-Sim

---

## Slide 13: Critical Path to Submission (2 min)

### P0 (must finish):
1. ~~V2V communication knee~~ **DONE** (PRR curve validated)
2. ~~Visibility-transfer failure~~ **DONE** (documented with GT diagnostics)
3. Retrain SMART on V2X-Sim 2.0 **IN PROGRESS**
4. Planner-aware utility metric **NOT STARTED**
5. Static N-sweep {4,8,16,24,32,48,64} **NOT STARTED** (needs SMART + Azure)
6. Rate-only controller **NOT STARTED**
7. V2V vs edge comparison on Scenario 4 **PARTIALLY DONE**

### Timeline to May 1:
- Week of Mar 24: SMART retrained, CMP validated, N-sweep running on Azure
- Week of Mar 31: Deadline cliff characterized, rate controller implemented
- Week of Apr 7: All evaluation runs complete
- Week of Apr 14: Paper draft
- Week of Apr 21: Revision
- Apr 28: Final submission

---

## Slide 14: Paper Claim Set (1 min)

**Claim 1:** Dense decentralized cooperative perception encounters a communication-induced scaling knee under realistic wireless assumptions.

**Claim 2:** A centralized edge-hosted global world-model pipeline avoids recipient-centric inefficiencies and scales further, but hits a bounded-compute deadline cliff.

**Claim 3:** A lightweight runtime rate controller shifts that cliff right and improves planner-qualified utility and closed-loop outcomes under load.

**NOT claiming:** first cooperative prediction system, first edge perception, universal V2V failure, novel tracking or prediction architecture.

---

## Slide 15: Questions / Discussion
