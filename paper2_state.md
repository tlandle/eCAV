# Paper 2 (SEC 2026): Complete State Document

**Target:** ACM/IEEE Symposium on Edge Computing (SEC) 2026
**Deadlines:** Abstract April 24, Paper May 1
**Working title:** Deadline-Aware Adaptive Resource Allocation for Edge Cooperative World Models

This document consolidates all findings, design decisions, measurements, and open questions for Paper 2. Use it as context for research assistance and future planning.

---

## 1. Research Problem

### 1.1 The scaling problem

As the number of cooperative agents (N) grows in a dense urban scenario (e.g., a busy intersection with 16-64 connected vehicles + roadside units), the full edge cooperative perception pipeline (fusion + tracking + prediction) exceeds the planner's deadline (100 ms at 20 Hz). The pipeline's compute cost scales approximately linearly with N for fusion and tracking, and quadratically with tracked objects for modern transformer-based predictors.

### 1.2 Why existing work doesn't address this

Prior cooperative perception systems optimize three things: what to upload (EMP, EdgeCooper, Where2Comm), how to fuse (CoBEVT, V2X-ViT, BM2CP), and how to communicate (Harbor, ML-Cooper). None of them optimize what happens after fusion. The full world-model pipeline (fusion + tracking + prediction) is treated as a fixed-cost black box. None evaluate at dense scale (N >= 16). None formulate a compute deadline constraint.

### 1.3 The research question

Can we design a runtime system that keeps the edge cooperative perception pipeline under a hard planner deadline as N grows, while preserving safety-critical prediction quality, by exploiting information that is only available at the edge?

---

## 2. Closest Prior Work

### 2.1 Full pipeline competitors

**CMP (IEEE RA-L 2025)** - The closest prior work.
- Pipeline: CoBEVT fusion + AB3DMOT tracking + MTR prediction
- Reports 67.3 ms total per tick on an NVIDIA A6000 (breakdown: BEV feature 8.4 ms, fusion 14.9 ms, tracking 7.0 ms, prediction 21.7 ms, aggregation 17.1 ms)
- Evaluated on OPV2V and V2V4Real at small agent counts (2-7)
- No dense-scale evaluation, no deadline constraint, no adaptation, no closed-loop
- V2V architecture: each CAV runs full stack, then aggregation merges predictions across CAVs

### 2.2 Edge cooperative perception

**F-Cooper (SEC 2019)** - First feature-level fusion at the edge. Tested with 2-3 vehicles. No tracking or prediction stage. Treats edge compute as unlimited.

**EMP (MobiCom 2021)** - Edge-assisted multi-vehicle perception. Reports 93 ms average E2E latency at small scale. Key technique: REAP (Voronoi-based spatial partitioning) for bandwidth-aware uplink. Detection only, no tracking or prediction. No compute deadline management.

**EdgeCooper (2024)** - Network-aware cooperative LiDAR perception. Multi-hop 5G V2X upload scheduling via max-flow formulation. Uses 2D convolution instead of 3D for speed. No tracking or prediction, no compute deadline.

**Harbor (SenSys 2024)** - V2I-primary with V2V relay. Real-world Mcity testing with 3 Lincoln MKZ vehicles. Emulation up to 100 agents. Detection only (no tracking/prediction). Uses a frame timeout (start fusion even if late frames missing), not a compute budget.

### 2.3 Adaptive edge compute (general)

**D3 / ERDOS (Berkeley)** - Dynamic deadline-driven pipeline for a single AV. The planner's deadline varies with vehicle speed. Adapts via supernet subnet selection. Single vehicle, no cooperation, no edge.

**TimelyNet (EMSOFT)** - Supernet-based neural architecture swap for dynamic deadlines. Single AV.

**Prophet** - Inference time prediction for a single pipeline. No edge, no cooperation.

None of these handle multi-stage cooperative pipelines at the edge under a compute deadline with the ability to exploit cross-vehicle global state.

---

## 3. Key Insight

The edge has three pieces of information that no decentralized (V2V) architecture has:

1. **Persistent track state across ticks.** AB3DMOT at the edge maintains Kalman-filtered trajectories for every tracked object. The edge knows every object's predicted next position before new sensor data arrives.

2. **Global scene graph.** Every ego vehicle's pose and planned path, every tracked object's trajectory, and every agent's FOV are all simultaneously visible. Pairwise conflict geometry (time-to-collision, closing speed, lane conflict) is only computable at the edge.

3. **Centralized compute with a single deadline.** One GPU, one budget, one scheduler. The edge can make globally optimal resource allocation decisions.

The contribution is a runtime resource allocation system that exploits each of these assets with a distinct adaptation mechanism. The individual ML models (WorldFusion, AB3DMOT, MTR) are off-the-shelf. The contribution is the scheduler, not the components.

---

## 4. Design Decisions (With Rationale)

### 4.1 Predictor choice: SMART -> MTR

**Initial approach:** SMART (NeurIPS 2024) as the scene-level joint predictor. It predicts all agents in a single forward pass, which matches the edge architecture.

**Measured problems:**

1. **SMART has a constant 143 ms baseline cost** regardless of N (measured on RTX 4080 SUPER). The autoregressive token decoder generates 16 future tokens sequentially, each requiring full graph attention. At N=64 (179 tracks), SMART measured 248 ms. The 143 ms baseline alone exceeds the 100 ms planner deadline at any N >= 1.

2. **SMART has a Waymo domain gap on OPV2V data.** We ran both models on 100 OPV2V test scenes:

| Model | ADE (m) | FDE (m) | Latency (ms) |
|-------|---------|---------|--------------|
| MTR | 4.54 | 9.40 | 57 |
| SMART | 42.70 | 50.25 | 137 |

SMART was trained on Waymo world coordinates. Its learned token codebook is tuned to Waymo driving patterns and coordinate conventions, and does not generalize to CARLA/OPV2V data. MTR (trained on OPV2V) produces 9.4x lower ADE and 5.3x lower FDE.

**Validation of MTR as a scene-level edge predictor:** CMP runs MTR per ego vehicle (N forward passes). The edge runs MTR once over the global track list (num_cavs=1). We verified single-call scene-level inference produces equivalent quality:

| Mode | ADE (m) | FDE (m) | Latency (ms) |
|------|---------|---------|--------------|
| Multi-CAV (CMP style) | 4.098 | 8.997 | 73.8 |
| Single-CAV (Edge style) | 4.132 | 8.848 | 56.8 |
| Degradation | +0.8% | -1.7% | -23% |

Single-call inference is 1.3x faster with statistically indistinguishable quality. This result is novel - CMP never evaluated this configuration because their V2V architecture has no natural path to global context.

**Outcome:** MTR is our predictor. 27 ms at 2 tracks, 155 ms at 89 tracks on our hardware.

### 4.2 AB3DMOT improvements

The baseline AB3DMOT has several issues at dense scale:

1. **Serial Kalman predict loop.** Per-track filterpy calls with Python overhead. At 179 tracks, this dominates tracking cost.
2. **Ghost tracks** from transient detections even with `min_hits=3`.
3. **Track ID instability** under occlusion, breaking trajectory history and forcing cache misses.
4. **Hungarian matching cost** O(N^2) on large detection sets.

**Changes made:**

1. **Vectorized Kalman predict.** Replaced per-track loop with batched numpy einsum:
   ```python
   all_x = np.stack([trk.kf.x for trk in trackers])
   all_P = np.stack([trk.kf.P for trk in trackers])
   all_x_pred = np.einsum('ij,njk->nik', F, all_x)
   all_P_pred = np.einsum('ij,njk,lk->nil', F, all_P, F) + Q
   ```
   Tracking time at N=8 dropped from 28.6 ms to 15.4 ms (1.9x). At N=16, from 55.6 ms to 30.8 ms (1.8x).

2. **Tightened confirmation and aging.** `max_age` reduced from 6 to 3. Combined with self-beacon filtering for managed ego vehicles, eliminated most ghost tracks.

3. **Duplicate detection filtering.** Added `dup_x_max`, `dup_y_max`, `dup_size_ratio` to cull geometric duplicates from multi-agent fusion NMS imperfections.

4. **Kinematic innovation gate.** Rejects matches where implied velocity exceeds physical limits, preventing parked-vs-moving mismatches at intersections.

5. **Beacon anchoring integration.** For managed ego vehicles, enforces ego-uniqueness via beacon temporary IDs.

**Result:** Vectorized AB3DMOT is the cheapest stage through N=32. Track counts are stable across ticks, improving cache hit rate for temporal amortization.

### 4.3 Fusion model choice

We use WorldFusion's Where2Comm for the attention-based fusion, but the architecture is intended to be swappable. The cost comparison on post-backbone features (`[N, 256, H, W]`):

| N | Where2Comm (ms) | AttenComm (ms) | Maxout (ms) |
|---|-----------------|----------------|-------------|
| 4 | 3.5 | 3.2 | 0.1 |
| 16 | 12.2 | 11.6 | 1.1 |
| 32 | 29.5 | 27.6 | 0.7 |

Where2Comm and AttenComm are similar in cost. Maxout is 20-40x cheaper but sacrifices detection quality. For the paper, fusion tier swapping is a secondary mechanism because the cost difference between Where2Comm and AttenComm is negligible at the N values that matter.

### 4.4 Post-backbone compression architecture

The original WorldFusion pipeline transmits pre-backbone features `[1, 64, 200, 200]` = ~10 MB per vehicle per tick. This is infeasible over any V2X link (confirmed by ns-3 5G-LENA: PRR = 0 at 200 KB payload on sidelink, regardless of N).

We moved to post-backbone compression: vehicles run the backbone and a NaiveCompressor (256x channel compression), transmit `[1, 1, 48, 176]` = ~17 KB per vehicle per tick. The edge receives post-backbone compressed features and runs fusion + detection heads + tracking + prediction.

This required:
- Adding `NaiveCompressor` to the WorldFusion model architecture
- Retraining WorldFusion with compression enabled (joint training of encoder/decoder with detection loss)
- Adding `fuse_post_backbone()` method to the model for edge-side inference
- Modifying the perception manager to run encoder on the vehicle side

The retraining is still in progress. For the offline profiler, we use real OPV2V post-backbone CoBEVT features as a stand-in since they have the same tensor shape.

---

## 5. Adaptive Controller Design

### 5.1 Three adaptation mechanisms

The controller has three mechanisms, each attacking a different scaling dimension:

| Mechanism | Attacks | Dimension |
|-----------|---------|-----------|
| Input contribution filtering | Fusion + tracking | N physical -> K effective |
| Temporal amortization | Prediction frequency | M all tracks -> M divergent |
| Risk-budgeted scheduling | Prediction selection | M divergent -> M selected |

Each mechanism exploits a different edge-unique asset (global scene graph, persistent track state, or centralized deadline).

### 5.2 Mechanism 1: Input Contribution Filtering

**Motivation:** At dense scale, fusion and tracking become the bottleneck regardless of prediction optimization. Input filtering caps the number of agents fused, bounding fusion and tracking cost.

**Why not Voronoi (EMP's approach):** Voronoi partitions by agent pose only. It does not use feature content. An agent whose FOV covers empty space has the same Voronoi cell as an agent covering a conflict zone. Voronoi is blind to what each agent actually contributes.

**Our approach:** Contribution scoring using per-agent detection head outputs. Each agent's post-backbone features are passed through the cls_head to produce a confidence map (psm). The edge integrates each psm against a conflict zone mask (the union of ego vehicles' planned paths extended forward by 5 seconds). The resulting scalar is that agent's contribution score. Select top-K agents by contribution within the fusion budget.

**Why this is more novel than Voronoi:**
- Uses feature content, not just geometry
- Directional toward safety-critical regions
- No new training (cls_head already exists)
- Impossible outside the edge (no individual vehicle has other vehicles' feature maps)

**Name:** "contribution-based agent selection" or "conflict-zone contribution filtering". We avoid "saliency" because it is ML jargon.

### 5.3 Mechanism 2: Temporal Amortization with Divergence Gating

**Motivation:** Most tracked objects follow predictable trajectories between 50 ms ticks. A vehicle going straight at constant speed does not need a fresh MTR prediction every tick. The cached prediction remains valid if the object's actual state matches what was predicted.

**Mechanism:** Per-track prediction cache. On each new tick, the edge compares the tracker's updated state to the cached prediction at the current time. If position error exceeds 1 m or heading error exceeds 10 degrees, mark as divergent and re-predict. Otherwise, use the cache.

**Thresholds:** 1 m position, 10 degrees heading, 10 ticks max cache age. These bound staleness at a level the planner's safety envelope can tolerate (1 m is within typical lane-keeping tolerance).

**Why impossible before:** No prior cooperative perception system runs persistent multi-tick tracking at the edge with a prediction cache. CMP runs prediction per frame from scratch. EMP and Harbor stop at detection. The mechanism requires the edge's unique combination of multi-stage pipeline, persistent state across ticks, and a compute deadline.

### 5.4 Mechanism 3: Risk-Budgeted Scheduling

**Motivation:** Among divergent objects, not all are equally important. The planner needs accurate predictions only for objects that might affect an ego vehicle's braking decision.

**Mechanism:** Risk score per divergent object:
```
risk = speed * proximity_factor * occlusion_factor * lane_conflict
```
- `speed`: from Kalman filter state
- `proximity_factor`: up to 10x boost if close to any ego
- `occlusion_factor`: 2x if not directly visible to any ego (edge-only knowledge)
- `lane_conflict`: 0 or 1, based on whether the object's lane intersects any ego's planned path within 5 seconds

Greedy knapsack: given compute budget B and marginal MTR cost c, select top-ranked min(B/c, |divergent|) objects. Rest get linear extrapolation from cached state.

**Why greedy is sufficient:** MTR has approximately uniform per-object marginal cost. With uniform costs, greedy sorting by risk is optimal for 0/1 knapsack. O(M log M), under 1 ms overhead.

**Why edge-only:** Computing `lane_conflict` requires every ego vehicle's planned path. Proximity requires every ego's position. Both aggregate globally only at the edge.

### 5.5 Rate adaptation: demoted to fallback

Earlier drafts included rate adaptation (tick skipping) as a primary mechanism. We demoted it because:

1. Reducing tick rate degrades freshness for ALL objects uniformly, including safety-critical ones.
2. Input filtering + temporal amortization should bound the effective load such that the pipeline fits within the deadline at 20 Hz in realistic scenarios.
3. Rate adaptation is the generic "run things less often" hammer that any system can apply. It does not exploit edge-unique structure.

Rate adaptation is kept as a degenerate fallback for extreme cases where even the three primary mechanisms cannot fit the budget, but it is not part of the core contribution.

### 5.6 Optimization formulation

**Decision variables:**
- K: number of agents kept after input filter
- x_i: binary, whether to predict track i with MTR
- (f: fusion tier, still considered but may be dropped)

**Objective:**
```
maximize  U(K, x) = sum_i r_i * q_i(x_i)
```
where q_i is the prediction quality function (1.0 for MTR, decaying with cache age, constant low value for linear fallback).

**Constraint:**
```
c_fusion(K) + c_tracking(K) + c_prediction(x) + overhead <= Deadline
```

**Solution:** Two-stage greedy decomposition:
1. Select K to bound fusion + tracking cost
2. Within remaining budget, select x via knapsack on risk-scored divergent tracks

Complexity: O(N log N) for filter, O(M log M) for knapsack, total under 2 ms.

---

## 6. Current Measurements (All GPU on RTX 4080 SUPER)

### 6.1 Per-stage cost scaling

| N | Fusion (ms) | Tracking (ms) | MTR (ms) |
|---|-------------|---------------|----------|
| 1 | 4.1 | 1.4 | 26.6 |
| 4 | 8.9 | 6.8 | 33.1 |
| 8 | 17.9 | 15.4 | 47.3 |
| 16 | 38.1 | 30.8 | 77.3 |
| 32 | 83.5 | 60.6 | 155.3 |
| 48 | 127.8 (extrap) | 89.7 | 259.8 (extrap) |
| 64 | 174.3 (extrap) | 119.9 | 389.7 (extrap) |

Cliff: static pipeline exceeds 100 ms at N ~ 10.

### 6.2 Network (ns-3 5G-LENA)

C-V2X sidelink at 200 KB (intermediate fusion payload):
- PRR = 0 at all N (single-TTI limit 750 bytes, 200 KB needs 267 TTIs, compound PRR underflows)
- Even at 20 KB compressed, sidelink PRR drops from 0.30 at N=2 to 0.006 at N=64

**Uu uplink**: Implementation started but SIGABRT in 5G-LENA NR topology setup. Not yet functional. TODO for next week.

**Finding:** V2V intermediate fusion is physically infeasible over sidelink regardless of compression. The edge Uu uplink is the only viable architecture.

### 6.3 End-to-end offline profile (5-way ablation)

Real OPV2V post-backbone fused features through full pipeline (fusion -> detection -> tracking -> MTR). Steady state ticks 22-29.

**Timing:**

| Config | N=4 | N=8 | N=16 | N=24 | N=32 | N=48 | N=64 |
|--------|-----|-----|------|------|------|------|------|
| static | 46 | 76 | 129 | 121 | 104 | 134 | 184 |
| filter_only | 46 | 75 | 145 | 78 | 59 | 58 | 58 |
| amort_only | 29 | 48 | 77 | 72 | 84 | 107 | 152 |
| risk_only | 53 | 46 | 67 | 70 | 81 | 120 | 139 |
| adaptive_all | 38 | 46 | 66 | 56 | 54 | 50 | 50 |

**Deadline compliance:**

| Config | N=4 | N=8 | N=16 | N=24 | N=32 | N=48 | N=64 |
|--------|-----|-----|------|------|------|------|------|
| static | 100 | 100 | 0 | 0 | 50 | 0 | 0 |
| filter_only | 100 | 100 | 0 | 100 | 100 | 100 | 100 |
| amort_only | 100 | 100 | 62 | 88 | 100 | 25 | 0 |
| risk_only | 88 | 100 | 100 | 100 | 100 | 0 | 0 |
| adaptive_all | 100 | 100 | 100 | 100 | 100 | 100 | 100 |

Key findings:
- Static exceeds 100 ms between N=8 and N=16.
- Adaptive maintains 100% deadline compliance through N=64.
- Input filter alone bounds compute at high N but misses deadline at N=16 (overhead doesn't pay off until N > K_MAX).
- Amortization alone fails at very high N because fusion+tracking exceed budget.
- Risk budgeting alone meets deadline through N=32 but fails at N=48+ (fusion+tracking dominate).

### 6.4 Quality metrics (ADE/FDE) - NEEDS METHODOLOGY FIX

Current measurements (with caveats):

| Config | N=16 ADE | N=16 FDE | N=32 ADE | N=32 FDE |
|--------|----------|----------|----------|----------|
| static | 26.4 | 27.0 | 24.1 | 24.2 |
| amort_only | 26.2 | 26.5 | 23.7 | 23.6 |
| risk_only | 2.7 | 3.2 | 5.2 | 5.6 |
| adaptive_all | 4.7 | 5.0 | 24.3 | 24.1 |

**Problems with current ADE/FDE methodology** (identified, not yet fixed):

1. **Nearest-neighbor GT matching is fragile.** Ghost tracks get matched to random real objects, producing spurious errors. The ~25 m ADE for static is from predicting ghost tracks and comparing to unrelated GT objects, not from genuine prediction quality degradation.

2. **Subset mismatch.** Risk_only predicts a small subset and has low ADE. Static predicts everything including ghosts and has high ADE. The numbers are not directly comparable because they are computed on different sets of tracks.

3. **Synthetic trajectories are too regular.** Perfect circular motion is trivial to predict. Linear extrapolation would score nearly perfect. The numbers don't exercise the prediction model meaningfully.

4. **Input filter at high N causes quality collapse in adaptive_all.** When K drops from 32 to 16, the number of predictions drops from 26 to 12. ADE jumps because many tracks are simply not predicted (the metric treats missing predictions as ignored, not as a failure).

### 6.5 Evaluation methodology fixes planned

1. **Fixed-GT evaluation.** Compute ADE against a fixed set of GT objects, not tracker-matched predictions. Missing predictions count as misses with a penalty distance.

2. **Miss rate metric.** Fraction of GT objects without a matching prediction within 2 m. This captures the cost of dropping tracks.

3. **Risk-stratified quality.** Separate ADE for high-risk GT (safety-critical) and low-risk GT. The adaptive system should preserve high-risk quality, sacrifice low-risk.

4. **Real OPV2V trajectories instead of synthetic circles.** The CMP preprocessed tracking data has actual vehicle trajectories with turns, stops, lane changes. More meaningful prediction benchmark.

---

## 7. Limitations (Honest Self-Assessment)

1. **Divergence thresholds are speed-independent.** A 1 m position error matters more at 100 km/h than at 10 km/h. A speed-dependent threshold is future work.

2. **Risk score is heuristic, not learned.** The multiplicative risk function is reasonable but not validated against closed-loop safety outcomes. A learned risk model would be more principled.

3. **Cache coherence breaks on tracker ID switches.** Rare after our AB3DMOT improvements but not zero. A forced cache eviction on ID change is implemented but occasionally causes unnecessary re-prediction.

4. **Rate adaptation has no hysteresis.** Rapid switching between k values could oscillate. Minimum dwell time at each rate would prevent this.

5. **Closed-loop evaluation not yet done.** All measurements are offline. Closed-loop CARLA impact on collision/false-brake/success rates is the next step.

6. **ADE/FDE methodology needs fixing** (see 6.5 above).

7. **Input filter degrades quality too aggressively in current form.** At high N, the K=16 cap drops too many tracks. Needs refinement to preserve safety-critical tracks always.

8. **Ns-3 Uu uplink not functional.** SIGABRT during 5G-LENA NR topology setup. Needs debugging.

---

## 8. Open Research Questions

1. **Which metric really captures "safety-critical prediction quality"?** Pure ADE is misleading when the prediction set changes. Is risk-weighted ADE better? Miss rate on high-risk objects? Closed-loop collision rate?

2. **Is the multiplicative risk score optimal?** A learned risk score trained on closed-loop outcomes might reveal non-obvious features (e.g., object class, historical interaction patterns).

3. **How does the controller compose with a faster predictor?** If Mamba-MTR or a similar SSM-based predictor cuts MTR cost by 3-5x, does the adaptive controller still provide value, or does the problem move elsewhere?

4. **Is there a theoretical bound on how much the adaptive controller can help?** Given a compute budget B and a set of objects with risk r_i, what is the maximum achievable safety utility? How close does greedy get to this bound?

5. **How should the system degrade gracefully past N=64?** At some point even the adaptive controller runs out of tricks. Is there a smooth transition to a safety fallback mode (emergency braking envelope, lower fidelity warnings)?

6. **What's the right edge hardware tier?** We measured on RTX 4080 SUPER. Real MEC deployments use T4, A10, or L4. The A10 is slower (~30% less throughput), shifting the cliff earlier. What's the right tier for evaluation?

7. **Does input contribution filtering generalize beyond contribution-in-conflict-zone?** Could entropy-based or mutual-information-based filtering provide better signal?

---

## 9. Implementation Status

### Completed:
- WorldFusion edge manager with linear predictor (`edge_manager_worldfusion_ab3dmot_linear_predictor.py`)
- WorldFusion edge manager with MTR predictor (`edge_manager_worldfusion_ab3dmot_mtr.py`)
- WorldFusion edge manager with adaptive controller (`edge_manager_worldfusion_ab3dmot_mtr_adaptive.py`)
- MTR edge predictor with divergence gating + risk budgeting (`ecav/core/prediction/mtr_edge_predictor.py`)
- Vectorized AB3DMOT Kalman predict
- Offline end-to-end profiler with 5-way ablation (`paper2_offline_profiler_v2.py`)
- Five-policy deadline compliance benchmark
- MTR vs SMART quality comparison on OPV2V
- MTR scene-level equivalence validation (multi-CAV vs single-CAV)
- WorldFusion model with `fuse_post_backbone()` method for post-backbone edge architecture
- NaiveCompressor addition to WorldFusion for post-backbone compression
- Ns-3 5G-LENA sidelink integration (validates V2V infeasibility)

### In progress:
- WorldFusion retraining with compression enabled
- Mamba-based SSM tracker training (parallel effort for faster per-stage cost)

### Not started:
- Closed-loop evaluation in CARLA/eCAV simulator with adaptive edge manager
- Fixed-GT evaluation methodology for ADE/FDE
- Risk-stratified quality metrics
- Real OPV2V trajectory-based prediction evaluation
- Ns-3 Uu uplink SIGABRT fix
- Refined input contribution filter (current version too aggressive)
- Speed-dependent divergence thresholds
- Learned risk score (future work)
- Controller hysteresis for rate adaptation (if we keep rate adaptation)

---

## 10. Paper Narrative (Current Draft)

**Section 1 - Introduction:**
Cooperative perception at the edge promises a unified world model for autonomous vehicles. Prior work shows this works at small scale (2-8 agents). At dense scale (16-64 agents at a busy intersection), the full pipeline exceeds planner deadlines. Prior work does not address this because it treats the edge as having unlimited compute. We show the cliff empirically and propose an adaptive resource allocation system.

**Section 2 - Background and Motivation:**
- Cooperative perception pipelines and their compute structure
- The planner deadline constraint from the safety literature
- Dense-scale measurements showing where the cliff appears
- Why network architecture (V2V vs edge) matters for the problem formulation

**Section 3 - System Architecture:**
- WorldFusion + AB3DMOT + MTR pipeline
- Post-backbone compression for edge viability
- Our AB3DMOT improvements (vectorized, gated, beacon-integrated)
- Predictor choice: SMART vs MTR, scene-level validation

**Section 4 - Adaptive Resource Allocation:**
- Problem formulation: resource allocation under deadline constraint
- Three mechanisms: input filter, temporal amortization, risk budgeting
- Edge-unique information that enables each mechanism
- Greedy decomposition algorithm

**Section 5 - Evaluation:**
- Hardware: RTX 4080 SUPER (and Azure A10 for deployment realism)
- Offline microbenchmarks: per-stage scaling, policy ablation
- Quality metrics: ADE/FDE with risk stratification, miss rate
- Closed-loop CARLA: collision rate, false brake rate, success rate
- Network infeasibility: ns-3 5G-LENA sidelink PRR collapse

**Section 6 - Discussion:**
- What the edge enables vs V2V
- Composing with faster future predictors
- Deployment considerations (MEC hardware tiers)

**Section 7 - Related Work:**
- Cooperative perception (CMP, CoBEVT, V2X-ViT, BM2CP, Where2Comm)
- Edge-assisted (F-Cooper, EMP, EdgeCooper, Harbor)
- Single-AV adaptive compute (D3, TimelyNet, Prophet)

**Section 8 - Conclusion:**
- The contribution is not any single algorithm. It is the runtime system that exploits edge-specific global state to maintain safety-critical prediction quality under bounded compute.

---

## 11. Timeline

- **April 7:** All evaluation runs complete (offline + closed-loop at N={4,8,16})
- **April 14:** Paper draft
- **April 21:** Revision after self/co-author review
- **April 24:** Abstract submission
- **April 28:** Final paper submission
- **May 1:** Paper deadline

---

## 12. Files and Artifacts

- `paper2_architecture.md` - Architecture document for the two original mechanisms
- `paper2_optimization.md` - Formal optimization problem formulation
- `paper2_offline_profiler_v2.py` - End-to-end offline profiler with 5-way ablation
- `paper2_figures/offline_profile_v2.csv` - Per-tick timing and quality data
- `paper2_figures/fig_adaptive_*.{png,pdf}` - Adaptive controller plots
- `paper2_figures/fig_compute_mtr.{png,pdf}` - Per-stage scaling with MTR
- `paper2_figures/fig_mtr_vs_smart.{png,pdf}` - Predictor comparison
- `paper2_figures/benchmark_timing_mtr.csv` - Raw benchmark data
- `ecav/core/prediction/mtr_edge_predictor.py` - MTR wrapper with adaptation
- `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_mtr_adaptive.py` - Live adaptive edge manager
- `ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion_mtr.yaml` - Scenario config for closed-loop run
