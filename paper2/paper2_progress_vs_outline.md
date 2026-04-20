# Paper 2 Progress: Implementation vs. Paper Outline

Maps each section of "Resource-Aware Cooperative World Models at the Edge" to implementation status.

---

## Section 1: Introduction and Core Thesis

| Claim | Status | Evidence |
|-------|--------|----------|
| V2V fails under dense participation | **DONE** | C-V2X channel engine validated: PRR drops from 0.92 (N=4) to 0.27 (N=64) with M=20 subchannels |
| Edge-hosted world model hits deadline cliff | **NOT STARTED** | Need N-sweep evaluation {4..64} with the full edge pipeline (intermediate fusion + SSM tracker + SMART predictor) |
| Runtime controller shifts the cliff | **NOT STARTED** | AdaptiveEdge manager exists in sandbox but tri-factor controller (rate + fidelity + tier) not implemented |

---

## Section 2: Related Work and Subsystem Evolution

| Subsection | Status | Notes |
|------------|--------|-------|
| 2.1 Cooperative perception evolution (early/intermediate/late) | **DONE (implementation)** | All three fusion levels implemented: early (AutoCast), intermediate (BM2CP V2V), late (existing edge managers). Writing needed. |
| 2.2 SSM-based MOT | **PARTIALLY DONE** | AB3DMOT wrapper exists in `ecav/core/tracking/`. TrackSSM (Mamba-based) referenced in paper but NOT implemented. Currently using KF-based AB3DMOT. |
| 2.3 Edge computing and resource-aware orchestration | **PARTIALLY DONE** | Edge profiler exists (`EdgeProfiler`). Latency model extracted to `ecav/core/application/edge/latency/`. Adaptive edge manager shell exists. Full controller logic not implemented. |

---

## Section 3: The V2V Networking Fallacy

| Subsection | Status | Evidence |
|------------|--------|----------|
| 3.1 Bandwidth saturation (640 Mbps for 40 agents intermediate fusion) | **DONE (analytical)** | Paper math: 200KB/frame * 10Hz * 40 agents = 640 Mbps >> 50 Mbps C-V2X capacity |
| 3.2 MAC layer failures (SB-SPS collapse) | **DONE** | C-V2X channel engine implements SB-SPS with reselection counters. PRR collapse validated at N=32+ |
| 3.3 Accumulative jitter / synchronization penalty | **PARTIALLY DONE** | Jitter buffer exists in edge pipeline. V2V managers do not model jitter (instantaneous delivery assumed). Need to add per-link delay from channel engine to V2V fusion pipeline. |
| **NEW FINDING: Visibility-transfer failure** | **DONE** | BM2CP intermediate fusion V2V: RSU features contain Lincoln but ego-centric 32m crop discards it. Documented with GT diagnostic evidence. |
| **NEW FINDING: AutoCast/V2XVerse analysis** | **DONE** | AutoCast uses GT detection for cooperating vehicles (not real sensors). V2XVerse closed-loop coop agent passes ego LiDAR as placeholder for cooperative data. Neither demonstrates real closed-loop V2V perception. |

---

## Section 4: System Stack Architecture

| Subsection | Status | What Exists | What's Missing |
|------------|--------|-------------|----------------|
| 4.1 Intermediate fusion perception | **DONE** | BM2CP and WorldFusion perception managers, BEV feature extraction, Where2comm attention fusion | Works for edge. V2V version limited by 32m ego-centric crop. |
| 4.2 SSM-based MOT | **NOT DONE** | AB3DMOT (KF-based) with SBA, beacon ID manager, anchoring | TrackSSM / Mamba tracker not implemented. Paper describes it as core component. Need to decide: implement TrackSSM or acknowledge AB3DMOT as baseline tracker. |
| 4.3 Multimodal trajectory predictor | **DONE** | SMART predictor (NeurIPS 2024) integrated and running. Linear predictor as fallback. | Working in edge pipeline. V2V managers currently use linear predictor only. |
| 4.4 Planner-aware utility metric | **PARTIALLY DONE** | S_op metric (Paper 1) evaluates safety outcomes. EdgeMetrics tracks per-tick timing. | Deadline-qualified utility function (step function on time, weighted by kinematic relevance) NOT formally implemented. Currently using binary collision/no-collision. |

---

## Section 5: Dense-Scale Characterization and the Deadline Cliff

| Subsection | Status | What's Needed |
|------------|--------|---------------|
| 5.1 Joint pipeline failure boundary | **NOT STARTED** | Need N-sweep {4,8,16,24,32,40,48,64} measuring per-stage latency: fusion O(N^2), tracking O(M), prediction O(M^2). Multi-ego scenario configs exist for 2,4,8,16 ego. Need 24-64. |
| 5.2 Cross-stage queueing and AoI | **PARTIALLY DONE** | Jitter buffer models AoI. Edge profiler captures per-tick timing. Need to run at saturation load and measure queue depth divergence. |
| Empirical deadline cliff curve | **NOT STARTED** | The key figure: p99 latency vs N showing the sharp transition. Requires running the full edge pipeline at increasing N. |

---

## Section 6: Runtime Resource-Aware Adaptation Controller

| Subsection | Status | What Exists | What's Missing |
|------------|--------|-------------|----------------|
| 6.1 Service update rate (r) | **PARTIALLY DONE** | `edge_dt` config parameter already throttles edge update rate. AdaptiveEdge manager exists. | Dynamic rate adjustment based on queue depth not implemented. |
| 6.1 Model fidelity (f) | **NOT STARTED** | - | Voxel resolution scaling, attention head pruning, prediction horizon reduction. None implemented. |
| 6.1 Model tier (m) | **NOT STARTED** | - | Model tier swapping (heavy vs distilled). No distilled models trained. |
| 6.2 Planner-aware optimization | **NOT STARTED** | - | Formal optimization: max utility subject to C(r,f,m) + Q_delay <= T_deadline. Not implemented. |

---

## Section 7: Evaluation Matrix

| Parameter | Status | Notes |
|-----------|--------|-------|
| Agent scaling: 4,8,16,24,32,40,48,64 | **PARTIALLY DONE** | Scenario configs exist for 2,4,8,16 ego. Need 24-64 ego configs. Channel engine validated offline for all N. |
| Hardware tier A (multi-GPU MEC) | **PARTIALLY DONE** | Azure 4-node deployment exists from Paper 1. Need profiling at scale. |
| Hardware tier B (single Orin RSU) | **NOT STARTED** | No Orin hardware available. Could emulate via compute budget throttling. |
| Static full stack baseline | **DONE** | SOTAEdge manager with full pipeline (fusion + tracker + SMART predictor). |
| Rate-only adaptation | **NOT STARTED** | Need to implement dynamic rate control in AdaptiveEdge. |
| Fidelity-only adaptation | **NOT STARTED** | Need voxel scaling and prediction horizon control. |
| Tier-only adaptation | **NOT STARTED** | Need distilled model variants. |
| Full controller | **NOT STARTED** | Requires all three knobs + optimization loop. |
| V2V comparison baselines | **DONE** | Three V2V managers: intermediate (BM2CP), early (AutoCast), hybrid (Harbor). |

### Results Table from Paper (Target, Not Yet Achieved)

| Control Policy | p99 Latency | Deadline Miss | Utility | Collision Rate |
|----------------|-------------|---------------|---------|----------------|
| Static Stack | 245.3 ms | 98.2% | 0.12 | 64.5% |
| Rate-Only | 92.1 ms | 4.1% | 0.58 | 18.2% |
| Fidelity-Only | 115.4 ms | 35.6% | 0.65 | 29.8% |
| Tier-Only | 88.5 ms | 2.3% | 0.71 | 12.4% |
| Full Controller | 82.7 ms | 0.5% | 0.94 | 1.1% |

**These numbers are targets from the paper outline, not measured results. All evaluation runs are pending.**

---

## Section 8: Conclusion

Depends on Sections 5-7 results. Writing pending.

---

## Overall Progress Summary

| Category | Done | Partial | Not Started | Total |
|----------|------|---------|-------------|-------|
| Infrastructure / Networking | 4 | 0 | 0 | 4 |
| V2V Baselines | 3 | 0 | 0 | 3 |
| V2V Analysis / Findings | 3 | 1 | 0 | 4 |
| Edge Pipeline Components | 2 | 2 | 1 | 5 |
| Deadline Cliff Characterization | 0 | 1 | 2 | 3 |
| Runtime Controller | 0 | 1 | 3 | 4 |
| Evaluation Runs | 0 | 1 | 4 | 5 |
| Scenarios | 2 | 1 | 0 | 3 |
| **Total** | **14** | **7** | **10** | **31** |

### Completion: ~45% of implementation, ~0% of evaluation runs

---

## Critical Path to Paper Submission (SEC 2026, May 1 deadline)

### Must-have (blocks the paper):
1. **N-sweep with full edge pipeline** (Section 5) -- need to measure the deadline cliff
2. **At least rate adaptation** in the controller (Section 6) -- need to show it shifts the cliff
3. **Scenario 4 with all manager types** (Section 7) -- need the V2V vs edge comparison
4. **Planner-aware utility metric** (Section 4.4) -- need the evaluation framework

### Should-have (strengthens the paper):
5. TrackSSM integration (replaces AB3DMOT with SSM tracker)
6. Fidelity and tier adaptation knobs
7. BM2CP retrained with wider range (shows intermediate V2V could work with correct architecture)
8. C++ channel engine compiled (faster N-sweep)

### Nice-to-have (if time permits):
9. Full tri-factor optimization loop
10. Real V2I trace replay (instead of synthetic jitter)
11. Orin hardware tier emulation
12. Harbor with real PointPillars at edge (instead of GT detection)
