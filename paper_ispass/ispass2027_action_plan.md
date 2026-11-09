# ISPASS 2027 Action Plan: Addressing the Literature Review & Submission Assessment

This document maps every critique and recommendation from the Deep Research assessment to concrete code mechanisms, existing datasets in [ecloudsim_distributed_sandbox](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox), new validation scripts, and section-by-section revisions in [ecav_ispass](file:///home/atlas/repos/ecav_ispass).

---

## 1. Resolution of the Six Critical Manuscript Issues

### Issue 4.1: Reconciling the Scaling Numbers (`256`, `128`, `64`, and `4–16` Full-Stack CAVs)
- **What the review caught**: [evaluation.tex](file:///home/atlas/repos/ecav_ispass/contents/evaluation.tex#L22-L29) conflated the 128-vehicle $6\times$ speedup (which was measured without perception in `arXiv:2506.16535`) with the 64-vehicle perception-enabled scaling limit.
- **Exact verified numbers to report in Section V (`Platform Validation`)**:
  1. **Control-Only Distributed Scaling (`cumulative_stats_dist_no_perception`)**: Scales to **256 containerized vehicle clients** across 4 host machines with 45 ms p99 network overhead, and achieves a **$6\times$ speedup at 128 vehicles** over the single-process GIL-serialized OpenCDA baseline.
  2. **Per-Vehicle Perception Distributed Scaling (`cumulative_stats_dist_with_perception`)**: Scales to **64 containerized vehicle clients** with onboard YOLOv5 + LiDAR perception across 4 host machines with 35 ms p99 network overhead, bounded by CARLA's server-side camera/LiDAR ray-cast actor limit.
  3. **Full Cooperative Edge Stack Scaling (`WorldFusion + Mamba3DMOT + MTR` on 16 GB RTX 4080S / 24 GB A10)**:
     - Live CARLA sensor rendering supports **1–4 full-sensor ego CAVs per locale** (16 across a 4-locale corridor; late-fusion cycle 85 / 143 / 251 / 470 / 970 ms for $N = 1 / 2 / 4 / 8 / 16$ CAVs; 1 full WorldFusion+Mamba+MTR CAV uses 9.8 GB VRAM at 260 ms/tick).
     - Feature-cache sensor replay (Multi-V2X) scales the live GPU edge pipeline to **33 concurrent contributors per locale**.
     - Metropolitan trace multiplexing (pNEUMA Athens CBD + SUMO) scales the edge tracking, prediction, and migration runtime to **106 locales and 3,200+ concurrent tracks**.

### Issue 4.2: Formalizing the Two Determinism Modes (Live Measurement vs. Timing-Trace Replay)
- **What the review caught**: In a 50 ms tick-stepped loop, if live GPU wall-clock execution fluctuates between 98 ms and 104 ms across repeats, the logical completion crosses the $t + 100\text{ ms}$ tick boundary on one run and slips to $t + 150\text{ ms}$ on the next, altering `BehaviorAgent` braking decisions and diverging closed-loop trajectories.
- **Architectural resolution in `ecloudsim_distributed_sandbox`**:
  Define and evaluate two explicit execution modes in [edge_manager_worldfusion_ab3dmot_linear_predictor.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py#L809-L838):
  1. **Live-Measured Mode (`--timing-mode live`)**: The C++ barrier enforces deterministic tick stepping and intra-tick message ordering while live wall-clock GPU durations (`compute_ms`) determine logical service occupancy (`ready_ms = now_ms + compute_ms + dl_ms`). Ensembles under a fixed seed measure how physical GPU tail jitter propagates into closed-loop trajectory variance.
  2. **Recorded-Stamp Replay Mode (`--timing-mode replay --timing-trace <file>`)**: Replays recorded per-tick `(compute_ms, ul_ms, dl_ms)` stamp sequences (or deterministic LUT/trace stamps) into `_outbound_queue`. Because logical service occupancy becomes a deterministic function of the seed, world evolution, message arrival ticks, and closed-loop trajectories are **100% bitwise reproducible** across repeats and across counterfactual branches in [runner_branch.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/scripts/branching/runner_branch.py).

### Issue 4.3: Explicit Semantics and Independent Validation of the Two-Time-Base Model
- **What the review caught**: The manuscript must explicitly specify the scheduling discipline modeled by the two-time-base gate and validate its accuracy against a continuously running (non-barrier) GPU service, analogous to **LLMServingSim 2.0** (ISPASS 2026 Best Paper).
- **Exact scheduling semantics in code**:
  - **Admission & Queueing Discipline**: Latest-frame, non-queueing replacement (`expected_tick_id` idempotency gate in [edge_process.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/ecav2/edge_process.py#L256-L341) and `_outbound_queue` in [edge_manager_worldfusion_ab3dmot_linear_predictor.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py#L809-L848)). When an edge pass admitted at tick $t_0$ measures $\tau_{\text{compute}}$ ms, the service is logically busy until $t_{\text{free}} = t_0 + \tau_{\text{compute}}$. Sensor frames arriving at ticks $t \in (t_0, t_{\text{free}})$ update the latest-frame buffer in [`JitterBuffer`](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/core/application/edge/latency/jitter_buffer.py#L24-L65) (superseding older unconsumed frames rather than building an unbounded FIFO queue). The next edge invocation is admitted at the first tick $t_1 \ge \lceil t_{\text{free}} / \Delta t \rceil \cdot \Delta t$.
  - **Quantization Error Bound**: Because planners act at discrete control ticks $\Delta t = 50\text{ ms}$, quantizing logical visibility to $\lceil (t_0 + \tau_{\text{ul}} + \tau_{\text{compute}} + \tau_{\text{dl}}) / \Delta t \rceil \cdot \Delta t$ introduces at most $[0, \Delta t)$ of discretization offset relative to a continuous wall-clock timeline, which we directly measure in **Experiment A (P0)**.

### Issue 4.4: Updating Implementation Details & Radio Terminology
- **What the review caught**:
  1. [implementation.tex](file:///home/atlas/repos/ecav_ispass/contents/implementation.tex#L7-L8) still claims the shared perception service runs on LitServe, whereas the codebase migrated to standalone `grpc.aio` servers ([worldfusion_grpc_server.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/ml_manager/worldfusion_grpc_server.py#L1-L70), [yolo_grpc_server.py](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/ml_manager/yolo_grpc_server.py#L1-L103)) and O5 cross-agent batching (`_run_o5_batch_encoder`).
  2. LTE-V2X Mode 4 SB-SPS (`mac_model.py`, `channel_engine.cpp`) and 5G NR-V2X Mode 2 (`ns3_bridge.py` with 5G-LENA) must be distinguished by their exact 3GPP standards (3GPP Release 14 LTE-V2X Mode 4 vs. 3GPP Release 16 NR-V2X Mode 2 sidelink and Uu).

### Issue 4.5: Separating LUT Approximation Accuracy from Field-Trace Calibration
- **What the review caught**: The 476-tick comparison between live [`NS3Bridge`](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/code_overlay/ns3_bridge_wired.py#L1-L320) (UL p50 19.0 ms, DL p50 8.5 ms) and [`Ns3LutSampler`](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/core/application/edge/latency/ns3_lut_sampler.py#L21-L135) (UL p50 16.7 ms, DL p50 9.0 ms) validates the lookup-table approximation against live ns-3 co-simulation, whereas calibration against real networks is provided by comparing the ns-3 / analytical delay and PRR distributions against the empirical **SEE-V2X** (213K C-V2X packets) and **5G-MOBIX** field traces. Both must be presented as distinct validation steps in Section V.

### Issue 4.6: Separating Platform Contributions from Application Case Studies (SEC / SenSys / NSDI)
- **What the review caught**: Conductor (SEC 2026), Self-Beacon Anchoring (SenSys submission), and Khonsu (NSDI submission) are application algorithms built *using* eCAV. In the ISPASS paper, eCAV is the **measurement instrument**, and those studies serve purely as case studies demonstrating *design conclusions that change* when measured compute, packet radio, and sensor-derived state are composed (mirroring **SCALE-Sim v3**). All prior work must be cited in the third person for double-blind compliance.

---

## 2. Positioning Against CADET and 2024–2026 Related Work

### 2.1 Defensible Distinction Against CADET (`arXiv:2606.04072`, ICRA 2026)
 Do not claim that prior platforms never run distributed actors or never measure GPU inference. Instead, contrast the **experimental semantics**:
- **CADET** executes distributed autonomy modules asynchronously in wall-clock time across heterogeneous devices and Mininet-emulated links. Because physical execution is coupled directly to wall-clock progression, (1) simulator rendering or host CPU contention alters the effective sensor/control rate relative to model execution, (2) runs with live GPU jitter cannot replay identical closed-loop trajectories or branch counterfactually from an identical pre-conflict state, and (3) network impairments are applied via Mininet link emulation or trace distributions rather than packet-level 3GPP 5G-LENA Uu / Mode 2 PC5 MAC co-simulation.
- **eCAV** decouples physical execution from logical timing under a deterministic two-level lockstep barrier (`ecloud_server` + `EdgeProcess`). A 130 ms GPU pass always sees a deterministic 50 ms CARLA world step, stamps its measured wall-clock service occupancy onto the logical timeline, composes per-message 5G-LENA / Mode 4 radio delay, and supports both live-measured execution and bitwise-reproducible timing-trace replay alongside matched ground-truth (`Oracle`), latency-stamped ground-truth (`C-lat`), and sensor-derived state models.

### 2.2 Required Citations to Add to `references.bib` and Section II
1. **ISPASS Precedents**:
   - `llmservingsim2026`: *LLMServingSim 2.0* (ISPASS 2026 Best Paper, `arXiv:2602.23036`)
   - `scalesimv3_2025`: *SCALE-Sim v3* (ISPASS 2025, `arXiv:2504.15377`)
   - `muchisim2024`: *MuchiSim* (ISPASS 2024, `arXiv:2312.10244`)
   - `powersensor3_2025`: *PowerSensor3* (ISPASS 2025, `arXiv:2504.17883`)
   - `cimloop2024`: *CiMLoop* (ISPASS 2024, DOI `10.1109/ISPASS61541.2024.00012`)
2. **Recent Vehicular & 5G Co-Simulation**:
   - `cadet2026`: Sharma, Wang, and Srivastava, *CADET* (`arXiv:2606.04072`, 2026) — already in `references.bib`, expand discussion in Section II.
   - `van3twin2025`: *VaN3Twin* (`arXiv:2505.14184`, 2025) — ray-tracing vehicular network co-simulation with CARLA and field calibration.
   - `b5gcybertest2026`: *B5GCyberTestV2X* (IEEE 2026, Document `11629110`) — CARLA + OMNeT++/Simu5G intermediate vs. late fusion study.
   - `opencams2025`: Update author list for `arXiv:2507.09186` in `references.bib`.
   - `msvan3tcarla2024`: Qualify prose in `related.tex` to note that `ms-van3t-CARLA` supports ML perception options, while lacking measured edge-service occupancy feedback into logical planner age.

---

## 3. Concrete Implementation Plan for the Four P0 Experiments

### Experiment A (P0): Independent Timing-Model Fidelity Validation (`LLMServingSim 2.0` Style)
- **Objective**: Quantify how accurately eCAV's barrier-stepped two-time-base model predicts the update rate, superseded-frame ratio, and planner-visible age of a **continuously running real-time edge service** on the same GPU hardware.
- **Harness to run (`scripts/bench_timing_fidelity.py`)**:
  1. **Continuous Standalone Baseline**: A real-time 20 Hz wall-clock driver (`time.perf_counter()` every 50 ms) feeds pre-recorded Multi-V2X / CARLA sensor frames for $N \in \{1, 2, 4, 8, 12, 16, 24, 32\}$ contributors into the actual `WorldFusion + Mamba3DMOT/AB3DMOT + MTR` service running continuously without CARLA barriers (using latest-frame non-queueing admission when the GPU is busy).
  2. **eCAV Two-Time-Base Execution**: Runs the exact same frame sequence under eCAV's 50 ms barrier with logical occupancy stamping (`ready_ms = now_ms + compute_ms + dl_ms`).
  3. **Metrics & Figure 4**: Plot continuous vs. eCAV-simulated p50/p95/p99 planner-side age ($\Delta_{\text{use}}$), achieved service update rate (Hz), superseded-frame fraction, and percentage modeling error across $N=1..32$.

### Experiment B (P0): Demonstrating a Changed Design Conclusion (`SCALE-Sim v3` Style Reversal)
- **Objective**: Show that progressively enabling eCAV's measurement dimensions (Compute $\rightarrow$ Radio $\rightarrow$ Sensor Perception) **reverses the architectural ranking** of cooperative autonomy designs.
- **Data Already Available + Unified Table/Figure (Figure 1 & Figure 5)**:
  Compare four architectures across four evaluation configurations at $N=1..16$ contributors:
  - *Architectures*: (1) Local-Only Perception, (2) Edge Late Fusion (1–5 KB), (3) V2V Intermediate Fusion (`BM2CP` 200 KB / `CMP` 32 KB), (4) Unbounded Edge Intermediate Fusion (`WorldFusion + MTR`, 16.9 KB), and (5) Deadline-Bounded Edge Intermediate Fusion.
  - *Configuration 1 (Idealized: 0 ms compute, ideal radio, GT/offline)*: Unbounded Intermediate ($K=N$) wins monotonically as $N$ grows (AP@0.7 = 0.82, minADE = 1.86 m).
  - *Configuration 2 (+ Measured GPU Compute)*: Beyond $N=10$, Unbounded Edge Intermediate exceeds the 130 ms compute budget (reaching 260–602 ms at $N=16..32$), causing planner age $\Delta_{\text{use}}$ to cross 300–400 ms and **reversing the closed-loop ranking** so lightweight Late Fusion or Bounded Intermediate outperforms Unbounded Intermediate.
  - *Configuration 3 (+ Packet-Level 5G-LENA Uu & PC5 Radio)*: Uncompressed V2V Intermediate (`BM2CP` 200 KB) collapses on PC5 sidelink (PRR = 0.0 vs. $>0.97$ for Late Fusion), and Uu downlink queueing at $N \ge 24$ adds up to 304 ms p50 (952 ms p95) delay.
  - *Configuration 4 (+ Sensor-Derived Perception vs. GT)*: Reveals (a) local perception failing under blind-corner occlusion, (b) ego-cropped V2V (`CMP`) failing on perpendicular cross-traffic (0.67 success vs. 0.90+ scene-anchored edge), and (c) unanchored fusion suffering a 100–150 ms self-ghosting false-brake cliff ($P(S_{\text{op}}) = 0.10$).

### Experiment C (P0): Reproducibility & Counterfactual Branching Validation
- **Objective**: Prove the exact boundary between physical timing variance and logical reproducibility.
- **Harness**:
  1. Run 10 fixed-seed repeats of `Scenario 1` / `Scenario 3` under **Live-Measured Mode**, reporting wall-clock GPU stamp standard deviation ($\sigma_{\text{compute}}$ in ms) and the fraction of ticks where boundary-straddling jitter shifts logical delivery by 1 tick (50 ms).
  2. Run 10 fixed-seed repeats under **Recorded-Stamp Replay Mode**, verifying **0.0 m trajectory RMS difference** and **100% identical message order**.
  3. Demonstrate [`scripts/branching/runner_branch.py`](file:///home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/scripts/branching/runner_branch.py) forking two edge scheduling/handoff policies from an identical pre-conflict snapshot $S(t_0)$.

### Experiment D (P0): Four-Arm Attribution of Timing vs. Perception Error (`Oracle`, `C-lat`, `Local`, `Coop`)
- **Objective**: Separate the **timing channel** (compute + radio age) from the **information channel** (sensor noise, occlusion, association error) using matched scenarios:
  - **Arm A (`Oracle`)**: Ground-truth state, zero compute latency.
  - **Arm C-lat (`Latency-Matched Oracle`)**: Ground-truth state delayed by the exact measured compute + radio stamps of the cooperative neural stack.
  - **Arm B (`Local Sensor`)**: Onboard sensor-derived perception only.
  - **Arm C (`Cooperative Sensor`)**: Full edge-assisted sensor-derived pipeline.
- **Metrics**: First-detection distance (m before conflict), planner-side $\Delta_{\text{use}}$ (ms), false-brake rate, and collision-free passage rate $P(S_{\text{op}})$.

---

## 4. Reorganized 9-Page ISPASS 2027 Manuscript Blueprint

| Section | Target Length | Content & Key Floats |
|---|---|---|
| **I. Introduction** | 1.2 pp | Problem framing (why evaluating cooperative autonomy as a deployed service requires composing measured compute, packet radio, and closed-loop sensor state); **Figure 1** (design-conclusion reversal across the 4 evaluation fidelity configurations); contributions list. |
| **II. Requirements & Related Work** | 1.0 p | Five experimental requirements; structured comparison across 4 clusters (Closed-Loop Autonomy, Network Co-Simulation incl. `VaN3Twin`/`B5GCyberTestV2X`, Compute-Aware/Distributed incl. `Pylot`/`D3`/`CADET`, and ISPASS Performance Simulators incl. `LLMServingSim 2.0`/`SCALE-Sim v3`); **Table I** (Up-front Platform Comparison Matrix with `Unverified` vs `Unsupported` distinctions). |
| **III. Design** | 2.0 pp | **Figure 2** (Full System Architecture TikZ); **Figure 3** (Two-Time-Base Physical-vs-Logical Execution Timeline showing admission, logical occupancy, superseded frames, radio transit, and $\Delta_{\text{use}}$ quantization); formal definition of Live-Measured vs. Recorded-Stamp Replay determinism; hierarchical two-level barrier; dual state models (`Oracle`, `C-lat`, `Local`, `Coop`) and counterfactual branching; three-tier radio plane. |
| **IV. Implementation** | 0.7 p | C++17 `ecloud_server` (`ServerUnaryReactor`) + `EdgeProcess` hierarchy; standalone `grpc.aio` inference servers (`yolo_grpc_server`, `worldfusion_grpc_server`) + O5 cross-agent batching; POSIX shared-memory `NS3Bridge` (3GPP Rel-16 NR-V2X Uu/Mode 2) vs. C++ Mode 4 SB-SPS MAC (`channel_engine.cpp`) vs. SEE-V2X/5G-MOBIX trace replay; pluggable model registry. |
| **V. Platform Validation** | 1.8 pp | **Figure 4** (Experiment A: Two-Time-Base Fidelity vs. Continuous Standalone GPU Service across $N=1..32$); Radio Validation (Live ns-3 vs. LUT error + SEE-V2X/5G-MOBIX calibration); Multi-Tier Scalability (256 no-perc, 64 local-perc, 1–4 full-sensor live vs. 33 feature-replay vs. 106-locale metro replay) & gRPC vs. LitServe optimization table; Experiment C (Live-Measured variance vs. Bitwise-Identical Recorded-Stamp Replay). |
| **VI. Enabled Research Experiments** | 1.6 pp | **Figure 5 & Table** (Experiment B: Architectural Ranking Reversal when adding Measured Compute, Packet Radio, and Sensor Perception); Experiment D (4-Arm Timing vs. Perception Attribution: `Oracle` vs. `C-lat` vs. `Local` vs. `Coop`); brief third-person synthesis of the three enabled studies (compute deadline cliff, 100–150 ms AoI logic cliff vs. 350–400 ms physics boundary, and multi-locale state migration). |
| **VII. Limitations & Artifact** | 0.5 p | 50 ms tick quantization bound ($\le \Delta t$); latest-frame vs. FIFO queueing scope; hardware dependence of cliff locations vs. invariance of methodology; interactive CARLA sensor ceiling vs. trace replay; open-source artifact workflow for adding new models/network profiles. |
| **VIII. Conclusion** | 0.2 p | Concise summary of the validated measurement instrument and its impact on cooperative autonomy systems evaluation. |
