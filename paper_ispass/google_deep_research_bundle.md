# Google Deep Research Package: eCAV ISPASS 2027 Platform Paper

Copy and paste (or upload) this entire document into Google Deep Research in the Gemini web app.

---

## PART 1: PROMPT FOR GOOGLE DEEP RESEARCH

You are an expert computer systems architect, autonomous driving systems researcher, and IEEE ISPASS / USENIX OSDI / SOSP / SenSys program committee reviewer.

I am developing a 9-page double-blind conference paper (plus unlimited references, IEEEtran format) for **ISPASS 2027 (Tool / Benchmark / Platform Track)** on **eCAV**, a distributed, closed-loop evaluation platform for edge-hosted cooperative autonomy.

Below, I provide four complete reference sections:
- **Part 2**: Deep code-level architectural specification and inventory of 12 distinct capabilities implemented in our repository (`ecloudsim_distributed_sandbox`), along with measured experimental data from our codebase and three companion studies built on the platform (Safety Envelope / SenSys, Conductor / SEC 2026, and Khonsu / NSDI).
- **Part 3**: Quantitative and qualitative SOTA simulator and baseline comparisons audited in our workspace.
- **Part 4**: Our strict internal Paper Writing and Reviewing Guide and Checklist (distilled from advisor reviews by Dr. Umakishore Ramachandran and Dr. Alexandros Daglis).
- **Part 5**: The complete, verbatim LaTeX source of our current ISPASS 2027 manuscript (`main.tex` and `contents/*.tex`).

Please execute a deep research and editorial audit answering the following five objectives:

1. **Literature & Positioning Audit (2024–2026 SOTA)**:
   - Search recent literature (ISPASS, IISWC, MICRO, HPCA, OSDI, SOSP, NSDI, MobiCom, MobiSys, SenSys, SEC, IV, ITSC, ICRA, CoRL 2023–2026) for distributed autonomous driving simulators, V2X co-simulation frameworks (e.g., OpenCDA, V2Xverse, AutoCast, Coopernaut, EI-Drive, ms-van3t-CARLA, OpenCAMS, Pylot, D3, CADET, Carla-SUMO-ns3 bridges), and edge ML serving benchmarks.
   - Verify whether any recent platform (2024–2026) composes deterministic lockstep multi-container actors, measured GPU service time feeding into planner-side age at use ($\Delta_{\text{use}}$), packet-level 5G-LENA Uu/PC5 radio co-simulation, and matched ground-truth vs. sensor-derived state models. Identify any missing citations we must add to `tab:platforms` and Section III.

2. **Codebase-to-Manuscript Gap Analysis (What to Incorporate)**:
   - Compare the implemented capabilities and already-measured experimental datasets in **Part 2** and **Part 3** against the current LaTeX draft in **Part 5** (which is currently ~3.5 pages with `[PLANNED]` markers in Section VI).
   - Recommend exactly which capabilities, architectural mechanisms, figures, and existing measured datasets from **Part 2** should be promoted into the 9-page ISPASS manuscript, and specify how to allocate the 9-page budget across sections, tables, and figures.

3. **Methodological & Experimental Tightening for ISPASS Reviewers**:
   - Critique the experimental plan (E1–E9) from the perspective of an ISPASS reviewer evaluating a performance analysis tool/instrument.
   - For the remaining `[PLANNED]` items (E2 shared-service GPU concurrency knee, E3 fixed-seed logical reproducibility vs. wall-clock variance, E4 timing-model fidelity validation, E5 radio-plane calibration, E6 state-model divergence ladder), specify the exact figure axes, statistical metrics, and control baselines needed to make the validation airtight.

4. **Strict Writing & Style Audit Against Advisor Guidelines**:
   - Audit every section and paragraph of the verbatim LaTeX in **Part 5** against the rules in **Part 4** (and our standing stylistic rules: no dangling "this" or "it" pronouns; expand every acronym on first body use; no coined compound nouns; no "it is not X, it is Y" phrasing; no hype words; cite every quantitative claim; one idea per sentence; USENIX OSDI/SOSP/SenSys reviewer tone).
   - Provide a line-by-line list of violations in the current LaTeX draft and supply corrected replacement LaTeX prose.

5. **Concrete Section-by-Section Expansion Draft**:
   - Provide expanded, publication-ready LaTeX prose and table structures for the under-populated sections (`Design`, `Implementation`, `Evaluation`, and `Discussion`) that incorporate the codebase's real mechanisms and measured numbers while adhering strictly to the writing guide in **Part 4**.

---

## PART 2: DEEP CODEBASE ARCHITECTURE, CAPABILITIES, AND MEASURED DATA

### 2.1 Core Execution Architecture (`ecloudsim_distributed_sandbox`)
- **Primary Orchestrator (`ecav.py`, `sim_api.py`, `ecloud_server.cc`)**:
  - `ecav/ecloud_server/ecloud_server.cc` (976 lines, C++17) implements `EcloudServiceImpl` using gRPC C++ asynchronous callback reactors (`grpc::ServerUnaryReactor`) and native C++ worker threads on port 50051, avoiding Python GIL serialization across up to 256 concurrent actors.
  - **Two-Level Hierarchical Tick Barrier**: When edge processes are active (`hasEdges_ == true`), `ecloud_server` delegates per-actor synchronization to each `EdgeProcess` (`ecav/ecav2/edge_process.py`, 1,014 lines). On `Server_DoTick`, `ecloud_server` fans out `PushTick` to registered edges; each edge fans out `PushTick` via `asyncio.gather` to its assigned vehicle/RSU containers (`Ecav2ActorClient` in `ecav/ecav2/ecloud_actor_client.py`), waits on `tick_complete_event` until all assigned actors send `Edge_ActorSendUpdate`, executes the 4-stage edge pipeline (`run_edge_step`), and sends `Edge_TickComplete` to `ecloud_server`.
- **Three Actor Process Topologies**:
  1. **Sequential (`ecav.py -t <scenario> --apply_ml`)**: All actors and edge managers run in one Python process. Includes O5 Cross-Agent Batched Encoding (`_run_o5_batch_encoder`), which defers per-actor voxel/camera encoding during `update_info()` and batches all $N$ agents into one GPU pass.
  2. **Edge-Only Distributed (`ecav.py -t <scenario> -eo`)**: Vehicles and RSUs run in the host process while edge servers run in isolated Docker containers registered via `EdgeRegistrationServer` (port 50055) and invoked in parallel via `Edge_PerformFusion` (`EdgeFusionClient`, idempotent on `expected_tick_id`).
  3. **Full Container Distributed (`ecav.py -t <scenario> -d`)**: Every vehicle, RSU, and edge server runs in its own Docker container (`start_actors.sh` / `stop_actors.sh`) with causal log-based readiness barriers (`"registered successfully"`, `"Registered with"`).
- **Standalone gRPC Inference Service (`-l`)**:
  - `yolo_grpc_server.py` (`PerceptionServicer`) dynamically batches multi-camera RGB frames (`max_batch_size=8`, `batch_timeout_s=0.005`).
  - `worldfusion_grpc_server.py` (`WorldFusionServicer`) runs PointPillar + Lift-Splat-Shoot BEV feature extraction over `grpc.aio` (64 MB limit) with `msgpack_numpy` + `zlib` framing.
  - **Measured Optimization Gains (`docs/kb/wiki/decisions.md` & `worldfusion_performance.md`)**:
    - Replacing LitServe HTTP + `multiprocessing.Manager` IPC queues (which added 100–120 ms of pickle IPC overhead on 861 KB payloads vs. 67 ms actual GPU inference) with standalone gRPC cut vehicle feature extraction latency from **185 ms to 130 ms (-55 ms)** and RSU latency from **380 ms to 140 ms (-240 ms)**.
    - Sending camera tensors as `uint8` instead of `float32` (O4) cut request size by **53% (8,021 KB to 3,803 KB)** at 0.31 ms encode cost (~4 MB/s saved per intersection on cloud links).
    - Returning `spatial_features` as `float16` (O1) halved response size (**10 MB to 5 MB**) and cut HTTP transport time by **40% (291 ms to 175 ms)**.
    - Setting CARLA native camera capture resolution to $640 \times 480$ matching YOLOv5 input (D4) eliminated resize overhead and reduced end-to-end YOLO latency from **82 ms to 22 ms (-73%)**.

### 2.2 Two-Time-Base Accounting (Physical Execution vs. Logical Timing)
- **Physical Time Base**: CARLA steps deterministically by $\Delta t = 50\text{ ms}$ (20 Hz) only after all actors and edges complete their barrier slot. World evolution and message order are 100% deterministic under a fixed seed regardless of GPU speed fluctuations.
- **Logical Time Base**:
  - Uplink frames from vehicles and RSUs are stamped with capture tick and sampled radio uplink delay $\tau_{\text{ul}}$, entering `JitterBuffer` (`ecav/core/application/edge/latency/jitter_buffer.py`).
  - `EdgeProfiler` (`ecav/core/application/edge/edge_profiler.py`, 1,404 lines) measures wall-clock GPU execution $\tau_{\text{compute}}$ across fusion, tracking, and prediction.
  - Outbound predictions enter `_outbound_queue` with logical release time `ready_ms = now_ms + compute_ms + dl_ms`. While the edge service is logically occupied until $t + \tau_{\text{compute}}$, frames arriving at intermediate ticks ($t+50$, $t+100$) are superseded rather than queued.
  - Every published prediction carries `obs_tick = source_tick - time_since_update`, so the vehicle planner (`BehaviorAgent`) and `PlanningMetricsCollector` record exact observation-to-actuation age at use ($\Delta_{\text{use}}$).

### 2.3 Multi-Fidelity V2X Radio Plane
1. **Live ns-3 5G-LENA Co-Simulation & Lookup Tables (`ns3_bridge.py`, `ns3_lut_sampler.py`)**:
   - Bridges CARLA actor positions to an external C++ ns-3 5G-LENA process via POSIX shared memory (`shm_open`/`mmap`), modeling both scheduled cellular Uu (40 MHz @ 5.9 GHz, numerology 0) and autonomous NR-V2X Mode 2 PC5 sidelink.
   - `Ns3LutSampler` interpolates `(N_contenders, log(payload_bytes))` over `ns3_uplink_lut.csv` and `ns3_downlink_lut.csv` with max-of-$N$ uplink contention barrier sampling.
   - **Measured Live Co-Sim vs. LUT Validation**: Over 476 live closed-loop fusion ticks, live ns-3 Uu uplink p50 is **19.0 ms** (p99 **20.0 ms**) vs. LUT **16.7 ms** (p99 **17.8 ms**); downlink p50 is **8.5 ms** (p99 **8.6 ms**) vs. LUT **9.0 ms** (p99 **9.3 ms**); shared-memory bridge wall-clock overhead is **18.3 ms p50** (27.3 ms p95) per tick.
   - **Measured Cellular Uu Scaling (`ns3_uplink_lut.csv`, `ns3_downlink_lut.csv`, 16.9 KB payload)**: Uplink p50 grows from **9.6 ms ($N=4$) to 37.8 ms ($N=31$)**; downlink p50 grows from **5.3 ms ($N=4$) to 304 ms ($N=31$)**, with p95 reaching **952 ms at $N=31$**.
   - **Measured PC5 Sidelink Capacity Limit**: Single-TTI capacity is ~750 B. Basic Safety Messages (300 B) and Late Fusion object lists (1–5 KB) sustain Packet Reception Ratio (PRR) $> 0.97$ up to $N=33$, whereas uncompressed intermediate BEV features (200 KB, e.g., BM2CP) suffer **PRR = 0.0 even at $N=1$**.
2. **Standalone C++ 3GPP Mode 4 SB-SPS MAC & WINNER+ B1 Channel Engine (`channel_engine.cpp`, `mac_model.py`)**:
   - Models 3GPP TR 36.885 WINNER+ B1 urban path loss, 3-level CARLA ray-cast occlusion (LOS, +15 dB vehicle-blocked, building-blocked NLOS), Rician/Rayleigh fading, and sensing-based semi-persistent scheduling ($M=20$ subchannels, reselection counter $RC \in [5, 15]$).
   - **Measured Density Sweep**:
     - $N=4$: PRR 0.917, Channel Busy Ratio (CBR) 0.000, mean SINR +8.9 dB
     - $N=8$: PRR 0.857, CBR 0.129, mean SINR +15.3 dB
     - $N=16$: PRR 0.667, CBR 0.294, mean SINR +10.1 dB
     - $N=32$: PRR 0.433, CBR 0.713, mean SINR -2.9 dB
     - $N=64$: PRR 0.269, CBR 0.823, mean SINR -10.3 dB
3. **Empirical Field-Trace Replay (`HybridModel` in `latency_model.py`)**:
   - Replays 213,000 real C-V2X packets from the SEE-V2X dataset (3 urban intersections, converting sequence gaps into deterministic packet-drop bursts) and 5G-MOBIX cross-border corridor traces for Uu and inter-MEC backhaul.

### 2.4 Four-Stage Edge World Model Pipeline & Pluggable Backends
1. **Stage 1: Cooperative Perception & World-Canvas Decoding**:
   - Backends: `Oracle` (CARLA ground truth with configurable range/FOV/occlusion filter), `LateFusion` (YOLOv5 + LiDAR frustum + 3D IoU connected-component clustering, 1–5 KB/msg), `LateFusionETSI` (ETSI TR 103 562 sender-side CPM self-announcement omission), `VIPS` (`edge_manager_vips_temporal_alignment.py`, velocity forward-projection + Mahalanobis matching), `AutoCast` (MCKP point-cloud patch scheduling, 10–50 KB/msg), `BM2CP` (PointPillar + camera intermediate fusion, ~200 KB/msg), `CoBEVT/CMP` (256$\times$ compressed BEV, ~32 KB/msg), and `WorldFusion` (PointPillar + 4-camera Lift-Splat-Shoot + Where2comm confidence-gated attention + 256$\times$ `NaiveCompressor`, **16.9 KB/msg**).
   - **Scene-Anchored World Canvas (`WorldVoxelPostprocessor`)**: Unlike ego-centric V2V models (BM2CP, CMP) whose $\pm 32\text{ m}$ crop around the ego discards RSU-observed perpendicular threats until ~2.7 s before impact, `WorldFusion` pins the RSU at index 0 and decodes anchors across the full conflict zone (`[-102.4, 102.4] m`).
2. **Stage 2: 3D Multi-Object Tracking & Self-Beacon Anchoring (SBA)**:
   - **Vectorized `AB3DMOT`**: Batched `numpy.einsum` Kalman predict + vectorized 3D corner GIoU speeds up tracking by **$1.80\times\text{–}1.86\times$** ($N=4$: 13.7 ms $\rightarrow$ 7.6 ms; $N=8$: 28.6 ms $\rightarrow$ 15.4 ms; $N=16$: 55.6 ms $\rightarrow$ 30.8 ms). Cross-view handover gating (5 m cross-source NMS + coasting velocity preservation) reduces track IDs per run from 10.5 to 1.8 and planner uncovered ticks from 53.6 to 11.2.
   - **Learned SSM Tracker (`Mamba3DMOT`)**: 3-layer bidirectional selective state-space model (`BiMambaDynamicsNet`, `d_model=64`, `d_state=16`, 10-frame window) running in **0.11–0.12 ms/track** (0.06 ms/track batched), maintaining a coast-through-blind-window `memo_bank`.
   - **Self-Beacon Anchoring (`BeaconIdManager`, `EgoUniquenessMonitor`)**: Enforces an ego-uniqueness invariant at the edge publish boundary via Mahalanobis beacon-to-track Hungarian matching, asymmetric spatial exclusion (`dup_x_ahead = 2.5 m`), and SAE J2945 temporary ID rotation reconciliation. Eliminates self-ghosting (where a vehicle's own delayed bounding box is reflected back as a phantom obstacle in its path). Also records formal brake provenance (`true_positive`, `self_ghost`, `track_merge_identity_switch`, `external_stale_fp`, `other_fp`).
3. **Stage 3: Scene-Level Trajectory Prediction & Deadline-Aware Adaptation (`Conductor`)**:
   - **Predictors**: `LinearPredictorManager` (<1 ms), `SMARTPredictorManager3D` (143 ms at $N=2..4$, 248 ms at $N=64$; requires 22-tick history), and `MTREdgePredictor` (64 intention queries, 6 modes over 5.0 s horizon consuming 11-frame trajectories + 256$\times$256 RSU lane raster + pooled 256$\times$48$\times$176 WorldFusion BEV features).
   - **Measured MTR GPU Batch Latency Curve**: $N=1$: 33.7 ms; $N=2$: 43.0 ms; $N=4$: 38.8 ms; $N=8$: 43.2 ms; $N=16$: 89.3 ms; $N=32$: 220.8 ms; $N=64$: 320.7 ms; $N=128$: 567.1 ms; $N=256$: 1,453 ms.
   - **Conductor 3-Stage Deadline Controller (`WorldFusionAdaptiveEdge`)**: Combines (i) online least-squares fusion latency regression + 2D ray-cast occlusion shadow-mask scoring to select contributor count $K_{\text{cav}}^*$, (ii) transmission rate stride adaptation, and (iii) MTR temporal amortization (`_is_divergent` cache reuse) + risk-budgeted track scheduling (`risk = speed * max(proximity, conflict_prox) * occlusion`). Bounds edge compute to $\le 130\text{ ms}$ (vs. 602 ms unbounded at $N=32$) and holds p95 $\Delta_{\text{use}} \le 300\text{ ms}$ through 33 contributors.
4. **Stage 4: Closed-Loop Safety-Aware Planning (`BehaviorAgent`, `CollisionChecker`)**:
   - Merges onboard predictions with a 24-tick (1.2 s) edge hold cache (`_edge_pred_cache`), performs two-pass collision checking (bumper-to-bumper circle check + multimodal top-$K$ trajectory corridor check), executes an RSS-inspired proper response state machine (`stop`, `speed_adapt`, `ttc_graded`), and gates opposing-lane overtakes using the full 5.0 s MTR arrival horizon.

### 2.5 Multi-Locale Edge Handoff & Metropolitan Scale-Out (`Khonsu`)
- **Proactive Two-Phase Latent Migration (`SequentialMigrationDaemon`, `OwnershipManager`, `factories.py`)**:
  - Forecasts boundary crossings at lead time $L = T_{\text{xfer}} + T_{\text{fold}} + M$, pre-stages shadow tracks (`_migration_shadow = True`) on the destination edge during `prepare_handoff`, and atomically fences the source epoch (`_migration_fenced_epoch`) while unmarking the destination track on `commit_handoff`.
  - Wire payload is **440 B to 1,462 B per track** (`MambaTracklet` SSM hidden state `h`, `conv_state`, 4–10 frame `memo_bank`, and velocity `vel_mps`).
- **Measured Multi-Locale Results**:
  - Across $L=106$ locales (3,200 concurrent tracks) and pNEUMA Athens CBD metropolitan replay, continuous boundary-band replication (`replall` / `delta_ready`) wastes 71–72% of transfers on non-crossing vehicles and overloads destination prediction slack by **370–375%**, dropping 2 s post-crossing freshness to **0.416** (0.252 on pNEUMA).
  - Proactive prepare-commit consumes only **13.5% of destination compute slack** across $L=4..106$, maintains **0.994–0.996 post-crossing freshness**, and achieves **0.90 closed-loop overtake safety** ($n=50$/arm, 8–24 m/s) vs. **0.60 for cold start**.
- **Conflict-Anchored Map Partitioning (`locale_partition.py`)**:
  - Anchors locales on conflict zones and places boundaries at inter-conflict road midpoints. Across 66 boundaries in Town01 and Town03, median boundary-to-conflict separation is **21–25 m**. At arterial speed ($14\text{ m/s}$), reactive EdgeWarp ($D^* = 29\text{ m}$) serves only **33%** of boundaries and cold start ($D^* = 56\text{ m}$) serves **18%**, whereas proactive latent migration ($D^* < 1\text{ m}$) serves **100%**.

### 2.6 Summary of Evaluation Items E1–E9 for the ISPASS Paper
- **E1 (Platform Scaling, Measured from arXiv `2506.16535`)**: Scales to **256 distributed vehicle clients** without perception ($8\times$ largest comparable closed-loop platform) and **128 vehicles with perception** ($6\times$ faster than single-host baseline). Multi-node execution across 4 machines holds p99 network overhead to **45 ms at 256 clients** (no perception) and **35 ms at 64 clients** (with perception).
- **E1b (Synchronization Overhead, Measured)**: Transport and protobuf serialization remain a small fraction of step time (`ecloud_server` compute 0.28–0.49 ms; serialization 6.5–9.0 ms); barrier wait tracks the slowest actor.
- **E1c (Distribution Fidelity, Measured)**: Localization, velocity, and GNSS traces match the single-host baseline on identical scenarios.
- **E2 (Shared-Service GPU Concurrency Knee)**: Currently marked `[PLANNED]` in `evaluation.tex`, but workspace already has measured stage latency curves (`WorldFusion` $N=1..32$ reaching 602 ms; `MTR` $N=1..256$ from 33.7 ms to 1,453 ms; gRPC vs. LitServe optimization table) and single-host full-sensor CAV ceiling measurements (late-fusion cycle 85 / 143 / 251 / 470 / 970 ms for $N = 1 / 2 / 4 / 8 / 16$ CAVs; 1 full WorldFusion+Mamba+MTR CAV uses 9.8 GB of 16 GB VRAM at 260 ms/tick).
- **E3 (Fixed-Seed Reproducibility vs. Physical Timing Variance)**: `[PLANNED]` ensemble check showing identical logical world evolution and message order alongside non-zero wall-clock GPU timestamp variance.
- **E4 (Timing-Model Fidelity)**: `[PLANNED]` validation comparing composed planner-side $\Delta_{\text{use}}$ in simulation against standalone hardware service measurements under equivalent load.
- **E5 (Radio-Plane Calibration)**: Marked `[PLANNED]` in text, but data is already on disk (`ns3_uplink_lut.csv`, `ns3_downlink_lut.csv`, live 476-tick ns-3 Uu co-sim validation: UL p50 19.0 ms vs. 16.7 ms LUT, DL p50 8.5 ms vs. 9.0 ms LUT, PC5 PRR curves $N=4..64$, and SEE-V2X / 5G-MOBIX trace distributions).
- **E6 (State-Model Divergence)**: Dedicated weather/contributor ladder (`scripts/e6_sweep.sh`, arms A `oracle`, C-lat `gt_latency`, B `ego_local`, C-K$n$ `edge_worldfusion`) built; paired GT vs. sensor-derived closed-loop results already measured in SenSys (Oracle vs. Late Fusion / ETSI / VIPS across 0–500 ms delay) and Conductor/Khonsu (`freeze-1n` GT vs. `freeze-1p`/`freeze-1q` WorldFusion sensor stack).
- **E7 (Case Study 1 — Deadline Cliff & Runtime Controller, Measured)**: Unbounded edge pipeline crosses 130 ms compute allocation near 10 contributors on an A10 and p95 $\Delta_{\text{use}}$ crosses 300 ms at 17–19 contributors; Conductor controller holds compliance through 33 contributors and achieves **0.90 closed-loop success** vs. **0.10–0.48** for static, late-fusion, and V2V CMP (0.67 in 4-ego) baselines.
- **E8 (Case Study 2 — AoI Safety Envelope, Measured)**: Deterministic age sweep exposes the **100–150 ms self-ghosting logic cliff** ($P(S_{\text{op}})$ drops from 1.00 to 0.10; ETSI self-announcement drops to 0.10 at 100 ms); Self-Beacon Anchoring removes the logic cliff and extends safe operation to the **350–400 ms physical braking boundary**.
- **E9 (Case Study 3 — Cross-Edge Service Handoff, Measured)**: Proactive two-phase latent migration across edge locales with 440 B–1,462 B per-track state achieves **0.994–0.996 post-crossing freshness** at **13.5% compute slack** across 106 locales (vs. 375% overload for boundary-band replication) and **0.90 closed-loop overtake safety** (vs. 0.60 cold start).

---

## PART 3: SOTA SIMULATOR AND BASELINE AUDIT DATA

| Platform / System | Closed-Loop 3D Physics | Compute Model | Network Model | Actor & Edge Distribution | Dual State (GT + Sensor) | Max Reported Coop. Scale | Code-Verified Limitations from Workspace Audit |
|---|---|---|---|---|---|---|---|
| **OpenCDA** (ITSC 2021) | Yes (CARLA) | Ignored (0 ms) | Lossless / basic | Single host, single process | Yes | Platoon demo (~4–5 CAVs) | Serializes all CAVs under one Python GIL (`for v in vehicles: v.step()`); RSU is sensor-only with no edge compute tier or AoI accounting. |
| **V2Xverse** (ICRA 2024) | Yes (CARLA) | Injected constant frame offset | Bandwidth budget | Single host, single process | Sensor-derived | Not stated (~2–4 CAVs) | Code audit (`coop_agent.py:465`) found `ego_lidar.unsqueeze(0)` passed as both ego and peer input; peer sharing lines 406–454 commented out. |
| **AutoCast** (MobiSys 2022) | Yes (CARLA) | Offline benchmark | WINNER II + MQTT | Single host (multi-GPU) | Hybrid (GT for peers) | 3 scenarios | Non-ego cooperating vehicles use CARLA ground-truth queries rather than live LiDAR sensors in simulation mode; V2V point-cloud patches only. |
| **Coopernaut** (CVPR 2022) | Yes (CARLA) | Offline benchmark (90 ms reported, not enforced in loop) | Idealized | Single host | Sensor-derived | 2 vehicles | End-to-end V2V point-cloud control for a single pair; no edge server, tracking, trajectory prediction, or radio stack. |
| **EI-Drive** (2024) | Yes (CARLA) | Ignored | Modeled latency/error | Single host | Sensor-derived | Not stated | Single-host execution; no measured GPU service occupancy, no multi-edge handoff, and no packet-level 5G-LENA co-simulation. |
| **ms-van3t-CARLA** (2024) | Yes (CARLA) | Ignored | ns-3 C-V2X | Single host | Ground-truth / basic | Not stated | Couples CARLA and OpenCDA to ns-3 over gRPC on one host, but hosts no GPU edge service and measures no compute time. |
| **OpenCAMS** (2025) | Yes (CARLA + SUMO) | Ignored | OMNeT++ C-V2X | Single host | Ground-truth / basic | Not stated | Couples traffic and network simulators without containerized multi-GPU perception/prediction services or compute-to-age feedback. |
| **Pylot / D3** (EuroSys 2021 / NSDI 2022) | Yes (CARLA) | Measured wall-clock (1 ego) | None | Single host (ERDOS dataflow) | Both | 1 ego vehicle | Establishes measured-compute closed-loop discipline for a single vehicle, with no V2X network, no RSUs, and no cooperating actors. |
| **CADET** (2026) | Yes | Heterogeneous | Modeled | Distributed | Sensor-derived | Not stated | Lacks the unified deterministic lockstep two-time-base clock, trace-calibrated 5G-LENA radio plane, and matched GT/sensor divergence ladder. |
| **eCAV (Ours)** | Yes (CARLA 0.9.15) | Measured GPU service stamps ($\tau_{\text{compute}} \rightarrow \Delta_{\text{use}}$) | ns-3 5G-LENA (Uu+PC5) + Mode 4 MAC + SEE-V2X/5G-MOBIX traces | Containerized lockstep (`ecloud_server` C++17 + multi-edge) | Matched GT (`oracle`/`C-lat`) + Sensor-derived | 256 vehicle clients (closed loop); 106 locales / 3,200 tracks (metro replay) | Composes all five measurement requirements under one deterministic clock. |

---

## PART 4: ADVISOR PAPER WRITING GUIDE AND CHECKLIST

### 4.1 Writing Principles (`docs/paper_writing_guide.md`)
1. **Know the reader**:
   - Most reviewers stop at the introduction if they do not buy in.
   - Write for someone with zero context in your subfield.
   - Read your draft as a skeptical outside reviewer; every sentence must parse on the first pass.
2. **Abstract and introduction**:
   - Open one level of abstraction higher than feels natural.
   - Lead each paragraph with its intent (one sentence stating what the paragraph establishes).
   - Define core concepts plainly and early before building on them.
   - Order: gap, then question, then answer.
3. **Terminology and precision**:
   - Expand every acronym on first use in the body (the abstract does not count).
   - Expand acronyms to their exact standard term (e.g., RSU = "roadside unit", not "roadside sensing unit").
   - Never invent a compound term; use the literature term with a citation or a plain description.
   - Prefer simple words over technical ones when they mean the same thing (e.g., "physically plausible" instead of "kinematically plausible", "AV pipeline" instead of "autonomy pipeline").
   - Reserve precise systems terms ("correct", "consistent", "stale", "duplicate") for their exact meanings.
   - Name the version or standard for any protocol or tool modeled or measured (e.g., CARLA 0.9.15, 3GPP Mode 4 PC5 sidelink, 3GPP Release 16 NR-V2X).
4. **Sentence-level clarity**:
   - One idea per sentence.
   - **No dangling pronouns**: Never write a sentence with a dangling "this", "it", or "they". Always follow with or substitute the explicit noun.
   - Define every term (gate, threshold, window, age at use) before using it.
   - Avoid overusing "it is not X, it is Y" phrasing, semicolons, colons, bold/italics emphasis, or excessive parentheses.
5. **Structure, Claims, and Figures**:
   - Definitions come before the things that depend on them.
   - Introduce every tool, dataset, model, or metric with one line of context and a citation at first mention.
   - Say each point once; do not repeat.
   - Cite every quantitative claim that is not your own measurement.
   - Every figure and table must be referenced with an exact target (`\Cref{tab:...}`) and discussed in the text.
   - Do not over-claim or use hype words; state the mechanism and let the measured result speak.
   - Do not state what something is NOT unless a question demanded the contrast.
   - Never describe measured results as idealized or abstract.
   - Figure captions must stand alone; legends must not cover data; qualify "oracle" baselines clearly (e.g., "single-source oracle" or "ground-truth state").

---

## PART 5: VERBATIM CURRENT LATEX SOURCE (`ecav_ispass`)

### File 1: `main.tex`
```latex
\documentclass[10pt,conference,letterpaper]{IEEEtran}
\IEEEoverridecommandlockouts

% ISPASS 2027 target: 9 pages content + unlimited references, IEEE conference style,
% double blind, tool/benchmark paper track (check the tool paper box at submission).
% Do NOT include the repository URL in the submission; state that the tool will be
% open-sourced on acceptance (it already is; the URL is withheld for review).

\usepackage{amsmath,amsfonts}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{tikz}
\usetikzlibrary{positioning,arrows.meta}
\usepackage{xcolor}
\usepackage[hidelinks]{hyperref}
\usepackage[capitalise,noabbrev]{cleveref}

% TODO(anonymity): decide whether the system name stays "eCAV" under double blind.
% The arXiv preprint (2506.16535) is public under this name; concurrent preprints
% are normally tolerated, but confirm against the ISPASS 2027 CFP when it posts.
\newcommand{\sys}{eCAV\xspace}
\usepackage{xspace}
% Kishore review apparatus: visible per-paragraph intent tags until approved.
% Flip to the empty definition for the anonymized submission build.
\newcommand\ptag[1]{\noindent{\color{teal}{\bf \fbox{Intent:}}~{\bf #1}}\\}
%\newcommand\ptag[1]{}   % <- submission version: tags off

\title{eCAV: Measured Compute and Simulated Time for\\ Edge-Hosted Cooperative Autonomy}
% TODO: title is a working title. Alternative: "Simulated Time, Measured Compute:
% A Distributed Evaluation Platform for Edge-Hosted Cooperative Perception and Prediction"

% Internal review copy: real authors (eCAV arXiv roster; confirm before
% submission). The ISPASS submission itself is double blind.
\author{\IEEEauthorblockN{Tyler Landle, Jordan Rapp, Dean Blank, Chandramouli Amarnath,\\ Abhijit Chatterjee, Alexandros Daglis, Umakishore Ramachandran}
\IEEEauthorblockA{Georgia Institute of Technology}}

\begin{document}
\maketitle
\begin{center}
\fbox{\parbox{0.92\columnwidth}{\small \textbf{Draft for internal review.}
Target: ISPASS 2027 (papers due early December 2026; 9 pages + references,
double blind, tool/benchmark track). Scaling, overhead, and fidelity numbers
are measured (from the arXiv evaluation); items marked [PLANNED] are the
new experiments this submission adds. Figures not yet placed.}}
\end{center}

\input{contents/abstract}
\input{contents/introduction}
\input{contents/motivation}
\input{contents/related}
\input{contents/design}
\input{contents/implementation}
\input{contents/evaluation}
\input{contents/discussion}
\input{contents/conclusion}

\bibliographystyle{IEEEtran}
\bibliography{references}

\end{document}
```

### File 2: `contents/abstract.tex`
```latex
\begin{abstract}
Edge-hosted cooperative autonomy runs perception, tracking, and trajectory
prediction as a shared service that many connected vehicles consume. Whether
such a service is deployable turns on properties of the deployed service
under load, tail latency, accelerator contention, overload under dense
participation, and staleness at the planner, none of which is a property of
any model. No existing evaluation platform composes the ingredients needed
to measure them: closed-loop cooperative frameworks run on one host with
modeled communication, network co-simulations measure no compute, and
latency-first platforms measure one ego with no network and no cooperation.
We present \sys, an open-source distributed evaluation platform that
composes all three under one deterministic clock: containerized actors in
lock-step across machines, a deployable GPU service whose measured
wall-clock compute enters the age each planner consumes, and a packet-level
cellular radio plane calibrated against field traces. \sys separates
physical execution from logical timing, so a service slower than the sensor
period appears as a lower update rate and higher age, never as lost
simulation fidelity, and every run replays identically under a fixed seed.
We characterize the platform itself, scaling to 256 distributed vehicle
clients, synchronization overhead, service latency and jitter, and
reproducibility, and we demonstrate measurements only this composition
enables: the compute-driven deadline cliff of a cooperative prediction
service under city-scale load, the qualitative divergence between
ground-truth and sensor-derived closed-loop outcomes, and a live cross-edge
service handoff.
\end{abstract}
```

### File 3: `contents/introduction.tex`
```latex
\section{Introduction}
\label{sec:intro}

\ptag{Deployment questions are properties of a service under load, and no model benchmark can answer them.}
The systems questions that decide whether edge-hosted cooperative autonomy
is deployable are questions about a service under load. How late does a
fused update arrive when thirty vehicles contribute? What does accelerator
contention among co-located actors do to the tail? When does participation
overload the pipeline, and what does a planner do with the stale result?
None of these is a property of a perception or prediction model, so no model
benchmark can answer them. They are properties of a deployed service, its
hardware, and its network, exercised in closed loop, and they are precisely
the properties that published evaluations of cooperative autonomy do not
measure.

\ptag{Prior platforms each supply one ingredient; none composes measured compute, packet radio, and distributed actors.}
Prior evaluation platforms each supply one ingredient. Closed-loop
cooperative driving frameworks such as OpenCDA~\cite{xu2021opencda} and
V2Xverse~\cite{liu2024codriving} run every actor on one host with modeled or
injected communication, so compute contention between actors is a
measurement artifact and service time is never measured. Network
co-simulations such as ms-van3t-CARLA~\cite{msvan3tcarla2024} put
packet-level radio inside the driving loop but host no service and measure
no compute. Latency-first platforms, Pylot~\cite{gog2021pylot} and
D3~\cite{gog2022d3}, measure real wall-clock compute while driving closed
loop, for a single ego with no network and no cooperating actors. Our
system-by-system review found no platform that composes measured compute,
packet-level radio, and distributed cooperative actors under one
deterministic clock.

\ptag{The central mechanism separates physical execution from logical timing, so a slow service appears as age, never as lost fidelity.}
\sys is an open-source distributed evaluation platform built around that
composition. Its central mechanism is the separation of physical execution
from logical timing: a central orchestration server advances the world over
a per-tick barrier, every service invocation is stamped with its measured
wall-clock duration, and the service's logical occupancy is derived from
those stamps. A 130\,ms inference admitted at simulated time $t$ makes its
result visible at $t+130$\,ms, frames released inside the busy interval are
superseded rather than queued, and the world's determinism is untouched: a
service slower than the sensor period appears in the measurements exactly as
it would in deployment, as a lower update rate and higher age, never as lost
simulation fidelity.

\ptag{Four contributions: the execution model, the platform characterization, the measurements it enables, and the artifact.}
This paper makes four contributions:
\begin{enumerate}
  \item \textbf{Design.} An execution model that composes deterministic
  lock-step across distributed, containerized actors with measured
  accelerator service time and a packet-level cellular radio plane, and
  matched ground-truth and sensor-derived state models for the same
  scenario (\Cref{sec:design}).
  \item \textbf{Platform characterization.} Scaling to 256 distributed
  vehicle clients, roughly 8$\times$ the largest comparable closed-loop
  cooperative platform, with measured synchronization overhead, service
  latency and jitter, and fixed-seed reproducibility
  (\Cref{sec:eval}).
  \item \textbf{Measurements the composition enables.} The compute-driven
  deadline cliff of an edge-hosted cooperative prediction pipeline under
  city-scale load, the divergence between ground-truth and sensor-derived
  closed-loop outcomes, and a live two-edge service handoff
  (\Cref{sec:eval}).
  \item \textbf{Artifact.} The platform, scenario corpus, and experiment
  harness are open source; the URL is withheld for double-blind review and
  will be released with the paper.
\end{enumerate}
```

### File 4: `contents/motivation.tex`
```latex
\section{Background and Motivation}
\label{sec:motivation}

\subsection{The workload}
\ptag{The workload is a shared edge service with a three-orders-of-magnitude payload spectrum, and the figure of merit is planner-side age.}
Edge-hosted cooperative autonomy places a shared service at a Multi-Access
Edge Computing (MEC) site near the radio network. Connected vehicles and
roadside units (RSUs) contribute observations; the service fuses them,
tracks every vehicle and pedestrian in a road region, predicts each actor's
trajectory, and returns the result to every participating planner. The
payload spectrum spans three orders of magnitude: basic safety messages of
roughly 300\,B, object lists of 1--5\,KB, and compressed intermediate
feature maps of roughly 17\,KB per vehicle per frame. The figure of merit is
not any stage's accuracy in isolation but the age of information the planner
acts on: the elapsed time from sensor capture through encoding,
transmission, fusion, tracking, prediction, and delivery. Measured freshness
budgets for closed-loop driving fall between 220 and 450\,ms depending on
the maneuver, so tens of milliseconds of compute or queueing anywhere in the
chain are material.

\subsection{Why small-scale and single-host study fails}
\ptag{Single-host and small-scale evaluation fails three ways: contention artifacts, missing tails, and oracle shortcuts.}
Three failure modes make the obvious evaluation shortcuts unsound. First,
contention artifacts: when every actor runs on one host, actors contend for
the same cores and accelerator, so measured latencies reflect the harness,
not the deployment, and the error grows exactly where the measurements
matter, at high participation. Second, missing tails: the deployment
questions live at p95 and p99 under load, which small-scale runs never
exercise. Third, oracle shortcuts: evaluations that read actor state from
the simulator bypass the perception error, detection latency, and
association noise that a deployed service consumes; as we show in
\Cref{sec:eval}, closed-loop outcomes change qualitatively when
sensor-derived perception replaces ground truth, so conclusions drawn under
oracle state need not transfer.

\subsection{Measurement requirements}
\ptag{The failure modes yield five measurement requirements, and no prior platform satisfies more than two.}
These failure modes translate into five requirements for an evaluation
instrument. (1)~Determinism and replay: identical world evolution and
message order under a fixed seed, so configurations are comparable and runs
are debuggable. (2)~Measured service time: the compute cost of the service
under test must be the wall-clock cost of the real artifact on real
hardware, not a modeled constant. (3)~Packet-level communication: delivery,
loss, and delay must come from a radio model exercised per message, not from
an injected distribution. (4)~Distribution: actors must run as separate
processes, on separate machines when the experiment requires it, so
contention is a measured phenomenon rather than an artifact.
(5)~Dual state models: the same scenario must run under ground-truth state,
sensor-derived state, or a mix, so the cost of real perception is itself
measurable. No prior platform satisfies more than two of these
simultaneously (\Cref{sec:related}).
```

### File 5: `contents/related.tex`
```latex
\section{Related Work}
\label{sec:related}

\ptag{Closed-loop cooperative frameworks run on one host with modeled communication and unmeasured compute.}
\textbf{Closed-loop cooperative driving frameworks.}
OpenCDA~\cite{xu2021opencda} established closed-loop cooperative driving on
CARLA, modeling V2X as lossless transfer and naming network simulation as
future work. V2Xverse~\cite{liu2024codriving} adds roadside units and online
closed-loop evaluation of an end-to-end driving system, still on one host
with a bandwidth budget and injected delays applied to a frozen model.
AutoCast~\cite{Autocast} and Coopernaut~\cite{coopernaut} schedule and learn
V2V sharing in closed loop; Coopernaut reports 90\,ms inference measured
offline on a desktop GPU rather than enforced in the loop. EI-Drive
\cite{eidrive2024} injects modeled transmission latency and errors into
closed-loop perception. None of these runs distributed actors, measures
compute on real hardware, or hosts a deployable edge service.

\ptag{Open-loop toolkits have no loop, no network, no service; \sys consumes them as sources.}
\textbf{Open-loop toolkits and datasets.}
OpenCOOD, released with OPV2V~\cite{xu2022opencood}, benchmarks cooperative
perception models over recorded scenes; V2X-Sim~\cite{v2xsim} caps
cooperating agents at six and defers latency simulation; DAIR-V2X
\cite{dairv2x} and Multi-V2X~\cite{multiv2x2024} provide multi-agent
recordings. None closes the loop, models the network, or runs a service;
\sys consumes tooling of this class as scenario and dataset sources.

\ptag{Network co-simulations put packet radio in the loop and measure no compute.}
\textbf{Network co-simulation.}
ms-van3t-CARLA~\cite{msvan3tcarla2024} bridges CARLA and OpenCDA to ns-3's
C-V2X models over gRPC, and OpenCAMS~\cite{opencams2025} couples SUMO,
CARLA, and OMNeT++. The federated traffic simulators, Veins and Eclipse
MOSAIC, put packet-level radio under traffic models without perception.
These confirm that network behavior belongs inside the loop; none measures
compute or hosts a service, so network delay never composes with service
time into the age a planner consumes.

\ptag{Latency-first platforms measure real compute for one ego; \sys extends the discipline to a distributed networked service.}
\textbf{Latency-first platforms.}
Pylot~\cite{gog2021pylot} measures per-component wall-clock latency while
driving CARLA closed loop, and D3~\cite{gog2022d3} schedules the pipeline
under dynamic deadlines, both for a single ego with no network and no
cooperating actors. Hardware-in-the-loop work decoupled simulated time from
wall clock for one vehicle. These establish the measured-compute discipline
that \sys extends to a distributed, multi-actor, network-mediated service.

\ptag{CADET is the closest recent platform; the distinction is the composed tuple under one deterministic clock.}
\textbf{Closest recent platform.}
CADET~\cite{cadet2026} targets distributed cooperative autonomy with modular
deployment, compute heterogeneity, and task-level outcomes. \sys differs in
the combination it composes under one deterministic clock: lock-step
replayability, a packet-level radio plane, and a deployable GPU service
whose measured compute enters planner-side age.
\Cref{tab:platforms} summarizes.

\begin{table*}[t]
\centering
\caption{Evaluation platforms for cooperative autonomy. \sys composes the
columns no prior platform composes. (n.s. = not stated.)}
\label{tab:platforms}
\small
\begin{tabular}{llllll}
\toprule
System & Loop & Network model & Compute model & Distribution & Coop.\ scale \\
\midrule
OpenCDA~\cite{xu2021opencda} & closed & lossless & ignored & single host & platoon demo \\
V2Xverse~\cite{liu2024codriving} & closed & bandwidth budget & injected delay & single host & n.s. \\
AutoCast/Coopernaut~\cite{Autocast,coopernaut} & closed & Winner II + MQTT & offline benchmark & multi-GPU host & 3 scenarios \\
EI-Drive~\cite{eidrive2024} & closed & modeled latency/error & ignored & single host & n.s. \\
OpenCOOD/V2X-Sim~\cite{xu2022opencood,v2xsim} & open & none & none & n/a & $\leq$6 agents \\
ms-van3t-CARLA~\cite{msvan3tcarla2024} & closed & ns-3 C-V2X & ignored & single host & n.s. \\
OpenCAMS~\cite{opencams2025} & closed & OMNeT++ C-V2X & ignored & single host & n.s. \\
Pylot/D3~\cite{gog2021pylot,gog2022d3} & closed & none & measured, 1 ego & single host & 1 ego \\
CADET~\cite{cadet2026} & closed & modeled & heterogeneous & distributed & n.s. \\
\midrule
\textbf{\sys} & closed & ns-3 5G-LENA, trace-calibrated & measured service & containerized, lock-step & 256 clients \\
\bottomrule
\end{tabular}
\end{table*}
```

### File 6: `contents/design.tex`
```latex
\section{Design}
\label{sec:design}

\subsection{Execution model: one clock, two time bases}
\label{sec:design-time}
\ptag{A per-tick barrier across all distributed actors makes world evolution and message order deterministic under a fixed seed.}
A central orchestration server owns the clock. Every actor, each vehicle
client in its own process or container, each RSU, the edge service, and the
scenario engine, registers with the server at startup and joins a per-tick
barrier: the server broadcasts tick $t$, each actor consumes its sensor data
for that tick and runs its own perception, planning, and control, each actor
reports completion, and only then does the server step the world by one
fixed 50\,ms interval and broadcast $t+1$. Because no actor sees $t+1$
before every actor finishes $t$, message order is fixed regardless of how
fast individual workers run, and a run with dozens of distributed actors
replays identically under a fixed seed. Determinism here means world
evolution and message order; accelerator wall-clock varies across runs and
enters the system only as measured stamps.

\ptag{Measured stamps define logical service occupancy, so backlog appears as update rate and age, never as lost fidelity.}
Physical execution and logical timing are separate. Physically, each
service invocation runs to completion inside its own barrier slot, so the
world always advances. Logically, every invocation is stamped with its
measured wall-clock duration, and the service's occupancy derives from those
stamps: an invocation admitted at simulated time $t$ and measured at
130\,ms occupies the service until simulated $t+130$\,ms, the frames
released at $t+50$ and $t+100$\,ms are superseded rather than queued, the
next admission is the first frame after $t+130$\,ms, and the result becomes
visible to consumers at $t+130$\,ms. A service slower than the frame period
therefore appears in the measurements exactly as it would in deployment, as
a lower update rate and higher age, never as lost simulation fidelity.
Data-plane delivery composes both time bases: a message reaches its consumer
after its producer's measured compute plus the radio delay the network plane
assigns to that transmission.
% TODO: timeline figure (admission at t, 130ms service, releases at t+50/t+100
% superseded, visibility at t+130).

\subsection{Distributed containerized actors}
\ptag{Each actor is a process or container on any machine, so contention is a measured phenomenon rather than an artifact.}
Each actor is a separate process, optionally a Docker container, placed on
any machine; the barrier protocol is the only coupling. Distribution is what
makes contention a measured phenomenon: co-locating actors is a
configuration choice whose cost the platform reports, not an artifact the
platform imposes. The synchronization protocol is designed for hundreds of
agents; \Cref{sec:eval} measures the achieved scaling.

\subsection{Pluggable service model}
\ptag{Every pipeline stage sits behind one swappable interface, and the service artifact is the one a deployment would run.}
The perception service is a first-class module behind an explicit interface:
a scenario names its backend in configuration, and the platform serves it.
Perception runs per vehicle, as a shared GPU service multiplexing many
clients, or as ground-truth injection. The same interface extends through
the stack: intermediate-fusion backends, classical and learned tracking,
multi-modal trajectory predictors, and closed-loop planners swap without
changing the simulator, so an experiment holds the scenario fixed and varies
exactly the component under study. The service artifact is the one a
deployment would run.

\subsection{Dual state models}
\ptag{The same scenario runs under ground-truth or sensor-derived state, making the divergence a measurement the platform produces.}
The same scenario runs under ground-truth state, queried directly from the
simulator, under sensor-derived state, what each vehicle's sensors and
models extract, or under a mix. The two diverge precisely in the occluded,
contended situations edge assistance targets, and the divergence is a
measurement the platform produces rather than a threat to validity it
ignores (\Cref{sec:eval}).

\subsection{Radio plane}
\ptag{The radio plane assigns delay per message on every path and is calibrated against field traces.}
A packet-level cellular model assigns delay, delivery, and loss per message:
per-cell scheduled uplink (Uu) and contention-based sidelink (PC5), with
every data-plane path crossing the radio plane whether traffic flows through
the edge service or directly between actors. Latency models are calibrated
against public field-trial traces.

\subsection{Deterministic scenario engine}
\ptag{Scenarios compile with determinism checks and replay identically under a fixed seed.}
Scenarios are specified hierarchically and compiled before execution, with
static checks for determinism hazards such as nondeterministic actor
ordering and race-prone triggers; layouts and traffic draw from established
cooperative perception datasets. A scenario that passes compilation replays
identically under a fixed seed.
```

### File 7: `contents/implementation.tex`
```latex
\section{Implementation}
\label{sec:impl}
\ptag{Implementation names live in one place; the design sections describe components by role.}
The orchestration server is a multithreaded C++ process; all control-plane
and data-plane messaging uses gRPC with protobuf schemas. CARLA 0.9.15 is
the world engine, stepped synchronously at 50\,ms. Vehicle and RSU actors
run as Python processes or Docker containers; the shared perception service
runs on LitServe, multiplexing one GPU across many clients; the radio plane
is ns-3 with the 5G-LENA NR V2X module, co-simulated per message. Reference
backends span YOLOv5 and intermediate-fusion detection (Where2comm-family),
AB3DMOT and state-space-model tracking, Motion-Transformer-class prediction,
and rule-based and learned planners. The platform is open source; the URL is
withheld for review.
```

### File 8: `contents/evaluation.tex`
```latex
\section{Evaluation}
\label{sec:eval}

% Status notes for Tyler's review:
%   E1/E1b/E1c: real numbers from the arXiv evaluation (data + scripts in the
%     sandbox repo; figures regenerate without re-running).
%   E2-E6: planned; marked [PLANNED] inline. E5 inputs exist on disk.
%   E7-E9: real numbers from the dissertation results, summarized.

\ptag{Two questions: what the instrument itself costs, and what only it can measure.}
Two questions structure the evaluation. First, what does the instrument
itself cost: how fast does it run, how far does it scale, what does
synchronization overhead amount to, and do runs reproduce? Second, what can
it measure that no prior platform can: the compute cliff of a deployed
cooperative service, the outcome divergence between state models, and
cross-edge continuity.

\subsection{Platform characterization}
\label{sec:eval-platform}

\ptag{Takeaway: 256 distributed clients, linear scaling, roughly 8x the largest comparable platform.}
\textbf{Scaling.} Without perception, \sys runs 256 vehicle clients with
individual control algorithms, roughly 8$\times$ more than the largest
comparable closed-loop cooperative platform, and client step time scales
linearly where the GIL-serialized baseline degrades multiplicatively. With
per-vehicle perception enabled, \sys runs 128 vehicles approximately
6$\times$ faster than the state of the art. Multi-node execution across four
client machines holds 99th-percentile network overhead to 45\,ms at 256
clients without perception and 35\,ms at 64 clients with perception.
% Figures: regenerate from cumulative_stats_* via create_eval_graphs.py.

\ptag{Takeaway: transport overhead stays small; barrier wait grows with the slowest client.}
\textbf{Synchronization overhead.} Transport and serialization overhead is a
small fraction of client step time across the range; barrier wait grows with
the slowest client as N rises, and the breakdown attributes it.

\ptag{Takeaway: distribution preserves simulation accuracy against the single-host baseline.}
\textbf{Fidelity.} On a matched scenario, localization, velocity, and GNSS
traces are practically identical to the single-host baseline, so
distribution preserves simulation accuracy.

\ptag{Planned: fixed-seed ensembles separating logical reproducibility from physical timing variance.}
\textbf{Reproducibility. [PLANNED]} Fixed-seed ensembles verifying identical
world evolution and message order across repeats, with the variance of
measured wall-clock stamps reported alongside, demonstrating that logical
results reproduce while physical timing varies.

\ptag{Planned: the central validation, composed planner-side age against standalone measurement.}
\textbf{Timing-model fidelity. [PLANNED]} The central validation: composed
planner-side age in simulation compared against the same service measured
standalone under equivalent load on the same hardware.

\ptag{Planned: the calibration figure from already-collected traces.}
\textbf{Radio calibration. [PLANNED]} ns-3 5G-LENA latency and delivery
distributions against public field-trial traces; inputs collected, figure
pending.

\subsection{Shared-service concurrency [PLANNED]}
\ptag{Planned: the clients-per-GPU knee on MEC-representative hardware.}
Clients per GPU versus p50/p95/p99 latency and jitter on the shared
perception service on MEC-representative hardware, locating the knee where
tail latency departs from its small-scale trend under full-city load.

\subsection{State-model divergence [PLANNED]}
\ptag{Planned: the contributor-count ladder between the oracle and the full sensor-derived stack.}
Matched scenarios under ground-truth and under sensor-derived state, with a
contributor-count ladder between the oracle and the full sensor-derived
stack and a latency-stamped ground-truth control separating the information
channel from the timing channel. Outcome deltas, first-detection distance,
and per-arm age quantify how far conclusions drawn under oracle state
transfer.

\subsection{Case studies}
\label{sec:eval-cases}
\ptag{Three completed studies demonstrate measurements only the composition enables.}
Three studies, each published or under submission with full detail
elsewhere, demonstrate measurements only this composition enables.

\ptag{Takeaway: the cliff and its controller require measured compute, per-cell radio, and distribution simultaneously.}
\textbf{Deadline cliff and runtime control.} An edge-hosted cooperative
prediction pipeline, intermediate fusion, multi-object tracking, and
trajectory prediction, crosses its 130\,ms compute allocation near 10
contributors on an A10, and p95 planner-side age crosses the 300\,ms
operating point between 17 and 19 contributors; a deadline-aware controller
holds compliance through 33 contributors, and in closed loop the controlled
service succeeds in 0.90 of occluded-conflict runs against 0.10--0.48 for
static, late-fusion, and per-ego baselines. Both the cliff and the
controller evaluation require measured service time, per-cell radio, and
distributed actors simultaneously.

\ptag{Takeaway: the envelope requires deterministic replay of identical scenarios across age settings.}
\textbf{Age-of-information safety envelope.} Sweeping delivered age with
deterministic injection while holding perception at ground truth exposes a
logic-induced failure cliff at 100--150\,ms, an identity contract removes
it, and the physics boundary appears at 350--400\,ms. The experiment
requires deterministic replay of identical scenarios across age settings.

\ptag{Takeaway: cross-edge continuity requires multi-edge distribution no single-host platform expresses.}
\textbf{Cross-edge service handoff.} Two edge services in one simulation,
each running the full pipeline, exchange per-track temporal context ahead of
a vehicle's crossing, with warm resume in both directions; the mechanism
depends on multi-edge distribution no single-host platform can express.
```

### File 9: `contents/discussion.tex`
```latex
\section{Discussion and Limitations}
\label{sec:discussion}
\ptag{Scope: the fidelity boundary, hardware dependence, the determinism definition, and why traces cannot substitute.}
\sys's fidelity boundary is deliberate: rendering and sensor physics belong
to the underlying driving simulator, and the platform's contribution is the
systems layer above it. Compute results are reported for the hardware that
produced them, a development-class RTX 4080 and an MEC-representative A10;
the cliff positions move with hardware, the measurement methodology does
not. Determinism covers world evolution and message order under a fixed
seed, not bitwise-identical accelerator timing, which is why measured stamps
rather than repeated timing define logical occupancy. Trace-based traffic
simulators cannot substitute for the closed loop: a replayed trajectory
cannot react to an intervention, so the effect of a service on driving
behavior is undefined in a trace.
```

### File 10: `contents/conclusion.tex`
```latex
\section{Conclusion}
\label{sec:conclusion}
Deployment questions about edge-hosted cooperative autonomy are performance
questions about a service under load, and answering them requires an
instrument that composes what prior platforms hold separately. \sys
composes deterministic lock-step execution across distributed containerized
actors, measured accelerator service time that enters planner-side age,
packet-level trace-calibrated radio, and matched ground-truth and
sensor-derived state models. The platform scales to 256 distributed vehicle
clients, and the measurements it enables, the deadline cliff of a
cooperative prediction service, the outcome divergence between ground-truth
and sensor-derived state, and live cross-edge handoff, are measurements no
prior platform could produce. The platform, scenario corpus, and experiment
harness will be released with the paper.
```
