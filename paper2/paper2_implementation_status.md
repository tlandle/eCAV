# Paper 2: V2V Cooperative Perception Implementation Status

## Date: March 18, 2026

---

## 1. What We Built

### 1.1 C-V2X Channel Engine (`ecav/core/networking/`)

A standalone channel model for C-V2X Mode 4 / NR-V2X PC5 sidelink, replacing the idealized distance-only radio models used in prior work (AutoCast, V2XVerse).

**Components:**
- `channel_engine.cpp` / `channel_engine.h` -- C++ implementation with pybind11 bindings
- `channel_engine_py.py` -- Pure Python fallback (functionally identical, ~50-100x slower for large N)
- `occlusion_model.py` -- CARLA ray-cast based 3-level occlusion (clear LOS / vehicle-blocked / building-blocked)
- `CMakeLists.txt`, `bindings.cpp`, `build.sh` -- Build infrastructure

**Channel model details:**
- **Propagation**: WINNER+ B1 urban microcell (3GPP TR 36.885), extracted from ns-3 CNI implementation
  - LOS: `22.7*log10(d) + 27.0 + 20*log10(fc)` below breakpoint, dual-slope above
  - NLOS: `(44.9 - 6.55*log10(hbs))*log10(d) + 5.83*log10(hbs) + 18.38 + 23*log10(fc) - 5`
  - Vehicle-blocked: LOS base + 15 dB per blocking vehicle body (Boban et al., IEEE TAP 2014)
  - Building-blocked: full NLOS model
- **Fading**: Rician (LOS, K=3dB), Rayleigh (NLOS), log-normal shadow fading
- **MAC**: SB-SPS with reselection counters (RC_min=5, RC_max=15, p_keep=0.0)
- **SINR**: Per-link computation with co-channel interference from all same-subchannel transmitters
- **Delivery**: SINR threshold (default 0 dB) determines packet delivery

**Validated PRR degradation curve (Python fallback, M=20 subchannels):**

| N vehicles | PRR   | CBR   | Mean SINR |
|-----------|-------|-------|-----------|
| 4         | 0.917 | 0.000 | +8.9 dB   |
| 8         | 0.857 | 0.129 | +15.3 dB  |
| 16        | 0.667 | 0.294 | +10.1 dB  |
| 32        | 0.433 | 0.713 | -2.9 dB   |
| 64        | 0.269 | 0.823 | -10.3 dB  |

This is the network saturation cliff: PRR drops from 0.92 at N=4 to 0.27 at N=64.

**Backhaul queuing model** for edge path comparison: `delay_ms = (N * feature_bytes * 8 / backhaul_bw) * 0.5`

### 1.2 Three V2V Manager Implementations (`ecav/core/application/v2v/`)

All three implement the same lifecycle interface as edge managers (`add_member`, `add_rsu`, `set_destination`, `start_edge`, `update_information`, `run_step`) so they plug into the existing simulation loop. None inherit from `_BaseEdgeManager` because V2V is architecturally peer-to-peer, not centralized.

#### A. Intermediate Fusion V2V (`v2v_manager.py`, `manager_type: v2v_coop`)

BM2CP cooperative perception model with per-vehicle fusion over C-V2X sidelink.

**Pipeline per tick:**
1. Each vehicle/RSU extracts BEV spatial features locally (BM2CP perception manager)
2. CARLA ray-cast occlusion matrix for all N*(N-1)/2 pairs
3. C-V2X channel engine computes delivery decisions per directed link
4. Per-vehicle: collect self + delivered peer features
5. Per-vehicle: BM2CP `fusion_net` (backbone + Where2comm attention)
6. Per-vehicle: VoxelPostprocessor (ego-centric, 32m crop)
7. Per-vehicle: AB3DMOT tracking
8. Per-vehicle: LinearPredictor (KF velocity extrapolation)
9. Each vehicle receives its own predictions

**Key finding: Visibility-transfer failure.**
The RSU at (-82, 126) has its BM2CP features encoded with the Lincoln's signature once the Lincoln enters the RSU's 32m BEV crop (~frame 55). The features are transmitted to the ego and fused. But the VoxelPostprocessor crops the fused output to 32m around the ego. At frame 55, the ego is at y~94. The Lincoln at y=128 is 34m away in y. Outside the 32m box crop. The Lincoln's information is present in the fused features but gets discarded by ego-centric post-processing.

The Lincoln first appears as a true positive detection at frame 102, when the ego is at y=119.6 and the Lincoln is at (-82.9, 128.6), ~9m away. The collision occurs ~18 frames later. Not enough time for track maturity (10 frames for anonymous tracks) + prediction + braking.

**GT diagnostic output confirmed:**
- 40 detections per frame, but only 5-6 are true positives (firetrucks at known positions)
- Lincoln never appears as TP until frame 102 (ego at y=119.6)
- FP scores (0.2-1.0) overlap completely with TP scores (0.3-1.0), no separable threshold
- BM2CP 32m detection range is the bottleneck, not detection quality

#### B. Early Fusion V2V / AutoCast-style (`v2v_early_fusion_manager.py`, `manager_type: v2v_autocast`)

Object-level point cloud sharing with MCKP bandwidth-aware scheduling.

**Pipeline per tick:**
1. Each agent runs local detection (GT-based, matching AutoCast's simulation mode)
2. Detected objects represented as point cloud segments + bounding boxes
3. MCKP greedy scheduler selects objects to transmit under bandwidth budget (6 Mbps, 100ms slot)
4. Objects scored by occlusion-based utility: objects not visible to receiver get utility=1.0
5. C-V2X channel determines delivery per link
6. Receiver concatenates delivered point clouds into own LiDAR frame
7. Re-runs detection on fused point cloud (GT-based for now)
8. AB3DMOT tracking + linear prediction

**MCKP Scheduler** (`mckp_scheduler.py`):
- Greedy knapsack: sort objects by utility/transmission_time ratio, select until budget exhausted
- Utility function: 1.0 if receiver hasn't detected object at that position, 0.0 if already visible
- Transmission time: `n_points * 12 bytes * 8 / rate_mbps / 1e6 * 1000` ms
- Default budget: 100ms at 6 Mbps effective rate

**Key advantage over intermediate fusion**: Avoids ego-centric BEV crop problem. Shared points are in world coordinates, transformed to receiver's frame before voxelization. Detection range effectively expands to cover sender's observations.

#### C. Harbor-style Hybrid V2I+V2V (`harbor_manager.py`, `manager_type: harbor`)

Based on Harbor (Zhu et al., SenSys 2024). V2I-primary with opportunistic V2V relay. Edge server performs point cloud merging and detection.

**Architecture:**
- Vehicles classified as **helpers** (V2I bandwidth > 1 Mbps) or **helpees** (poor V2I)
- Helpees relay point clouds to nearest helper via V2V (C-V2X PC5 sidelink)
- Helpers upload own + relayed point clouds via V2I (cellular Uu interface)
- **Edge server** (NOT at RSU, through backhaul) merges point clouds and runs detection
- Edge returns detection results to vehicles via V2I downlink
- Vehicles run local detection in parallel; merge if edge results arrive before 500ms E2E deadline

**Key path**: Vehicle -> V2V relay -> Helper -> V2I uplink -> backhaul -> edge server -> detection -> backhaul -> V2I downlink -> Vehicle

**Harbor-specific parameters:**
- E2E deadline: 500ms (from Harbor paper)
- Helper bandwidth threshold: 1 Mbps (from Harbor empirical measurements)
- V2I bandwidth: heterogeneous, mean=12 Mbps, std=8 Mbps (from Harbor Fig. 1 measurements)
- BW_req per vehicle: 4.8 Mbps at 10fps with encoding
- Helper-helpee assignment: updated every 100ms via weighted bipartite matching
- Edge compute time: ~50ms for point cloud merge + PointPillars detection

**Key difference from our edge system**: Harbor uses early fusion (raw point clouds) at the edge, not intermediate fusion (learned features). Different compute profile. Our edge system uses BM2CP/WorldFusion intermediate features with SSM tracking and multimodal prediction.

### 1.3 Supporting Infrastructure

- `v2v_fusion.py` -- Feature exchange module (confidence masking, delivery map construction)
- `v2v_metrics.py` -- PRR, CBR, AoI, feature delivery fraction tracking per tick
- `mckp_scheduler.py` -- MCKP greedy bandwidth scheduler for AutoCast baseline

### 1.4 Scenario Configurations and Launchers

Created for each manager type, all using the same Scenario 3 intersection in Town03:

| File | Manager | Description |
|------|---------|-------------|
| `openscenario_3_v2v_coop.yaml` + `.py` | `v2v_coop` | Intermediate fusion V2V (BM2CP) |
| `openscenario_3_v2v_autocast.yaml` + `.py` | `v2v_autocast` | Early fusion V2V (AutoCast-style) |
| `openscenario_3_harbor.yaml` + `.py` | `harbor` | Hybrid V2I+V2V (Harbor-style) |

### 1.5 Integration Points Modified

- `ecav/scenario_testing/utils/sim_api.py` -- `_select_edge_manager()` routes `v2v_coop`, `v2v_autocast`, `harbor` to the correct manager class
- `ecav/core/application/v2v/__init__.py` -- Package exports with lazy import for `V2VCooperativeManager` (avoids carla import at module level)
- `ecav/core/networking/__init__.py` -- Try/except for carla-dependent occlusion module

### 1.6 BM2CP Perception Manager Fix

`ecav/core/sensing/perception/bm2cp_perception_manager.py` line 179: Changed `record_len` from `N_cams` (4) to `1`. The BM2CP scatter operation interprets `record_len` as the number of agents, not cameras. With `record_len=4`, it tried to split the single vehicle's voxels across 4 agent slots, leaving 3 empty ("SCATTER BUG" in logs).

---

## 2. Key Findings

### 2.1 Visibility-Transfer Failure (Intermediate Fusion V2V)

**The core discovery**: Standard cooperative BEV stacks can fail to convert remote cooperative visibility into planner-visible objects when post-processing is ego-centric and range-limited.

- The RSU acquires the Lincoln's features once it enters the RSU's 32m BEV crop (~frame 55)
- These features are successfully transmitted to the ego over the channel
- BM2CP fusion_net processes them correctly
- But the VoxelPostprocessor's 32m ego-centric crop discards the Lincoln because it's >32m from the ego at that time
- Communication cost is paid with zero planner utility
- The Lincoln is only detected at frame 102, 0.9 seconds before collision. Not enough for track maturity + prediction + braking.

**This is NOT a detection quality problem.** When the Lincoln enters the ego's 32m crop, it's detected with score=1.000, 3.6m match distance. The model works fine. The architecture discards the cooperative information before it can be used.

### 2.2 BM2CP 32m Range vs V2X-Sim 2.0 Dataset

The BM2CP model was trained with `cav_lidar_range: [-32, -32, -3, 32, 32, 1]`. Analysis of V2X-Sim 2.0 training data:

| Metric | Value |
|--------|-------|
| Inter-agent distance | min=16.8m, max=158.7m, mean=49.3m, median=37.4m |
| GT box distance from ego | p90=63.0m, p99=69.4m |
| GT boxes beyond 32m | 62.2% (6077/9773) |
| GT boxes beyond 64m | 9.3% (905/9773) |

62% of GT boxes in the training data are beyond 32m from ego. The model was trained on a narrow slice. High AP during validation is because `gt_range` also uses 32m, so those 62% of boxes are excluded from evaluation entirely.

Reference BM2CP config (OpenV2V) uses `[-140.8, -38.4, -3, 140.8, 38.4, 1]`. HEAL on V2X-Sim also uses 32m but accepts this limitation.

### 2.3 AutoCast Architecture Analysis

AutoCast's cooperating vehicles use CARLA GT queries (simulation mode) for detection, not real LiDAR sensors. The sensor setup code for non-ego actors is commented out. Only the ego has a real LiDAR sensor. Cooperating vehicles know where objects are from GT and share that knowledge. The channel model, scheduling, and bandwidth constraints are the variables under test.

AutoCast succeeds in their intersection scenario because:
1. Cooperating vehicles have GT-perfect detection of the collider
2. Point cloud sharing happens BEFORE voxelization (early fusion), so no ego-centric crop problem
3. 70m detection range per agent, 100m communication range
4. MCKP scheduler prioritizes occluded objects

### 2.4 How V2XVerse "Closed Loop" Works

V2XVerse's cooperative agent (`coop_agent.py` line 465) passes the ego's own LiDAR as both ego and "other" input: `self.model(ego_lidar, ego_speed, ego_lidar.unsqueeze(0), other_transform.unsqueeze(0))`. The actual cooperative data sharing code is commented out (lines 406-454). V2XVerse's closed-loop cooperative mode does not actually receive real other-vehicle data in the simulation loop. Their end-to-end model directly maps perception to controls with a 36x12m detection range.

### 2.5 Harbor Architecture (from paper, SenSys 2024)

Key details confirmed from reading the paper:
- **Edge is NOT at the RSU**. Path: Vehicle -> V2V relay -> Helper -> V2I uplink (cellular) -> backhaul -> edge server -> detection -> backhaul -> downlink -> Vehicle
- Edge server runs PointPillars via OpenCOOD with 50m crop per vehicle
- V2I bandwidth: 5.4-55.9 Mbps (LTE/5G traces), highly heterogeneous (Fig. 1 shows 8-15s periods of 0-1 Mbps)
- V2V bandwidth: ~12 Mbps (802.11g)
- E2E latency threshold: 500ms
- BW_req = 4.8 Mbps per vehicle at 10fps
- MAC-layer prioritization: detection results (downlink) get higher priority than sensor upload
- Real-world evaluation at Mcity testbed with 3 Lincoln MKZ vehicles
- Outperforms EMP and AVR by 12-36% detection accuracy, 39% latency improvement

---

## 3. What Remains To Do

### 3.1 Scenario 4: AutoCast-style Occluded Intersection

Scenario XML and Python behavior tree created but need refinement:
- **Current issue**: Parked cars were listed as potential cooperating nodes. That's unrealistic.
- **Fix needed**: Add a cooperating CAV (a second equipped vehicle driving through the area with line-of-sight to the collider). Remove parked cars from the cooperating set. The cooperating CAV needs to be positioned where it can see the collider but the ego cannot (because of the truck wall).
- For V2I modes (Harbor, edge), the RSU at the intersection provides the cooperative perception.
- For V2V modes (AutoCast, BM2CP), the cooperating CAV provides it.
- Parked cars remain as non-participating scene objects.

### 3.2 Retrain BM2CP with Wider Range

The 32m BEV crop makes intermediate fusion V2V useless in this geometry. Options:
1. Retrain with `[-70.4, -70.4, -3, 70.4, 70.4, 1]` (covers 99% of V2X-Sim 2.0 GT)
2. Retrain with `[-140.8, -38.4, -3, 140.8, 38.4, 1]` (reference BM2CP config)
3. Accept 32m as the HEAL baseline and show it fails (paper argument)

### 3.3 Replace GT Detection in AutoCast/Harbor Baselines

Both currently use CARLA GT for detection to isolate channel/scheduling effects. For fair comparison need:
- AutoCast: real LiDAR processing (PointPillars) on the cooperating CAV, or keep GT and be explicit about it in the paper
- Harbor: real PointPillars at edge (currently GT)

### 3.4 Profiling and Timing Instrumentation

The V2V managers lack per-component timing. Need to add:
- Occlusion ray cast time
- Channel engine time
- Per-vehicle fusion time
- Detection time
- Tracking time
- Prediction time
- Total tick time

### 3.5 N-Sweep Evaluation

Run all manager types across N = {4, 8, 16, 32, 64} to show:
- V2V PRR degradation curve (already validated offline)
- Edge compute cliff (from paper 2 thesis)
- AutoCast bandwidth saturation
- Harbor helper/helpee ratio under load

### 3.6 Build C++ Channel Engine

The pybind11 C++ implementation exists but hasn't been compiled (pybind11 not installed on the machine). Python fallback works but is ~50-100x slower. For N=64 sweeps, the C++ version is needed.

### 3.7 Paper Figures

From the paper structure doc:
1. Visibility-transfer timeline (RSU sees hazard, ego crop discards it)
2. Utility vs communication cost (early/intermediate/edge comparison)
3. Failure taxonomy heatmap (crop-limited, comm-limited, compute-limited, queue-limited)

---

## 4. File Inventory

### New Files in `ecav/core/networking/`
```
ecav/core/networking/__init__.py
ecav/core/networking/occlusion_model.py
ecav/core/networking/channel_engine_py.py
ecav/core/networking/channel_engine/channel_engine.h
ecav/core/networking/channel_engine/channel_engine.cpp
ecav/core/networking/channel_engine/bindings.cpp
ecav/core/networking/channel_engine/CMakeLists.txt
ecav/core/networking/channel_engine/__init__.py
```

### New Files in `ecav/core/application/v2v/`
```
ecav/core/application/v2v/v2v_manager.py              (intermediate fusion V2V)
ecav/core/application/v2v/v2v_early_fusion_manager.py  (AutoCast-style early fusion)
ecav/core/application/v2v/harbor_manager.py            (Harbor hybrid V2I+V2V)
ecav/core/application/v2v/v2v_fusion.py                (feature exchange module)
ecav/core/application/v2v/v2v_metrics.py               (PRR/CBR/AoI metrics)
ecav/core/application/v2v/mckp_scheduler.py            (MCKP bandwidth scheduler)
```

### New Scenario Files
```
ecav/scenario_testing/scenarios/scenario_4_autocast_intersection.xml
ecav/scenario_testing/scenarios/scenario_4_autocast_intersection.py
ecav/scenario_testing/config_yaml/openscenario_3_v2v_coop.yaml
ecav/scenario_testing/config_yaml/openscenario_3_v2v_autocast.yaml
ecav/scenario_testing/config_yaml/openscenario_3_harbor.yaml
ecav/scenario_testing/openscenario_3_v2v_coop.py
ecav/scenario_testing/openscenario_3_v2v_autocast.py
ecav/scenario_testing/openscenario_3_harbor.py
```

### Modified Files
```
ecav/scenario_testing/utils/sim_api.py          (V2V manager routing)
ecav/core/application/v2v/__init__.py            (package exports)
ecav/core/sensing/perception/bm2cp_perception_manager.py  (record_len fix)
```

### Data Copied
```
ecav/worldfusion/opencood/logs/v2xsim_bm2cp_ego_baseline_2026_01_02_20_01_42/
  (BM2CP checkpoint copied from ecloudsim)
```

---

## 5. Comparison Architecture Summary

| System | Fusion Level | Shared Data | Post-Processing | Detection Range | Channel Model | Edge Compute |
|--------|-------------|-------------|-----------------|-----------------|---------------|-------------|
| **BM2CP V2V** | Intermediate | BEV features | Ego-centric 32m crop | 32m (trained) | C-V2X WINNER+ B1 | None (per-vehicle) |
| **AutoCast V2V** | Early | Object point clouds | Per-agent, then fuse | 70m (GT query) | C-V2X WINNER+ B1 | None (per-vehicle) |
| **Harbor** | Early (at edge) | Raw point clouds via relay | Edge-side merge, 50m crop | 50m (PointPillars) | C-V2X relay + cellular V2I | Edge server via backhaul |
| **Our Edge** | Intermediate | BEV features via V2I | World-anchored or ego-centric | 32-80m (model dependent) | Cellular Uu uplink | Edge server via backhaul |

### Harbor Vehicle Roles (Realistic Representation)

In Harbor's architecture, vehicles fall into two groups:

**Helpers** (good V2I connectivity, bandwidth > 1 Mbps):
- Upload their own LiDAR point clouds to edge via cellular V2I
- Receive and relay helpees' point clouds to edge via their V2I link
- Run local detection in parallel as fallback
- Have full sensor stack (LiDAR, cameras)

**Helpees** (poor V2I connectivity, bandwidth < 1 Mbps):
- Cannot reliably upload to edge directly
- Send point clouds to assigned helper via V2V (PC5 sidelink)
- Run local detection as fallback
- Have full sensor stack but rely on V2V relay for cooperative benefit

**Non-participating vehicles** (not equipped):
- No sensors, no communication
- Pure obstacles / scene objects
- The collider (red-light violator) is non-participating

**RSU / Edge server**:
- RSU is a radio relay point, NOT the compute node
- Edge server is at the base station or MEC rack, reached through cellular backhaul
- Edge merges all received point clouds, runs PointPillars detection, returns results
- 500ms E2E deadline; if missed, vehicles fall back to local detection

In our Scenario 4, a realistic Harbor setup would have:
- Ego = helper (good V2I, uploads own data + receives edge results)
- Cooperating CAV = could be helper or helpee depending on V2I quality
- RSU at intersection = provides radio coverage, not compute
- Edge server = processes merged point clouds, reached via backhaul
- Collider + parked cars + trucks = non-participating

---

## 6. Paper Positioning

From the paper structure document, the V2V results support Section 3 ("The V2V Networking Fallacy") and the comparative evaluation in Section 7.

**Three-way comparison:**

1. **Feature-sharing V2V (BM2CP)** -- Exposes visibility-transfer failure. Communication cost paid, zero planner utility in the critical window.

2. **Object-sharing V2V (AutoCast)** -- Avoids the BEV crop problem via early fusion. Communication-efficient with MCKP scheduling. But provides a thinner world model (no learned features, no joint perception-prediction).

3. **Edge-hosted world model (our system)** -- Centralized fusion + tracking + prediction. Full cooperative visibility via world-anchored post-processing. Fails under compute load at high N (deadline cliff). Runtime controller shifts the cliff.

**The paper's claim is NOT "V2V is useless." The claim is:**

> Standard cooperative BEV stacks fail to convert remote cooperative visibility into planner-visible objects when post-processing is ego-centric and range-limited. Object-prioritized V2V (AutoCast) avoids this but provides a thinner cooperative state. Edge-hosted world models preserve richer utility but hit a compute-driven deadline cliff under dense participation. A runtime resource-aware controller is required to keep the world model planner-feasible under load.
