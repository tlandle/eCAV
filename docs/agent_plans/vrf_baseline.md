# Plan: VRF baseline (Vehicle Road-side Point Cloud Fusion)

## What VRF actually is

VRF (Khan, Khalid, Turkar, Dantu, Ahmad. "VRF: Vehicle Road-side Point Cloud
Fusion." MobiSys '24). Source: https://github.com/nsslofficial/VRF, paper PDF
https://fawadahm.github.io/assets/pdf/VRF_Mobisys_2024.pdf.

Early fusion of raw LiDAR, fused **on the vehicle**:

- RSU is a thin LiDAR sensor. It does not detect, track, or plan.
- **Handshake, once per vehicle-RSU association:** RSU sends a compressed
  reference point cloud (static scene, captured with no dynamic objects).
  Reported avg 500 KB, 12 ms.
- **Per frame:** RSU sends only a **diff cloud** = current cloud minus
  reference, via double-buffer Octree XOR, then compressed. ~56x size
  reduction. Per-frame bandwidth measured in **Kbps**, not Mbps. The vehicle
  reconstructs the current RSU cloud by adding the diff to the reference.
- **Indirect alignment:** both clouds are aligned to a pre-built 3D map. The
  vehicle localizes in the map with NDT. The RSU-to-map transform is solved
  **offline at install** (~7 s) and cached because the RSU is stationary.
- **Direct alignment:** GICP refines the RSU-to-vehicle alignment, gated by an
  alignment-accuracy forecaster. ICP runs in parallel with diff-cloud transit
  using the reference cloud (RSU is static), so the transform is ready when the
  diff arrives.
- Fused cloud feeds the vehicle's downstream perception.

**Downstream detector (reaction-time experiment, the safety-relevant one):**
background subtraction against the 3D map to extract dynamic points, then
Euclidean clustering into objects. No neural network. PointPillars is **not**
used. The learned detector cited ([60] Yuan et al.) appears only in a general
point-density discussion, not the experiment.

Reported end-to-end fusion latency ~12.5 ms avg, <15.5 ms at 80 km/hr;
alignment error ~1.6 cm / 0.02 deg.

## Why bandwidth is not the discriminator (reviewer correction)

The reviewer is right: VRF is not bandwidth-heavy. Diff clouds put it in the
Kbps range, the same order as our object lists (~10 KB/s). Any paper text that
dismisses VRF on bandwidth is wrong and must be dropped. The real comparison
axes against VRF:

1. HD-map dependency (NDT localization + alignment anchor). We need none.
2. Offline per-RSU calibration (~7 s at install, binds RSU to a surveyed pose).
3. Per-association handshake cold-start (500 KB reference cloud on every locale
   entry before fusion works). Directly relevant to the Scale-Out handoff story.
4. Per-vehicle compute multiplication: N vehicles each run fusion + detection +
   tracking + prediction + planning. Our edge does it once for N.
5. Different failure mode: alignment staleness/accuracy, not publish-boundary
   self-ghosting. VRF has no object-list matching, so it cannot self-ghost; it
   pays for that immunity with on-vehicle compute and the map/calibration/
   handshake dependencies.

## Latency: measured, not borrowed

For an AoI-at-use paper, charging VRF a reported constant while measuring our
own pipeline is non-comparable. Everything we run is measured on the same
hardware; payloads go through the same ns-3 cosim; results feed the same
envelope. Accounting:

| Stage | Accounting |
|---|---|
| RSU diff-cloud build (Octree XOR + compress) | Measured (open3d Octree on real RSU cloud) |
| Diff-cloud transmission | ns-3, same path as all baselines; payload = real compressed diff bytes |
| Handshake reference cloud (once/association) | Measured size + ns-3 transfer, charged on locale entry |
| Vehicle reconstruct (decompress + add diff) | Measured |
| Direct alignment (GICP) | Measured (pygicp = VRF's fast_gicp core, not open3d) |
| Fusion (merge into ego frame) | Measured |
| Background subtraction + DBSCAN clustering | Measured (open3d) |
| Tracking + prediction | Measured (existing VehicleSideTracker + LinearPredictor) |
| NDT localization | Substituted by GT ego pose; not counted. Undercounts VRF latency, conservative (favors baseline). Flag in paper. |

Registration backend must be faithful to VRF, so use `pygicp` (Koide fast_gicp,
which their pybind wraps), not open3d's GICP, for the measured registration
cost. open3d is used only for the Octree-XOR diff and DBSCAN clustering.

## Architecture placement in our sim

VRF is **vehicle-side**, so it is NOT an edge-manager subclass (unlike
InfraOnly/CIP). The RSU is a thin sensor shipping diff clouds; the edge does no
object-level work. Implement as a vehicle-side fusion+detection component whose
output feeds the already-wired `VehicleSideTracker` -> `LinearPredictorManager`
-> local planner fallback (`vehicle_manager.py:769-792`, `behavior_agent.py`
local_predictions path).

The tracking/prediction/planning stack is held **identical** to our system so
the comparison isolates the fusion architecture, not the predictor.

## Sim facts confirmed

- Raw LiDAR `(N,4)` available per tick: ego `perception_manager.lidar.data`
  (sensor frame, `LidarSensor.data`), RSU via `RSUManager.perception_manager`.
- CARLA gives exact ego/RSU extrinsics (substitutes for NDT + offline calib).
- open3d 0.18 installed (Octree, GICP, `cluster_dbscan`). `pygicp` NOT installed.
- Detection in this sim is camera-first (YOLO on RGB, LiDAR only adds depth via
  `o3d_camera_lidar_fusion_from_tracker`). There is NO live LiDAR 3D detector.
  This is why VRF's own background-subtraction + clustering detector is the
  right and faithful choice, and makes VRF detection real and measured rather
  than GT-injected.

## Build steps

### 0. Dependency
- `pip install pygicp` in the `opencda` env. If the build fights us, fall back
  to open3d `registration_generalized_icp` and note the backend in the paper.

### 1. New module: `ecav/core/sensing/fusion/vrf_fusion_manager.py`
Vehicle-side VRF pipeline. Responsibilities:
- `register_rsu(rsu_id, reference_cloud, rsu_to_world)` — handshake. Store
  compressed reference cloud, measure its size, mark handshake cost pending for
  the ns-3/AoI path on locale entry.
- `step(ego_cloud, ego_pose, rsu_current_cloud, tick)`:
  1. RSU side: diff = OctreeXOR(rsu_current, reference); compress; record bytes.
  2. ns-3 accounting hook for diff payload (and handshake on first association).
  3. Vehicle side: reconstruct rsu_current = reference + diff.
  4. GICP align reference -> ego_cloud with GT-pose init guess; apply transform.
  5. Fuse: ego_cloud ∪ transformed rsu_cloud in ego frame.
  6. Background subtraction: fused minus reference-map -> dynamic points.
  7. DBSCAN cluster dynamic points -> per-cluster boxes (centroid + extent + yaw
     from principal axis).
  8. Return boxes in the AB3DMOT bundle format via
     `ecav.core.tracking.ab3dmot_format.stack_rows` (reuse the shared lib).
- Each compute stage wrapped in a timer; expose per-stage latencies for the
  AoI decomposition.

### 2. Wire into vehicle perception path
- Config flag `vrf_fusion` block on the CAV (off by default).
- When enabled, `vehicle_manager.update_info` feeds VRF boxes into
  `self.local_tracker.process_detections(...)` instead of the camera-derived
  `objects['vehicles']`. The downstream tracker/predictor/planner path is
  unchanged (already wired).
- RSU LiDAR access: resolve how the vehicle reaches the RSU's raw cloud in the
  centralized sim (likely via sim_api / edge passing RSU handles, or cav_world).
  OPEN ITEM, resolve during implementation.

### 3. ns-3 / AoI accounting
- Route diff-cloud payload bytes and handshake bytes through the same cosim
  path the other baselines use. Identify the exact hook (where payload bytes are
  recorded; grep frame.time / AoI / ns3). OPEN ITEM.

### 4. Reference-cloud capture
- Capture the static reference at scenario start (no dynamic actors) per RSU, or
  pre-capture and load. The same cloud is the GICP reference and the
  background-subtraction map.

### 5. YAML config
- `ecav/scenario_testing/config_yaml/openscenario_3_edge_vrf.yaml` (and the
  scenario 20-23 variants if needed): enable `vrf_fusion` on CAVs, RSU as thin
  LiDAR sensor, edge in passthrough.

### 6. Docstring/paper correctness fixes (known errors)
- `edge_manager_infra_only_ab3dmot_linear_predictor.py` docstring: remove the
  claim that InfraOnly "represents the VRF ... architecture class." VRF is
  vehicle-side early fusion, not RSU-only edge tracking. InfraOnly represents
  the 3GPP-MEC / VI-Eye-RSU-tracking class, not VRF.
- `safety_envelope_sensys/contents/system_architecture.tex:8`: VRF does not
  "co-locate compute at the RSU." Fusion is on the vehicle.
- Drop any bandwidth-based dismissal of VRF (keep it for EMP only).
- Fix the VRF bib entry (wrong title "Vehicle Recall Framework", placeholder
  authors). Correct: K. Khan, A. Khalid, Y. Turkar, K. Dantu, F. Ahmad,
  "VRF: Vehicle Road-side Point Cloud Fusion", MobiSys '24,
  doi 10.1145/3643832.3661874.

## Verification

1. Reference capture: confirm static reference cloud has no dynamic actors.
2. Diff cloud: confirm OctreeXOR diff size is ~1-2 orders smaller than full
   cloud; log compressed bytes.
3. GICP: confirm aligned RSU cloud overlays ego cloud (cm-level) on a static
   landmark.
4. Occlusion recovery: in scenario_3 LTAP, confirm the occluded cross-traffic
   Tesla produces dynamic points in the fused cloud and a cluster/box before the
   ego camera could see it. This is the VRF reaction-time benefit.
5. Tracker: confirm the Tesla gets a stable track id across ticks and the
   predictor emits a forward trajectory.
6. AoI: confirm per-stage measured latencies are non-trivial and feed the
   envelope; confirm diff payload goes through ns-3.
7. Closed-loop: scenario_3 with VRF vs our system vs InfraOnly/CIP; compare
   reaction time / brake-collision events under matched AoI.

## Open items to resolve during implementation
- RSU raw-cloud access path from the vehicle side in the centralized sim.
- Exact ns-3 payload-accounting hook.
- pygicp build vs open3d GICP fallback.
- Cluster-to-box yaw estimation (principal axis vs bounding-box fit).
- Whether reference cloud is captured live at t0 or pre-generated per scenario.

## Out of scope
- VRF's alignment-accuracy forecaster (decides when to run direct ICP). We
  always run GICP; note as a minor fidelity gap that, if anything, costs VRF
  latency (conservative).
- Full ROS-bridge of their C++ nodes (deadline risk, not worth it).
- VRF's offline RSU-to-map calibration (replaced by CARLA GT extrinsics).
