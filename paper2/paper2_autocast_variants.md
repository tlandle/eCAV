# AutoCast Baseline Variants for Paper 2

## What AutoCastSim Actually Does (Code Evidence)

### Detection Pipeline (AVR/Collaborator.py lines 428-454)
1. Each cooperating vehicle has a real CARLA LiDAR sensor attached
   (`scenario_manager.py` line 251: `setup_sensors(vehicle)`)
2. LiDAR point cloud processed via `LidarPreprocessor.process_lidar()`
   - Voxelization to occupancy grid (0.5m cells, 140x140x10 grid)
   - Connected component / island detection for object clustering
   - Bounding box extraction per cluster
3. Output: `detected_object_list` with real point cloud segments and estimated bboxes

### GT Enrichment (AVR/PCProcess.py lines 1092-1142)
After LiDAR detection, `estimated_actor_and_speed_from_detected_object()`:
1. For each LiDAR-detected cluster, find nearest CARLA actor within 3m
2. If matched: inject GT velocity, GT acceleration, GT world position, GT actor ID
3. **If NOT matched: detection is DROPPED** (line 1142, only matched objects appended)

This means:
- False positive detections are silently eliminated (GT-assisted filtering)
- Surviving detections have perfect velocity/acceleration (GT oracle)
- World position is from GT, not LiDAR estimate
- Point cloud segments are from real LiDAR (not synthesized)

### Communication (AVR/DetectedObject.py lines 23-33)
`get_obj_for_comm()` packages into the V2V message:
- `point_cloud_list` (real LiDAR points)
- `bounding_box` (LiDAR-derived)
- `estimated_speed` (GT)
- `estimated_accel` (GT)
- `esitmated_position` (GT)
- `actor_id` (GT)

### Participating Vehicles (scenario_manager.py lines 221-255)
Every non-ego, non-passive vehicle within 50m gets:
- Real CARLA LiDAR sensor
- A Collaborator (communication + detection pipeline)
- Including stationary "parked" vehicles spawned by the scenario

Only vehicles with `role_name == PASSIVE_ACTOR_ROLENAME` are excluded.

---

## Four Variants for Our Evaluation

### Variant 1: AutoCast-Faithful (replicates their evaluation)
- **Cooperating vehicles**: All scenario vehicles (including parked cars at intersection)
- **Detection**: Real LiDAR + GT enrichment + GT false-positive filtering
- **Shared data**: Point cloud segments with GT kinematics
- **Purpose**: Reproduce AutoCast's reported results. Show it works under their assumptions.

### Variant 2: AutoCast Sensor-Only (no GT enrichment)
- **Cooperating vehicles**: Same as Variant 1 (including parked cars)
- **Detection**: Real LiDAR detection only. No GT matching, no FP filtering, no oracle kinematics.
- **Shared data**: Point cloud segments with LiDAR-estimated bboxes and no velocity info
- **Purpose**: Show the impact of removing GT-assisted filtering and oracle kinematics.

### Variant 3: Moving-CAV-Only (no parked helpers)
- **Cooperating vehicles**: Only moving vehicles and RSU. Parked cars are non-participating obstacles.
- **Detection**: Real LiDAR + GT enrichment (same as Variant 1)
- **Shared data**: Same as Variant 1
- **Purpose**: Show whether AutoCast works without the convenient parked-car infrastructure.

### Variant 4: Realistic AutoCast (no parked helpers, no GT enrichment)
- **Cooperating vehicles**: Only moving vehicles and RSU
- **Detection**: Real LiDAR only, no GT
- **Shared data**: Point cloud segments with estimated (not GT) kinematics
- **Purpose**: Most realistic V2V early fusion baseline. Expected to be the weakest.

---

## Expected Results

| Variant | Parked helpers | GT enrichment | Expected outcome |
|---------|---------------|---------------|-----------------|
| 1 (Faithful) | Yes | Yes | Works (matches AutoCast paper) |
| 2 (Sensor-only) | Yes | No | Degraded (more FPs, no velocity) |
| 3 (Moving-only) | No | Yes | Fails if no moving CAV has LOS to collider |
| 4 (Realistic) | No | No | Fails (no LOS + no GT cleanup) |

The key finding: **AutoCast's success depends on (a) stationary helper vehicles at the intersection with LOS to the threat, and (b) GT-assisted false positive filtering. Remove either, and the system degrades significantly.**

For comparison with our edge system, Variant 4 is the fair baseline. The edge system uses an RSU (infrastructure) rather than parked cars, and uses learned detection (PointPillars/BM2CP) rather than GT-assisted occupancy grids.

---

## Implementation Changes Needed

### v2v_early_fusion_manager.py
1. Add a config flag `gt_enrichment: true/false` controlling whether detected objects get GT velocity/position injection
2. Add a config flag `parked_cars_participate: true/false` controlling which vehicles get treated as cooperating nodes
3. Replace `_extract_gt_objects()` with real LiDAR-based detection from each agent's perception manager
4. Add the GT enrichment step (match detected clusters to CARLA actors within 3m, inject GT state) as an optional post-processing step
