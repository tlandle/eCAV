"""Smoke test for SMARTPredictorManager3D.

Loads the SMART model, creates synthetic tracked trajectories that mimic
AB3DMOT output, runs generate_predicted_trajectories(), and verifies
output shapes and basic sanity.

Usage:
    conda run -n ecav310 python tests/test_smart_predictor.py
"""
import sys
import os
import time
from collections import deque

# Project root on path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np
from ecav.ecav_carla import Location, Rotation, Transform
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory


def make_synthetic_track(track_id, carla_id, start_x, start_y, vx, vy,
                         yaw_deg, num_ticks=30):
    """Create a synthetic ObstacleTrajectory mimicking AB3DMOT output.

    Generates num_ticks of 20Hz positions with constant velocity.
    trajectory[0] = newest, trajectory[-1] = oldest (edge manager convention).
    """
    # Minimal corners for BoundingBox (2m x 4.8m vehicle)
    corners = np.array([
        [start_x - 2.4, start_y - 1.0, 0.0],
        [start_x + 2.4, start_y - 1.0, 0.0],
        [start_x + 2.4, start_y + 1.0, 0.0],
        [start_x - 2.4, start_y + 1.0, 0.0],
        [start_x - 2.4, start_y - 1.0, 1.5],
        [start_x + 2.4, start_y - 1.0, 1.5],
        [start_x + 2.4, start_y + 1.0, 1.5],
        [start_x - 2.4, start_y + 1.0, 1.5],
    ])
    obs = ObstacleVehicle(corners, None, track_id=track_id)
    obs.carla_id = carla_id
    obs.kf_vx = vx * 0.05   # m/tick at 20Hz
    obs.kf_vy = vy * 0.05
    obs.kf_speed_mps = (vx**2 + vy**2)**0.5

    # Build trajectory: newest first
    traj = deque()
    for i in range(num_ticks):
        # i=0 is newest, i=num_ticks-1 is oldest
        age = i  # ticks ago
        x = start_x - vx * 0.05 * age
        y = start_y - vy * 0.05 * age
        tf = Transform(
            location=Location(x=x, y=y, z=0.3),
            rotation=Rotation(roll=0.0, pitch=0.0, yaw=yaw_deg))
        traj.append(tf)

    obs.location = Location(x=start_x, y=start_y, z=0.3)
    ot = ObstacleTrajectory(obs, list(traj))
    return track_id, ot


def main():
    checkpoint = os.path.join(
        ROOT, 'ecav', 'core', 'prediction', 'smart',
        'checkpoints', 'epoch=105.ckpt')

    if not os.path.isfile(checkpoint):
        print(f"CHECKPOINT NOT FOUND: {checkpoint}")
        print("Place the checkpoint at ecav/core/prediction/smart/checkpoints/epoch=105.ckpt")
        sys.exit(1)

    print(f"Checkpoint: {checkpoint}")
    print()

    # ── 1. Load model ────────────────────────────────────────────────────
    print("Loading SMARTPredictorManager3D...")
    t0 = time.perf_counter()

    from ecav.core.prediction.smart_predictor_manager import (
        SMARTPredictorManager3D)

    pred = SMARTPredictorManager3D(
        checkpoint_path=checkpoint,
        map_cache_path=None,
        device='cuda',
        num_output_steps=25)

    t_load = time.perf_counter() - t0
    print(f"Model loaded in {t_load:.1f}s")
    print()

    # ── 2. Create synthetic tracks ───────────────────────────────────────
    # 3 vehicles at an intersection:
    #   Vehicle A: heading east at 10 m/s
    #   Vehicle B: heading north at 8 m/s
    #   Vehicle C: stationary (parked)
    tracks = {}

    tid, ot = make_synthetic_track(
        track_id=1, carla_id=100,
        start_x=-50.0, start_y=128.0, vx=10.0, vy=0.0, yaw_deg=0.0,
        num_ticks=30)
    tracks[tid] = ot

    tid, ot = make_synthetic_track(
        track_id=2, carla_id=101,
        start_x=-78.0, start_y=110.0, vx=0.0, vy=8.0, yaw_deg=90.0,
        num_ticks=30)
    tracks[tid] = ot

    tid, ot = make_synthetic_track(
        track_id=3, carla_id=102,
        start_x=-90.0, start_y=130.0, vx=0.0, vy=0.0, yaw_deg=45.0,
        num_ticks=30)
    tracks[tid] = ot

    print(f"Created {len(tracks)} synthetic tracks (30 ticks each at 20Hz)")
    print()

    # ── 3. Run prediction ────────────────────────────────────────────────
    print("Running generate_predicted_trajectories()...")
    t0 = time.perf_counter()

    predictions = pred.generate_predicted_trajectories(
        tracks, source_tick=100, publish_tick=100)

    t_pred = time.perf_counter() - t0
    print(f"Prediction done in {t_pred*1000:.1f}ms")
    print(f"Got {len(predictions)} predictions")
    print()

    # ── 4. Verify output ─────────────────────────────────────────────────
    ok = True

    if len(predictions) == 0:
        print("FAIL: no predictions returned")
        ok = False
    else:
        for p in predictions:
            tid = p.obstacle_trajectory.obstacle.track_id
            n = len(p.predicted_trajectory)
            if n != 25:
                print(f"FAIL: track {tid} has {n} predicted steps, expected 25")
                ok = False
                continue

            # Check predicted positions are finite
            for j, tf in enumerate(p.predicted_trajectory):
                if not (np.isfinite(tf.location.x) and np.isfinite(tf.location.y)):
                    print(f"FAIL: track {tid} step {j} has non-finite position")
                    ok = False
                    break

            # Print summary
            first = p.predicted_trajectory[0]
            last = p.predicted_trajectory[-1]
            dx = last.location.x - first.location.x
            dy = last.location.y - first.location.y
            dist = (dx**2 + dy**2)**0.5
            print(f"  Track {tid}: start=({first.location.x:.1f}, {first.location.y:.1f}) "
                  f"end=({last.location.x:.1f}, {last.location.y:.1f}) "
                  f"displacement={dist:.2f}m over 1.25s")

    print()
    if ok:
        print("PASS: all checks passed")
    else:
        print("FAIL: some checks failed")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
