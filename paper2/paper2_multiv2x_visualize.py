#!/usr/bin/env python3
"""Visualize Multi-V2X vehicle/RSU trajectories on the actual CARLA map.

Connects to a running CARLA server, loads the correct town, and draws
all CAV and RSU trajectories as debug lines. Also spawns spectator at
a good viewpoint.

Usage:
    # Start CARLA first (headless or windowed)
    # Then:
    python paper2_multiv2x_visualize.py \
        --data-root /data1/Datasets/Multi-V2X \
        --town Town10HD__2023_11_15_00_20_38 \
        --host localhost --port 2000
"""

import os
import sys
import argparse
import time
from pathlib import Path
from collections import defaultdict

import numpy as np
import yaml

try:
    import carla
except ImportError:
    sys.exit("CARLA Python API not found. Source the CARLA egg or install carla package.")


def load_town_trajectories(data_root: str, town: str, max_frames: int = 0):
    """Load all agent trajectories from a Multi-V2X town.

    Returns:
      map_name: str (e.g. 'Town10HD')
      trajectories: dict[agent_name] = list of (timestamp, [x,y,z,roll,yaw,pitch])
      rsu_positions: dict[agent_name] = [x,y,z,roll,yaw,pitch]
    """
    town_dir = Path(data_root) / town

    # Get map name from data_protocal.yaml
    dp_path = town_dir / 'data_protocal.yaml'
    if dp_path.exists():
        with open(dp_path) as f:
            dp = yaml.safe_load(f)
        map_name = dp['world']['map_name']
    else:
        map_name = town.split('__')[0]

    # Find all agents
    agent_dirs = sorted([
        d.name for d in town_dir.iterdir()
        if d.is_dir() and (d.name.startswith('cav_') or d.name.startswith('rsu_'))
    ])

    trajectories = {}
    rsu_positions = {}

    import re
    pose_re = re.compile(r'^lidar_pose:\s*\[(.+)\]', re.MULTILINE)

    for idx, agent_name in enumerate(agent_dirs):
        agent_dir = town_dir / agent_name
        yamls = sorted([
            f.stem for f in agent_dir.glob('*.yaml')
            if f.stem.isdigit()
        ])
        if max_frames and len(yamls) > max_frames:
            yamls = yamls[:max_frames]

        # Fast: read lidar_pose lines without full yaml parse
        traj = []
        for ts in yamls:
            fpath = agent_dir / f'{ts}.yaml'
            with open(fpath) as f:
                lines = f.readlines()
            for li, line in enumerate(lines):
                if line.startswith('lidar_pose:'):
                    pose = []
                    for j in range(1, 7):
                        val_line = lines[li + j].strip()
                        pose.append(float(val_line.lstrip('- ')))
                    traj.append((ts, pose))
                    break

        if traj:
            trajectories[agent_name] = traj
            if agent_name.startswith('rsu_'):
                rsu_positions[agent_name] = traj[0][1]

        if (idx + 1) % 10 == 0 or idx == len(agent_dirs) - 1:
            print(f"  [{idx+1}/{len(agent_dirs)}] loaded {agent_name}")

    print(f"Loaded {town}: map={map_name}, "
          f"{sum(1 for a in agent_dirs if a.startswith('cav_'))} CAVs, "
          f"{sum(1 for a in agent_dirs if a.startswith('rsu_'))} RSUs, "
          f"{len(trajectories)} agents with trajectories")

    return map_name, trajectories, rsu_positions


def visualize_on_carla(client, map_name, trajectories, rsu_positions,
                       line_life: float = 120.0):
    """Draw trajectories on the CARLA map using debug drawing."""
    world = client.get_world()

    # Load correct map if needed
    current_map = world.get_map().name.split('/')[-1]
    if current_map != map_name:
        print(f"Loading map {map_name} (current: {current_map})...")
        world = client.load_world(map_name)
        time.sleep(2.0)

    debug = world.debug

    # Color palette
    cav_color = carla.Color(0, 150, 255)      # blue
    rsu_color = carla.Color(255, 50, 50)       # red
    conn_color = carla.Color(100, 255, 100)    # green (for connections)

    # Draw CAV trajectories (subsample for performance)
    n_drawn = 0
    step = max(1, len(next(iter(trajectories.values()))) // 60)
    for agent_name, traj in trajectories.items():
        if agent_name.startswith('rsu_'):
            color = rsu_color
            thickness = 0.15
        else:
            color = cav_color
            thickness = 0.08

        sampled = traj[::step]
        for i in range(len(sampled) - 1):
            _, pose0 = sampled[i]
            _, pose1 = sampled[i + 1]
            p0 = carla.Location(x=pose0[0], y=pose0[1], z=pose0[2] + 0.5)
            p1 = carla.Location(x=pose1[0], y=pose1[1], z=pose1[2] + 0.5)
            debug.draw_line(p0, p1, thickness=thickness,
                           color=color, life_time=line_life)

        # Draw start point as a larger marker
        _, start_pose = traj[0]
        start_loc = carla.Location(
            x=start_pose[0], y=start_pose[1], z=start_pose[2] + 1.0)
        debug.draw_point(start_loc, size=0.15,
                        color=carla.Color(255, 255, 0),
                        life_time=line_life)
        n_drawn += 1

    # Draw RSU positions as large red boxes
    for rsu_name, pose in rsu_positions.items():
        center = carla.Location(x=pose[0], y=pose[1], z=pose[2])
        box = carla.BoundingBox(center, carla.Vector3D(x=2.0, y=2.0, z=3.0))
        rot = carla.Rotation()
        debug.draw_box(box, rot, thickness=0.3, color=rsu_color,
                       life_time=line_life)
        debug.draw_string(
            carla.Location(x=pose[0], y=pose[1], z=pose[2] + 6.0),
            rsu_name, draw_shadow=True, color=rsu_color, life_time=line_life)

    # Move spectator to overview position
    # Compute centroid of all first-frame positions
    all_positions = []
    for traj in trajectories.values():
        _, pose = traj[0]
        all_positions.append(pose[:3])
    centroid = np.mean(all_positions, axis=0)

    spectator = world.get_spectator()
    spectator.set_transform(carla.Transform(
        carla.Location(x=centroid[0], y=centroid[1], z=centroid[2] + 80),
        carla.Rotation(pitch=-70, yaw=0)))

    print(f"Drew {n_drawn} trajectories, {len(rsu_positions)} RSUs")
    print(f"Spectator at centroid ({centroid[0]:.0f}, {centroid[1]:.0f}), altitude +80m")
    print(f"Lines persist for {line_life}s. Move spectator in CARLA to explore.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', default='/data1/Datasets/Multi-V2X')
    parser.add_argument('--town', default='Town10HD__2023_11_15_00_20_38')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Limit frames per agent (0=all)')
    parser.add_argument('--life-time', type=float, default=120.0,
                        help='How long debug lines persist (seconds)')
    args = parser.parse_args()

    map_name, trajectories, rsu_positions = load_town_trajectories(
        args.data_root, args.town, max_frames=args.max_frames)

    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)

    visualize_on_carla(client, map_name, trajectories, rsu_positions,
                       line_life=args.life_time)

    print("\nDone. Trajectories are visible in the CARLA window.")
    print("Blue = CAV paths, Red = RSU positions, Yellow = start points")


if __name__ == '__main__':
    main()
