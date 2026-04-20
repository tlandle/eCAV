#!/usr/bin/env python3
"""Document all Multi-V2X town scenarios with CARLA visualization and statistics.

For each Multi-V2X town:
1. Connects to local CARLA server
2. Loads the correct map
3. Draws CAV trajectories (blue) and RSU positions (red boxes)
4. Captures bird's-eye screenshot via CARLA RGB camera sensor
5. Computes scenario statistics (CAVs, RSUs, frames, objects, connections, speeds)
6. Writes a markdown description file

Usage:
    python paper2_multiv2x_document_scenarios.py --data-root /data1/Datasets/Multi-V2X
"""

import os
import sys
import argparse
import time
import re
from pathlib import Path
from collections import defaultdict, Counter

import numpy as np
import yaml

try:
    import carla
except ImportError:
    sys.exit("CARLA Python API not found. Source the CARLA egg or install carla package.")

# Towns to process
TOWNS = [
    'Town01__2023_11_10_22_23_28',
    'Town03__2023_11_13_18_26_24',
    'Town05__2023_11_13_23_03_07',
    'Town06__2023_11_14_16_39_31',
    'Town07__2023_11_14_22_06_38',
    'Town10HD__2023_11_15_00_20_38',
]

# Qualitative descriptions per town (CARLA map characteristics)
TOWN_DESCRIPTIONS = {
    'Town01': (
        'Town01 is a small town with T-junctions and a mix of two-lane roads. '
        'The layout includes residential-style streets with moderate curvature '
        'and a few signalized intersections. Vegetation lines both sides of the road.'
    ),
    'Town03': (
        'Town03 is a larger urban environment with a central roundabout, '
        'multi-lane roads, and several signalized intersections. The road network '
        'includes highway on-ramps and varied elevation changes.'
    ),
    'Town05': (
        'Town05 features a grid-like urban layout with multi-lane roads, '
        'numerous signalized four-way intersections, and highway sections with '
        'overpasses. The environment includes both urban core and suburban fringe areas.'
    ),
    'Town06': (
        'Town06 is characterized by long highway segments with entrance and exit ramps, '
        'connected to a small urban area with four-way intersections. The highway portion '
        'features multi-lane divided roads.'
    ),
    'Town07': (
        'Town07 is a rural setting with narrow two-lane roads, sharp curves, and '
        'minimal signalized intersections. The road network winds through countryside '
        'terrain with limited infrastructure.'
    ),
    'Town10HD': (
        'Town10HD is a dense downtown environment with narrow one-way streets, '
        'numerous four-way intersections in a grid pattern, and pedestrian-heavy zones. '
        'Buildings line both sides of the streets, creating urban canyon effects.'
    ),
}


def load_town_trajectories(town_dir: Path):
    """Load all agent trajectories using fast line-scanning for lidar_pose.

    Returns:
        map_name: str
        trajectories: dict[agent_name] -> list of (timestamp_str, [x,y,z,roll,yaw,pitch])
        rsu_positions: dict[agent_name] -> [x,y,z,roll,yaw,pitch]
        cav_names: list of str
        rsu_names: list of str
    """
    # Get map name from data_protocal.yaml
    dp_path = town_dir / 'data_protocal.yaml'
    if dp_path.exists():
        with open(dp_path) as f:
            dp = yaml.safe_load(f)
        map_name = dp['world']['map_name']
    else:
        map_name = town_dir.name.split('__')[0]

    # Find all agents
    agent_dirs = sorted([
        d.name for d in town_dir.iterdir()
        if d.is_dir() and (d.name.startswith('cav_') or d.name.startswith('rsu_'))
    ])

    cav_names = [a for a in agent_dirs if a.startswith('cav_')]
    rsu_names = [a for a in agent_dirs if a.startswith('rsu_')]

    trajectories = {}
    rsu_positions = {}

    for idx, agent_name in enumerate(agent_dirs):
        agent_dir = town_dir / agent_name
        yamls = sorted([
            f.stem for f in agent_dir.glob('*.yaml')
            if f.stem.isdigit()
        ])

        # Fast line-scan for lidar_pose (multi-line yaml format)
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
            print(f"  [{idx+1}/{len(agent_dirs)}] loaded {agent_name} ({len(traj)} frames)")

    print(f"  Loaded: map={map_name}, {len(cav_names)} CAVs, {len(rsu_names)} RSUs")
    return map_name, trajectories, rsu_positions, cav_names, rsu_names


def sample_frame_statistics(town_dir: Path, agent_names: list,
                            sample_every: int = 10):
    """Parse a sample of full YAML frames to extract object/connection/speed stats.

    Parses every `sample_every`-th frame to avoid full-parse overhead on all frames.

    Returns:
        objects_per_frame: list of int (number of objects per sampled frame)
        conn_counts: list of int (number of connected agents per sampled frame)
        speeds: list of float (ego_speed values across sampled frames)
        category_counts: Counter of object categories
    """
    objects_per_frame = []
    conn_counts = []
    speeds = []
    category_counts = Counter()

    for agent_name in agent_names:
        agent_dir = town_dir / agent_name
        yamls = sorted([
            f.stem for f in agent_dir.glob('*.yaml')
            if f.stem.isdigit()
        ])

        sampled_ts = yamls[::sample_every]
        for ts in sampled_ts:
            fpath = agent_dir / f'{ts}.yaml'
            try:
                with open(fpath) as f:
                    data = yaml.safe_load(f)
            except Exception:
                continue

            # ego_speed
            if 'ego_speed' in data and data['ego_speed'] is not None:
                sp = data['ego_speed']
                if isinstance(sp, list) and len(sp) > 0:
                    speeds.append(float(sp[0]))
                elif isinstance(sp, (int, float)):
                    speeds.append(float(sp))

            # conn_agents (connected agents within comm range)
            if 'conn_agents' in data and data['conn_agents'] is not None:
                ca = data['conn_agents']
                if isinstance(ca, list):
                    conn_counts.append(len(ca))
                else:
                    conn_counts.append(0)

            # objects
            if 'objects' in data and isinstance(data['objects'], dict):
                obj_dict = data['objects']
                objects_per_frame.append(len(obj_dict))
                for obj_id, obj_data in obj_dict.items():
                    if isinstance(obj_data, dict) and 'category' in obj_data:
                        category_counts[obj_data['category']] += 1

    return objects_per_frame, conn_counts, speeds, category_counts


def draw_on_carla(world, trajectories, rsu_positions, line_life=300.0):
    """Draw CAV trajectories (blue) and RSU boxes (red) on the CARLA map."""
    debug = world.debug

    cav_color = carla.Color(0, 150, 255)      # blue
    rsu_color = carla.Color(255, 50, 50)       # red
    start_color = carla.Color(255, 255, 0)     # yellow

    # Subsample trajectories for drawing performance
    for agent_name, traj in trajectories.items():
        if agent_name.startswith('rsu_'):
            color = rsu_color
            thickness = 0.15
        else:
            color = cav_color
            thickness = 0.08

        step = max(1, len(traj) // 80)
        sampled = traj[::step]
        for i in range(len(sampled) - 1):
            _, pose0 = sampled[i]
            _, pose1 = sampled[i + 1]
            p0 = carla.Location(x=pose0[0], y=pose0[1], z=pose0[2] + 0.5)
            p1 = carla.Location(x=pose1[0], y=pose1[1], z=pose1[2] + 0.5)
            debug.draw_line(p0, p1, thickness=thickness,
                            color=color, life_time=line_life)

        # Start marker
        _, start_pose = traj[0]
        start_loc = carla.Location(
            x=start_pose[0], y=start_pose[1], z=start_pose[2] + 1.0)
        debug.draw_point(start_loc, size=0.15, color=start_color,
                         life_time=line_life)

    # RSU boxes
    for rsu_name, pose in rsu_positions.items():
        center = carla.Location(x=pose[0], y=pose[1], z=pose[2])
        box = carla.BoundingBox(center, carla.Vector3D(x=2.0, y=2.0, z=3.0))
        rot = carla.Rotation()
        debug.draw_box(box, rot, thickness=0.3, color=rsu_color,
                       life_time=line_life)
        debug.draw_string(
            carla.Location(x=pose[0], y=pose[1], z=pose[2] + 6.0),
            rsu_name, draw_shadow=True, color=rsu_color, life_time=line_life)


def compute_spectator_transform(trajectories, rsu_positions):
    """Compute a bird's-eye spectator transform centered on all agents."""
    all_positions = []
    for traj in trajectories.values():
        for _, pose in traj:
            all_positions.append(pose[:3])
    for pose in rsu_positions.values():
        all_positions.append(pose[:3])

    all_positions = np.array(all_positions)
    centroid = np.mean(all_positions, axis=0)

    # Compute bounding box extent to set altitude
    mins = np.min(all_positions, axis=0)
    maxs = np.max(all_positions, axis=0)
    extent_x = maxs[0] - mins[0]
    extent_y = maxs[1] - mins[1]
    max_extent = max(extent_x, extent_y)

    # Altitude proportional to scene extent (FOV ~90 degrees means
    # altitude ~ half the extent for full coverage, add margin)
    altitude = max(max_extent * 0.7, 80.0)

    transform = carla.Transform(
        carla.Location(x=float(centroid[0]), y=float(centroid[1]),
                       z=float(centroid[2]) + altitude),
        carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0)
    )
    return transform, centroid, mins, maxs


def capture_screenshot(world, spectator_transform, output_path: str,
                       width=1920, height=1080):
    """Attach an RGB camera to the spectator, capture one frame, save as PNG."""
    import queue

    # Set spectator
    spectator = world.get_spectator()
    spectator.set_transform(spectator_transform)

    # Tick a couple of times to let debug drawings render
    world.tick()
    world.tick()
    time.sleep(0.5)

    # Create camera blueprint
    bp_lib = world.get_blueprint_library()
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(width))
    camera_bp.set_attribute('image_size_y', str(height))
    camera_bp.set_attribute('fov', '90')

    # Spawn camera at spectator location
    camera = world.spawn_actor(camera_bp, spectator_transform)

    # Use a queue to receive the image callback
    image_queue = queue.Queue()

    def on_image(image):
        image_queue.put(image)

    camera.listen(on_image)

    # Tick to trigger capture
    world.tick()
    world.tick()

    try:
        image = image_queue.get(timeout=10.0)
        image.save_to_disk(output_path)
        print(f"  Screenshot saved: {output_path}")
    except queue.Empty:
        print(f"  WARNING: Camera capture timed out for {output_path}")

    # Cleanup
    camera.stop()
    camera.destroy()


def write_description_md(output_path: str, town_name: str, map_name: str,
                         n_cavs: int, n_rsus: int, n_frames_per_agent: dict,
                         objects_per_frame: list, conn_counts: list,
                         speeds: list, category_counts: Counter,
                         bbox_min: np.ndarray, bbox_max: np.ndarray):
    """Write a markdown file describing the scenario."""
    total_frames = sum(n_frames_per_agent.values())
    max_agent_frames = max(n_frames_per_agent.values()) if n_frames_per_agent else 0
    duration_s = max_agent_frames / 10.0  # 10 Hz

    # Speed stats (km/h for readability, input is m/s)
    speeds_arr = np.array(speeds) if speeds else np.array([0.0])
    speed_mean = np.mean(speeds_arr) * 3.6
    speed_max = np.max(speeds_arr) * 3.6
    speed_std = np.std(speeds_arr) * 3.6

    # Connection stats
    conn_arr = np.array(conn_counts) if conn_counts else np.array([0])
    conn_mean = np.mean(conn_arr)
    conn_max = np.max(conn_arr)
    conn_median = np.median(conn_arr)

    # Objects per frame stats
    opf_arr = np.array(objects_per_frame) if objects_per_frame else np.array([0])
    opf_mean = np.mean(opf_arr)
    opf_max = np.max(opf_arr)
    opf_std = np.std(opf_arr)

    # Spatial extent
    extent_x = bbox_max[0] - bbox_min[0]
    extent_y = bbox_max[1] - bbox_min[1]

    # Category list
    categories_observed = sorted(category_counts.keys())

    # Connection distribution histogram (binned)
    conn_hist = Counter(conn_arr.tolist())
    conn_dist_lines = []
    for k in sorted(conn_hist.keys()):
        conn_dist_lines.append(f"  - {int(k)} connections: {conn_hist[k]} samples")

    # Qualitative description
    base_town = map_name.replace('HD', '')
    if map_name in TOWN_DESCRIPTIONS:
        qual_desc = TOWN_DESCRIPTIONS[map_name]
    elif base_town in TOWN_DESCRIPTIONS:
        qual_desc = TOWN_DESCRIPTIONS[base_town]
    else:
        qual_desc = f'{map_name} environment.'

    md = f"""# {town_name}

## Map
- **CARLA Town**: {map_name}
- **Dataset directory**: {town_name}

## Agents
- **CAVs**: {n_cavs}
- **RSUs**: {n_rsus}
- **Total agents**: {n_cavs + n_rsus}

## Duration
- **Max frames per agent**: {max_agent_frames}
- **Duration**: {duration_s:.1f} seconds (at 10 Hz)
- **Total agent-frames**: {total_frames}

## Object Categories Observed
{chr(10).join(f'- {cat}: {category_counts[cat]} observations (sampled)' for cat in categories_observed) if categories_observed else '- None observed'}

## Objects Per Frame
- **Mean**: {opf_mean:.1f}
- **Max**: {int(opf_max)}
- **Std**: {opf_std:.1f}

## Connection Statistics
- **Mean connections per agent-frame**: {conn_mean:.2f}
- **Median**: {conn_median:.0f}
- **Max**: {int(conn_max)}

### Connection Count Distribution (sampled)
{chr(10).join(conn_dist_lines)}

## Speed Statistics
- **Mean speed**: {speed_mean:.1f} km/h ({speed_mean/3.6:.1f} m/s)
- **Max speed**: {speed_max:.1f} km/h ({speed_max/3.6:.1f} m/s)
- **Std**: {speed_std:.1f} km/h

## Spatial Coverage
- **Bounding box**: x=[{bbox_min[0]:.1f}, {bbox_max[0]:.1f}], y=[{bbox_min[1]:.1f}, {bbox_max[1]:.1f}]
- **Extent**: {extent_x:.1f} m x {extent_y:.1f} m

## Scenario Description
{qual_desc}

## Screenshot
![{town_name} overview]({town_name}_overview.png)
"""

    with open(output_path, 'w') as f:
        f.write(md)
    print(f"  Description saved: {output_path}")


def process_town(client, data_root: str, town_name: str, output_dir: str):
    """Process a single town: load data, draw, capture, compute stats, write md."""
    print(f"\n{'='*60}")
    print(f"Processing {town_name}")
    print(f"{'='*60}")

    town_dir = Path(data_root) / town_name

    if not town_dir.exists():
        print(f"  WARNING: Town directory not found: {town_dir}")
        return

    # 1. Load trajectories (fast line-scan)
    print("  Loading trajectories...")
    map_name, trajectories, rsu_positions, cav_names, rsu_names = \
        load_town_trajectories(town_dir)

    if not trajectories:
        print("  WARNING: No trajectories loaded, skipping.")
        return

    # 2. Load correct map
    world = client.get_world()
    current_map = world.get_map().name.split('/')[-1]
    if current_map != map_name:
        print(f"  Loading map {map_name} (current: {current_map})...")
        # Set synchronous mode off before loading
        settings = world.get_settings()
        settings.synchronous_mode = False
        world.apply_settings(settings)

        world = client.load_world(map_name)
        time.sleep(3.0)

    # Enable synchronous mode for controlled ticking
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 3. Draw trajectories and RSU positions
    print("  Drawing trajectories...")
    draw_on_carla(world, trajectories, rsu_positions, line_life=300.0)

    # 4. Compute spectator transform and capture screenshot
    print("  Computing overview camera position...")
    spec_transform, centroid, bbox_min, bbox_max = \
        compute_spectator_transform(trajectories, rsu_positions)

    screenshot_path = os.path.join(output_dir, f'{town_name}_overview.png')
    print("  Capturing screenshot...")
    capture_screenshot(world, spec_transform, screenshot_path)

    # 5. Compute statistics (sample frames with full yaml parse)
    print("  Computing scenario statistics (sampling every 10th frame)...")
    all_agent_names = cav_names + rsu_names
    objects_per_frame, conn_counts, speeds, category_counts = \
        sample_frame_statistics(town_dir, all_agent_names, sample_every=10)

    # Frame counts per agent
    n_frames_per_agent = {}
    for agent_name, traj in trajectories.items():
        n_frames_per_agent[agent_name] = len(traj)

    # 6. Write markdown description
    md_path = os.path.join(output_dir, f'{town_name}_description.md')
    write_description_md(
        md_path, town_name, map_name,
        n_cavs=len(cav_names), n_rsus=len(rsu_names),
        n_frames_per_agent=n_frames_per_agent,
        objects_per_frame=objects_per_frame,
        conn_counts=conn_counts,
        speeds=speeds,
        category_counts=category_counts,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
    )

    # Restore async mode
    settings = world.get_settings()
    settings.synchronous_mode = False
    world.apply_settings(settings)

    print(f"  Done with {town_name}.")


def main():
    parser = argparse.ArgumentParser(
        description='Document all Multi-V2X scenarios with CARLA visualization and stats.')
    parser.add_argument('--data-root', default='/data1/Datasets/Multi-V2X',
                        help='Root directory of Multi-V2X dataset')
    parser.add_argument('--host', default='localhost',
                        help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000,
                        help='CARLA server port')
    parser.add_argument('--towns', nargs='*', default=None,
                        help='Specific town(s) to process (default: all 6)')
    parser.add_argument('--output-dir',
                        default='paper2_figures/multiv2x_scenarios',
                        help='Output directory for screenshots and descriptions')
    parser.add_argument('--width', type=int, default=1920,
                        help='Screenshot width')
    parser.add_argument('--height', type=int, default=1080,
                        help='Screenshot height')
    args = parser.parse_args()

    towns = args.towns if args.towns else TOWNS

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)

    # Connect to CARLA
    print(f"Connecting to CARLA at {args.host}:{args.port}...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    print(f"Connected. Server version: {client.get_server_version()}")

    for town_name in towns:
        try:
            process_town(client, args.data_root, town_name, args.output_dir)
        except Exception as e:
            print(f"  ERROR processing {town_name}: {e}")
            import traceback
            traceback.print_exc()
            continue

    print(f"\nAll towns processed. Output in {args.output_dir}/")


if __name__ == '__main__':
    main()
