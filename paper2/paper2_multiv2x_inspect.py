#!/usr/bin/env python3
"""Inspect Multi-V2X dataset structure and enumerate scenes.

Multi-V2X layout (RadetzkyLi custom OpenCOOD fork):
  {data_root}/{scenario}/cav_{id}/{timestamp}.{pcd,jpg,yaml}
  {data_root}/{scenario}/rsu_{id}/{timestamp}.{pcd,jpg,yaml}

Camera files use named views (cameraFront/Rear/Left/Right for CAVs,
cameraForward/Backward for RSUs) with .jpg extension, not .png.
A _view.yaml companion file exists alongside the main {timestamp}.yaml.

This script walks the dataset once and produces a summary JSON that the
offline profiler and scenario extractor can consume without re-scanning.

Usage:
    python paper2_multiv2x_inspect.py --data-root /media/atlas/HDD/Datasets/Multi-V2X
"""

import os
import sys
import json
import argparse
from collections import defaultdict
from pathlib import Path


def _is_agent_dir(name: str) -> str:
    """Return 'cav' or 'rsu' if the directory name matches, else ''."""
    if name.startswith('cav_'):
        return 'cav'
    if name.startswith('rsu_'):
        return 'rsu'
    return ''


def _agent_id(name: str) -> int:
    """Parse integer id from 'cav_123' or 'rsu_42'."""
    try:
        return int(name.split('_', 1)[1])
    except (IndexError, ValueError):
        return -1


def _frame_yamls(agent_dir: Path):
    """Return sorted list of frame yaml paths, excluding *_view.yaml."""
    out = []
    for p in agent_dir.glob('*.yaml'):
        stem = p.stem
        if stem.endswith('_view'):
            continue
        # Frame yamls have 6-digit numeric stems in Multi-V2X
        if not stem.isdigit():
            continue
        out.append(p)
    out.sort()
    return out


def scan_dataset(data_root: Path) -> dict:
    """Walk Multi-V2X directory and enumerate scenarios, CAVs, RSUs, frames."""
    scenarios = {}

    for scenario_dir in sorted(data_root.iterdir()):
        if not scenario_dir.is_dir():
            continue
        scenario_name = scenario_dir.name

        cavs = {}
        rsus = {}
        for agent_dir in sorted(scenario_dir.iterdir()):
            if not agent_dir.is_dir():
                continue
            agent_type = _is_agent_dir(agent_dir.name)
            if not agent_type:
                continue

            yamls = _frame_yamls(agent_dir)
            timestamps = [y.stem for y in yamls]
            n_pcd = sum(1 for _ in agent_dir.glob('*.pcd'))
            n_jpg = sum(1 for _ in agent_dir.glob('*.jpg'))

            entry = {
                'agent_id': _agent_id(agent_dir.name),
                'n_frames': len(timestamps),
                'first_timestamp': timestamps[0] if timestamps else None,
                'last_timestamp': timestamps[-1] if timestamps else None,
                'n_pcd': n_pcd,
                'n_jpg': n_jpg,
            }
            if agent_type == 'cav':
                cavs[agent_dir.name] = entry
            else:
                rsus[agent_dir.name] = entry

        if cavs or rsus:
            # Timestamp intersection across CAVs: frames where all CAVs
            # have data. This is the set usable for cooperative perception.
            cav_frame_sets = [set(e['first_timestamp'] and
                                  _collect_timestamps(scenario_dir / name))
                              for name, e in cavs.items()]
            common = set.intersection(*cav_frame_sets) if cav_frame_sets else set()

            scenarios[scenario_name] = {
                'n_cavs': len(cavs),
                'n_rsus': len(rsus),
                'n_agents': len(cavs) + len(rsus),
                'n_common_frames': len(common),
                'cavs': cavs,
                'rsus': rsus,
            }

    return scenarios


def _collect_timestamps(agent_dir: Path):
    return {y.stem for y in _frame_yamls(agent_dir)}


def summarize(scenarios: dict) -> dict:
    """Compute high-level statistics about the dataset."""
    total_cavs = sum(s['n_cavs'] for s in scenarios.values())
    total_rsus = sum(s['n_rsus'] for s in scenarios.values())
    agent_counts = [s['n_agents'] for s in scenarios.values()]
    frame_counts = []
    for s in scenarios.values():
        for agent in list(s['cavs'].values()) + list(s['rsus'].values()):
            frame_counts.append(agent['n_frames'])

    if not agent_counts:
        return {'n_scenarios': 0, 'error': 'no scenarios found'}

    # Bucket scenes by how many agents they have, so downstream scripts
    # can select scenes at target N without rescanning.
    buckets = defaultdict(int)
    for n in agent_counts:
        if n >= 32:
            buckets['>=32'] += 1
        elif n >= 16:
            buckets['16-31'] += 1
        elif n >= 8:
            buckets['8-15'] += 1
        elif n >= 4:
            buckets['4-7'] += 1
        else:
            buckets['<4'] += 1

    return {
        'n_scenarios': len(scenarios),
        'total_cavs': total_cavs,
        'total_rsus': total_rsus,
        'agents_per_scenario': {
            'min': min(agent_counts),
            'max': max(agent_counts),
            'mean': sum(agent_counts) / len(agent_counts),
        },
        'agent_bucket_counts': dict(buckets),
        'frames_per_agent': {
            'min': min(frame_counts) if frame_counts else 0,
            'max': max(frame_counts) if frame_counts else 0,
            'mean': (sum(frame_counts) / len(frame_counts)) if frame_counts else 0,
        },
        'total_frames': sum(frame_counts),
    }


def inspect_sample_yaml(data_root: Path, scenarios: dict) -> dict:
    """Read one frame yaml to verify format matches OPV2V."""
    if not scenarios:
        return {}
    first_scene = next(iter(scenarios.keys()))
    cavs = scenarios[first_scene]['cavs']
    if not cavs:
        return {}
    first_cav = next(iter(cavs.keys()))
    cav_dir = data_root / first_scene / first_cav
    yamls = _frame_yamls(cav_dir)
    if not yamls:
        return {}

    import yaml
    with open(yamls[0]) as f:
        try:
            data = yaml.safe_load(f)
        except Exception as e:
            return {'error': str(e)}

    return {
        'sample_path': str(yamls[0]),
        'top_level_keys': list(data.keys()) if isinstance(data, dict) else None,
        'has_lidar_pose': 'lidar_pose' in (data or {}),
        'has_vehicles': 'vehicles' in (data or {}),
        'has_true_ego_pos': 'true_ego_pos' in (data or {}),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', required=True, type=str,
                        help='Path to Multi-V2X root directory')
    parser.add_argument('--out', default='paper2_figures/multiv2x_inventory.json',
                        help='Output JSON path')
    args = parser.parse_args()

    root = Path(args.data_root)
    if not root.is_dir():
        print(f"ERROR: {root} is not a directory")
        sys.exit(1)

    print(f"Scanning {root}...")
    scenarios = scan_dataset(root)
    summary = summarize(scenarios)
    sample = inspect_sample_yaml(root, scenarios)

    output = {
        'data_root': str(root),
        'summary': summary,
        'sample_yaml': sample,
        'scenarios': scenarios,
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump(output, f, indent=2)

    print(f"\n=== Summary ===")
    print(json.dumps(summary, indent=2))
    print(f"\n=== Sample YAML ({sample.get('sample_path', 'none')}) ===")
    print(json.dumps(sample, indent=2))
    print(f"\nSaved inventory to {out_path}")


if __name__ == '__main__':
    main()
