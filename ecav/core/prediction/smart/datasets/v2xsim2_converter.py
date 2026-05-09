# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Convert V2X-Sim 2.0 data to SMART's expected HeteroData format.

SMART expects per-scenario HeteroData with:
  agent['position']:   (N_agents, T_total, 3)  -- x, y, z
  agent['heading']:    (N_agents, T_total)      -- radians
  agent['valid_mask']: (N_agents, T_total)      -- bool
  agent['type']:       (N_agents,)              -- 0=vehicle, 1=ped, 2=cyclist
  agent['id']:         list of str              -- unique IDs
  agent['shape']:      (N_agents, T_total, 3)   -- length, width, height
  agent['av_index']:   int                      -- index of the AV/ego

  T_total = num_historical_steps + num_future_steps (e.g. 11 + 80 = 91)

V2X-Sim 2.0 provides GT boxes at 10Hz per scene with persistent object IDs.
This matches SMART's 10Hz expectation directly.
"""

import os
import pickle
import logging
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Tuple

import numpy as np
import torch
from torch_geometric.data import HeteroData
from tqdm import tqdm

logger = logging.getLogger(__name__)

NUM_HISTORICAL_STEPS = 11  # SMART expects 11 frames of history at 10Hz
NUM_FUTURE_STEPS = 80      # SMART predicts 80 frames at 10Hz (8 seconds)
TOTAL_STEPS = NUM_HISTORICAL_STEPS + NUM_FUTURE_STEPS  # 91


def load_v2xsim2_info(info_path: str) -> list:
    """Load the V2X-Sim 2.0 info pkl."""
    with open(info_path, 'rb') as f:
        return pickle.load(f)


def group_by_scene(raw_data: list) -> Dict[str, Dict[int, dict]]:
    """Group samples by scene ID and frame index."""
    scenes = {}
    for sample in raw_data:
        lp = sample.get('lidar_path_1', '')
        fname = os.path.basename(lp).replace('.pcd.bin', '')
        parts = fname.split('_')
        if len(parts) >= 3:
            scene_id = f'{parts[0]}_{parts[1]}'
            frame_idx = int(parts[2])
            if scene_id not in scenes:
                scenes[scene_id] = {}
            scenes[scene_id][frame_idx] = sample
    return scenes


def extract_scene_trajectories(
    frames: Dict[int, dict]
) -> Dict[int, Dict[int, Tuple[float, ...]]]:
    """
    Extract per-object trajectories from a scene.

    Returns:
        {object_id: {frame_idx: (x, y, z, l, w, h, yaw_rad)}}
    """
    obj_data = defaultdict(dict)
    sorted_frames = sorted(frames.keys())

    for frame_idx in sorted_frames:
        sample = frames[frame_idx]
        for agent_key in ['labels_1', 'labels_2']:
            if agent_key not in sample:
                continue
            labels = sample[agent_key]
            boxes = labels.get('gt_boxes_global', np.array([]))
            obj_ids = labels.get('gt_object_ids', np.array([]))
            if len(boxes) == 0:
                continue
            for i, obj_id in enumerate(obj_ids):
                obj_id = int(obj_id)
                box = boxes[i]
                # [x, y, z, l, w, h, yaw_rad, vx, vy, vz]
                if frame_idx not in obj_data[obj_id]:
                    obj_data[obj_id][frame_idx] = (
                        float(box[0]), float(box[1]), float(box[2]),
                        float(box[3]), float(box[4]), float(box[5]),
                        float(box[6]))

    return dict(obj_data)


def scene_to_smart_scenarios(
    scene_id: str,
    frames: Dict[int, dict],
    obj_data: Dict[int, Dict[int, Tuple]],
    stride: int = 5,
) -> List[HeteroData]:
    """
    Convert one V2X-Sim scene into multiple SMART training scenarios.

    Each scenario is centered on a different timestep, with stride frames
    between successive scenarios from the same scene.

    Returns list of HeteroData objects ready for SMART training.
    """
    sorted_frames = sorted(frames.keys())
    n_frames = len(sorted_frames)
    scenarios = []

    # Need at least TOTAL_STEPS frames
    if n_frames < TOTAL_STEPS:
        return scenarios

    for start_idx in range(0, n_frames - TOTAL_STEPS + 1, stride):
        window_frames = sorted_frames[start_idx:start_idx + TOTAL_STEPS]

        # Find all objects present in any frame of this window
        all_obj_ids = set()
        for f in window_frames:
            for obj_id, fdata in obj_data.items():
                if f in fdata:
                    all_obj_ids.add(obj_id)

        if len(all_obj_ids) == 0:
            continue

        obj_ids = sorted(all_obj_ids)
        N = len(obj_ids)

        # Build arrays
        position = np.zeros((N, TOTAL_STEPS, 3), dtype=np.float32)
        heading = np.zeros((N, TOTAL_STEPS), dtype=np.float32)
        valid_mask = np.zeros((N, TOTAL_STEPS), dtype=bool)
        shape = np.zeros((N, TOTAL_STEPS, 3), dtype=np.float32)

        for i, obj_id in enumerate(obj_ids):
            fdata = obj_data[obj_id]
            for j, f in enumerate(window_frames):
                if f in fdata:
                    x, y, z, l, w, h, yaw = fdata[f]
                    position[i, j] = [x, y, z]
                    heading[i, j] = yaw
                    valid_mask[i, j] = True
                    shape[i, j] = [l, w, h]

        # Pick AV index: choose the object with the most valid frames
        valid_counts = valid_mask.sum(axis=1)
        av_index = int(valid_counts.argmax())

        # Create HeteroData
        data = HeteroData()
        data['agent'] = {
            'num_nodes': N,
            'position': torch.tensor(position, dtype=torch.float32),
            'heading': torch.tensor(heading, dtype=torch.float32),
            'valid_mask': torch.tensor(valid_mask, dtype=torch.bool),
            'type': torch.zeros(N, dtype=torch.uint8),  # all vehicles
            'category': torch.zeros(N, dtype=torch.long),
            'id': [str(oid) for oid in obj_ids],
            'shape': torch.tensor(shape, dtype=torch.float32),
            'av_index': torch.tensor(av_index, dtype=torch.long),
        }

        scenarios.append(data)

    return scenarios


def convert_v2xsim2_to_smart(
    info_path: str,
    output_dir: str,
    stride: int = 5,
    split: str = 'train',
):
    """
    Main conversion: V2X-Sim 2.0 info pkl -> SMART scenario files.

    Each scenario is saved as a separate .pt file.
    An index file maps scenario IDs to file paths.
    """
    output_path = Path(output_dir) / split
    output_path.mkdir(parents=True, exist_ok=True)

    logger.info(f'Loading V2X-Sim 2.0 info from {info_path}')
    raw_data = load_v2xsim2_info(info_path)

    logger.info('Grouping by scene...')
    scenes = group_by_scene(raw_data)
    logger.info(f'Found {len(scenes)} scenes')

    all_scenarios = []
    scenario_idx = 0

    for scene_id in tqdm(sorted(scenes.keys()), desc=f'Converting {split}'):
        frames = scenes[scene_id]
        obj_data = extract_scene_trajectories(frames)
        scene_scenarios = scene_to_smart_scenarios(
            scene_id, frames, obj_data, stride=stride)

        for data in scene_scenarios:
            fname = f'scenario_{scenario_idx:06d}.pt'
            torch.save(data, output_path / fname)
            all_scenarios.append({
                'file': fname,
                'scene_id': scene_id,
                'num_agents': data['agent']['num_nodes'],
            })
            scenario_idx += 1

    # Save index
    index_path = output_path / 'index.pkl'
    with open(index_path, 'wb') as f:
        pickle.dump(all_scenarios, f)

    logger.info(f'Converted {scenario_idx} scenarios from {len(scenes)} scenes '
                f'to {output_path}')
    return scenario_idx


if __name__ == '__main__':
    import argparse
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(
        description='Convert V2X-Sim 2.0 to SMART training format')
    parser.add_argument('--train_info',
                        default='ecav/worldfusion/data/v2xsim2_info/v2xsim_infos_train.pkl')
    parser.add_argument('--val_info',
                        default='ecav/worldfusion/data/v2xsim2_info/v2xsim_infos_val.pkl')
    parser.add_argument('--output_dir',
                        default='ecav/core/prediction/smart/data/v2xsim2')
    parser.add_argument('--stride', type=int, default=5)
    args = parser.parse_args()

    n_train = convert_v2xsim2_to_smart(
        args.train_info, args.output_dir, stride=args.stride, split='train')
    n_val = convert_v2xsim2_to_smart(
        args.val_info, args.output_dir, stride=args.stride, split='val')

    print(f'\nDone. Train: {n_train} scenarios, Val: {n_val} scenarios')
