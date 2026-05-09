# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2X-Sim 2.0 dataset loader for CMP/MTR motion prediction training.

V2X-Sim 2.0 uses nuScenes-style format with:
  - Per-frame GT boxes in global coordinates: [x, y, z, l, w, h, yaw, vx, vy, vz]
  - Persistent gt_object_ids across frames within a scene
  - 100 frames per scene at 10Hz (frames 6-105)
  - 2 cooperating agents (vehicles) + 1 RSU (id_0) per scene
  - Up to 20 surrounding vehicles per frame

Trajectory format matches CMP/MTR contract:
  [x, y, z, l, w, h, heading_rad, valid]

The loader:
  1. Groups frames by scene using lidar_path naming convention
  2. Links gt_object_ids across frames to build full trajectories
  3. For each ego timestamp, extracts PAST_FRAMES history + FUTURE_FRAMES GT
  4. Applies the same normalization as OPV2V (transform_trajs_to_center_coords)
"""

import os
import math
import pickle
import logging
from collections import OrderedDict
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from tqdm import tqdm

from mtr.datasets.dataset import DatasetTemplate
from mtr.utils import common_utils

logger = logging.getLogger(__name__)


class V2XSim2MultiEgoDataset(DatasetTemplate):
    """V2X-Sim 2.0 dataset for cooperative motion prediction training."""

    def __init__(self, dataset_cfg, training=True, logger=None, visualize=False):
        super().__init__(dataset_cfg=dataset_cfg, training=training, logger=logger)
        self.is_train = training
        self.visualize = visualize

        # Load the info pkl (already preprocessed by opencood/worldfusion)
        if self.is_train:
            info_path = dataset_cfg['train_info_path']
        else:
            info_path = dataset_cfg['val_info_path']

        self.past_frames = dataset_cfg.get('PAST_FRAMES', 10)
        self.future_frames = dataset_cfg.get('FUTURE_FRAMES', 50)
        self.max_objects = dataset_cfg.get('MAX_OBJECTS', 64)

        # Cache path
        cache_suffix = 'train' if self.is_train else 'val'
        cache_path = dataset_cfg.get('DATASET_CACHE_DIR', 'preprocessed_data/v2xsim2')
        os.makedirs(cache_path, exist_ok=True)
        cache_file = os.path.join(
            cache_path, f'v2xsim2_trajs_{cache_suffix}.pkl')

        if os.path.exists(cache_file):
            if logger:
                logger.info(f'Loading cached V2X-Sim 2.0 trajectories from {cache_file}')
            with open(cache_file, 'rb') as f:
                cache = pickle.load(f)
            self.scene_trajs = cache['scene_trajs']
            self.records = cache['records']
        else:
            if logger:
                logger.info(f'Building V2X-Sim 2.0 trajectories from {info_path}')
            with open(info_path, 'rb') as f:
                raw_data = pickle.load(f)
            self.scene_trajs, self.records = self._build_trajectories(raw_data)

            with open(cache_file, 'wb') as f:
                pickle.dump({'scene_trajs': self.scene_trajs,
                             'records': self.records}, f)
            if logger:
                logger.info(f'Cached {len(self.records)} records to {cache_file}')

        if logger:
            logger.info(f'V2X-Sim 2.0 dataset: {len(self.records)} records, '
                        f'{len(self.scene_trajs)} scenes')

    def _build_trajectories(self, raw_data: list) -> Tuple[dict, list]:
        """
        Build per-scene trajectory dictionaries from the raw info pkl.

        Each sample in raw_data has:
          - lidar_path_1: 'data/v2xsim2_complete/sweeps/LIDAR_TOP_id_1/scene_83_000006.pcd.bin'
          - labels_1: {gt_names, gt_boxes_global, gt_object_ids, gt_boxes_token}
          - lidar_pose_1: (4,4) ego pose
          - Same for agent 2

        Returns:
            scene_trajs: {scene_id: {object_id: [(frame_idx, x, y, z, l, w, h, yaw), ...]}}
            records: [(scene_id, frame_idx)] tuples that have enough past+future
        """
        # Group samples by scene
        scenes: Dict[str, Dict[int, dict]] = {}  # scene_id -> {frame_idx -> sample}

        for sample in raw_data:
            lp = sample.get('lidar_path_1', '')
            fname = os.path.basename(lp).replace('.pcd.bin', '')
            parts = fname.split('_')
            # e.g. scene_83_000006 -> scene_id='scene_83', frame=6
            if len(parts) >= 3:
                scene_id = f'{parts[0]}_{parts[1]}'
                frame_idx = int(parts[2])
            else:
                continue

            if scene_id not in scenes:
                scenes[scene_id] = {}
            scenes[scene_id][frame_idx] = sample

        # Build trajectories per scene
        scene_trajs = {}

        for scene_id, frames in tqdm(scenes.items(), desc='Building trajectories'):
            sorted_frames = sorted(frames.keys())
            obj_trajs: Dict[int, list] = {}  # object_id -> list of (frame, x, y, z, l, w, h, yaw)

            for frame_idx in sorted_frames:
                sample = frames[frame_idx]

                # Collect GT boxes from both agents
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
                        # gt_boxes_global: [x, y, z, l, w, h, yaw, vx, vy, vz]
                        x, y, z = float(box[0]), float(box[1]), float(box[2])
                        l, w, h = float(box[3]), float(box[4]), float(box[5])
                        yaw = float(box[6])  # already in radians

                        entry = (frame_idx, x, y, z, l, w, h, yaw)

                        if obj_id not in obj_trajs:
                            obj_trajs[obj_id] = []

                        # Avoid duplicate frame entries from multiple agents
                        if not obj_trajs[obj_id] or obj_trajs[obj_id][-1][0] != frame_idx:
                            obj_trajs[obj_id].append(entry)

            # Sort each trajectory by frame
            for obj_id in obj_trajs:
                obj_trajs[obj_id].sort(key=lambda x: x[0])

            scene_trajs[scene_id] = obj_trajs

        # Build records: (scene_id, frame_idx) where we have enough history and future
        records = []
        for scene_id, obj_trajs in scene_trajs.items():
            sorted_frames = sorted(scenes[scene_id].keys())
            if len(sorted_frames) < self.past_frames + self.future_frames + 1:
                continue

            for t_idx in range(self.past_frames, len(sorted_frames) - self.future_frames):
                frame = sorted_frames[t_idx]
                # Check that at least some objects have valid trajectories here
                n_valid = 0
                for obj_id, traj in obj_trajs.items():
                    traj_frames = [e[0] for e in traj]
                    if frame in traj_frames:
                        n_valid += 1
                if n_valid >= 1:
                    records.append((scene_id, frame, t_idx, sorted_frames))

        return scene_trajs, records

    def __len__(self):
        return len(self.records)

    def __getitem__(self, index):
        scene_id, cur_frame, t_idx, sorted_frames = self.records[index]
        obj_trajs = self.scene_trajs[scene_id]

        past_frames = sorted_frames[t_idx - self.past_frames: t_idx + 1]
        future_frames = sorted_frames[t_idx + 1: t_idx + 1 + self.future_frames]
        all_frames = past_frames + future_frames

        # Collect trajectories for all objects visible at current frame
        frame_set = set(all_frames)
        visible_obj_ids = []
        for obj_id, traj in obj_trajs.items():
            traj_frames = {e[0] for e in traj}
            if cur_frame in traj_frames:
                visible_obj_ids.append(obj_id)

        if len(visible_obj_ids) == 0:
            return self.__getitem__((index + 1) % len(self))

        # Limit objects
        visible_obj_ids = visible_obj_ids[:self.max_objects]
        N = len(visible_obj_ids)

        # Build trajectory arrays
        n_past = len(past_frames)
        n_future = len(future_frames)
        n_total = n_past + n_future

        # [x, y, z, l, w, h, heading, valid]
        obj_trajs_past = np.zeros((N, n_past, 8), dtype=np.float32)
        obj_trajs_future = np.zeros((N, n_future, 8), dtype=np.float32)

        for i, obj_id in enumerate(visible_obj_ids):
            traj = obj_trajs[obj_id]
            traj_dict = {e[0]: e[1:] for e in traj}  # frame -> (x,y,z,l,w,h,yaw)

            for j, f in enumerate(past_frames):
                if f in traj_dict:
                    x, y, z, l, w, h, yaw = traj_dict[f]
                    obj_trajs_past[i, j] = [x, y, z, l, w, h, yaw, 1.0]

            for j, f in enumerate(future_frames):
                if f in traj_dict:
                    x, y, z, l, w, h, yaw = traj_dict[f]
                    obj_trajs_future[i, j] = [x, y, z, l, w, h, yaw, 1.0]

        # Build full trajectory for normalization
        obj_trajs_full = np.concatenate([obj_trajs_past, obj_trajs_future], axis=1)

        obj_types = np.array(['TYPE_VEHICLE'] * N)
        obj_ids = np.array(visible_obj_ids)
        track_index_to_predict = np.arange(N)

        # Center objects: state at current time (last of past frames)
        center_objects = obj_trajs_past[:, -1, :].copy()

        # Normalize to center-agent coordinates
        (obj_trajs_data, obj_trajs_mask, obj_trajs_pos, obj_trajs_last_pos,
         obj_trajs_future_state, obj_trajs_future_mask,
         center_gt_trajs, center_gt_trajs_mask,
         center_gt_final_valid_idx,
         track_index_to_predict_new, sdc_track_index_new,
         obj_types, obj_ids) = self.create_agent_data_for_center_objects(
            center_objects=center_objects,
            obj_trajs_past=obj_trajs_past,
            obj_trajs_future=obj_trajs_future,
            track_index_to_predict=track_index_to_predict,
            sdc_track_index=0,
            timestamps=np.arange(n_past) * 0.1,
            obj_types=obj_types,
            obj_ids=obj_ids,
        )

        ret_dict = {
            'scenario_id': np.array([scene_id] * len(track_index_to_predict_new)),
            'obj_trajs': obj_trajs_data,
            'obj_trajs_mask': obj_trajs_mask,
            'track_index_to_predict': track_index_to_predict_new,
            'obj_trajs_pos': obj_trajs_pos,
            'obj_trajs_last_pos': obj_trajs_last_pos,
            'obj_types': obj_types,
            'obj_ids': obj_ids,
            'center_objects_world': center_objects,
            'center_objects_id': obj_ids[track_index_to_predict_new],
            'center_objects_type': obj_types[track_index_to_predict_new],
            'obj_trajs_future_state': obj_trajs_future_state,
            'obj_trajs_future_mask': obj_trajs_future_mask,
            'center_gt_trajs': center_gt_trajs,
            'center_gt_trajs_mask': center_gt_trajs_mask,
            'center_gt_final_valid_idx': center_gt_final_valid_idx,
        }

        # Placeholder for lane/map features (can be added later)
        ret_dict['center_gt_trajs_src'] = obj_trajs_full[track_index_to_predict_new]

        return ret_dict

    @staticmethod
    def create_agent_data_for_center_objects(
        center_objects, obj_trajs_past, obj_trajs_future,
        track_index_to_predict, sdc_track_index, timestamps,
        obj_types, obj_ids
    ):
        """
        Normalize trajectories to center-agent coordinates.
        Matches the CMP/MTR create_agent_data_for_center_objects interface.
        """
        num_center_objects = center_objects.shape[0]
        num_objects = obj_trajs_past.shape[0]
        num_past = obj_trajs_past.shape[1]
        num_future = obj_trajs_future.shape[1]

        # Combine past + future
        obj_trajs_full = np.concatenate([obj_trajs_past, obj_trajs_future], axis=1)
        num_total = obj_trajs_full.shape[1]

        # Extract masks
        obj_trajs_mask = obj_trajs_full[:, :num_past, -1].astype(bool)  # (N, past)
        obj_trajs_future_mask = obj_trajs_full[:, num_past:, -1].astype(bool)  # (N, future)

        # Transform to center coordinates
        obj_trajs_torch = torch.tensor(obj_trajs_full).float()
        center_xyz = torch.tensor(center_objects[:, :3]).float()
        center_heading = torch.tensor(center_objects[:, 6]).float()

        obj_trajs_centered = V2XSim2MultiEgoDataset.transform_trajs_to_center_coords(
            obj_trajs_torch.unsqueeze(0).expand(num_center_objects, -1, -1, -1).clone(),
            center_xyz, center_heading, heading_index=6
        )

        # Extract components
        obj_trajs_data = obj_trajs_centered[:, :, :num_past, :].numpy()  # (C, N, past, 8)
        obj_trajs_future_state = obj_trajs_centered[:, :, num_past:, :].numpy()

        # Position fields
        obj_trajs_pos = obj_trajs_data[:, :, :, :2]  # (C, N, past, 2)
        obj_trajs_last_pos = obj_trajs_data[:, :, -1, :2]  # (C, N, 2)

        # Center GT trajectories (future of the center objects)
        center_gt_trajs = obj_trajs_future_state[
            np.arange(num_center_objects), track_index_to_predict, :, :2]
        center_gt_trajs_mask = obj_trajs_future_mask[track_index_to_predict]
        center_gt_final_valid_idx = np.array([
            np.where(center_gt_trajs_mask[i])[0][-1]
            if center_gt_trajs_mask[i].any() else 0
            for i in range(num_center_objects)
        ])

        track_index_to_predict_new = np.arange(num_center_objects)
        sdc_track_index_new = 0

        return (obj_trajs_data, obj_trajs_mask, obj_trajs_pos, obj_trajs_last_pos,
                obj_trajs_future_state, obj_trajs_future_mask,
                center_gt_trajs, center_gt_trajs_mask, center_gt_final_valid_idx,
                track_index_to_predict_new, sdc_track_index_new,
                obj_types, obj_ids)

    @staticmethod
    def transform_trajs_to_center_coords(obj_trajs, center_xyz, center_heading,
                                          heading_index, rot_vel_index=None):
        """
        Transform trajectories to center-agent coordinates.
        Matches CMP/MTR transform_trajs_to_center_coords.
        """
        num_center_objects = center_xyz.shape[0]

        # Subtract center position
        obj_trajs[:, :, :, 0:center_xyz.shape[1]] -= center_xyz[:, None, None, :]

        # Rotate by -center_heading
        obj_trajs[:, :, :, 0:2] = common_utils.rotate_points_along_z(
            points=obj_trajs[:, :, :, 0:2].reshape(num_center_objects, -1, 2),
            angle=-center_heading
        ).reshape(obj_trajs.shape[0], obj_trajs.shape[1], obj_trajs.shape[2], 2)

        # Adjust heading
        obj_trajs[:, :, :, heading_index] -= center_heading[:, None, None]

        return obj_trajs
