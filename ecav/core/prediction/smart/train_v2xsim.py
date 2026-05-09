#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Train SMART on V2X-Sim 2.0 converted scenarios.

Usage:
    # First convert data (if not done):
    python ecav/core/prediction/smart/datasets/v2xsim2_converter.py \
        --output_dir /media/atlas/HDD/Datasets/v2xsim2_smart

    # Then train:
    python ecav/core/prediction/smart/train_v2xsim.py \
        --data_dir /media/atlas/HDD/Datasets/v2xsim2_smart \
        --ckpt ecav/core/prediction/smart/checkpoints/epoch=105.ckpt \
        --epochs 50 \
        --lr 1e-4 \
        --batch_size 4 \
        --gpus 1

The script finetunes the Waymo-pretrained SMART checkpoint on V2X-Sim 2.0
scenarios. This adapts SMART's learned motion priors from Waymo's coordinate
space and traffic patterns to CARLA/V2X-Sim's geometry.
"""

import argparse
import logging
import os
import pickle
import sys
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset
from torch_geometric.data import HeteroData
from torch_geometric.loader import DataLoader
import pytorch_lightning as pl
from pytorch_lightning.callbacks import ModelCheckpoint, LearningRateMonitor

# Add project root and SMART package to path
_project_root = str(Path(__file__).parent.parent.parent.parent.parent)
_smart_root = str(Path(__file__).parent)  # ecav/core/prediction/smart/
sys.path.insert(0, _project_root)
# Make 'smart' importable (SMART's internal imports use 'from smart.xxx import yyy')
if 'smart' not in sys.modules:
    sys.path.insert(0, str(Path(_smart_root).parent))  # ecav/core/prediction/

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class V2XSimSMARTDataset(Dataset):
    """Load pre-converted V2X-Sim scenarios for SMART training."""

    def __init__(self, data_dir: str, split: str = 'train'):
        self.data_dir = Path(data_dir) / split
        index_path = self.data_dir / 'index.pkl'
        with open(index_path, 'rb') as f:
            self.index = pickle.load(f)
        logger.info(f'Loaded {len(self.index)} {split} scenarios from {self.data_dir}')

    def __len__(self):
        return len(self.index)

    def __getitem__(self, idx):
        fname = self.index[idx]['file']
        data = torch.load(self.data_dir / fname, weights_only=False)

        # Apply the same target builder transform as Waymo training
        # (score_ego_agent, score_trained_vehicle, clip to max agents)
        agent = data['agent']
        N = agent['num_nodes']

        # Clip to max 32 agents (matching SMART training)
        if N > 32:
            # Keep the 32 agents with most valid frames
            valid_counts = agent['valid_mask'].sum(dim=1)
            keep_idx = valid_counts.argsort(descending=True)[:32]
            # Update av_index
            old_av = agent['av_index'].item() if isinstance(agent['av_index'], torch.Tensor) else agent['av_index']
            new_av_pos = (keep_idx == old_av).nonzero()
            new_av = new_av_pos[0, 0].item() if len(new_av_pos) > 0 else 0

            for key, val in agent.items():
                if key in ('num_nodes', 'av_index', 'id'):
                    continue
                if isinstance(val, torch.Tensor) and val.shape[0] == N:
                    agent[key] = val[keep_idx]
            agent['num_nodes'] = 32
            agent['av_index'] = torch.tensor(new_av, dtype=torch.long)
            if isinstance(agent.get('id'), list):
                agent['id'] = [agent['id'][i] for i in keep_idx.tolist()]

        # Set categories (all vehicles = 0, ego = 5)
        agent['category'] = torch.zeros(agent['num_nodes'], dtype=torch.long)
        av_idx = agent['av_index'].item() if isinstance(agent['av_index'], torch.Tensor) else agent['av_index']
        agent['category'][av_idx] = 5

        return HeteroData(data)


def main():
    parser = argparse.ArgumentParser(description='Train SMART on V2X-Sim 2.0')
    parser.add_argument('--data_dir', type=str,
                        default='/media/atlas/HDD/Datasets/v2xsim2_smart')
    parser.add_argument('--ckpt', type=str,
                        default='ecav/core/prediction/smart/checkpoints/epoch=105.ckpt',
                        help='Waymo pretrained checkpoint to finetune from')
    parser.add_argument('--output_dir', type=str,
                        default='ecav/core/prediction/smart/checkpoints/v2xsim2')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--lr', type=float, default=1e-4)
    parser.add_argument('--batch_size', type=int, default=4)
    parser.add_argument('--gpus', type=int, default=1)
    parser.add_argument('--num_workers', type=int, default=4)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    # Load datasets
    train_dataset = V2XSimSMARTDataset(args.data_dir, split='train')
    val_dataset = V2XSimSMARTDataset(args.data_dir, split='val')

    train_loader = DataLoader(
        train_dataset, batch_size=args.batch_size, shuffle=True,
        num_workers=args.num_workers, pin_memory=True)
    val_loader = DataLoader(
        val_dataset, batch_size=args.batch_size, shuffle=False,
        num_workers=args.num_workers, pin_memory=True)

    # Load SMART model from Waymo checkpoint
    # Import SMART directly to avoid package __init__ issues
    import importlib.util
    smart_spec = importlib.util.spec_from_file_location(
        "smart_model",
        os.path.join(os.path.dirname(__file__), "model", "smart.py"))
    smart_mod = importlib.util.module_from_spec(smart_spec)

    # Mock waymo_open_dataset before loading SMART (not needed for training)
    import types as _types
    for mod_name in ['waymo_open_dataset', 'waymo_open_dataset.protos',
                     'waymo_open_dataset.protos.sim_agents_submission_pb2']:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = _types.ModuleType(mod_name)
    # Add mock JointScene class (referenced at class definition time in smart.py)
    wod_pb2 = sys.modules['waymo_open_dataset.protos.sim_agents_submission_pb2']
    if not hasattr(wod_pb2, 'JointScene'):
        wod_pb2.JointScene = type('JointScene', (), {})
    if not hasattr(wod_pb2, 'SimAgentResult'):
        wod_pb2.SimAgentResult = type('SimAgentResult', (), {})
    if not hasattr(wod_pb2, 'SimulatedTrajectory'):
        wod_pb2.SimulatedTrajectory = type('SimulatedTrajectory', (), {})

    smart_spec.loader.exec_module(smart_mod)
    SMART = smart_mod.SMART

    logger.info(f'Loading SMART from {args.ckpt}')
    # Allow easydict in checkpoint (saved by original SMART training)
    import easydict
    torch.serialization.add_safe_globals([easydict.EasyDict])
    model = SMART.load_from_checkpoint(args.ckpt, strict=False)

    # Adjust learning rate for finetuning
    model.learning_rate = args.lr

    # Callbacks
    checkpoint_callback = ModelCheckpoint(
        dirpath=args.output_dir,
        filename='v2xsim2-{epoch:03d}-{val_loss:.4f}',
        save_top_k=3,
        monitor='val_loss',
        mode='min',
    )
    lr_monitor = LearningRateMonitor(logging_interval='step')

    # Trainer
    trainer = pl.Trainer(
        max_epochs=args.epochs,
        accelerator='gpu' if args.gpus > 0 else 'cpu',
        devices=args.gpus if args.gpus > 0 else 1,
        callbacks=[checkpoint_callback, lr_monitor],
        log_every_n_steps=10,
        val_check_interval=0.5,
        gradient_clip_val=1.0,
    )

    logger.info(f'Training SMART on V2X-Sim 2.0: {len(train_dataset)} train, '
                f'{len(val_dataset)} val scenarios')
    trainer.fit(model, train_loader, val_loader)

    logger.info(f'Training complete. Checkpoints saved to {args.output_dir}')


if __name__ == '__main__':
    main()
