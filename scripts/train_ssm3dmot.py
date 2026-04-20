#!/usr/bin/env python3
"""
Train SSM3DMOT's Mamba motion predictor on OPV2V GT tracking data.

Uses CMP's preprocessed GT trajectories (x,y,z,l,w,h,heading) to train
the SSMMotionPredictor to predict next-frame state from history.

Usage:
    python scripts/train_ssm3dmot.py [--epochs 50] [--batch-size 128] [--lr 1e-3]

Input data: preprocessed_data/opv2v/gt_multiego_speedless/test/*.pickle
    Each pickle has {data: {track_id: [(x,y,z,l,w,h,heading,valid), ...]}, timestamps: [...]}

Output: ecav/core/tracking/ssm3dmot_weights.pth
"""

import argparse
import glob
import logging
import os
import pickle
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')
logger = logging.getLogger("TrainSSM3DMOT")

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))


class TrackSequenceDataset(Dataset):
    """
    Dataset of track history windows for SSM motion prediction training.

    Each sample: (input_history, target_state)
        input_history: (Q+1, 14) = [state(7) + diff(7)] for Q+1 frames
        target_state: (7,) = next frame's [x,y,z,l,w,h,heading]
    """

    def __init__(self, data_dirs, window_size=5, scale_pos=0.01, scale_angle=0.01):
        self.window_size = window_size
        self.scale_pos = scale_pos
        self.scale_angle = scale_angle
        self.samples = []

        for data_dir in data_dirs:
            files = glob.glob(os.path.join(data_dir, "*.pickle"))
            for f in files:
                with open(f, 'rb') as fh:
                    data = pickle.load(fh)
                for tid, frames in data['data'].items():
                    arr = np.array(frames, dtype=np.float32)
                    valid = arr[:, -1] == 1
                    seq = arr[valid, :7]  # drop valid flag
                    if len(seq) < window_size + 2:
                        continue
                    # Slide window
                    for i in range(len(seq) - window_size - 1):
                        history = seq[i:i + window_size + 1]  # Q+1 frames
                        target = seq[i + window_size + 1]     # next frame
                        self.samples.append((history, target))

        logger.info("Dataset: %d samples from %s", len(self.samples),
                     [str(d) for d in data_dirs])

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        history, target = self.samples[idx]

        # Compute diffs
        states = history.copy()
        diffs = np.zeros_like(states)
        diffs[1:] = states[1:] - states[:-1]

        # Scale positions and angles for better convergence
        # (matching TrackSSM's scale_factor approach)
        states[:, :3] *= self.scale_pos    # x,y,z
        states[:, 3:6] *= self.scale_pos   # l,w,h
        states[:, 6] *= self.scale_angle   # heading
        diffs[:, :3] *= self.scale_pos
        diffs[:, 3:6] *= self.scale_pos
        diffs[:, 6] *= self.scale_angle

        input_features = np.concatenate([states, diffs], axis=1)  # (Q+1, 14)

        # Target: delta from last history frame
        delta = target - history[-1]
        delta[:3] *= self.scale_pos
        delta[3:6] *= self.scale_pos
        delta[6] *= self.scale_angle

        return (torch.from_numpy(input_features).float(),
                torch.from_numpy(delta).float())


def train(args):
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    logger.info("Device: %s", device)

    # Find data
    data_dirs = []
    for pattern in [
        "preprocessed_data/opv2v/gt_multiego_speedless/cv_train",
        "preprocessed_data/opv2v/gt_multiego_speedless/train",
    ]:
        full = os.path.join(args.cmp_root, pattern)
        if os.path.isdir(full):
            data_dirs.append(full)

    if not data_dirs:
        logger.error("No training data found. Set --cmp-root to CMP directory.")
        return

    dataset = TrackSequenceDataset(data_dirs, window_size=args.window_size)
    if len(dataset) == 0:
        logger.error("No training samples generated")
        return

    # Split 90/10
    n_train = int(0.9 * len(dataset))
    n_val = len(dataset) - n_train
    train_ds, val_ds = torch.utils.data.random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_ds, batch_size=args.batch_size,
                              shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size,
                            shuffle=False, num_workers=2)

    logger.info("Train: %d, Val: %d", n_train, n_val)

    # Model
    from ecav.core.tracking.ssm3dmot import SSMMotionPredictor
    model = SSMMotionPredictor(
        d_model=args.d_model,
        d_state=args.d_state,
        input_dim=14,
        output_dim=7,
        device=device,
    ).train()

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr,
                                   weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=args.epochs, eta_min=args.lr * 0.01)
    loss_fn = nn.SmoothL1Loss()

    best_val_loss = float('inf')
    output_path = args.output

    for epoch in range(args.epochs):
        # Train
        model.train()
        train_loss = 0
        for batch_x, batch_y in train_loader:
            batch_x = batch_x.to(device)
            batch_y = batch_y.to(device)

            pred_delta = model(batch_x)
            loss = loss_fn(pred_delta, batch_y)

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_loss += loss.item()

        train_loss /= len(train_loader)
        scheduler.step()

        # Validate
        model.eval()
        val_loss = 0
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                batch_x = batch_x.to(device)
                batch_y = batch_y.to(device)
                pred_delta = model(batch_x)
                val_loss += loss_fn(pred_delta, batch_y).item()
        val_loss /= len(val_loader)

        lr = optimizer.param_groups[0]['lr']
        logger.info("Epoch %3d/%d: train=%.6f val=%.6f lr=%.2e",
                     epoch + 1, args.epochs, train_loss, val_loss, lr)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save({
                'model': model.state_dict(),
                'epoch': epoch + 1,
                'val_loss': val_loss,
                'config': {
                    'd_model': args.d_model,
                    'd_state': args.d_state,
                    'window_size': args.window_size,
                },
            }, output_path)
            logger.info("  -> Saved best model (val_loss=%.6f)", val_loss)

    logger.info("Training complete. Best val_loss=%.6f saved to %s",
                 best_val_loss, output_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cmp-root', default='SOTA_Predictors/CMP',
                        help='Path to CMP directory with preprocessed data')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch-size', type=int, default=128)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--window-size', type=int, default=5)
    parser.add_argument('--d-model', type=int, default=256)
    parser.add_argument('--d-state', type=int, default=16)
    parser.add_argument('--output', default='ecav/core/tracking/ssm3dmot_weights.pth')
    args = parser.parse_args()
    train(args)


if __name__ == '__main__':
    main()
