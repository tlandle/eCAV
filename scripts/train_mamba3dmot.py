#!/usr/bin/env python3
"""
Train MambaTrack3D motion predictor on OPV2V GT tracking data.

Uses MambaTrack (Xiao et al., ACM MM 2024) architecture adapted for 3D.
Input: inter-frame motion deltas [dx, dy, dz, dyaw] (4-dim)
Output: predicted next-frame delta (4-dim)
Loss: Smooth L1

Only position (x,y,z) and yaw are predicted. l/w/h never change per-track
in OPV2V GT data, so they are excluded from training.

Normalization: each dimension is divided by 2*std (computed from training data),
then clamped to [-5, 5]. This keeps all features in a range the Mamba SSM
handles stably, matching the published MambaTrack's ~[-0.5, 0.5] regime
after their image-normalization + scale_factor=50.

Usage:
    python scripts/train_mamba3dmot.py [--epochs 50] [--batch-size 64]

Output: ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth
"""

import argparse
import glob
import logging
import math
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
logger = logging.getLogger("TrainMamba3DMOT")

sys.path.insert(0, str(Path(__file__).parent.parent))

MOTION_DIM = 4  # [dx, dy, dz, dyaw]
MOTION_INDICES = [0, 1, 2, 6]  # indices into 7-dim [x,y,z,l,w,h,yaw]
CLAMP_VAL = 1.0  # clamp normalized features to match published ~[-0.5, 0.5] regime


class TrackDeltaDataset(Dataset):
    """
    Dataset of inter-frame motion deltas for MambaTrack training.

    Each sample: (input_deltas, target_delta)
        input_deltas: (q, 4) normalized+clamped deltas for q frames
        target_delta: (4,) normalized+clamped delta for next frame

    Only [dx, dy, dz, dyaw] are used. l/w/h deltas are always zero
    in OPV2V GT data and are excluded.
    """

    def __init__(self, data_dirs, q=10):
        self.q = q
        self.samples = []
        all_deltas = []

        for data_dir in data_dirs:
            files = sorted(glob.glob(os.path.join(data_dir, "*.pickle")))
            for f in files:
                with open(f, 'rb') as fh:
                    data = pickle.load(fh)
                for tid, frames in data['data'].items():
                    arr = np.array(frames, dtype=np.float32)
                    valid = arr[:, -1] == 1
                    seq = arr[valid]
                    if len(seq) < q + 2:
                        continue

                    # Position deltas: (T-1, 3)
                    pos_deltas = np.diff(seq[:, :3], axis=0)

                    # Yaw deltas with proper angular wrapping
                    yaw_rad = np.radians(seq[:, 6])
                    dyaw = np.diff(yaw_rad)
                    dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))

                    # Combined motion deltas: [dx, dy, dz, dyaw]
                    deltas = np.column_stack([pos_deltas, dyaw]).astype(np.float32)

                    # Filter unreasonable deltas
                    valid_pos = np.all(np.abs(deltas[:, :3]) < 5.0, axis=1)
                    valid_yaw = np.abs(deltas[:, 3]) < np.radians(45)
                    valid_mask = valid_pos & valid_yaw

                    # Sliding window: q input deltas, 1 target delta
                    for i in range(len(deltas) - q):
                        if not np.all(valid_mask[i:i + q + 1]):
                            continue
                        self.samples.append((deltas[i:i + q], deltas[i + q]))
                        all_deltas.append(deltas[i + q])

        # Per-dimension scale: use p99 of absolute values so 99% of data
        # lands in [-1, 1] after division. Matches published MambaTrack regime
        # where image-normalized diffs * scale_factor produce ~[-0.5, 0.5].
        if all_deltas:
            arr = np.array(all_deltas)
            self.norm_scale = np.percentile(
                np.abs(arr), 99, axis=0).astype(np.float32)
            self.norm_scale = np.maximum(self.norm_scale, 1e-6)
        else:
            self.norm_scale = np.ones(MOTION_DIM, dtype=np.float32)

        logger.info("Dataset: %d samples from %d dirs", len(self.samples),
                     len(data_dirs))
        logger.info("Per-dim norm_scale (2*std): %s", self.norm_scale)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        inp, tgt = self.samples[idx]
        inp_n = np.clip(inp / self.norm_scale, -CLAMP_VAL, CLAMP_VAL)
        tgt_n = np.clip(tgt / self.norm_scale, -CLAMP_VAL, CLAMP_VAL)
        return torch.from_numpy(inp_n).float(), torch.from_numpy(tgt_n).float()


def train(args):
    device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    logger.info("Device: %s", device)

    # Find data
    data_dirs = []
    for pattern in [
        "preprocessed_data/opv2v/gt/train",
        "preprocessed_data/opv2v/gt_multiego_speedless/train",
    ]:
        full = os.path.join(args.cmp_root, pattern)
        if os.path.isdir(full):
            data_dirs.append(full)
            logger.info("  Found: %s", full)

    if not data_dirs:
        logger.error("No training data found")
        return

    dataset = TrackDeltaDataset(data_dirs, q=args.q)
    if len(dataset) == 0:
        logger.error("No training samples")
        return

    n_train = int(0.9 * len(dataset))
    n_val = len(dataset) - n_train
    train_ds, val_ds = torch.utils.data.random_split(dataset, [n_train, n_val])

    train_loader = DataLoader(train_ds, batch_size=args.batch_size,
                              shuffle=True, num_workers=4, pin_memory=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size,
                            shuffle=False, num_workers=2)

    steps_per_epoch = len(train_loader)
    total_steps = steps_per_epoch * args.epochs
    logger.info("Train: %d, Val: %d, Steps/epoch: %d",
                 n_train, n_val, steps_per_epoch)

    # Model: MambaTrack3D with 4-dim motion deltas
    from ecav.core.tracking.mamba3dmot.MambaTrack import MambaTrack

    cfgs = {
        'd_m': args.d_m,
        'd_state': args.d_state,
        'L': args.L,
        'box_dim': MOTION_DIM,
        'avg_pool_out_dim': [1, 128],
        'pred_head_dims': [64, MOTION_DIM],
    }
    model = MambaTrack(cfgs).to(device)

    # Initialize prediction head last layer with small weights
    # so initial predictions are near-zero (stable starting point)
    for module in model.pred_head:
        if isinstance(module, nn.Linear):
            last_linear = module
    nn.init.xavier_uniform_(last_linear.weight, gain=0.01)
    nn.init.zeros_(last_linear.bias)

    model.train()
    logger.info("MambaTrack3D: d_m=%d, d_state=%d, L=%d, box_dim=%d",
                 args.d_m, args.d_state, args.L, MOTION_DIM)

    # SGD matching published MambaTrack (no momentum, no weight decay)
    optimizer = torch.optim.SGD(model.parameters(), lr=1.0)

    # Per-step transformer warmup schedule (Vaswani et al.)
    # LR = d_model^{-0.5} * min(step^{-0.5}, step * warmup^{-1.5})
    # Peak LR at warmup_steps: ~d_m^{-0.5} * warmup^{-0.5} ≈ 7e-4 for d_m=512, warmup=4000
    warmup_steps = args.warmup_steps
    d_m = args.d_m

    def lr_lambda(step):
        s = max(step, 1)
        return (d_m ** -0.5) * min(s ** -0.5, s * warmup_steps ** -1.5)

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda)

    best_val_loss = float('inf')
    output_path = args.output
    global_step = 0

    for epoch in range(args.epochs):
        model.train()
        train_loss = 0
        nan_batches = 0

        for batch_x, batch_y in train_loader:
            batch_x = batch_x.to(device)
            batch_y = batch_y.to(device)

            loss = model(batch_x, label=batch_y)

            if torch.isnan(loss) or torch.isinf(loss):
                nan_batches += 1
                optimizer.zero_grad()
                global_step += 1
                scheduler.step()
                continue

            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()
            global_step += 1
            scheduler.step()
            train_loss += loss.item()

        n_good = len(train_loader) - nan_batches
        train_loss = train_loss / max(n_good, 1)

        if nan_batches > 0:
            logger.warning("Epoch %d: %d/%d batches had NaN/Inf loss",
                           epoch + 1, nan_batches, len(train_loader))

        model.eval()
        val_loss = 0
        val_count = 0
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                batch_x = batch_x.to(device)
                batch_y = batch_y.to(device)
                loss = model(batch_x, label=batch_y)
                if not (torch.isnan(loss) or torch.isinf(loss)):
                    val_loss += loss.item()
                    val_count += 1
        val_loss = val_loss / max(val_count, 1)

        lr = optimizer.param_groups[0]['lr']
        logger.info("Epoch %3d/%d: train=%.6f val=%.6f lr=%.2e step=%d",
                     epoch + 1, args.epochs, train_loss, val_loss, lr, global_step)

        if val_loss < best_val_loss and not math.isnan(val_loss):
            best_val_loss = val_loss
            torch.save({
                'model': model.state_dict(),
                'epoch': epoch + 1,
                'val_loss': val_loss,
                'cfgs': cfgs,
                'norm_scale': dataset.norm_scale,
                'motion_indices': MOTION_INDICES,
                'clamp_val': CLAMP_VAL,
            }, output_path)
            logger.info("  -> Saved best (val_loss=%.6f)", val_loss)

    logger.info("Done. Best val=%.6f -> %s", best_val_loss, output_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cmp-root', default='/home/atlas/TrafficSimulator_eCloud/SOTA_Predictors/CMP')
    parser.add_argument('--epochs', type=int, default=50)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--q', type=int, default=10)
    parser.add_argument('--d-m', type=int, default=512)
    parser.add_argument('--d-state', type=int, default=16)
    parser.add_argument('--L', type=int, default=3)
    parser.add_argument('--warmup-steps', type=int, default=4000)
    parser.add_argument('--output', default='ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth')
    args = parser.parse_args()
    train(args)


if __name__ == '__main__':
    main()
