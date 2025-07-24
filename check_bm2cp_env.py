#!/usr/bin/env python3
"""
check_bm2cp_env.py – one-shot BM2CP sanity-check
"""

import pathlib, sys, yaml, torch

# ── make BM2CP importable ──────────────────────────────────────────
root = pathlib.Path(__file__).resolve().parent
bm2cp = root / "opencda" / "BM2CP"
sys.path.insert(0, str(bm2cp))

from opencood.tools import train_utils          # BM2CP’s helper (already works)

# ── paths to your run ─────────────────────────────────────────────
ckpt_dir = bm2cp / "opencood/logs/opv2v_bm2cp_det_2025_07_03_23_44_53"
yaml_path = ckpt_dir / "config.yaml"
pt_path   = ckpt_dir / "net_epoch21.pth"         # adjust epoch if needed

# ── read YAML with unsafe loader (allows NumPy tags) ──────────────
with yaml_path.open() as f:
    hypes = yaml.load(f, Loader=yaml.UnsafeLoader)

# ── build backbone exactly like inference.py ──────────────────────
device = "cuda:0" if torch.cuda.is_available() else "cpu"
model  = train_utils.create_model(hypes).to(device)
_, _ = train_utils.load_model(ckpt_dir, model, 49)          # epoch 21
model.eval()

print(f"\n✅ BM2CP ready on {device}  "
      f"({sum(p.numel() for p in model.parameters())/1e6:.1f} M params)\n")

