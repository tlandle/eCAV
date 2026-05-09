#!/usr/bin/env python3
"""Multi-V2X scene loader for paper2 offline profiler.

This module depends on RadetzkyLi's OpenCOOD fork, which is the custom
OpenCOOD build the Multi-V2X authors trained on:

    https://github.com/RadetzkyLi/OpenCOOD

It has the Multi-V2X-specific database manager (Opv2vDatabaseManagerV2),
ManagerUtility for cav_N/rsu_N directory parsing, the PR (penetration rate)
config loader, and the key_objects machinery. Our in-repo
ecav/worldfusion/opencood is the older WorldFusion-specific fork without
these features; we do not patch it (it is used for training).

Setup (one-time):

    # clone to HDD next to the dataset
    MULTIV2X_HOME=/media/atlas/HDD/Datasets/Multi-V2X
    OPENCOOD_HOME=/media/atlas/HDD/Repos/Multi-V2X-OpenCOOD
    git clone https://github.com/RadetzkyLi/OpenCOOD "$OPENCOOD_HOME"
    cd "$OPENCOOD_HOME" && python setup.py develop

    # export so this loader can find it
    export MULTIV2X_OPENCOOD_ROOT="$OPENCOOD_HOME"

Two layers:

1. `MultiV2XSceneIndex` (lightweight): reads the JSON produced by
   paper2_multiv2x_inspect.py and provides scene/frame selection by
   target N. No opencood dependency.

2. `MultiV2XFrameLoader` (heavy): uses RadetzkyLi's opencood to load
   scenes in Multi-V2X's PR config format, produces per-agent voxelized
   features, then runs our WorldFusion sensor encoder to return
   post-backbone feature tensors the profiler can consume.
"""

import os
import sys
import json
from pathlib import Path
from typing import Dict, List, Tuple, Optional

REPO = os.path.abspath(os.path.dirname(__file__))


def _locate_multiv2x_opencood() -> str:
    """Find the RadetzkyLi OpenCOOD checkout.

    Priority:
      1. MULTIV2X_OPENCOOD_ROOT env var
      2. /media/atlas/HDD/Repos/Multi-V2X-OpenCOOD (recommended location)
      3. ./external/Multi-V2X-OpenCOOD (in-repo fallback)
    """
    candidates = [
        os.environ.get('MULTIV2X_OPENCOOD_ROOT', ''),
        '/media/atlas/HDD/Repos/Multi-V2X-OpenCOOD',
        os.path.join(REPO, 'external/Multi-V2X-OpenCOOD'),
    ]
    for c in candidates:
        if c and os.path.isdir(os.path.join(c, 'opencood')):
            return c
    raise RuntimeError(
        "RadetzkyLi OpenCOOD fork not found. Clone it:\n"
        "  git clone https://github.com/RadetzkyLi/OpenCOOD "
        "/media/atlas/HDD/Repos/Multi-V2X-OpenCOOD\n"
        "or set MULTIV2X_OPENCOOD_ROOT to its path.")


# ─── Layer 1: lightweight scene index ──────────────────────────────

class MultiV2XSceneIndex:
    """Scene selection over a Multi-V2X inventory JSON."""

    def __init__(self, inventory_path: str):
        with open(inventory_path) as f:
            inv = json.load(f)
        self.data_root = Path(inv['data_root'])
        self.scenarios: Dict[str, dict] = inv['scenarios']

    def scenarios_at_n(self, target_n: int, tol: int = 2) -> List[str]:
        out = []
        for name, meta in self.scenarios.items():
            n = meta.get('n_agents', meta.get('n_cavs', 0))
            if abs(n - target_n) <= tol:
                out.append(name)
        return sorted(out)

    def pick_scenario(self, target_n: int, tol: int = 2,
                       seed: int = 0) -> Optional[str]:
        candidates = self.scenarios_at_n(target_n, tol)
        if not candidates:
            return None
        return candidates[seed % len(candidates)]

    def agent_names(self, scenario_name: str
                    ) -> Tuple[List[str], List[str]]:
        meta = self.scenarios[scenario_name]
        return (sorted(meta.get('cavs', {}).keys()),
                sorted(meta.get('rsus', {}).keys()))

    def common_timestamps(self, scenario_name: str) -> List[str]:
        meta = self.scenarios[scenario_name]
        scenario_root = self.data_root / scenario_name
        ts_sets: List[set] = []
        for cav_name in meta.get('cavs', {}):
            cav_dir = scenario_root / cav_name
            ts = {p.stem for p in cav_dir.glob('*.yaml')
                  if p.stem.isdigit()}
            ts_sets.append(ts)
        return sorted(set.intersection(*ts_sets)) if ts_sets else []


# ─── Layer 2: feature extraction via WorldFusion ───────────────────

class MultiV2XFrameLoader:
    """Run Multi-V2X frames through WorldFusion to post-backbone features.

    Uses RadetzkyLi's OpenCOOD fork for Multi-V2X data loading + voxelization,
    then feeds into our in-repo WorldFusion checkpoint's sensor encoder.
    """

    def __init__(self, wf_model_dir: str, pr_config_path: str,
                 data_root: str, device: str = 'cuda:0',
                 partname: str = 'test', epoch: int = 50):
        # 1. Put RadetzkyLi's opencood on sys.path FIRST so its imports win.
        mv_oc = _locate_multiv2x_opencood()
        if mv_oc not in sys.path:
            sys.path.insert(0, mv_oc)
        # Our WorldFusion model still needs our opencood for its model defs.
        # We import it under a different name by tweaking import order.
        self._mv_oc_root = mv_oc

        # 2. Load Multi-V2X dataset via RadetzkyLi's opencood.
        from opencood.hypes_yaml.yaml_utils import load_yaml as mv_load_yaml
        from opencood.data_utils.datasets.intermediate_fusion_dataset import \
            IntermediateFusionDataset as MVIntermediateDataset

        # Build a minimal params dict that points at the Multi-V2X tree.
        # We piggyback on the trained WorldFusion config for preprocess /
        # postprocess / model args, overriding only the data paths.
        wf_cfg_path = os.path.join(wf_model_dir, 'config.yaml')
        wf_params = mv_load_yaml(wf_cfg_path)
        wf_params['dataset_format'] = 'multi-v2x'
        wf_params['root_dir'] = data_root
        wf_params['validate_dir'] = data_root
        wf_params['pr_setting'] = {
            'value': wf_params.get('pr_setting', {}).get('value', 10),
            'path': pr_config_path,
        }
        self.mv_params = wf_params
        self.mv_dataset = MVIntermediateDataset(
            wf_params, visualize=False, partname=partname)

        # 3. Load our WorldFusion model (from in-repo opencood fork).
        # We isolate it under a separate path entry appended after mv_oc.
        wf_oc_root = os.path.join(REPO, 'ecav/worldfusion')
        if wf_oc_root not in sys.path:
            sys.path.append(wf_oc_root)

        # Import our worldfusion model class explicitly by file to avoid
        # shadowing from the already-loaded mv opencood package.
        import importlib.util
        wf_model_file = os.path.join(
            wf_oc_root, 'opencood/models/point_pillar_worldfusion.py')
        spec = importlib.util.spec_from_file_location(
            'ecav_worldfusion_point_pillar', wf_model_file)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)

        self.device = device
        import torch
        self.torch = torch
        self.model = mod.PointPillarWorldFusion(
            wf_params['model']['args']).to(device).eval()

        # Load weights
        ckpt_path = os.path.join(
            wf_model_dir, f'net_epoch{epoch}.pth')
        if not os.path.isfile(ckpt_path):
            # Fall back to highest epoch
            import glob
            cands = sorted(glob.glob(os.path.join(
                wf_model_dir, 'net_epoch*.pth')))
            if not cands:
                raise FileNotFoundError(
                    f"No WorldFusion checkpoint in {wf_model_dir}")
            ckpt_path = cands[-1]
        state = torch.load(ckpt_path, map_location=device)
        self.model.load_state_dict(state, strict=False)
        print(f"[MultiV2XFrameLoader] loaded {ckpt_path}")

    # ── Public API ─────────────────────────────────────────────

    def __len__(self) -> int:
        return len(self.mv_dataset)

    def load_sample_features(self, idx: int
                              ) -> Tuple['torch.Tensor', Dict]:
        """Run WorldFusion sensor on the idx-th Multi-V2X sample.

        Returns:
          features_2d: [N, C, H, W] post-backbone post-shrink features.
          meta: dict with pairwise_t_matrix, record_len, gt objects, etc.
        """
        torch = self.torch
        sample = self.mv_dataset[idx]
        ego = sample['ego']

        # Collate to a single-batch tensor dict.
        batch_list = [sample]
        if hasattr(self.mv_dataset, 'collate_batch_test'):
            batch = self.mv_dataset.collate_batch_test(batch_list)
        else:
            batch = self.mv_dataset.collate_batch_train(batch_list)
        batch_ego = batch['ego']

        # Move tensors to device.
        def _to_dev(x):
            if isinstance(x, dict):
                return {k: _to_dev(v) for k, v in x.items()}
            if hasattr(x, 'to'):
                return x.to(self.device)
            return x
        batch_ego = _to_dev(batch_ego)

        sensor_batch = {
            'processed_lidar': batch_ego['processed_lidar'],
            'record_len': batch_ego['record_len'],
        }
        with torch.no_grad():
            result = self.model.sensor(sensor_batch)
        features_2d = result['spatial_features_2d']  # [N, C, H, W]

        meta = {
            'record_len': batch_ego['record_len'],
            'pairwise_t_matrix': batch_ego.get('pairwise_t_matrix'),
            'object_bbx_center': batch_ego.get('object_bbx_center'),
            'object_bbx_mask': batch_ego.get('object_bbx_mask'),
            'object_ids': ego.get('object_ids'),
            'lidar_pose': batch_ego.get('lidar_pose'),
        }
        return features_2d, meta


# ─── CLI for quick smoke test ─────────────────────────────────────

def _dump_scenes(args):
    idx = MultiV2XSceneIndex(args.inventory)
    print(f"Loaded inventory: {len(idx.scenarios)} scenarios")
    for target_n in [4, 8, 16, 24, 32]:
        names = idx.scenarios_at_n(target_n, tol=args.tol)
        print(f"  target N={target_n:3d} (tol={args.tol}): "
              f"{len(names)} scenarios")
    name = idx.pick_scenario(args.target_n, tol=args.tol)
    if not name:
        print(f"No scenario with N≈{args.target_n}")
        return
    cavs, rsus = idx.agent_names(name)
    ts_list = idx.common_timestamps(name)
    print(f"\nSelected scenario: {name}")
    print(f"  CAVs: {len(cavs)}  RSUs: {len(rsus)}  "
          f"common_frames: {len(ts_list)}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--inventory', required=True,
                    help='JSON from paper2_multiv2x_inspect.py')
    ap.add_argument('--target-n', type=int, default=16)
    ap.add_argument('--tol', type=int, default=4)
    args = ap.parse_args()
    _dump_scenes(args)


if __name__ == '__main__':
    main()
