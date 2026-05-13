# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
LitServe WorldFusion feature extraction server.

Serves the WorldFusion sensor encoder (LiDAR voxelizer + camera encoder →
BEV spatial_features) via HTTP POST /extract_features.

YOLO (late-fusion path) is not served here — it is unused in WorldFusion
scenarios and was the sole reason this server previously required
api_server_worker_type="thread". With YOLO removed, the default "process"
mode is used, enabling workers_per_device=N with independent CUDA contexts.

Workers:
    workers_per_device = min(WF_MAX_WORKERS, WF_NUM_ACTORS)
    WF_NUM_ACTORS  — actors calling /extract_features per tick (default 2)
    WF_MAX_WORKERS — GPU headroom limit; run --profile to compute for your GPU

Profile mode:
    python litserve_models.py --profile
"""

import os
import sys

# perception_pb2 stubs live in ecav/protos/; add to path when running standalone
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ecav', 'protos'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import time
import zlib

import litserve as ls
import msgpack
import msgpack_numpy as m
import numpy as np
import torch
from starlette.responses import Response

m.patch()  # Patch msgpack to handle numpy arrays

# ============= WorldFusion model (global per-worker) =============

# Each worker process has its own copy of these globals.
# setup() populates them after spawn; __main__ must NOT touch them.
_wf_model = None
_wf_device = None


def load_wf_model():
    """Load WorldFusion model into this process. Idempotent."""
    global _wf_model, _wf_device
    if _wf_model is not None:
        return _wf_model

    import pathlib
    from opencood.hypes_yaml.yaml_utils import load_yaml
    from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
    import opencood.tools.train_utils as train_utils

    _wf_device = 'cuda'

    hypes_path = os.environ.get(
        'WF_HYPES_YAML',
        'ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10/config.yaml'
    )
    ckpt_path = os.environ.get(
        'WF_CHECKPOINT',
        'ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10/net_epoch50.pth'
    )

    print(f"[WorldFusionServer] Loading hypes: {hypes_path}")
    hypes = load_yaml(str(hypes_path))
    _wf_model = PointPillarWorldFusion(hypes['model']['args']).to(_wf_device).eval()

    ckpt = pathlib.Path(ckpt_path)
    epoch_id = int(str(ckpt.name).split('epoch')[-1].split('.')[0])
    _, _wf_model = train_utils.load_model(str(ckpt.parent), _wf_model, epoch=epoch_id)
    print(f"[WorldFusionServer] Model loaded from epoch {epoch_id} on {_wf_device}")
    return _wf_model


def _merge_wf_batches(batch_list):
    """Merge list of per-actor msgpack dicts into a single batch dict.

    voxel_coords[:,0] is the batch/agent index used by PointPillarScatter.
    Each actor arrives with coords[:,0]=0; must be reindexed to agent_idx
    so scatter canvas slots are populated correctly (slot i → agent i's BEV).

    record_len = [1] * N matches the O5 edge manager convention.
    """
    if len(batch_list) == 1:
        return batch_list[0]

    pc_list = [b['processed_lidar'] for b in batch_list]

    coords_reindexed = []
    for i, pc in enumerate(pc_list):
        coords = pc['voxel_coords'].copy()
        coords[:, 0] = i
        coords_reindexed.append(coords)

    merged = {
        'processed_lidar': {
            'voxel_features':   np.concatenate([pc['voxel_features']   for pc in pc_list]),
            'voxel_coords':     np.concatenate(coords_reindexed),
            'voxel_num_points': np.concatenate([pc['voxel_num_points'] for pc in pc_list]),
        },
        'record_len': np.concatenate([b['record_len'] for b in batch_list]),
    }
    if batch_list[0].get('image_inputs') is not None:
        merged['image_inputs'] = {
            k: np.concatenate([b['image_inputs'][k] for b in batch_list], axis=0)
            for k in batch_list[0]['image_inputs']
        }
    return merged


class WorldFusionFeatureServer(ls.LitAPI):
    """LitServe native pipeline for WorldFusion feature extraction.

    decode_request → batch → predict → unbatch → encode_response

    Requests are automatically batched by LitServe (max_batch_size, batch_timeout)
    so vehicle + RSU are merged into a single batch=2 forward pass, eliminating
    the CUDA serialization seen with the previous custom-endpoint approach.
    """

    def setup(self, device="cuda"):
        load_wf_model()

    def decode_request(self, request):
        # request is raw bytes from _prepare_request (patched below to handle
        # application/octet-stream without calling request.json())
        try:
            body = zlib.decompress(request)
        except zlib.error:
            body = request
        return msgpack.unpackb(body, raw=False)

    def batch(self, inputs):
        return _merge_wf_batches(inputs)

    def predict(self, x):
        t0 = time.time()
        result = _extract_wf_features(x)
        batch_size = result['spatial_features'].shape[0]
        print(f"[WorldFusion pid={os.getpid()}] batch={batch_size} "
              f"inference={int((time.time()-t0)*1000)}ms "
              f"payload={result['spatial_features'].nbytes//1024}KB")
        return result

    def unbatch(self, output):
        N = output['spatial_features'].shape[0]
        return [{'spatial_features': output['spatial_features'][i:i+1]} for i in range(N)]

    def encode_response(self, output):
        return Response(content=msgpack.packb(output, use_bin_type=True),
                        media_type="application/octet-stream")


@torch.inference_mode()
def _extract_wf_features(batch):
    """Extract intermediate features from WorldFusion sensor encoder.

    Accepts numpy arrays from msgpack (single or merged batch), returns numpy.
    """
    import opencood.tools.train_utils as train_utils

    def _numpy_to_tensors(obj):
        if isinstance(obj, np.ndarray):
            return torch.from_numpy(obj.copy())
        if isinstance(obj, dict):
            return {k: _numpy_to_tensors(v) for k, v in obj.items()}
        return obj

    batch = _numpy_to_tensors(batch)
    model = load_wf_model()
    batch = train_utils.to_device(batch, _wf_device)
    sensor = model.sensor

    pc = batch['processed_lidar']
    rec_len = batch['record_len']
    bd = {
        'voxel_features': pc['voxel_features'],
        'voxel_coords': pc['voxel_coords'],
        'voxel_num_points': pc['voxel_num_points'],
        'record_len': rec_len,
    }
    bd = sensor.scatter(sensor.pillar_vfe(bd))

    if 'image_inputs' in batch and batch['image_inputs'] is not None and sensor.camenc is not None:
        from einops import rearrange
        # Cast back to float32 if client sent as uint8 to reduce payload
        imgs = batch['image_inputs']['imgs'].float()
        B, N, C, imH, imW = imgs.shape
        x = imgs.view(B * N, C, imH, imW)
        _, x = sensor.camenc(x, batch['image_inputs']['depth_map'], batch['record_len'])
        x = rearrange(x, '(b n) c d h w -> b n c d h w', b=B, n=N)
        x = x.permute(0, 1, 3, 4, 5, 2)
        geom = sensor._get_geometry(batch['image_inputs'])
        img_voxel = sensor.voxel_pooling(geom, x)
        bd, thres_map, mask, each_mask = sensor.vox_fuse(img_voxel, bd)
    else:
        spatial_3d = bd['spatial_features_3d']
        b, c, z, y, x_dim = spatial_3d.shape
        bd['spatial_features'] = spatial_3d.view(b, c * z, y, x_dim)

    # Return as float16 to halve response payload (~10MB → ~5MB).
    # The edge manager casts back to float32 via .float().cuda() before fusion.
    return {'spatial_features': bd['spatial_features'].cpu().half().numpy()}


# ============= Profile mode =============

def _run_profile_worker():
    """Run inside a subprocess: load model, measure VRAM, print recommendation.

    Executed as a child process so CUDA init stays isolated from the parent.
    """
    if not torch.cuda.is_available():
        print("[Profile] No CUDA device found.")
        return

    gpu = torch.cuda.get_device_properties(0)
    total_mb = gpu.total_memory / (1024 ** 2)

    torch.cuda.empty_cache()
    free_before, _ = torch.cuda.mem_get_info(0)
    load_wf_model()
    free_after, _ = torch.cuda.mem_get_info(0)

    model_mb = (free_before - free_after) / (1024 ** 2)
    # Inference overhead ≈ 2.5× model size (activations, workspace for batch=1)
    per_worker_mb = model_mb * 2.5
    usable_mb = total_mb * 0.80
    carla_budget_mb = 2500
    usable_with_carla_mb = (total_mb - carla_budget_mb) * 0.85

    gpu_max             = max(1, int(usable_mb / per_worker_mb))
    gpu_max_with_carla  = max(1, int(usable_with_carla_mb / per_worker_mb))

    print(f"[Profile] GPU: {gpu.name} ({total_mb:.0f} MB total)")
    print(f"[Profile] WorldFusion model footprint: {model_mb:.0f} MB")
    print(f"[Profile] Per-worker estimate (model + inference overhead): {per_worker_mb:.0f} MB")
    print(f"[Profile] Recommended WF_MAX_WORKERS: {gpu_max} (without CARLA) / "
          f"{gpu_max_with_carla} (with CARLA)")
    print(f"[Profile] Export: export WF_MAX_WORKERS={gpu_max_with_carla}")


def _run_profile():
    """Spawn an isolated subprocess to measure VRAM footprint."""
    import subprocess
    print("[Profile] Measuring WorldFusion VRAM footprint in isolated subprocess...")
    result = subprocess.run(
        [sys.executable, __file__, '--profile-worker'],
        capture_output=False,   # stream output directly
    )
    if result.returncode != 0:
        print(f"[Profile] Subprocess exited with code {result.returncode}")


# ============= Entry point =============

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="WorldFusion LitServe server")
    parser.add_argument('--profile', action='store_true',
                        help="Measure GPU VRAM footprint and print WF_MAX_WORKERS recommendation")
    parser.add_argument('--profile-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.profile_worker:
        _run_profile_worker()
        sys.exit(0)

    if args.profile:
        _run_profile()
        sys.exit(0)

    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "18000"))

    import math
    MAX_BATCH_SIZE = 2

    num_actors  = int(os.environ.get('WF_NUM_ACTORS', '2'))
    max_workers = int(os.environ.get('WF_MAX_WORKERS', str(num_actors)))
    # One worker per MAX_BATCH_SIZE actors: requests batch together rather than racing
    # to separate workers. e.g. 2 actors / batch=2 → 1 worker; 4 actors → 2 workers.
    workers = max(1, min(max_workers, math.ceil(num_actors / MAX_BATCH_SIZE)))

    print(f"[LitServe] Starting WorldFusion server — HTTP on {host}:{port}")
    print(f"[LitServe] workers_per_device={workers} "
          f"(WF_NUM_ACTORS={num_actors}, WF_MAX_WORKERS={max_workers})")
    print(f"[LitServe] max_batch_size={MAX_BATCH_SIZE} batch_timeout=15ms endpoint=/extract_features")

    api = WorldFusionFeatureServer(
        max_batch_size=MAX_BATCH_SIZE,
        batch_timeout=0.015,
        api_path='/extract_features',
    )
    server = ls.LitServer(api, accelerator="auto", workers_per_device=workers)

    # LitServe's _prepare_request calls request.json() for any non-form POST body,
    # which fails for our application/octet-stream msgpack payload. Patch the class
    # method to return raw bytes for binary content before server.run() registers endpoints.
    from litserve.server import BaseRequestHandler
    from starlette.requests import Request as _StarlRequest

    _orig_prepare = BaseRequestHandler._prepare_request

    async def _binary_prepare(self, request, request_type):
        if request_type == _StarlRequest:
            if request.headers.get("Content-Type", "") == "application/octet-stream":
                return await request.body()
        return await _orig_prepare(self, request, request_type)

    BaseRequestHandler._prepare_request = _binary_prepare

    server.run(host=host, port=port)
