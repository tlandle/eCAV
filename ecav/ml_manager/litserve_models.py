# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
LitServe model servers for distributed inference
"""

import os
import sys
from concurrent import futures

# Suppress YOLOv5's auto-install via system pip, which fails on this host
# because the system pip is too old to parse `python_version > 3.8` markers.
os.environ.setdefault('YOLOv5_AUTOINSTALL', 'false')

# perception_pb2 stubs live in ecav/protos/; add to path when running standalone
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ecav', 'protos'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

import grpc
import litserve as ls
import msgpack
import msgpack_numpy as m
import numpy as np
import torch
from fastapi import Request
from starlette.responses import JSONResponse, Response
from typing import List

import perception_pb2_grpc
from perception_servicer import PerceptionServicer

m.patch()  # Patch msgpack to handle numpy arrays

YOLO_PATH = './yolov5'  # Path to local YOLOv5 repo
YOLO_FILE = 'hubconf.py'  # Detect local yolov5 repo (weights may be elsewhere)


class YOLOv5Server(ls.LitAPI):
    def setup(self, device="cuda"):
        """Load YOLOv5 model"""
        self.device = device

        # Check if local YOLOv5 repo exists
        if os.path.exists(os.path.join(YOLO_PATH, YOLO_FILE)):
            print(f"[YOLOv5Server] Loading from local path: {YOLO_PATH}")
            self.model = torch.hub.load(YOLO_PATH, model='yolov5m', source='local')
        else:
            print("[YOLOv5Server] Loading from Ultralytics repository")
            self.model = torch.hub.load("ultralytics/yolov5", "yolov5m")

        self.model = self.model.to(device).eval()
        print(f"[YOLOv5Server] Model loaded on {device}")

    def decode_request(self, request: dict) -> List[np.ndarray]:
        """Decode images - msgpack sends numpy arrays directly"""
        imgs = request.get("images")
        if imgs is None:
            raise ValueError("Request must contain key 'images'.")

        if not isinstance(imgs, list):
            raise ValueError("'images' must be a list.")

        for i, img in enumerate(imgs):
            if not isinstance(img, np.ndarray):
                raise ValueError(f"Image {i} is {type(img)}, expected numpy.ndarray")

        print(f"[YOLOv5Server] Received {len(imgs)} numpy arrays directly")
        return imgs

    def predict(self, images: List[np.ndarray]):
        """Run YOLO inference"""
        with torch.no_grad():
            results = self.model(images)
            return results

    def encode_response(self, output):
        """Encode response as dict"""
        resp = {"detections": []}

        for idx, pred in enumerate(output.xyxy):
            if hasattr(pred, 'is_cuda') and pred.is_cuda:
                p = pred.cpu()
            else:
                p = pred

            boxes = p.numpy().tolist()
            cls_ids = [int(b[5]) for b in boxes] if boxes else []
            counts = {}
            for cid in cls_ids:
                counts[cid] = counts.get(cid, 0) + 1

            summary = ", ".join(
                f"{n} {output.names.get(cid, str(cid))}" + ("" if n == 1 else "s")
                for cid, n in counts.items()
            ) or "(no detections)"

            names_str = {str(k): v for k, v in output.names.items()}

            resp["detections"].append({
                "boxes": boxes,
                "names": names_str,
                "summary": summary
            })

        return resp


# ============= WorldFusion Feature Server =============

# Global WorldFusion model instance (loaded lazily on first request)
_wf_model = None
_wf_device = None


def load_wf_model():
    """Load WorldFusion model once for the custom endpoint (main process)."""
    global _wf_model, _wf_device
    if _wf_model is None:
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
        _, _wf_model = train_utils.load_model(
            str(ckpt.parent), _wf_model, epoch=epoch_id
        )
        print(f"[WorldFusionServer] Model loaded from epoch {epoch_id} on {_wf_device}")
    return _wf_model


class WorldFusionFeatureServer(ls.LitAPI):
    """LitServe server stub — real work done via custom /extract_features endpoint."""

    def setup(self, device="cuda"):
        pass

    def decode_request(self, request):
        return request

    def predict(self, x):
        return x

    def encode_response(self, output):
        return output


@torch.inference_mode()
def _extract_wf_features(batch):
    """Extract intermediate features from WorldFusion sensor encoder.

    Replicates WorldFusionPerceptionManager._extract_intermediate_features()
    for server-side execution. Accepts numpy arrays from msgpack, returns numpy.
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

    if 'image_inputs' in batch and batch['image_inputs'] is not None:
        from einops import rearrange
        imgs = batch['image_inputs']['imgs']
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


if __name__ == "__main__":
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "18000"))
    grpc_port = int(os.getenv("GRPC_PORT", "18001"))

    print(f"[LitServe] Starting server — HTTP on {host}:{port}, gRPC on {host}:{grpc_port}")
    print(f"[LitServe] HTTP endpoint: /extract_features (WorldFusion)")
    print(f"[LitServe] gRPC endpoint: PerceptionService.DetectYolo")

    # Load YOLO model once; shared between gRPC servicer and this process
    yolo_api = YOLOv5Server()
    if os.path.exists(os.path.join(YOLO_PATH, YOLO_FILE)):
        _yolo_model = torch.hub.load(YOLO_PATH, model='yolov5m', source='local')
    else:
        _yolo_model = torch.hub.load("ultralytics/yolov5", "yolov5m")
    _yolo_model = _yolo_model.cuda().eval()
    print("[LitServe] YOLO model loaded")

    # Pre-load WorldFusion model before LitServe spawns worker processes.
    # Lazy loading inside a spawned worker re-initializes CUDA cleanly, but
    # pre-loading here ensures the first request pays no cold-start penalty.
    load_wf_model()
    print("[LitServe] WorldFusion model pre-loaded")

    # Start gRPC server in background thread
    servicer = PerceptionServicer(_yolo_model, yolo_api.encode_response)
    grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    perception_pb2_grpc.add_PerceptionServiceServicer_to_server(servicer, grpc_server)
    grpc_server.add_insecure_port(f"[::]:{grpc_port}")
    grpc_server.start()
    print(f"[gRPC] PerceptionService listening on port {grpc_port}")

    # WorldFusion HTTP server (uvicorn/LitServe)
    api = WorldFusionFeatureServer()
    server = ls.LitServer(api, accelerator="auto")

    @server.app.post("/extract_features")
    async def extract_features(request: Request):
        """WorldFusion intermediate feature extraction via msgpack.

        Uses asyncio.to_thread() so that the synchronous GPU inference
        does not block the uvicorn event loop.
        """
        import asyncio
        import time

        try:
            t0 = time.time()
            body = await request.body()
            t_read = time.time()

            def _process(body_bytes):
                t1 = time.time()
                batch = msgpack.unpackb(body_bytes, raw=False)
                t2 = time.time()
                result = _extract_wf_features(batch)
                t3 = time.time()
                payload = msgpack.packb(result, use_bin_type=True)
                t4 = time.time()
                return payload, (t2 - t1), (t3 - t2), (t4 - t3)

            payload, decode_s, infer_s, encode_s = await asyncio.to_thread(
                _process, body
            )
            t_done = time.time()

            read_ms = int((t_read - t0) * 1000)
            decode_ms = int(decode_s * 1000)
            infer_ms = int(infer_s * 1000)
            encode_ms = int(encode_s * 1000)
            total_ms = int((t_done - t0) * 1000)

            print(f"[WorldFusion] read={read_ms}ms "
                  f"decode={decode_ms}ms "
                  f"inference={infer_ms}ms "
                  f"encode={encode_ms}ms "
                  f"total={total_ms}ms "
                  f"payload={len(payload)/1024:.0f}KB")

            headers = {
                'X-Server-Read-Ms': str(read_ms),
                'X-Server-Decode-Ms': str(decode_ms),
                'X-Server-Inference-Ms': str(infer_ms),
                'X-Server-Encode-Ms': str(encode_ms),
                'X-Server-Total-Ms': str(total_ms),
            }

            return Response(
                content=payload,
                media_type="application/octet-stream",
                headers=headers,
            )

        except Exception as e:
            import traceback
            print(f"[WorldFusion] Error: {e}")
            traceback.print_exc()
            return JSONResponse(content={"error": str(e)}, status_code=500)

    # Use threads for HTTP server workers so they share the parent's CUDA context.
    # LitServe's default "process" mode forks uvicorn after CUDA is initialized
    # by YOLO loading, which causes "Cannot re-initialize CUDA in forked subprocess".
    server.run(host=host, port=port, api_server_worker_type="thread")
