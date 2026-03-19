# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
LitServe model servers for distributed inference
"""

import os
import litserve as ls
import torch
import numpy as np
import cv2
import msgpack
import msgpack_numpy as m
m.patch()  # Patch msgpack to handle numpy arrays
from typing import List
from fastapi import Request
from starlette.responses import JSONResponse, Response

YOLO_PATH = './yolov5'  # Path to local YOLOv5 repo
YOLO_FILE = 'yolov5m.pt'  # Local model file name


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

        # With msgpack-numpy, images are already numpy arrays!
        if not isinstance(imgs, list):
            raise ValueError("'images' must be a list.")

        # Verify they're numpy arrays
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


# Global model instance for custom endpoint (loaded once on startup)
_global_model = None

def load_global_model():
    """Load model once for the custom endpoint"""
    global _global_model
    if _global_model is None:
        print("[Global] Loading YOLOv5 model for msgpack endpoint...")
        if os.path.exists(os.path.join(YOLO_PATH, YOLO_FILE)):
            _global_model = torch.hub.load(YOLO_PATH, model='yolov5m', source='local')
        else:
            _global_model = torch.hub.load("ultralytics/yolov5", "yolov5m")
        _global_model = _global_model.cuda().eval()
        print("[Global] Model loaded!")
    return _global_model


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

    return {'spatial_features': bd['spatial_features'].cpu().numpy()}


if __name__ == "__main__":
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "18000"))

    print(f"[LitServe] Starting combined server on {host}:{port}")
    print(f"[LitServe] Endpoints: /predict_msgpack (YOLO), /extract_features (WorldFusion)")

    api = WorldFusionFeatureServer()
    server = ls.LitServer(api, accelerator="auto")

    # YOLO endpoint
    @server.app.post("/predict_msgpack")
    async def predict_msgpack(request: Request):
        """YOLO object detection via msgpack binary data."""
        import asyncio
        import time
        try:
            t0 = time.time()
            model = load_global_model()

            body = await request.body()
            t1 = time.time()

            def _yolo_process(body_bytes):
                data = msgpack.unpackb(body_bytes, raw=False)
                images = data.get("images", [])
                with torch.no_grad():
                    results = model(images)
                return images, results

            images, results = await asyncio.to_thread(_yolo_process, body)
            t2 = time.time()

            response_data = api_yolo.encode_response(results)
            t3 = time.time()
            print(f"[YOLO] {len(images)} imgs, inference={((t2-t1)*1000):.0f}ms, "
                  f"total={((t3-t0)*1000):.0f}ms")

            return JSONResponse(content=response_data)

        except Exception as e:
            import traceback
            print(f"[YOLO] Error: {e}")
            traceback.print_exc()
            return JSONResponse(content={"error": str(e)}, status_code=500)

    # WorldFusion feature extraction endpoint
    @server.app.post("/extract_features")
    async def extract_features(request: Request):
        """WorldFusion intermediate feature extraction via msgpack.

        Uses asyncio.to_thread() so that the synchronous GPU inference
        does not block the uvicorn event loop. This allows concurrent
        body reads and response writes for multiple clients.
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

    # Create a YOLO API instance for encode_response in the msgpack endpoint
    api_yolo = YOLOv5Server()

    server.run(host=host, port=port)
