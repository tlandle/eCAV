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
from fastapi import Request, Response
from starlette.responses import JSONResponse

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


if __name__ == "__main__":
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", "18000"))

    print(f"[YOLOv5Server] Starting server on {host}:{port}")

    api = YOLOv5Server()
    server = ls.LitServer(api, accelerator="auto")

    # Add custom msgpack endpoint
    @server.app.post("/predict_msgpack")
    async def predict_msgpack(request: Request):
        """Custom endpoint that handles msgpack binary data"""
        import time
        try:
            print(f"\n=== SERVER TIMING ===")
            t0 = time.time()

            # Load model (only happens once)
            model = load_global_model()
            t1 = time.time()
            print(f"[Server] Model load check: {(t1-t0)*1000:.1f}ms")

            # Read raw body
            body = await request.body()
            t2 = time.time()
            print(f"[Server] Read body: {(t2-t1)*1000:.1f}ms, size: {len(body)/1024:.1f}KB")

            # Decode msgpack
            data = msgpack.unpackb(body, raw=False)
            t3 = time.time()
            print(f"[Server] msgpack decode: {(t3-t2)*1000:.1f}ms")

            # Get images
            images = data.get("images", [])
            t4 = time.time()
            print(f"[Server] Extract images: {(t4-t3)*1000:.1f}ms, count: {len(images)}")

            # Run YOLO
            with torch.no_grad():
                results = model(images)
            t5 = time.time()
            print(f"[Server] YOLO inference: {(t5-t4)*1000:.1f}ms")

            # Encode response (use api.encode_response for consistency)
            response_data = api.encode_response(results)
            t6 = time.time()
            print(f"[Server] encode_response: {(t6-t5)*1000:.1f}ms")

            # Return as JSON
            json_response = JSONResponse(content=response_data)
            t7 = time.time()
            print(f"[Server] JSONResponse creation: {(t7-t6)*1000:.1f}ms")
            print(f"[Server] TOTAL SERVER TIME: {(t7-t0)*1000:.1f}ms")
            print(f"======================\n")

            return json_response

        except Exception as e:
            import traceback
            print(f"[YOLOv5Server] Error in msgpack endpoint: {e}")
            traceback.print_exc()
            return JSONResponse(
                content={"error": str(e)},
                status_code=500
            )

    server.run(host=host, port=port)
