# -*- coding: utf-8 -*-
# Author: Jordan Rapp <jrapp@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Standalone gRPC server for YOLO distributed object detection.

Loads YOLOv5 once in-process and serves DetectYolo on PerceptionService
at YOLO_GRPC_PORT (default 18001). Replaces the gRPC thread that was
previously co-hosted inside litserve_models.py before YOLO was stripped
during the WorldFusion-only LitServe refactor.

Usage:
    python yolo_grpc_server.py [--port 18001] [--workers 4]
"""

import argparse
import os
import sys

# Suppress YOLOv5's auto-install via system pip — fails on this host
# because the system pip is too old to parse `python_version > 3.8` markers.
os.environ.setdefault('YOLOv5_AUTOINSTALL', 'false')

sys.path.insert(0, os.path.dirname(__file__))  # for perception_servicer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ecav', 'protos'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from concurrent import futures

import torch
import grpc
import perception_pb2_grpc

from perception_servicer import PerceptionServicer

YOLO_PATH = './yolov5'
YOLO_FILE = 'hubconf.py'

_16MB = 16 * 1024 * 1024
_GRPC_OPTIONS = [
    ("grpc.max_send_message_length",    _16MB),
    ("grpc.max_receive_message_length", _16MB),
]


def load_yolo_model(device: str = 'cuda') -> torch.nn.Module:
    if os.path.exists(os.path.join(YOLO_PATH, YOLO_FILE)):
        print(f"[YoloGrpcServer] Loading from local path: {YOLO_PATH}")
        model = torch.hub.load(YOLO_PATH, model='yolov5m', source='local')
    else:
        print("[YoloGrpcServer] Loading from Ultralytics repository")
        model = torch.hub.load("ultralytics/yolov5", "yolov5m")
    model = model.to(device).eval()
    print(f"[YoloGrpcServer] Model loaded on {device}")
    return model


def encode_response(results) -> dict:
    """Convert YOLOv5 output to the dict format expected by PerceptionServicer."""
    resp = {"detections": []}
    for pred in results.xyxy:
        p = pred.cpu() if pred.is_cuda else pred
        boxes = p.numpy().tolist()
        cls_ids = [int(b[5]) for b in boxes] if boxes else []
        counts = {}
        for cid in cls_ids:
            counts[cid] = counts.get(cid, 0) + 1
        summary = ", ".join(
            f"{n} {results.names.get(cid, str(cid))}" + ("" if n == 1 else "s")
            for cid, n in counts.items()
        ) or "(no detections)"
        resp["detections"].append({
            "boxes":   boxes,
            "names":   {str(k): v for k, v in results.names.items()},
            "summary": summary,
        })
    return resp


def serve(port: int, max_workers: int) -> None:
    model = load_yolo_model()

    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=max_workers),
        options=_GRPC_OPTIONS,
    )
    perception_pb2_grpc.add_PerceptionServiceServicer_to_server(
        PerceptionServicer(model, encode_response), server
    )
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    print(f"[YoloGrpcServer] Listening on port {port} (workers={max_workers})", flush=True)
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLO gRPC server")
    parser.add_argument("--port",    type=int, default=int(os.environ.get("YOLO_GRPC_PORT", 18001)))
    parser.add_argument("--workers", type=int, default=int(os.environ.get("YOLO_GRPC_WORKERS", 4)))
    args = parser.parse_args()

    serve(args.port, args.workers)
