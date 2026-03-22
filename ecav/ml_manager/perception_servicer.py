# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
gRPC servicer for YOLO distributed perception inference.
Runs in a ThreadPoolExecutor thread alongside the LitServe uvicorn process,
sharing the global YOLO model loaded by load_global_model().
"""

import time

import cv2
import numpy as np
import torch

import perception_pb2
import perception_pb2_grpc


class PerceptionServicer(perception_pb2_grpc.PerceptionServiceServicer):
    """Implements the PerceptionService gRPC service."""

    def __init__(self, model, encode_response_fn):
        """
        Parameters
        ----------
        model : torch.nn.Module
            Loaded YOLOv5 model (cuda, eval mode).
        encode_response_fn : callable
            YOLOv5Server.encode_response — converts model output to a
            dict with 'detections' list. Reused to avoid duplicating
            box extraction logic.
        """
        self._model = model
        self._encode_response = encode_response_fn

    def DetectYolo(self, request, context):
        t0 = time.time()

        # Decode JPEG frames → BGR numpy arrays (YOLO expects BGR)
        images = [
            cv2.imdecode(np.frombuffer(b, dtype=np.uint8), cv2.IMREAD_COLOR)
            for b in request.jpeg_frames
        ]
        t_decoded = time.time()

        with torch.no_grad():
            results = self._model(images)
        t_inferred = time.time()

        # Reuse YOLOv5Server.encode_response to build the detections dict
        result_dict = self._encode_response(results)
        t_encoded = time.time()

        decode_ms = (t_decoded - t0) * 1000
        inference_ms = (t_inferred - t_decoded) * 1000
        encode_ms = (t_encoded - t_inferred) * 1000
        total_ms = (t_encoded - t0) * 1000

        print(
            f"[PerceptionServicer] actor={request.actor_id} "
            f"frames={len(images)} "
            f"decode={decode_ms:.0f}ms "
            f"inference={inference_ms:.0f}ms "
            f"encode={encode_ms:.0f}ms "
            f"total={total_ms:.0f}ms"
        )

        response = perception_pb2.YoloResponse(
            decode_ms=decode_ms,
            img_prep_ms=decode_ms,   # img_prep == JPEG decode for this path
            inference_ms=inference_ms,
            encode_ms=encode_ms,
            total_ms=total_ms,
        )

        for det in result_dict["detections"]:
            img_dets = perception_pb2.ImageDetections(
                summary=det["summary"],
            )
            # class_names: proto map<int32, string> — keys must be int
            for k, v in det["names"].items():
                img_dets.class_names[int(k)] = v
            for box in det["boxes"]:
                img_dets.detections.append(
                    perception_pb2.Detection(box=[float(x) for x in box])
                )
            response.per_image.append(img_dets)

        return response
