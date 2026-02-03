# -*- coding: utf-8 -*-
"""
ML Manager supporting both local and distributed modes for model inference
"""

# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os
import time
import torch
import torch.multiprocessing as mp
import cv2
import numpy as np
import requests
import json
from typing import Dict, Any, List, Optional
import msgpack
import msgpack_numpy as m

m.patch()

# Local model imports
from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.models.point_pillar_bm2cp import PointPillarBM2CP
import opencood.tools.train_utils as train_utils
from opencood.data_utils.post_processor import VoxelPostprocessor

YOLO_PATH = "yolov5/"
YOLO_FILE = "hubconf.py"

class MLManager(object):
    """
    ML Manager that can operate in both local and distributed modes.

    Attributes
    ----------
    object_detector : torch model or client
        The YoloV5 detector (local model or distributed client)
    bm2cp_model : torch model or client
        The BM2CP model (local model or distributed client)
    run_distributed : bool
        Whether to use distributed services
    """

    def __init__(self, apply_ml: bool, rank: int = 0,
                 run_distributed: bool = False, config: Dict[str, Any] = None):
        """
        Initialize ML Manager in either local or distributed mode

        Parameters
        ----------
        apply_ml : bool
            Whether to load ML models
        rank : int
            Process rank (for compatibility)
        run_distributed : bool
            Whether to use distributed services via LitServe
        config : dict
            Configuration dictionary with model paths and service endpoints
        """
        self.apply_ml = apply_ml
        self.run_distributed = run_distributed
        self.config = config or {}
        self.rank = rank

        if not apply_ml:
            return

        if run_distributed:
            self._init_distributed()
        else:
            self._init_local()

    def _init_local(self):
        """Initialize local models"""
        print("[ML Manager] Initializing in LOCAL mode...")

        # Load YOLOv5 locally
        if self.config.get('load_yolo', True):
            print("[ML Manager] Loading YOLOv5 locally...")
            torch.cuda.empty_cache()
            torch.cuda.memory_summary(device=None, abbreviated=False)

            if os.path.exists(os.path.join(YOLO_PATH, YOLO_FILE)) and self.rank == 0:
                self.object_detector = torch.hub.load(YOLO_PATH, model='yolov5m', source='local')
            else:
                self.object_detector = torch.hub.load('ultralytics/yolov5', 'yolov5m')

            self.object_detector.eval()

        # Load BM2CP locally if configured
        if 'bm2cp_model' in self.config:
            print("[ML Manager] Loading BM2CP locally...")
            self._load_local_bm2cp(self.config['bm2cp_model'])

    def _init_distributed(self):
        """Initialize distributed service endpoints"""
        print("[ML Manager] Initializing in DISTRIBUTED mode...")

        # Service endpoints
        self.yolo_endpoint = self.config.get('yolo_endpoint', 'http://localhost:18000')

        print(f"[ML Manager] YOLO endpoint: {self.yolo_endpoint}")

        # Set object_detector to None to indicate distributed mode
        self.object_detector = None

    def _load_local_bm2cp(self, bm2cp_config):
        """Load BM2CP model locally"""
        hypes_path = bm2cp_config['hypes_yaml']
        checkpoint_path = bm2cp_config['checkpoint']

        self.bm2cp_hypes = load_yaml(hypes_path)

        # Create model
        self.bm2cp_model = PointPillarBM2CP(self.bm2cp_hypes['model']['args'])

        # Load checkpoint
        ckpt_dir = os.path.dirname(checkpoint_path)
        epoch_id = int(checkpoint_path.split('epoch')[-1].split('.')[0])
        _, self.bm2cp_model = train_utils.load_model(ckpt_dir, self.bm2cp_model, epoch_id)

        self.bm2cp_model = self.bm2cp_model.cuda().eval()

        # Create post-processor
        self.bm2cp_post_processor = VoxelPostprocessor(
            self.bm2cp_hypes['postprocess'],
            dataset=None,
            train=False
        )

        print(f"[ML Manager] BM2CP loaded locally from epoch {epoch_id}")

    # ============= Main Detection Method (backward compatible) =============

    def detect(self, rgb_images):
        """
        Main detection method - maintains backward compatibility

        Parameters
        ----------
        rgb_images : list
            List of RGB images

        Returns
        -------
        Detection results (YOLO format)
        """
        if not self.apply_ml:
            return None

        if self.run_distributed:
            return self._detect_yolo_distributed(rgb_images)
        else:
            return self.object_detector(rgb_images)

    # ============= YOLO Methods =============

    def detect_yolo(self, rgb_images: List[np.ndarray]):
        """
        Run YOLO detection (local or distributed)
        Alias for detect() method for clarity
        """
        return self.detect(rgb_images)

    def _detect_yolo_distributed(self, rgb_images):
        """Run YOLO via distributed service using msgpack"""
        t0 = time.time()

        # Pack numpy arrays with msgpack
        request_data = {'images': rgb_images}
        packed = msgpack.packb(request_data, use_bin_type=True)

        t1 = time.time()
        print(f"[ML Manager] Packing time: {(t1-t0)*1000:.1f}ms, size: {len(packed)/1024:.1f}KB")

        try:
            # Use the /predict_msgpack endpoint
            response = requests.post(
                f"{self.yolo_endpoint}/predict_msgpack",
                data=packed,
                headers={'Content-Type': 'application/msgpack'},
                timeout=30
            )

            if response.status_code != 200:
                raise RuntimeError(f"YOLO service error: {response.text}")

            t2 = time.time()
            print(f"[ML Manager] Network time: {(t2-t1)*1000:.1f}ms")

            # Response is JSON
            result = response.json()

            t3 = time.time()
            print(f"[ML Manager] Decode time: {(t3-t2)*1000:.1f}ms")
            print(f"[ML Manager] TOTAL: {(t3-t0)*1000:.1f}ms")

            return self._parse_yolo_response(result)

        except Exception as e:
            print(f"[ML Manager] YOLO distributed error: {e}")
            import traceback
            traceback.print_exc()
            return None

    # ============= BM2CP Methods =============

    def encode_bm2cp_vehicle(self, batch: Dict) -> Optional[Dict]:
        """
        Run BM2CP vehicle encoding (local or distributed)

        Parameters
        ----------
        batch : dict
            Batch containing processed lidar and image inputs

        Returns
        -------
        Feature dictionary
        """
        if not self.apply_ml:
            return None

        if self.run_distributed:
            return self._encode_bm2cp_vehicle_distributed(batch)
        else:
            return self._encode_bm2cp_vehicle_local(batch)

    @torch.no_grad()
    def _encode_bm2cp_vehicle_local(self, batch: Dict) -> Dict:
        """Run BM2CP vehicle encoding locally"""
        # Move to GPU
        batch_gpu = train_utils.to_device(batch, 'cuda')

        # Get features
        feature_dict = self.bm2cp_model.get_feature(batch_gpu)

        # Convert to CPU for compatibility
        return {k: v.cpu() if torch.is_tensor(v) else v
                for k, v in feature_dict.items()}

    def _encode_bm2cp_vehicle_distributed(self, batch: Dict) -> Dict:
        """Run BM2CP vehicle encoding via distributed service"""
        # Serialize batch for transmission
        request_data = {
            'processed_lidar': self._serialize_dict(batch['processed_lidar']),
            'image_inputs': self._serialize_dict(batch['image_inputs']),
            'record_len': batch['record_len'].item() if torch.is_tensor(batch['record_len']) else batch['record_len']
        }

        try:
            response = requests.post(
                f"{self.bm2cp_vehicle_endpoint}/predict",
                json=request_data,
                timeout=30
            )

            if response.status_code != 200:
                raise RuntimeError(f"BM2CP vehicle encoder error: {response.text}")

            # Deserialize response
            return self._deserialize_feature_dict(response.json())

        except Exception as e:
            print(f"[ML Manager] BM2CP vehicle distributed error: {e}")
            return None

    def fuse_bm2cp_edge(self,
                       feature_dicts: List[Dict],
                       pairwise_t_matrix: torch.Tensor,
                       record_len: torch.Tensor) -> Optional[Dict]:
        """
        Run BM2CP edge fusion (local or distributed)

        Parameters
        ----------
        feature_dicts : list
            List of feature dictionaries from vehicles
        pairwise_t_matrix : torch.Tensor
            Transformation matrices
        record_len : torch.Tensor
            Number of vehicles

        Returns
        -------
        Fusion results with detections
        """
        if not self.apply_ml:
            return None

        if self.run_distributed:
            return self._fuse_bm2cp_edge_distributed(feature_dicts, pairwise_t_matrix, record_len)
        else:
            return self._fuse_bm2cp_edge_local(feature_dicts, pairwise_t_matrix, record_len)

    @torch.no_grad()
    def _fuse_bm2cp_edge_local(self,
                               feature_dicts: List[Dict],
                               pairwise_t_matrix: torch.Tensor,
                               record_len: torch.Tensor) -> Dict:
        """Run BM2CP edge fusion locally"""
        # Stack features
        features_tensor = torch.cat([d['spatial_features'] for d in feature_dicts], dim=0).cuda()
        psm_tensor = torch.cat([d['psm'] for d in feature_dicts], dim=0).cuda()
        rm_tensor = torch.cat([d['rm'] for d in feature_dicts], dim=0).cuda()
        thres_map_tensor = torch.cat([d['thres_map'] for d in feature_dicts], dim=0).cuda()

        # Run fusion
        fused_feature, communication_rates, result_dict = self.bm2cp_model.fusion_net(
            features_tensor,
            psm_tensor,
            thres_map_tensor,
            record_len.cuda(),
            pairwise_t_matrix.cuda(),
            backbone=self.bm2cp_model.backbone,
            heads=[self.bm2cp_model.shrink_conv, self.bm2cp_model.cls_head, self.bm2cp_model.reg_head]
        )

        # Generate predictions
        if self.bm2cp_model.shrink_flag:
            fused_feature = self.bm2cp_model.shrink_conv(fused_feature)

        pred_dict = {
            'psm': self.bm2cp_model.cls_head(fused_feature),
            'rm': self.bm2cp_model.reg_head(fused_feature)
        }

        # Post-process
        anchor_box = self.bm2cp_post_processor.generate_anchor_box()
        data_dict = {
            'ego': {
                'anchor_box': torch.from_numpy(anchor_box).cuda(),
                'transformation_matrix': torch.eye(4).cuda()
            }
        }
        output_dict = {'ego': pred_dict}

        boxes, scores = self.bm2cp_post_processor.post_process(data_dict, output_dict)

        return {
            'boxes': boxes,
            'scores': scores,
            'communication_rates': communication_rates,
            'pred_dict': pred_dict
        }

    def _fuse_bm2cp_edge_distributed(self,
                                    feature_dicts: List[Dict],
                                    pairwise_t_matrix: torch.Tensor,
                                    record_len: torch.Tensor) -> Dict:
        """Run BM2CP edge fusion via distributed service"""
        # Serialize feature dictionaries
        serialized_features = []
        for feat_dict in feature_dicts:
            serialized_features.append(self._serialize_dict(feat_dict))

        request_data = {
            'feature_dicts': serialized_features,
            'pairwise_t_matrix': pairwise_t_matrix.cpu().numpy().tolist(),
            'record_len': record_len.cpu().numpy().tolist()
        }

        try:
            response = requests.post(
                f"{self.bm2cp_edge_endpoint}/predict",
                json=request_data,
                timeout=30
            )

            if response.status_code != 200:
                raise RuntimeError(f"BM2CP edge fusion error: {response.text}")

            result = response.json()

            # Convert back to tensors
            if result['boxes']:
                result['boxes'] = torch.tensor(result['boxes'])
                result['scores'] = torch.tensor(result['scores'])
            else:
                result['boxes'] = None
                result['scores'] = None

            result['communication_rates'] = torch.tensor(result['communication_rates'])

            return result

        except Exception as e:
            print(f"[ML Manager] BM2CP edge distributed error: {e}")
            return None

    # ============= Existing draw_2d_box method (unchanged) =============

    def draw_2d_box(self, result, rgb_image, index):
        """
        Draw 2d bounding box based on the yolo detection.

        Args:
            -result (yolo.Result): Detection result from yolo 5.
            -rgb_image (np.ndarray): Camera rgb image.
            -index(int): Indicate the index.

        Returns:
            -rgb_image (np.ndarray): camera image with bbx drawn.
        """
        # Handle both local and distributed result formats
        if hasattr(result, 'xyxy'):
            # Local YOLO format
            bounding_box = result.xyxy[index]
            names = result.names
        elif isinstance(result, dict) and 'detections' in result:
            # Distributed format
            bounding_box = torch.tensor(result['detections'][index]['boxes'])
            names = result['detections'][index]['names']
        else:
            # Assume it's already in the right format
            bounding_box = result.xyxy[index] if hasattr(result, 'xyxy') else result
            names = result.names if hasattr(result, 'names') else {}

        if bounding_box.is_cuda:
            bounding_box = bounding_box.cpu().detach().numpy()
        else:
            bounding_box = bounding_box.detach().numpy()

        for i in range(bounding_box.shape[0]):
            detection = bounding_box[i]

            # the label has 80 classes, which is the same as coco dataset
            label = int(detection[5])

            # FIX: Use .get() to avoid KeyError
            label_name = names.get(label, str(label)) if names else str(label)

            if is_vehicle_cococlass(label):
                label_name = 'vehicle'

            x1, y1, x2, y2 = int(detection[0]), int(detection[1]), \
                             int(detection[2]), int(detection[3])
            cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # draw text on it
            cv2.putText(rgb_image, label_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 1)

        return rgb_image

    # ============= Helper Methods =============

    def _serialize_dict(self, data: Dict) -> Dict:
        """Serialize tensor dictionary for transmission"""
        serialized = {}
        for key, value in data.items():
            if isinstance(value, torch.Tensor):
                serialized[key] = {
                    'data': value.cpu().numpy().tolist(),
                    'shape': list(value.shape),
                    'dtype': str(value.dtype)
                }
            elif isinstance(value, np.ndarray):
                serialized[key] = {
                    'data': value.tolist(),
                    'shape': list(value.shape),
                    'dtype': str(value.dtype)
                }
            elif isinstance(value, dict):
                serialized[key] = self._serialize_dict(value)
            else:
                serialized[key] = value
        return serialized

    def _deserialize_feature_dict(self, data: Dict) -> Dict:
        """Deserialize received feature dictionary"""
        deserialized = {}
        for key, value in data.items():
            if isinstance(value, dict) and 'data' in value:
                # Reconstruct tensor
                tensor = torch.tensor(value['data'])
                tensor = tensor.reshape(value['shape'])
                deserialized[key] = tensor
            else:
                deserialized[key] = value
        return deserialized


    def _parse_yolo_response(self, response: Dict):
        """Parse distributed YOLO response to a Results-like proxy."""
        detections = response.get('detections', [])

        # Convert string keys back to integers
        names_raw = detections[0]['names'] if detections else {}
        names = {int(k): v for k, v in names_raw.items()}  # String keys → int keys
        class YOLOResult:
            def __init__(self, dets, names):
                self.xyxy = []
                self.names = names or {}
                for det in dets:
                    boxes = det.get('boxes', [])
                    self.xyxy.append(torch.tensor(boxes))  # [N,6]: x1,y1,x2,y2,conf,cls

                # AUTO-PRINT on creation to match local YOLO behavior
                self.print()

            def print(self):
                # Recreate: "image i/n: 600x800 1 traffic light, ...  Speed: ..."
                n = len(self.xyxy)
                for i, pred in enumerate(self.xyxy, 1):
                    if isinstance(pred, torch.Tensor):
                        pred_cpu = pred.cpu()
                    else:
                        pred_cpu = torch.tensor(pred)
                    if pred_cpu.numel() == 0:
                        desc = "(no detections)"
                    else:
                        # Count by class id
                        ids = pred_cpu[:, 5].to(dtype=torch.int64).tolist()
                        counts = {}
                        for cid in ids:
                            counts[cid] = counts.get(cid, 0) + 1
                        parts = []
                        for cid, cnt in counts.items():
                            label = self.names.get(cid, str(cid))
                            # make plural like Ultralytics does (roughly)
                            parts.append(f"{cnt} {label}{'' if cnt == 1 else 's'}")
                        desc = ", ".join(parts) if parts else "(no detections)"
                    print(f"image {i}/{n}: {desc}")

            @property
            def names_dict(self):
                return self.names

        return YOLOResult(detections, names)

def is_vehicle_cococlass(label):
    """
    Check whether the label belongs to the vehicle class according
    to coco dataset.
    Args:
        -label(int): yolo detection prediction.
    Returns:
        -is_vehicle: bool
            whether this label belongs to the vehicle class
    """
    vehicle_class_array = np.array([1, 2, 3, 5, 7], dtype=np.int64)
    return True if 0 in (label - vehicle_class_array) else False


# Legacy compatibility for old imports
def run_worker(rank, world_size, num_split):
    """Legacy function for compatibility"""
    os.environ['MASTER_ADDR'] = 'localhost'
    os.environ['MASTER_PORT'] = '29500'
    print("[ML Manager] Legacy run_worker called - no longer needed in unified mode")


if __name__ == "__main__":
    # Test script
    print("ML Manager can now run in both local and distributed modes")
