
"""
LitServe model servers for distributed inference
Author: Tyler Landle <tlandle@gatech.edu>
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
#from opencood.models.point_pillar_bm2cp import PointPillarBM2CP
#from opencood.hypes_yaml.yaml_utils import load_yaml
#import opencood.tools.train_utils as train_utils

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
    
'''
class BM2CPVehicleEncoder(ls.LitAPI):
    """LitServe server for BM2CP vehicle-side encoding"""
    
    def setup(self, device="cuda", config_path=None, checkpoint_path=None):
        """Load BM2CP model for vehicle-side encoding"""
        self.device = device
        
        # Load configuration
        self.hypes = load_yaml(config_path)
        
        # Create model
        self.model = PointPillarBM2CP(self.hypes['model']['args'])
        
        # Load checkpoint
        epoch_id = int(checkpoint_path.split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(
            checkpoint_path.parent, self.model, epoch_id
        )
        
        self.model = self.model.to(device)
        self.model.eval()
    
    def decode_request(self, request: Dict) -> Dict:
        """Convert request to model input batch"""
        batch = {
            'processed_lidar': request['processed_lidar'],
            'image_inputs': request['image_inputs'],
            'record_len': torch.tensor([request['record_len']], dtype=torch.int64)
        }
        
        # Move to device
        batch = train_utils.to_device(batch, self.device)
        return batch
    
    @torch.no_grad()
    def predict(self, batch: Dict) -> Dict:
        """Generate BEV features for vehicle"""
        feature_dict = self.model.get_feature(batch)
        return feature_dict
    
    def encode_response(self, output: Dict) -> Dict:
        """Convert features to transmittable format"""
        response = {}
        for key, value in output.items():
            if isinstance(value, torch.Tensor):
                response[key] = {
                    'data': value.cpu().numpy().tolist(),
                    'shape': list(value.shape),
                    'dtype': str(value.dtype)
                }
            else:
                response[key] = value
        return response


class BM2CPEdgeFusion(ls.LitAPI):
    """LitServe server for BM2CP edge-side fusion and detection"""
    
    def setup(self, device="cuda", config_path=None, checkpoint_path=None):
        """Load BM2CP model for edge fusion"""
        self.device = device
        
        # Load configuration
        self.hypes = load_yaml(config_path)
        
        # Create model
        self.model = PointPillarBM2CP(self.hypes['model']['args'])
        
        # Load checkpoint  
        epoch_id = int(checkpoint_path.split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(
            checkpoint_path.parent, self.model, epoch_id
        )
        
        self.model = self.model.to(device)
        self.model.eval()
        
        # Create post-processor
        from opencood.data_utils.post_processor import VoxelPostprocessor
        self.post_processor = VoxelPostprocessor(
            self.hypes['postprocess'],
            dataset=None,
            train=False
        )
    
    def decode_request(self, request: Dict) -> Dict:
        """Decode feature dictionaries from multiple vehicles"""
        # Reconstruct tensors from transmitted format
        feature_dicts = []
        for feat_dict in request['feature_dicts']:
            reconstructed = {}
            for key, value in feat_dict.items():
                if isinstance(value, dict) and 'data' in value:
                    # Reconstruct tensor
                    tensor = torch.tensor(value['data'], dtype=eval(value['dtype']))
                    tensor = tensor.reshape(value['shape']).to(self.device)
                    reconstructed[key] = tensor
                else:
                    reconstructed[key] = value
            feature_dicts.append(reconstructed)
        
        pairwise_t_matrix = torch.tensor(
            request['pairwise_t_matrix'], 
            dtype=torch.float32
        ).to(self.device)
        
        return {
            'feature_dicts': feature_dicts,
            'pairwise_t_matrix': pairwise_t_matrix,
            'record_len': torch.tensor(request['record_len'], dtype=torch.int64)
        }
    
    @torch.no_grad()
    def predict(self, inputs: Dict) -> Dict:
        """Perform edge fusion and detection"""
        feature_dicts = inputs['feature_dicts']
        pairwise_t_matrix = inputs['pairwise_t_matrix']
        record_len = inputs['record_len']
        
        # Stack features
        features_tensor = torch.cat([d['spatial_features'] for d in feature_dicts], dim=0)
        psm_tensor = torch.cat([d['psm'] for d in feature_dicts], dim=0)
        rm_tensor = torch.cat([d['rm'] for d in feature_dicts], dim=0)
        thres_map_tensor = torch.cat([d['thres_map'] for d in feature_dicts], dim=0)
        
        # Run fusion
        fused_feature, communication_rates, result_dict = self.model.fusion_net(
            features_tensor,
            psm_tensor,
            thres_map_tensor,
            record_len,
            pairwise_t_matrix,
            backbone=self.model.backbone,
            heads=[self.model.shrink_conv, self.model.cls_head, self.model.reg_head]
        )
        
        # Generate predictions
        if self.model.shrink_flag:
            fused_feature = self.model.shrink_conv(fused_feature)
        
        pred_dict = {
            'psm': self.model.cls_head(fused_feature),
            'rm': self.model.reg_head(fused_feature)
        }
        
        # Post-process to get detections
        anchor_box = self.post_processor.generate_anchor_box()
        data_dict = {
            'ego': {
                'anchor_box': torch.from_numpy(anchor_box).cuda(),
                'transformation_matrix': torch.eye(4).cuda()
            }
        }
        output_dict = {'ego': pred_dict}
        
        boxes, scores = self.post_processor.post_process(data_dict, output_dict)
        
        return {
            'boxes': boxes,
            'scores': scores,
            'communication_rates': communication_rates
        }
    
    def encode_response(self, output: Dict) -> Dict:
        """Convert detections to response format"""
        response = {}
        
        if output['boxes'] is not None:
            response['boxes'] = output['boxes'].cpu().numpy().tolist()
            response['scores'] = output['scores'].cpu().numpy().tolist()
        else:
            response['boxes'] = []
            response['scores'] = []
        
        response['communication_rates'] = output['communication_rates'].cpu().item()
        
        return response
'''
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
