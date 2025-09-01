"""
LitServe model servers for distributed inference
Author: Tyler Landle <tlandle@gatech.edu>
"""
import litserve as ls
import torch
import numpy as np
from typing import Dict, Any, List
import torch.hub
from opencood.models.point_pillar_bm2cp import PointPillarBM2CP
from opencood.hypes_yaml.yaml_utils import load_yaml
import opencood.tools.train_utils as train_utils

class YOLOv5Server(ls.LitAPI):
    """LitServe server for YOLOv5 object detection"""
    
    def setup(self, device="cuda"):
        """Load YOLOv5 model"""
        self.device = device
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m')
        self.model = self.model.to(device)
        self.model.eval()
    
    def decode_request(self, request: Dict) -> torch.Tensor:
        """Convert request to model input"""
        # Expecting {'images': List[np.ndarray]}
        images = request['images']
        if isinstance(images[0], np.ndarray):
            # Convert numpy arrays to torch tensors
            images = [torch.from_numpy(img).float() for img in images]
        return images
    
    def predict(self, images: List[torch.Tensor]) -> Any:
        """Run YOLOv5 inference"""
        with torch.no_grad():
            results = self.model(images)
        return results
    
    def encode_response(self, output: Any) -> Dict:
        """Convert model output to response"""
        # Extract bounding boxes, scores, and labels
        response = {
            'detections': []
        }
        
        for i, pred in enumerate(output.xyxy):
            if pred.is_cuda:
                pred = pred.cpu()
            response['detections'].append({
                'boxes': pred.numpy().tolist(),
                'names': output.names
            })
        
        return response


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
