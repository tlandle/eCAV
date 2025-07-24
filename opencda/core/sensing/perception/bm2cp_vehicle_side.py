import torch
from opencood.models.point_pillar_bm2cp import PointPillarBM2CP

class VehicleEncoder:
    """
    This class runs on each vehicle. It loads the full BM2CP model
    but only uses the encoder parts to generate BEV feature maps.
    """
    def __init__(self, checkpoint_path):
        # Load the entire pre-trained model from its checkpoint
        # Set 'train_params' to None as we are in inference mode
        self.full_model = PointPillarBM2CP(train_params=None)
        self.full_model.load_state_dict(torch.load(checkpoint_path))
        self.full_model.eval() # Set model to evaluation mode

        # -- Exact APIs to use from PointPillarBM2CP --
        # Extract only the modules needed for on-vehicle encoding
        self.pillar_vfe = self.full_model.pillar_vfe
        self.scatter = self.full_model.scatter
        self.cam_encode = self.full_model.cam_encode

    @torch.no_grad() # Disable gradient calculation for efficiency
    def encode(self, processed_lidar, camera_data):
        """
        Takes preprocessed sensor data and returns the BEV feature map.

        Args:
            processed_lidar (dict): A dictionary containing voxelized lidar data.
            camera_data (torch.Tensor): A tensor of camera image data.

        Returns:
            torch.Tensor: The final BEV feature map to be sent to the edge.
        """
        # 1. Process LiDAR to create a LiDAR BEV map
        voxel_features = self.pillar_vfe(processed_lidar['voxel_features'],
                                         processed_lidar['voxel_num_points'],
                                         processed_lidar['voxel_coords'])
        lidar_bev_features = self.scatter(voxel_features,
                                          processed_lidar['voxel_coords'],
                                          processed_lidar['batch_size'])

        # 2. Process Camera images to create a Camera BEV map
        camera_bev_features = self.cam_encode(camera_data)

        # 3. Concatenate LiDAR and Camera BEV features
        # The exact dimension might need checking from the original forward pass
        final_bev_features = torch.cat([lidar_bev_features, camera_bev_features], dim=1)

        return final_bev_features
