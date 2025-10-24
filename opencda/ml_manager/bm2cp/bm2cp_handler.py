# bm2cp_handler.py
import base64
import io
import json
import numpy as np
import torch
from ts.torch_handler.base_handler import BaseHandler
from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor
from opencood.models.point_pillar_bm2cp import PointPillarBM2CP
import opencood.tools.train_utils as train_utils

class BM2CPHandler(BaseHandler):
    """
    TorchServe handler for the BM2CP point‑pillar feature extractor.
    It expects a JSON input with base64‑encoded lidar and images plus
    extrinsic/intrinsic matrices.
    """

    def initialize(self, ctx):
        """
        Called once during model loading.  Load hypes, preprocessor and model here.
        """
        model_dir = ctx.system_properties.get("model_dir")
        # hypes.yaml and checkpoint.pth must be packaged in extra-files
        hypes_path = os.path.join(model_dir, "hypes.yaml")
        ckpt_path  = os.path.join(model_dir, "checkpoint.pth")

        self.hypes = load_yaml(hypes_path)
        self.preproc = SpVoxelPreprocessor(self.hypes["preprocess"], train=False)

        # build model and load checkpoint
        self.model = PointPillarBM2CP(self.hypes['model']['args']).cuda().eval()
        epoch, self.model = train_utils.load_model(
            os.path.dirname(ckpt_path),
            self.model,
            int(os.path.basename(ckpt_path).split('epoch')[-1].split('.')[0])
        )

    def preprocess(self, requests):
        """
        Decode base64 lidar/images and convert extrinsics/intrinsics.
        Returns a list of batches to be passed to inference().
        """
        preprocessed = []
        for req in requests:
            data = req.get("body")
            if isinstance(data, (bytes, bytearray)):
                # TorchServe might pass raw bytes – decode JSON
                data = json.loads(data.decode("utf-8"))
            # lidar: base64 string of float32 buffer shaped (N,4)
            lidar_bytes = base64.b64decode(data["lidar"])
            lidar = np.frombuffer(lidar_bytes, dtype=np.float32).reshape(-1, 4)

            imgs, rots, trans, intrins = [], [], [], []
            for cam in data["cameras"]:
                img = np.frombuffer(base64.b64decode(cam["image"]), dtype=np.uint8)
                img = img.reshape((cam["height"], cam["width"], 3))
                # convert BGR->RGB as in your perception manager
                img_rgb = img[..., ::-1].copy()
                imgs.append(torch.from_numpy(img_rgb).permute(2, 0, 1).float())
                rots.append(torch.tensor(cam["R"], dtype=torch.float32))
                trans.append(torch.tensor(cam["t"], dtype=torch.float32))
                intrins.append(torch.tensor(cam["K"], dtype=torch.float32))

            B, N_cams = 1, len(imgs)
            placeholder_depth = torch.zeros(B, N_cams, cam["height"], cam["width"])

            # preprocess lidar to voxels
            proc_lidar_np = self.preproc.preprocess(
                np.ascontiguousarray(lidar, dtype=np.float32)
            )
            proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}
            coords = proc_lidar['voxel_coords']
            batch_index_column = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
            proc_lidar['voxel_coords'] = torch.cat((batch_index_column, coords), dim=1).int()

            image_inputs = {
                "imgs": torch.stack(imgs).view(B, N_cams, *imgs[0].shape),
                "rots": torch.stack(rots).view(B, N_cams, 3, 3),
                "trans": torch.stack(trans).view(B, N_cams, 3),
                "intrins": torch.stack(intrins).view(B, N_cams, 3, 3),
                "post_rots": torch.eye(3).view(1,1,3,3).expand(B, N_cams,3,3),
                "post_trans": torch.zeros(3).view(1,1,3).expand(B, N_cams,3),
                "depth_map": placeholder_depth
            }
            record_len = torch.tensor([N_cams], dtype=torch.int64)
            preprocessed.append({
                "processed_lidar": proc_lidar,
                "image_inputs": image_inputs,
                "record_len": record_len
            })
        return preprocessed

    def inference(self, inputs):
        """
        Run the model's get_feature() on each batch.
        """
        results = []
        with torch.inference_mode():
            for batch in inputs:
                batch_cuda = {k: (train_utils.to_device(v, 'cuda:0') if isinstance(v, dict) 
                                  else v.cuda()) if k != "record_len" else batch[k].cuda()
                              for k,v in batch.items()}
                feature_dict_gpu = self.model.get_feature(batch_cuda)
                # move to CPU and convert to base64 to minimise size over HTTP
                result = {k: base64.b64encode(v.cpu().numpy().tobytes()).decode('utf-8')
                          for k,v in feature_dict_gpu.items()}
                results.append(result)
        return results

    def postprocess(self, inference_output):
        """
        TorchServe expects a list of responses; just return the inference_output.
        """
        return inference_output
