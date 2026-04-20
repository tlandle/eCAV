# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
WorldFusion perception manager.

Extracts intermediate spatial_features (before backbone) for
transmission to edge where Where2comm fusion is performed.
"""
from __future__ import annotations
import datetime
import os
import pathlib
import zlib
import torch
import numpy as np
from typing import Dict, Any, Optional

from opencood.hypes_yaml.yaml_utils import load_yaml
import opencood.tools.train_utils as train_utils
from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor
from ecav.core.sensing.perception.perception_manager import PerceptionManager
from ecav.ml_manager.litserve_transport import make_session, TimingRecorder
import ecav.core.sensing.perception.sensor_transformation as st

_WF_TIMING_FIELDS = [
    'tick', 'mode', 'agent_type',
    'build_batch_ms',
    'to_numpy_ms', 'pack_ms', 'payload_bytes',
    'grpc_ms',
    'server_read_ms', 'server_decode_ms', 'server_inference_ms',
    'server_encode_ms', 'server_total_ms',
    'response_bytes', 'unpack_ms', 'to_tensor_ms',
    'total_e2e_ms',
]


class WorldFusionPerceptionManager(PerceptionManager):
    """
    Perception manager for WorldFusion edge pipeline.

    This manager extracts intermediate BEV features (spatial_features) from the
    WorldFusion sensor encoder BEFORE the backbone. These features are transmitted
    to the edge where Where2comm fusion and detection heads are run.

    Key differences from BM2CPPerceptionManager:
    - Uses PointPillarWorldFusion model (specifically the _SensorEncoder)
    - Only extracts and stores spatial_features (not psm/rm/thres_map)
    - Features are extracted before backbone (true intermediate fusion)
    """

    def __init__(self,
                 vehicle,
                 config_yaml: Dict[str, Any],
                 cav_world=None,
                 data_dump: bool = False,
                 carla_world=None,
                 infra_id: Optional[int] = None,
                 tracking_manager=None,
                 debug_helper=None,
                 device: str = "cuda:0",
                 **kw):

        super().__init__(vehicle=vehicle,
                         config_yaml=config_yaml,
                         cav_world=cav_world,
                         data_dump=data_dump,
                         carla_world=carla_world,
                         infra_id=infra_id,
                         tracking_manager=tracking_manager,
                         debug_helper=debug_helper)
        self.device = device

        print("\n[WorldFusion] Initialising Perception Manager...")
        model_config = config_yaml['worldfusion_model']
        hypes_path = pathlib.Path(model_config['hypes_yaml'])

        self.hypes = load_yaml(str(hypes_path))
        print("[WorldFusion] Hypes loaded and parsed successfully.")

        pre_config = self.hypes["preprocess"]
        self._vp = SpVoxelPreprocessor(pre_config, train=False)
        print("[WorldFusion] SpVoxelPreprocessor initialized.")

        # Remote endpoint resolution — gRPC takes priority over LitServe.
        # Priority: WF_GRPC_ENDPOINT env (distributed containers) →
        #           worldfusion_grpc_endpoint per-agent YAML key →
        #           ml_manager attribute (only when -l/litserve active)
        self.grpc_endpoint = os.environ.get('WF_GRPC_ENDPOINT', None)
        if self.grpc_endpoint is None:
            self.grpc_endpoint = model_config.get('worldfusion_grpc_endpoint', None)
        if self.grpc_endpoint is None and cav_world is not None and getattr(cav_world, 'litserve', False) and cav_world.ml_manager is not None:
            self.grpc_endpoint = getattr(cav_world.ml_manager, 'worldfusion_grpc_endpoint', None)
        self.use_grpc = self.grpc_endpoint is not None

        # LitServe: WF_LITSERVE_ENDPOINT env → litserve_endpoint YAML key → global -l flag
        self.litserve_endpoint = os.environ.get('WF_LITSERVE_ENDPOINT', None)
        if self.litserve_endpoint is None:
            self.litserve_endpoint = model_config.get('litserve_endpoint', None)
        if self.litserve_endpoint is None and cav_world is not None and getattr(cav_world, 'litserve', False):
            if cav_world.ml_manager is not None:
                self.litserve_endpoint = getattr(cav_world.ml_manager, 'worldfusion_endpoint', 'http://localhost:18000')
        self.use_litserve = self.litserve_endpoint is not None and not self.use_grpc

        self._session = None
        self._wf_channel = None
        self._wf_stub = None

        if self.use_grpc:
            import grpc
            import perception_pb2_grpc as _pb2_grpc
            _32MB = 32 * 1024 * 1024
            self._wf_channel = grpc.insecure_channel(
                self.grpc_endpoint,
                options=[
                    ("grpc.max_send_message_length",    _32MB),
                    ("grpc.max_receive_message_length", _32MB),
                ],
            )
            self._wf_stub = _pb2_grpc.PerceptionServiceStub(self._wf_channel)
            self.model = None
            print(f"[WorldFusion] Using gRPC endpoint: {self.grpc_endpoint}")
        elif self.use_litserve:
            self._session = make_session()
            self.model = None
            print(f"[WorldFusion] Using LitServe endpoint: {self.litserve_endpoint}")
        else:
            # Share model across perception managers via cav_world to avoid
            # loading duplicate copies on GPU.
            shared = None
            if cav_world is not None:
                shared = getattr(cav_world, '_worldfusion_shared_model', None)

            if shared is not None:
                self.model = shared
                print("[WorldFusion] Reusing shared model from cav_world.")
            else:
                ckpt_path = pathlib.Path(model_config['checkpoint'])
                from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
                self.model = PointPillarWorldFusion(self.hypes['model']['args']).to(self.device).eval()
                print("[WorldFusion] Model created successfully.")

                epoch, self.model = train_utils.load_model(
                    str(ckpt_path.parent),
                    self.model,
                    epoch=int(str(ckpt_path.name).split('epoch')[-1].split('.')[0])
                )
                print(f"[WorldFusion] Loaded model weights from epoch {epoch}.")

                if cav_world is not None:
                    cav_world._worldfusion_shared_model = self.model
                    print("[WorldFusion] Model stored on cav_world for sharing.")

        # Feature dictionary - only stores spatial_features for edge transmission
        self.feature_dict = None
        self.feature_map = None
        self._first_run = True
        self._tick = 0
        self._timing = TimingRecorder(fieldnames=_WF_TIMING_FIELDS)
        self._agent_type = "RSU" if infra_id is not None else "Vehicle"

        # O5 batch-encoder state
        self._pending_batch = None       # set by build_batch(), cleared by apply_features()
        self._build_batch_ms = 0.0       # timing for build_batch(), passed to apply_features()
        self._features_extracted_this_tick = False  # True after apply_features(); clears on next detect()

        print("[WorldFusion] Init complete.\n")

    def detect(self, ego_pos, **kw):
        """
        Run detection step - extracts features for edge transmission.

        Note: Actual detection is performed on the edge, not here.
        This method just ensures features are extracted when sensors are ready.
        """
        lidar_ready = self.lidar and self.lidar.data is not None
        cams_ready = self.rgb_camera and all(c.image is not None for c in self.rgb_camera)
        has_cameras = self.rgb_camera and len(self.rgb_camera) > 0

        if lidar_ready and (cams_ready or not has_cameras):
            if self._features_extracted_this_tick:
                # O5: features already set by apply_features() from the edge batch encoder
                self._features_extracted_this_tick = False
            else:
                self.run_step()
        return {"vehicles": [], "traffic_lights": [], "static": []}

    def build_batch(self):
        """
        O5 phase 1: run _build_batch() and store result without sending to LitServe.
        Called by WorldFusionEdge before _run_o5_batch_encoder() aggregates all agents.
        No-ops if sensors are not yet ready.
        """
        import time as _t
        lidar_ready = self.lidar and self.lidar.data is not None
        cams_ready = self.rgb_camera and all(c.image is not None for c in self.rgb_camera)
        if not (lidar_ready and cams_ready):
            self._pending_batch = None
            self._build_batch_ms = 0.0
            return
        t0 = _t.time()
        self._pending_batch = self._build_batch()
        self._build_batch_ms = (_t.time() - t0) * 1000.0
        self._features_extracted_this_tick = False

    def apply_features(self, feature_dict_cpu, batch_timing=None):
        """
        O5 phase 2: store spatial_features returned by the edge batch encoder.
        Called by WorldFusionEdge after splitting the batched LitServe response.

        batch_timing : dict matching _WF_TIMING_FIELDS (grpc_ms, server_* etc.)
                       covering the shared gRPC/HTTP cost for the entire batch.
        """
        self.feature_dict = feature_dict_cpu
        self.feature_map = self.feature_dict['spatial_features'].half()
        self._features_extracted_this_tick = True
        self._pending_batch = None
        self._tick += 1

        if batch_timing is not None:
            # total_e2e covers this agent's build_batch + the shared batch HTTP round-trip
            total_e2e = round(self._build_batch_ms + batch_timing.get('total_e2e_ms', 0.0), 2)
            self._timing.record({
                'tick':             self._tick,
                'mode':             'distributed',
                'agent_type':       self._agent_type,
                'build_batch_ms':   round(self._build_batch_ms, 2),
                **batch_timing,
                'total_e2e_ms':     total_e2e,
            })

        if self._first_run:
            print(f"[WorldFusion O5] {self._agent_type}: first features applied. "
                  f"shape={self.feature_map.shape}", flush=True)
            self._first_run = False

    @torch.inference_mode()
    def run_step(self):
        """
        Extract intermediate spatial_features for edge transmission.

        This runs the sensor encoder up to (but not including) the backbone,
        extracting the fused multimodal BEV features.
        """
        # DEBUG: Print raw LiDAR statistics to check coordinate system
        lidar_data = self.lidar.data
        agent_type = "RSU" if self.vehicle is None else "Vehicle"

        # Save all ticks for offline comparison
        _save_tick = getattr(self, '_debug_save_tick', 0)
        if lidar_data is not None:
            save_path = f'/tmp/carla_lidar_{agent_type}_{_save_tick}.npy'
            np.save(save_path, lidar_data)
            self._debug_save_tick = _save_tick + 1

        # Quick periodic check for Lincoln's position (every 20 frames for RSU only)
        if agent_type == "RSU" and lidar_data is not None:
            frame_num = getattr(self, '_debug_frame', 0)
            self._debug_frame = frame_num + 1
            if frame_num % 20 == 0:
                try:
                    sensor_tf = self.lidar.sensor.get_transform()
                    for actor in self.carla_world.get_actors():
                        if 'lincoln' in actor.type_id.lower():
                            loc = actor.get_location()
                            # Compute Lincoln's LOCAL position
                            dx = loc.x - sensor_tf.location.x
                            dy = loc.y - sensor_tf.location.y
                            yaw_rad = np.radians(sensor_tf.rotation.yaw)
                            local_x = dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)
                            local_y = -dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)
                            # Count LiDAR points near Lincoln
                            dist = np.sqrt((lidar_data[:, 0] - local_x)**2 + (lidar_data[:, 1] - local_y)**2)
                            pts_near = (dist < 5.0).sum()
                            print(f"[LINCOLN CHECK] Frame {frame_num}: world=({loc.x:.1f},{loc.y:.1f},z={loc.z:.1f}), local=({local_x:.1f},{local_y:.1f}), LiDAR pts within 5m: {pts_near}")
                            break
                except Exception as e:
                    print(f"[LINCOLN CHECK] Frame {frame_num}: Error - {e}")

        if lidar_data is not None and self._first_run:
            # Print actual sensor world position
            sensor_tf = self.lidar.sensor.get_transform()
            print(f"\n[WorldFusion LIDAR DEBUG] {agent_type} id={self.id}")
            print(f"[WorldFusion LIDAR DEBUG] LiDAR SENSOR world pos: x={sensor_tf.location.x:.2f}, y={sensor_tf.location.y:.2f}, z={sensor_tf.location.z:.2f}")
            print(f"[WorldFusion LIDAR DEBUG] LiDAR SENSOR world rot: yaw={sensor_tf.rotation.yaw:.2f}")
            print(f"[WorldFusion LIDAR DEBUG] global_position from config: {self.global_position}")
            print(f"[WorldFusion LIDAR DEBUG] Raw LiDAR shape: {lidar_data.shape}")
            print(f"[WorldFusion LIDAR DEBUG] X range: [{lidar_data[:, 0].min():.1f}, {lidar_data[:, 0].max():.1f}]")
            print(f"[WorldFusion LIDAR DEBUG] Y range: [{lidar_data[:, 1].min():.1f}, {lidar_data[:, 1].max():.1f}]")
            print(f"[WorldFusion LIDAR DEBUG] Z range: [{lidar_data[:, 2].min():.1f}, {lidar_data[:, 2].max():.1f}]")

            # For RSU: compute Lincoln's expected local position dynamically
            # First, find Lincoln's ACTUAL position from CARLA actors
            lincoln_world_x, lincoln_world_y = -42.4, 127.7  # default from scenario
            try:
                actors = self.carla_world.get_actors()
                for actor in actors:
                    if 'lincoln' in actor.type_id.lower() or 'mkz' in actor.type_id.lower():
                        loc = actor.get_location()
                        lincoln_world_x, lincoln_world_y = loc.x, loc.y
                        print(f"[WorldFusion LIDAR DEBUG] Found Lincoln actor at ACTUAL world pos: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                        break
            except Exception as e:
                print(f"[WorldFusion LIDAR DEBUG] Could not get Lincoln actor: {e}")
            rsu_x, rsu_y = sensor_tf.location.x, sensor_tf.location.y
            rsu_yaw_rad = np.radians(sensor_tf.rotation.yaw)

            # Transform Lincoln's world position to sensor-local frame
            dx = lincoln_world_x - rsu_x
            dy = lincoln_world_y - rsu_y
            # Rotation: local = R^T @ world_delta (inverse rotation)
            cos_yaw, sin_yaw = np.cos(rsu_yaw_rad), np.sin(rsu_yaw_rad)
            lincoln_local_x = dx * cos_yaw + dy * sin_yaw
            lincoln_local_y = -dx * sin_yaw + dy * cos_yaw

            print(f"[WorldFusion LIDAR DEBUG] Lincoln world: ({lincoln_world_x}, {lincoln_world_y})")
            print(f"[WorldFusion LIDAR DEBUG] Sensor world: ({rsu_x:.2f}, {rsu_y:.2f}), yaw={sensor_tf.rotation.yaw:.1f}°")
            print(f"[WorldFusion LIDAR DEBUG] Expected Lincoln LOCAL: ({lincoln_local_x:.2f}, {lincoln_local_y:.2f})")

            # Check if there are points near Lincoln's expected position (within 5m)
            dist_to_lincoln = np.sqrt((lidar_data[:, 0] - lincoln_local_x)**2 + (lidar_data[:, 1] - lincoln_local_y)**2)
            nearby_points = lidar_data[dist_to_lincoln < 5.0]
            print(f"[WorldFusion LIDAR DEBUG] Points within 5m of expected Lincoln LOCAL ({lincoln_local_x:.1f}, {lincoln_local_y:.1f}): {len(nearby_points)}")
            if len(nearby_points) > 0:
                print(f"[WorldFusion LIDAR DEBUG]   Nearby points X: [{nearby_points[:, 0].min():.1f}, {nearby_points[:, 0].max():.1f}]")
                print(f"[WorldFusion LIDAR DEBUG]   Nearby points Y: [{nearby_points[:, 1].min():.1f}, {nearby_points[:, 1].max():.1f}]")

            # Print the cluster of points that are likely Lincoln (looking for vehicle-sized cluster around X=20-25)
            vehicle_region = lidar_data[(lidar_data[:, 0] > 15) & (lidar_data[:, 0] < 30) &
                                        (lidar_data[:, 1] > -5) & (lidar_data[:, 1] < 35)]
            if len(vehicle_region) > 0:
                print(f"[WorldFusion LIDAR DEBUG] Points in X=[15,30], Y=[-5,35]: {len(vehicle_region)}")
                print(f"[WorldFusion LIDAR DEBUG]   These points X: [{vehicle_region[:, 0].min():.1f}, {vehicle_region[:, 0].max():.1f}]")
                print(f"[WorldFusion LIDAR DEBUG]   These points Y: [{vehicle_region[:, 1].min():.1f}, {vehicle_region[:, 1].max():.1f}]")

        import time as _t
        t_batch_start = _t.time()
        batch = self._build_batch()
        t_batch_end = _t.time()
        build_batch_ms = (t_batch_end - t_batch_start) * 1000.0

        if self.use_grpc:
            feature_dict_cpu, remote_timing = self._extract_features_grpc(batch)
            t_step_end = _t.time()
            self._timing.record({
                'tick': self._tick,
                'mode': 'distributed',
                'agent_type': self._agent_type,
                'build_batch_ms': round(build_batch_ms, 2),
                **remote_timing,
                'total_e2e_ms': round((t_step_end - t_batch_start) * 1000.0, 2),
            })
        elif self.use_litserve:
            # Send preprocessed batch to remote GPU server
            feature_dict_cpu, remote_timing = self._extract_features_remote(batch)
            t_step_end = _t.time()
            self._timing.record({
                'tick': self._tick,
                'mode': 'distributed',
                'agent_type': self._agent_type,
                'build_batch_ms': round(build_batch_ms, 2),
                **remote_timing,
                # Override total_e2e_ms to cover full run_step() including build_batch
                'total_e2e_ms': round((t_step_end - t_batch_start) * 1000.0, 2),
            })
        else:
            # Run locally on GPU
            batch = train_utils.to_device(batch, self.device)
            feature_dict_gpu = self._extract_intermediate_features(batch)
            feature_dict_cpu = {
                'spatial_features': feature_dict_gpu['spatial_features'].cpu()
            }
            t_step_end = _t.time()
            local_ms = (t_step_end - t_batch_end) * 1000.0
            total_ms = (t_step_end - t_batch_start) * 1000.0
            self._timing.record({
                'tick': self._tick,
                'mode': 'local',
                'agent_type': self._agent_type,
                'build_batch_ms': round(build_batch_ms, 2),
                'to_numpy_ms': 0, 'pack_ms': 0, 'payload_bytes': 0,
                'grpc_ms': 0,
                'server_read_ms': 0, 'server_decode_ms': 0,
                'server_inference_ms': round(local_ms, 2),
                'server_encode_ms': 0, 'server_total_ms': 0,
                'response_bytes': 0, 'unpack_ms': 0, 'to_tensor_ms': 0,
                'total_e2e_ms': round(total_ms, 2),
            })

        self._tick += 1

        # Store only spatial_features for transmission (keep on CPU for transfer)
        self.feature_dict = feature_dict_cpu
        self.feature_map = self.feature_dict['spatial_features'].half()

        print(f"[WorldFusion run_step] {agent_type}: "
              f"build_batch={build_batch_ms:.0f}ms | "
              f"total={self._timing._rows[-1]['total_e2e_ms']:.0f}ms", flush=True)

        if self._first_run:
            print(f"\n[WorldFusion] >>> SUCCESS: FEATURE EXTRACTION COMPLETE. <<<")
            print(f"[WorldFusion] spatial_features shape: {self.feature_map.shape}")
            self._first_run = False

    def _extract_features_remote(self, batch: Dict):
        """
        Send preprocessed batch to LitServe endpoint for GPU feature extraction.

        Parameters
        ----------
        batch : dict
            CPU batch from _build_batch() with processed_lidar, image_inputs, record_len

        Returns
        -------
        (result_tensors, timing_dict)
            result_tensors : dict with 'spatial_features' tensor on CPU
            timing_dict    : per-call timing breakdown for TimingRecorder
        """
        import time as _t
        import msgpack
        import msgpack_numpy as m_np
        m_np.patch()

        t0 = _t.time()

        def _tensors_to_numpy(obj):
            if isinstance(obj, torch.Tensor):
                return obj.contiguous().numpy()
            if isinstance(obj, dict):
                return {k: _tensors_to_numpy(v) for k, v in obj.items()}
            return obj

        batch_np = _tensors_to_numpy(batch)
        # Cast imgs float32 → uint8 to reduce payload by ~75% (~8MB → ~2MB).
        # Lossless: CARLA captures uint8 pixels; _build_batch() casts to float32
        # only for local inference. Server casts back before camenc.
        if 'image_inputs' in batch_np and 'imgs' in batch_np['image_inputs']:
            batch_np['image_inputs']['imgs'] = batch_np['image_inputs']['imgs'].astype(np.uint8)
        t1 = _t.time()

        payload = zlib.compress(msgpack.packb(batch_np, use_bin_type=True))
        t2 = _t.time()

        response = self._session.post(
            f"{self.litserve_endpoint}/extract_features",
            data=payload,
            headers={
                "Content-Type": "application/octet-stream",
                "Content-Encoding": "zlib",
            },
            timeout=30,
        )
        t3 = _t.time()

        if response.status_code != 200:
            raise RuntimeError(
                f"WorldFusion LitServe error: {response.status_code} {response.text}"
            )

        result = msgpack.unpackb(response.content, raw=False)
        t4 = _t.time()

        result_tensors = {k: torch.from_numpy(v) for k, v in result.items()}
        t5 = _t.time()

        def _hdr(name, default='0'):
            return float(response.headers.get(name, default))

        srv_read_ms   = _hdr('X-Server-Read-Ms')
        srv_decode_ms = _hdr('X-Server-Decode-Ms')
        srv_infer_ms  = _hdr('X-Server-Inference-Ms')
        srv_encode_ms = _hdr('X-Server-Encode-Ms')
        srv_total_ms  = _hdr('X-Server-Total-Ms')

        to_numpy_ms  = (t1 - t0) * 1000.0
        pack_ms      = (t2 - t1) * 1000.0
        http_ms      = (t3 - t2) * 1000.0
        unpack_ms    = (t4 - t3) * 1000.0
        to_tensor_ms = (t5 - t4) * 1000.0
        total_ms     = (t5 - t0) * 1000.0

        print(f"[WorldFusion REMOTE {self._agent_type}] "
              f"to_numpy={to_numpy_ms:.0f}ms | "
              f"pack={pack_ms:.0f}ms ({len(payload)/1024:.0f}KB) | "
              f"http={http_ms:.0f}ms "
              f"(srv_read={srv_read_ms:.0f} decode={srv_decode_ms:.0f} "
              f"infer={srv_infer_ms:.0f} encode={srv_encode_ms:.0f} total={srv_total_ms:.0f}ms) | "
              f"unpack={unpack_ms:.0f}ms ({len(response.content)/1024:.0f}KB) | "
              f"to_tensor={to_tensor_ms:.0f}ms | "
              f"total={total_ms:.0f}ms", flush=True)

        timing = {
            'to_numpy_ms':          round(to_numpy_ms, 2),
            'pack_ms':              round(pack_ms, 2),
            'payload_bytes':        len(payload),
            'http_ms':              round(http_ms, 2),
            'server_read_ms':       srv_read_ms,
            'server_decode_ms':     srv_decode_ms,
            'server_inference_ms':  srv_infer_ms,
            'server_encode_ms':     srv_encode_ms,
            'server_total_ms':      srv_total_ms,
            'response_bytes':       len(response.content),
            'unpack_ms':            round(unpack_ms, 2),
            'to_tensor_ms':         round(to_tensor_ms, 2),
            'total_e2e_ms':         round(total_ms, 2),
        }
        return result_tensors, timing

    def _extract_features_grpc(self, batch: Dict):
        """
        Send preprocessed batch to WorldFusion gRPC server for feature extraction.

        Same encoding as _extract_features_remote (zlib+msgpack) — only the transport
        changes. Server timing comes from WfResponse fields instead of HTTP headers.

        Returns (result_tensors, timing_dict) matching _WF_TIMING_FIELDS.
        """
        import time as _t
        import msgpack
        import msgpack_numpy as m_np
        import perception_pb2
        m_np.patch()

        t0 = _t.time()

        def _tensors_to_numpy(obj):
            if isinstance(obj, torch.Tensor):
                return obj.contiguous().numpy()
            if isinstance(obj, dict):
                return {k: _tensors_to_numpy(v) for k, v in obj.items()}
            return obj

        batch_np = _tensors_to_numpy(batch)
        if 'image_inputs' in batch_np and 'imgs' in batch_np['image_inputs']:
            batch_np['image_inputs']['imgs'] = batch_np['image_inputs']['imgs'].astype(np.uint8)
        t1 = _t.time()

        payload = zlib.compress(msgpack.packb(batch_np, use_bin_type=True))
        t2 = _t.time()

        actor_id = int(getattr(self, 'id', 0) or 0)
        response = self._wf_stub.ExtractWfFeatures(
            perception_pb2.WfRequest(payload=payload, actor_id=actor_id),
            timeout=30,
        )
        t3 = _t.time()

        result = msgpack.unpackb(response.payload, raw=False)
        t4 = _t.time()

        result_tensors = {k: torch.from_numpy(v) for k, v in result.items()}
        t5 = _t.time()

        to_numpy_ms  = (t1 - t0) * 1000.0
        pack_ms      = (t2 - t1) * 1000.0
        grpc_ms      = (t3 - t2) * 1000.0
        unpack_ms    = (t4 - t3) * 1000.0
        to_tensor_ms = (t5 - t4) * 1000.0
        total_ms     = (t5 - t0) * 1000.0

        print(f"[WorldFusion gRPC {self._agent_type}] "
              f"to_numpy={to_numpy_ms:.0f}ms | "
              f"pack={pack_ms:.0f}ms ({len(payload)/1024:.0f}KB) | "
              f"grpc={grpc_ms:.0f}ms "
              f"(srv_unpack={response.unpack_ms:.0f} infer={response.inference_ms:.0f} "
              f"pack={response.pack_ms:.0f} total={response.total_ms:.0f}ms) | "
              f"unpack={unpack_ms:.0f}ms ({len(response.payload)/1024:.0f}KB) | "
              f"to_tensor={to_tensor_ms:.0f}ms | "
              f"total={total_ms:.0f}ms", flush=True)

        timing = {
            'to_numpy_ms':          round(to_numpy_ms, 2),
            'pack_ms':              round(pack_ms, 2),
            'payload_bytes':        len(payload),
            'grpc_ms':              round(grpc_ms, 2),
            'server_read_ms':       0,
            'server_decode_ms':     round(response.unpack_ms, 2),
            'server_inference_ms':  round(response.inference_ms, 2),
            'server_encode_ms':     round(response.pack_ms, 2),
            'server_total_ms':      round(response.total_ms, 2),
            'response_bytes':       len(response.payload),
            'unpack_ms':            round(unpack_ms, 2),
            'to_tensor_ms':         round(to_tensor_ms, 2),
            'total_e2e_ms':         round(total_ms, 2),
        }
        return result_tensors, timing

    def close(self):
        """Flush timing CSV and release transport resources."""
        if len(self._timing) > 0:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            agent_tag = 'rsu' if self.vehicle is None else 'vehicle'
            path = os.path.join('logs', f'wf_timing_{agent_tag}_{ts}.csv')
            self._timing.dump_csv(path)
            print(f"[WorldFusion] Timing CSV written ({len(self._timing)} rows): {path}")
        if self._wf_channel is not None:
            self._wf_channel.close()
            self._wf_channel = None
        if self._session is not None:
            self._session.close()
            self._session = None

    def _extract_intermediate_features(self, batch: Dict) -> Dict:
        """
        Extract intermediate features from sensor encoder (before backbone).

        This replicates the first part of _SensorEncoder.forward() up to the
        multimodal fusion, but stops before running the backbone.
        """
        sensor = self.model.sensor

        # LiDAR branch
        pc = batch['processed_lidar']
        rec_len = batch['record_len']
        bd = {
            'voxel_features': pc['voxel_features'],
            'voxel_coords': pc['voxel_coords'],
            'voxel_num_points': pc['voxel_num_points'],
            'record_len': rec_len,
        }
        bd = sensor.scatter(sensor.pillar_vfe(bd))

        # Camera branch (if model supports it and image_inputs available)
        if (sensor.use_camera and 'image_inputs' in batch
                and batch['image_inputs'] is not None):
            from einops import rearrange
            imgs = batch['image_inputs']['imgs']
            B, N, C, imH, imW = imgs.shape
            x = imgs.view(B * N, C, imH, imW)
            _, x = sensor.camenc(x, batch['image_inputs']['depth_map'], batch['record_len'])
            x = rearrange(x, '(b n) c d h w -> b n c d h w', b=B, n=N)
            x = x.permute(0, 1, 3, 4, 5, 2)  # [B,N,D,fH,fW,C]
            geom = sensor._get_geometry(batch['image_inputs'])  # [B,N,D,fH,fW,3]
            img_voxel = sensor.voxel_pooling(geom, x)

            # Fuse with LiDAR voxels
            bd, thres_map, mask, each_mask = sensor.vox_fuse(img_voxel, bd)
        else:
            # LiDAR-only mode: collapse Z dimension manually
            spatial_3d = bd['spatial_features_3d']
            b, c, z, y, x = spatial_3d.shape
            bd['spatial_features'] = spatial_3d.view(b, c * z, y, x)

        # Return intermediate features (before backbone)
        # spatial_features shape: [B, C*Z, Y, X] where typically [1, 64*4, 192, 704]
        return {
            'spatial_features': bd['spatial_features']
        }

    def _build_batch(self) -> Dict[str, Any]:
        """
        Build input batch from sensor data.
        Same as BM2CPPerceptionManager._build_batch().
        """
        # LiDAR processing
        # Both Multi-V2X and CARLA use the same Z convention: negative = below
        # sensor, positive = above. Multi-V2X clips Z > 0 during data generation.
        # The voxel z-range [-6, 2] handles the clipping at inference.
        proc_lidar_np = self._vp.preprocess(
            np.ascontiguousarray(self.lidar.data, dtype=np.float32)
        )
        proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}
        coords = proc_lidar['voxel_coords']

        # DEBUG: Print voxel coordinate statistics
        if self._first_run:
            # Voxel coords are in [z, y, x] format for spconv
            voxel_x = coords[:, 2].numpy()
            voxel_y = coords[:, 1].numpy()
            voxel_z = coords[:, 0].numpy()
            print(f"[WorldFusion VOXEL DEBUG] Voxel coords shape: {coords.shape}")
            print(f"[WorldFusion VOXEL DEBUG] Voxel X (grid): [{voxel_x.min()}, {voxel_x.max()}]")
            print(f"[WorldFusion VOXEL DEBUG] Voxel Y (grid): [{voxel_y.min()}, {voxel_y.max()}]")
            print(f"[WorldFusion VOXEL DEBUG] Voxel Z (grid): [{voxel_z.min()}, {voxel_z.max()}]")

            # Expected Lincoln position in voxel space (voxel_size=0.4, range=[-40,40])
            # local (21.2, 13.7) -> voxel = (coord + 40) / 0.4
            lincoln_vx = int((21.2 + 40) / 0.4)  # = 153
            lincoln_vy = int((13.7 + 40) / 0.4)  # = 134
            print(f"[WorldFusion VOXEL DEBUG] Expected Lincoln voxel: ({lincoln_vx}, {lincoln_vy})")

            # Check if voxels near Lincoln's expected position exist
            nearby_mask = (np.abs(voxel_x - lincoln_vx) < 10) & (np.abs(voxel_y - lincoln_vy) < 10)
            nearby_count = nearby_mask.sum()
            print(f"[WorldFusion VOXEL DEBUG] Voxels near expected Lincoln: {nearby_count}")

        batch_index_column = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
        proc_lidar['voxel_coords'] = torch.cat((batch_index_column, coords), dim=1).int()

        # Get target dimensions from model config (data_aug_conf.final_dim)
        data_aug_conf = self.hypes.get('fusion', {}).get('args', {}).get('data_aug_conf', {})
        final_dim = data_aug_conf.get('final_dim', [360, 480])  # [H, W]
        target_h, target_w = final_dim[0], final_dim[1]

        # Camera processing
        imgs, rots, trans, intrins = [], [], [], []
        for cam in self.rgb_camera:
            img_bgr_to_rgb = cam.image[..., ::-1].copy()
            orig_h, orig_w = img_bgr_to_rgb.shape[:2]

            # Resize image to match model's expected dimensions
            img_tensor = torch.from_numpy(img_bgr_to_rgb).permute(2, 0, 1).float()  # [C, H, W]
            img_resized = torch.nn.functional.interpolate(
                img_tensor.unsqueeze(0),  # [1, C, H, W]
                size=(target_h, target_w),
                mode='bilinear',
                align_corners=False
            ).squeeze(0)  # [C, H, W]
            imgs.append(img_resized)

            cam_to_lidar = (
                np.array(self.lidar.sensor.get_transform().get_inverse_matrix()) @
                np.array(cam.sensor.get_transform().get_matrix())
            )
            rots.append(torch.from_numpy(cam_to_lidar[:3, :3].copy()).float())
            trans.append(torch.from_numpy(cam_to_lidar[:3, 3].copy()).float())

            # Scale intrinsics to match resized image
            K = st.get_camera_intrinsic(cam.sensor).copy()
            K[0, :] *= target_w / orig_w  # Scale fx, cx
            K[1, :] *= target_h / orig_h  # Scale fy, cy
            intrins.append(torch.from_numpy(K).float())

        B, N_cams = 1, len(self.rgb_camera)
        placeholder_depth = torch.zeros(B, N_cams, target_h, target_w)

        image_inputs = {
            "imgs": torch.stack(imgs).view(B, N_cams, *imgs[0].shape),
            "rots": torch.stack(rots).view(B, N_cams, 3, 3),
            "trans": torch.stack(trans).view(B, N_cams, 3),
            "intrins": torch.stack(intrins).view(B, N_cams, 3, 3),
            "post_rots": torch.eye(3, dtype=torch.float32).view(1, 1, 3, 3).expand(B, N_cams, 3, 3),
            "post_trans": torch.zeros(3, dtype=torch.float32).view(1, 1, 3).expand(B, N_cams, 3),
            "depth_map": placeholder_depth
        }

        # record_len represents number of agents in this batch, not number of cameras
        # For a single vehicle/RSU extracting features, this is always 1
        return {
            "processed_lidar": proc_lidar,
            "image_inputs": image_inputs,
            "record_len": torch.tensor([1], dtype=torch.int64)
        }
