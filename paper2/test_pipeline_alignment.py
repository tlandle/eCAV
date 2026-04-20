"""
Minimal test: feed the SAME Multi-V2X data through both the profiler
pipeline and the live edge manager pipeline, compare outputs at each
stage to find where they diverge.

Usage:
    conda run -n opencda310 python paper2/test_pipeline_alignment.py
"""
import os, sys, math
import numpy as np
import torch

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(REPO)
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'ecav/worldfusion'))

DEVICE = 'cuda'
DATA_ROOT = '/data1/Datasets/Multi-V2X'
TOWN = 'Town05__2023_11_13_23_03_07'
RSU = 'rsu_250'
TICK = 5

# ── Load model (single copy) ─────────────────────────────────────
from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
import opencood.tools.train_utils as train_utils

WF_DIR = os.path.join(REPO, 'ecav/ml_manager/models/worldfusion_multiv2x_caronly_ndm')
hypes = load_yaml(os.path.join(WF_DIR, 'config.yaml'))
model = PointPillarWorldFusion(hypes['model']['args']).to(DEVICE).eval()
_, model = train_utils.load_model(WF_DIR, model, epoch=16)
print(f"Model loaded (epoch 16), use_camera={model.sensor.use_camera}")

# ── Load one frame from Multi-V2X (profiler path) ────────────────
from paper2.paper2_offline_profiler_v3 import MultiV2XScene, OfflinePipelineV3
import yaml as pyyaml

scene = MultiV2XScene(DATA_ROOT, TOWN, rsu_name=RSU, max_frames=10)
agents = scene.connected_agents(TICK)
if scene.ego_cav not in agents:
    agents = [scene.ego_cav] + agents
N = len(agents)
print(f"\nFrame: {TOWN}/{RSU} tick={TICK}, N={N} agents")

# Get agent poses
poses = []
for a in agents:
    pose = scene.agent_lidar_pose(a, TICK)
    poses.append(pose)
    print(f"  {a}: pose=[{pose[0]:.1f}, {pose[1]:.1f}, {pose[2]:.1f}, yaw={pose[4]:.1f}]")

# ── Profiler path: voxelize + fuse + detect ───────────────────────
print("\n=== PROFILER PATH ===")
pipeline = OfflinePipelineV3()

with torch.no_grad():
    sf_prof, rl_prof, pw_prof = pipeline.voxelize_agents(scene, TICK, agents)
print(f"spatial_features: {sf_prof.shape}")
print(f"pairwise_t: {pw_prof.shape}")
print(f"pairwise_t[0,0,1] (ego to agent1):\n{pw_prof[0,0,1].cpu().numpy().round(3)}")
print(f"pairwise_t[0,1,0] (agent1 to ego):\n{pw_prof[0,1,0].cpu().numpy().round(3)}")

with torch.no_grad():
    fused_prof, pred_dict_prof = pipeline.run_edge_fusion(sf_prof, pw_prof, rl_prof)
print(f"fused_feature: {fused_prof.shape}")
print(f"psm max: {pred_dict_prof['psm'].sigmoid().max():.4f}")

ego_pose = scene.agent_lidar_pose(scene.ego_cav, TICK)
dets_prof, info_prof = pipeline.run_detection(pred_dict_prof, ego_pose)
print(f"detections: {len(dets_prof)}")
if len(dets_prof) > 0:
    for i in range(min(5, len(dets_prof))):
        print(f"  det {i}: x={dets_prof[i,3]:.1f} y={dets_prof[i,5]:.1f} score={dets_prof[i,7]:.3f}")

# ── Live edge manager path: same data, different transform ───────
print("\n=== LIVE EDGE MANAGER PATH ===")
from opencood.utils import transformation_utils

# Compute pairwise transforms the way the live edge manager does
world_anchor = [0, 0, 0, 0, 0, 0]
max_cav = hypes['train_params']['max_cav']

# Method 1: Live edge manager (x1_to_x2 to world origin)
T_to_world_live = []
for j in range(N):
    T = transformation_utils.x1_to_x2(
        list(poses[j]),
        [0, 0, 0, 0, 0, 0]
    )
    T_to_world_live.append(T)

pw_live = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))
for i in range(N):
    for j in range(N):
        if i == j:
            pw_live[0, i, j] = np.eye(4)
        else:
            pw_live[0, i, j] = np.linalg.inv(T_to_world_live[j]) @ T_to_world_live[i]
pw_live_t = torch.from_numpy(pw_live).float().to(DEVICE)

print(f"pairwise_t_live[0,0,1]:\n{pw_live[0,0,1].round(3)}")
print(f"pairwise_t_live[0,1,0]:\n{pw_live[0,1,0].round(3)}")

# Method 2: Profiler path (from voxelize_agents)
print(f"\npairwise_t_prof[0,0,1]:\n{pw_prof[0,0,1].cpu().numpy().round(3)}")
print(f"pairwise_t_prof[0,1,0]:\n{pw_prof[0,1,0].cpu().numpy().round(3)}")

# Compare
diff_01 = np.abs(pw_live[0,0,1] - pw_prof[0,0,1].cpu().numpy()).max()
diff_10 = np.abs(pw_live[0,1,0] - pw_prof[0,1,0].cpu().numpy()).max()
print(f"\nMax diff pairwise[0,0,1]: {diff_01:.6f}")
print(f"Max diff pairwise[0,1,0]: {diff_10:.6f}")

if diff_01 > 0.01 or diff_10 > 0.01:
    print("*** PAIRWISE TRANSFORMS DIFFER ***")
else:
    print("Pairwise transforms match.")

# Run fusion with live transforms on same spatial features
with torch.no_grad():
    rl_live = torch.tensor([N], dtype=torch.int64, device=DEVICE)
    fused_live, pred_dict_live = pipeline.run_edge_fusion(sf_prof, pw_live_t, rl_live)
print(f"\nfused_feature (live transforms): {fused_live.shape}")
print(f"psm max (live): {pred_dict_live['psm'].sigmoid().max():.4f}")
print(f"psm max (prof): {pred_dict_prof['psm'].sigmoid().max():.4f}")

fused_diff = (fused_live - fused_prof).abs().max().item()
print(f"Max fused feature diff: {fused_diff:.6f}")

if fused_diff > 0.01:
    print("*** FUSED FEATURES DIFFER - transform mismatch causes different fusion ***")
    # Run detection with live fused features
    dets_live, info_live = pipeline.run_detection(pred_dict_live, ego_pose)
    print(f"detections (live transforms): {len(dets_live)}")
    if len(dets_live) > 0:
        for i in range(min(5, len(dets_live))):
            print(f"  det {i}: x={dets_live[i,3]:.1f} y={dets_live[i,5]:.1f} score={dets_live[i,7]:.3f}")
else:
    print("Fused features match - transform is not the issue.")

# ── Also test: what does the post-processor do with world_anchor? ─
print("\n=== POST-PROCESSOR COMPARISON ===")
from opencood.data_utils.post_processor import build_postprocessor

pp = build_postprocessor(hypes['postprocess'], dataset='opv2v', train=False)
anchor_box = pp.generate_anchor_box()
print(f"anchor_box shape: {anchor_box.shape}")

# Profiler uses origin_pose
origin_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
data_dict_origin = {
    'ego': {
        'anchor_box': torch.from_numpy(anchor_box).to(DEVICE),
        'transformation_matrix': torch.eye(4).to(DEVICE),
        'world_anchor': [origin_pose],
        'lidar_pose': np.array([origin_pose]),
    }
}

# Live edge manager originally used world_anchor
wa = [-78.0, 128.0, 0.0, 0.0, 0.0, 0.0]
data_dict_anchor = {
    'ego': {
        'anchor_box': torch.from_numpy(anchor_box).to(DEVICE),
        'transformation_matrix': torch.eye(4).to(DEVICE),
        'world_anchor': [wa],
        'lidar_pose': np.array([wa]),
    }
}

output_dict = {'ego': pred_dict_prof}

boxes_origin, scores_origin = pp.post_process(data_dict_origin, output_dict)
boxes_anchor, scores_anchor = pp.post_process(data_dict_anchor, output_dict)

if boxes_origin is not None:
    from opencood.utils import box_utils
    b7_origin = box_utils.corner_to_center(boxes_origin.cpu().numpy(), order='hwl')
    print(f"\nWith origin pose: {len(b7_origin)} detections")
    for i in range(min(3, len(b7_origin))):
        print(f"  det {i}: x={b7_origin[i,0]:.1f} y={b7_origin[i,1]:.1f}")

if boxes_anchor is not None:
    b7_anchor = box_utils.corner_to_center(boxes_anchor.cpu().numpy(), order='hwl')
    print(f"\nWith world_anchor pose: {len(b7_anchor)} detections")
    for i in range(min(3, len(b7_anchor))):
        print(f"  det {i}: x={b7_anchor[i,0]:.1f} y={b7_anchor[i,1]:.1f}")
    print(f"\nDifference (should be world_anchor offset):")
    if boxes_origin is not None and len(b7_origin) == len(b7_anchor):
        dx = b7_anchor[0,0] - b7_origin[0,0]
        dy = b7_anchor[0,1] - b7_origin[0,1]
        print(f"  dx={dx:.1f} (expected {wa[0]:.1f})")
        print(f"  dy={dy:.1f} (expected {wa[1]:.1f})")

print("\n=== GT comparison ===")
gt = pipeline.gt_objects_in_range(scene, TICK)
print(f"GT objects in range: {len(gt)}")
for g in gt[:5]:
    print(f"  oid={g['oid']} x={g['x']:.1f} y={g['y']:.1f} dist={g['dist']:.1f}")
