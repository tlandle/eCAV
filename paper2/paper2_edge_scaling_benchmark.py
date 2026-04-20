#!/usr/bin/env python3
"""
Edge Pipeline Scaling Benchmark
================================
Measures wall-clock GPU timing for each stage of the edge cooperative
perception pipeline (WorldFusion + AB3DMOT + SMART/MTR) at increasing
agent counts N = {1, 2, 4, 8, 16, 24, 32, 48, 64}.

For each N, we construct synthetic inputs that match the expected data
distribution at that scale and run them through the actual model weights
on GPU. We also run the SB-SPS MAC model analytically for the V2V
communication baseline.

Outputs:
  - paper2_figures/benchmark_timing.csv       (raw timing data)
  - paper2_figures/fig_edge_stage_latency.pdf  (stacked per-stage vs N)
  - paper2_figures/fig_v2v_prr_vs_n.pdf        (PRR collapse)
  - paper2_figures/fig_edge_vs_v2v_e2e.pdf     (combined comparison)

Methodology documented in paper2_scaling_methodology.md
"""

import os
import sys
import time
import json
import csv
import math
import argparse
import warnings
from pathlib import Path
from collections import defaultdict

import numpy as np
import torch

warnings.filterwarnings("ignore")

# ── Output directory ────────────────────────────────────────────────
OUT_DIR = Path("paper2_figures")
OUT_DIR.mkdir(exist_ok=True)

# ── Agent counts to sweep ──────────────────────────────────────────
N_AGENTS = [1, 2, 4, 8, 16, 24, 32, 48, 64]
N_WARMUP = 3       # GPU warmup iterations (discarded)
N_REPEATS = 10     # Timed iterations per N

# ── Calibration from real experiment_results (see methodology) ─────
# Detections per frame scales roughly as: dets ~ 4.0 * N_egos
DETS_PER_EGO = 4.0
# Fraction of detections that become confirmed tracks (min_hits=3)
TRACK_CONFIRM_RATIO = 0.7

DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"


# ====================================================================
#  1.  WorldFusion (Where2Comm) Fusion Benchmark
# ====================================================================

def benchmark_worldfusion(n_values, model_dir=None):
    """
    Benchmark WorldFusion backbone + Where2Comm fusion at varying N.

    Constructs synthetic spatial_features [N, C, H, W] and runs through
    the model's forward path (backbone + fusion + heads).
    """
    print("\n=== WorldFusion Fusion Benchmark ===")

    sys.path.insert(0, str(Path("ecav/worldfusion").resolve()))
    from opencood.hypes_yaml.yaml_utils import load_yaml
    from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
    import opencood.tools.train_utils as train_utils

    # Load model config
    if model_dir is None:
        model_dir = "opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10"
    hypes = load_yaml(os.path.join(model_dir, "config.yaml"))
    model = PointPillarWorldFusion(hypes['model']['args']).to(DEVICE).eval()
    epoch, model = train_utils.load_model(model_dir, model, epoch=50)
    print(f"  Loaded WorldFusion epoch {epoch}")

    # Determine spatial_features shape from config
    # Pre-backbone: [N, C*Z, H, W] where C=64, Z=1, H=W=200
    zbound = hypes['model']['args']['img_params']['grid_conf']['zbound']
    z_dim = int((zbound[1] - zbound[0]) / zbound[2])
    C = hypes['model']['args']['pc_params']['point_pillar_scatter']['num_features']
    C_spatial = C * z_dim  # 64 * 1 = 64 for v2xsim

    # Grid size from preprocess config
    # For v2xsim: 200x200 at 0.4m voxel, 80m canvas
    xbound = hypes['model']['args']['img_params']['grid_conf']['xbound']
    ybound = hypes['model']['args']['img_params']['grid_conf']['ybound']
    H = int((xbound[1] - xbound[0]) / xbound[2])
    W = int((ybound[1] - ybound[0]) / ybound[2])
    print(f"  spatial_features shape per agent: [{C_spatial}, {H}, {W}]")

    results = {}
    for N in n_values:
        # Construct synthetic batch
        spatial_features = torch.randn(N, C_spatial, H, W, device=DEVICE)
        record_len = torch.tensor([N], dtype=torch.int64, device=DEVICE)
        pairwise_t = torch.eye(4, device=DEVICE).unsqueeze(0).unsqueeze(0).expand(1, N, N, 4, 4).clone()

        batch = {
            'spatial_features': spatial_features,
            'record_len': record_len,
            'pairwise_t_matrix': pairwise_t,
        }

        # Warmup
        for _ in range(N_WARMUP):
            with torch.no_grad():
                try:
                    _ = model.backend(batch)
                except Exception as e:
                    print(f"  N={N}: backend failed ({e}), trying sensor.backend per-agent")
                    break

        # Timed runs: measure backbone (per-agent) + Where2Comm fusion separately
        import torch.nn.functional as F

        backbone_times = []
        fusion_times = []
        total_times = []

        for _ in range(N_REPEATS):
            # --- Backbone per agent ---
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            with torch.no_grad():
                bd = model.sensor.backbone({'spatial_features': spatial_features})
                bev2d = bd['spatial_features_2d']
                if model.sensor.shrink_flag:
                    bev2d = model.sensor.shrink_conv(bev2d)
                psm_all = model.cls_head(bev2d)
            torch.cuda.synchronize()
            t1 = time.perf_counter()
            backbone_times.append((t1 - t0) * 1000)

            # --- Where2Comm fusion ---
            H1 = int(model.sensor.nx[1].item()) // 2
            W1 = int(model.sensor.nx[0].item()) // 2
            psm_resized = psm_all
            if psm_all.shape[-2:] != (H1, W1):
                psm_resized = F.interpolate(psm_all, size=(H1, W1), mode='nearest')
            thres = torch.ones_like(psm_resized[:, :1]) * 0.5

            torch.cuda.synchronize()
            t2 = time.perf_counter()
            with torch.no_grad():
                if model.multi_scale:
                    fused, _, _ = model.fusion(
                        spatial_features, psm_resized, record_len, pairwise_t,
                        backbone=model.sensor.backbone,
                        heads=[model.shrink, model.cls_head, model.reg_head])
                else:
                    fused, _, _ = model.fusion(
                        spatial_features, psm_resized, thres, record_len, pairwise_t)
            torch.cuda.synchronize()
            t3 = time.perf_counter()
            fusion_times.append((t3 - t2) * 1000)
            total_times.append((t1 - t0 + t3 - t2) * 1000)

        times = total_times

        mean_ms = np.mean(times)
        std_ms = np.std(times)
        p95_ms = np.percentile(times, 95)
        results[N] = {'mean_ms': mean_ms, 'std_ms': std_ms, 'p95_ms': p95_ms}
        print(f"  N={N:3d}: {mean_ms:7.1f} ms (std={std_ms:.1f}, p95={p95_ms:.1f})")

    return results


# ====================================================================
#  2.  AB3DMOT Tracking Benchmark
# ====================================================================

def benchmark_ab3dmot(n_values):
    """
    Benchmark AB3DMOT tracking at varying detection counts.

    Generates synthetic 3D detections distributed in a realistic scene
    layout and times the tracker update.
    """
    print("\n=== AB3DMOT Tracking Benchmark ===")

    sys.path.insert(0, str(Path(".").resolve()))
    from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper

    results = {}
    for N in n_values:
        n_dets = int(DETS_PER_EGO * N)

        # Fresh tracker each N to avoid state accumulation
        times = []
        for rep in range(N_REPEATS):
            tracker = AB3DMOTWrapper({'min_hits': 3, 'max_age': 6})

            # Simulate a few frames to build up tracks, then time the last frame
            for frame in range(10):
                # Detections: [h, w, l, x, y, z, yaw, score]
                dets = np.zeros((n_dets, 8), dtype=np.float32)
                for d in range(n_dets):
                    angle = 2 * np.pi * d / n_dets
                    radius = 20 + 5 * np.random.randn()
                    dets[d, 0] = 1.5   # h
                    dets[d, 1] = 1.8   # w
                    dets[d, 2] = 4.5   # l
                    dets[d, 3] = radius * np.cos(angle) + 0.3 * frame  # x (moving)
                    dets[d, 4] = 0.5   # z (height)
                    dets[d, 5] = radius * np.sin(angle) + 0.1 * frame  # y
                    dets[d, 6] = angle  # yaw
                    dets[d, 7] = 0.8 + 0.2 * np.random.rand()  # score
                info = np.zeros((n_dets, 3), dtype=np.int64)

                dets_all = {'dets': dets, 'info': info}
                if frame < 9:
                    tracker.track(dets_all, frame)
                else:
                    t0 = time.perf_counter()
                    tracker.track(dets_all, frame)
                    t1 = time.perf_counter()
                    times.append((t1 - t0) * 1000)

        mean_ms = np.mean(times)
        std_ms = np.std(times)
        p95_ms = np.percentile(times, 95)
        n_tracks = int(TRACK_CONFIRM_RATIO * n_dets)
        results[N] = {
            'mean_ms': mean_ms, 'std_ms': std_ms, 'p95_ms': p95_ms,
            'n_dets': n_dets, 'n_tracks': n_tracks,
        }
        print(f"  N={N:3d} ({n_dets:3d} dets, ~{n_tracks:3d} tracks): "
              f"{mean_ms:7.2f} ms (std={std_ms:.2f}, p95={p95_ms:.2f})")

    return results


# ====================================================================
#  3.  SMART Prediction Benchmark
# ====================================================================

def _build_synthetic_trajectories(n_tracks, n_history=30):
    """Build synthetic ObstacleTrajectory dict mimicking AB3DMOT output.

    Each track has n_history transforms (at 20Hz) with smooth circular motion.
    """
    from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
    from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
    from ecav.ecav_carla import Location, Rotation, Transform

    trajectories = {}
    for i in range(n_tracks):
        angle0 = 2 * np.pi * i / n_tracks
        radius = 20 + 5 * (i % 3)
        speed = 8.0 + 2.0 * (i % 4)  # m/s

        # Build transform history (newest first in deque)
        traj = []
        for t in range(n_history):
            dt = t * 0.05  # 20Hz
            angle = angle0 + speed * dt / radius
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            yaw = np.degrees(angle + np.pi / 2)
            traj.append(Transform(
                location=Location(x=x, y=y, z=0.5),
                rotation=Rotation(roll=0, pitch=0, yaw=yaw)))

        # ObstacleVehicle needs 8 corners (8,3) for BoundingBox
        cx, cy, cz = radius * np.cos(angle0), radius * np.sin(angle0), 0.5
        hw, hl, hh = 1.0, 2.4, 0.75  # half width/length/height
        corners = np.array([
            [cx-hl, cy-hw, cz-hh], [cx+hl, cy-hw, cz-hh],
            [cx+hl, cy+hw, cz-hh], [cx-hl, cy+hw, cz-hh],
            [cx-hl, cy-hw, cz+hh], [cx+hl, cy-hw, cz+hh],
            [cx+hl, cy+hw, cz+hh], [cx-hl, cy+hw, cz+hh],
        ], dtype=np.float32)
        obs = ObstacleVehicle(corners, None)
        obs.kf_speed_mps = speed
        obs.carla_id = -1
        ot = ObstacleTrajectory(obs, traj)
        trajectories[1000 + i] = ot

    return trajectories


def benchmark_smart(n_values):
    """
    Benchmark SMART trajectory prediction at varying agent counts.

    Constructs synthetic ObstacleTrajectory dicts and runs real GPU inference.
    """
    print("\n=== SMART Prediction Benchmark ===")

    try:
        from ecav.core.prediction.smart_predictor_manager import SMARTPredictorManager3D

        # Find SMART checkpoint
        import glob
        ckpts = glob.glob("ecav/core/prediction/smart/checkpoints/*.ckpt")
        if not ckpts:
            ckpts = glob.glob("ecav/core/prediction/smart/**/*.ckpt", recursive=True)
        if not ckpts:
            raise FileNotFoundError("No SMART checkpoint found")

        map_cache = None  # Run without map context for benchmarking

        print(f"  Loading SMART from {ckpts[0]}")
        predictor = SMARTPredictorManager3D(
            checkpoint_path=ckpts[0],
            map_cache_path=map_cache,
            device=DEVICE,
            num_output_steps=25)
        print("  SMART model loaded")

    except Exception as e:
        raise RuntimeError(f"SMART model failed to load: {e}")

    results = {}
    for N in n_values:
        n_tracks = max(int(TRACK_CONFIRM_RATIO * DETS_PER_EGO * N), 1)
        trajs = _build_synthetic_trajectories(n_tracks, n_history=30)

        # Warmup
        for _ in range(N_WARMUP):
            predictor.generate_predicted_trajectories(trajs, source_tick=100, publish_tick=100)

        # Timed runs
        times = []
        for _ in range(N_REPEATS):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            predictor.generate_predicted_trajectories(trajs, source_tick=100, publish_tick=100)
            torch.cuda.synchronize()
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000)

        mean_ms = np.mean(times)
        std_ms = np.std(times)
        p95_ms = np.percentile(times, 95)
        results[N] = {
            'mean_ms': mean_ms, 'std_ms': std_ms, 'p95_ms': p95_ms,
            'n_tracks': n_tracks, 'source': 'measured',
        }
        print(f"  N={N:3d} ({n_tracks:3d} tracks): {mean_ms:7.2f} ms")

    return results



# ====================================================================
#  4.  SB-SPS V2V Network Model
# ====================================================================

def benchmark_v2v_network(n_values,
                          m_subchannels=20,
                          feature_bytes=200_000,
                          effective_rate_bps=6e6,
                          p_keep=0.4,
                          p_loss_base=0.02):
    """
    Analytical SB-SPS MAC model for C-V2X PC5 sidelink.

    Models subchannel collision probability and PRR at varying N.
    Also computes whether the payload fits in a single TTI.

    Parameters from 3GPP TS 36.321 and channel_engine_py.py.
    """
    print("\n=== V2V SB-SPS Network Model ===")

    results = {}
    for N in n_values:
        # PRR from uniform random subchannel selection with persistence
        # P(no collision on my subchannel) = ((M-1)/M)^(N-1)
        # With persistence: effective contenders = N * (1 - p_keep) + p_keep
        # Simplified: each TTI, ~N*(1-p_keep) vehicles reselect
        n_active = N  # all vehicles transmit every TTI
        p_no_collision = ((m_subchannels - 1) / m_subchannels) ** (n_active - 1)
        prr = p_no_collision * (1 - p_loss_base)

        # Payload feasibility: can feature_bytes fit in 1 TTI at effective_rate?
        tti_capacity_bytes = effective_rate_bps / 8.0 / 1000.0  # bytes per 1ms TTI
        ttis_needed = math.ceil(feature_bytes / tti_capacity_bytes)
        # If >1 TTI needed, PRR degrades (all TTIs must succeed)
        prr_payload = prr ** ttis_needed

        # Channel busy ratio
        cbr = 1.0 - ((m_subchannels - 1) / m_subchannels) ** n_active

        # Backhaul queue delay for edge comparison
        backhaul_ms = (N * feature_bytes * 8) / (100e6) * 1000 * 0.5  # 100 Mbps backhaul

        results[N] = {
            'prr_ideal': prr,
            'prr_payload': prr_payload,
            'cbr': cbr,
            'ttis_needed': ttis_needed,
            'backhaul_ms': backhaul_ms,
        }
        print(f"  N={N:3d}: PRR={prr:.3f} (payload-adj={prr_payload:.3f}), "
              f"CBR={cbr:.3f}, TTIs={ttis_needed}, backhaul={backhaul_ms:.1f}ms")

    return results


# ====================================================================
#  5.  Edge Backhaul + Compute Latency Model
# ====================================================================

def compute_edge_e2e(fusion_results, tracking_results, prediction_results,
                     n_values, radio_ms=15.0, backhaul_bw_mbps=100.0,
                     feature_bytes=200_000):
    """Compute total edge E2E latency: radio + backhaul + compute."""
    print("\n=== Edge E2E Latency Model ===")
    results = {}
    for N in n_values:
        # Network: radio uplink + backhaul queuing
        backhaul_bits = N * feature_bytes * 8
        backhaul_ms = (backhaul_bits / (backhaul_bw_mbps * 1e6)) * 1000
        network_ms = radio_ms + backhaul_ms

        # Compute: fusion + tracking + prediction (sequential)
        fusion_ms = fusion_results[N]['mean_ms']
        tracking_ms = tracking_results[N]['mean_ms']
        prediction_ms = prediction_results[N]['mean_ms']
        compute_ms = fusion_ms + tracking_ms + prediction_ms

        total_ms = network_ms + compute_ms
        results[N] = {
            'network_ms': network_ms,
            'radio_ms': radio_ms,
            'backhaul_ms': backhaul_ms,
            'fusion_ms': fusion_ms,
            'tracking_ms': tracking_ms,
            'prediction_ms': prediction_ms,
            'compute_ms': compute_ms,
            'total_ms': total_ms,
        }
        print(f"  N={N:3d}: net={network_ms:.1f} + fuse={fusion_ms:.1f} + "
              f"track={tracking_ms:.1f} + pred={prediction_ms:.1f} = "
              f"total={total_ms:.1f} ms")
    return results


# ====================================================================
#  6.  Plotting
# ====================================================================

def plot_results(edge_results, v2v_results, n_values):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    plt.rcParams.update({
        'font.size': 11,
        'font.family': 'serif',
        'figure.figsize': (7, 4.5),
        'axes.grid': True,
        'grid.alpha': 0.3,
    })

    Ns = np.array(n_values)

    # ── Fig 1: Edge per-stage latency (stacked area) ──────────────
    fig, ax = plt.subplots()
    radio = np.array([edge_results[n]['radio_ms'] for n in Ns])
    backhaul = np.array([edge_results[n]['backhaul_ms'] for n in Ns])
    fusion = np.array([edge_results[n]['fusion_ms'] for n in Ns])
    tracking = np.array([edge_results[n]['tracking_ms'] for n in Ns])
    prediction = np.array([edge_results[n]['prediction_ms'] for n in Ns])

    ax.stackplot(Ns, radio, backhaul, fusion, tracking, prediction,
                 labels=['Radio uplink', 'Backhaul queue', 'Fusion (Where2Comm)',
                         'Tracking (AB3DMOT)', 'Prediction (SMART)'],
                 colors=['#4e79a7', '#59a14f', '#e15759', '#f28e2b', '#b07aa1'],
                 alpha=0.85)
    ax.axhline(y=100, color='k', linestyle='--', linewidth=1.5, label='100 ms deadline')
    ax.set_xlabel('Number of cooperative agents (N)')
    ax.set_ylabel('End-to-end latency (ms)')
    ax.set_title('Edge Pipeline: Per-Stage Latency vs Agent Count')
    ax.legend(loc='upper left', fontsize=9)
    ax.set_xlim(Ns[0], Ns[-1])
    ax.set_ylim(0)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    fig.tight_layout()
    fig.savefig(OUT_DIR / 'fig_edge_stage_latency.pdf', dpi=150)
    fig.savefig(OUT_DIR / 'fig_edge_stage_latency.png', dpi=150)
    plt.close(fig)
    print(f"  Saved {OUT_DIR / 'fig_edge_stage_latency.pdf'}")

    # ── Fig 2: V2V PRR vs N ──────────────────────────────────────
    fig, ax = plt.subplots()
    prr_ideal = np.array([v2v_results[n]['prr_ideal'] for n in Ns])
    prr_payload = np.array([v2v_results[n]['prr_payload'] for n in Ns])
    cbr = np.array([v2v_results[n]['cbr'] for n in Ns])

    ax.plot(Ns, prr_ideal, 'o-', color='#4e79a7', label='PRR (single-TTI ideal)', linewidth=2)
    ax.plot(Ns, prr_payload, 's-', color='#e15759', label='PRR (200 KB payload)', linewidth=2)
    ax.axhline(y=0.9, color='gray', linestyle=':', label='90% reliability target')
    ax.set_xlabel('Number of V2V agents (N)')
    ax.set_ylabel('Packet Reception Ratio')
    ax.set_title('C-V2X PC5 Sidelink: PRR Collapse with Intermediate Fusion')
    ax.legend(fontsize=9)
    ax.set_xlim(Ns[0], Ns[-1])
    ax.set_ylim(0, 1.05)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))

    ax2 = ax.twinx()
    ax2.bar(Ns, cbr, width=2, alpha=0.2, color='orange', label='Channel Busy Ratio')
    ax2.set_ylabel('Channel Busy Ratio', color='orange')
    ax2.set_ylim(0, 1.05)
    ax2.tick_params(axis='y', labelcolor='orange')

    fig.tight_layout()
    fig.savefig(OUT_DIR / 'fig_v2v_prr_vs_n.pdf', dpi=150)
    fig.savefig(OUT_DIR / 'fig_v2v_prr_vs_n.png', dpi=150)
    plt.close(fig)
    print(f"  Saved {OUT_DIR / 'fig_v2v_prr_vs_n.pdf'}")

    # ── Fig 3: Edge vs V2V E2E comparison ────────────────────────
    fig, ax = plt.subplots()
    edge_total = np.array([edge_results[n]['total_ms'] for n in Ns])
    # V2V E2E: assume 10ms radio + 5ms fusion (local) + tracking + prediction
    # but multiply by (1/PRR) for expected retransmission delay
    v2v_compute = np.array([
        10 + 5 +  # local radio + fusion
        edge_results[n]['tracking_ms'] +
        edge_results[n]['prediction_ms']
        for n in Ns
    ])
    # V2V effective: if PRR < 1, features don't arrive, AoI grows
    v2v_effective = v2v_compute / np.maximum(prr_payload, 0.01)

    ax.plot(Ns, edge_total, 'o-', color='#4e79a7', label='Edge (compute-limited)', linewidth=2)
    ax.plot(Ns, v2v_effective, 's-', color='#e15759', label='V2V (comm-limited)', linewidth=2)
    ax.axhline(y=100, color='k', linestyle='--', linewidth=1.5, label='100 ms deadline')
    ax.set_xlabel('Number of cooperative agents (N)')
    ax.set_ylabel('Effective E2E latency (ms)')
    ax.set_title('Edge vs V2V: Where Does Each Architecture Fail?')
    ax.legend(fontsize=9)
    ax.set_xlim(Ns[0], Ns[-1])
    ax.set_ylim(0, min(500, max(edge_total.max(), v2v_effective.max()) * 1.1))
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    fig.tight_layout()
    fig.savefig(OUT_DIR / 'fig_edge_vs_v2v_e2e.pdf', dpi=150)
    fig.savefig(OUT_DIR / 'fig_edge_vs_v2v_e2e.png', dpi=150)
    plt.close(fig)
    print(f"  Saved {OUT_DIR / 'fig_edge_vs_v2v_e2e.pdf'}")

    # ── Fig 4: Detection/track count scaling ─────────────────────
    fig, ax = plt.subplots()
    n_dets = DETS_PER_EGO * Ns
    n_tracks = TRACK_CONFIRM_RATIO * n_dets
    ax.plot(Ns, n_dets, 'o-', color='#f28e2b', label='Detections / frame', linewidth=2)
    ax.plot(Ns, n_tracks, 's-', color='#b07aa1', label='Confirmed tracks / frame', linewidth=2)
    ax.set_xlabel('Number of cooperative agents (N)')
    ax.set_ylabel('Object count per frame')
    ax.set_title('Scene Complexity vs Agent Count')
    ax.legend(fontsize=9)
    ax.set_xlim(Ns[0], Ns[-1])
    ax.set_ylim(0)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    fig.tight_layout()
    fig.savefig(OUT_DIR / 'fig_scene_complexity.pdf', dpi=150)
    fig.savefig(OUT_DIR / 'fig_scene_complexity.png', dpi=150)
    plt.close(fig)
    print(f"  Saved {OUT_DIR / 'fig_scene_complexity.pdf'}")


# ====================================================================
#  7.  Save CSV
# ====================================================================

def save_csv(edge_results, v2v_results, fusion_results, tracking_results,
             prediction_results, n_values):
    csv_path = OUT_DIR / 'benchmark_timing.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'N', 'n_dets', 'n_tracks',
            'fusion_mean_ms', 'fusion_p95_ms',
            'tracking_mean_ms', 'tracking_p95_ms',
            'prediction_mean_ms', 'prediction_source',
            'edge_compute_ms', 'edge_network_ms', 'edge_total_ms',
            'v2v_prr_ideal', 'v2v_prr_payload', 'v2v_cbr',
        ])
        for N in n_values:
            w.writerow([
                N,
                int(DETS_PER_EGO * N),
                int(TRACK_CONFIRM_RATIO * DETS_PER_EGO * N),
                f"{fusion_results[N]['mean_ms']:.2f}",
                f"{fusion_results[N]['p95_ms']:.2f}",
                f"{tracking_results[N]['mean_ms']:.2f}",
                f"{tracking_results[N]['p95_ms']:.2f}",
                f"{prediction_results[N]['mean_ms']:.2f}",
                prediction_results[N].get('source', 'measured'),
                f"{edge_results[N]['compute_ms']:.2f}",
                f"{edge_results[N]['network_ms']:.2f}",
                f"{edge_results[N]['total_ms']:.2f}",
                f"{v2v_results[N]['prr_ideal']:.4f}",
                f"{v2v_results[N]['prr_payload']:.4f}",
                f"{v2v_results[N]['cbr']:.4f}",
            ])
    print(f"\n  Saved {csv_path}")


# ====================================================================
#  Main
# ====================================================================

def main():
    parser = argparse.ArgumentParser(description="Edge pipeline scaling benchmark")
    parser.add_argument('--model-dir', type=str, default=None,
                        help='WorldFusion checkpoint directory')
    parser.add_argument('--skip-fusion', action='store_true',
                        help='Skip WorldFusion GPU benchmark (use analytical)')
    args = parser.parse_args()

    print("=" * 60)
    print("Edge Pipeline Scaling Benchmark")
    print(f"Device: {DEVICE}")
    if DEVICE != "cpu":
        print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"N sweep: {N_AGENTS}")
    print("=" * 60)

    # Run benchmarks
    if args.skip_fusion:
        print("\n  [Skipping WorldFusion GPU benchmark, using analytical]")
        fusion_results = {}
        for N in N_AGENTS:
            # Analytical: backbone is O(N), Where2Comm attention is O(N^2)
            # Calibrated to ~15ms at N=1 (backbone) + O(N^2) attention
            backbone_ms = 12.0 * N
            attention_ms = 0.3 * N * N
            total = backbone_ms + attention_ms
            fusion_results[N] = {'mean_ms': total, 'std_ms': 0, 'p95_ms': total * 1.1}
            print(f"  N={N:3d}: {total:.1f} ms [analytical]")
    else:
        fusion_results = benchmark_worldfusion(N_AGENTS, args.model_dir)

    tracking_results = benchmark_ab3dmot(N_AGENTS)
    prediction_results = benchmark_smart(N_AGENTS)
    v2v_results = benchmark_v2v_network(N_AGENTS)

    # Compute E2E
    edge_results = compute_edge_e2e(
        fusion_results, tracking_results, prediction_results, N_AGENTS)

    # Save and plot
    save_csv(edge_results, v2v_results, fusion_results, tracking_results,
             prediction_results, N_AGENTS)
    plot_results(edge_results, v2v_results, N_AGENTS)

    print("\n" + "=" * 60)
    print("Benchmark complete. Figures in paper2_figures/")
    print("=" * 60)


if __name__ == '__main__':
    main()
