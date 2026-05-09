#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Generate V2V scenario variants from existing edge late fusion scenarios.

Takes the openscenario_3_edge_late_fusion_{N}ego.yaml configs and produces
V2V variants for the communication knee characterization study:

  - v2v_late_fusion: ~5KB payloads (detected objects + tracks)
  - v2v_intermediate: ~200KB payloads (BM2CP BEV features)

The vehicle positions, perception configs, and hazard geometry are preserved.
Only the edge/fusion config block changes.

Usage:
    python ecav/scenario_testing/generate_v2v_scenarios.py
"""

import os
import sys
import copy
import yaml
from pathlib import Path

CONFIG_DIR = Path(__file__).parent / "config_yaml"

# Source scenarios (edge late fusion, various ego counts)
SOURCE_SCENARIOS = [
    "openscenario_3_edge_late_fusion",        # 1 ego
    "openscenario_3_edge_late_fusion_2ego",
    "openscenario_3_edge_late_fusion_4ego",
    "openscenario_3_edge_late_fusion_8ego",
    "openscenario_3_edge_late_fusion_16ego",
]

# V2V channel config shared by all V2V variants
V2V_CHANNEL_CONFIG = {
    "carrier_ghz": 5.9,
    "bandwidth_mhz": 40,
    "tx_power_dbm": 23,
    "sinr_threshold_db": 0.0,
    "antenna_h": 1.5,
}

V2V_MAC_CONFIG = {
    "M": 20,
    "RC_min": 5,
    "RC_max": 15,
    "p_keep": 0.0,
}

V2V_NETWORK_CONFIG = {
    "network_engine": "auto",  # tries ns-3, falls back to analytical
}

# BM2CP model paths (for intermediate fusion)
BM2CP_MODEL = {
    "hypes_yaml": "ecav/worldfusion/opencood/logs/v2xsim_bm2cp_ego_baseline_2026_01_02_20_01_42/config.yaml",
    "checkpoint": "ecav/worldfusion/opencood/logs/v2xsim_bm2cp_ego_baseline_2026_01_02_20_01_42/net_epoch7.pth",
}

# CoBEVT model paths (for CMP baseline)
COBEVT_MODEL = {
    "hypes_yaml": "ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/corpbevtlidar_delay_1_frame_aug_c256/config.yaml",
    "checkpoint": "ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/corpbevtlidar_delay_1_frame_aug_c256/net_epoch25.pth",
}

# Fusion level configs
FUSION_LEVELS = {
    "v2v_late_fusion": {
        "manager_type": "v2v_autocast",  # uses GT detections, shares object lists
        "payload_bytes": 5000,
        "description": "V2V late fusion (~5KB detected objects over NR V2X Mode 2 sidelink)",
        "extra_edge_config": {
            "bandwidth": {
                "rate_mbps": 6.0,
                "slot_ms": 100.0,
            },
            "feature_budget": {
                "confidence_threshold": 0.1,
                "max_bytes_per_tti": 5000,
            },
        },
    },
    "v2v_intermediate": {
        "manager_type": "v2v_coop",  # BM2CP intermediate fusion
        "payload_bytes": 200000,
        "description": "V2V intermediate fusion (~200KB BEV features over NR V2X Mode 2 sidelink). Expected to fail at N>=1 due to sidelink TB capacity.",
        "extra_edge_config": {
            "bm2cp_model": BM2CP_MODEL,
            "feature_budget": {
                "confidence_threshold": 0.1,
                "max_bytes_per_tti": 200000,
            },
        },
    },
    "v2v_cmp": {
        "manager_type": "cmp",  # CMP: CoBEVT perception + MTR prediction
        "payload_bytes": 32000,
        "description": "V2V CMP baseline (Wang et al., RA-L 2025). CoBEVT 256x compressed BEV features (~32KB) over NR V2X Mode 2 sidelink.",
        "extra_edge_config": {
            "cobevt_model": COBEVT_MODEL,
            "feature_budget": {
                "confidence_threshold": 0.1,
                "max_bytes_per_tti": 32000,
            },
        },
        "perception_backend": "cobevt",
        # OPV2V-matched LiDAR config (verified from OPV2V .pcd files:
        # 64 channels, 58K pts/scan, FOV [-25, 2], range 120m)
        "lidar_override": {
            "channels": 64,
            "range": 120,
            "points_per_second": 1300000,
            "rotation_frequency": 20,
            "upper_fov": 2.0,
            "lower_fov": -25.0,
            "dropoff_general_rate": 0.0,
            "dropoff_intensity_limit": 1.0,
            "dropoff_zero_intensity": 0.0,
            "noise_stddev": 0.0,
        },
    },
}


def load_yaml_raw(path):
    """Load YAML without resolving anchors (preserves structure)."""
    with open(path) as f:
        return yaml.safe_load(f)


def generate_v2v_variant(source_path, fusion_key, fusion_cfg):
    """Generate a V2V variant from an edge late fusion scenario."""
    cfg = load_yaml_raw(source_path)

    source_name = Path(source_path).stem

    # Determine ego count suffix
    if "16ego" in source_name:
        ego_suffix = "_16ego"
    elif "8ego" in source_name:
        ego_suffix = "_8ego"
    elif "4ego" in source_name:
        ego_suffix = "_4ego"
    elif "2ego" in source_name:
        ego_suffix = "_2ego"
    else:
        ego_suffix = ""

    # New scenario name
    new_name = f"openscenario_3_{fusion_key}{ego_suffix}"
    cfg["scenario_name"] = new_name
    cfg["description"] = (
        f"{fusion_cfg['description']}\n"
        f"  Generated from {source_name} by generate_v2v_scenarios.py.\n"
        f"  Same vehicle positions and perception as the edge variant."
    )

    # Modify the edge config block
    edge_list = cfg.get("scenario", {}).get("edge_list", [])
    if not edge_list:
        print(f"  WARNING: no edge_list in {source_name}, skipping")
        return None

    edge = edge_list[0]

    # Change manager type
    edge["manager_type"] = fusion_cfg["manager_type"]

    # Add V2V-specific config
    edge["v2v_channel"] = V2V_CHANNEL_CONFIG
    edge["mac"] = V2V_MAC_CONFIG
    edge["network"] = V2V_NETWORK_CONFIG
    edge["payload_bytes"] = fusion_cfg["payload_bytes"]

    # Remove edge-specific latency config (V2V is peer-to-peer)
    edge["latency"] = 0.0
    edge["jitter_std"] = 0.0
    edge["latency_distribution"] = "fixed"
    edge["uplink_packet_loss_pct"] = 0.0

    # Add tracker config
    edge.setdefault("tracker", "ab3dmot")
    edge.setdefault("tracker_cfg", {"min_hits": 3, "max_age": 6})

    # Add fusion-level-specific config
    for k, v in fusion_cfg.get("extra_edge_config", {}).items():
        edge[k] = v

    # Override LiDAR config if specified (to match training dataset)
    if "lidar_override" in fusion_cfg:
        lidar_cfg = fusion_cfg["lidar_override"]
        vehicles = edge.get("vehicles", [])
        for veh in vehicles:
            sensing = veh.get("sensing", {})
            if "perception" in sensing and "lidar" in sensing["perception"]:
                sensing["perception"]["lidar"].update(lidar_cfg)
            elif "lidar" in sensing:
                sensing["lidar"].update(lidar_cfg)

        rsus = edge.get("rsus", [])
        for rsu in rsus:
            sensing = rsu.get("sensing", {})
            if "perception" in sensing and "lidar" in sensing["perception"]:
                sensing["perception"]["lidar"].update(lidar_cfg)
            elif "lidar" in sensing:
                sensing["lidar"].update(lidar_cfg)

    # Set perception backend if specified (e.g., cobevt for CMP)
    if "perception_backend" in fusion_cfg:
        backend = fusion_cfg["perception_backend"]
        vehicles = edge.get("vehicles", [])
        for veh in vehicles:
            sensing = veh.get("sensing", {})
            percep = sensing.get("perception", {})
            percep["backend"] = backend
            if "cobevt_model" in fusion_cfg.get("extra_edge_config", {}):
                percep["cobevt_model"] = fusion_cfg["extra_edge_config"]["cobevt_model"]

        rsus = edge.get("rsus", [])
        for rsu in rsus:
            sensing = rsu.get("sensing", {})
            percep = sensing.get("perception", {})
            percep["backend"] = backend
            if "cobevt_model" in fusion_cfg.get("extra_edge_config", {}):
                percep["cobevt_model"] = fusion_cfg["extra_edge_config"]["cobevt_model"]

    # Write output
    out_path = CONFIG_DIR / f"{new_name}.yaml"
    with open(out_path, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False,
                  width=120)

    return out_path


def main():
    print("Generating V2V scenario variants...")
    print(f"Config dir: {CONFIG_DIR}")
    print()

    generated = []

    for fusion_key, fusion_cfg in FUSION_LEVELS.items():
        print(f"=== {fusion_key}: {fusion_cfg['description'][:60]}... ===")

        for source_name in SOURCE_SCENARIOS:
            source_path = CONFIG_DIR / f"{source_name}.yaml"
            if not source_path.exists():
                print(f"  SKIP: {source_name}.yaml not found")
                continue

            out_path = generate_v2v_variant(
                source_path, fusion_key, fusion_cfg)
            if out_path:
                print(f"  {out_path.name}")
                generated.append(out_path)

        print()

    print(f"Generated {len(generated)} scenario files.")
    print()
    print("To run a V2V late fusion scenario with 4 egos:")
    print("  python test_runner.py --scenario-yaml \\")
    print("    ecav/scenario_testing/config_yaml/openscenario_3_v2v_late_fusion_4ego.yaml")


if __name__ == "__main__":
    main()
