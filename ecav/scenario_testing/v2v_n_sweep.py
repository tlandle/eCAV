#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V N-sweep test runner for the communication knee characterization study.

Runs CARLA + ns-3 co-simulation at increasing N (number of cooperative
vehicles) to measure:
  - PRR vs N at each fusion level (late, early, intermediate)
  - Per-vehicle compute time vs N
  - Collision avoidance success rate vs N
  - E2E latency from feature extraction to prediction delivery

Produces a summary CSV for downstream figure generation (paper Figures 1-4).

Usage:
    python -m ecav.scenario_testing.v2v_n_sweep \\
        --fusion-level late \\
        --n-values 4 8 16 32 \\
        --output-dir evaluation_outputs/v2v_n_sweep/

    # Or run all fusion levels:
    python -m ecav.scenario_testing.v2v_n_sweep \\
        --fusion-level all \\
        --n-values 4 8 16 \\
        --output-dir evaluation_outputs/v2v_n_sweep/
"""

import argparse
import csv
import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional

logger = logging.getLogger("V2VNSweep")

# Mapping from fusion level to scenario config template
FUSION_CONFIGS = {
    "late": {
        "base_yaml": "openscenario_3_v2v_coop",
        "manager_type": "v2v_coop",
        "payload_bytes": 5000,
        "description": "Late fusion (detected objects + tracks, ~5KB)",
    },
    "early": {
        "base_yaml": "openscenario_3_v2v_autocast",
        "manager_type": "v2v_autocast",
        "payload_bytes": 50000,
        "description": "Early fusion / AutoCast (object point clouds, ~50KB)",
    },
    "intermediate": {
        "base_yaml": "openscenario_3_v2v_coop",
        "manager_type": "v2v_coop",
        "payload_bytes": 200000,
        "description": "Intermediate fusion / BM2CP (BEV features, ~200KB)",
    },
}


def generate_n_vehicle_yaml(base_yaml_path: str, n_vehicles: int,
                             fusion_level: str, output_dir: str,
                             network_engine: str = "auto") -> str:
    """
    Generate a scenario YAML for N cooperative vehicles.

    Takes the base V2V scenario YAML and modifies it to include
    N vehicles with perception sensors.

    Returns the path to the generated YAML.
    """
    import yaml

    with open(base_yaml_path) as f:
        # Use safe_load with a custom loader that handles anchors
        cfg = yaml.safe_load(f)

    fusion_cfg = FUSION_CONFIGS[fusion_level]

    # Update scenario name
    cfg["scenario_name"] = f"v2v_nsweep_{fusion_level}_N{n_vehicles}"

    # Add network engine config to edge_base
    edge = cfg.get("scenario", {}).get("edge_list", [{}])[0]
    edge.setdefault("network", {})["network_engine"] = network_engine
    edge["payload_bytes"] = fusion_cfg["payload_bytes"]

    # Generate N vehicle entries
    # Base vehicle config from the first vehicle in the template
    base_vehicles = edge.get("vehicles", [])
    base_vehicle = base_vehicles[0] if base_vehicles else {}

    vehicles = []
    for i in range(n_vehicles):
        veh = dict(base_vehicle)
        veh["name"] = f"cav{i+1}"
        vehicles.append(veh)

    edge["vehicles"] = vehicles

    # Update scenario runner for additional actors
    if "scenario_runner" in cfg:
        cfg["scenario_runner"]["num_actors"] = n_vehicles + 1  # +1 for threat

    # Write the generated YAML
    out_path = os.path.join(output_dir,
                            f"v2v_{fusion_level}_N{n_vehicles}.yaml")
    os.makedirs(output_dir, exist_ok=True)
    with open(out_path, 'w') as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)

    logger.info("Generated scenario YAML: %s (N=%d, fusion=%s)",
                out_path, n_vehicles, fusion_level)
    return out_path


def run_single_experiment(yaml_path: str, n_vehicles: int,
                          fusion_level: str, output_dir: str,
                          timeout_s: int = 600) -> Dict:
    """
    Run a single CARLA scenario and collect results.

    Returns a dict with metrics from the run.
    """
    run_output = os.path.join(output_dir, f"N{n_vehicles}_{fusion_level}")
    os.makedirs(run_output, exist_ok=True)

    result = {
        "n_vehicles": n_vehicles,
        "fusion_level": fusion_level,
        "payload_bytes": FUSION_CONFIGS[fusion_level]["payload_bytes"],
        "status": "pending",
    }

    logger.info("Running N=%d %s fusion...", n_vehicles, fusion_level)
    t0 = time.time()

    try:
        # Run the test runner
        cmd = [
            sys.executable, "test_runner.py",
            "--scenario-yaml", yaml_path,
            "--output-dir", run_output,
        ]
        proc = subprocess.run(
            cmd, capture_output=True, text=True,
            timeout=timeout_s, cwd=str(Path(__file__).parent.parent.parent))

        elapsed_s = time.time() - t0
        result["walltime_s"] = elapsed_s
        result["returncode"] = proc.returncode

        if proc.returncode == 0:
            result["status"] = "success"
            # Parse output metrics
            metrics = _parse_run_output(run_output)
            result.update(metrics)
        else:
            result["status"] = "failed"
            result["stderr"] = proc.stderr[-500:] if proc.stderr else ""
            logger.error("N=%d %s failed (rc=%d): %s",
                        n_vehicles, fusion_level, proc.returncode,
                        proc.stderr[-200:] if proc.stderr else "")

    except subprocess.TimeoutExpired:
        result["status"] = "timeout"
        result["walltime_s"] = timeout_s
        logger.error("N=%d %s timed out after %ds",
                    n_vehicles, fusion_level, timeout_s)

    except Exception as e:
        result["status"] = "error"
        result["error"] = str(e)
        logger.error("N=%d %s error: %s", n_vehicles, fusion_level, e)

    return result


def _parse_run_output(run_dir: str) -> Dict:
    """Parse metrics from a completed run's output directory."""
    metrics = {}

    # Look for simulation_metrics.json
    metrics_file = os.path.join(run_dir, "simulation_metrics.json")
    if os.path.exists(metrics_file):
        with open(metrics_file) as f:
            sim_metrics = json.load(f)
        metrics["sim_metrics"] = sim_metrics

        # Extract key values
        for key in ["mean_prr", "mean_cbr", "collision_avoidance_success",
                     "n_collisions", "n_ticks"]:
            if key in sim_metrics:
                metrics[key] = sim_metrics[key]

    # Look for V2V metrics
    v2v_file = os.path.join(run_dir, "v2v_metrics.json")
    if os.path.exists(v2v_file):
        with open(v2v_file) as f:
            v2v = json.load(f)
        metrics["v2v_metrics"] = v2v
        if "mean_prr" in v2v:
            metrics["mean_prr"] = v2v["mean_prr"]

    # Look for timing data
    timing_file = os.path.join(run_dir, "timing_log.json")
    if os.path.exists(timing_file):
        with open(timing_file) as f:
            timing = json.load(f)
        if timing:
            import numpy as np
            ch_ms = [t.get("channel_ms", 0) for t in timing]
            pipe_ms = [t.get("total_pipeline_ms", 0) for t in timing]
            metrics["channel_ms_mean"] = float(np.mean(ch_ms))
            metrics["channel_ms_p95"] = float(np.percentile(ch_ms, 95))
            metrics["pipeline_ms_mean"] = float(np.mean(pipe_ms))
            metrics["pipeline_ms_p95"] = float(np.percentile(pipe_ms, 95))

    return metrics


def run_sweep(fusion_levels: List[str], n_values: List[int],
              output_dir: str, network_engine: str = "auto",
              timeout_s: int = 600) -> List[Dict]:
    """
    Run the full N-sweep across fusion levels.

    Returns list of result dicts (one per (fusion_level, N) pair).
    """
    os.makedirs(output_dir, exist_ok=True)
    results = []

    yaml_dir = os.path.join(output_dir, "generated_yamls")
    config_dir = os.path.join(
        os.path.dirname(__file__), "config_yaml")

    for fusion in fusion_levels:
        cfg = FUSION_CONFIGS[fusion]
        base_yaml = os.path.join(config_dir, f"{cfg['base_yaml']}.yaml")

        if not os.path.exists(base_yaml):
            logger.error("Base YAML not found: %s", base_yaml)
            continue

        logger.info("=== %s: %s ===", fusion.upper(), cfg["description"])

        for n in sorted(n_values):
            # Generate scenario YAML for this N
            yaml_path = generate_n_vehicle_yaml(
                base_yaml, n, fusion, yaml_dir, network_engine)

            # Run experiment
            result = run_single_experiment(
                yaml_path, n, fusion, output_dir, timeout_s)
            results.append(result)

            # Write incremental results
            _write_summary_csv(results, output_dir)

    return results


def _write_summary_csv(results: List[Dict], output_dir: str):
    """Write sweep results to CSV."""
    csv_path = os.path.join(output_dir, "v2v_n_sweep_summary.csv")

    fieldnames = [
        "fusion_level", "n_vehicles", "payload_bytes", "status",
        "mean_prr", "channel_ms_mean", "pipeline_ms_mean",
        "collision_avoidance_success", "walltime_s",
    ]

    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames,
                                extrasaction='ignore')
        writer.writeheader()
        for r in results:
            writer.writerow(r)

    logger.info("Summary written to %s", csv_path)


def main():
    parser = argparse.ArgumentParser(
        description="V2V N-sweep characterization study")
    parser.add_argument("--fusion-level", type=str, default="late",
                        choices=["late", "early", "intermediate", "all"],
                        help="Fusion level to sweep (default: late)")
    parser.add_argument("--n-values", type=int, nargs="+",
                        default=[4, 8, 16, 32],
                        help="Vehicle counts to sweep (default: 4 8 16 32)")
    parser.add_argument("--output-dir", type=str,
                        default="evaluation_outputs/v2v_n_sweep/",
                        help="Output directory for results")
    parser.add_argument("--network-engine", type=str, default="auto",
                        choices=["auto", "ns3", "analytical"],
                        help="Network engine (default: auto)")
    parser.add_argument("--timeout", type=int, default=600,
                        help="Per-run timeout in seconds (default: 600)")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose logging")

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s")

    if args.fusion_level == "all":
        fusion_levels = list(FUSION_CONFIGS.keys())
    else:
        fusion_levels = [args.fusion_level]

    logger.info("V2V N-sweep characterization study")
    logger.info("Fusion levels: %s", fusion_levels)
    logger.info("N values: %s", args.n_values)
    logger.info("Network engine: %s", args.network_engine)
    logger.info("Output: %s", args.output_dir)

    results = run_sweep(
        fusion_levels, args.n_values, args.output_dir,
        args.network_engine, args.timeout)

    # Print summary
    print("\n" + "=" * 70)
    print("V2V N-SWEEP RESULTS")
    print("=" * 70)
    for r in results:
        prr = r.get("mean_prr", "N/A")
        prr_str = f"{prr:.3f}" if isinstance(prr, float) else str(prr)
        print(f"  {r['fusion_level']:>15s}  N={r['n_vehicles']:>3d}  "
              f"PRR={prr_str:>6s}  "
              f"status={r['status']}")
    print("=" * 70)

    # Write full results JSON
    json_path = os.path.join(args.output_dir, "v2v_n_sweep_full.json")
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f"Full results: {json_path}")


if __name__ == "__main__":
    main()
