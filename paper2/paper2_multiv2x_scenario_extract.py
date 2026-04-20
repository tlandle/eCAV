#!/usr/bin/env python3
"""Extract Multi-V2X dataset scenes into eCAV scenario YAML configs.

Reads a Multi-V2X town directory, extracts spawn positions and trajectories
from per-frame yaml files, identifies frames with dense cooperation, and
generates an eCAV-compatible scenario YAML for closed-loop CARLA evaluation.

Multi-V2X layout:
  {data_root}/{town}/data_protocal.yaml
  {data_root}/{town}/cav_{id}/{timestamp}.yaml
  {data_root}/{town}/rsu_{id}/{timestamp}.yaml

Each frame yaml has: lidar_pose [x,y,z,roll,yaw,pitch], ego_speed [speed, 0.0],
conn_agents [list of agent ids], objects dict.

Usage:
    python paper2_multiv2x_scenario_extract.py \
        --data-root /data1/Datasets/Multi-V2X \
        --town Town10HD__2023_11_15_00_20_38 \
        --out-dir paper2_figures/multiv2x_scenarios/
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import yaml


# ---- Multi-V2X reading utilities ----------------------------------------

def read_data_protocal(town_dir: Path) -> dict:
    """Read data_protocal.yaml for map name and RSU locations."""
    p = town_dir / "data_protocal.yaml"
    if not p.exists():
        # Fall back: infer map name from directory name
        town_name = town_dir.name.split("__")[0]
        return {"map_name": town_name, "traffic_lights": []}
    with open(p) as f:
        data = yaml.safe_load(f) or {}
    map_name = data.get("world", {}).get("map_name", town_dir.name.split("__")[0])
    traffic_lights = data.get("traffic_lights", [])
    return {"map_name": map_name, "traffic_lights": traffic_lights}


def discover_agents(town_dir: Path):
    """Find all cav_* and rsu_* directories."""
    cavs = sorted([d.name for d in town_dir.iterdir()
                   if d.is_dir() and d.name.startswith("cav_")])
    rsus = sorted([d.name for d in town_dir.iterdir()
                   if d.is_dir() and d.name.startswith("rsu_")])
    return cavs, rsus


def frame_yamls(agent_dir: Path):
    """Return sorted list of frame yaml paths (exclude *_view.yaml)."""
    out = []
    for p in agent_dir.glob("*.yaml"):
        if p.stem.endswith("_view"):
            continue
        if not p.stem.isdigit():
            continue
        out.append(p)
    out.sort(key=lambda x: int(x.stem))
    return out


def read_frame_yaml(path: Path) -> dict:
    """Read a single frame yaml file."""
    with open(path) as f:
        return yaml.safe_load(f) or {}


def extract_pose(frame_data: dict):
    """Extract [x, y, z, roll, yaw, pitch] from lidar_pose field."""
    pose = frame_data.get("lidar_pose")
    if pose is None:
        return None
    # lidar_pose is a list of 6 floats
    return [float(v) for v in pose]


def extract_speed(frame_data: dict) -> float:
    """Extract ego speed from ego_speed field."""
    sp = frame_data.get("ego_speed")
    if sp is None:
        return 0.0
    if isinstance(sp, (list, tuple)):
        return float(sp[0])
    return float(sp)


def extract_conn_agents(frame_data: dict) -> list:
    """Extract connected agent ids from conn_agents field."""
    ca = frame_data.get("conn_agents")
    if ca is None:
        return []
    return list(ca)


# ---- Core extraction logic -----------------------------------------------

def extract_town_data(town_dir: Path, max_frames: int = 0):
    """Extract all agent poses and connection stats from a town.

    Args:
        town_dir: Path to the town directory.
        max_frames: If > 0, limit to first N frames per agent.

    Returns:
        Dictionary with per-agent trajectories and per-frame conn stats.
    """
    protocal = read_data_protocal(town_dir)
    cav_names, rsu_names = discover_agents(town_dir)

    agents = {}  # name -> {type, first_pose, trajectory, agent_id}
    frame_conn_stats = defaultdict(dict)  # timestamp -> {agent_name: n_conn}

    for agent_list, agent_type in [(cav_names, "cav"), (rsu_names, "rsu")]:
        for name in agent_list:
            agent_dir = town_dir / name
            yamls = frame_yamls(agent_dir)
            if max_frames > 0:
                yamls = yamls[:max_frames]
            if not yamls:
                continue

            trajectory = []
            first_pose = None

            for yp in yamls:
                ts = yp.stem
                fd = read_frame_yaml(yp)
                pose = extract_pose(fd)
                speed = extract_speed(fd)
                conn = extract_conn_agents(fd)

                if pose is not None:
                    if first_pose is None:
                        first_pose = pose
                    trajectory.append({"timestamp": ts, "pose": pose, "speed": speed})

                frame_conn_stats[ts][name] = len(conn)

            agent_id = int(name.split("_", 1)[1])
            agents[name] = {
                "type": agent_type,
                "agent_id": agent_id,
                "first_pose": first_pose,
                "n_frames": len(trajectory),
                "trajectory": trajectory,
            }

    return {
        "map_name": protocal["map_name"],
        "traffic_lights": protocal["traffic_lights"],
        "agents": agents,
        "frame_conn_stats": frame_conn_stats,
    }


def find_interesting_windows(frame_conn_stats: dict, window_size: int = 10,
                              top_k: int = 5):
    """Find frame windows with highest total connectivity.

    Returns list of (start_ts, end_ts, total_connections, max_single_frame).
    """
    sorted_ts = sorted(frame_conn_stats.keys(), key=lambda x: int(x))
    if len(sorted_ts) < window_size:
        window_size = len(sorted_ts)

    # Per-frame total connection count
    per_frame_total = []
    for ts in sorted_ts:
        total = sum(frame_conn_stats[ts].values())
        per_frame_total.append((ts, total))

    if not per_frame_total:
        return []

    # Sliding window
    windows = []
    for i in range(len(per_frame_total) - window_size + 1):
        chunk = per_frame_total[i:i + window_size]
        total = sum(c for _, c in chunk)
        max_single = max(c for _, c in chunk)
        windows.append((chunk[0][0], chunk[-1][0], total, max_single))

    windows.sort(key=lambda w: w[2], reverse=True)
    return windows[:top_k]


def compute_frame_level_stats(frame_conn_stats: dict) -> dict:
    """Compute summary statistics over frame connection data."""
    sorted_ts = sorted(frame_conn_stats.keys(), key=lambda x: int(x))
    per_frame_totals = []
    per_frame_max_agent = []

    for ts in sorted_ts:
        counts = frame_conn_stats[ts]
        total = sum(counts.values())
        max_agent = max(counts.values()) if counts else 0
        per_frame_totals.append(total)
        per_frame_max_agent.append(max_agent)

    n = len(per_frame_totals)
    if n == 0:
        return {"n_frames": 0}

    return {
        "n_frames": n,
        "first_timestamp": sorted_ts[0],
        "last_timestamp": sorted_ts[-1],
        "total_connections_mean": sum(per_frame_totals) / n,
        "total_connections_max": max(per_frame_totals),
        "max_agent_connections_mean": sum(per_frame_max_agent) / n,
        "max_agent_connections_max": max(per_frame_max_agent),
    }


# ---- eCAV YAML generation ------------------------------------------------

def generate_ecav_yaml(town_data: dict, scenario_name: str,
                       max_vehicles: int = 10) -> str:
    """Generate an eCAV scenario YAML config from extracted Multi-V2X data.

    Selects up to max_vehicles CAVs (those with most frames / longest
    trajectories) and all RSUs. Spawn positions come from each agent's
    first-frame lidar_pose.
    """
    map_name = town_data["map_name"]
    agents = town_data["agents"]

    # Separate CAVs and RSUs
    cavs = {k: v for k, v in agents.items() if v["type"] == "cav"}
    rsus = {k: v for k, v in agents.items() if v["type"] == "rsu"}

    # Select CAVs: pick those with most frames, then by agent_id for stability
    cav_list = sorted(cavs.values(), key=lambda c: (-c["n_frames"], c["agent_id"]))
    selected_cavs = cav_list[:max_vehicles]

    # For each selected CAV, use last pose as destination
    vehicle_entries = []
    for i, cav in enumerate(selected_cavs, 1):
        sp = cav["first_pose"]  # [x, y, z, roll, yaw, pitch]
        traj = cav["trajectory"]
        last_pose = traj[-1]["pose"] if traj else sp
        vehicle_entries.append({
            "name": f"cav{i}",
            "multiv2x_id": cav["agent_id"],
            "spawn_position": [round(sp[0], 1), round(sp[1], 1), round(sp[2], 1)],
            "spawn_yaw": round(sp[4], 1),  # yaw is index 4
            "destination": [round(last_pose[0], 1), round(last_pose[1], 1), round(last_pose[2], 1)],
        })

    rsu_entries = []
    for i, (name, rsu) in enumerate(sorted(rsus.items()), 1):
        sp = rsu["first_pose"]
        if sp is None:
            continue
        rsu_entries.append({
            "name": f"rsu{i}",
            "multiv2x_id": rsu["agent_id"],
            "spawn_position": [round(sp[0], 1), round(sp[1], 1), round(sp[2], 1)],
        })

    # Build the YAML string matching the eCAV config format
    town_lower = map_name.lower()  # e.g. "town10hd"

    lines = []
    lines.append(f"description: |-")
    lines.append(f"  Auto-generated from Multi-V2X dataset scenario.")
    lines.append(f"  Source town: {town_data.get('source_town', scenario_name)}")
    lines.append(f"  {len(selected_cavs)} CAVs, {len(rsu_entries)} RSUs extracted.")
    lines.append(f"")
    lines.append(f"scenario_name: {scenario_name}")
    lines.append(f"")
    lines.append(f"distributed: false")
    lines.append(f"define: &perception_is_active true")
    lines.append(f"perception_active: *perception_is_active")
    lines.append(f"")
    lines.append(f"ml_manager:")
    lines.append(f"  yolo_endpoint: 'http://localhost:18000'")
    lines.append(f"")
    lines.append(f"world:")
    lines.append(f"  sync_mode: true")
    lines.append(f"  client_host: &host localhost")
    lines.append(f"  client_port: &port 2000")
    lines.append(f"  fixed_delta_seconds: 0.05")
    lines.append(f"  seed: 11")
    lines.append(f"  weather:")
    lines.append(f"    sun_altitude_angle: 15")
    lines.append(f"    cloudiness: 75")
    lines.append(f"    precipitation: 0")
    lines.append(f"    precipitation_deposits: 0")
    lines.append(f"    wind_intensity: 0")
    lines.append(f"    fog_density: 0")
    lines.append(f"    fog_distance: 0")
    lines.append(f"    fog_falloff: 0")
    lines.append(f"    wetness: 0")
    lines.append(f"")
    lines.append(f"carla_traffic_manager:")
    lines.append(f"  sync_mode: true")
    lines.append(f"  global_distance: 5.0")
    lines.append(f"  global_speed_perc: -50")
    lines.append(f"  set_osm_mode: true")
    lines.append(f"  auto_lane_change: false")
    lines.append(f"  random: false")
    lines.append(f"  ignore_lights_percentage: 0")
    lines.append(f"  vehicle_list:")
    lines.append(f"  range: []")
    lines.append(f"")
    lines.append(f"ego_vehicle_max_speed: 43")
    lines.append(f"")
    lines.append(f"scenario_runner:")
    lines.append(f"  town: {town_lower}")
    lines.append(f"  scenario: multiv2x_replay")
    lines.append(f"  num_actors: {len(selected_cavs)}")
    lines.append(f"  host: *host")
    lines.append(f"  port: *port")
    lines.append(f"  timeout: 10")
    lines.append(f"  debug: false")
    lines.append(f"  sync: false")
    lines.append(f"  repetitions: 1")
    lines.append(f"  agent: null")
    lines.append(f"  openscenario: null")
    lines.append(f"  route: null")
    lines.append(f"  reloadWorld: false")
    lines.append(f"  waitForEgo: false")
    lines.append(f"  trafficManagerPort: '8000'")
    lines.append(f"  trafficManagerSeed: '0'")
    lines.append(f"  record: ''")
    lines.append(f"  agentConfig: ''")
    lines.append(f"  file: false")
    lines.append(f"  json: false")
    lines.append(f"  junit: false")
    lines.append(f"  list: false")
    lines.append(f"  openscenarioparams:")
    lines.append(f"    - ego_vehicle_max_speed=43")
    lines.append(f"  output: false")
    lines.append(f"  outputDir: ''")
    lines.append(f"  randomize: false")
    lines.append(f"  vehicle_index: 0")
    lines.append(f"")
    lines.append(f"# Vehicle and RSU base configs use YAML anchors (see openscenario_3_edge_late_fusion.yaml)")
    lines.append(f"# Omitted here for brevity. Import via YAML merge or copy from base config.")
    lines.append(f"")
    lines.append(f"vehicle_base: &vehicle_base")
    lines.append(f"  sensing:")
    lines.append(f"    perception:")
    lines.append(f"      activate: true")
    lines.append(f"      camera:")
    lines.append(f"        visualize: 0")
    lines.append(f"        num: 4")
    lines.append(f"        image_width: 1920")
    lines.append(f"        image_height: 1440")
    lines.append(f"        positions:")
    lines.append(f"          - [1.5, 0.0, 1.7, 0]")
    lines.append(f"          - [0.0, -1.5, 1.7, -90]")
    lines.append(f"          - [0.0, 1.5, 1.7, 90]")
    lines.append(f"          - [-1.5, 0.0, 1.7, 180]")
    lines.append(f"      lidar:")
    lines.append(f"        visualize: false")
    lines.append(f"        channels: 64")
    lines.append(f"        range: 50")
    lines.append(f"        points_per_second: 1150000")
    lines.append(f"        rotation_frequency: 20")
    lines.append(f"        upper_fov: 10.0")
    lines.append(f"        lower_fov: -30.0")
    lines.append(f"        dropoff_general_rate: 0.0")
    lines.append(f"        dropoff_intensity_limit: 1.0")
    lines.append(f"        dropoff_zero_intensity: 0.0")
    lines.append(f"        noise_stddev: 0.0")
    lines.append(f"    localization:")
    lines.append(f"      activate: true")
    lines.append(f"      dt: ${{world.fixed_delta_seconds}}")
    lines.append(f"      gnss:")
    lines.append(f"        noise_alt_stddev: 0.5")
    lines.append(f"        noise_lat_stddev: 1.5e-6")
    lines.append(f"        noise_lon_stddev: 1.5e-6")
    lines.append(f"        heading_direction_stddev: 1.0")
    lines.append(f"        speed_stddev: 0.5")
    lines.append(f"  behavior:")
    lines.append(f"    max_speed: 43")
    lines.append(f"    tailgate_speed: 121")
    lines.append(f"    speed_lim_dist: 3")
    lines.append(f"    speed_decrease: 15")
    lines.append(f"    safety_time: 4")
    lines.append(f"    emergency_param: 0.4")
    lines.append(f"    ignore_traffic_light: true")
    lines.append(f"    overtake_allowed: true")
    lines.append(f"    collision_time_ahead: 2")
    lines.append(f"    overtake_counter_recover: 35")
    lines.append(f"    sample_resolution: 4.5")
    lines.append(f"    local_planner:")
    lines.append(f"      buffer_size: 12")
    lines.append(f"      trajectory_update_freq: 15")
    lines.append(f"      waypoint_update_freq: 9")
    lines.append(f"      min_dist: 3")
    lines.append(f"      trajectory_dt: 0.20")
    lines.append(f"      debug: false")
    lines.append(f"      debug_trajectory: false")
    lines.append(f"  controller:")
    lines.append(f"    type: pid_controller")
    lines.append(f"    args:")
    lines.append(f"      lat:")
    lines.append(f"        k_p: 0.75")
    lines.append(f"        k_d: 0.02")
    lines.append(f"        k_i: 0.4")
    lines.append(f"      lon:")
    lines.append(f"        k_p: 0.37")
    lines.append(f"        k_d: 0.024")
    lines.append(f"        k_i: 0.032")
    lines.append(f"      dynamic: false")
    lines.append(f"      dt: ${{world.fixed_delta_seconds}}")
    lines.append(f"      max_brake: 1.0")
    lines.append(f"      max_throttle: 1.0")
    lines.append(f"      max_steering: 0.3")
    lines.append(f"  v2x:")
    lines.append(f"    enabled: true")
    lines.append(f"    communication_range: 70  # Multi-V2X uses 70m range")
    lines.append(f"    loc_noise: 0.0")
    lines.append(f"    yaw_noise: 0.0")
    lines.append(f"    speed_noise: 0.0")
    lines.append(f"    lag: 0")
    lines.append(f"")
    lines.append(f"rsu_base: &rsu_base")
    lines.append(f"  sensing:")
    lines.append(f"    perception:")
    lines.append(f"      activate: true")
    lines.append(f"      camera:")
    lines.append(f"        visualize: 0")
    lines.append(f"        num: 4")
    lines.append(f"        image_width: 1920")
    lines.append(f"        image_height: 1440")
    lines.append(f"        positions:")
    lines.append(f"          - [2.5, 0, 1.0, 0]")
    lines.append(f"          - [0.0, 0.3, 1.8, 100]")
    lines.append(f"          - [0.0, -0.3, 1.8, -100]")
    lines.append(f"          - [-2.0, 0.0, 1.5, 180]")
    lines.append(f"      lidar:")
    lines.append(f"        visualize: false")
    lines.append(f"        channels: 32")
    lines.append(f"        range: 120")
    lines.append(f"        points_per_second: 1000000")
    lines.append(f"        rotation_frequency: 20")
    lines.append(f"        upper_fov: 10.0")
    lines.append(f"        lower_fov: -70.0")
    lines.append(f"        dropoff_general_rate: 0.0")
    lines.append(f"        dropoff_intensity_limit: 1.0")
    lines.append(f"        dropoff_zero_intensity: 0.0")
    lines.append(f"        noise_stddev: 0.0")
    lines.append(f"    localization:")
    lines.append(f"      activate: true")
    lines.append(f"      dt: ${{world.fixed_delta_seconds}}")
    lines.append(f"      gnss:")
    lines.append(f"        noise_alt_stddev: 0.05")
    lines.append(f"        noise_lat_stddev: 3e-6")
    lines.append(f"        noise_lon_stddev: 3e-6")
    lines.append(f"")
    lines.append(f"ecloud:")
    lines.append(f"  num_servers: 2")
    lines.append(f"  server_ping_time_s: 0.005")
    lines.append(f"  client_world_time_factor: 0.9")
    lines.append(f"  client_ping_spawn_s: 0.05")
    lines.append(f"  client_ping_tick_s: 0.005")
    lines.append(f"")
    lines.append(f"edge_base: &edge_base")
    lines.append(f"  max_capacity: {len(selected_cavs)}")
    lines.append(f"  inter_gap: 0.6")
    lines.append(f"  open_gap: 1.2")
    lines.append(f"  warm_up_speed: 55")
    lines.append(f"  change_leader_speed: false")
    lines.append(f"  leader_speeds_profile: [85, 95]")
    lines.append(f"  stage_duration: 10")
    lines.append(f"  target_speed: 100")
    lines.append(f"  edge_dt: 0.2")
    lines.append(f"  search_dt: 2")
    lines.append(f"  latency: 0.0")
    lines.append(f"  jitter_std: 0.0")
    lines.append(f"  latency_distribution: hybrid")
    lines.append(f"  edge_sets_destination: false")
    lines.append(f"  uplink_packet_loss_pct: 0.0")
    lines.append(f"  downlink_packet_loss_pct: 0.0")
    lines.append(f"  mode: PREDICTION")
    lines.append(f"")
    lines.append(f"scenario:")
    lines.append(f"  edge_list:")
    lines.append(f"    - <<: *edge_base")
    lines.append(f"      manager_type: late_fusion")
    lines.append(f"      predictor_type: smart")
    lines.append(f"      smart_predictor:")
    lines.append(f"        checkpoint: ecav/core/prediction/smart/checkpoints/epoch=105.ckpt")
    lines.append(f"        map_cache: null")
    lines.append(f"        device: cuda")
    lines.append(f"")

    # RSUs
    lines.append(f"      rsus:")
    if rsu_entries:
        for rsu in rsu_entries:
            sp = rsu["spawn_position"]
            lines.append(f"        - <<: *rsu_base")
            lines.append(f"          name: {rsu['name']}")
            lines.append(f"          spawn_position: [{sp[0]}, {sp[1]}, {sp[2]}]")
            lines.append(f"          id: -1")
            lines.append(f"          # Multi-V2X agent_id: {rsu['multiv2x_id']}")
    else:
        lines.append(f"        []")

    lines.append(f"")

    # Vehicles
    lines.append(f"      vehicles:")
    for veh in vehicle_entries:
        sp = veh["spawn_position"]
        dst = veh["destination"]
        lines.append(f"        - <<: *vehicle_base")
        lines.append(f"          name: {veh['name']}")
        lines.append(f"          destination: [{dst[0]}, {dst[1]}, {dst[2]}]")
        lines.append(f"          spawn_position: [{sp[0]}, {sp[1]}, {sp[2]}]")
        lines.append(f"          spawn_yaw: {veh['spawn_yaw']}")
        lines.append(f"          # Multi-V2X agent_id: {veh['multiv2x_id']}")

    return "\n".join(lines) + "\n"


def build_summary(town_name: str, town_data: dict,
                  interesting_windows: list) -> dict:
    """Build the summary JSON for one town extraction."""
    agents = town_data["agents"]
    cavs = {k: v for k, v in agents.items() if v["type"] == "cav"}
    rsus = {k: v for k, v in agents.items() if v["type"] == "rsu"}

    cav_spawns = {}
    for name, cav in sorted(cavs.items()):
        if cav["first_pose"]:
            cav_spawns[name] = {
                "agent_id": cav["agent_id"],
                "position": [round(v, 2) for v in cav["first_pose"][:3]],
                "yaw": round(cav["first_pose"][4], 2),
            }

    rsu_spawns = {}
    for name, rsu in sorted(rsus.items()):
        if rsu["first_pose"]:
            rsu_spawns[name] = {
                "agent_id": rsu["agent_id"],
                "position": [round(v, 2) for v in rsu["first_pose"][:3]],
            }

    frame_stats = compute_frame_level_stats(town_data["frame_conn_stats"])

    windows_out = []
    for start_ts, end_ts, total, max_single in interesting_windows:
        windows_out.append({
            "start_frame": start_ts,
            "end_frame": end_ts,
            "total_connections": total,
            "max_single_frame_connections": max_single,
        })

    return {
        "town": town_name,
        "map_name": town_data["map_name"],
        "n_cavs": len(cavs),
        "n_rsus": len(rsus),
        "cav_spawn_positions": cav_spawns,
        "rsu_spawn_positions": rsu_spawns,
        "interesting_windows": windows_out,
        "frame_connection_statistics": frame_stats,
    }


# ---- Main ----------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Extract Multi-V2X scenes to eCAV scenario configs.")
    parser.add_argument("--data-root", required=True,
                        help="Multi-V2X dataset root directory")
    parser.add_argument("--town", required=True,
                        help="Town directory name (e.g. Town10HD__2023_11_15_00_20_38)")
    parser.add_argument("--out-dir", default="paper2_figures/multiv2x_scenarios/",
                        help="Output directory for generated files")
    parser.add_argument("--max-vehicles", type=int, default=10,
                        help="Maximum number of CAVs to include in eCAV config")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="Limit frames read per agent (0 = all)")
    parser.add_argument("--window-size", type=int, default=10,
                        help="Frame window size for interesting window detection")
    parser.add_argument("--top-k", type=int, default=5,
                        help="Number of top interesting windows to report")
    args = parser.parse_args()

    data_root = Path(args.data_root)
    town_dir = data_root / args.town
    if not town_dir.is_dir():
        print(f"ERROR: {town_dir} is not a directory")
        sys.exit(1)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Extract the town
    print(f"Reading Multi-V2X town: {args.town}")
    town_data = extract_town_data(town_dir, max_frames=args.max_frames)
    town_data["source_town"] = args.town

    n_cavs = sum(1 for a in town_data["agents"].values() if a["type"] == "cav")
    n_rsus = sum(1 for a in town_data["agents"].values() if a["type"] == "rsu")
    n_frames = len(town_data["frame_conn_stats"])
    print(f"  Map: {town_data['map_name']}, CAVs: {n_cavs}, RSUs: {n_rsus}, Frames: {n_frames}")

    # Find interesting windows
    interesting = find_interesting_windows(
        town_data["frame_conn_stats"],
        window_size=args.window_size,
        top_k=args.top_k)

    if interesting:
        print(f"  Top {len(interesting)} cooperation windows (window_size={args.window_size}):")
        for start_ts, end_ts, total, max_single in interesting:
            print(f"    frames {start_ts}-{end_ts}: total_conn={total}, max_single={max_single}")

    # Generate eCAV scenario YAML
    map_name = town_data["map_name"]
    scenario_name = f"multiv2x_{map_name.lower()}"
    yaml_content = generate_ecav_yaml(town_data, scenario_name,
                                      max_vehicles=args.max_vehicles)
    yaml_path = out_dir / f"{scenario_name}.yaml"
    with open(yaml_path, "w") as f:
        f.write(yaml_content)
    print(f"  Wrote eCAV config: {yaml_path}")

    # Generate summary JSON
    summary = build_summary(args.town, town_data, interesting)
    json_path = out_dir / f"{scenario_name}_summary.json"
    with open(json_path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"  Wrote summary: {json_path}")

    # Also write per-CAV trajectory files for route replay
    traj_dir = out_dir / f"{scenario_name}_trajectories"
    traj_dir.mkdir(parents=True, exist_ok=True)
    for name, agent in sorted(town_data["agents"].items()):
        if agent["type"] != "cav":
            continue
        traj_path = traj_dir / f"{name}_trajectory.json"
        traj_data = {
            "agent_id": agent["agent_id"],
            "n_frames": agent["n_frames"],
            "waypoints": [
                {"timestamp": wp["timestamp"],
                 "x": round(wp["pose"][0], 3),
                 "y": round(wp["pose"][1], 3),
                 "z": round(wp["pose"][2], 3),
                 "yaw": round(wp["pose"][4], 3),
                 "speed": round(wp["speed"], 3)}
                for wp in agent["trajectory"]
            ],
        }
        with open(traj_path, "w") as f:
            json.dump(traj_data, f, indent=2)

    print(f"  Wrote {n_cavs} CAV trajectories to {traj_dir}/")
    print("Done.")


if __name__ == "__main__":
    main()
