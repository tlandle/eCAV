#!/usr/bin/env python3
"""
generate_multi_ego_scenario.py
------------------------------
Parametric multi-ego intersection scenario generator for Town03.

Generates YAML config + XML + visualization .py for N ego vehicles at the
Town03 intersection (-78, 128). Distributes egos across 4 approach roads,
creating NHTSA-classified conflict patterns with cascading knock-on effects.

NHTSA Pre-Crash Typologies generated:
  - LTAP/OD: Left Turn Across Path / Opposite Direction
  - SCP:     Straight Crossing Path (perpendicular)
  - Rear-End: Lead vehicle brakes → cascade to following egos

Usage:
    python scripts/generate_multi_ego_scenario.py --num-egos 2
    python scripts/generate_multi_ego_scenario.py --num-egos 4
    python scripts/generate_multi_ego_scenario.py --num-egos 8 --stagger 20
    python scripts/generate_multi_ego_scenario.py --num-egos 16 --stagger 15

Output:
    ecav/scenario_testing/config_yaml/openscenario_3_edge_late_fusion_{N}ego.yaml
    ecav/scenario_testing/scenarios/scenario_3_{N}ego.xml
    ecav/scenario_testing/scenarios/scenario_3_{N}ego.py
    ecav/scenario_testing/openscenario_3_edge_late_fusion_{N}ego.py
"""

import argparse
import math
import os
import sys
from pathlib import Path
from textwrap import dedent, indent

ROOT = Path(__file__).resolve().parent.parent
SCENARIO_DIR = ROOT / "ecav" / "scenario_testing"
CONFIG_DIR = SCENARIO_DIR / "config_yaml"
SCENES_DIR = SCENARIO_DIR / "scenarios"

# ── Town03 intersection geometry (validated from existing scenarios) ──────

INTERSECTION_CENTER = (-78.0, 128.0)

# Each approach: (base_x, base_y, yaw, dx_stagger, dy_stagger, lane_offsets)
# dx/dy_stagger: direction to place following vehicles (away from intersection)
# lane_offsets: list of (dx, dy) for parallel lanes
APPROACHES = {
    "south": {
        "label": "South approach (northbound)",
        "base": (-84.8, 80.0),
        "yaw": 90,
        "stagger_dir": (0, -1),       # stagger south (away from intersection)
        "lanes": [(0, 0), (3.8, 0)],  # lane 1 at x=-84.8, lane 2 at x=-81.0
        "default_intention": "left_turn",
        "destination_left": (-28.8, 134.7, 0.5),   # east after left turn
        "destination_straight": (-85.1, 155.0, 0.5), # continue north (road 69, validated on-road)
    },
    "west": {
        "label": "West approach (eastbound)",
        "base": (-130.0, 134.7),
        "yaw": 0,
        "stagger_dir": (-1, 0),       # stagger west (away from intersection)
        "lanes": [(0, 0), (0, -3.5)], # lane 1 at y=134.7, lane 2 at y=131.2
        "default_intention": "straight",
        "destination_straight": (-28.8, 134.7, 0.5),  # continue east
        "destination_right": (-84.8, 60.0, 0.5),      # right turn south
    },
    "east": {
        "label": "East approach (westbound)",
        "base": (-15.0, 127.7),       # moved east from -30 to avoid Lincoln at (-35, 127.7)
        "yaw": 180,
        "stagger_dir": (1, 0),        # stagger east (away from intersection)
        "lanes": [(0, 0), (0, 3.5)],  # lane 1 at y=127.7, lane 2 at y=131.2
        "default_intention": "straight",
        "destination_straight": (-150.0, 127.7, 0.5),  # continue west
        "destination_left": (-84.8, 60.0, 0.5),        # left turn south
    },
    "north": {
        "label": "North approach (southbound)",
        "base": (-77.5, 175.0),
        "yaw": 270,
        "stagger_dir": (0, 1),        # stagger north (away from intersection)
        "lanes": [(0, 0), (-3.8, 0)], # lane 1 at x=-77.5, lane 2 at x=-81.3
        "default_intention": "straight",
        "destination_straight": (-77.5, 50.0, 0.5),    # continue south
        "destination_right": (-28.8, 134.7, 0.5),      # right turn east
    },
}

# Round-robin approach order for distributing egos
APPROACH_ORDER = ["south", "west", "east", "north"]


def distribute_egos(n_egos, stagger_m=20.0, scenario_type="ltap"):
    """Distribute N egos across approaches. Returns list of ego dicts.

    Distribution strategy:
      - Round-robin across approaches
      - Within each approach, first ego in lane 1, second in lane 2,
        third back to lane 1 but staggered further back, etc.
      - scenario_type="ltap": First ego on south approach gets left_turn
        (the primary NHTSA LTAP scenario)
      - scenario_type="scp": All egos go straight (NHTSA SCP scenario)
    """
    # Count egos per approach
    approach_counts = {a: 0 for a in APPROACH_ORDER}
    approach_assignment = []
    for i in range(n_egos):
        approach = APPROACH_ORDER[i % len(APPROACH_ORDER)]
        approach_counts[approach] += 1
        approach_assignment.append(approach)

    # Generate spawn positions
    egos = []
    approach_idx = {a: 0 for a in APPROACH_ORDER}  # current index per approach

    for i in range(n_egos):
        approach_name = approach_assignment[i]
        approach = APPROACHES[approach_name]
        idx = approach_idx[approach_name]
        approach_idx[approach_name] += 1

        # Lane selection: alternate between available lanes
        lanes = approach["lanes"]
        lane_idx = idx % len(lanes)
        lane_offset = lanes[lane_idx]

        # Stagger: how far back from base position
        stagger_rank = idx // len(lanes)  # 0 for first in each lane, 1 for second, etc.
        dx_s, dy_s = approach["stagger_dir"]
        stagger_offset = stagger_rank * stagger_m

        # Compute spawn position
        bx, by = approach["base"]
        lx, ly = lane_offset
        x = bx + lx + dx_s * stagger_offset
        y = by + ly + dy_s * stagger_offset
        yaw = approach["yaw"]

        # Intention: ltap mode → first ego on south turns left; scp → all straight
        if scenario_type == "ltap" and approach_name == "south" and idx == 0:
            intention = "left_turn"
            dest = approach["destination_left"]
        elif "destination_straight" in approach:
            intention = "straight"
            dest = approach["destination_straight"]
        else:
            intention = approach.get("default_intention", "straight")
            dest = approach.get("destination_straight", approach.get("destination_left"))

        # NHTSA classification
        nhtsa_types = []
        if intention == "left_turn":
            nhtsa_types.append("LTAP/OD")
        # Will be crossing paths with perpendicular approaches
        if approach_name in ("south", "north"):
            nhtsa_types.append("SCP-with-EW")
        else:
            nhtsa_types.append("SCP-with-NS")
        if stagger_rank > 0 or (idx > 0 and lane_idx == 0):
            nhtsa_types.append("Rear-End-Chain")

        egos.append({
            "index": i,
            "name": f"cav{i+1}",
            "approach": approach_name,
            "approach_label": approach["label"],
            "lane": lane_idx + 1,
            "stagger_rank": stagger_rank,
            "x": round(x, 1),
            "y": round(y, 1),
            "z": 1.0,
            "yaw": yaw,
            "intention": intention,
            "destination": dest,
            "nhtsa": nhtsa_types,
            "is_hero": (i == 0),  # first ego is ScenarioRunner hero
        })

    return egos


def nhtsa_conflict_matrix(egos):
    """Generate pairwise NHTSA conflict types between egos."""
    conflicts = []
    for i, e1 in enumerate(egos):
        for j, e2 in enumerate(egos):
            if j <= i:
                continue
            # Determine conflict type
            if e1["approach"] == e2["approach"]:
                ctype = "Rear-End (same approach)"
            elif e1["intention"] == "left_turn" and e2["approach"] in ("west", "east"):
                ctype = "LTAP/OD (left turn across path)"
            elif e2["intention"] == "left_turn" and e1["approach"] in ("west", "east"):
                ctype = "LTAP/OD (left turn across path)"
            elif (e1["approach"] in ("south", "north") and e2["approach"] in ("east", "west")) or \
                 (e1["approach"] in ("east", "west") and e2["approach"] in ("south", "north")):
                ctype = "SCP (straight crossing path)"
            elif e1["approach"] in ("south", "north") and e2["approach"] in ("south", "north"):
                ctype = "Opposite Direction"
            else:
                ctype = "Opposite Direction"
            conflicts.append((e1["name"], e2["name"], ctype))
    return conflicts


def generate_xml(egos, n_egos):
    """Generate ScenarioRunner XML with ego 1 as hero, others as other_actors."""
    hero = egos[0]
    lines = [
        '<?xml version="1.0"?>',
        f'<!-- Multi-ego ({n_egos} vehicles) variant of Scenario 3 -->',
        f'<!-- Generated by generate_multi_ego_scenario.py -->',
        '<scenarios>',
        f'    <scenario name="Scenario_3_{n_egos}Ego" type="Scenario_3" town="Town03">',
        f'        <!-- Ego 1 (hero): {hero["approach_label"]}, {hero["intention"]} -->',
        f'        <ego_vehicle x="{hero["x"]}" y="{hero["y"]}" z="{hero["z"]}" '
        f'yaw="{hero["yaw"]}" model="vehicle.nissan.patrol" />',
        '',
    ]

    # Additional egos as other_actors (for visualization; spawned by VehicleManager at runtime)
    for ego in egos[1:]:
        lines.append(
            f'        <!-- Ego {ego["index"]+1}: {ego["approach_label"]}, {ego["intention"]} '
            f'(spawned by VehicleManager) -->')
        lines.append(
            f'        <other_actor x="{ego["x"]}" y="{ego["y"]}" z="{ego["z"]}" '
            f'yaw="{ego["yaw"]}" model="vehicle.lincoln.mkz_2017" />')

    # Lincoln NPC
    lines.extend([
        '',
        '        <!-- Lincoln: crossing east-to-west through intersection (NPC) -->',
        '        <other_actor x="-35.0" y="127.7" z="-500" yaw="180" model="vehicle.lincoln.mkz_2017" />',
        '',
        '        <!-- Occlusion vehicles (stationary) -->',
        '        <other_actor x="-81.0" y="119.7" z="-500" yaw="90" model="vehicle.dodge.charger_2020" />',
        '        <other_actor x="-81.0" y="110.7" z="-500" yaw="90" model="vehicle.lincoln.mkz_2020" />',
        '        <other_actor x="-81.0" y="101.7" z="-500" yaw="90" model="vehicle.nissan.patrol_2021" />',
        '        <other_actor x="-81.0" y="92.7" z="-500" yaw="90" model="vehicle.jeep.wrangler_rubicon" />',
        '        <other_actor x="-81.0" y="83.7" z="-500" yaw="90" model="vehicle.dodge.charger_police_2020" />',
        '',
        '        <weather cloudiness="0" precipitation="75" precipitation_deposits="0" '
        'wind_intensity="0" sun_azimuth_angle="0" sun_altitude_angle="75" fog_density="0" />',
        '    </scenario>',
        '</scenarios>',
    ])
    return '\n'.join(lines) + '\n'


def generate_viz_py(egos, n_egos):
    """Generate .py file for show_spawn_points.py to parse velocities."""
    lines = [
        '#!/usr/bin/env python',
        '"""',
        f'Scenario 3 with {n_egos} ego vehicles — visualization metadata.',
        'Used by show_spawn_points.py for path visualization.',
        '"""',
        '',
        'import carla',
        '',
        f'class Scenario_3_{n_egos}Ego_Viz:',
        f'    """Metadata for show_spawn_points.py."""',
        '',
        '    def __init__(self):',
    ]

    # Velocities for additional egos (actor indices start at 0 for first other_actor)
    for i, ego in enumerate(egos[1:]):
        vel = 70  # all egos at 70 km/h
        lines.append(f'        self.vehicle_{i+1:02d}_velocity = {vel}  '
                     f'# {ego["name"]}: {ego["approach_label"]}')

    # Lincoln velocity
    lincoln_idx = len(egos)  # after all ego other_actors
    lines.append(f'        self.vehicle_{lincoln_idx:02d}_velocity = 25  # Lincoln NPC')

    # Occlusion vehicles (stationary)
    for j in range(5):
        idx = lincoln_idx + 1 + j
        lines.append(f'        self.vehicle_{idx:02d}_velocity = 0  # Occlusion vehicle')

    # Lincoln waypoint plan
    lines.extend([
        '',
        '        # Lincoln waypoint plan',
        f'        if i == {lincoln_idx - 1}:',
        '            waypoint = [carla.Location(x=-108.6, y=129.5, z=0.5), '
        'carla.Location(x=-120.6, y=129.5, z=0.5), '
        'carla.Location(x=-140.6, y=115.2, z=0.5), '
        'carla.Location(x=-142.0, y=87.6, z=0.5)]',
    ])
    return '\n'.join(lines) + '\n'


def generate_vehicle_yaml_block(ego, is_first):
    """Generate YAML for one vehicle entry."""
    comment = (f"# ── Ego {ego['index']+1}: {ego['approach_label']}, "
               f"{ego['intention']} ──")
    if is_first:
        comment += "\n        # ScenarioRunner hero (no spawn_position)"
    else:
        comment += "\n        # Self-spawned by VehicleManager from spawn_position"

    nhtsa_str = ", ".join(ego["nhtsa"])
    comment += f"\n        # NHTSA: {nhtsa_str}"

    spawn_line = ""
    if not is_first:
        spawn_line = (f"\n          spawn_position: [{ego['x']}, {ego['y']}, {ego['z']}, "
                      f"0, {ego['yaw']}, 0]  # [x, y, z, roll, yaw, pitch]")

    dest = ego["destination"]
    viz = 4 if is_first else 0  # only visualize first ego's cameras

    return dedent(f"""\
        {comment}
        - <<: *vehicle_base
          name: {ego['name']}{spawn_line}
          destination: [{dest[0]}, {dest[1]}, {dest[2]}]
          sensing:
            <<: *base_sensing
            perception:
              <<: *base_perception
              camera:
                visualize: {viz}
                num: 4
                positions:
                  - [ 1.5,  0.0, 1.7,   0]
                  - [ 0.0, -1.5, 1.7, -90]
                  - [ 0.0,  1.5, 1.7,  90]
                  - [-1.5,  0.0, 1.7, 180]
                backend: default
              localization:
                <<: *base_localize
                debug_helper:
                  <<: *loc_debug_helper
                  show_animation: false
          v2x:
            <<: *base_v2x
            enabled: false
            communication_range: 35
          behavior:
            <<: *base_behavior
            local_planner:
              <<: *base_local_planner
              debug_trajectory: false
              debug: false
""")


def generate_yaml(egos, n_egos, prefix="openscenario_3_edge_late_fusion", no_rsu=False):
    """Generate the full YAML config."""
    # Build vehicle blocks
    vehicle_blocks = []
    for i, ego in enumerate(egos):
        vehicle_blocks.append(generate_vehicle_yaml_block(ego, is_first=(i == 0)))

    vehicles_yaml = "\n".join(vehicle_blocks)

    scenario_name = f"{prefix}_{n_egos}ego"

    # Read template from existing single-ego config and replace vehicles section
    yaml_content = dedent(f"""\
description: |-
  Multi-ego ({n_egos} vehicles) variant of {prefix}.
  Generated by generate_multi_ego_scenario.py.
  Egos approach from {len(set(e['approach'] for e in egos))} directions with cascading conflict patterns.

scenario_name: {scenario_name}

distributed: false
define: &perception_is_active true
perception_active: *perception_is_active

ml_manager:
  yolo_endpoint: 'http://localhost:18000'

world:
  sync_mode: true
  client_host: &host localhost
  client_port: &port 2000
  fixed_delta_seconds: 0.05
  seed: 11
  weather:
    sun_altitude_angle: 15
    cloudiness: 75
    precipitation: 0
    precipitation_deposits: 0
    wind_intensity: 0
    fog_density: 0
    fog_distance: 0
    fog_falloff: 0
    wetness: 0

carla_traffic_manager:
  sync_mode: true
  global_distance: 5.0
  global_speed_perc: -50
  set_osm_mode: true
  auto_lane_change: false
  random: false
  ignore_lights_percentage: 0
  vehicle_list:
  range: []

ego_vehicle_max_speed: 70

scenario_runner:
  town: town03
  scenario: Scenario_3
  num_actors: 2
  configFile: ecav/scenario_testing/scenarios/scenario_3.xml
  additionalScenario: ecav/scenario_testing/scenarios/scenario_3.py
  host: *host
  port: *port
  timeout: 10
  debug: false
  sync: false
  repetitions: 1
  agent: null
  openscenario: null
  route: null
  reloadWorld: false
  waitForEgo: false
  trafficManagerPort: '8000'
  trafficManagerSeed: '0'
  record: ''
  agentConfig: ''
  file: false
  json: false
  junit: false
  list: false
  openscenarioparams:
    - ego_vehicle_max_speed=70
  output: false
  outputDir: ''
  randomize: false
  vehicle_index: 0

vehicle_base: &vehicle_base
  sensing: &base_sensing
    perception: &base_perception
      activate: true
      camera:
        visualize: 4
        num: 4
        positions:
          - [ 1.5,  0.0, 1.7,   0]
          - [ 0.0, -1.5, 1.7, -90]
          - [ 0.0,  1.5, 1.7,  90]
          - [-1.5,  0.0, 1.7, 180]
      lidar:
        visualize: false
        channels: 32
        range: 50
        points_per_second: 100000
        rotation_frequency: 20
        upper_fov: 10.0
        lower_fov: -30.0
        dropoff_general_rate: 0.0
        dropoff_intensity_limit: 1.0
        dropoff_zero_intensity: 0.0
        noise_stddev: 0.0
    localization: &base_localize
      activate: true
      dt: ${{world.fixed_delta_seconds}}
      gnss:
        noise_alt_stddev: 0.5
        noise_lat_stddev: 1.5e-6
        noise_lon_stddev: 1.5e-6
        heading_direction_stddev: 1.0
        speed_stddev: 0.5
      debug_helper: &loc_debug_helper
        show_animation: false
        x_scale: 1.0
        y_scale: 1.0
  behavior: &base_behavior
    max_speed: 70
    tailgate_speed: 121
    speed_lim_dist: 3
    speed_decrease: 15
    safety_time: 4
    emergency_param: .4
    ignore_traffic_light: true
    overtake_allowed: true
    collision_time_ahead: 2
    overtake_counter_recover: 35
    sample_resolution: 4.5
    local_planner: &base_local_planner
      buffer_size: 12
      trajectory_update_freq: 15
      waypoint_update_freq: 9
      min_dist: 3
      trajectory_dt: 0.20
      debug: true
      debug_trajectory: true
  controller: &base_controller
    type: pid_controller
    args: &control_args
      lat:
        k_p: 0.75
        k_d: 0.02
        k_i: 0.4
      lon:
        k_p: 0.37
        k_d: 0.024
        k_i: 0.032
      dynamic: false
      dt: ${{world.fixed_delta_seconds}}
      max_brake: 1.0
      max_throttle: 1.0
      max_steering: 0.3
  v2x: &base_v2x
    enabled: true
    communication_range: 35
    loc_noise: 0.0
    yaw_noise: 0.0
    speed_noise: 0.0
    lag: 0
  map_manager:
    pixels_per_meter: 2
    raster_size: [224, 224]
    lane_sample_resolution: 0.1
    visualize: false
    activate: true
  safety_manager:
    print_message: true
    collision_sensor:
      history_size: 30
      col_thresh: 1
    stuck_dector:
      len_thresh: 500
      speed_thresh: 0.5
    offroad_dector: [ ]
    traffic_light_detector:
      light_dist_thresh: 20

rsu_base: &rsu_base
  sensing:
    perception:
      activate: true
      camera:
        visualize: 4
        num: 4
        positions:
          - [2.5, 0, 1.0, 0]
          - [0.0, 0.3, 1.8, 100]
          - [0.0, -0.3, 1.8, -100]
          - [-2.0, 0.0, 1.5, 180]
      lidar:
        visualize: false
        channels: 32
        range: 120
        points_per_second: 1000000
        rotation_frequency: 20
        upper_fov: 10.0
        lower_fov: -30.0
        dropoff_general_rate: 0.0
        dropoff_intensity_limit: 1.0
        dropoff_zero_intensity: 0.0
        noise_stddev: 0.0
    localization:
      activate: true
      dt: ${{world.fixed_delta_seconds}}
      gnss:
        noise_alt_stddev: 0.05
        noise_lat_stddev: 3e-6
        noise_lon_stddev: 3e-6

ecloud:
  num_servers: 2
  server_ping_time_s: 0.005
  client_world_time_factor: 0.9
  client_ping_spawn_s: 0.05
  client_ping_tick_s: 0.005

edge_base: &edge_base
  max_capacity: {max(n_egos * 2, 10)}
  inter_gap: 0.6
  open_gap: 1.2
  warm_up_speed: 55
  change_leader_speed: true
  leader_speeds_profile: [ 85, 95 ]
  stage_duration: 10
  target_speed: 100
  edge_dt: 0.2
  search_dt: 2
  latency: 0.0
  jitter_std: 0.0
  latency_distribution: hybrid
  edge_sets_destination: false
  uplink_packet_loss_pct: 0.0
  downlink_packet_loss_pct: 0.0
  mode: PREDICTION

scenario:
  edge_list:
    - <<: *edge_base
      manager_type: late_fusion

      rsus:{'  []' if no_rsu else """
        - <<: *rsu_base
          name: rsu1
          spawn_position: [-82.0, 126.0, 7.0]
          sensing:
            <<: *base_sensing
            perception:
              <<: *base_perception
              backend: default
          id: -1"""}

      vehicles:
{indent(vehicles_yaml, '        ')}
""")
    return yaml_content


def generate_run_scenario(n_egos):
    """Generate the run_scenario Python script."""
    return dedent(f"""\
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

\"\"\"
Multi-ego ({n_egos} vehicles) variant of openscenario_3_edge_late_fusion.
Generated by generate_multi_ego_scenario.py.

Ego 1 is the ScenarioRunner hero. Egos 2-{n_egos} are self-spawned by
VehicleManager from their spawn_position config.
\"\"\"

import time
from multiprocessing import Process
import asyncio

import carla
import scenario_runner.scenario_runner as sr
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import \\
    EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import add_current_time

import ecloud_pb2 as ecloud

MAX_STEP = 600
SCENARIO_NAME = 'openscenario_3_edge_late_fusion_{n_egos}ego'
scenario_runner = None


def exec_scenario_runner(scenario_params):
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_vehicle(opt, scenario_params):
    assert opt.distributed
    try:
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()
    except Exception as e:
        print(f"vehicle_index: {{scenario_params.scenario_runner.vehicle_index}}")
        raise e


def run_scenario(opt, scenario_params):
    global scenario_runner
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0

    try:
        scenario_params = add_current_time(scenario_params)

        cav_world = CavWorld(
            apply_ml=opt.apply_ml,
            config=scenario_params,
            litserve=getattr(opt, 'litserve', False)
        )

        scenario_manager = sim_api.ScenarioManager(
            scenario_params,
            opt.apply_ml,
            opt.version,
            town=scenario_params.scenario_runner.town,
            cav_world=cav_world,
            distributed=opt.distributed
        )

        if opt.distributed:
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
        else:
            sr_process = Process(target=exec_scenario_runner,
                                 args=(scenario_params,))
            sr_process.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero' and ego_vehicle is None:
                    print("Ego 1 (hero) found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {{num_actors}} actors')

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert edge_dt % world_dt == 0

        try:
            edge_list = scenario_manager.create_edge_manager_from_scenario_runner(
                application=['edge'],
                edge_dt=edge_dt,
                world_dt=world_dt,
                ego_vehicle=ego_vehicle,
                other_vehicles=[],
            )
        except Exception:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

        n_egos = len(edge_list[0].vehicle_manager_list) if edge_list else 0
        print(f"[MULTI-EGO] Edge managing {{n_egos}} ego vehicles")
        for i, vm in enumerate(edge_list[0].vehicle_manager_list):
            loc = vm.vehicle.get_transform().location
            print(f"  cav{{i+1}}: carla_id={{vm.vehicle.id}}, "
                  f"pos=({{loc.x:.1f}}, {{loc.y:.1f}}, {{loc.z:.1f}})")
            # Non-hero egos should not sys.exit(0) on destination arrival
            if i > 0 and hasattr(vm.agent, '_exit_on_destination'):
                vm.agent._exit_on_destination = False

        eval_manager = EvaluationManager(
            scenario_manager.cav_world,
            script_name=SCENARIO_NAME,
            scenario_params=scenario_params,
            current_time=scenario_params['current_time'],
            output_dir=opt.output_dir
        )

        spectator = ego_vehicle.get_world().get_spectator()
        spectator_altitude = 160
        spectator_bird_pitch = -90

        flag = True
        while flag:
            if opt.distributed:
                command = ecloud.Command.PULL_OBJECTS_AND_TICK if step > 0 else ecloud.Command.TICK
                flag = scenario_manager.broadcast_message(command)
                scenario_manager.tick_world()
            else:
                scenario_manager.tick()

            # Bird view centered on intersection
            view_transform = carla.Transform()
            view_transform.location = carla.Location(
                x=-78.0, y=128.0, z=spectator_altitude)
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            for edge in edge_list:
                serialized_predictions = edge.run_step(step)
                if opt.distributed and serialized_predictions is not None:
                    scenario_manager.push_edge_objects(serialized_predictions)

            step += 1
            if step >= MAX_STEP:
                print("Reached maximum step limit, exiting")
                break

            time.sleep(0.001)

    except SystemExit as e:
        print(f"Caught SystemExit({{e.code}}) — proceeding to evaluation/cleanup")

    except Exception as e:
        print(f"Caught {{type(e).__name__}}: {{e}} — proceeding to evaluation/cleanup")
        import traceback
        print(traceback.format_exc())

    finally:
        if sr_process is not None:
            sr_process.terminate()
            sr_process.join(timeout=5)

        if scenario_runner is not None:
            scenario_runner.destroy()

        for edge in edge_list:
            for i, vm in enumerate(edge.vehicle_manager_list):
                for vid, step_number in vm.vehicles_detected.items():
                    print(f"VID: {{vm.vehicle.id}} found VID {{vid}} at step {{step_number}}")

        if opt.distributed and scenario_manager is not None:
            scenario_manager.end()

        if eval_manager is not None:
            eval_manager.evaluate()

        if scenario_manager is not None:
            scenario_manager.close()
""")


def main():
    ap = argparse.ArgumentParser(
        description="Generate multi-ego intersection scenario for Town03")
    ap.add_argument("--num-egos", type=int, required=True,
                    help="Number of ego vehicles (2, 4, 8, 16)")
    ap.add_argument("--stagger", type=float, default=20.0,
                    help="Distance (m) between staggered vehicles in same lane (default: 20)")
    ap.add_argument("--scenario-type", choices=["ltap", "scp"], default="ltap",
                    help="Scenario type: ltap (left turn across path) or scp (straight crossing path)")
    ap.add_argument("--prefix", type=str, default=None,
                    help="Output filename prefix (default: auto from scenario-type)")
    ap.add_argument("--no-rsu", action="store_true",
                    help="Generate non-edge variant (no RSU)")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print scenario layout without writing files")
    args = ap.parse_args()

    n = args.num_egos
    scenario_type = args.scenario_type
    if args.prefix is None:
        if scenario_type == "scp":
            args.prefix = "scp_non_edge" if args.no_rsu else "scp_edge"
        else:
            args.prefix = "openscenario_3_edge_late_fusion"
    egos = distribute_egos(n, stagger_m=args.stagger, scenario_type=scenario_type)

    # Print scenario layout
    print(f"\n{'='*70}")
    print(f"  Multi-Ego Scenario: {n} ego vehicles at Town03 intersection")
    print(f"{'='*70}\n")

    by_approach = {}
    for ego in egos:
        by_approach.setdefault(ego["approach"], []).append(ego)

    for approach_name in APPROACH_ORDER:
        if approach_name not in by_approach:
            continue
        approach_egos = by_approach[approach_name]
        print(f"  {APPROACHES[approach_name]['label']}:")
        for ego in approach_egos:
            nhtsa = ", ".join(ego["nhtsa"])
            hero = " [HERO]" if ego["is_hero"] else ""
            print(f"    {ego['name']}{hero}: ({ego['x']}, {ego['y']}) "
                  f"yaw={ego['yaw']} lane={ego['lane']} "
                  f"{ego['intention']} → ({ego['destination'][0]}, {ego['destination'][1]})")
            print(f"      NHTSA: {nhtsa}")
        print()

    # Print conflict matrix
    conflicts = nhtsa_conflict_matrix(egos)
    print(f"  NHTSA Conflict Matrix ({len(conflicts)} pairwise conflicts):")
    for e1, e2, ctype in conflicts:
        print(f"    {e1} ↔ {e2}: {ctype}")
    print()

    # Print cascade chains
    print("  Knock-on Cascade Chains:")
    for approach_name in APPROACH_ORDER:
        if approach_name not in by_approach:
            continue
        chain = by_approach[approach_name]
        if len(chain) > 1:
            names = " → ".join(e["name"] for e in chain)
            print(f"    {APPROACHES[approach_name]['label']}: {names} (rear-end cascade)")
    print()

    if args.dry_run:
        print("  [DRY RUN] No files written.")
        return

    # Generate files
    prefix = args.prefix
    xml_path = SCENES_DIR / f"scenario_3_{n}ego.xml"
    py_path = SCENES_DIR / f"scenario_3_{n}ego.py"
    yaml_path = CONFIG_DIR / f"{prefix}_{n}ego.yaml"
    run_path = SCENARIO_DIR / f"{prefix}_{n}ego.py"

    xml_path.write_text(generate_xml(egos, n))
    py_path.write_text(generate_viz_py(egos, n))
    yaml_path.write_text(generate_yaml(egos, n, prefix=prefix, no_rsu=args.no_rsu))
    run_path.write_text(generate_run_scenario(n))

    print(f"  Generated files:")
    print(f"    XML:    {xml_path.relative_to(ROOT)}")
    print(f"    Viz:    {py_path.relative_to(ROOT)}")
    print(f"    YAML:   {yaml_path.relative_to(ROOT)}")
    print(f"    Runner: {run_path.relative_to(ROOT)}")
    print()
    print(f"  Visualize: python show_spawn_points.py scenario_3_{n}ego")
    print(f"  Run:       python test_runner.py -t openscenario_3_edge_late_fusion_{n}ego")
    print()


if __name__ == "__main__":
    main()
