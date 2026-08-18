# -*- coding: utf-8 -*-
"""Multi-V2X-format training-data generation, Town01 corridor.

Dumps per-agent PCD (intensity in the color channel) + per-frame GT yaml
in the OPV2V/Multi-V2X folder layout via DataDumper, from our deployment
geometry: RSU masts at z=3 over the blind-overtake corridor, two CAVs,
and background traffic whose density is swept per episode.

Env knobs:
  DENSITY  vehicles to spawn (default 20; sweep 3-60 across episodes)
  FRAMES   sim ticks to run (default 600, ~30 s)
"""
import os
import carla
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.utils.yaml_utils import add_current_time, save_yaml


def run_scenario(opt, scenario_params):
    scenario_manager = None
    try:
        scenario_params = add_current_time(scenario_params)

        density = int(os.environ.get('DENSITY', 20))
        frames = int(os.environ.get('FRAMES', 600))
        # single corridor spawn grid; last field is the vehicle count
        scenario_params['carla_traffic_manager']['range'] = [
            [40, 320, 190, 204, 14, 4, density]]

        cav_world = CavWorld(opt.apply_ml)
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town01',
                                                   cav_world=cav_world)

        # RSU-only v1: this fork's VehicleManager requires edge-list
        # vehicles; the RSU masts are the deployment viewpoint and the
        # Multi-V2X format treats RSUs as agents, so RSU dumpers +
        # autopilot traffic already produce valid training scenes.
        rsu_list = scenario_manager.create_rsu_manager(data_dump=True)

        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        current_path = os.path.dirname(os.path.realpath(__file__))
        save_yaml_name = os.path.join(
            current_path, '../../data_dumping',
            scenario_params['current_time'], 'data_protocol.yaml')
        save_yaml(scenario_params, save_yaml_name)
        print(f"[DATADUMP] density={density} frames={frames} "
              f"bg_spawned={len(bg_veh_list)}")

        for _ in range(frames):
            scenario_manager.tick()
            for rsu in rsu_list:
                rsu.update_info()
                rsu.run_step()
        print("[DATADUMP] episode complete")

    finally:
        if scenario_manager is not None:
            scenario_manager.close()
