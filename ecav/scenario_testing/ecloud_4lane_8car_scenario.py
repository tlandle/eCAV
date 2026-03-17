# -*- coding: utf-8 -*-
"""
Scenario testing: merging vehicle joining a platoon in the customized 2-lane
freeway sorely with carla Warning: You have to load the 2lanefreecomplete
map into your ue4 editor before running this
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import carla

import ecav.scenario_testing.utils.sim_api as sim_api
import ecav.scenario_testing.utils.customized_map_api as map_api

from ecav.core.common.cav_world import CavWorld


import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import load_yaml


def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town='Town06',
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder("ecloud_4lane.log", True)

        # create single cavs
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        #traffic_manager, bg_veh_list = \
            #scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_4lane_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = single_cav_list[0].vehicle

        # run steps
        while True:
            scenario_manager.tick()
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))

            for i, single_cav in enumerate(single_cav_list):
                # this function should be added in wrapper
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)

    finally:
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close(single_cav_list[0])
        for platoon in platoon_list:
            platoon.destroy()
        for v in bg_veh_list:
            v.destroy()
