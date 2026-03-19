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
                                                   cav_world=cav_world,
                                                   config_file=config_yaml)

        if opt.record:
            scenario_manager.client. \
                start_recorder("ecloud_4lane.log", True)


        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']

        # create single cavs
        edge_list = \
            scenario_manager.create_edge_manager(application=['edge'], edge_dt=edge_dt, world_dt=world_dt)

        # create background traffic in carla
        #traffic_manager, bg_veh_list = \
            #scenario_manager.create_traffic_carla()

        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_edge_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        spectator_vehicle = edge_list[0].vehicle_manager_list[0].vehicle

        # run steps

        eval_time = 0
        flag = True
        waypoint_buffer = []

        world_time = 0

        while flag:
            eval_time += 1
            #print("Stepping, ", eval_time*0.2)

            scenario_manager.tick_world()

            world_time += world_dt

            if world_time > edge_dt:
                world_time = 0

                waypoint_buffer.clear()
                for edge in edge_list:
                    edge.update_information()
                    waypoint_buffer = edge.run_step()

                scenario_manager.push_waypoint_buffer(waypoint_buffer)

            flag = scenario_manager.broadcast_tick()
            
            transform = spectator_vehicle.get_transform()
            spectator.set_transform(
                carla.Transform(
                    transform.location +
                    carla.Location(
                        z=80),
                    carla.Rotation(
                        pitch=-
                        90)))


    finally:

        scenario_manager.end()

        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()       

        scenario_manager.close(single_cav_list[0])
        for edge in edge_list:
            edge.destroy()
