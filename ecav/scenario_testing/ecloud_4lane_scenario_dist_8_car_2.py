# -*- coding: utf-8 -*-
"""
Scenario testing: two vehicle driving in the customized 2 lane highway map.
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import os

import carla
from ecav.core.common.vehicle_manager_proxy import VehicleManagerProxy

import ecav.scenario_testing.utils.sim_api as sim_api
import ecav.scenario_testing.utils.customized_map_api as map_api

from ecav.core.common.cav_world import CavWorld
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

        # create single cavs
        single_cav_list = \
            scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='ecloud_4lane_scenario',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
       
        flag = True
        while flag:
            scenario_manager.tick()

            # gRPC begin
            # call sim_api to update tick
            # loop here --> sim_api should not return True until tick completed

            #gRPC end

            # TODO eCloud - figure out another way to have the vehicle follow a CAV. Perhaps still access the bp since it's read only?
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=80),
                carla.Rotation(
                    pitch=-
                    90)))

            # for _, single_cav in enumerate(single_cav_list):
            #     result = single_cav.do_tick()
            #     if result == 1: # Need to figure out how to use a const
            #         print("Unexpected termination: Sending END to all vehicles and ending.")
            #         flag = False
            #         break
            #     elif result == 2:
            #         print("Simulation ended: Sending END to all vehicles and ending.")
            #         flag = False
            #         break

        # TODO gRPC    
        #for _, single_cav in enumerate(single_cav_list):
        #    single_cav.end_step()

    finally:
        print("Evaluating simulation results...")
        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close(single_cav_list[0])

        for v in bg_veh_list:
            v.destroy()
