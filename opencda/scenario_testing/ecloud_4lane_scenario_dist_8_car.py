# -*- coding: utf-8 -*-
"""
eCloudSim
---------
Scenario testing: *TEMPLATE* 
Use for OpenCDA vs eCloudSim DIST-ONLY comparisons

*NOT for Edge*

Town 06 Scenarios *ONLY*

DO NOT USE for 2-Lane Free
"""
# Author: Jordan Rapp, Dean Blank, Tyler Landle <Georgia Tech>
# License: TDG-Attribution-NonCommercial-NoDistrib

# Core
import os

# 3rd Party
import carla

# OpenCDA Utils
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# ONLY *required* for 2 Lane highway scenarios
# import opencda.scenario_testing.utils.customized_map_api as map_api

import ecloud_pb2 as ecloud

# Consts
LOG_NAME = "ecloud_4lane.log" # data drive from file name?
SCENARIO_NAME = "ecloud_4lane_scenario" # data drive from file name?
TOWN = 'Town06'

def run_scenario(opt, config_yaml):
#    try:
        scenario_params = load_yaml(config_yaml)

        # sanity checks...
        assert('edge_list' not in scenario_params['scenario']) # do NOT use this template for edge scenarios
        assert('sync_mode' in scenario_params['world'] and scenario_params['world']['sync_mode'] == True)
        assert(scenario_params['world']['fixed_delta_seconds'] == 0.03 or scenario_params['world']['fixed_delta_seconds'] == 0.05)
        
        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=TOWN,
                                                   cav_world=cav_world)

        if opt.record:
            scenario_manager.client. \
                start_recorder(LOG_NAME, True)

        # create single cavs
        run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else False
        if run_distributed:
            single_cav_list = \
                scenario_manager.create_distributed_vehicle_manager(application=['single']) 
        else:    
            single_cav_list = \
                scenario_manager.create_vehicle_manager(application=['single'])

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name=SCENARIO_NAME,
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()
        # run steps
        step = 0 
        flag = True
        while flag:
            if run_distributed:
                print("Step: %d" %step)
                scenario_manager.tick_world()
                flag = scenario_manager.broadcast_tick()

                # update the Carla data on VehicleManagerProxies
                # REQUIRED for Evaluation which runs on Proxies' local data
                # DO NOT WANT THIS TO RUN! SLOW!!
                # for _, single_cav in enumerate(single_cav_list):
                #     single_cav.update_info()

                step = step + 1
                if(step > 250):
                    flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                    break
            
            else:    
                # non-dist will break automatically; don't need to set flag
                scenario_manager.tick()

            # same for dist / non-dist - only required for specate
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=80),
                carla.Rotation(
                    pitch=-
                    90)))                

#    finally:
        if run_distributed:
            scenario_manager.end() # only dist requires explicit scenario end call

        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close(single_cav_list[0])
  
        for v in bg_veh_list:
            print("destroying background vehicle")
            try:
                v.destroy()
            except:
                print("failed to destroy background vehicle")  
