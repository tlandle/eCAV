# -*- coding: utf-8 -*-
"""
Scenario testing: two vehicle driving in the customized 2 lane highway map.
"""
# Author: Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

# Core
import os
import time

# 3rd Party
import carla

# eCAV Utils
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.scenario_testing.utils.yaml_utils import load_yaml
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# ONLY *required* for 2 Lane highway scenarios
import ecav.scenario_testing.utils.customized_map_api as map_api

import ecloud_pb2 as ecloud

def run_scenario(opt, config_yaml):
    try:
        scenario_params = load_yaml(config_yaml)

        # sanity checks...
        assert('edge_list' not in scenario_params['scenario']) # do NOT use this template for edge scenarios
        assert('sync_mode' in scenario_params['world'] and scenario_params['world']['sync_mode'] == True)
        assert(scenario_params['world']['fixed_delta_seconds'] == 0.03 or scenario_params['world']['fixed_delta_seconds'] == 0.05)

        current_path = os.path.dirname(os.path.realpath(__file__))
        xodr_path = os.path.join(
            current_path,
            '../assets/2lane_freeway_simplified/2lane_freeway_simplified.xodr')

        run_distributed = scenario_params['distributed'] if 'distributed' in scenario_params else \
                          True if 'ecloud' in scenario_params else \
                          False

        # create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   xodr_path=xodr_path,
                                                   cav_world=cav_world,
                                                   config_file=config_yaml,
                                                   distributed=run_distributed)                                         

        if opt.record:
            scenario_manager.client. \
                start_recorder("multi_2lanefree_carla.log", True)

        # create single cavs        
        if run_distributed:
            single_cav_list = \
                scenario_manager.create_distributed_vehicle_manager(application=['single']) 
        else:    
            single_cav_list = \
                scenario_manager.create_vehicle_manager(application=['single'])#,
                                                        #map_helper=map_api.spawn_helper_2lanefree)

        # create background traffic in carla
        traffic_manager, bg_veh_list = \
            scenario_manager.create_traffic_carla()

        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name='multi_2lanefree_carla',
                              current_time=scenario_params['current_time'])

        spectator = scenario_manager.world.get_spectator()

        # run steps
        step = 0 
        flag = True
        while flag:
            print("Step: %d" %step)
            scenario_manager.tick_world()
            if run_distributed:
                flag = scenario_manager.broadcast_tick()
            
            else:    
                # non-dist will break automatically; don't need to set flag
                pre_client_tick_time = time.time()
                for i, single_cav in enumerate(single_cav_list):
                    single_cav.update_info()
                    control = single_cav.run_step()
                    single_cav.vehicle.apply_control(control)
                post_client_tick_time = time.time()
                print("Client tick completion time: %s" %(post_client_tick_time - pre_client_tick_time))
                if step > 0: # discard the first tick as startup is a major outlier
                    scenario_manager.sim_metrics.update_client_tick((post_client_tick_time - pre_client_tick_time)*1000)

            # same for dist / non-dist - only required for specate
            transform = single_cav_list[0].vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                transform.location +
                carla.Location(
                    z=120),
                carla.Rotation(
                    pitch=-90)))   

            step = step + 1
            if step > 750:
                if run_distributed:
                    flag = scenario_manager.broadcast_message(ecloud.Command.REQUEST_DEBUG_INFO)
                break             

    finally:
        if run_distributed:
            scenario_manager.end() # only dist requires explicit scenario end call

        eval_manager.evaluate()

        if opt.record:
            scenario_manager.client.stop_recorder()

        scenario_manager.close()
  
        if not run_distributed:
            for v in single_cav_list:
                v.destroy()

        for v in bg_veh_list:
            print("destroying background vehicle")
            try:
                v.destroy()
            except:
                print("failed to destroy background vehicle")  