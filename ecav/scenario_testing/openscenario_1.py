# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# from ecav.scenario_testing.utils.keyboard_listener import KeyListener

import time
from multiprocessing import Process
import psutil
from ecav.scenario_testing.utils.yaml_utils import add_current_time
import scenario_runner.scenario_runner as sr
from threading import Thread
#from scenario_runner import ScenarioRunner

SCENARIO_NAME = 'openscenario_1'


def exec_scenario_runner(scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    #print(help(scenario_runner))
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_scenario(opt, scenario_params):
    scenario_runner = None
    cav_world = None
    scenario_manager = None
    step = 0

    try:
        scenario_params = add_current_time(scenario_params)

        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)
        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(scenario_params,
                                                   opt.apply_ml,
                                                   opt.version,
                                                   town=scenario_params.scenario_runner.town,
                                                   cav_world=cav_world,
                                                   distributed=False)

        # Create a background process to init and execute scenario runner
        #sr_process = Process(target=exec_scenario_runner,
        #                     args=(scenario_params, ))
        #sr_process.start()

        sr_thread = Thread(target=exec_scenario_runner, args=(scenario_params, ))
        sr_thread.start()
        
        # key_listener = KeyListener()
        # key_listener.start()
        
        #scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)

        



        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero':
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')

        single_cav_list = scenario_manager.create_vehicle_manager_from_scenario_runner(
            vehicle=ego_vehicle,
        )


        # create evaluation manager
        eval_manager = \
            EvaluationManager(scenario_manager.cav_world,
                              script_name=SCENARIO_NAME,
                              current_time=scenario_params['current_time'])

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 100
        spectator_bird_pitch = -90

        while True:
            # if key_listener.keys['esc']:
            #     sr_process.kill()
            #     # Terminate the main process
            #     return
            # if key_listener.keys['p']:
            #     psutil.Process(sr_process.pid).suspend()
            #     continue
            # if not key_listener.keys['p']:
            #     psutil.Process(sr_process.pid).resume()

            scenario_manager.tick()
            ego_cav = single_cav_list[0].vehicle

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            # Apply the control to the ego vehicle
            for _, single_cav in enumerate(single_cav_list):
                single_cav.update_info(step)
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
            step = step + 1
            time.sleep(.001)

    finally:
        for i, single_cav in enumerate(single_cav_list):
            for vid, step_number in single_cav.vehicles_detected.items():    
                print("VID: %s found VID %s at step %s" %(single_cav.vehicle.id, vid, step_number))
        eval_manager.evaluate()
        if cav_world is not None:
            cav_world.destroy()
        print("Destroyed cav_world")
        if scenario_manager is not None:
            scenario_manager.close()
        print("Destroyed scenario_manager")
        if scenario_runner is not None:
            scenario_runner.destroy()
        print("Destroyed scenario_runner")

