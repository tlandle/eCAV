# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import time
from multiprocessing import Process

import carla
import scenario_runner.scenario_runner as sr
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time

MAX_STEP = 600
SCENARIO_NAME = 'openscenario_3_edge'
scenario_runner = None

def run_vehicle(opt, scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    try:
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()
    except Exception as e:
        print(f"vehicle_index: {scenario_params.scenario_runner.vehicle_index}")
        raise e

def run_scenario(opt, scenario_params):
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
                                                   cav_world=cav_world)

        print("Scenario params Scenario Runner: %s" % scenario_params.scenario_runner)
    
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

        other_vehicles = []

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert( edge_dt % world_dt == 0 ) # we need edge time to be an exact multiple of world time because we send waypoints every Nth tick 

        edge_list = scenario_manager.create_edge_manager_from_scenario_runner(application=['edge'], edge_dt=edge_dt, world_dt=world_dt,ego_vehicle=ego_vehicle, other_vehicles=other_vehicles)

        eval_manager = EvaluationManager(scenario_manager.cav_world, 
                                         script_name=SCENARIO_NAME, 
                                         scenario_params=scenario_params,
                                         current_time=scenario_params['current_time'],
                                         output_dir=opt.output_dir)

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 100
        spectator_bird_pitch = -90

        while True:
            scenario_manager.tick()
            print("about to set ego cav")
            ego_cav = edge_list[0].vehicle_manager_list[0].vehicle
            print("bird view following")

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            print("ego_cav.get_transform().location: %s" %ego_cav.get_transform().location)
            if ego_cav.get_transform().location.x == 0 and ego_cav.get_transform().location.y == 0:
                break;
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)
            

            # Apply the control to the ego vehicle
            for edge in edge_list:
                edge.update_information(step)
                edge.run_step(step)
            step = step + 1
            if step >= MAX_STEP:
                print("Reached maximum step limit, exiting")
                break
            time.sleep(0.001)

    except SystemExit as e:
        print(f"Caught SystemExit({e.code}) in run_scenario — proceeding to evaluation/cleanup")

    except Exception as e:
        print(f"Caught exception {type(e).__name__} in run_scenario: {e} — proceeding to evaluation/cleanup")

    finally:
        for edge in edge_list:
            for i, vehicle_manager in enumerate(edge.vehicle_manager_list):
                for vid, step_number in vehicle_manager.vehicles_detected.items():
                    print("VID: %s found VID %s at step %s" %(vehicle_manager.vehicle.id, vid, step_number))
        
        if eval_manager is not None:
             eval_manager.evaluate()
        
        if scenario_manager is not None:
            scenario_manager.close()
            print("Destroyed scenario_manager")

