# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import carla
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
# from opencda.scenario_testing.utils.keyboard_listener import KeyListener

import time
from multiprocessing import Process
import psutil
from opencda.scenario_testing.utils.yaml_utils import add_current_time
import scenario_runner.scenario_runner as sr
from threading import Thread

MAX_STEP = 600
SCENARIO_NAME = 'openscenario_3_edge_worldfusion'
scenario_runner = None

def exec_scenario_runner(scenario_params):
    """
    Execute the ScenarioRunner process

    Parameters
    ----------
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    #global scenario_runner
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    #print(scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_scenario(opt, scenario_params):
    #scenario_runner = None
    global scenario_runner
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

        #scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        #scenario_runner.run()

        # Create a background process to init and execute scenario runner

        print("Scenario params Scenario Runner: %s" % scenario_params.scenario_runner)

        sr_process = Process(target=exec_scenario_runner,
                             args=(scenario_params,))
        sr_process.start()
        #sr_thread = Thread(target=exec_scenario_runner, args=(scenario_params.scenario_runner,))
        #sr_thread.start()

        # key_listener = KeyListener()
        # key_listener.start()

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

        # Get all vehicle actor ids and add them to the edge manager
        #other_vehicles = scenario_runner.manager.scenario.agents
        #input("Other vehicles: %s" %other_vehicles)
    
        #for vehicle in other_vehicles:
        #    print(vehicle._actor.id)
        #    print(vehicle._local_planner_dict)
        other_vehicles = []

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert( edge_dt % world_dt == 0 ) # we need edge time to be an exact multiple of world time because we send waypoints every Nth tick 

        edge_list = []                      # make sure the name exists
        try:
            edge_list = scenario_manager.create_edge_manager_from_scenario_runner(
                application=['edge'],
                edge_dt=edge_dt,
                world_dt=world_dt,
                ego_vehicle=ego_vehicle,
                other_vehicles=other_vehicles,
            )
        except AssertionError as err:
            # print the message carried by the assert – that’s what we need
            import traceback, sys
            print("\n\n>>> ASSERTION INSIDE create_edge_manager_from_scenario_runner <<<")
            traceback.print_exc()           # full stack-trace
            sys.exit(1)                     # stop cleanly so we can read it
        except Exception:
            # any other bug – still show the full trace
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

        #edge_list = scenario_manager.create_edge_manager_from_scenario_runner(application=['edge'], edge_dt=edge_dt, world_dt=world_dt,ego_vehicle=ego_vehicle, other_vehicles=other_vehicles)



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
        if scenario_runner is not None:
            scenario_runner.destroy()
        if sr_process is not None:
            sr_process.terminate()
            sr_process.join()
            print("Joined scenario_runner process")
        print("Destroyed scenario_runner")
