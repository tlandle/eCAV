# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import datetime
import os
import time
from multiprocessing import Process
import asyncio

import carla
import scenario_runner.scenario_runner as sr
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import add_current_time

import ecloud_pb2 as ecloud

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
    scenario_params.scenario_runner.distributed = scenario_params.get(
        'distributed', False)
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_vehicle(opt, scenario_params):
    """
    Execute a distributed vehicle actor.

    This function is called when running in distributed mode with a specific
    vehicle index. Each vehicle runs in its own process/container.

    Parameters
    ----------
    opt: Command line options
    scenario_params: Parameters of ScenarioRunner

    Returns
    -------
    """
    assert opt.distributed, "Must run in distributed mode when specifying vehicle index"
    try:
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()
    except Exception as e:
        print(f"vehicle_index: {scenario_params.scenario_runner.vehicle_index}")
        raise e


def run_scenario(opt, scenario_params):
    """
    Run the WorldFusion scenario, either in sequential or distributed mode.

    Parameters
    ----------
    opt: Command line options (includes .distributed flag)
    scenario_params: Scenario configuration
    """
    global scenario_runner
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0

    try:
        scenario_params = add_current_time(scenario_params)

        # Create CAV world with config for ML manager settings
        cav_world = CavWorld(
            apply_ml=opt.apply_ml,
            config=scenario_params,
            litserve=getattr(opt, 'litserve', False)
        )

        # Create scenario manager
        scenario_manager = sim_api.ScenarioManager(
            scenario_params,
            opt.apply_ml,
            opt.version,
            town=scenario_params.scenario_runner.town,
            cav_world=cav_world,
            distributed=opt.distributed
        )

        if opt.distributed:
            # Distributed mode: wait for actors to connect via gRPC
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
        else:
            # Sequential mode: launch ScenarioRunner in subprocess
            print("Scenario params Scenario Runner: %s" % scenario_params.scenario_runner, flush=True)
            sr_process = Process(target=exec_scenario_runner,
                                 args=(scenario_params,))
            sr_process.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors", flush=True)
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero' and ego_vehicle is None:
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')

        other_vehicles = []

        world_dt = scenario_params['world']['fixed_delta_seconds']
        edge_dt = scenario_params['edge_base']['edge_dt']
        assert edge_dt % world_dt == 0, \
            "edge_dt must be an exact multiple of world_dt"

        try:
            edge_list = scenario_manager.create_edge_manager_from_scenario_runner(
                application=['edge'],
                edge_dt=edge_dt,
                world_dt=world_dt,
                ego_vehicle=ego_vehicle,
                other_vehicles=other_vehicles,
            )
        except AssertionError as err:
            import traceback, sys
            print("\n\n>>> ASSERTION INSIDE create_edge_manager_from_scenario_runner <<<")
            traceback.print_exc()
            sys.exit(1)
        except Exception:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

        eval_manager = EvaluationManager(
            scenario_manager.cav_world,
            script_name=SCENARIO_NAME,
            scenario_params=scenario_params,
            current_time=scenario_params['current_time'],
            output_dir=opt.output_dir
        )

        spectator = ego_vehicle.get_world().get_spectator()
        # Bird view following
        spectator_altitude = 133
        spectator_bird_pitch = -90

        flag = True
        while flag:
            t_step_start = time.time()

            # Determine continue condition and command based on mode
            if opt.distributed:
                t0 = time.time()
                command = ecloud.Command.PULL_OBJECTS_AND_TICK if step > 0 else ecloud.Command.TICK
                flag = scenario_manager.broadcast_message(command)
                t_broadcast = time.time()

                scenario_manager.tick_world()
                t_tick = time.time()
            else:
                t0 = time.time()
                scenario_manager.tick()
                t_broadcast = t0
                t_tick = time.time()

            ego_cav = edge_list[0].vehicle_manager_list[0].vehicle
            loc = ego_cav.get_transform().location
            if loc.x == 0 and loc.y == 0:
                break

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = loc
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)
            t_spectator = time.time()

            # Run edge processing step (update_information is called internally)
            for edge in edge_list:
                serialized_predictions = edge.run_step(step)

                if opt.distributed and serialized_predictions is not None:
                    scenario_manager.push_edge_objects(serialized_predictions)
            t_edge = time.time()

            t_total = time.time() - t_step_start
            print(f"\n[TICK {step}] total={t_total*1000:.0f}ms | "
                  f"broadcast={((t_broadcast-t0)*1000):.0f}ms | "
                  f"world_tick={((t_tick-t_broadcast)*1000):.0f}ms | "
                  f"spectator={((t_spectator-t_tick)*1000):.0f}ms | "
                  f"edge={((t_edge-t_spectator)*1000):.0f}ms\n")

            step = step + 1
            if step >= MAX_STEP:
                print("Reached maximum step limit, exiting")
                break

            time.sleep(0.001)

    except SystemExit as e:
        print(f"Caught SystemExit({e.code}) in run_scenario - proceeding to evaluation/cleanup")

    except Exception as e:
        print(f"Caught exception {type(e).__name__} in run_scenario: {e} - proceeding to evaluation/cleanup")
        import traceback
        print(traceback.format_exc())

    finally:
        # Terminate ScenarioRunner subprocess FIRST to avoid blocking
        if sr_process is not None:
            sr_process.terminate()
            sr_process.join(timeout=5)
            print("Joined scenario_runner process")

        if scenario_runner is not None:
            scenario_runner.destroy()
            print("Destroyed scenario_runner")

        for edge in edge_list:
            for i, vehicle_manager in enumerate(edge.vehicle_manager_list):
                for vid, step_number in vehicle_manager.vehicles_detected.items():
                    print("VID: %s found VID %s at step %s" % (vehicle_manager.vehicle.id, vid, step_number))

        # Flush per-agent WorldFusion timing CSVs and save EdgeProfiler JSON
        os.makedirs('logs', exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        for edge in edge_list:
            for vm in edge.vehicle_manager_list:
                pm = getattr(vm, 'perception_manager', None)
                if pm is not None and hasattr(pm, 'close'):
                    pm.close()
            for rsu in edge.rsu_manager_list:
                pm = getattr(rsu, 'perception_manager', None)
                if pm is not None and hasattr(pm, 'close'):
                    pm.close()
            edge.profiler.save_report(os.path.join('logs', f'edge_profiler_{ts}.json'))

        if opt.distributed and scenario_manager is not None:
            scenario_manager.end()

        if eval_manager is not None:
            eval_manager.evaluate()

        if scenario_manager is not None:
            scenario_manager.close()
            print("Destroyed scenario_manager")
