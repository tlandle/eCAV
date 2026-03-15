# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

import time
from multiprocessing import Process
import asyncio

import carla
import scenario_runner.scenario_runner as sr
import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.evaluations.evaluate_manager import \
    EvaluationManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time

import ecloud_pb2 as ecloud

MAX_STEP = 250
SCENARIO_NAME = 'openscenario_3_edge'
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
    Run the main scenario, either in sequential or distributed mode.

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

        # Create CAV world
        cav_world = CavWorld(opt.apply_ml)

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
            print("Scenario params Scenario Runner: %s" % scenario_params.scenario_runner)
            sr_process = Process(target=exec_scenario_runner,
                                 args=(scenario_params,))
            sr_process.start()

        world = scenario_manager.world
        ego_vehicle = None
        num_actors = 0

        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
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
            # Determine continue condition and command based on mode
            if opt.distributed:
                command = ecloud.Command.PULL_OBJECTS_AND_TICK if step > 0 else ecloud.Command.TICK
                flag = scenario_manager.broadcast_message(command)
                scenario_manager.tick_world()
            else:
                scenario_manager.tick()

            print("about to set ego cav")
            ego_cav = edge_list[0].vehicle_manager_list[0].vehicle
            print("bird view following")

            # Bird view following
            view_transform = carla.Transform()
            view_transform.location = ego_cav.get_transform().location
            print("ego_cav.get_transform().location: %s" % ego_cav.get_transform().location)
            if ego_cav.get_transform().location.x == 0 and ego_cav.get_transform().location.y == 0:
                break
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

        if opt.distributed and scenario_manager is not None:
            scenario_manager.end()

        if eval_manager is not None:
            eval_manager.evaluate()

        if scenario_manager is not None:
            scenario_manager.close()
            print("Destroyed scenario_manager")
