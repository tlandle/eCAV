# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import asyncio
import time
from multiprocessing import Process

import carla
import ecloud_pb2 as ecloud

import scenario_runner.scenario_runner as sr
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import add_current_time
from ecav.scenario_testing.utils.edge_fusion_client import EdgeFusionClient
from ecav.scenario_testing.utils.edge_registration_server import EdgeRegistrationServer

MAX_STEP = 300
SCENARIO_NAME = 'openscenario_3_edge_late_fusion'
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
    Run the Late Fusion scenario, either in sequential or distributed mode.

    Late Fusion uses YOLO-based detection from each agent, fused at the edge
    via AB3DMOT tracking, with linear velocity prediction.

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

    fusion_clients = []  # EdgeFusionClient list — populated in -eo mode

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
        elif getattr(opt, 'edge_only', False):
            # Edge-only distributed mode: start registration server, wait for edge containers
            reg_server = EdgeRegistrationServer(
                scenario_params=dict(scenario_params),
                port=getattr(opt, 'edge_reg_port', 50055),
            )
            fusion_clients = asyncio.get_event_loop().run_until_complete(
                reg_server.start_and_wait(timeout_s=120.0)
            )
            for fc in fusion_clients:
                fc.connect(retry_timeout_s=60.0)
            print(f"[EDGE-ONLY] {len(fusion_clients)} edge(s) ready", flush=True)
            sr_process = Process(target=exec_scenario_runner, args=(scenario_params,))
            sr_process.start()
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
            if opt.distributed and scenario_manager is not None and scenario_manager.all_vehicles_done:
                print(f"All vehicles reported done ({scenario_manager.num_completed_vehicles}/{scenario_manager.vehicle_count}), ending simulation")
                break
            view_transform.location.z = view_transform.location.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            # Fusion step:
            #   sequential:      edge manager runs perception + fusion + planning locally
            #   distributed (-d): fusion runs in edge_process containers; nothing to do here
            #   edge-only (-eo): collect detections locally, fuse via RPC, apply predictions
            if getattr(opt, 'edge_only', False):
                for edge, fc in zip(edge_list, fusion_clients):
                    batch = edge.collect_features(step)
                    result = fc.fuse(step, batch)
                    edge.apply_predictions(step, result)
            elif not opt.distributed:
                for edge in edge_list:
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

        for fc in fusion_clients:
            fc.end_scenario()
            fc.close()

        if opt.distributed and scenario_manager is not None:
            scenario_manager.end()

        if eval_manager is not None:
            eval_manager.evaluate()

        if cav_world is not None:
            cav_world.close()

        if scenario_manager is not None:
            scenario_manager.close()
            print("Destroyed scenario_manager")
