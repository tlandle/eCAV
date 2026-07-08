# -*- coding: utf-8 -*-
# Author: Jordan Rapp + Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Two-edge sequential handoff scenario — Town03, Mamba tracker (full-latent migration).

Extends openscenario_3_multi_edge_late_fusion: both edges run the pluggable
SOTA manager with the Mamba3DMOT tracker, so request_handoff migrates the
tracker's full learned latent (memo/diff banks) instead of a KF snapshot.
Original features:
  - Per-tick snapshot upload to sim_api state store.
  - Tick-based handoff trigger (HandoffManager) at HANDOFF_TICK.
  - SequentialMigrationDaemon.request_handoff wires ownership move + cost record.

Reuses scenario_3.xml and scenario_3.py (ScenarioRunner) unchanged.
"""

import asyncio
import logging
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
from ecav.core.application.edge.migration import (
    HandoffManager,
    InterLocaleLink,
    SequentialMigrationDaemon,
)

logger = logging.getLogger(__name__)

MAX_STEP = 300
SCENARIO_NAME = 'openscenario_3_multi_edge_mamba'

# Tick at which handoff is triggered.
# At 43 km/h (11.9 m/s) and dt=0.05s, ego travels 0.6 m/tick.
# From spawn y=80, tick 60 → y≈116: well past the locale 0/1 overlap (y≈98-108).
HANDOFF_TICK = 60

scenario_runner = None


def exec_scenario_runner(scenario_params):
    scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
    scenario_runner.run()
    scenario_runner.destroy()


def run_vehicle(opt, scenario_params):
    assert opt.distributed, "Must run in distributed mode when specifying vehicle index"
    try:
        scenario_runner = sr.ScenarioRunner(scenario_params.scenario_runner)
        scenario_runner.run()
        scenario_runner.destroy()
    except Exception as e:
        print(f"vehicle_index: {scenario_params.scenario_runner.vehicle_index}")
        raise e


def run_scenario(opt, scenario_params):
    """Run the two-edge handoff scenario (sequential mode only)."""
    global scenario_runner
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0
    fusion_clients = []

    handoff_done = False
    transfer_costs = []
    vid = None

    try:
        scenario_params = add_current_time(scenario_params)

        cav_world = CavWorld(
            apply_ml=opt.apply_ml,
            config=scenario_params,
            litserve=getattr(opt, 'litserve', False)
        )

        scenario_manager = sim_api.ScenarioManager(
            scenario_params,
            opt.apply_ml,
            opt.version,
            town=scenario_params.scenario_runner.town,
            cav_world=cav_world,
            distributed=opt.distributed
        )

        if opt.distributed:
            asyncio.get_event_loop().run_until_complete(scenario_manager.run_comms())
        elif getattr(opt, 'edge_only', False):
            reg_server = EdgeRegistrationServer(
                scenario_params=dict(scenario_params),
                port=getattr(opt, 'edge_reg_port', 50055),
            )
            fusion_clients = asyncio.get_event_loop().run_until_complete(
                reg_server.start_and_wait(timeout_s=120.0)
            )
            for fc in fusion_clients:
                fc.connect(retry_timeout_s=60.0)
            sr_process = Process(target=exec_scenario_runner, args=(scenario_params,))
            sr_process.start()
        else:
            sr_process = Process(target=exec_scenario_runner, args=(scenario_params,))
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
        assert edge_dt % world_dt == 0, "edge_dt must be an exact multiple of world_dt"

        try:
            edge_list = scenario_manager.create_edge_manager_from_scenario_runner(
                application=['edge'],
                edge_dt=edge_dt,
                world_dt=world_dt,
                ego_vehicle=ego_vehicle,
                other_vehicles=other_vehicles,
            )
        except AssertionError:
            import traceback, sys
            print("\n\n>>> ASSERTION INSIDE create_edge_manager_from_scenario_runner <<<")
            traceback.print_exc()
            sys.exit(1)
        except Exception:
            import traceback, sys
            traceback.print_exc()
            sys.exit(1)

        # Set up handoff primitives (after edge_list is populated so latency_model exists).
        handoff_manager = HandoffManager(trigger_tick=HANDOFF_TICK)
        daemon = SequentialMigrationDaemon()
        link = InterLocaleLink(edge_list[0].latency_model)

        eval_manager = EvaluationManager(
            scenario_manager.cav_world,
            script_name=SCENARIO_NAME,
            scenario_params=scenario_params,
            current_time=scenario_params['current_time'],
            output_dir=opt.output_dir
        )

        spectator = ego_vehicle.get_world().get_spectator()
        spectator_altitude = 133
        spectator_bird_pitch = -90

        flag = True
        while flag:
            if opt.distributed:
                command = ecloud.Command.PULL_OBJECTS_AND_TICK if step > 0 else ecloud.Command.TICK
                flag = scenario_manager.broadcast_message(command)
                scenario_manager.tick_world()
            else:
                scenario_manager.tick()

            # Per-tick snapshot upload — export managed vehicle state to store.
            for edge in edge_list:
                for vm in edge.vehicle_manager_list:
                    payload = edge.export_vehicle_state(vm.vehicle.id)
                    if payload:
                        scenario_manager.store_vehicle_state(vm.vehicle.id, payload)

            # Tick-based handoff trigger.
            if not handoff_done:
                if vid is None and edge_list[0].vehicle_manager_list:
                    vid = edge_list[0].vehicle_manager_list[0].vehicle.id
                    logger.info("[HANDOFF] resolved vid=%d", vid)
                if vid is not None:
                    event = handoff_manager.evaluate(vid, step, step * world_dt)
                    if event:
                        cost = daemon.request_handoff(
                            vid, edge_list[0], edge_list[1],
                            scenario_manager, link, step
                        )
                        transfer_costs.append(cost)
                        handoff_done = True
                        logger.info(
                            "[HANDOFF] tick=%d vid=%d bytes=%d total_ms=%.3f",
                            step, vid, cost.payload_bytes, cost.total_ms,
                        )

            # Find ego across all edges (after handoff it moves to edge_list[1]).
            ego_cav = None
            for edge in edge_list:
                if edge.vehicle_manager_list:
                    ego_cav = edge.vehicle_manager_list[0].vehicle
                    break
            if ego_cav is None:
                logger.warning("[HANDOFF] no ego vehicle found in any edge at tick %d", step)
                break

            loc = ego_cav.get_transform().location
            if loc.x == 0 and loc.y == 0:
                break
            if opt.distributed and scenario_manager is not None and scenario_manager.all_vehicles_done:
                break

            view_transform = carla.Transform()
            view_transform.location = loc
            view_transform.location.z = loc.z + spectator_altitude
            view_transform.rotation.pitch = spectator_bird_pitch
            spectator.set_transform(view_transform)

            if getattr(opt, 'edge_only', False):
                for edge, fc in zip(edge_list, fusion_clients):
                    batch = edge.collect_features(step)
                    result = fc.fuse(step, batch)
                    edge.apply_predictions(step, result)
            elif not opt.distributed:
                for edge in edge_list:
                    edge.run_step(step)

            step += 1
            if step >= MAX_STEP:
                print("Reached maximum step limit, exiting")
                break

            time.sleep(0.001)

    except SystemExit as e:
        print(f"Caught SystemExit({e.code}) in run_scenario - proceeding to cleanup")

    except Exception as e:
        print(f"Caught exception {type(e).__name__}: {e}")
        import traceback
        print(traceback.format_exc())

    finally:
        if sr_process is not None:
            sr_process.terminate()
            sr_process.join(timeout=5)
            print("Joined scenario_runner process")

        if scenario_runner is not None:
            scenario_runner.destroy()

        for edge in edge_list:
            for vm in edge.vehicle_manager_list:
                for vid_det, step_num in vm.vehicles_detected.items():
                    print(f"VID: {vm.vehicle.id} found VID {vid_det} at step {step_num}")

        for cost in transfer_costs:
            logger.info(
                "[TRANSFER_COST] vid=%d tick=%d bytes=%d "
                "serialize_ms=%.4f network_ms=%.4f total_ms=%.4f",
                cost.vehicle_id, cost.tick, cost.payload_bytes,
                cost.sim_serialize_ms, cost.sim_network_ms, cost.total_ms,
            )
        if not transfer_costs:
            logger.warning("[TRANSFER_COST] no handoff was executed (HANDOFF_TICK=%d, steps=%d)",
                           HANDOFF_TICK, step)

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
