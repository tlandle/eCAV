# -*- coding: utf-8 -*-
# Author: Jordan Rapp + Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Scenario B — Town06 right-merge / obstacle hand-off.

The research scenario. Hand-off is geometry-driven for EVERY vehicle that
crosses the locale 0 -> locale 1 boundary (VehicleLocaleTracker):

* Managed vehicles (ego) -> ``daemon.request_handoff``: ownership moves from
  edge 0 to edge 1 (Scenario A's mechanism, now triggered by geometry rather
  than a scripted tick).
* Tracked obstacles (the fast NPC) -> ``daemon.transfer_obstacle_state``: KF
  share, no ownership move — locale 1 receives the NPC's KF history BEFORE
  its own RSU can directly detect it; both edges keep tracking it.

Ego drives the LEFTMOST eastbound lane -3 (solid line to the shoulder on its
left — no left escape). A stationary emergency vehicle blocks lane -3 ahead;
the NPC overtakes in the adjacent lane -4. After its own handoff, ego is
served by edge 1, whose RSU sees the parked blockage long before ego's local
sensors — the edge-detection story. Once the NPC passes, ego merges right and
continues to its far destination.

Reuses Scenario_MultiEdgeRightMerge (scenario_multi_edge_right_merge.xml/.py).
"""

import asyncio
import logging
import time
from multiprocessing import Process

import carla
import numpy as np
import ecloud_pb2 as ecloud

import scenario_runner.scenario_runner as sr
import ecav.scenario_testing.utils.sim_api as sim_api
from ecav.core.common.cav_world import CavWorld
from ecav.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from ecav.scenario_testing.utils.yaml_utils import add_current_time
from ecav.scenario_testing.utils.edge_fusion_client import EdgeFusionClient
from ecav.scenario_testing.utils.edge_registration_server import EdgeRegistrationServer
from ecav.core.application.edge.migration import (
    InterLocaleLink,
    Locale,
    LocaleRegistry,
    LocaleRouter,
    SequentialMigrationDaemon,
    VehicleLocaleTracker,
)

logger = logging.getLogger(__name__)

MAX_STEP = 500
SCENARIO_NAME = 'openscenario_multi_edge_right_merge'

# Speed (m/s) above which a non-hero vehicle is taken to be the fast NPC.
# The emergency vehicle is stationary, so this cleanly disambiguates the two.
NPC_MIN_SPEED_MPS = 2.0

# Geometry proxy for "RSU1 could first directly detect the NPC": the tick the
# NPC first enters this radius of RSU1. Compared against the hand-off tick, the
# gap is the advance-warning window the hand-off buys locale 1. A proxy, not a
# YOLO-pipeline detection tick — good enough to size the window.
RSU1_DETECT_RANGE_M = 60.0

# Locale hysteresis: consecutive ticks in the destination locale before the
# tracker fires the crossing event (0.2 s at 20 Hz).
LOCALE_MIN_DWELL_TICKS = 4

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


def _build_locale_router(edge_cfgs, edge_list):
    """Build a LocaleRouter from each edge's YAML `locale` block.

    Returns ``(router, edge_by_locale)`` where ``edge_by_locale`` maps
    ``locale_id`` to its hosting edge instance, so a HandoffEvent's locale ids
    resolve directly to the src/dst edges of the transfer.
    """
    registry = LocaleRegistry()
    edge_by_locale = {}
    for i, edge_cfg in enumerate(edge_cfgs):
        if 'locale' not in edge_cfg:
            continue
        lc = edge_cfg['locale']
        polygon = np.array([[float(v) for v in row] for row in lc['polygon']],
                           dtype=np.float64)
        locale_id = str(lc['id'])
        registry.register(Locale(
            locale_id=locale_id,
            polygon=polygon,
            edge_host_id=edge_list[i].edgeid,
        ))
        edge_by_locale[locale_id] = edge_list[i]
    return LocaleRouter(registry), edge_by_locale


def run_scenario(opt, scenario_params):
    """Run the Town06 obstacle-handoff scenario (sequential mode only)."""
    global scenario_runner
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0
    fusion_clients = []

    npc_carla_id = None
    obstacle_handoff_done = False
    obstacle_handoff_pending = None  # (src_edge, dst_edge) after the crossing fires
    handoff_tick = None
    rsu1_first_detect_tick = None

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

        # Handoff primitives (after edge_list is populated so edgeid/latency_model exist).
        edge_cfgs = scenario_params['scenario']['edge_list']
        router, edge_by_locale = _build_locale_router(edge_cfgs, edge_list)
        locale_tracker = VehicleLocaleTracker(router, min_dwell_ticks=LOCALE_MIN_DWELL_TICKS)
        daemon = SequentialMigrationDaemon()
        link = InterLocaleLink(edge_list[0].latency_model)

        # RSU1 position for the advance-warning proxy (locale 1's RSU).
        rsu1_pos = None
        if 'rsus' in edge_cfgs[1] and edge_cfgs[1]['rsus']:
            sp = edge_cfgs[1]['rsus'][0]['spawn_position']
            rsu1_pos = (float(sp[0]), float(sp[1]))

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

            # Per-tick snapshot upload (parity with Scenario A; keeps the store warm).
            for edge in edge_list:
                for vm in edge.vehicle_manager_list:
                    payload = edge.export_vehicle_state(vm.vehicle.id)
                    if payload:
                        scenario_manager.store_vehicle_state(vm.vehicle.id, payload)

            # ── Geometry-driven handoff: EVERY vehicle that crosses the locale
            # boundary is handed off. Managed vehicles move ownership
            # (request_handoff); tracked obstacles share KF state
            # (transfer_obstacle_state). Iterate over snapshots because
            # relinquish/accept mutate the vehicle_manager_lists.
            for edge in list(edge_list):
                for vm in list(edge.vehicle_manager_list):
                    vloc = vm.vehicle.get_transform().location
                    event = locale_tracker.update(
                        vm.vehicle.id, (vloc.x, vloc.y), step, step * world_dt)
                    if event is None or event.source_locale_id is None:
                        continue  # no crossing (or initial bind)
                    src_edge = edge_by_locale.get(event.source_locale_id)
                    dst_edge = edge_by_locale.get(event.destination_locale_id)
                    if src_edge is None or dst_edge is None or src_edge is dst_edge:
                        continue
                    cost = daemon.request_handoff(
                        vm.vehicle.id, src_edge, dst_edge,
                        scenario_manager, link, step)
                    scenario_manager.record_handoff_cost(cost)
                    logger.info(
                        "[SCENB] VEHICLE HANDOFF tick=%d vid=%d %s->%s "
                        "bytes=%d total_ms=%.3f",
                        step, vm.vehicle.id,
                        event.source_locale_id, event.destination_locale_id,
                        cost.payload_bytes, cost.total_ms)

            # Resolve the fast NPC (moving, non-hero) once it is up and moving.
            if npc_carla_id is None and step > 10:
                for v in world.get_actors().filter('vehicle.*'):
                    if v.attributes.get('role_name') != 'hero':
                        vel = v.get_velocity()
                        if (vel.x ** 2 + vel.y ** 2) ** 0.5 > NPC_MIN_SPEED_MPS:
                            npc_carla_id = v.id
                            logger.info("[SCENB] fast NPC carla_id=%d resolved at tick=%d",
                                        npc_carla_id, step)
                            break

            # NPC-driven advance-warning proxy + obstacle handoff on locale crossing.
            if npc_carla_id is not None:
                npc_actor = world.get_actor(npc_carla_id)
                if npc_actor is not None:
                    nloc = npc_actor.get_transform().location
                    npc_xy = (nloc.x, nloc.y)

                    if rsu1_first_detect_tick is None and rsu1_pos is not None:
                        d = ((npc_xy[0] - rsu1_pos[0]) ** 2
                             + (npc_xy[1] - rsu1_pos[1]) ** 2) ** 0.5
                        if d <= RSU1_DETECT_RANGE_M:
                            rsu1_first_detect_tick = step
                            logger.info("[SCENB] NPC within RSU1 %.0fm range (d=%.1f) at tick=%d",
                                        RSU1_DETECT_RANGE_M, d, step)

                    if not obstacle_handoff_done:
                        event = locale_tracker.update(
                            npc_carla_id, npc_xy, step, step * world_dt)
                        if event is not None and event.source_locale_id is not None:
                            src_edge = edge_by_locale.get(event.source_locale_id)
                            dst_edge = edge_by_locale.get(event.destination_locale_id)
                            if src_edge is not None and dst_edge is not None \
                                    and src_edge is not dst_edge:
                                # Crossing fired: mark the transfer pending. The
                                # source edge's track of the NPC lags its true
                                # position, so the export may not succeed on the
                                # crossing tick itself — retry below each tick.
                                obstacle_handoff_pending = (src_edge, dst_edge)
                        if obstacle_handoff_pending is not None:
                            src_edge, dst_edge = obstacle_handoff_pending
                            cost = daemon.transfer_obstacle_state(
                                npc_carla_id, src_edge, dst_edge, link, step,
                                position=npc_xy)
                            if cost is not None:
                                scenario_manager.record_handoff_cost(cost)
                                obstacle_handoff_done = True
                                obstacle_handoff_pending = None
                                handoff_tick = step
                                logger.info(
                                    "[SCENB] OBSTACLE HANDOFF tick=%d carla_id=%d "
                                    "bytes=%d total_ms=%.3f",
                                    step, npc_carla_id, cost.payload_bytes, cost.total_ms)

            # Find ego wherever it currently lives (it moves edges on handoff).
            ego_cav = None
            ego_vm = None
            for edge in edge_list:
                if edge.vehicle_manager_list:
                    ego_vm = edge.vehicle_manager_list[0]
                    ego_cav = ego_vm.vehicle
                    break
            if ego_cav is None:
                logger.warning("[SCENB] no ego vehicle found in any edge at tick %d", step)
                break

            # TEMP DIAGNOSTIC: sample ego behavior-agent decision state (merge
            # progress, overtake state machine). Remove once Scenario B validates.
            if ego_vm is not None and step % 5 == 0:
                a = ego_vm.agent
                eloc = ego_cav.get_transform().location
                evel = ego_cav.get_velocity()
                logger.warning(
                    "[EGO-DBG] tick=%d pos=(%.1f,%.1f) spd=%.1f ttc=%s "
                    "hazard_flag=%s ov_allowed=%s ov_ctr=%s ov_wait=%s do_ov=%s "
                    "brake_ttl=%s curved=%s push=%s",
                    step, eloc.x, eloc.y,
                    (evel.x ** 2 + evel.y ** 2) ** 0.5,
                    getattr(a, 'ttc', '?'),
                    getattr(a, 'hazard_flag', '?'),
                    getattr(a, 'overtake_allowed', '?'),
                    getattr(a, 'overtake_counter', '?'),
                    getattr(a, 'overtake_wait_counter', '?'),
                    getattr(a, 'do_overtake', '?'),
                    getattr(a, '_committed_brake_ttl', '?'),
                    getattr(a.get_local_planner(), 'potential_curved_road', '?'),
                    getattr(a, 'destination_push_flag', '?'),
                )

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

        # Advance-warning window: hand-off tick vs. NPC entering RSU1's range.
        if handoff_tick is not None and rsu1_first_detect_tick is not None:
            logger.info(
                "[SCENB] advance-warning window = %d ticks "
                "(handoff tick=%d, RSU1 in-range tick=%d)",
                rsu1_first_detect_tick - handoff_tick, handoff_tick, rsu1_first_detect_tick)
        elif handoff_tick is not None:
            logger.info(
                "[SCENB] handoff at tick=%d; NPC never entered RSU1 %.0fm range this run",
                handoff_tick, RSU1_DETECT_RANGE_M)
        else:
            logger.warning("[SCENB] no obstacle handoff fired (npc_resolved=%s, steps=%d)",
                           npc_carla_id is not None, step)

        transfer_costs = (
            scenario_manager.get_handoff_costs()
            if scenario_manager is not None else []
        )
        for cost in transfer_costs:
            logger.info(
                "[TRANSFER_COST] vid=%d tick=%d bytes=%d "
                "serialize_ms=%.4f network_ms=%.4f total_ms=%.4f",
                cost.vehicle_id, cost.tick, cost.payload_bytes,
                cost.sim_serialize_ms, cost.sim_network_ms, cost.total_ms,
            )

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
