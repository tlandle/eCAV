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
WF variant: RELAY production stack (WorldFusion + Mamba3DMOT + MTR).
"""

import asyncio
import logging
import os
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
from ecav.core.application.edge.migration.metrics import MigrationMetricsLogger
from ecav.core.application.edge.migration import (
    InterLocaleLink,
    Locale,
    LocaleRegistry,
    LocaleRouter,
    SequentialMigrationDaemon,
    VehicleLocaleTracker,
)

logger = logging.getLogger(__name__)

MAX_STEP = 1100  # blind overtake: stage+hold+commit+pass runs ~1000 ticks
SCENARIO_NAME = 'openscenario_1_flow_gt'
MIGRATION_MODE = os.environ.get("MIGRATION_MODE", "warm").lower()

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

# Predictive handoff: fire the obstacle transfer this many seconds before the
# NPC's constant-velocity trajectory exits locale 0. Chosen to fire while RSU0
# still has an active NPC track (~1.0s = 18m at cruising speed, inside the
# x=90-115 reacquisition window after moving RSU0 to x=55).
OBSTACLE_HANDOFF_LOOKAHEAD_S = float(os.environ.get('LOOKAHEAD_S', 1.0))

# ── Trigger-axis / refresh knobs (dissertation Q3 extension) ──
# TRIGGER_MODE: 'predictive' projects the constant-velocity trajectory and
# fires LOOKAHEAD_S before the predicted exit (default). 'band' is the
# geometric alternative: fire when the NPC is within BAND_W_M meters of the
# source-locale boundary, no trajectory involved. Both reuse MIGRATION_MODE's
# payload semantics.
# Scenario env contract DEFAULTS: the validated flow geometry requires a
# 12 m/s uniform oncoming stream and the 300 m actor trigger. The uniform
# sweep invoked this runner bare and silently reverted to mixed speeds and
# the 150 m trigger, reproducing the pre-fix commit geometry (measured:
# bare 762 contact ticks vs contract 0, same code). Defaults make a bare
# invocation THE validated scenario; explicit envs still override.
os.environ.setdefault('ONCOMING_SPEED', '12')
os.environ.setdefault('TRIGGER_DIST', '300')
TRIGGER_MODE = os.environ.get('TRIGGER_MODE', 'predictive').lower()
BAND_W_M = float(os.environ.get('BAND_W_M', 20.0))
# COMMIT_REFRESH='full' re-sends the full state once at the actual crossing
# (the "phase 2" question: does a refresh at commit buy anything at this
# payload size). Default 'none'.
COMMIT_REFRESH = os.environ.get('COMMIT_REFRESH', 'none').lower()
# MIRROR_PERIOD_S>0 re-sends the state every P seconds after the first
# trigger until the crossing (the mirroring-rate axis: trigger-once at P=0
# through standby replication as P shrinks).
MIRROR_PERIOD_S = float(os.environ.get('MIRROR_PERIOD_S', 0.0))

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

    Returns ``(router, edge_by_locale, locale_by_id)`` where
    ``edge_by_locale`` maps ``locale_id`` to its hosting edge instance and
    ``locale_by_id`` maps ``locale_id`` to the ``Locale`` object (needed by
    the predictive trigger to call ``predicted_to_exit_within``).
    """
    registry = LocaleRegistry()
    edge_by_locale = {}
    locale_by_id = {}
    for i, edge_cfg in enumerate(edge_cfgs):
        if 'locale' not in edge_cfg:
            continue
        lc = edge_cfg['locale']
        polygon = np.array([[float(v) for v in row] for row in lc['polygon']],
                           dtype=np.float64)
        locale_id = str(lc['id'])
        locale = Locale(
            locale_id=locale_id,
            polygon=polygon,
            edge_host_id=edge_list[i].edgeid,
        )
        registry.register(locale)
        edge_by_locale[locale_id] = edge_list[i]
        locale_by_id[locale_id] = locale
    return LocaleRouter(registry), edge_by_locale, locale_by_id


def _npc_track_on_edge(edge, carla_id, gt_loc, pos_gate_m=6.0):
    """Return the edge's tracklet for carla_id.

    Identity first (resolved carla_id) so a warm-imported track counts as
    PRESENT even while it coasts far from ground truth; position fallback
    (pos_gate_m) for the cold arm where identity was never stamped. This
    separates presence (does the edge hold the track) from accuracy
    (its position error, computed downstream from the logged state).
    """
    rf = getattr(edge, '_raw_tracker', None)
    if rf is None:
        return None
    pool = getattr(rf(), 'tracked_tracklets', None)
    if not pool:
        return None
    for t in pool:
        if edge._resolved_carla_id(getattr(t, 'carla_id', None)) == carla_id:
            return t
    best, bd = None, pos_gate_m
    for t in pool:
        d = ((float(t.state[0]) - gt_loc.x) ** 2 +
             (float(t.state[1]) - gt_loc.y) ** 2) ** 0.5
        if d < bd:
            best, bd = t, d
    return best


def run_scenario(opt, scenario_params):
    """Run the Town06 obstacle-handoff scenario (sequential mode only)."""
    global scenario_runner
    # Q6 impairment knobs: env-driven backhaul latency/jitter/loss applied
    # to every edge before manager construction.
    _lat = os.environ.get('NET_LAT_MS')
    _jit = os.environ.get('NET_JITTER_MS')
    _loss = os.environ.get('NET_LOSS_PCT')
    if _lat or _jit or _loss:
        for _e in scenario_params['scenario']['edge_list']:
            if _lat:
                _e['latency'] = float(_lat) / 1000.0
            if _jit:
                _e['jitter_std'] = float(_jit) / 1000.0
            if _loss:
                _e['uplink_packet_loss_pct'] = float(_loss)
                _e['downlink_packet_loss_pct'] = float(_loss)
    cav_world = None
    scenario_manager = None
    eval_manager = None
    sr_process = None
    edge_list = []
    step = 0
    fusion_clients = []

    metrics_logger = None
    npc_ids = set()            # moving non-hero, non-managed vehicles (Leons)
    npc_locale = {}            # carla_id -> sticky source locale id
    npc_handoff_done = {}      # carla_id -> (tick, dst_locale_id)
    npc_rsu_detect_tick = {}   # carla_id -> tick dst-RSU came in range
    npc_refresh_done = {}      # carla_id -> tick of the commit refresh
    npc_mirror_last_tick = {}  # carla_id -> tick of the last mirror resend

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
        # Determinism hardening: freeze every traffic light green so the
        # oncoming gap structure comes from the seeded spawn layout, not
        # from whatever light phase the world clock happened to be in at
        # episode start (measured: phase-coupled bunching made absolute
        # rates irreproducible across identical configs).
        for _tl in world.get_actors().filter('traffic.traffic_light*'):
            try:
                import carla as _carla
                _tl.set_state(_carla.TrafficLightState.Green)
                _tl.freeze(True)
            except Exception:  # noqa: BLE001
                pass
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
        router, edge_by_locale, locale_by_id = _build_locale_router(edge_cfgs, edge_list)
        locale_tracker = VehicleLocaleTracker(router, min_dwell_ticks=LOCALE_MIN_DWELL_TICKS)
        daemon = SequentialMigrationDaemon()
        metrics_logger = MigrationMetricsLogger(MIGRATION_MODE, -1)
        logger.info(
            "[MIGRATION] mode=%s trigger=%s band_w=%.1f refresh=%s "
            "mirror_period=%.2f lookahead=%.2f",
            MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
            MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S)
        link = InterLocaleLink(edge_list[0].latency_model)

        # Per-locale RSU positions for the advance-warning proxy.
        rsu_pos_by_locale = {}
        for ec in edge_cfgs:
            if ec.get('rsus'):
                sp = ec['rsus'][0]['spawn_position']
                rsu_pos_by_locale[ec['locale']['id']] = (float(sp[0]), float(sp[1]))

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

            # T7 measurement: count double-publish windows — ticks where
            # more than one edge holds a publishable epoch for the same
            # track. Zero with fencing on; the FENCING=off arm reports the
            # unfenced baseline.
            _dbl = getattr(run_scenario, '_dbl_publish_ticks', {})
            for _nid2 in list(npc_ids):
                _pubs = []
                for _e2 in edge_list:
                    _own = getattr(_e2, 'ownership', None)
                    if _own is not None:
                        _pubs += _own.publishable_epochs(_nid2)
                if len(_pubs) > 1:
                    _dbl[_nid2] = _dbl.get(_nid2, 0) + 1
            run_scenario._dbl_publish_ticks = _dbl

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

            # Resolve moving non-hero, non-managed NPCs (the oncoming Leons).
            if step > 10 and step % 5 == 0:
                managed = {vm.vehicle.id for e in edge_list
                           for vm in e.vehicle_manager_list}
                for v in world.get_actors().filter('vehicle.*'):
                    if v.id in npc_ids or v.id in managed \
                            or v.attributes.get('role_name') == 'hero':
                        continue
                    vel = v.get_velocity()
                    if (vel.x ** 2 + vel.y ** 2) ** 0.5 > NPC_MIN_SPEED_MPS:
                        npc_ids.add(v.id)
                        logger.info("[SCENB] NPC carla_id=%d resolved at tick=%d",
                                    v.id, step)

            # Per-NPC predictive transfer: when the projected trajectory
            # exits the current locale within the lookahead, ship the
            # latent to the neighbor while the source still has a track.
            for nid in list(npc_ids):
                actor = world.get_actor(nid)
                if actor is None:
                    continue
                nloc = actor.get_transform().location
                nxy = (nloc.x, nloc.y)

                if nid in npc_handoff_done:
                    # advance-warning bookkeeping vs the destination RSU
                    if nid not in npc_rsu_detect_tick:
                        htick, dst_lid = npc_handoff_done[nid]
                        rp = rsu_pos_by_locale.get(dst_lid)
                        if rp is not None:
                            d = ((nxy[0] - rp[0]) ** 2
                                 + (nxy[1] - rp[1]) ** 2) ** 0.5
                            if d <= RSU1_DETECT_RANGE_M:
                                npc_rsu_detect_tick[nid] = step
                                logger.info(
                                    "[SCENB] NPC %d advance-warning = %d ticks "
                                    "(handoff=%d, %s RSU in-range=%d)",
                                    nid, step - htick, htick, dst_lid, step)
                    # Post-trigger resend policies. Both re-fire the same
                    # transfer path, so bytes land in record_handoff_cost.
                    if (COMMIT_REFRESH == 'full' or MIRROR_PERIOD_S > 0.0) \
                            and nid not in npc_refresh_done \
                            and MIGRATION_MODE != 'cold':
                        _htick, _dst_lid = npc_handoff_done[nid]
                        _src_lid2 = next(
                            (l for l in locale_by_id if l != _dst_lid), None)
                        _se = edge_by_locale.get(_src_lid2)
                        _de = edge_by_locale.get(_dst_lid)
                        _crossed = locale_by_id[_dst_lid].contains(nxy)
                        if COMMIT_REFRESH == 'full' and _crossed \
                                and _se is not None and _de is not None:
                            _c = daemon.transfer_obstacle_state(
                                nid, _se, _de, link, step, position=nxy)
                            npc_refresh_done[nid] = step
                            if _c is not None:
                                scenario_manager.record_handoff_cost(_c)
                                logger.info(
                                    "[SCENB] COMMIT REFRESH tick=%d npc=%d "
                                    "%s->%s bytes=%d", step, nid, _src_lid2,
                                    _dst_lid, _c.payload_bytes)
                        elif MIRROR_PERIOD_S > 0.0 and not _crossed \
                                and _se is not None and _de is not None:
                            _last = npc_mirror_last_tick.get(nid, _htick)
                            if (step - _last) * world_dt >= MIRROR_PERIOD_S:
                                _c = daemon.transfer_obstacle_state(
                                    nid, _se, _de, link, step, position=nxy)
                                npc_mirror_last_tick[nid] = step
                                if _c is not None:
                                    scenario_manager.record_handoff_cost(_c)
                                    logger.info(
                                        "[SCENB] MIRROR RESEND tick=%d npc=%d "
                                        "%s->%s bytes=%d", step, nid,
                                        _src_lid2, _dst_lid, _c.payload_bytes)
                    continue

                # Sticky assignment: update only when containment is
                # unambiguous. next()-style resolution flipped eastbound
                # NPCs to locale_0 on entering the 240-250 overlap, which
                # shrank the exit-prediction window to ~2 ticks (measured:
                # both Leons crossed silently).
                _cur = [lid for lid, loc_ in locale_by_id.items()
                        if loc_.contains(nxy)]
                if len(_cur) == 1:
                    npc_locale[nid] = _cur[0]
                src_lid = npc_locale.get(nid)
                if src_lid is None:
                    continue
                dst_lid = next((l for l in locale_by_id if l != src_lid), None)
                if dst_lid is None:
                    continue
                nvel = actor.get_velocity()
                # Trigger lead: OURS fires LOOKAHEAD_S before the actor is
                # projected to cross (predictive, on the OBSTACLE trajectory).
                # REACTIVE (EdgeWarp-degenerate) fires only AT the crossing:
                # EdgeWarp's mobility hint is about the session client, not an
                # observed obstacle, and the tracker latent is all-dynamic
                # (nothing to background-sync), so for this content it reduces
                # to a blocking transfer at the boundary with no lead.
                if TRIGGER_MODE == 'band' and MIGRATION_MODE not in (
                        "reactive", "edgewarp"):
                    # Geometric trigger: within BAND_W_M of the CROSSING
                    # boundary, measured as distance to the destination
                    # locale. Distance to the source's own polygon would
                    # fire on the road's lateral edges for every vehicle
                    # (measured: NPC 79 m from the crossing fired at W=20),
                    # which is the boundary-parallel over-firing failure.
                    _sd_dst = locale_by_id[dst_lid].signed_distance(nxy)
                    if _sd_dst > BAND_W_M:
                        continue
                elif MIGRATION_MODE in ("reactive", "edgewarp"):
                    # At-crossing transfer: fire when the NPC has actually
                    # entered the destination locale. The old zero-lead
                    # projection (horizon_s=0) could never fire, so these
                    # arms silently ran as cold in this scenario.
                    if not locale_by_id[dst_lid].contains(nxy):
                        continue
                else:
                    if TRIGGER_MODE == 'computed':
                        # Budget-computed lead: fire prepare exactly when the
                        # predicted time-to-crossing equals the measured
                        # migration budget - transfer time (EMA over prior
                        # handoffs this run) plus the destination fold-in
                        # (edge cycles to ingest the latent into its own
                        # fusion/tracking loop) plus a safety margin. Replaces
                        # the fixed lookahead: fast vehicles fire earlier in
                        # distance, slow ones later, and the lead never
                        # exceeds what warmth requires (the measured cost of
                        # too-early transfer is staleness on arrival).
                        _xfer_s = getattr(run_scenario, '_xfer_ema_s', 0.05)
                        _fold_s = 3 * 0.2   # 3 edge cycles at edge_dt
                        _lead = min(2.5, _xfer_s + _fold_s + 0.35)
                        logger.info(
                            "[LEADROW] npc=%d lead=%.3f xfer_ema=%.3f "
                            "fold=%.3f margin=0.350 tick=%d",
                            nid, _lead, _xfer_s, _fold_s, step)
                    else:
                        _lead = OBSTACLE_HANDOFF_LOOKAHEAD_S
                    n_steps = int(_lead / world_dt) + 1
                    t_arr = np.arange(n_steps, dtype=np.float64) * world_dt
                    traj = np.column_stack([nloc.x + nvel.x * t_arr,
                                            nloc.y + nvel.y * t_arr])
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            traj, _lead, world_dt):
                        continue
                src_edge = edge_by_locale.get(src_lid)
                dst_edge = edge_by_locale.get(dst_lid)
                if src_edge is None or dst_edge is None or src_edge is dst_edge:
                    continue
                cost = None if MIGRATION_MODE == "cold" \
                    else daemon.transfer_obstacle_state(
                        nid, src_edge, dst_edge, link, step, position=nxy)
                if cost is not None:
                    scenario_manager.record_handoff_cost(cost)
                    # transfer-time EMA feeds the computed trigger's budget
                    _prev = getattr(run_scenario, '_xfer_ema_s', None)
                    _cur_s = cost.total_ms / 1000.0
                    run_scenario._xfer_ema_s = _cur_s if _prev is None \
                        else 0.3 * _cur_s + 0.7 * _prev
                    npc_handoff_done[nid] = (step, dst_lid)
                    logger.info(
                        "[SCENB] PREDICTIVE OBSTACLE HANDOFF tick=%d "
                        "carla_id=%d npc=(%.1f,%.1f) %s->%s bytes=%d "
                        "total_ms=%.3f",
                        step, nid, nxy[0], nxy[1], src_lid, dst_lid,
                        cost.payload_bytes, cost.total_ms)
                elif step % 10 == 0:
                    logger.warning(
                        "[SCENB-DBG] predictive tick=%d npc=%d (%.1f,%.1f) — "
                        "no track on %s yet",
                        step, nid, nxy[0], nxy[1], src_lid)


            for _nid in list(npc_ids):
                _a = world.get_actor(_nid)
                if _a is None:
                    continue
                _g = _a.get_transform().location
                for _edge in edge_list:
                    _trk = _npc_track_on_edge(_edge, _nid, _g)
                    metrics_logger.log_frame(
                        step, _edge.edgeid, _nid, (_g.x, _g.y, _g.z),
                        _trk, plain_axes=True)

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

        # Per-NPC hand-off summary.
        if npc_handoff_done:
            for nid, (htick, dst_lid) in npc_handoff_done.items():
                dtick = npc_rsu_detect_tick.get(nid)
                if dtick is not None:
                    logger.info(
                        "[SCENB] NPC %d advance-warning = %d ticks "
                        "(handoff=%d, %s RSU in-range=%d)",
                        nid, dtick - htick, htick, dst_lid, dtick)
                else:
                    logger.info(
                        "[SCENB] NPC %d handoff at tick=%d; never entered "
                        "dst-RSU range this run", nid, htick)
        else:
            logger.warning("[SCENB] no obstacle handoff fired "
                           "(npcs=%d, steps=%d)", len(npc_ids), step)

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

        if metrics_logger is not None:
            try:
                out_dir = os.path.join(
                    'evaluation_outputs',
                    f"migration_{scenario_params['current_time']}")
                metrics_logger.dump(out_dir)
            except Exception:  # noqa: BLE001
                logger.exception("migration metrics dump failed")

        # One machine-readable row per run so the sweep extractor never
        # reverse-engineers logs (KB: "fix the eval to log episodes").
        try:
            _hist = []
            for edge in edge_list:
                for vm in edge.vehicle_manager_list:
                    _sm = getattr(vm, 'safety_manager', None)
                    if _sm is None:
                        continue
                    for _sens in _sm.sensors:
                        if hasattr(_sens, '_history'):
                            _hist.extend(f for (f, *_rest) in _sens._history)
            _hist.sort()
            _contact = len(_hist)
            _eps, _prev = 0, None
            for _f in _hist:
                if _prev is None or _f - _prev > 2:
                    _eps += 1
                _prev = _f
            _tbytes = sum(c.payload_bytes for c in transfer_costs)
            import os as _os2
            _dblsum = sum(getattr(run_scenario, '_dbl_publish_ticks',
                                  {}).values())
            logger.info(
                "[RUNROW] mode=%s trigger=%s band_w=%.1f refresh=%s "
                "mirror=%.2f lookahead=%.2f episodes=%d contact_ticks=%d "
                "transfers=%d bytes=%d fault=%s fencing=%s dbl_ticks=%d",
                MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
                MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S,
                _eps, _contact, len(transfer_costs), _tbytes,
                _os2.environ.get('FAULT_MODE', 'none') or 'none',
                _os2.environ.get('FENCING', 'on'), _dblsum)
        except Exception:  # noqa: BLE001
            logger.exception("RUNROW emission failed")

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
