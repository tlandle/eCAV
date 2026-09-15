# -*- coding: utf-8 -*-
# Author: Jordan Rapp + Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Reduced three-locale CORRIDOR obstacle-handoff runner (Town01).

Forked from runner_1l.py (the freeze-1k+1l flow overlay). Two functional
changes make the two-locale obstacle path chain across N-1 boundaries:

1. Corridor next hop. Both two-locale ``next((l for l in locale_by_id if
   l != ...))`` heuristics are replaced by ``_corridor_next_hop``: locales are
   ordered by centroid x and the destination is the immediate neighbour in the
   NPC's direction of travel (sign of nvel.x). At N=3 the old heuristic picked
   an arbitrary non-source locale; the ordered next hop routes each crossing to
   the correct adjacent edge.

2. Per-crossing re-arm. ``npc_handoff_done`` is keyed by ``(nid, src_lid)``
   instead of ``nid``, and the sticky source-locale recompute is moved ABOVE the
   handoff-done guard. A track handed off out of locale_2 therefore re-arms once
   its sticky source advances to locale_1 and hands off again at the next
   boundary. The ego path (VehicleLocaleTracker) is untouched (already chains),
   and the ownership epoch / shadow / fencing path is untouched (chains
   natively: each crossing is a new src/dst edge pair, so the source increments
   the epoch and installs a fresh shadow at the destination).

Per-crossing rows are emitted as [CORRIDORCROSS] (one per crossing, with a
1-based crossing_index) for the corridor lander; [RUNROW]/[COMPUTEROW]/
[FAULTROW] are unchanged.
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

MAX_STEP = 2200  # corridor route (~400 m, two overtakes) runs longer than flow
SCENARIO_NAME = 'openscenario_1_corridor_gt'
MIGRATION_MODE = os.environ.get("MIGRATION_MODE", "warm").lower()

# Speed (m/s) above which a non-hero vehicle is taken to be an oncoming NPC.
NPC_MIN_SPEED_MPS = 2.0

# Geometry proxy for "the destination RSU could first directly detect the NPC".
RSU1_DETECT_RANGE_M = 60.0

# Locale hysteresis: consecutive ticks in the destination locale before the
# vehicle tracker fires the crossing event (0.2 s at 20 Hz).
LOCALE_MIN_DWELL_TICKS = 4

# Predictive handoff lookahead (s).
OBSTACLE_HANDOFF_LOOKAHEAD_S = float(os.environ.get('LOOKAHEAD_S', 1.0))

# Scenario env contract DEFAULTS (validated flow geometry): 12 m/s uniform
# oncoming stream and the 300 m actor trigger. A bare invocation is the
# validated scenario; explicit envs still override.
os.environ.setdefault('ONCOMING_SPEED', '12')
os.environ.setdefault('TRIGGER_DIST', '300')
TRIGGER_MODE = os.environ.get('TRIGGER_MODE', 'predictive').lower()
MTR_THETA = float(os.environ.get('MTR_THETA', 0.5))
# At-crossing FALLBACK for the warm arm (Tyler, 2026-09-14), corridor port:
# per hop (key (npc, source locale)), when the road user is inside the next
# locale (the overlap band) or has changed locale with no PREPARE fired for
# that hop, fire the reactive rule (the full record to the entered locale;
# the final update follows at commit as for every warm handoff), so Khonsu is
# never worse than history at crossing. [FALLBACKROW] logs the reason and
# what a guess would have done. Default on for warm; FALLBACK_AT_CROSSING=0
# restores the untransferred case. DEADLINE_GUESS=1 (rule 3, default off).
FALLBACK_AT_CROSSING = os.environ.get('FALLBACK_AT_CROSSING', '1') != '0'
DEADLINE_GUESS = os.environ.get('DEADLINE_GUESS', '0') == '1'
BAND_W_M = float(os.environ.get('BAND_W_M', 20.0))
COMMIT_REFRESH = os.environ.get('COMMIT_REFRESH', 'none').lower()
MIRROR_PERIOD_S = float(os.environ.get('MIRROR_PERIOD_S', 0.0))
# Replication baselines: source mirrors the FULL record every REPL_PERIOD_S with
# no single prepare trigger. repl_final adds a final sync at the crossing.
IS_REPL = MIGRATION_MODE in ('replication', 'repl_final')
REPL_PERIOD_S = float(os.environ.get('REPL_PERIOD_S', 0.2))
# Geometry sweep: shift the locale boundaries by this many metres along x.
ONCOMING_SHIFT_X = float(os.environ.get('ONCOMING_SHIFT_X', 0.0))

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

    Returns ``(router, edge_by_locale, locale_by_id)``.
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
        if ONCOMING_SHIFT_X:
            polygon[:, 0] = polygon[:, 0] + ONCOMING_SHIFT_X
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


def _corridor_next_hop(locale_by_id, src_lid, travel_sign):
    """Return the adjacent locale in the corridor in the direction of travel.

    Locales are ordered along the corridor by centroid x (ascending, west to
    east). ``travel_sign`` is the sign of the NPC's along-corridor velocity
    (nvel.x): +1 moves toward increasing x (east), -1 toward decreasing x
    (west). Returns the id of the immediate neighbour in that direction, or
    ``None`` at the end of the corridor. Replaces the two-locale
    ``next((l for l in locale_by_id if l != ...))`` heuristics.
    """
    order = sorted(locale_by_id,
                   key=lambda lid: float(locale_by_id[lid].polygon[:, 0].mean()))
    if src_lid not in order:
        return None
    i = order.index(src_lid)
    j = i + 1 if travel_sign >= 0 else i - 1
    if 0 <= j < len(order):
        return order[j]
    return None


def _npc_track_on_edge(edge, carla_id, gt_loc, pos_gate_m=6.0):
    """Return the edge's tracklet for carla_id (identity first, position gate)."""
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
    """Run the three-locale corridor obstacle-handoff scenario (sequential)."""
    global scenario_runner
    # Q6 impairment knobs applied to every edge before manager construction.
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
    # Per-crossing bookkeeping keyed by hk = (carla_id, src_locale_id) so an NPC
    # can hand off once PER SOURCE LOCALE (chained across boundaries).
    npc_handoff_done = {}      # hk -> (prepare_tick, dst_locale_id)
    npc_rsu_detect_tick = {}   # hk -> tick dst-RSU came in range
    npc_refresh_done = {}      # hk -> tick of the commit refresh / final sync
    npc_mirror_last_tick = {}  # hk -> tick of the last mirror resend
    xfer_bytes_by_hk = {}      # hk -> total transfer bytes for this crossing
    epoch_by_hk = {}           # hk -> ownership epoch installed at the dst edge
    t19_first_dst = {}         # hk -> tick dst edge first maps a track to nid
    t19_first_use = {}         # hk -> tick ego first consumes a forecast for nid
    t19_crossing = {}          # hk -> tick containment flipped to destination

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
        # Determinism hardening: freeze every traffic light green.
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

        # Handoff primitives (after edge_list is populated).
        edge_cfgs = scenario_params['scenario']['edge_list']
        router, edge_by_locale, locale_by_id = _build_locale_router(edge_cfgs, edge_list)
        locale_tracker = VehicleLocaleTracker(router, min_dwell_ticks=LOCALE_MIN_DWELL_TICKS)
        daemon = SequentialMigrationDaemon()
        metrics_logger = MigrationMetricsLogger(MIGRATION_MODE, -1)
        logger.info(
            "[MIGRATION] mode=%s trigger=%s band_w=%.1f refresh=%s "
            "mirror_period=%.2f lookahead=%.2f locales=%d",
            MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
            MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S, len(locale_by_id))
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

            # T7 measurement: count double-publish windows (ticks where more than
            # one edge holds a publishable epoch for the same track). Keyed by
            # carla_id (ownership token), unchanged from flow.
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

            # Per-crossing firsts, keyed by hk = (nid, src_lid).
            for hk3, (ph3, _dlid3) in list(npc_handoff_done.items()):
                _nid3 = hk3[0]
                _de3 = edge_by_locale.get(_dlid3)
                if hk3 not in t19_first_dst:
                    if _de3 is not None and any(
                            _c == _nid3 for _c in
                            getattr(_de3, 'track_to_carla', {}).values()):
                        t19_first_dst[hk3] = step
                if hk3 not in t19_crossing:
                    _a3 = world.get_actor(_nid3)
                    if _a3 is not None and locale_by_id[_dlid3].contains(
                            (_a3.get_transform().location.x,
                             _a3.get_transform().location.y)):
                        t19_crossing[hk3] = step
                        # COMMIT: actor entered the dest locale -> the imported
                        # track becomes publishable (shadow cleared).
                        if _de3 is not None and hasattr(_de3, '_shadow_obstacles'):
                            _de3._shadow_obstacles[int(_nid3)] = False
                if hk3 not in t19_first_use and step >= ph3:
                    _found = False
                    for _e3 in edge_list:
                        for _vm3 in _e3.vehicle_manager_list:
                            for _p3 in getattr(_vm3.agent,
                                               'generated_predictions', []):
                                _o3 = _p3.obstacle_trajectory.obstacle
                                if getattr(_o3, 'carla_id', -1) == _nid3:
                                    t19_first_use[hk3] = step
                                    _found = True
                                    break
                            if _found:
                                break
                        if _found:
                            break

            # Per-tick snapshot upload (keeps the store warm).
            for edge in edge_list:
                for vm in edge.vehicle_manager_list:
                    payload = edge.export_vehicle_state(vm.vehicle.id)
                    if payload:
                        scenario_manager.store_vehicle_state(vm.vehicle.id, payload)

            # ── Vehicle (ego) handoff: geometry-driven via VehicleLocaleTracker.
            # UNCHANGED from flow: the tracker rebinds src->dst on every crossing,
            # so the arriving destination becomes the source for the next
            # boundary automatically (chains across N-1 crossings as-is).
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
                        _vl = v.get_transform().location
                        _sds = [locale_by_id[_l].signed_distance((_vl.x, _vl.y)) for _l in locale_by_id]
                        _nd = min([_x for _x in _sds if _x > 0.0], default=-1.0)
                        _rd = getattr(run_scenario, '_npc_resolve_dist', None)
                        if _rd is None:
                            _rd = run_scenario._npc_resolve_dist = {}
                        _rd[v.id] = _nd
                        logger.info("[SCENB] NPC carla_id=%d resolved at tick=%d dist_to_boundary_m=%.1f",
                                    v.id, step, _nd)

            # Per-NPC obstacle handoff, chained across boundaries.
            for nid in list(npc_ids):
                actor = world.get_actor(nid)
                if actor is None:
                    continue
                nloc = actor.get_transform().location
                nxy = (nloc.x, nloc.y)
                nvel = actor.get_velocity()

                # Sticky source-locale assignment, recomputed EVERY tick and
                # BEFORE the handoff-done guard so an NPC that has advanced into a
                # new locale after a prior handoff re-arms for the next boundary.
                # Update only when containment is unambiguous (single locale) to
                # avoid the overlap band flipping the source.
                _cur = [lid for lid, loc_ in locale_by_id.items()
                        if loc_.contains(nxy)]
                if len(_cur) == 1:
                    npc_locale[nid] = _cur[0]
                src_lid = npc_locale.get(nid)
                hk = (nid, src_lid) if src_lid is not None else None

                # Post-handoff bookkeeping for THIS crossing (keyed by hk):
                # advance-warning + final-sync/mirror, then continue. When the
                # sticky source advances to the next locale, hk changes and this
                # branch is skipped -> the next boundary re-arms below.
                if hk is not None and hk in npc_handoff_done:
                    _htick, _dst_lid = npc_handoff_done[hk]
                    _src_lid2 = src_lid  # source is the crossing key, not a guess
                    if hk not in npc_rsu_detect_tick:
                        rp = rsu_pos_by_locale.get(_dst_lid)
                        if rp is not None:
                            d = ((nxy[0] - rp[0]) ** 2
                                 + (nxy[1] - rp[1]) ** 2) ** 0.5
                            if d <= RSU1_DETECT_RANGE_M:
                                npc_rsu_detect_tick[hk] = step
                                logger.info(
                                    "[SCENB] NPC %d advance-warning = %d ticks "
                                    "(handoff=%d, %s RSU in-range=%d)",
                                    nid, step - _htick, _htick, _dst_lid, step)
                    # FINAL UPDATE at commit: warm and the final-sync arms send
                    # the source's latest record when the actor enters the
                    # destination. Plain kf / plain replication are NOT in the
                    # set (no final sync).
                    _final_sync = MIGRATION_MODE in (
                        'warm', 'edgewarp', 'edgewarp_full',
                        'repl_final', 'kf_final')
                    _mirror_period = REPL_PERIOD_S if IS_REPL else MIRROR_PERIOD_S
                    if (COMMIT_REFRESH == 'full' or _mirror_period > 0.0
                            or _final_sync) \
                            and hk not in npc_refresh_done \
                            and MIGRATION_MODE not in ('cold', 'warm_nofinal'):
                        _se = edge_by_locale.get(_src_lid2)
                        _de = edge_by_locale.get(_dst_lid)
                        _crossed = locale_by_id[_dst_lid].contains(nxy)
                        if (COMMIT_REFRESH == 'full' or _final_sync) \
                                and _crossed \
                                and _se is not None and _de is not None:
                            _c = daemon.transfer_obstacle_state(
                                nid, _se, _de, link, step, position=nxy,
                                committed=True)
                            npc_refresh_done[hk] = step
                            if _c is not None:
                                scenario_manager.record_handoff_cost(_c)
                                xfer_bytes_by_hk[hk] = \
                                    xfer_bytes_by_hk.get(hk, 0) + _c.payload_bytes
                                logger.info(
                                    "[SCENB] COMMIT REFRESH tick=%d npc=%d "
                                    "%s->%s bytes=%d", step, nid, _src_lid2,
                                    _dst_lid, _c.payload_bytes)
                        elif _mirror_period > 0.0 and not _crossed \
                                and _se is not None and _de is not None:
                            _last = npc_mirror_last_tick.get(hk, _htick)
                            if (step - _last) * world_dt >= _mirror_period:
                                _c = daemon.transfer_obstacle_state(
                                    nid, _se, _de, link, step, position=nxy)
                                npc_mirror_last_tick[hk] = step
                                if _c is not None:
                                    scenario_manager.record_handoff_cost(_c)
                                    xfer_bytes_by_hk[hk] = \
                                        xfer_bytes_by_hk.get(hk, 0) \
                                        + _c.payload_bytes
                                    logger.info(
                                        "[SCENB] MIRROR RESEND tick=%d npc=%d "
                                        "%s->%s bytes=%d", step, nid,
                                        _src_lid2, _dst_lid, _c.payload_bytes)
                    continue

                if src_lid is None:
                    continue
                # Corridor next hop: the adjacent locale in the direction of
                # travel (sign of nvel.x). Replaces the two-locale
                # next((l != src_lid)) heuristic.
                dst_lid = _corridor_next_hop(
                    locale_by_id, src_lid, 1.0 if nvel.x >= 0.0 else -1.0)
                # ---- at-crossing fallback (warm arm), per hop ----------------
                _pl = getattr(run_scenario, '_npc_prev_locale', None)
                if _pl is None:
                    _pl = run_scenario._npc_prev_locale = {}
                _prev_l = _pl.get(nid)
                _pl[nid] = src_lid
                if FALLBACK_AT_CROSSING and MIGRATION_MODE == 'warm':
                    _lm = getattr(run_scenario, '_last_mtr_eval', {}).get(nid)
                    if TRIGGER_MODE == 'mtr':
                        _reason = ('no_forecast' if _lm is None else
                                   ('no_candidate_at_theta' if _lm[1] < MTR_THETA
                                    else 'lead_never_opened'))
                    else:
                        _reason = 'trigger_never_fired'
                    _fb_src = _fb_dst = None
                    if (_prev_l is not None and _prev_l != src_lid
                            and (nid, _prev_l) not in npc_handoff_done):
                        _fb_src, _fb_dst, _bk = _prev_l, src_lid, 1   # BACKSTOP: band skipped
                        logger.warning("[FALLBACK-MISS] tick=%d npc=%d from=%s to=%s (backstop fires)",
                                       step, nid, _prev_l, src_lid)
                    elif (dst_lid is not None and hk is not None and hk not in npc_handoff_done
                            and locale_by_id[dst_lid].contains(nxy)):
                        _fb_src, _fb_dst, _bk = src_lid, dst_lid, 0     # in the next hop's overlap band
                    if _fb_dst is not None:
                        _fse = edge_by_locale.get(_fb_src)
                        _fde = edge_by_locale.get(_fb_dst)
                        if _fse is not None and _fde is not None and _fse is not _fde:
                            _fc = daemon.transfer_obstacle_state(
                                nid, _fse, _fde, link, step, position=nxy)
                            _fhk = (nid, _fb_src)
                            npc_handoff_done[_fhk] = (step, _fb_dst)
                            if _fc is not None:
                                scenario_manager.record_handoff_cost(_fc)
                                xfer_bytes_by_hk[_fhk] = xfer_bytes_by_hk.get(_fhk, 0) + _fc.payload_bytes
                                logger.info(
                                    "[FALLBACKROW] tick=%d npc=%d entered=%s from=%s reason=%s "
                                    "trigger=%s argmax=%s p_argmax=%.3f last_eval_tick=%d "
                                    "spawn_dist_m=%.1f bytes=%d backstop=%d",
                                    step, nid, _fb_dst, _fb_src, _reason, TRIGGER_MODE,
                                    (_lm[2] if _lm else ''), (_lm[1] if _lm else -1.0),
                                    (_lm[0] if _lm else -1),
                                    getattr(run_scenario, '_npc_resolve_dist', {}).get(nid, -1.0),
                                    _fc.payload_bytes, _bk)
                                logger.info(
                                    "[SCENB] PREDICTIVE OBSTACLE HANDOFF tick=%d "
                                    "carla_id=%d npc=(%.1f,%.1f) %s->%s bytes=%d "
                                    "total_ms=%.3f epoch=-1 shadow=fallback fallback=1",
                                    step, nid, nxy[0], nxy[1], _fb_src, _fb_dst,
                                    _fc.payload_bytes, _fc.total_ms)
                        continue
                if dst_lid is None:
                    continue  # end of the corridor in the travel direction
                # Trigger lead (unchanged from flow).
                if IS_REPL:
                    # Continuous replication: fire the initial mirror on the
                    # first eligible cycle; the resend block mirrors thereafter.
                    pass
                elif TRIGGER_MODE == 'band' and MIGRATION_MODE not in (
                        "reactive", "handover_snapshot"):
                    _sd_dst = locale_by_id[dst_lid].signed_distance(nxy)
                    if _sd_dst > BAND_W_M:
                        continue
                elif TRIGGER_MODE == 'mtr':
                    _src_edge_m = edge_by_locale.get(src_lid)
                    _modes = None
                    if _src_edge_m is not None:
                        _pred = getattr(_src_edge_m, 'predictor', None)
                        _mm = getattr(_pred, 'last_mtr_modes', {}) \
                            if _pred is not None else {}
                        _tid = next((t for t, c in getattr(
                            _src_edge_m, 'track_to_carla', {}).items()
                            if c == nid), None)
                        if _tid is not None:
                            _modes = _mm.get(int(_tid))
                    if not _modes:
                        continue
                    _p_dst = sum(
                        p for (wx, wy, p) in _modes
                        if locale_by_id[dst_lid].contains((wx, wy)))
                    logger.info(
                        "[PDSTROW] tick=%d npc=%d p_dst=%.4f mtr_theta=%.3f "
                        "fired=%s", step, nid, _p_dst, MTR_THETA,
                        "YES" if _p_dst >= MTR_THETA else "no")
                    _lme = getattr(run_scenario, '_last_mtr_eval', None)
                    if _lme is None:
                        _lme = run_scenario._last_mtr_eval = {}
                    _lme[nid] = (step, _p_dst, dst_lid)   # last sample: what a guess would use
                    if _p_dst < MTR_THETA:
                        # DEADLINE GUESS (rule 3, default off): lead window closing
                        # below theta -> fire the next hop (the only candidate)
                        if not DEADLINE_GUESS:
                            continue
                        _xg = getattr(run_scenario, '_xfer_ema_s', 0.003)
                        _lg = min(2.5, max(0.6 + 0.35, _xg + 0.6 + 0.35))
                        _ng = int(_lg / world_dt) + 1
                        _tg = np.arange(_ng, dtype=np.float64) * world_dt
                        _trg = np.column_stack([nloc.x + nvel.x * _tg, nloc.y + nvel.y * _tg])
                        if not locale_by_id[src_lid].predicted_to_exit_within(_trg, _lg, world_dt):
                            continue
                        logger.info("[GUESSROW] tick=%d npc=%d dst=%s p_dst=%.3f theta=%.2f",
                                    step, nid, dst_lid, _p_dst, MTR_THETA)
                    _xfer_s = getattr(run_scenario, '_xfer_ema_s', 0.003)
                    _fold_s = 3 * 0.2
                    _lead_m = min(2.5, max(_fold_s + 0.35,
                                           _xfer_s + _fold_s + 0.35))
                    _ns_m = int(_lead_m / world_dt) + 1
                    _ta_m = np.arange(_ns_m, dtype=np.float64) * world_dt
                    _traj_m = np.column_stack([nloc.x + nvel.x * _ta_m,
                                              nloc.y + nvel.y * _ta_m])
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _traj_m, _lead_m, world_dt):
                        continue
                elif TRIGGER_MODE == 'oracle':
                    _L = 3 * 0.2 + 0.35
                    _ns = int(_L / world_dt) + 1
                    _ta = np.arange(_ns, dtype=np.float64) * world_dt
                    _gt = np.column_stack([nloc.x + nvel.x * _ta,
                                           nloc.y + nvel.y * _ta])
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _gt, _L, world_dt):
                        continue
                elif MIGRATION_MODE in ("reactive", "handover_snapshot"):
                    # At-crossing transfer: fire once the NPC has entered the
                    # destination locale (the overlap band, where the sticky
                    # source still resolves to src_lid).
                    if not locale_by_id[dst_lid].contains(nxy):
                        continue
                else:
                    if TRIGGER_MODE == 'computed':
                        _xfer_s = getattr(run_scenario, '_xfer_ema_s', 0.003)
                        _fold_s = 3 * 0.2
                        _lead = min(2.5, max(_fold_s + 0.35, _xfer_s + _fold_s + 0.35))
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
                # Record the crossing (re-arm key) whether or not state moved:
                # cold still crosses the boundary, it just carries no latent.
                npc_handoff_done[hk] = (step, dst_lid)
                if cost is not None:
                    scenario_manager.record_handoff_cost(cost)
                    _prev = getattr(run_scenario, '_xfer_ema_s', None)
                    _cur_s = cost.total_ms / 1000.0
                    run_scenario._xfer_ema_s = _cur_s if _prev is None \
                        else 0.3 * _cur_s + 0.7 * _prev
                    xfer_bytes_by_hk[hk] = \
                        xfer_bytes_by_hk.get(hk, 0) + cost.payload_bytes
                    # Readback (no ownership-logic change): the epoch the source
                    # incremented and the destination committed for this track.
                    # Chains natively -> crossing 1 installs epoch 1, crossing 2
                    # (whose source edge was crossing 1's destination) installs
                    # epoch 2. shadow=<installed-then-cleared at commit>.
                    _own_d = getattr(dst_edge, 'ownership', None)
                    _ep = (_own_d.tracks[nid].epoch
                           if _own_d is not None and nid in _own_d.tracks
                           else -1)
                    _shadow = (getattr(dst_edge, '_shadow_obstacles', {}) or {}
                               ).get(int(nid))
                    epoch_by_hk[hk] = _ep
                    logger.info(
                        "[SCENB] PREDICTIVE OBSTACLE HANDOFF tick=%d "
                        "carla_id=%d npc=(%.1f,%.1f) %s->%s bytes=%d "
                        "total_ms=%.3f epoch=%d shadow=%s",
                        step, nid, nxy[0], nxy[1], src_lid, dst_lid,
                        cost.payload_bytes, cost.total_ms, _ep, _shadow)
                else:
                    logger.info(
                        "[SCENB] COLD CROSSING tick=%d carla_id=%d "
                        "npc=(%.1f,%.1f) %s->%s (no transfer)",
                        step, nid, nxy[0], nxy[1], src_lid, dst_lid)

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

        # Per-crossing advance-warning summary.
        if npc_handoff_done:
            for hk, (htick, dst_lid) in npc_handoff_done.items():
                dtick = npc_rsu_detect_tick.get(hk)
                if dtick is not None:
                    logger.info(
                        "[SCENB] NPC %d (%s) advance-warning = %d ticks "
                        "(handoff=%d, %s RSU in-range=%d)",
                        hk[0], hk[1], dtick - htick, htick, dst_lid, dtick)
                else:
                    logger.info(
                        "[SCENB] NPC %d (%s) handoff at tick=%d; never entered "
                        "dst-RSU range this run", hk[0], hk[1], htick)
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

        # Machine-readable rows.
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
            for _ei, _edge in enumerate(edge_list):
                _cms = getattr(_edge, '_t21_compute_ms_sum', 0.0)
                _vs = getattr(_edge, '_t21_veh_seconds', 0.0)
                _cpvs = (_cms / 1000.0 / _vs) if _vs > 0 else 0.0
                logger.info(
                    "[COMPUTEROW] edge=%d locale=%s compute_s=%.3f "
                    "veh_seconds=%.3f compute_per_veh_s=%.4f",
                    _ei, getattr(_edge, 'edgeid', _ei), _cms / 1000.0,
                    _vs, _cpvs)

            # Per-crossing rows: order each NPC's crossings by prepare tick to
            # assign a 1-based crossing_index, then emit one [CORRIDORCROSS]
            # line per crossing for the corridor lander.
            _by_npc = {}
            for (_hn, _hs), (_ph, _dl) in npc_handoff_done.items():
                _by_npc.setdefault(_hn, []).append((_ph, _hs, _dl))
            for _nidc, _cl in _by_npc.items():
                _cl.sort(key=lambda r: r[0])
                for _ci, (_ph, _slid, _dlid) in enumerate(_cl, start=1):
                    _hk = (_nidc, _slid)
                    _fd = t19_first_dst.get(_hk, -1)
                    _fu = t19_first_use.get(_hk, -1)
                    _cx = t19_crossing.get(_hk, -1)
                    _warm = (_fd >= 0 and (_fu < 0 or _fd <= _fu)
                             and (_cx < 0 or _fd <= _cx))
                    _by = xfer_bytes_by_hk.get(_hk, 0)
                    _ep = epoch_by_hk.get(_hk, -1)
                    logger.info(
                        "[CORRIDORCROSS] npc=%d crossing_index=%d src=%s "
                        "dst=%s prepare_tick=%d crossing_tick=%d "
                        "first_dst_track_tick=%d first_use_tick=%d warm=%s "
                        "bytes=%d epoch=%d",
                        _nidc, _ci, _slid, _dlid, _ph, _cx, _fd, _fu,
                        "YES" if _warm else "no", _by, _ep)

            import os as _os2
            _dblsum = sum(getattr(run_scenario, '_dbl_publish_ticks',
                                  {}).values())
            _trigger_emit = 'continuous' if IS_REPL else TRIGGER_MODE
            _n_crossings = len(npc_handoff_done)
            logger.info(
                "[RUNROW] mode=%s trigger=%s band_w=%.1f refresh=%s "
                "mirror=%.2f lookahead=%.2f episodes=%d contact_ticks=%d "
                "transfers=%d bytes=%d fault=%s fencing=%s dbl_ticks=%d "
                "shift_x_m=%.2f crossings=%d",
                MIGRATION_MODE, _trigger_emit, BAND_W_M, COMMIT_REFRESH,
                MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S,
                _eps, _contact, len(transfer_costs), _tbytes,
                _os2.environ.get('FAULT_MODE', 'none') or 'none',
                _os2.environ.get('FENCING', 'on'), _dblsum,
                ONCOMING_SHIFT_X, _n_crossings)
            _fault = _os2.environ.get('FAULT_MODE', 'none') or 'none'
            _dual_now = 0
            for _nidf in list(npc_ids):
                _pf = []
                for _ef in edge_list:
                    _ownf = getattr(_ef, 'ownership', None)
                    if _ownf is not None:
                        _pf += _ownf.publishable_epochs(_nidf)
                if len(_pf) > 1:
                    _dual_now += 1
            _recovered = "YES" if (_fault != 'none' and _dual_now == 0) else (
                "no" if _fault != 'none' else "")
            logger.info(
                "[FAULTROW] fault=%s double_emission_ms=%.1f recovered=%s",
                _fault, _dblsum * world_dt * 1000.0, _recovered)
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
