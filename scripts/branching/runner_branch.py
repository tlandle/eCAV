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
SCENARIO_NAME = os.path.splitext(os.path.basename(__file__))[0]  # branching: copied per site
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
MTR_THETA = float(os.environ.get('MTR_THETA', 0.5))
# At-crossing FALLBACK for the warm arm (Tyler, 2026-09-14): when the road user
# enters a candidate locale (the mouth's overlap band) with no PREPARE fired for
# it, fire the reactive rule into the entered locale through the same fire loop
# (rule 'fallback'), so Khonsu is never worse than history at crossing.
# [FALLBACKROW] logs the reason and what a guess would have done. Default on for
# warm; FALLBACK_AT_CROSSING=0 restores the untransferred case.
FALLBACK_AT_CROSSING = os.environ.get('FALLBACK_AT_CROSSING', '1') != '0'
# DEADLINE_GUESS=1 (rule 3, default off): lead window closing with no candidate
# at theta -> fire the argmax candidate alone ([GUESSROW]).
DEADLINE_GUESS = os.environ.get('DEADLINE_GUESS', '0') == '1'
BAND_W_M = float(os.environ.get('BAND_W_M', 20.0))
# COMMIT_REFRESH='full' re-sends the full state once at the actual crossing
# (the "phase 2" question: does a refresh at commit buy anything at this
# payload size). Default 'none'.
COMMIT_REFRESH = os.environ.get('COMMIT_REFRESH', 'none').lower()
# MIRROR_PERIOD_S>0 re-sends the state every P seconds after the first
# trigger until the crossing (the mirroring-rate axis: trigger-once at P=0
# through standby replication as P shrinks).
MIRROR_PERIOD_S = float(os.environ.get('MIRROR_PERIOD_S', 0.0))
# ── Branching-junction harness (NSDI experiment 1). The source locale has k
# candidate destinations (yaml locale.neighbors; default: every other locale, which
# reproduces the two-locale runner exactly). Each arm decides a FIRE SET at its
# trigger tick; prepares go to every locale in the set; the transferred record is
# identical across arms (payload hash in HANDOFFROW).
BRANCH_TAKEN = os.environ.get('BRANCH_TAKEN', '')            # locale the scripted actor takes (oracle + scoring)
REPLICATE_ALL = os.environ.get('REPLICATE_ALL', '0') == '1'   # fire set = all candidates (brute-force arm)
MTR_TOP1 = os.environ.get('MTR_TOP1', '0') == '1'             # mtr: argmax MODE's branch only; same tick and L
CONF_LOG = os.environ.get('CONF_LOG', '1') == '1'             # [CONFROW] per tick in a branching source
CONFLICT_R_M = 3.0                                            # actor within this of the branch conflict point = conflict tick

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
    npc_handoff_done = {}
    # T19 per-handoff timing: first destination track, first ego use,
    # crossing tick — emitted as [HANDOFFROW] lines at scenario end.
    t19_first_dst = {}     # nid -> tick dst edge first maps a track to nid
    t19_first_use = {}     # nid -> tick ego first consumes a forecast for nid
    t19_crossing = {}      # nid -> tick containment flipped to destination      # carla_id -> (tick, dst_locale_id)
    npc_rsu_detect_tick = {}   # carla_id -> tick dst-RSU came in range
    npc_refresh_done = {}      # carla_id -> tick of the commit refresh
    npc_mirror_last_tick = {}  # carla_id -> tick of the last mirror resend
    # branching: per-(npc, destination) bookkeeping; t19_first_dst / t19_first_use
    # are keyed (nid, locale_id) below.
    npc_prepare_ticks = {}     # (nid, dst_lid) -> first prepare tick
    npc_prepare_hash = {}      # (nid, dst_lid) -> payload hash of that prepare
    npc_prepare_depth = {}     # (nid, dst_lid) -> record history depth (memo frames) of that prepare
    npc_conflict_tick = {}     # nid -> tick within CONFLICT_R_M of the entered branch's conflict point
    npc_entered = {}           # nid -> (tick, locale_id) first candidate locale containing the actor
    npc_trigger_tick = {}      # nid -> tick the arm's trigger fired
    npc_cv_pick = {}           # nid -> constant-velocity projected branch at the trigger tick

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

        while num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            num_actors = len(vehicles) + len(walkers)
        # branching: k heroes (one recipient CAV per branch). Scenario runner spawns
        # the XML ego list in order, so actor-id order is XML order; sim_api binds
        # hero[i] to edge_list[0]['vehicles'][i] via member_vehicles.
        _heroes = sorted([v for v in vehicles if v.attributes['role_name'] == 'hero'],
                         key=lambda v: v.id)
        assert _heroes, "no hero vehicle found"
        ego_vehicle = _heroes[0] if len(_heroes) == 1 else _heroes
        for _h in _heroes:
            _hl = _h.get_transform().location
            logger.info("[HEROROW] id=%d x=%.2f y=%.2f yaw=%.1f", _h.id, _hl.x, _hl.y,
                        _h.get_transform().rotation.yaw)
        print(f'Found all {num_actors} actors; heroes {[h.id for h in _heroes]}')

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
        # branching: candidate destinations per source locale + per-branch conflict points
        neighbors_by_locale, conflict_by_locale = {}, {}
        for ec in edge_cfgs:
            if 'locale' not in ec:
                continue
            _lid = str(ec['locale']['id'])
            _nb = ec['locale'].get('neighbors')
            neighbors_by_locale[_lid] = [str(n) for n in _nb] if _nb else \
                [l for l in locale_by_id if l != _lid]
            _cp = ec['locale'].get('conflict_point')
            if _cp is not None:
                conflict_by_locale[_lid] = (float(_cp[0]), float(_cp[1]))
        K_BRANCH = max((len(v) for v in neighbors_by_locale.values()), default=1)
        logger.info("[BRANCHCFG] taken=%s k=%d replicate_all=%d top1=%d theta=%.2f candidates=%s",
                    BRANCH_TAKEN, K_BRANCH, int(REPLICATE_ALL), int(MTR_TOP1), MTR_THETA,
                    {k: v for k, v in neighbors_by_locale.items()})

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

        spectator = world.get_spectator()
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

            # T19 firsts, branching form: first destination track per CANDIDATE
            # locale, crossing = first candidate that contains the actor (commit
            # there), conflict tick, first use per locale (the recipient CAV of
            # that locale consuming a forecast for the actor).
            for _nid3 in list(npc_ids):
                _src3 = npc_locale.get(_nid3)
                _cands3 = neighbors_by_locale.get(_src3, []) if _src3 else []
                for _dl3 in _cands3:
                    if (_nid3, _dl3) in t19_first_dst:
                        continue
                    _dst3 = edge_by_locale.get(_dl3)
                    if _dst3 is not None and any(
                            _c == _nid3 for _c in
                            getattr(_dst3, 'track_to_carla', {}).values()):
                        t19_first_dst[(_nid3, _dl3)] = step
                _a3 = world.get_actor(_nid3)
                if _a3 is None:
                    continue
                _l3 = _a3.get_transform().location
                if _nid3 not in npc_entered:
                    for _dl3 in _cands3:
                        if locale_by_id[_dl3].contains((_l3.x, _l3.y)):
                            npc_entered[_nid3] = (step, _dl3)
                            t19_crossing[_nid3] = step
                            # COMMIT (paper 3.5): the actor entered a destination
                            # locale -> its imported track (if any) becomes
                            # publishable there. Arm-independent.
                            _de3 = edge_by_locale.get(_dl3)
                            if _de3 is not None and hasattr(_de3, '_shadow_obstacles'):
                                _de3._shadow_obstacles[int(_nid3)] = False
                            logger.info("[ENTERROW] tick=%d npc=%d entered=%s taken=%s",
                                        step, _nid3, _dl3, BRANCH_TAKEN)
                            break
                if _nid3 in npc_entered and _nid3 not in npc_conflict_tick:
                    _cp3 = conflict_by_locale.get(npc_entered[_nid3][1])
                    if _cp3 is not None and ((_l3.x - _cp3[0]) ** 2 +
                                             (_l3.y - _cp3[1]) ** 2) ** 0.5 <= CONFLICT_R_M:
                        npc_conflict_tick[_nid3] = step
                for _lid3, _e3 in edge_by_locale.items():
                    if (_nid3, _lid3) in t19_first_use:
                        continue
                    _hit3 = False
                    for _vm3 in _e3.vehicle_manager_list:
                        for _p3 in getattr(_vm3.agent, 'generated_predictions', []):
                            _o3 = _p3.obstacle_trajectory.obstacle
                            if getattr(_o3, 'carla_id', -1) == _nid3:
                                _hit3 = True
                                break
                        if _hit3:
                            break
                    if _hit3:
                        t19_first_use[(_nid3, _lid3)] = step

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
                        _vl = v.get_transform().location
                        _sds = [locale_by_id[_l].signed_distance((_vl.x, _vl.y)) for _l in locale_by_id]
                        _nd = min([_x for _x in _sds if _x > 0.0], default=-1.0)
                        _rd = getattr(run_scenario, '_npc_resolve_dist', None)
                        if _rd is None:
                            _rd = run_scenario._npc_resolve_dist = {}
                        _rd[v.id] = _nd
                        logger.info("[SCENB] NPC carla_id=%d resolved at tick=%d dist_to_boundary_m=%.1f",
                                    v.id, step, _nd)

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
                        htick, _src_h, _dsts_h = npc_handoff_done[nid]
                        dst_lid = BRANCH_TAKEN if BRANCH_TAKEN in _dsts_h else (_dsts_h[0] if _dsts_h else None)
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
                    # T16 faithful EdgeWarp: pre-copy a snapshot at the
                    # predicted attachment (predictive timing above) then a
                    # FINAL SYNC (delta resend) at the actual handover. The
                    # commit-resend path below is that final sync; force it on
                    # for the edgewarp arm.
                    # FINAL UPDATE at commit (paper 3.4): warm and faithful
                    # EdgeWarp both send the source's LATEST record when the
                    # actor enters the destination, applied before the
                    # destination publishes. Without it the destination
                    # publishes from the record prepared L s earlier -> stale
                    # with lead (measured: 9/10 -> 2/10 as lead grows). The
                    # ablation arm warm_nofinal keeps the no-final behavior to
                    # show early prep is only safe with the final update.
                    # edgewarp_full (Exp B): EdgeWarp timing (predictive pre-copy
                    # + this final sync) carrying the FULL record rather than the
                    # depth-1 snapshot. A new mode value; adding it here changes
                    # no existing arm's membership result.
                    _final_sync = MIGRATION_MODE in ('warm', 'edgewarp', 'edgewarp_full')
                    if (COMMIT_REFRESH == 'full' or MIRROR_PERIOD_S > 0.0
                            or _final_sync) \
                            and nid not in npc_refresh_done \
                            and MIGRATION_MODE not in ('cold', 'warm_nofinal'):
                        _htick, _src_lid2, _dsts2 = npc_handoff_done[nid]
                        _ent2 = npc_entered.get(nid)
                        # final sync goes to the locale the actor ENTERED, and only
                        # if that locale was prepared (a wrong-destination prepare
                        # has nothing to update; the miss shows in READYROW).
                        _dst_lid = _ent2[1] if _ent2 is not None else (_dsts2[0] if _dsts2 else None)
                        _se = edge_by_locale.get(_src_lid2)
                        _de = edge_by_locale.get(_dst_lid) if _dst_lid else None
                        _crossed = _ent2 is not None and _dst_lid in _dsts2
                        if (COMMIT_REFRESH == 'full' or _final_sync) \
                                and _crossed \
                                and _se is not None and _de is not None:
                            _c = daemon.transfer_obstacle_state(
                                nid, _se, _de, link, step, position=nxy,
                                committed=True)
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
                cands = neighbors_by_locale.get(src_lid) or \
                    [l for l in locale_by_id if l != src_lid]
                if not cands:
                    continue
                nvel = actor.get_velocity()
                if step % 5 == 0:
                    logger.info("[NPCROW] tick=%d npc=%d x=%.2f y=%.2f v=%.2f src=%s",
                                step, nid, nxy[0], nxy[1],
                                (nvel.x ** 2 + nvel.y ** 2) ** 0.5, src_lid)
                # Lead L, computed exactly as the two-locale computed trigger
                # (transfer EMA + fold-in + margin, capped); shared by mtr/top1.
                _xfer_s = getattr(run_scenario, '_xfer_ema_s', 0.003)
                _fold_s = 3 * 0.2
                _lead_c = min(2.5, max(_fold_s + 0.35, _xfer_s + _fold_s + 0.35))

                def _cv_traj(_lead):
                    _ns = int(_lead / world_dt) + 1
                    _ta = np.arange(_ns, dtype=np.float64) * world_dt
                    return np.column_stack([nloc.x + nvel.x * _ta,
                                            nloc.y + nvel.y * _ta])

                def _cv_branch():
                    """Candidate the constant-velocity projection enters first
                    (nearest candidate to the 6 s point if it enters none)."""
                    _t = _cv_traj(6.0)
                    for _p in _t:
                        for _c in cands:
                            if locale_by_id[_c].contains(_p):
                                return _c, 'contains'
                    _far = _t[-1]
                    return min(cands, key=lambda _c: locale_by_id[_c].signed_distance(_far)), 'nearest'

                def _mtr_probs():
                    """Per-candidate destination probability from the SOURCE edge's
                    predictor mode stash (the same object the trigger reads):
                    mode probabilities summed by the candidate containing the
                    mode's world-frame endpoint. Also the top mode's branch."""
                    _se_m = edge_by_locale.get(src_lid)
                    _pred = getattr(_se_m, 'predictor', None) if _se_m is not None else None
                    _mm = getattr(_pred, 'last_mtr_modes', {}) if _pred is not None else {}
                    _tid = next((t for t, c in getattr(_se_m, 'track_to_carla', {}).items()
                                 if c == nid), None) if _se_m is not None else None
                    _modes = _mm.get(int(_tid)) if _tid is not None else None
                    if not _modes:
                        return None, None, 0
                    _pc = {_c: 0.0 for _c in cands}
                    for (wx, wy, p) in _modes:
                        for _c in cands:
                            if locale_by_id[_c].contains((wx, wy)):
                                _pc[_c] += p
                                break
                    _top = max(_modes, key=lambda m: m[2])
                    _topc = next((_c for _c in cands
                                  if locale_by_id[_c].contains((_top[0], _top[1]))), None)
                    return _pc, _topc, len(_modes)

                # CONFROW every tick while the actor is in a branching source, from
                # the same predictor instance every arm's trigger reads.
                if len(cands) >= 2 and CONF_LOG:
                    _pc, _topc, _nm = _mtr_probs()
                    _cvb, _cvr = _cv_branch()
                    if _pc is not None:
                        _am = max(_pc, key=_pc.get)
                        logger.info(
                            "[CONFROW] tick=%d npc=%d src=%s taken=%s n_modes=%d argmax=%s "
                            "p_argmax=%.3f top_mode_branch=%s cv_branch=%s %s",
                            step, nid, src_lid, BRANCH_TAKEN, _nm, _am, _pc[_am], _topc, _cvb,
                            " ".join("p[%s]=%.3f" % (_c, _pc[_c]) for _c in cands))
                    else:
                        logger.info("[CONFROW] tick=%d npc=%d src=%s taken=%s n_modes=0 cv_branch=%s",
                                    step, nid, src_lid, BRANCH_TAKEN, _cvb)

                # ── Trigger tick + fire set per arm. Only the DECISION varies. ──
                fire, _rule = None, ''
                # ---- at-crossing fallback (warm arm): the actor is inside a
                # candidate's polygon (the mouth's overlap band, src still the
                # approach) with no PREPARE fired for it -> fire into it now
                # through the same loop; bookkeeping = a handoff at lead zero.
                _pl = getattr(run_scenario, '_npc_prev_locale', None)
                if _pl is None:
                    _pl = run_scenario._npc_prev_locale = {}
                _prev_l = _pl.get(nid)
                _pl[nid] = src_lid
                _ent_fb = [_c for _c in cands if locale_by_id[_c].contains(nxy)]
                if (FALLBACK_AT_CROSSING and MIGRATION_MODE == 'warm'
                        and nid not in npc_handoff_done and _ent_fb):
                    fire, _rule = [_ent_fb[0]], 'fallback'
                    _lm = getattr(run_scenario, '_last_mtr_eval', {}).get(nid)
                    if TRIGGER_MODE == 'mtr':
                        _reason = ('no_forecast' if _lm is None else
                                   ('no_candidate_at_theta' if _lm[1] < MTR_THETA
                                    else 'lead_never_opened'))
                    else:
                        _reason = 'trigger_never_fired'
                    logger.info(
                        "[FALLBACKROW] tick=%d npc=%d entered=%s from=%s reason=%s "
                        "trigger=%s argmax=%s p_argmax=%.3f last_eval_tick=%d "
                        "spawn_dist_m=%.1f taken=%s",
                        step, nid, _ent_fb[0], src_lid, _reason, TRIGGER_MODE,
                        (_lm[2] if _lm else ''), (_lm[1] if _lm else -1.0),
                        (_lm[0] if _lm else -1),
                        getattr(run_scenario, '_npc_resolve_dist', {}).get(nid, -1.0),
                        BRANCH_TAKEN)
                elif (FALLBACK_AT_CROSSING and MIGRATION_MODE == 'warm'
                        and nid not in npc_handoff_done
                        and _prev_l is not None and _prev_l != src_lid):
                    # the actor changed locale without passing the overlap band
                    # (band skipped between ticks): the fallback could not fire
                    logger.warning("[FALLBACK-MISS] tick=%d npc=%d from=%s to=%s",
                                   step, nid, _prev_l, src_lid)
                    continue
                elif TRIGGER_MODE == 'band' and MIGRATION_MODE not in (
                        "reactive", "handover_snapshot"):
                    _cvb, _ = _cv_branch()
                    if locale_by_id[_cvb].signed_distance(nxy) > BAND_W_M:
                        continue
                    fire, _rule = [_cvb], 'band'
                elif TRIGGER_MODE == 'mtr':
                    # Khonsu: trigger tick = L gate AND some candidate at/above
                    # theta; fire set = every candidate at/above theta. MTR_TOP1
                    # shares the SAME tick and L and fires only the top mode's
                    # branch (destination selection differs, timing never).
                    _pc, _topc, _nm = _mtr_probs()
                    _lead_ok = locale_by_id[src_lid].predicted_to_exit_within(
                        _cv_traj(_lead_c), _lead_c, world_dt)
                    _lme = getattr(run_scenario, '_last_mtr_eval', None)
                    if _lme is None:
                        _lme = run_scenario._last_mtr_eval = {}
                    if _pc:
                        _am = max(_pc, key=_pc.get)
                        _lme[nid] = (step, _pc[_am], _am, bool(_lead_ok))   # what a guess would use
                    if _pc is None or max(_pc.values()) < MTR_THETA:
                        # DEADLINE GUESS (rule 3, default off): lead window closing
                        # with no candidate at theta -> the argmax candidate alone
                        if not (DEADLINE_GUESS and _pc and _lead_ok):
                            continue
                        fire, _rule = [max(_pc, key=_pc.get)], 'guess'
                        logger.info("[GUESSROW] tick=%d npc=%d dst=%s p=%.3f theta=%.2f taken=%s",
                                    step, nid, fire[0], _pc[fire[0]], MTR_THETA, BRANCH_TAKEN)
                    else:
                        if not _lead_ok:
                            continue
                        if MTR_TOP1:
                            fire = [_topc] if _topc is not None else [max(_pc, key=_pc.get)]
                            _rule = 'top1'
                        else:
                            fire = [_c for _c in cands if _pc[_c] >= MTR_THETA]
                            _rule = 'theta'
                elif TRIGGER_MODE == 'oracle':
                    # Knows the taken branch; timing = L before the GT-velocity
                    # projected exit, as the two-locale oracle.
                    _L = 3 * 0.2 + 0.35
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _cv_traj(_L), _L, world_dt):
                        continue
                    fire = [BRANCH_TAKEN] if BRANCH_TAKEN in cands else [_cv_branch()[0]]
                    _rule = 'oracle'
                elif MIGRATION_MODE in ("reactive", "handover_snapshot"):
                    # At-crossing: fire when the actor is inside a candidate; the
                    # destination is the cell being entered (EdgeWarp radio trigger).
                    _ent = [_c for _c in cands if locale_by_id[_c].contains(nxy)]
                    if not _ent:
                        continue
                    fire, _rule = [_ent[0]], 'crossing'
                else:
                    if TRIGGER_MODE == 'computed':
                        _lead = _lead_c
                        logger.info(
                            "[LEADROW] npc=%d lead=%.3f xfer_ema=%.3f "
                            "fold=%.3f margin=0.350 tick=%d",
                            nid, _lead, _xfer_s, _fold_s, step)
                    else:
                        _lead = OBSTACLE_HANDOFF_LOOKAHEAD_S
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _cv_traj(_lead), _lead, world_dt):
                        continue
                    _cvb, _cvr = _cv_branch()
                    fire, _rule = [_cvb], 'cv_' + _cvr
                if REPLICATE_ALL:
                    fire, _rule = list(cands), _rule + '+replicate_all'
                src_edge = edge_by_locale.get(src_lid)
                if src_edge is None or not fire:
                    continue
                npc_trigger_tick.setdefault(nid, step)
                npc_cv_pick.setdefault(nid, _cv_branch()[0])
                _done_any = False
                for _dl in fire:
                    dst_edge = edge_by_locale.get(_dl)
                    if dst_edge is None or dst_edge is src_edge:
                        continue
                    cost = None if MIGRATION_MODE == "cold" \
                        else daemon.transfer_obstacle_state(
                            nid, src_edge, dst_edge, link, step, position=nxy)
                    if cost is None:
                        continue
                    _done_any = True
                    scenario_manager.record_handoff_cost(cost)
                    _prev = getattr(run_scenario, '_xfer_ema_s', None)
                    _cur_s = cost.total_ms / 1000.0
                    run_scenario._xfer_ema_s = _cur_s if _prev is None \
                        else 0.3 * _cur_s + 0.7 * _prev
                    npc_prepare_ticks.setdefault((nid, _dl), step)
                    npc_prepare_hash.setdefault((nid, _dl), getattr(cost, 'payload_hash', ''))
                    npc_prepare_depth.setdefault((nid, _dl), int(getattr(cost, 'history_depth', -1)))
                    logger.info(
                        "[SCENB] PREDICTIVE OBSTACLE HANDOFF tick=%d "
                        "carla_id=%d npc=(%.1f,%.1f) %s->%s bytes=%d "
                        "total_ms=%.3f rule=%s hash=%s depth=%d",
                        step, nid, nxy[0], nxy[1], src_lid, _dl,
                        cost.payload_bytes, cost.total_ms, _rule,
                        getattr(cost, 'payload_hash', ''),
                        int(getattr(cost, 'history_depth', -1)))
                if _done_any or MIGRATION_MODE == "cold":
                    npc_handoff_done[nid] = (
                        step, src_lid, [d for d in fire if (nid, d) in npc_prepare_ticks])
                    logger.info("[FIREROW] tick=%d npc=%d src=%s rule=%s fire=%s prepared=%s taken=%s",
                                step, nid, src_lid, _rule, ",".join(fire),
                                ",".join(npc_handoff_done[nid][2]) or "none", BRANCH_TAKEN)
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
            _tk_edge = edge_by_locale.get(BRANCH_TAKEN) if BRANCH_TAKEN else None
            if _tk_edge is not None and _tk_edge.vehicle_manager_list:
                ego_vm = _tk_edge.vehicle_manager_list[0]   # recipient CAV of the taken branch
                ego_cav = ego_vm.vehicle
            for edge in edge_list:
                if ego_cav is not None:
                    break
                if edge.vehicle_manager_list:
                    ego_vm = edge.vehicle_manager_list[0]
                    ego_cav = ego_vm.vehicle
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
            for nid, (htick, _sl6, _dl6) in npc_handoff_done.items():
                dst_lid = BRANCH_TAKEN if BRANCH_TAKEN in _dl6 else (_dl6[0] if _dl6 else 'none')
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
            for _ei, _edge in enumerate(edge_list):
                _cms = getattr(_edge, '_t21_compute_ms_sum', 0.0)
                _vs = getattr(_edge, '_t21_veh_seconds', 0.0)
                _cpvs = (_cms / 1000.0 / _vs) if _vs > 0 else 0.0
                logger.info(
                    "[COMPUTEROW] edge=%d locale=%s compute_s=%.3f "
                    "veh_seconds=%.3f compute_per_veh_s=%.4f",
                    _ei, getattr(_edge, 'edgeid', _ei), _cms / 1000.0,
                    _vs, _cpvs)
            for (_nid4, _dl4), _pt4 in sorted(npc_prepare_ticks.items()):
                _fd = t19_first_dst.get((_nid4, _dl4), -1)
                _fu = t19_first_use.get((_nid4, _dl4), -1)
                _cx = t19_crossing.get(_nid4, -1)
                _warm = (_fd >= 0 and (_fu < 0 or _fd <= _fu)
                         and (_cx < 0 or _fd <= _cx))
                logger.info(
                    "[HANDOFFROW] npc=%d prepare_tick=%d crossing_tick=%d "
                    "first_dst_track_tick=%d first_use_tick=%d "
                    "warm_before_first_use=%s dst=%s hash=%s depth=%d",
                    _nid4, _pt4, _cx, _fd, _fu,
                    "YES" if _warm else "no", _dl4,
                    npc_prepare_hash.get((_nid4, _dl4), ''),
                    npc_prepare_depth.get((_nid4, _dl4), -1))
            # READYROW / PLACEROW per actor: readiness at the TAKEN branch,
            # decomposed into wrong_destination and right_destination_too_late.
            for _nid5 in sorted(npc_ids):
                _ent5 = npc_entered.get(_nid5)
                _entl = _ent5[1] if _ent5 else ''
                _tk = BRANCH_TAKEN or _entl
                _prep = sorted(d for (n, d) in npc_prepare_ticks if n == _nid5)
                _fd = t19_first_dst.get((_nid5, _tk), -1)
                _fu = t19_first_use.get((_nid5, _tk), -1)
                _cx = t19_crossing.get(_nid5, -1)
                _cf = npc_conflict_tick.get(_nid5, -1)
                _pt = npc_prepare_ticks.get((_nid5, _tk), -1)
                _ready = (_tk in _prep) and _fd >= 0 and (_fu < 0 or _fd <= _fu) \
                    and (_cx < 0 or _fd <= _cx)
                _wrong = bool(_prep) and _tk not in _prep
                _late = (_tk in _prep) and not _ready
                _lead_s = ((_cx - _pt) * world_dt) if (_pt >= 0 and _cx >= 0) else float('nan')
                _tavail_s = ((_cf - _fd) * world_dt) if (_fd >= 0 and _cf >= 0) else float('nan')
                logger.info(
                    "[READYROW] npc=%d taken=%s entered=%s route_ok=%s k=%d prepared=%s "
                    "n_prepared=%d prepare_tick=%d trigger_tick=%d crossing_tick=%d "
                    "conflict_tick=%d first_dst_track_tick=%d first_use_tick=%d ready=%s "
                    "wrong_destination=%s right_destination_too_late=%s lead_s=%.3f "
                    "t_avail_s=%.3f cv_pick=%s",
                    _nid5, _tk, _entl or 'none',
                    "YES" if (_entl == _tk) else ("no_entry" if not _entl else "no"),
                    K_BRANCH, ",".join(_prep) or "none", len(_prep), _pt,
                    npc_trigger_tick.get(_nid5, -1), _cx, _cf, _fd, _fu,
                    "YES" if _ready else "no", "YES" if _wrong else "no",
                    "YES" if _late else "no", _lead_s, _tavail_s,
                    npc_cv_pick.get(_nid5, ''))
                logger.info("[PLACEROW] npc=%d k=%d prepared=%s count=%d run_bytes=%d",
                            _nid5, K_BRANCH, ",".join(_prep) or "none", len(_prep), _tbytes)
            import os as _os2
            _dblsum = sum(getattr(run_scenario, '_dbl_publish_ticks',
                                  {}).values())
            logger.info(
                "[RUNROW] mode=%s trigger=%s band_w=%.1f refresh=%s "
                "mirror=%.2f lookahead=%.2f episodes=%d contact_ticks=%d "
                "transfers=%d bytes=%d fault=%s fencing=%s dbl_ticks=%d "
                "k=%d taken=%s replicate_all=%d top1=%d theta=%.2f",
                MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
                MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S,
                _eps, _contact, len(transfer_costs), _tbytes,
                _os2.environ.get('FAULT_MODE', 'none') or 'none',
                _os2.environ.get('FENCING', 'on'), _dblsum,
                K_BRANCH, BRANCH_TAKEN, int(REPLICATE_ALL), int(MTR_TOP1), MTR_THETA)
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
