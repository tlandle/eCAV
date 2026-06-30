# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Common utilities used by all edge-manager backends.

Provides the base class with shared infrastructure:
- Pluggable latency model (via ``ecav.core.application.edge.latency``)
- Shared metric helper instances
- _BaseEdgeManager superclass (add_member, add_rsu, set_destination)
"""

from __future__ import annotations

import logging
import os
import uuid
import weakref
import time
from typing import Any, Dict, List, Optional

import numpy as np
import carla

from ecav.core.application.edge.edge_metrics import EdgeMetrics
from ecav.core.application.edge.migration.payload import KFState, MigrationPayload, TrackLatent
from ecav.core.application.edge.latency import create_latency_model
from ecav.core.application.edge.latency import create_mac_model, reset_global_macs
from ecav.core.prediction.linear_predictor_manager   import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory  import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle   import ObstacleVehicle


logger = logging.getLogger("EdgeManager")


# ──────────────────────────────────────────────────────────────
#  Minimal shared base-class
# ──────────────────────────────────────────────────────────────
class _BaseEdgeManager:
    """
    Functionality identical for every Edge-manager variant:

    * bookkeeping (vehicle / RSU lists, CavWorld registration)
    * latency / loss parameters parsed from YAML
    * shared debug helper
    * default hooks: add_member, add_rsu, set_destination
    """

    def __init__(
        self,
        world: carla.World,
        cfg:   Dict[str, Any],
        cav_world,
        carla_client: carla.Client,
        *,
        world_dt: float = 0.05,
        is_proxy: bool = False,
        **kwargs,
    ):
        self.is_proxy = is_proxy
        self.edgeid = str(uuid.uuid4())[:8]
        self.world  = world
        self.carla  = carla_client
        self.dt     = world_dt

        # lists of VehicleManagers / RSUManagers
        self.vehicle_manager_list: List[Any] = []
        self.rsu_manager_list:     List[Any] = []

        if not is_proxy:
            # latency model (pluggable: fixed, normal, lognormal, hybrid)
            self.latency_model = create_latency_model(cfg, world_dt)
            self.downlink_pl = float(cfg.get("downlink_packet_loss_pct", 0))

            # MAC model (pluggable: legacy/null/sbsps)
            reset_global_macs()  # idempotent — prevents cross-test contamination
            _mac_seed = cfg.get("mac", {}).get("seed", 0)
            self.mac_model = create_mac_model(
                cfg, latency_model=self.latency_model, seed=_mac_seed)

            # global debug helper for perf / viz
            self.debug = EdgeMetrics(0)
        else:
            self.latency_model = None
            self.downlink_pl = 0.0
            self.mac_model = None
            self.debug = None

        self._proxy_metrics: dict = {}

        # compute-contention model (ms)
        _budget = cfg.get("compute_budget_ms", None)
        self.compute_budget_ms: float | None = (
            float(_budget) if _budget is not None else None
        )
        self.per_vehicle_compute_ms: float = float(
            cfg.get("per_vehicle_compute_ms", 5.0)
        )

        # Spatial self-ID radius (m) for non-anchored ego identification
        self._self_id_radius = float(cfg.get("self_id_radius", 5.0))
        # Speed gate (m/s) for ego-consistency suppression G(e)
        self._self_id_speed_gate = float(cfg.get("self_id_speed_gate", 3.0))

        # A1 freshness contract threshold (ticks).  Default 20 = 1.0s at dt=0.05
        self._aoi_freshness_threshold_ticks = int(
            cfg.get("aoi_freshness_threshold_ticks", 20)
        )

        # shared predictor helper (used by some back-ends)
        self.lin_pred = LinearPredictorManager(num_future_steps=25) if not is_proxy else None

        # Ego-consistency gate violation tracking (publish-boundary invariant)
        self._ego_gate_violations_total = 0
        self._ego_gate_violation_ticks = []  # (tick, per_ego_counts_dict)

        # Competing-risk time-to-event tracking
        # Records the first tick each failure class occurs per ego vehicle
        self._first_event_ticks = {}  # {vehicle_id: {class: tick}}

        # Optional per-tick conflict-kinematics logger (LTAP fixture validation).
        # Off unless cfg['conflict_kinematics'] is present. Pins the closing
        # geometry so collisions can be classified as physics-limited (margin<0)
        # vs timing artifacts (margin>0).
        self._conflict_logger = None
        self._cfg_latency_s = float(cfg.get("latency", 0.0))
        _ck = cfg.get("conflict_kinematics") if not is_proxy else None
        if _ck and _ck.get("enabled", False):
            from ecav.core.application.edge.conflict_kinematics_logger import (
                ConflictKinematicsLogger,
            )
            # Write the trace into this run's own output dir (set on cfg as
            # run_output_dir) so each run/config has its own CSV that maps 1:1
            # to its results, instead of a shared path that auto-uniquifies and
            # cannot be attributed back to a run.
            _run_dir = cfg.get("run_output_dir")
            if _run_dir:
                _ck_out = os.path.join(_run_dir, "conflict_kinematics.csv")
            else:
                _ck_out = _ck.get("out_path", "/tmp/conflict_kinematics.csv")
            self._conflict_logger = ConflictKinematicsLogger(
                out_path=_ck_out,
                conflict_xy=tuple(_ck.get("conflict_xy", (-84.8, 127.7))),
                cross_traffic_type=_ck.get("cross_traffic_type", "tesla"),
                rho_s=float(_ck.get("rho_s", 0.1)),
                a_brake=float(_ck.get("a_brake", 6.0)),
            )

        # distributed mode flag (from cav_world)
        self.run_distributed = getattr(cav_world, 'run_distributed', False) if cav_world else False

        # register with CavWorld
        weakref.ref(cav_world)().update_edge(self)

    # ─── abstract hooks every backend must override ────────────
    def start_edge(self):
        if self.is_proxy:
            return
        raise NotImplementedError

    def update_information(self, frame_idx):  raise NotImplementedError

    def run_step(self, tick):
        if self.is_proxy:
            return None
        raise NotImplementedError

    def evaluate(self):
        if self.is_proxy:
            return None, "", self._proxy_metrics
        raise NotImplementedError

    # ─── helpers common to all concrete back-ends ────────────
    def add_member(self, vm: Any) -> None:
        """Register a new VehicleManager under this edge."""
        self.vehicle_manager_list.append(vm)

    def add_rsu(self, rsu: Any) -> None:
        """Register a new RSUManager under this edge."""
        self.rsu_manager_list.append(rsu)

    def set_destination(self, destination: carla.Location) -> None:
        """
        Called by sim_api to tell the edge where its goal is.
        Store it for use in your edge logic.
        """
        self.destination = destination

    # ─── Latency component statistics (for Fig 1 CDFs) ────────────
    def _get_latency_component_stats(self) -> Dict:
        """Return per-component latency statistics from sample history.

        Only meaningful for HybridModel which logs radio/backhaul/base.
        Other models return total-only stats.
        """
        history = getattr(self.latency_model, 'sample_history', [])
        if not history:
            return {}
        import numpy as _np
        result = {}
        for key in history[0]:
            vals = _np.array([s[key] for s in history])
            result[f'latency_{key}_mean'] = float(_np.mean(vals))
            result[f'latency_{key}_p50'] = float(_np.median(vals))
            result[f'latency_{key}_p95'] = float(_np.percentile(vals, 95))
            result[f'latency_{key}_p99'] = float(_np.percentile(vals, 99))
        result['latency_num_samples'] = len(history)
        return result

    # ─── Ego-consistency gate G(ego) at publish boundary ──────────
    def _check_ego_gate_violations(self, tick, preds):
        """Count predictions inside G(ego) that are NOT identified as ego.

        The ego-consistency invariant states: no published obstacle may lie
        inside the state-space gate G(ego_pos, self._self_id_radius) unless
        it carries the ego identity.  Violations → potential ghost brakes.

        Stores per-tick violation counts for later analysis.
        """
        per_ego = {}
        for vm in self.vehicle_manager_list:
            ego_loc = vm.vehicle.get_location()
            ego_id = vm.vehicle.id
            violations = 0
            for pred in preds:
                obs = pred.obstacle_trajectory.obstacle
                obs_loc = obs.location
                dist = ((obs_loc.x - ego_loc.x)**2 +
                        (obs_loc.y - ego_loc.y)**2)**0.5
                if dist < self._self_id_radius and obs.carla_id != ego_id:
                    violations += 1
            per_ego[ego_id] = violations
            self._ego_gate_violations_total += violations

        self._ego_gate_violation_ticks.append((tick, per_ego))
        return per_ego

    def get_ego_gate_metrics(self):
        """Return summary metrics for ego-gate violations across the run."""
        if not self._ego_gate_violation_ticks:
            return {
                'ego_gate_violations_total': 0,
                'ego_gate_violation_tick_count': 0,
                'ego_gate_violation_tick_fraction': 0.0,
            }
        total_ticks = len(self._ego_gate_violation_ticks)
        ticks_with_violations = sum(
            1 for _, per_ego in self._ego_gate_violation_ticks
            if any(v > 0 for v in per_ego.values())
        )
        return {
            'ego_gate_violations_total': self._ego_gate_violations_total,
            'ego_gate_violation_tick_count': ticks_with_violations,
            'ego_gate_violation_tick_fraction': ticks_with_violations / total_ticks,
        }

    # ─── Publish-boundary contract metrics (A1/A2/A3) ──────────
    def _get_contract_metrics(self) -> Dict:
        """Assemble unified contract metrics for Fig 12.

        A1 — Freshness: fraction of ticks where AoI > threshold.
        A2 — Ego-Gate:  fraction of ticks with ego-gate violations.
        A3 — Uniqueness: fraction of ticks with identity-dup violations.

        Requires ``self.profiler`` (set by every concrete subclass).
        """
        result: Dict = {}

        # Profiler frame history (all concrete subclasses set self.profiler)
        frames = (list(self.profiler.frame_history)
                  if hasattr(self, 'profiler') else [])

        # ── A1: Freshness ────────────────────────────────────────
        threshold = self._aoi_freshness_threshold_ticks
        if frames:
            # Only count ticks where the edge actually published (aoi > 0)
            published = [f for f in frames if f.aoi_ticks > 0]
            a1_violations = sum(1 for f in published
                                if f.aoi_ticks > threshold)
            result['contract_a1_freshness_threshold_ticks'] = threshold
            result['contract_a1_freshness_violations'] = a1_violations
            result['contract_a1_freshness_violation_fraction'] = (
                a1_violations / len(published) if published else 0.0
            )
        else:
            result['contract_a1_freshness_threshold_ticks'] = threshold
            result['contract_a1_freshness_violations'] = 0
            result['contract_a1_freshness_violation_fraction'] = 0.0

        # ── A2: Ego-Gate ─────────────────────────────────────────
        ego_gate = self.get_ego_gate_metrics()
        result['contract_a2_ego_gate_violations'] = (
            ego_gate['ego_gate_violations_total'])
        result['contract_a2_ego_gate_violation_tick_count'] = (
            ego_gate['ego_gate_violation_tick_count'])
        result['contract_a2_ego_gate_violation_fraction'] = (
            ego_gate['ego_gate_violation_tick_fraction'])

        # ── A3: Uniqueness ───────────────────────────────────────
        if frames:
            uniq_tick_count = sum(
                1 for f in frames if f.ego_uniqueness_violations > 0)
            uniq_total = sum(f.ego_uniqueness_violations for f in frames)
            result['contract_a3_uniqueness_violations'] = uniq_total
            result['contract_a3_uniqueness_violation_tick_count'] = (
                uniq_tick_count)
            result['contract_a3_uniqueness_violation_fraction'] = (
                uniq_tick_count / len(frames))
        else:
            result['contract_a3_uniqueness_violations'] = 0
            result['contract_a3_uniqueness_violation_tick_count'] = 0
            result['contract_a3_uniqueness_violation_fraction'] = 0.0

        return result

    # ─── Conflict-kinematics logging (LTAP fixture validation) ───
    def _live_gt_snapshot(self):
        """Build a {actor_id: {type,x,y,vx,vy,speed}} snapshot from live CARLA
        actors. Used by managers (e.g. PerceptionEdge) that do not precompute
        per-tick GT snapshots, so the conflict logger can find cross-traffic.
        """
        snap = {}
        try:
            acts = self.world.get_actors().filter('vehicle.*')
            for a in acts:
                loc = a.get_location()
                vel = a.get_velocity()
                snap[a.id] = {
                    # Full type_id (e.g. 'vehicle.tesla.model3') so the
                    # cross-traffic picker can match on make OR model.
                    'type': a.type_id,
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'vx': vel.x, 'vy': vel.y,
                    'speed': (vel.x ** 2 + vel.y ** 2) ** 0.5,
                }
        except Exception:
            pass
        return snap

    def _log_conflict_kinematics(self, tick, gt_snapshot=None):
        """Log per-tick closing geometry for the focal ego, if enabled.

        Focal ego = lowest managed vehicle id (the hero). gt_snapshot is the
        per-tick CARLA actor snapshot the manager already builds; if None, the
        logger falls back to live actor poses via the ego's world handle.
        """
        if self._conflict_logger is None or not self.vehicle_manager_list:
            return
        focal = min(self.vehicle_manager_list, key=lambda m: m.vehicle.id)
        # Per-tick collision flag from client metrics, if available; the CSV
        # mainly tracks the closing geometry, collision is cross-checked from
        # the eval report. Latch once true.
        coll = False
        try:
            cm = focal.client_metrics
            coll = bool(getattr(cm, 'collision', False)) or \
                len(getattr(cm, 'collision_event_list', []) or []) > 0
        except Exception:
            pass
        self._conflict_logger.log_tick(
            tick, focal, gt_snapshot or {}, coll,
            delta_use_s=self._cfg_latency_s)

    # ─── Competing-risk time-to-event tracking ──────────────────
    def _record_time_to_events(self, tick, vm):
        """Record first-occurrence tick for each brake failure class."""
        vid = vm.vehicle.id
        if vid not in self._first_event_ticks:
            self._first_event_ticks[vid] = {}

        events = self._first_event_ticks[vid]
        for attr in vm.agent.planning_metrics.brake_attributions:
            cls = attr.get('gt_brake_class')
            if cls and cls not in events:
                events[cls] = tick

    def get_competing_risk_metrics(self):
        """Return first-event ticks per vehicle and tie-rate analysis."""
        result = {}
        for vid, events in self._first_event_ticks.items():
            result[str(vid)] = dict(events)
        return result

    # Thresholds for GT hazard predicate
    _GT_HAZARD_SPEED_THRESH = 2.0      # m/s — actor must be moving
    _GT_HAZARD_DCA_THRESH   = 8.0      # m  — closest approach distance
    #   Perpendicular vehicles at intersection: need to account for both
    #   vehicles' physical dimensions (each ~4.5m × 2m).  Diagonal
    #   clearance ≈ sqrt((2.25+1)^2 + (2.25+1)^2) ≈ 4.6m center-to-center
    #   plus prediction noise margin.  8m captures real near-misses.
    _GT_HAZARD_TIME_HORIZON = 5.0      # s  — look-ahead for closest approach

    # Self-duplicate reclassification thresholds (heading-aligned near-ego box)
    _SELFDUP_LONG_M  = 8.0   # longitudinal (along ego heading)
    _SELFDUP_LAT_M   = 3.0   # lateral (perpendicular to heading)

    def _label_brake_attributions_gt(self, vm) -> None:
        """
        GT-label any new brake attributions from the behavior agent.

        For each brake event with ``ghost_brake_gt is None``:

        1. Match the triggering track's position (obs_x, obs_y) to the
           nearest CARLA actor by center distance (evaluation identity).
        2. Classify into three categories:
           - **self-ghost FP**: matched actor = ego vehicle
           - **true positive**: matched actor is a GT hazard (moving toward
             ego on collision course, per linear extrapolation using
             *relative* velocity — actor minus ego)
           - **other FP**: matched actor is NOT a GT hazard (parked,
             stationary, or moving away from ego)
        """
        attrs = vm.agent.planning_metrics.brake_attributions
        if not attrs:
            return

        # Only process unlabeled entries
        unlabeled = [a for a in attrs if a.get('ghost_brake_gt') is None]
        if not unlabeled:
            return

        # Get current GT state of all CARLA vehicle actors
        vehicles = self.world.get_actors().filter('vehicle.*')
        gt_actors = []
        for v in vehicles:
            loc = v.get_location()
            vel = v.get_velocity()
            gt_actors.append({
                'id': v.id,
                'x': loc.x, 'y': loc.y,
                'vx': vel.x, 'vy': vel.y,
                'speed': (vel.x**2 + vel.y**2)**0.5,
            })

        if not gt_actors:
            return

        ego_id = vm.vehicle.id
        # Set of all managed ego vehicle IDs for provenance tagging
        managed_ids = {m.vehicle.id for m in self.vehicle_manager_list}
        # Get ego velocity for relative-motion DCA calculation
        ego_vel = vm.vehicle.get_velocity()
        ego_vx, ego_vy = ego_vel.x, ego_vel.y

        gt_xy  = np.array([[a['x'], a['y']] for a in gt_actors])

        # Build a lookup for GT actors by CARLA ID
        gt_by_id = {a['id']: a for a in gt_actors}

        for attr in unlabeled:
            obs_xy = np.array([attr['obs_x'], attr['obs_y']])
            ego_xy = np.array([attr['ego_x'], attr['ego_y']])

            trigger_cid = attr.get('trigger_carla_id')

            # If the tracker identified this track as a specific non-ego
            # actor, trust the ID rather than position proximity.  At high
            # latency the prediction position is stale and the ego may
            # have drifted closer to it than the original actor, causing
            # false self_ghost labels.
            if (trigger_cid is not None
                    and trigger_cid != ego_id
                    and trigger_cid > 0
                    and trigger_cid in gt_by_id):
                matched = gt_by_id[trigger_cid]
                matched_id = trigger_cid
                match_dist = float(np.linalg.norm(
                    np.array([matched['x'], matched['y']]) - obs_xy))
            else:
                # Anonymous track or ego — fall back to position matching
                dists = np.linalg.norm(gt_xy - obs_xy, axis=1)
                best_idx = int(np.argmin(dists))
                matched = gt_actors[best_idx]
                matched_id = matched['id']
                match_dist = float(dists[best_idx])

            attr['gt_matched_actor_id'] = matched_id
            attr['gt_match_dist_m'] = match_dist

            # DIAGNOSTIC (env GHOST_DEBUG=1): dump what the matcher compared,
            # so we can tell a genuine ego self-echo from a stale cross-traffic
            # track that the ego coincided with. Lists the nearest few GT actors
            # to the stale track position with their types + distances.
            if os.environ.get('GHOST_DEBUG') == '1':
                order = np.argsort(dists)[:4]
                near = []
                for j in order:
                    a = gt_actors[int(j)]
                    is_ego = '<EGO>' if a['id'] == ego_id else ''
                    near.append(f"id{a['id']}{is_ego}@({a['x']:.1f},{a['y']:.1f})"
                                f"d{dists[int(j)]:.2f}spd{a['speed']:.1f}")
                logger.warning(
                    "[GHOST-MATCH] ego=%d trig_cid=%s obs_track=(%.1f,%.1f) "
                    "ego_at=(%.1f,%.1f) matched=%s d=%.2f | nearest: %s",
                    ego_id, trigger_cid, attr['obs_x'], attr['obs_y'],
                    attr['ego_x'], attr['ego_y'], matched_id, match_dist,
                    " ".join(near))

            # ── Provenance-history taxonomy ──────────────────────────────
            # A brake-triggering track is classified by what GT actor its
            # position matched over its RECENT HISTORY, not by which actor is
            # nearest NOW. nearest-now mislabels stale/merged tracks: a track
            # that tracked the cross-traffic Tesla then froze at the conflict
            # point gets called self_ghost only because the ego later drives
            # through that point. Taxonomy (last K provenance tags):
            #   consistently ego, no non-ego   -> self_ghost (true ego echo)
            #   both ego AND non-ego present    -> track_merge_identity_switch
            #   consistently non-ego           -> external_stale (real obstacle,
            #                                      stale position)
            #   no history                     -> fall back to nearest-now
            prov = None
            prov_hist = getattr(self, '_track_provenance', {}).get(
                attr.get('trigger_track_id'))
            if prov_hist:
                tags = [t[0] for t in prov_hist]  # 'ego' / 'nonego'
                n_ego = tags.count('ego')
                n_non = tags.count('nonego')
                if n_ego and n_non:
                    prov = 'track_merge'
                elif n_ego and not n_non:
                    prov = 'self_ghost'
                elif n_non and not n_ego:
                    prov = 'external_stale'
            attr['gt_provenance_class'] = prov

            # is_self_ghost only when provenance says the track was consistently
            # the ego (or, with no history, nearest-now is the ego as fallback).
            if prov is not None:
                is_self_ghost = (prov == 'self_ghost')
                if not is_self_ghost and matched_id == ego_id:
                    # nearest-now is ego but provenance says merge/external:
                    # re-match to nearest NON-ego so hazard/FP classification
                    # uses a real other actor, not the ego matched to itself.
                    non_ego = [(d, a) for d, a in zip(dists, gt_actors)
                               if a['id'] != ego_id]
                    if non_ego:
                        d2, a2 = min(non_ego, key=lambda t: t[0])
                        matched, matched_id, match_dist = a2, a2['id'], float(d2)
                        attr['gt_matched_actor_id'] = matched_id
                        attr['gt_match_dist_m'] = match_dist
                    logger.warning(
                        "[GHOST-RECLASS] ego=%d track=%s prov=%s (ego=%d,non=%d)"
                        " -> NOT self-ghost; rematched to %s",
                        ego_id, attr.get('trigger_track_id'), prov,
                        n_ego, n_non, matched_id)
            else:
                is_self_ghost = (matched_id == ego_id)

            # Category 1: self-ghost
            if is_self_ghost:
                attr['ghost_brake_gt'] = True
                attr['false_positive_gt'] = False
                attr['true_positive_gt'] = False
                attr['gt_brake_class'] = 'self_ghost'
                attr['gt_provenance'] = 'self'
                logger.warning(
                    "[GT GHOST] ego %d: track=%s cid=%s -> GT actor %d "
                    "(dist=%.2fm) obs=(%.1f,%.1f) ego=(%.1f,%.1f)",
                    ego_id, attr.get('trigger_track_id'),
                    attr.get('trigger_carla_id'),
                    matched_id, match_dist,
                    attr['obs_x'], attr['obs_y'],
                    attr['ego_x'], attr['ego_y'])
                continue

            # Category 2 or 3: check if matched actor is a GT hazard
            is_hazard, dca, t_ca = self._is_gt_hazard(
                matched, ego_xy, ego_vx, ego_vy)

            attr['ghost_brake_gt'] = False
            attr['gt_actor_speed'] = matched['speed']
            attr['gt_dca_m'] = dca
            attr['gt_t_ca_s'] = t_ca

            # Provenance: classify the source identity of the triggering actor
            if matched_id in managed_ids and matched_id != ego_id:
                attr['gt_provenance'] = 'cross_ego'
            elif matched['speed'] < 0.5:
                attr['gt_provenance'] = 'parked'
            elif is_hazard:
                attr['gt_provenance'] = 'npc_hazard'
            else:
                attr['gt_provenance'] = 'npc_non_hazard'

            if is_hazard:
                attr['false_positive_gt'] = False
                attr['true_positive_gt'] = True
                attr['gt_brake_class'] = 'true_positive'
                logger.debug(
                    "[GT TP] ego %d: track=%s -> GT actor %d "
                    "(speed=%.1f m/s, DCA=%.2fm, t_ca=%.2fs) "
                    "obs=(%.1f,%.1f) ego=(%.1f,%.1f)",
                    ego_id, attr.get('trigger_track_id'),
                    matched_id, matched['speed'], dca, t_ca,
                    attr['obs_x'], attr['obs_y'],
                    attr['ego_x'], attr['ego_y'])
            else:
                attr['false_positive_gt'] = True
                attr['true_positive_gt'] = False
                # Promote the provenance class to a first-class brake label so
                # track-merge / external-stale FPs are distinguished from
                # generic other_fp (self_ghost handled above via continue).
                if prov == 'track_merge':
                    attr['gt_brake_class'] = 'track_merge_identity_switch'
                elif prov == 'external_stale':
                    attr['gt_brake_class'] = 'external_stale_fp'
                else:
                    attr['gt_brake_class'] = 'other_fp'
                logger.warning(
                    "[GT OTHER-FP] ego %d: track=%s -> GT actor %d "
                    "(speed=%.1f m/s, DCA=%.2fm, t_ca=%.2fs prov=%s) "
                    "not a GT hazard. obs=(%.1f,%.1f) ego=(%.1f,%.1f)",
                    ego_id, attr.get('trigger_track_id'),
                    matched_id, matched['speed'], dca, t_ca,
                    attr['gt_provenance'],
                    attr['obs_x'], attr['obs_y'],
                    attr['ego_x'], attr['ego_y'])

            # ── Self-duplicate reclassification ──
            # If behavior agent flagged this as ego-ghost (cid=-1, near ego)
            # but GT matched a parked car, check if the obstacle is inside a
            # heading-aligned near-ego box.  If so, it's a phantom self-
            # duplicate, not a real parked-car detection.
            if (attr['gt_brake_class'] == 'other_fp'
                    and attr.get('is_ego_ghost', False)
                    and attr.get('trigger_carla_id', 0) == -1):
                self._reclassify_self_duplicate(
                    attr, ego_xy, ego_vx, ego_vy, ego_id)

            # Reconcile gt_provenance_class with the final gt_brake_class so the
            # two fields can never disagree (the early prov computation can be
            # None when the deque lookup misses, but the brake-class branches
            # below recompute the correct category). Derive from the consumer
            # label, which is authoritative.
            _final = attr.get('gt_brake_class')
            _prov_map = {
                'self_ghost': 'ego',
                'track_merge_identity_switch': 'track_merge',
                'external_stale_fp': 'external_stale',
            }
            if _final in _prov_map:
                attr['gt_provenance_class'] = _prov_map[_final]
            elif attr.get('gt_provenance_class') is None:
                attr['gt_provenance_class'] = _final

    @classmethod
    def _is_gt_hazard(cls, actor: dict, ego_xy: np.ndarray,
                      ego_vx: float = 0.0, ego_vy: float = 0.0):
        """
        GT conflict predicate: is ``actor`` actually on a collision course
        with the ego?

        Uses linear extrapolation of **relative** velocity (actor − ego)
        to compute the distance of closest approach (DCA).

        Returns
        -------
        (is_hazard, dca, t_ca) : (bool, float, float)
            is_hazard: True if closing, within DCA threshold, within horizon
            dca:       distance of closest approach (m), or inf if not closing
            t_ca:      time of closest approach (s), or -1 if not closing
        """
        speed = actor['speed']
        if speed < cls._GT_HAZARD_SPEED_THRESH:
            return False, float('inf'), -1.0

        # Relative position: ego − actor
        dx = ego_xy[0] - actor['x']
        dy = ego_xy[1] - actor['y']

        # Relative velocity: actor − ego (motion of actor in ego's frame)
        rel_vx = actor['vx'] - ego_vx
        rel_vy = actor['vy'] - ego_vy
        rel_v_sq = rel_vx**2 + rel_vy**2
        if rel_v_sq < 1e-6:
            return False, float('inf'), -1.0

        # Time of closest approach (relative motion)
        # t_ca = dot(ego−actor, actor_vel−ego_vel) / |rel_vel|^2
        t_ca = (dx * rel_vx + dy * rel_vy) / rel_v_sq

        # Must be in the future and within horizon
        if t_ca < 0 or t_ca > cls._GT_HAZARD_TIME_HORIZON:
            return False, float('inf'), float(t_ca)

        # Distance at closest approach (both vehicles extrapolated)
        actor_x_ca = actor['x'] + actor['vx'] * t_ca
        actor_y_ca = actor['y'] + actor['vy'] * t_ca
        ego_x_ca = ego_xy[0] + ego_vx * t_ca
        ego_y_ca = ego_xy[1] + ego_vy * t_ca
        dca = ((actor_x_ca - ego_x_ca)**2 + (actor_y_ca - ego_y_ca)**2)**0.5

        return dca < cls._GT_HAZARD_DCA_THRESH, float(dca), float(t_ca)

    def _reclassify_self_duplicate(self, attr, ego_xy, ego_vx, ego_vy,
                                     ego_id):
        """Reclassify an other_fp as self_ghost if the obstacle is inside a
        heading-aligned near-ego exclusion box.

        Called when behavior_agent flagged is_ego_ghost=True (cid=-1, near ego)
        but GT nearest-match landed on a parked car or non-hazard NPC.
        """
        obs_xy = np.array([attr['obs_x'], attr['obs_y']])
        d = obs_xy - ego_xy

        # Ego heading from velocity; fall back to ego→obs vector if stopped
        ego_speed = (ego_vx**2 + ego_vy**2)**0.5
        if ego_speed > 0.5:
            fwd = np.array([ego_vx, ego_vy]) / ego_speed
        else:
            dist = np.linalg.norm(d)
            if dist < 0.1:
                # obs on top of ego — definitely self-duplicate
                fwd = np.array([1.0, 0.0])
            else:
                fwd = d / dist

        lat_vec = np.array([-fwd[1], fwd[0]])
        along = float(np.dot(d, fwd))
        across = float(abs(np.dot(d, lat_vec)))

        if abs(along) < self._SELFDUP_LONG_M and across < self._SELFDUP_LAT_M:
            attr['ghost_brake_gt'] = True
            attr['false_positive_gt'] = False
            attr['gt_brake_class'] = 'self_ghost'
            attr['gt_provenance'] = 'self_duplicate'
            logger.warning(
                "[GT SELF-DUP] ego %d: track=%s cid=%s reclassified from "
                "other_fp (was prov=%s, matched GT actor %d dist=%.2fm). "
                "Obs in ego box: along=%.1fm, across=%.1fm",
                ego_id, attr.get('trigger_track_id'),
                attr.get('trigger_carla_id'),
                attr.get('_orig_provenance', attr.get('gt_provenance', '?')),
                attr.get('gt_matched_actor_id', -1),
                attr.get('gt_match_dist_m', -1),
                along, across)

    # ─── State-transfer interface (Phase 1: sequential handoff) ────
    def _vm_by_carla_id(self, vehicle_id: int) -> Optional[Any]:
        for vm in self.vehicle_manager_list:
            if vm.vehicle.id == vehicle_id:
                return vm
        return None

    def export_vehicle_state(self, vehicle_id: int) -> Optional[MigrationPayload]:
        """Return a MigrationPayload snapshot for vehicle_id.

        Base implementation produces a minimal payload with no tracker state.
        Override in pluggable subclasses for AB3DMOT-aware snapshots.
        """
        if self._vm_by_carla_id(vehicle_id) is None:
            return None
        track = TrackLatent(
            track_id=-1,
            persistent_vehicle_id=vehicle_id,
            hidden_state=np.zeros(1, dtype=np.float16),
        )
        return MigrationPayload(
            source_locale_id="",
            destination_locale_id="",
            trigger_time_s=0.0,
            tracks=[track],
        )

    def import_vehicle_state(self, vehicle_id: int, payload: MigrationPayload) -> None:
        """Restore per-vehicle tracker state from payload.

        Base implementation is a no-op; override for tracker state restore.
        """

    def relinquish(self, vehicle_id: int) -> Any:
        """Remove vehicle_id from vehicle_manager_list and return its VehicleManager."""
        for i, vm in enumerate(self.vehicle_manager_list):
            if vm.vehicle.id == vehicle_id:
                return self.vehicle_manager_list.pop(i)
        raise KeyError(f"vehicle {vehicle_id} not in vehicle_manager_list")

    def accept(self, vm: Any) -> None:
        """Add a VehicleManager to this edge's vehicle_manager_list."""
        self.vehicle_manager_list.append(vm)

    def export_tracked_obstacle_state(self, carla_id: int) -> Optional[MigrationPayload]:
        """Export KF state for an AB3DMOT-tracked obstacle (no VehicleManager required).

        Base no-op — override in AB3DMOT-aware subclasses.
        """
        return None

    def import_tracked_obstacle_state(self, carla_id: int, payload: MigrationPayload) -> None:
        """Inject a warm KF for a tracked obstacle into this edge's tracker.

        Base no-op — override in AB3DMOT-aware subclasses.
        """

    @staticmethod
    def _dict_extend(dest: Dict[str, list], src: Dict[str, list]) -> None:
        """`dest[k] += src[k]` for every key, creating lists if needed."""
        for k, v in src.items():
            dest.setdefault(k, []).extend(v)


# ──────────────────────────────────────────────────────────────
#  Compatibility aliases (import convenience)
# ──────────────────────────────────────────────────────────────
BaseEdgeManager = _BaseEdgeManager
EdgeManagerBase = _BaseEdgeManager

__all__ = ["_BaseEdgeManager", "BaseEdgeManager", "EdgeManagerBase"]
