# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""CIP-style edge manager (Cooperative Infrastructure Perception, IoTDI '24).

Architecture:
    RSU sensors only at the edge.
    Edge runs detection, tracking, prediction, AND planning for each managed
    vehicle. The edge returns the planner's trajectory (sampled points with a
    target speed each) over a unicast downlink. The vehicle keeps its own
    controller: every tick it selects the target point of the latest
    delivered trajectory the way the local planner does and runs the PID
    controller on it, so it continues along the last valid trajectory while
    a newer one is in flight or lost (CIP, IoTDI '24).

Distinction from the late-fusion baseline:
    Late-fusion: edge publishes tracks; each vehicle's planner consumes them
        locally; vehicle plans and actuates.
    CIP:        edge publishes the planned trajectory; vehicle tracks it
        without local perception or planning.

The consumer boundary for AoI-at-use is therefore the trajectory receipt at
the vehicle (CIP reports its latency to trajectory receipt), not the
planner's track-consume call. Self-ghosting cannot occur because ego tracks
are never republished as identified actors to a vehicle-side planner.

Until 2026-11-02 this manager shipped the finished carla.VehicleControl and
the vehicle was a thin actuator; that placed the boundary at actuation and
did not match CIP.

In our simulation harness, the per-vehicle planner instance lives on
``vm.agent`` for code reuse; conceptually it is executed at the edge.
This placement does not affect the AoI-at-use measurement because compute
time is profiled identically regardless of which process owns the agent.
"""
from __future__ import annotations

import logging
import math
from typing import Any, Dict, List

import carla
import numpy as np

from .edge_manager_infra_only_ab3dmot_linear_predictor import InfraOnlyEdge

logger = logging.getLogger(__name__)

# Unicast downlink payload: a trajectory message of n points, 20 B per point
# (x, y, z, yaw, speed as float32) plus a 16 B header.
_CIP_TRAJ_POINT_BYTES = 20
_CIP_TRAJ_HEADER_BYTES = 16


class CIPEdge(InfraOnlyEdge):
    """CIP-style edge: edge plans, vehicle actuates."""

    def __init__(self, world, cfg, cav_world, carla_client, *args, **kwargs):
        cfg = dict(cfg)
        # CIP is RSU-only by construction; SBA is irrelevant (parent class enforces).
        # Additionally: vehicle does no onboard planning. Mark for diagnostics.
        cfg["cip_mode"] = True
        super().__init__(world, cfg, cav_world, carla_client, *args, **kwargs)
        # Unicast-downlink trajectory queue per vehicle. The edge plans at
        # tick t and sends the planner's trajectory; the vehicle receives it
        # after a UNICAST Uu downlink delay sampled from the ns-3 DL LUT (NOT
        # the SEE-V2X sidelink trace, which is PC5 V2V/V2I, not edge->vehicle
        # unicast). Until a newer trajectory arrives the vehicle keeps tracking
        # the last delivered one with its own controller. This is the CIP
        # consumer boundary: AoI-at-use lives on the trajectory downlink.
        self._cip_pending_plans: Dict[int, list] = {}   # vid -> [(deliver_tick, plan)]
        self._cip_last_plan: Dict[int, Any] = {}        # vid -> last delivered plan (list of (Transform, speed_kmh)) or None

    # ------------------------------------------------------------------
    # Override the per-tick vehicle-advance loop.
    # In the parent class:
    #     for vm in self.vehicle_manager_list:
    #         vm.update_info(tick)              # runs vehicle perception
    #         vm.vehicle.apply_control(vm.run_step())  # vehicle plans + actuates
    #
    # In CIP:
    #     for vm:
    #         vm.localizer.localize()           # pose only, no perception
    #         feed edge perception to vm.agent  # planner sees edge tracks only
    #         plan = planner trajectory         # planner runs (conceptually at edge)
    #         ... unicast downlink delay ...
    #         control = vm.controller.run_step(target of the delivered plan)
    #         vm.vehicle.apply_control(control) # vehicle tracks the delivered plan
    # ------------------------------------------------------------------
    @staticmethod
    def _cip_plan_message(vm, target_speed, target_loc):
        """The trajectory the edge sends: the local planner's sampled
        trajectory (Transform, speed_kmh) points, or the planner's single
        target when the buffer is empty. None means emergency stop."""
        try:
            buf = list(vm.agent.get_local_planner().get_trajectory())
        except Exception:  # noqa: BLE001
            buf = []
        pts = [(tr, float(spd)) for tr, spd in buf if tr is not None]
        if not pts:
            if target_loc is None or not target_speed:
                return None
            tr = target_loc.transform if hasattr(target_loc, 'is_junction') else target_loc
            pts = [(tr, float(target_speed))]
        return pts

    @staticmethod
    def _cip_select_target(plan, ego_pos, ego_spd_kmh):
        """Vehicle side: the first delivered point at least
        max(2.5 m, 0.4 s of travel) ahead along the heading, else the last
        point (the local planner's own rule)."""
        loc = ego_pos.location
        yaw = math.radians(ego_pos.rotation.yaw)
        fx, fy = math.cos(yaw), math.sin(yaw)
        min_ahead = max(2.5, 0.4 * ego_spd_kmh / 3.6)
        for tr, spd in plan:
            dx = tr.location.x - loc.x
            dy = tr.location.y - loc.y
            if dx * fx + dy * fy >= min_ahead:
                return tr, spd
        return plan[-1]
    def _advance_actors(self, tick: int) -> None:
        for vm in self.vehicle_manager_list:
            try:
                # Localize: pose + speed only; no onboard perception.
                vm.localizer.localize()
                ego_pos = vm.localizer.get_ego_pos()
                ego_spd = vm.localizer.get_ego_spd()
            except Exception:  # noqa: BLE001
                logger.exception("CIP: localize failed for vehicle %s; skipping tick",
                                 getattr(vm.vehicle, 'id', '?'))
                continue

            # Edge-side planner: consume the predictions the edge already
            # populated on vm.agent.edge_predictions during the publish step.
            # Provide an empty local objects dict — CIP vehicles have no
            # onboard perception; all obstacles come from edge predictions.
            vid = vm.vehicle.id
            objects: Dict[str, List[Any]] = {"vehicles": [], "traffic_lights": [], "static": []}
            try:
                vm.agent.update_information(ego_pos, ego_spd, objects)
                # agent.run_step() runs the planner (conceptually at the edge)
                # and returns its target; the sampled trajectory behind that
                # target is what the edge sends down to the vehicle.
                vm.agent._current_global_tick = tick
                target_speed, target_pos = vm.agent.run_step()
                plan = self._cip_plan_message(vm, target_speed, target_pos)
            except Exception:  # noqa: BLE001
                logger.exception("CIP: planner failed for vehicle %s; emergency-stopping",
                                 getattr(vm.vehicle, 'id', '?'))
                plan = None

            # Queue the freshly-planned trajectory for delivery after a unicast
            # Uu downlink delay. CIP's consumer boundary IS this trajectory
            # downlink, so it must carry delay on the SAME controlled-AoI axis
            # as every other arm. Default: use the edge latency model
            # (controlled latency = the matrix's independent variable). Opt-in:
            # the ns-3 DL LUT (unicast, payload/N-aware) for the secondary
            # validation scatter only. Never zero-delay (that would make CIP
            # falsely safe).
            if self._lut_sampler is not None:
                payload = _CIP_TRAJ_HEADER_BYTES + _CIP_TRAJ_POINT_BYTES * (len(plan) if plan else 0)
                dl_ms = self._lut_sampler.sample_ms(
                    max(1, len(self.vehicle_manager_list)), max(payload, 64), 'dl')
                dl_ticks = int(math.ceil(dl_ms / self._sim_dt_ms))
            else:
                # Controlled latency: edge latency model maps source->arrival;
                # the (arrival - tick) gap is the trajectory downlink delay.
                dl_ticks = max(0, self.latency_model.stamp(tick) - tick)
            self._cip_pending_plans.setdefault(vid, []).append(
                (tick + dl_ticks, plan))

            # Deliver all trajectories whose downlink delay has elapsed; the
            # most recent delivered one becomes the held trajectory.
            due = [(dt, pl) for (dt, pl) in self._cip_pending_plans[vid] if dt <= tick]
            if due:
                due.sort(key=lambda x: x[0])
                self._cip_last_plan[vid] = due[-1][1]
                self._cip_pending_plans[vid] = [
                    (dt, pl) for (dt, pl) in self._cip_pending_plans[vid] if dt > tick]

            # Vehicle side: track the last DELIVERED trajectory with the
            # vehicle's own controller (the vehicle holds that trajectory until
            # a fresher one arrives over the downlink). Nothing delivered yet,
            # or an emergency-stop message, brakes.
            if vid in self._cip_last_plan:
                held = self._cip_last_plan[vid]
                try:
                    vm.controller.update_info(ego_pos, ego_spd)
                    if held:
                        tgt_tr, tgt_spd = self._cip_select_target(held, ego_pos, ego_spd)
                        control = vm.controller.run_step(tgt_spd, tgt_tr)
                    else:
                        control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                    vm.vehicle.apply_control(control)
                except Exception:  # noqa: BLE001
                    logger.exception("CIP: trajectory tracking failed for vehicle %s", vid)

            # Diagnostics still run on the parent's per-vm helpers.
            try:
                self._label_brake_attributions_gt(vm, tick=tick)
                self._record_time_to_events(tick, vm)
            except Exception:  # noqa: BLE001
                logger.debug("CIP: per-vm metric helper raised", exc_info=True)

        # RSUs still need their per-tick update (perception source for the edge).
        for rsu in self.rsu_manager_list:
            try:
                rsu.update_info()
                rsu.run_step()
            except Exception:  # noqa: BLE001
                logger.exception("CIP: RSU advance failed for rsu=%s",
                                 getattr(rsu, 'id', '?'))
