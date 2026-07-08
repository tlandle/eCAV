# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""CIP-style edge manager (Cooperative Infrastructure Perception, IoTDI '24).

Architecture:
    RSU sensors only at the edge.
    Edge runs detection, tracking, prediction, AND planning for each managed
    vehicle. The vehicle is a thin actuation client that receives a
    carla.VehicleControl each tick and applies it.

Distinction from the late-fusion baseline:
    Late-fusion: edge publishes tracks; each vehicle's planner consumes them
        locally; vehicle plans and actuates.
    CIP:        edge publishes commands; vehicle applies them without local
        planning.

The consumer boundary for AoI-at-use is therefore the actuation interface,
not the planner's track-consume call. Self-ghosting cannot occur because
ego tracks are never republished as identified actors to a vehicle-side
planner.

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

# Per-control unicast downlink payload (a carla.VehicleControl is a few floats).
_CIP_CONTROL_BYTES = 64


class CIPEdge(InfraOnlyEdge):
    """CIP-style edge: edge plans, vehicle actuates."""

    def __init__(self, world, cfg, cav_world, carla_client, *args, **kwargs):
        cfg = dict(cfg)
        # CIP is RSU-only by construction; SBA is irrelevant (parent class enforces).
        # Additionally: vehicle does no onboard planning. Mark for diagnostics.
        cfg["cip_mode"] = True
        super().__init__(world, cfg, cav_world, carla_client, *args, **kwargs)
        # Unicast-downlink command queue per vehicle. The edge plans a control
        # at tick t; the vehicle (thin actuator) receives it after a UNICAST Uu
        # downlink delay sampled from the ns-3 DL LUT (NOT the SEE-V2X sidelink
        # trace, which is PC5 V2V/V2I, not edge->vehicle unicast). Until the new
        # command arrives the vehicle holds its last delivered control. This is
        # the CIP consumer boundary: AoI-at-use lives on the command downlink.
        self._cip_pending_cmds: Dict[int, list] = {}   # vid -> [(deliver_tick, control)]
        self._cip_last_control: Dict[int, Any] = {}     # vid -> last delivered control

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
    #         control = vm.agent.run_step()     # planner runs (conceptually at edge)
    #         vm.vehicle.apply_control(control) # vehicle is a thin actuator
    # ------------------------------------------------------------------
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
                # agent.run_step() returns (target_speed, target_pos); the
                # controller converts that to a carla.VehicleControl, exactly
                # as the normal vehicle path does (vm.run_step). CIP runs this
                # at the edge and ships the resulting control to the actuator.
                vm.agent._current_global_tick = tick
                target_speed, target_pos = vm.agent.run_step()
                control = vm.controller.run_step(target_speed, target_pos)
            except Exception:  # noqa: BLE001
                logger.exception("CIP: planner failed for vehicle %s; emergency-stopping",
                                 getattr(vm.vehicle, 'id', '?'))
                control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            # Queue the freshly-planned control for delivery after a unicast Uu
            # downlink delay. CIP's consumer boundary IS this command downlink,
            # so it must carry delay on the SAME controlled-AoI axis as every
            # other arm. Default: use the edge latency model (controlled
            # latency = the matrix's independent variable). Opt-in: the ns-3 DL
            # LUT (unicast, payload/N-aware) for the secondary validation
            # scatter only. Never zero-delay (that would make CIP falsely safe).
            if self._lut_sampler is not None:
                dl_ms = self._lut_sampler.sample_ms(
                    max(1, len(self.vehicle_manager_list)),
                    _CIP_CONTROL_BYTES, 'dl')
                dl_ticks = int(math.ceil(dl_ms / self._sim_dt_ms))
            else:
                # Controlled latency: edge latency model maps source->arrival;
                # the (arrival - tick) gap is the command downlink delay.
                dl_ticks = max(0, self.latency_model.stamp(tick) - tick)
            self._cip_pending_cmds.setdefault(vid, []).append(
                (tick + dl_ticks, control))

            # Deliver all commands whose downlink delay has elapsed; the most
            # recent delivered command becomes the held control.
            due = [(dt, c) for (dt, c) in self._cip_pending_cmds[vid] if dt <= tick]
            if due:
                due.sort(key=lambda x: x[0])
                self._cip_last_control[vid] = due[-1][1]
                self._cip_pending_cmds[vid] = [
                    (dt, c) for (dt, c) in self._cip_pending_cmds[vid] if dt > tick]

            # Apply the last DELIVERED control (thin actuator holds last command
            # until a fresher one arrives over the downlink).
            held = self._cip_last_control.get(vid)
            if held is not None:
                try:
                    vm.vehicle.apply_control(held)
                except Exception:  # noqa: BLE001
                    logger.exception("CIP: apply_control failed for vehicle %s", vid)

            # Diagnostics still run on the parent's per-vm helpers.
            try:
                self._label_brake_attributions_gt(vm)
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
