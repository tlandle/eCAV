#!/usr/bin/env python

"""Scenario B — Town06 left-merge / obstacle hand-off.

Ego drives east in the right (inner) lane and must merge left around a
stationary emergency vehicle. A fast NPC approaches from behind in the left
(outer) lane. `SyncArrival` coordinates the NPC so it arrives at the merge
zone roughly when ego does — i.e. the left lane is unsafe at the moment ego
would merge. As the NPC crosses the locale 0 -> locale 1 boundary, the ecav
loop fires an obstacle hand-off so locale 1 gets the NPC's KF history before
its own RSU can see it.

Actor layout:
    ego_vehicles[0]    = CAV (right lane, ~43 km/h east)
    other_actors[0]    = emergency vehicle (stationary, blocks right lane)
    other_actors[1]    = fast NPC (left lane, SyncArrival then 80 km/h east)
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    WaypointFollower,
    SyncArrival,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenarios.basic_scenario import BasicScenario


class Scenario_MultiEdgeLeftMerge(BasicScenario):
    """Town06 forced-left-merge with a fast NPC crossing the locale boundary."""

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=600, vehicle_index=-1,
                 scenario_params=None, distributed=False):
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # scenario_params is a list of "key=value" strings (same convention as scenario_3).
        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 43))

        # ── Tunable NPC coordination (expect one iteration after a live run) ──
        # SyncArrival drives the NPC so its ETA to sync_target matches ego's ETA
        # to the same point; the NPC then holds npc_final_speed until termination.
        # The meeting longitude (x=250) sits ~30 m short of the emergency vehicle
        # (x=280) so the left lane is occupied exactly as ego reaches the merge zone.
        self.sync_target = carla.Location(x=250.0, y=143.5, z=1.0)
        self.sync_end_distance = 15.0     # end sync phase when NPC within this of target
        self.npc_final_speed_mps = 22.0   # ~80 km/h post-sync
        self.terminate_drive_distance = 300.0

        super(Scenario_MultiEdgeLeftMerge, self).__init__(
            "Scenario_MultiEdgeLeftMerge",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
            vehicle_index=vehicle_index,
            scenario_params=scenario_params)

    def _initialize_actors(self, config):
        # Ego/RSU containers (distributed) don't spawn background actors.
        if self.distributed and self.vehicle_index >= 0:
            return

        # Spawn both other_actors at the XML transforms (z=-500 keeps them
        # underground); ActorTransformSetter teleports them up in the tree.
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            if actor is None:
                raise RuntimeError(
                    f"Failed to spawn {actor_config.model} at {actor_config.transform}")
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=True)
            actor.set_autopilot(False)

        # Visible transforms: same x/y, lifted +501 back to ground level (z≈1).
        emrg_t = self.other_actors[0].get_transform()
        self.emrg_visible = carla.Transform(
            carla.Location(emrg_t.location.x, emrg_t.location.y, emrg_t.location.z + 501),
            emrg_t.rotation)
        npc_t = self.other_actors[1].get_transform()
        self.npc_visible = carla.Transform(
            carla.Location(npc_t.location.x, npc_t.location.y, npc_t.location.z + 501),
            npc_t.rotation)

    def _create_behavior(self):
        # Ego/RSU containers (distributed): minimal tree, terminate on ego progress.
        if self.distributed and self.vehicle_index >= 0:
            root = py_trees.composites.Parallel(
                "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            root.add_child(DriveDistance(self.ego_vehicles[self.vehicle_index],
                                         self.terminate_drive_distance))
            return root

        emrg = self.other_actors[0]
        npc = self.other_actors[1]

        # Emergency vehicle: teleport up, then hold position (blocks right lane).
        emrg_seq = py_trees.composites.Sequence("EmrgVehicle")
        emrg_seq.add_child(ActorTransformSetter(emrg, self.emrg_visible))
        emrg_seq.add_child(Idle())

        # Fast NPC: teleport up → SyncArrival (ETA-matched to ego) until it nears
        # the meeting point → WaypointFollower at final speed → hold.
        # SyncArrival.update() never returns SUCCESS, so it is wrapped in a
        # SUCCESS_ON_ONE Parallel with a distance trigger that ends the sync phase.
        sync_phase = py_trees.composites.Parallel(
            "NPCSyncPhase", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sync_phase.add_child(SyncArrival(npc, self.ego_vehicles[0], self.sync_target))
        sync_phase.add_child(InTriggerDistanceToLocation(
            npc, self.sync_target, self.sync_end_distance))

        npc_seq = py_trees.composites.Sequence("FastNPC")
        npc_seq.add_child(ActorTransformSetter(npc, self.npc_visible))
        npc_seq.add_child(sync_phase)
        npc_seq.add_child(WaypointFollower(npc, self.npc_final_speed_mps))
        npc_seq.add_child(Idle())

        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(emrg_seq)
        root.add_child(npc_seq)
        root.add_child(DriveDistance(self.ego_vehicles[0], self.terminate_drive_distance))
        return root

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
