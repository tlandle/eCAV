# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Client Metrics

Tracks client-side timing metrics: perception, tracking, localization,
control times, collisions, and lane invasions.
"""
from ecav.core.plan.planning_metrics import PlanningMetrics
from ecav.core.common.traffic_event import TrafficEvent, TrafficEventType

import ecloud_pb2 as ecloud


class ClientMetrics(PlanningMetrics):
    """
    Tracks client-side processing statistics.

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehicle.

    Attributes
    ----------
    perception_time_list : list
        Time for perception processing.

    tracking_time_list : list
        Time for tracking processing.

    localization_time_list : list
        Time for localization processing.

    control_time_list : list
        Time for control processing.
    """

    def __init__(self, actor_id):
        super(ClientMetrics, self).__init__(actor_id)

        self.perception_time_list = []
        self.tracking_time_list = []
        self.lidar_fusion_time_list = []
        self.localization_time_list = []
        self.update_info_time_list = []
        self.agent_update_info_time_list = []
        self.controller_update_info_time_list = []
        self.agent_step_time_list = []
        self.controller_step_time_list = []
        self.vehicle_step_time_list = []
        self.control_time_list = []
        self.timestamps_list = []
        self.collisions_event_list = []
        self.lane_invasions_list = []
        self.detections_time_list = []

        self.debug_data = {
            "client_control_time": self.control_time_list,
            "client_perception_time": self.perception_time_list,
            "client_tracking_time": self.tracking_time_list,
            "client_localization_time": self.localization_time_list,
            "client_update_info_time": self.update_info_time_list,
            "client_agent_update_info_time": self.agent_update_info_time_list,
            "client_controller_update_info_time_list": self.controller_update_info_time_list,
            "client_agent_step_time_list": self.agent_step_time_list,
            "client_controller_step_time_list": self.controller_step_time_list,
            "client_vehicle_step_time_list": self.vehicle_step_time_list,
            "client_control_time_list": self.control_time_list,
            "client_collisions_list": self.collisions_event_list,
            "client_lane_invasions_list": self.lane_invasions_list,
            "detections_time_list": self.detections_time_list,
            "lidar_fusion_time_list": self.lidar_fusion_time_list
        }

    def get_debug_data(self):
        return self.debug_data

    def update_perception_time(self, tick_time_step=None):
        """Update the client perception time."""
        self.perception_time_list.append(tick_time_step)

    def update_detections_time(self, tick_time_step=None):
        """Update the client detections time."""
        self.detections_time_list.append(tick_time_step)

    def update_tracking_time(self, tick_time_step=None):
        """Update the client tracking time."""
        self.tracking_time_list.append(tick_time_step)

    def update_lidar_fusion_time(self, tick_time_step=None):
        """Update the client lidar fusion time."""
        self.lidar_fusion_time_list.append(tick_time_step)

    def update_localization_time(self, tick_time_step=None):
        """Update the client localization time."""
        self.localization_time_list.append(tick_time_step)

    def update_update_info_time(self, time=None):
        """Update the general client update time."""
        self.update_info_time_list.append(time)

    def update_agent_update_info_time(self, time=None):
        """Update the agent update time."""
        self.agent_update_info_time_list.append(time)

    def update_controller_update_info_time(self, time=None):
        """Update the controller update time."""
        self.controller_update_info_time_list.append(time)

    def update_agent_step_time(self, time=None):
        """Update the agent step time."""
        self.agent_step_time_list.append(time)

    def update_vehicle_step_time(self, time=None):
        """Update the vehicle step time."""
        self.vehicle_step_time_list.append(time)

    def update_controller_step_time(self, time=None):
        """Update the controller step time."""
        self.controller_step_time_list.append(time)

    def update_control_time(self, time=None):
        """Update the client control time."""
        self.control_time_list.append(time)

    def update_timestamp(self, timestamps: ecloud.Timestamps):
        """Update the client timestamps."""
        t = ecloud.Timestamps()
        t.CopyFrom(timestamps)
        self.timestamps_list.append(t)

    def update_collision(self, collision_info=None):
        self.collisions_event_list.append(collision_info)

    def update_lane_invasions(self, lane_invasion_info=None):
        self.lane_invasions_list.append(lane_invasion_info)

    def get_client_metrics(self):
        """
        Get aggregated client timing metrics.

        Returns
        -------
        dict
            Dictionary with client-specific timing metrics
        """
        import numpy as np

        def safe_stats(lst):
            valid = [t for t in lst if t is not None]
            if not valid:
                return None, None
            return float(np.mean(valid)), float(np.std(valid))

        perc_avg, perc_std = safe_stats(self.perception_time_list)
        track_avg, track_std = safe_stats(self.tracking_time_list)
        loc_avg, loc_std = safe_stats(self.localization_time_list)
        ctrl_avg, ctrl_std = safe_stats(self.control_time_list)

        return {
            'perception_time_avg_ms': perc_avg,
            'perception_time_std_ms': perc_std,
            'tracking_time_avg_ms': track_avg,
            'tracking_time_std_ms': track_std,
            'localization_time_avg_ms': loc_avg,
            'localization_time_std_ms': loc_std,
            'control_time_avg_ms': ctrl_avg,
            'control_time_std_ms': ctrl_std,
            'collision_count': len(self.collisions_event_list),
            'lane_invasion_count': len(self.lane_invasions_list),
        }

    def serialize_debug_info(self, proto_debug_helper):
        # TODO: extend instead of append? or [:] = ?

        for obj in self.perception_time_list:
            proto_debug_helper.perception_time_list.append(obj)

        for obj in self.localization_time_list:
            proto_debug_helper.localization_time_list.append(obj)

        for obj in self.update_info_time_list:
            proto_debug_helper.update_info_time_list.append(obj)

        for obj in self.agent_update_info_time_list:
            proto_debug_helper.agent_update_info_time_list.append(obj)

        for obj in self.controller_update_info_time_list:
            proto_debug_helper.controller_update_info_time_list.append(obj)

        for obj in self.agent_step_time_list:
            proto_debug_helper.agent_step_time_list.append(obj)

        for obj in self.vehicle_step_time_list:
            proto_debug_helper.vehicle_step_time_list.append(obj)

        for obj in self.controller_step_time_list:
            proto_debug_helper.controller_step_time_list.append(obj)

        for obj in self.control_time_list:
            proto_debug_helper.control_time_list.append(obj)

        for obj in self.timestamps_list:
            t = ecloud.Timestamps()
            t.CopyFrom(obj)
            proto_debug_helper.timestamps_list.append(t)

        for obj in self.collisions_event_list:
            collision_event = ecloud.CollisionEvent()
            collision_event.type_id = obj.get_dict()['type']
            collision_event.other_actor_id = obj.get_dict()['id']
            collision_event.location.x = obj.get_dict()['x']
            collision_event.location.y = obj.get_dict()['y']
            collision_event.location.z = obj.get_dict()['z']
            proto_debug_helper.collisions_event_list.append(collision_event)

        for obj in self.lane_invasions_list:
            lane_invasion_event = ecloud.LaneInvasionEvent()
            lane_invasion_event.location.x = obj.get_dict()['x']
            lane_invasion_event.location.y = obj.get_dict()['y']
            lane_invasion_event.location.z = obj.get_dict()['z']
            proto_debug_helper.lane_invasions_list.append(lane_invasion_event)

    def deserialize_debug_info(self, proto_debug_helper):
        # call from Sim API to populate locally

        self.perception_time_list.clear()
        for obj in proto_debug_helper.perception_time_list:
            self.perception_time_list.append(obj)

        self.localization_time_list.clear()
        for obj in proto_debug_helper.localization_time_list:
            self.localization_time_list.append(obj)

        self.update_info_time_list.clear()
        for obj in proto_debug_helper.update_info_time_list:
            self.update_info_time_list.append(obj)

        self.agent_update_info_time_list.clear()
        for obj in proto_debug_helper.agent_update_info_time_list:
            self.agent_update_info_time_list.append(obj)

        self.controller_update_info_time_list.clear()
        for obj in proto_debug_helper.controller_update_info_time_list:
            self.controller_update_info_time_list.append(obj)

        self.agent_step_time_list.clear()
        for obj in proto_debug_helper.agent_step_time_list:
            self.agent_step_time_list.append(obj)

        self.vehicle_step_time_list.clear()
        for obj in proto_debug_helper.vehicle_step_time_list:
            self.vehicle_step_time_list.append(obj)

        self.controller_step_time_list.clear()
        for obj in proto_debug_helper.controller_step_time_list:
            self.controller_step_time_list.append(obj)

        self.control_time_list.clear()
        for obj in proto_debug_helper.control_time_list:
            self.control_time_list.append(obj)

        self.timestamps_list.clear()
        for obj in proto_debug_helper.timestamps_list:
            t = ecloud.Timestamps()
            t.CopyFrom(obj)
            self.timestamps_list.append(t)

        self.collisions_event_list.clear()
        for obj in proto_debug_helper.collisions_event_list:

            if ('static' in obj.type_id or 'traffic' in obj.type_id) \
                and 'sidewalk' not in obj.type_id:
                actor_type = TrafficEventType.COLLISION_STATIC
            elif 'vehicle' in obj.type_id:
                actor_type = TrafficEventType.COLLISION_VEHICLE
            elif 'walker' in obj.type_id:
                actor_type = TrafficEventType.COLLISION_PEDESTRIAN
            else:
                continue

            collision_event = TrafficEvent(event_type=actor_type)
            collision_event.set_dict({
                'type': obj.type_id,
                'id': obj.other_actor_id,
                'x': obj.location.x,
                'y': obj.location.y,
                'z': obj.location.z})
            self.collisions_event_list.append(collision_event)

        self.lane_invasions_list.clear()
        for obj in proto_debug_helper.lane_invasions_list:
            lane_invasion_event = TrafficEvent(event_type=TrafficEventType.LANE_INVASION)
            lane_invasion_event.set_dict({
                'x': obj.location.x,
                'y': obj.location.y,
                'z': obj.location.z})

            self.lane_invasions_list.append(lane_invasion_event)


# Backward compatibility alias
ClientDebugHelper = ClientMetrics
