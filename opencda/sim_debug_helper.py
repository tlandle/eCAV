# -*- coding: utf-8 -*-
"""
Analysis + visualization functions for platooning
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math

from opencda.core.plan.planer_debug_helper \
    import PlanDebugHelper


class SimDebugHelper(PlanDebugHelper):
    """This class aims to save statistics for platoon behaviour

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehcile.

    Attributes
    ----------
    time_gap_list : list
        The list containing intra-time-gap(s) of all time-steps.

    dist_gap_list : list
        The list containing distance gap(s) of all time-steps.
    """

    def __init__(self, actor_id):
        super(SimDebugHelper, self).__init__(actor_id)

        self.world_tick_time_list = [[]]
        self.client_tick_time_list = [[]]
        self.sim_start_timestamp = None
        self.startup_time_ms = 0
        self.shutdown_time_ms = 0
        self.network_time_dict = {}
        self.client_tick_time_dict = {}
        self.network_time_dict_per_client = {}
        self.client_tick_time_dict_per_client = {}   
        self.idle_time_dict = {}
        self.client_process_time_dict = {}
        self.client_velocity_dict = {}

    def update_velocity_per_client_timestamp(self, tick_id: int, velocity=None):
        if tick_id not in self.client_velocity_dict:
          self.client_velocity_dict[tick_id] = []
        scalar_v = None
        if velocity is not None:  
            scalar_v = math.sqrt(pow(velocity.x, 2) + pow(velocity.y, 2) + pow(velocity.z, 2)) 
        self.client_velocity_dict[tick_id].append(scalar_v)

    def update_world_tick(self, tick_time_step=None):
        self.world_tick_time_list[0].append(tick_time_step)

    def update_client_tick(self, tick_time_step=None):
        self.client_tick_time_list[0].append(tick_time_step)

    def update_overall_step_time_timestamp(self, tick_id: int, overall_step_time_ms):
        self.client_tick_time_dict[tick_id] = overall_step_time_ms

    def update_sim_start_timestamp(self, timestamp=None):
        self.sim_start_timestamp = timestamp

    def update_network_time_timestamp(self, tick_id: int, network_time_ms=None):
        self.network_time_dict[tick_id] = network_time_ms

    def update_network_time_per_client_timestamp(self, vehicle_index, time_step=None):
        if vehicle_index not in self.network_time_dict_per_client:
          self.network_time_dict_per_client[vehicle_index] = []
        self.network_time_dict_per_client[vehicle_index].append(time_step)

    def update_overall_step_time_per_client_timestamp(self, vehicle_index, time_step=None):
        if vehicle_index not in self.client_tick_time_dict_per_client:
          self.client_tick_time_dict_per_client[vehicle_index] = []
        self.client_tick_time_dict_per_client[vehicle_index].append(time_step)

    def update_idle_time_timestamp(self, vehicle_index, time_step=None):
        if vehicle_index not in self.idle_time_dict:
          self.idle_time_dict[vehicle_index] = []
        self.idle_time_dict[vehicle_index].append(time_step)

    def update_client_process_time_timestamp(self, vehicle_index, time_step=None):
        if vehicle_index not in self.client_process_time_dict:
          self.client_process_time_dict[vehicle_index] = []
        self.client_process_time_dict[vehicle_index].append(time_step)
        