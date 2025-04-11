# -*- coding: utf-8 -*-
"""
Analysis + Visualization functions for safety
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License:  TDG-Attribution-NonCommercial-NoDistrib
import warnings
import logging

import numpy as np
import matplotlib.pyplot as plt

import opencda.core.plan.drive_profile_plotting as open_plt

import ecloud_pb2 as ecloud

logger = logging.getLogger(__name__)

class SafetyDebugHelper(object):
    """
    This class aims to save statistics for planner behaviour.

    Parameters:
    -actor_id : int
        The actor ID of the target vehicle for bebuging.

    Attributes
    -speed_list : list
        The list containing speed info(m/s) of all time-steps.
    -acc_list : list
        The list containing acceleration info(m^2/s) of all time-steps.
    -ttc_list : list
        The list containing ttc info(s) for all time-steps.
    -count : int
        Used to count how many simulation steps have been executed.

    """

    def __init__(self, actor_id):
        self.actor_id = actor_id
        self.status_dict_list = []
        self.count = 0

    def update(self, status_dict):
        """
        Update the safety info.
        Args:
            -ego_speed (float): Ego speed in km/h.
            -ttc (flot): Time to collision in seconds.

        """
        self.count += 1
        # at the very beginning, the vehicle is in a spawn state, so we should
        # filter out the first 100 data points.
        if self.count > 100:
            self.status_dict_list.append(status_dict)
            
    def evaluate(self):
        """
        Evaluate the target vehicle and visulize the plot.
        Returns:
            -figure (matplotlib.pyplot.figure): The target vehicle's planning
             profile (velocity, acceleration, and ttc).
            -perform_txt (txt file): The target vehicle's planning profile
            as text files.

        """


        warnings.filterwarnings('ignore')
        # draw speed, acc and ttc plotting
        collisions = 0
        for status_dict in self.status_dict_list:
            if status_dict['collision'] == True:
                collisions += 1
        perform_txt += "Total Collisions: %d \n" % collisions

        return figure, perform_txt

    def serialize_debug_info(self, proto_debug_helper):
        # seems we only ever access [0] anywhere...
        # but need to consider this when de-serializing info from protobuf
        # TODO: extend instead of append? or [:] = ?

        for obj in self.speed_list[0]:
            proto_debug_helper.speed_list.append(obj)

        for obj in self.acc_list[0]:
            proto_debug_helper.acc_list.append(obj)

        for obj in self.ttc_list[0]:
            proto_debug_helper.ttc_list.append(obj)

        for sub_step_time_list in self.agent_step_list:
            step_list = proto_debug_helper.agent_step_list.add()
            for obj in sub_step_time_list:
                step_list.time_list.append(obj)

    def deserialize_debug_info(self, proto_debug_helper):
        # call from Sim API to populate locally

        self.ttc_list[0].clear()
        for obj in proto_debug_helper.ttc_list:
            self.ttc_list[0].append(obj)

        self.acc_list[0].clear()
        for obj in proto_debug_helper.acc_list:
            self.acc_list[0].append(obj)

        self.speed_list[0].clear()
        for obj in proto_debug_helper.speed_list:
            self.speed_list[0].append(obj)

        for time_list in self.agent_step_list:
            time_list.clear()
        for idx, proto_agent_list in enumerate(proto_debug_helper.agent_step_list):
            for obj in proto_agent_list.time_list:
                self.agent_step_list[idx].append(obj)
