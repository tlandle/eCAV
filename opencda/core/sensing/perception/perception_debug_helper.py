# -*- coding: utf-8 -*-
"""
Analysis + Visualization functions for perception
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

class PerceptionDebugHelper(object):
    """
    This class aims to save statistics for perception debugging.

    Parameters:
    -actor_id : int
        The actor ID of the target vehicle for bebuging.

    Attributes

    """

    def __init__(self, actor_id):
        self.actor_id = actor_id
        self.detection_time_list = [[]]
        self.tracking_time_list = [[]]

        self.count = 0

    def update(self, detection_time, tracking_time):
        """
        Update the detection and tracking time for the target vehicle.
        Parameters:
        -detection_time : float
            The detection time of the target vehicle.
        -tracking_time : float
            The tracking time of the target vehicle.

        """
        self.count += 1
        # at the very beginning, the vehicle is in a spawn state, so we should
        # filter out the first 100 data points.
        if self.count > 100: 
            self.detection_time_list[0].append(detection_time * 1000)
            self.tracking_time_list[0].append(tracking_time * 1000)

    def update_agent_step_list(self, decision_index, time_s=None):
        self.agent_step_list[decision_index].append(time_s*1000)

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
        figure = plt.figure()
        plt.subplot(311)
        open_plt.draw_velocity_profile_single_plot(self.speed_list)

        plt.subplot(312)
        open_plt.draw_acceleration_profile_single_plot(self.acc_list)

        plt.subplot(313)
        open_plt.draw_ttc_profile_single_plot(self.ttc_list)

        figure.suptitle('planning profile of actor id %d' % self.actor_id)

        # calculate the statistics
        spd_avg = np.mean(np.array(self.speed_list[0]))
        spd_std = np.std(np.array(self.speed_list[0]))

        acc_avg = np.mean(np.array(self.acc_list[0]))
        acc_std = np.std(np.array(self.acc_list[0]))

        ttc_array = np.array(self.ttc_list[0])
        ttc_array = ttc_array[ttc_array < 1000]
        ttc_avg = np.mean(ttc_array)
        ttc_std = np.std(ttc_array)

        perform_txt = 'Speed average: %f (m/s), ' \
                      'Speed std: %f (m/s) \n' % (spd_avg, spd_std)

        perform_txt += 'Acceleration average: %f (m/s), ' \
                       'Acceleration std: %f (m/s) \n' % (acc_avg, acc_std)

        perform_txt += 'TTC average: %f (m/s), ' \
                       'TTC std: %f (m/s) \n' % (ttc_avg, ttc_std)

        metrics = {
            "actor_id": self.actor_id,
            "avg_acceleration_mps": float(acc_avg),
            "avg_speed_mps": float(spd_avg),
            "avg_ttc_s": float(ttc_avg),
            "std_acceleration_mps": float(acc_std),
            "std_speed_mps": float(spd_std),
            "std_ttc_s": float(ttc_std)
        }



        for idx, sub_list in enumerate(self.agent_step_list):
            sub_list_mean = np.nanmean(np.array(sub_list))
            sub_list_std = np.nanstd(np.array(sub_list))
            logger.debug("actor %s | agent step list_%s - mean: %s", self.actor_id, idx, sub_list_mean)
            logger.debug("actor %s | agent step list_%s - std: %s", self.actor_id, idx, sub_list_std)

        return figure, perform_txt, metrics

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
