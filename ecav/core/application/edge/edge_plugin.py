# -*- coding: utf-8 -*-

"""Edge plugin for communication and track FSM
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import warnings

from ecav.core.common.misc import compute_distance, cal_distance_angle
from ecav.core.application.platooning.fsm import FSM


class EdgePlugin(object):
    """
    Edge plugin inside the V2X manager.

    Parameters
    ----------
    search_range : float
        The search range of the communication equipment.

    cda_enabled : boolean
        Whether connectivity is supported.

    Attributes
    ----------


    ego_pos : carla.transformation
        The current position (i.e., location and rotation) of the ego vehicle.

    ego_spd : float
        The current speed(km/h) of the ego vehicle.

    """

    def __init__(self, search_range, cda_enabled):

        self.search_range = search_range
        self.cda_enabled = cda_enabled

        self.edge_node = None

        # ego speed and position
        self.ego_pos = None
        self.ego_spd = None


        # the platoon in the black list won't be considered again
        self.platooning_blacklist = []

    def update_info(self, ego_pos, ego_spd):
        """
        Update the ego position and speed

        Parameters
        ----------
        ego_pos: carla.Transform
            Ego pose.

        ego_spd : float
            Ego speed(km/h).
        """
        self.ego_pos = ego_pos
        self.ego_spd = ego_spd
    
       
