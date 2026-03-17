# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Platoon Metrics

Tracks platooning-specific metrics: time gap and distance gap
between platoon members.
"""
from ecav.core.plan.planning_metrics import PlanningMetrics


class PlatoonMetrics(PlanningMetrics):
    """
    Tracks platoon-specific statistics.

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehicle.

    Attributes
    ----------
    time_gap_list : list
        The list containing intra-time-gap(s) of all time-steps.

    dist_gap_list : list
        The list containing distance gap(s) of all time-steps.
    """

    def __init__(self, actor_id):
        super(PlatoonMetrics, self).__init__(actor_id)

        self.time_gap_list = [[]]
        self.dist_gap_list = [[]]

    def update(self, ego_speed, ttc, time_gap=None, dist_gap=None):
        """
        Update the platoon related vehicle information.

        Parameters
        ----------
        ego_speed : float
            Ego vehicle speed.

        ttc : float
            Ego vehicle time-to-collision.

        time_gap : float
            Ego vehicle time gap with the front vehicle.

        dist_gap : float
            Ego vehicle distance gap with front vehicle.
        """
        super().update(ego_speed, ttc)
        # at the very beginning, the vehicle speed is 0, which causes
        # an infinite time gap.  So we need to filter out the first
        # 100 data points.
        if self.count > 100:
            self.time_gap_list[0].append(time_gap)
            self.dist_gap_list[0].append(dist_gap)

    def get_platoon_metrics(self):
        """
        Get aggregated platoon metrics.

        Returns
        -------
        dict
            Dictionary with platoon-specific metrics
        """
        import numpy as np

        time_gaps = [t for t in self.time_gap_list[0] if t is not None]
        dist_gaps = [d for d in self.dist_gap_list[0] if d is not None]

        return {
            'time_gap_avg_s': float(np.mean(time_gaps)) if time_gaps else None,
            'time_gap_std_s': float(np.std(time_gaps)) if time_gaps else None,
            'time_gap_min_s': float(np.min(time_gaps)) if time_gaps else None,
            'time_gap_max_s': float(np.max(time_gaps)) if time_gaps else None,
            'dist_gap_avg_m': float(np.mean(dist_gaps)) if dist_gaps else None,
            'dist_gap_std_m': float(np.std(dist_gaps)) if dist_gaps else None,
            'dist_gap_min_m': float(np.min(dist_gaps)) if dist_gaps else None,
            'dist_gap_max_m': float(np.max(dist_gaps)) if dist_gaps else None,
        }


# Backward compatibility alias
PlatoonDebugHelper = PlatoonMetrics
