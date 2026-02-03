# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Edge Metrics

Tracks edge-specific timing metrics: algorithm time, tracking time,
prediction time, and network latency.
"""
from opencda.core.plan.planning_metrics import PlanningMetrics


class EdgeMetrics(PlanningMetrics):
    """
    Tracks edge processing statistics.

    Parameters
    ----------
    actor_id : int
        The actor ID of the selected vehicle.

    Attributes
    ----------
    algorithm_time_list : list
        The list containing algorithm execution time for all time-steps.

    tracking_time_list : list
        The list containing tracking time for all time-steps.

    prediction_time_list : list
        The list containing prediction time for all time-steps.

    latency_list : list
        The list containing network latency for all time-steps.
    """

    def __init__(self, actor_id):
        super(EdgeMetrics, self).__init__(actor_id)

        self.algorithm_time_list = [[]]
        self.tracking_time_list = [[]]
        self.prediction_time_list = [[]]
        self.latency_list = [[]]

    def update_edge(self, algorithm_time_step=None, tracking_time=None, prediction_time=None, latency=None):
        """
        Update the edge-related timing information.

        Parameters
        ----------
        algorithm_time_step : float
            Time taken for the main algorithm step.
        tracking_time : float
            Time taken for tracking.
        prediction_time : float
            Time taken for prediction.
        latency : float
            Network latency.
        """
        self.algorithm_time_list[0].append(algorithm_time_step)
        self.tracking_time_list[0].append(tracking_time)
        self.prediction_time_list[0].append(prediction_time)
        self.latency_list[0].append(latency)

    def get_edge_metrics(self):
        """
        Get aggregated edge timing metrics.

        Returns
        -------
        dict
            Dictionary with edge-specific timing metrics
        """
        import numpy as np

        algorithm_times = [t for t in self.algorithm_time_list[0] if t is not None]
        tracking_times = [t for t in self.tracking_time_list[0] if t is not None]
        prediction_times = [t for t in self.prediction_time_list[0] if t is not None]
        latencies = [t for t in self.latency_list[0] if t is not None]

        return {
            'algorithm_time_avg_ms': float(np.mean(algorithm_times)) if algorithm_times else None,
            'algorithm_time_std_ms': float(np.std(algorithm_times)) if algorithm_times else None,
            'tracking_time_avg_ms': float(np.mean(tracking_times)) if tracking_times else None,
            'tracking_time_std_ms': float(np.std(tracking_times)) if tracking_times else None,
            'prediction_time_avg_ms': float(np.mean(prediction_times)) if prediction_times else None,
            'prediction_time_std_ms': float(np.std(prediction_times)) if prediction_times else None,
            'latency_avg_ms': float(np.mean(latencies)) if latencies else None,
            'latency_std_ms': float(np.std(latencies)) if latencies else None,
        }


# Backward compatibility alias
EdgeDebugHelper = EdgeMetrics
