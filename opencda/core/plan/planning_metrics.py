# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Planning Metrics

Tracks planning behavior metrics: speed, acceleration, jerk, TTC, lateral deviation.
Used by vehicle managers to evaluate planning performance.
"""
import warnings
import logging

import numpy as np
import matplotlib.pyplot as plt

import opencda.core.plan.drive_profile_plotting as open_plt

import ecloud_pb2 as ecloud

logger = logging.getLogger(__name__)


class PlanningMetrics(object):
    """
    Tracks planning behavior statistics for a vehicle.

    Parameters:
    -actor_id : int
        The actor ID of the target vehicle.

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
        self.speed_list = [[]] # doesn't ever use the double-list; only ever index 0
        self.acc_list = [[]] # doesn't ever use the double-list; only ever index 0
        self.ttc_list = [[]] # doesn't ever use the double-list; only ever index 0
        self.jerk_list = [[]]  # Rate of change of acceleration (m/s³) - comfort metric
        self.lateral_deviation_list = [[]]  # Deviation from planned path (m)
        self.position_history = []  # Store (x, y) positions for lateral deviation calc
        self.planned_path = []  # Store planned waypoints for comparison

        # Smoothing parameters for jerk calculation
        self._jerk_window_size = 5  # Use 5-point moving average for jerk smoothing
        self._raw_jerk_buffer = []  # Buffer for raw jerk values before smoothing
        self.agent_step_list = [
            [], # 0: sim end
            [], # 1: lights
            [], # 2: temp route
            [], # 3: path generation
            [], # 4: lane change
            [], # 5: collision
            [], # 6: no-lane-change composite
            [], # 7: push
            [], # 8: blocking
            [], # 9: overtake
            [], # 10: following
            [], # 11: normal
        ] # index corresponds to specific decision instance in BehaviorAgent.run_step

        self.count = 0

        # Exposure metrics (after spawn buffer)
        self.distance_traveled_m = 0.0
        self.time_alive_s = 0.0

        # Braking attribution — populated by BehaviorAgent.collision_manager
        self.brake_attributions = []

        # Per-tick edge prediction acceptance counters (Confound A diagnostic)
        self.edge_ticks_total = 0          # total ticks through collision_manager Pass 2
        self.edge_ticks_with_preds = 0     # ticks with >= 1 prediction
        self.edge_ticks_with_collision = 0 # ticks where collision was detected
        self.edge_preds_received_total = 0 # sum of predictions received
        self.edge_preds_ego_filtered = 0   # sum of predictions filtered by ego identity
        self.edge_collisions_total = 0     # sum of collision triggers

    def get_agent_step_list(self):
        return self.agent_step_list

    def update(self, ego_speed, ttc, ego_pos=None, planned_waypoint=None):
        """
        Update the speed info and compute derived metrics.
        Args:
            -ego_speed (float): Ego speed in km/h.
            -ttc (float): Time to collision in seconds.
            -ego_pos (tuple, optional): Current (x, y) position for lateral deviation.
            -planned_waypoint (tuple, optional): Target (x, y) on planned path.

        """
        self.count += 1
        # at the very beginning, the vehicle is in a spawn state, so we should
        # filter out the first 100 data points.
        if self.count > 100:
            speed_mps = ego_speed / 3.6
            self.speed_list[0].append(speed_mps)
            # Accumulate exposure metrics
            self.distance_traveled_m += speed_mps * 0.05
            self.time_alive_s += 0.05
            if len(self.speed_list[0]) <= 1:
                self.acc_list[0].append(0)
                self.jerk_list[0].append(0)
            else:
                # todo: time-step hardcoded
                dt = 0.05
                acc = (self.speed_list[0][-1] - self.speed_list[0][-2]) / dt
                self.acc_list[0].append(acc)

                # Compute jerk (rate of change of acceleration) with smoothing
                # Raw numerical differentiation amplifies noise significantly
                # Use moving average filter to get realistic jerk values
                if len(self.acc_list[0]) <= 1:
                    self.jerk_list[0].append(0)
                else:
                    # Compute raw jerk
                    raw_jerk = (self.acc_list[0][-1] - self.acc_list[0][-2]) / dt
                    self._raw_jerk_buffer.append(raw_jerk)

                    # Apply moving average smoothing
                    if len(self._raw_jerk_buffer) >= self._jerk_window_size:
                        # Use last N values for smoothed jerk
                        smoothed_jerk = np.mean(self._raw_jerk_buffer[-self._jerk_window_size:])
                    else:
                        # Not enough samples yet, use average of what we have
                        smoothed_jerk = np.mean(self._raw_jerk_buffer)

                    # Clamp to physically realistic range for vehicles
                    # Normal driving: ~10 m/s³, hard braking: ~50 m/s³, emergency: ~100 m/s³
                    # Values beyond 100 m/s³ are numerical artifacts, not real vehicle behavior
                    smoothed_jerk = np.clip(smoothed_jerk, -100.0, 100.0)
                    self.jerk_list[0].append(smoothed_jerk)

            self.ttc_list[0].append(ttc)

            # Track position and compute lateral deviation
            if ego_pos is not None:
                self.position_history.append(ego_pos)
                if planned_waypoint is not None:
                    self.planned_path.append(planned_waypoint)
                    # Compute lateral deviation as distance to nearest point on path
                    lat_dev = self._compute_lateral_deviation(ego_pos, planned_waypoint)
                    self.lateral_deviation_list[0].append(lat_dev)

    def _compute_lateral_deviation(self, ego_pos, waypoint):
        """
        Compute lateral deviation from the planned path.

        Args:
            ego_pos: Current (x, y) position
            waypoint: Target waypoint (x, y) on planned path

        Returns:
            float: Lateral deviation in meters
        """
        if waypoint is None or ego_pos is None:
            return 0.0

        # Simple Euclidean distance to waypoint
        # A more sophisticated approach would compute the perpendicular
        # distance to the path segment
        dx = ego_pos[0] - waypoint[0]
        dy = ego_pos[1] - waypoint[1]
        return np.sqrt(dx*dx + dy*dy)

    def update_agent_step_list(self, decision_index, time_s=None):
        self.agent_step_list[decision_index].append(time_s*1000)

    def get_metrics(self):
        """
        Get aggregated planning metrics.

        Returns
        -------
        dict
            Dictionary with all planning metrics
        """
        spd_avg = np.nanmean(np.array(self.speed_list[0])) if self.speed_list[0] else 0.0
        spd_std = np.nanstd(np.array(self.speed_list[0])) if self.speed_list[0] else 0.0

        acc_avg = np.nanmean(np.array(self.acc_list[0])) if self.acc_list[0] else 0.0
        acc_std = np.nanstd(np.array(self.acc_list[0])) if self.acc_list[0] else 0.0

        jerk_avg = np.nanmean(np.array(self.jerk_list[0])) if self.jerk_list[0] else 0.0
        jerk_std = np.nanstd(np.array(self.jerk_list[0])) if self.jerk_list[0] else 0.0
        jerk_max = np.nanmax(np.abs(np.array(self.jerk_list[0]))) if self.jerk_list[0] else 0.0

        lat_dev_avg = np.nanmean(np.array(self.lateral_deviation_list[0])) if self.lateral_deviation_list[0] else 0.0
        lat_dev_std = np.nanstd(np.array(self.lateral_deviation_list[0])) if self.lateral_deviation_list[0] else 0.0
        lat_dev_max = np.nanmax(np.array(self.lateral_deviation_list[0])) if self.lateral_deviation_list[0] else 0.0

        ttc_array = np.array(self.ttc_list[0])
        ttc_array = ttc_array[ttc_array < 1000]
        ttc_avg = np.nanmean(ttc_array) if len(ttc_array) > 0 else np.nan
        ttc_std = np.nanstd(ttc_array) if len(ttc_array) > 0 else np.nan

        # Braking attribution
        total_brakes = len(self.brake_attributions)
        ghost_brakes = sum(1 for a in self.brake_attributions if a.get('is_ego_ghost'))
        ghost_by_id = sum(1 for a in self.brake_attributions if a.get('ghost_reason') == 'id_match')
        ghost_by_proximity = sum(1 for a in self.brake_attributions if a.get('ghost_reason') == 'proximity')
        false_brake_rate = ghost_brakes / max(1, total_brakes)

        # GT-labeled brake classification (set by edge manager post-hoc)
        gt_labeled = [a for a in self.brake_attributions
                      if a.get('gt_brake_class') is not None]
        gt_labeled_count = len(gt_labeled)
        ghost_brake_gt_count = sum(1 for a in gt_labeled
                                   if a.get('gt_brake_class') == 'self_ghost')
        other_fp_gt_count = sum(1 for a in gt_labeled
                                if a.get('gt_brake_class') == 'other_fp')
        true_positive_gt_count = sum(1 for a in gt_labeled
                                     if a.get('gt_brake_class') == 'true_positive')

        return {
            "actor_id": self.actor_id,
            "avg_speed_mps": float(spd_avg),
            "std_speed_mps": float(spd_std),
            "avg_acceleration_mps2": float(acc_avg),
            "std_acceleration_mps2": float(acc_std),
            "avg_jerk_mps3": float(jerk_avg),
            "std_jerk_mps3": float(jerk_std),
            "max_jerk_mps3": float(jerk_max),
            "avg_ttc_s": float(ttc_avg) if not np.isnan(ttc_avg) else None,
            "std_ttc_s": float(ttc_std) if not np.isnan(ttc_std) else None,
            "avg_lateral_deviation_m": float(lat_dev_avg),
            "std_lateral_deviation_m": float(lat_dev_std),
            "max_lateral_deviation_m": float(lat_dev_max),
            "total_brake_events": total_brakes,
            "ghost_brake_events": ghost_brakes,
            "ghost_brake_by_id": ghost_by_id,
            "ghost_brake_by_proximity": ghost_by_proximity,
            "false_brake_rate": float(false_brake_rate),
            "ghost_brake_gt": ghost_brake_gt_count,
            "other_fp_gt": other_fp_gt_count,
            "true_positive_gt": true_positive_gt_count,
            "gt_labeled_count": gt_labeled_count,
            "distance_traveled_m": float(self.distance_traveled_m),
            "time_alive_s": float(self.time_alive_s),
            # Edge prediction acceptance (Confound A)
            "edge_ticks_total": self.edge_ticks_total,
            "edge_ticks_with_preds": self.edge_ticks_with_preds,
            "edge_ticks_with_collision": self.edge_ticks_with_collision,
            "edge_preds_received_total": self.edge_preds_received_total,
            "edge_preds_ego_filtered": self.edge_preds_ego_filtered,
            "edge_collisions_total": self.edge_collisions_total,
        }

    def evaluate(self):
        """
        Evaluate the target vehicle and visulize the plot.
        Returns:
            -figure (matplotlib.pyplot.figure): The target vehicle's planning
             profile (velocity, acceleration, jerk, ttc, and lateral deviation).
            -perform_txt (txt file): The target vehicle's planning profile
            as text files.

        """
        warnings.filterwarnings('ignore')

        # Determine subplot layout based on available data
        has_jerk = len(self.jerk_list[0]) > 0
        has_lat_dev = len(self.lateral_deviation_list[0]) > 0
        num_plots = 3 + (1 if has_jerk else 0) + (1 if has_lat_dev else 0)

        # Create figure with appropriate size
        figure = plt.figure(figsize=(10, 2.5 * num_plots))
        plot_idx = 1

        # Speed plot
        plt.subplot(num_plots, 1, plot_idx)
        open_plt.draw_velocity_profile_single_plot(self.speed_list)
        plot_idx += 1

        # Acceleration plot
        plt.subplot(num_plots, 1, plot_idx)
        open_plt.draw_acceleration_profile_single_plot(self.acc_list)
        plot_idx += 1

        # Jerk plot (comfort metric)
        if has_jerk:
            plt.subplot(num_plots, 1, plot_idx)
            jerk_data = np.array(self.jerk_list[0])
            plt.plot(jerk_data, 'g-', linewidth=0.8)
            plt.fill_between(range(len(jerk_data)), jerk_data, alpha=0.3, color='green')
            plt.xlabel('Time Step')
            plt.ylabel('Jerk (m/s³)')
            plt.title('Jerk Profile (Comfort Metric)')
            plt.grid(True, alpha=0.3)
            plot_idx += 1

        # TTC plot
        plt.subplot(num_plots, 1, plot_idx)
        open_plt.draw_ttc_profile_single_plot(self.ttc_list)
        plot_idx += 1

        # Lateral deviation plot
        if has_lat_dev:
            plt.subplot(num_plots, 1, plot_idx)
            lat_dev_data = np.array(self.lateral_deviation_list[0])
            plt.plot(lat_dev_data, 'm-', linewidth=0.8)
            plt.fill_between(range(len(lat_dev_data)), lat_dev_data, alpha=0.3, color='magenta')
            plt.xlabel('Time Step')
            plt.ylabel('Lateral Dev (m)')
            plt.title('Lateral Deviation from Planned Path')
            plt.grid(True, alpha=0.3)
            plot_idx += 1

        figure.suptitle('Planning Profile of Actor ID %d' % self.actor_id)
        plt.tight_layout()

        # Get metrics
        metrics = self.get_metrics()

        perform_txt = 'Speed average: %f (m/s), ' \
                      'Speed std: %f (m/s) \n' % (metrics['avg_speed_mps'], metrics['std_speed_mps'])

        perform_txt += 'Acceleration average: %f (m/s²), ' \
                       'Acceleration std: %f (m/s²) \n' % (metrics['avg_acceleration_mps2'], metrics['std_acceleration_mps2'])

        perform_txt += 'Jerk average: %f (m/s³), ' \
                       'Jerk std: %f (m/s³), Jerk max: %f (m/s³) \n' % (metrics['avg_jerk_mps3'], metrics['std_jerk_mps3'], metrics['max_jerk_mps3'])

        ttc_avg = metrics['avg_ttc_s'] if metrics['avg_ttc_s'] is not None else np.nan
        ttc_std = metrics['std_ttc_s'] if metrics['std_ttc_s'] is not None else np.nan
        perform_txt += 'TTC average: %f (s), ' \
                       'TTC std: %f (s) \n' % (ttc_avg, ttc_std)

        perform_txt += 'Lateral deviation average: %f (m), ' \
                       'Lateral deviation std: %f (m), max: %f (m) \n' % (metrics['avg_lateral_deviation_m'], metrics['std_lateral_deviation_m'], metrics['max_lateral_deviation_m'])

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


# Backward compatibility alias
PlanDebugHelper = PlanningMetrics
