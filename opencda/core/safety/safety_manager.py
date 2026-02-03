"""
The safety manager is used to collect the AV's hazard status and give the
control back to human if necessary
"""
import numpy as np
import carla
import logging

from opencda.core.safety.sensors import CollisionSensor, \
    TrafficLightDector, StuckDetector, OffRoadDetector
# SafetyMetrics functionality is now integrated directly into SafetyManager


class SafetyManager:
    """
    A class that manages the safety of a given vehicle in a simulation environment.

    Parameters
    ----------
    vehicle: carla.Actor
        The vehicle that the SafetyManager is responsible for.
    params: dict
        A dictionary of parameters that are used to configure the SafetyManager.
    """
    def __init__(self, vehicle, params, logger):
        self.vehicle = vehicle
        self.print_message = params['print_message']
        self.sensors = [CollisionSensor(vehicle, params['collision_sensor']),
                        StuckDetector(params['stuck_dector']),
                        OffRoadDetector(params['offroad_dector']),
                        TrafficLightDector(params['traffic_light_detector'],
                                           vehicle)]
        self.status_dict = {}
        self.logger = logger
        # Safety metrics are now provided via get_metrics() and evaluate()

    def update_info(self, data_dict) -> dict:
        self.status_dict = {}
        for sensor in self.sensors:
            sensor.tick(data_dict)
            self.status_dict.update(sensor.return_status())
        if self.print_message:
            print_flag = False
            # only print message when it has hazard
            for key, val in self.status_dict.items():
                if val == True:
                    print_flag = True
                    break
            if print_flag:
                self.logger.info("Safety Warning from the safety manager:")
                self.logger.debug(self.status_dict)


    def get_metrics(self) -> dict:
        """
        Get aggregated safety metrics from all sensors.

        Returns
        -------
        dict
            Dictionary containing:
            - collision_count: Total collisions detected
            - collision_history: List of (frame, intensity) tuples
            - stuck: Whether vehicle is currently stuck
            - offroad: Whether vehicle is currently offroad
            - red_light_violations: Total red lights run
            - total_traffic_lights: Total traffic lights encountered
        """
        metrics = {
            'collision_count': 0,
            'collision_history': [],
            'stuck': False,
            'offroad': False,
            'red_light_violations': 0,
            'total_traffic_lights': 0,
        }

        for sensor in self.sensors:
            if hasattr(sensor, 'collided'):
                # CollisionSensor
                metrics['collision_count'] = 1 if sensor.collided else 0
                metrics['collision_history'] = list(sensor._history)
            elif hasattr(sensor, 'stuck'):
                # StuckDetector
                metrics['stuck'] = sensor.stuck
            elif hasattr(sensor, 'off_road'):
                # OffRoadDetector
                metrics['offroad'] = sensor.off_road
            elif hasattr(sensor, 'total_lights_ran'):
                # TrafficLightDetector
                metrics['red_light_violations'] = sensor.total_lights_ran
                metrics['total_traffic_lights'] = sensor.total_lights

        return metrics

    def evaluate(self):
        """
        Evaluate safety metrics and return figure, text summary, and metrics dict.

        Returns
        -------
        tuple
            (matplotlib.figure.Figure, str, dict)
        """
        import matplotlib.pyplot as plt
        import warnings
        warnings.filterwarnings('ignore')

        metrics = self.get_metrics()

        # Create figure
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))

        # 1. Collision history
        if metrics['collision_history']:
            frames, intensities = zip(*metrics['collision_history'])
            axes[0, 0].bar(frames, intensities, color='red', alpha=0.7)
            axes[0, 0].set_xlabel('Frame')
            axes[0, 0].set_ylabel('Collision Intensity')
            axes[0, 0].set_title(f"Collisions: {metrics['collision_count']}")
        else:
            axes[0, 0].text(0.5, 0.5, 'No Collisions', ha='center', va='center', fontsize=14)
            axes[0, 0].set_title('Collision History')

        # 2. Status summary as text
        status_text = f"""
Safety Status Summary
---------------------
Collisions: {metrics['collision_count']}
Stuck: {metrics['stuck']}
Offroad: {metrics['offroad']}
Red Lights Run: {metrics['red_light_violations']}
Traffic Lights Seen: {metrics['total_traffic_lights']}
"""
        axes[0, 1].text(0.1, 0.5, status_text, fontsize=12, family='monospace',
                        verticalalignment='center')
        axes[0, 1].axis('off')
        axes[0, 1].set_title('Status Summary')

        # 3. Safety score calculation (penalty-based like leaderboard)
        score_penalty = 1.0
        PENALTY_COLLISION = 0.60
        PENALTY_RED_LIGHT = 0.70

        score_penalty *= (PENALTY_COLLISION ** metrics['collision_count'])
        score_penalty *= (PENALTY_RED_LIGHT ** metrics['red_light_violations'])

        if metrics['stuck']:
            score_penalty *= 0.5
        if metrics['offroad']:
            score_penalty *= 0.9

        metrics['score_penalty'] = score_penalty

        # Pie chart of penalty breakdown
        labels = ['Safe Driving', 'Collision Penalty', 'Red Light Penalty', 'Other']
        collision_penalty_pct = (1 - PENALTY_COLLISION ** metrics['collision_count']) * 100
        red_light_penalty_pct = (1 - PENALTY_RED_LIGHT ** metrics['red_light_violations']) * 100
        other_penalty_pct = max(0, (1 - score_penalty) * 100 - collision_penalty_pct - red_light_penalty_pct)
        safe_pct = max(0, 100 - collision_penalty_pct - red_light_penalty_pct - other_penalty_pct)

        sizes = [safe_pct, collision_penalty_pct, red_light_penalty_pct, other_penalty_pct]
        colors = ['green', 'red', 'orange', 'gray']
        axes[1, 0].pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        axes[1, 0].set_title(f'Safety Score: {score_penalty:.1%}')

        # 4. Empty or future use
        axes[1, 1].axis('off')

        fig.suptitle(f'Safety Evaluation - Vehicle {self.vehicle.id}')
        fig.tight_layout()

        # Build text summary
        perform_txt = f"""
Safety Metrics for Vehicle {self.vehicle.id}:
  Collisions: {metrics['collision_count']}
  Stuck: {metrics['stuck']}
  Offroad: {metrics['offroad']}
  Red Light Violations: {metrics['red_light_violations']}
  Traffic Lights Encountered: {metrics['total_traffic_lights']}
  Safety Score Penalty: {score_penalty:.3f}
"""

        return fig, perform_txt, metrics

    def destroy(self):
        for sensor in self.sensors:
            sensor.destroy()
