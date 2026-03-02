#!/usr/bin/env python
"""
Scenario 3 with 4 ego vehicles — visualization metadata.
Used by show_spawn_points.py for path visualization.
"""

import carla

class Scenario_3_4Ego_Viz:
    """Metadata for show_spawn_points.py."""

    def __init__(self):
        self.vehicle_01_velocity = 70  # cav2: West approach (eastbound)
        self.vehicle_02_velocity = 70  # cav3: East approach (westbound)
        self.vehicle_03_velocity = 70  # cav4: North approach (southbound)
        self.vehicle_04_velocity = 25  # Lincoln NPC
        self.vehicle_05_velocity = 0  # Occlusion vehicle
        self.vehicle_06_velocity = 0  # Occlusion vehicle
        self.vehicle_07_velocity = 0  # Occlusion vehicle
        self.vehicle_08_velocity = 0  # Occlusion vehicle
        self.vehicle_09_velocity = 0  # Occlusion vehicle

        # Lincoln waypoint plan
        if i == 3:
            waypoint = [carla.Location(x=-108.6, y=129.5, z=0.5), carla.Location(x=-120.6, y=129.5, z=0.5), carla.Location(x=-140.6, y=115.2, z=0.5), carla.Location(x=-142.0, y=87.6, z=0.5)]
