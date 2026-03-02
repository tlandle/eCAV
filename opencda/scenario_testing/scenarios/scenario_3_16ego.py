#!/usr/bin/env python
"""
Scenario 3 with 16 ego vehicles — visualization metadata.
Used by show_spawn_points.py for path visualization.
"""

import carla

class Scenario_3_16Ego_Viz:
    """Metadata for show_spawn_points.py."""

    def __init__(self):
        self.vehicle_01_velocity = 70  # cav2: West approach (eastbound)
        self.vehicle_02_velocity = 70  # cav3: East approach (westbound)
        self.vehicle_03_velocity = 70  # cav4: North approach (southbound)
        self.vehicle_04_velocity = 70  # cav5: South approach (northbound)
        self.vehicle_05_velocity = 70  # cav6: West approach (eastbound)
        self.vehicle_06_velocity = 70  # cav7: East approach (westbound)
        self.vehicle_07_velocity = 70  # cav8: North approach (southbound)
        self.vehicle_08_velocity = 70  # cav9: South approach (northbound)
        self.vehicle_09_velocity = 70  # cav10: West approach (eastbound)
        self.vehicle_10_velocity = 70  # cav11: East approach (westbound)
        self.vehicle_11_velocity = 70  # cav12: North approach (southbound)
        self.vehicle_12_velocity = 70  # cav13: South approach (northbound)
        self.vehicle_13_velocity = 70  # cav14: West approach (eastbound)
        self.vehicle_14_velocity = 70  # cav15: East approach (westbound)
        self.vehicle_15_velocity = 70  # cav16: North approach (southbound)
        self.vehicle_16_velocity = 25  # Lincoln NPC
        self.vehicle_17_velocity = 0  # Occlusion vehicle
        self.vehicle_18_velocity = 0  # Occlusion vehicle
        self.vehicle_19_velocity = 0  # Occlusion vehicle
        self.vehicle_20_velocity = 0  # Occlusion vehicle
        self.vehicle_21_velocity = 0  # Occlusion vehicle

        # Lincoln waypoint plan
        if i == 15:
            waypoint = [carla.Location(x=-108.6, y=129.5, z=0.5), carla.Location(x=-120.6, y=129.5, z=0.5), carla.Location(x=-140.6, y=115.2, z=0.5), carla.Location(x=-142.0, y=87.6, z=0.5)]
