# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Route XML Parser for eCAV multi-waypoint route definitions.

This module provides utilities to parse route XML files that define
multi-waypoint routes for vehicle navigation, integrating with eCAV's
YAML configuration system.

Route XML Format:
    <?xml version='1.0' encoding='utf-8'?>
    <routes>
      <route id="1" town="Town03">
        <waypoint x="-84.8" y="100.0" z="0.3" yaw="90" />
        <waypoint x="-78.0" y="128.0" z="0.3" yaw="0" />
        <waypoint x="-28.8" y="134.7" z="0.3" yaw="0" />
      </route>
    </routes>

Usage in YAML config:
    vehicles:
      - name: cav1
        route_file: "routes/scenario_3_route.xml"
        route_id: 1
        # OR use inline waypoints:
        route:
          - [-84.8, 100.0, 0.3, 90]   # [x, y, z, yaw]
          - [-78.0, 128.0, 0.3, 0]
          - [-28.8, 134.7, 0.3, 0]
"""

import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple, Any
from enum import Enum
import os
import logging

logger = logging.getLogger(__name__)


class RoadOption(Enum):
    """
    Road options for waypoint connections.
    Compatible with CARLA's agents.navigation.local_planner.RoadOption
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


@dataclass
class Waypoint:
    """Represents a single waypoint in a route."""
    x: float
    y: float
    z: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    road_option: RoadOption = RoadOption.LANEFOLLOW

    def to_tuple(self) -> Tuple[float, float, float, float]:
        """Return as (x, y, z, yaw) tuple for compatibility."""
        return (self.x, self.y, self.z, self.yaw)

    def to_dict(self) -> Dict[str, float]:
        """Return as dictionary."""
        return {
            'x': self.x,
            'y': self.y,
            'z': self.z,
            'pitch': self.pitch,
            'roll': self.roll,
            'yaw': self.yaw
        }


@dataclass
class Route:
    """Represents a complete route with multiple waypoints."""
    id: str
    town: str
    waypoints: List[Waypoint] = field(default_factory=list)

    @property
    def start(self) -> Optional[Waypoint]:
        """Get the starting waypoint."""
        return self.waypoints[0] if self.waypoints else None

    @property
    def destination(self) -> Optional[Waypoint]:
        """Get the destination (final) waypoint."""
        return self.waypoints[-1] if self.waypoints else None

    def get_waypoint_list(self) -> List[Tuple[float, float, float, float]]:
        """Return list of (x, y, z, yaw) tuples."""
        return [wp.to_tuple() for wp in self.waypoints]


class RouteParser:
    """
    Parser for eCAV route XML files.

    Supports both XML file parsing and inline route definitions from YAML.
    """

    @staticmethod
    def parse_routes_file(route_filename: str, single_route: Optional[str] = None) -> List[Route]:
        """
        Parse a route XML file and return list of Route objects.

        Parameters
        ----------
        route_filename : str
            Path to the route XML file.
        single_route : str, optional
            If specified, only parse the route with this ID.

        Returns
        -------
        List[Route]
            List of parsed Route objects.
        """
        if not os.path.exists(route_filename):
            raise FileNotFoundError(f"Route file not found: {route_filename}")

        routes = []
        tree = ET.parse(route_filename)
        root = tree.getroot()

        for route_elem in root.iter('route'):
            route_id = route_elem.get('id', '0')

            # Skip if we're looking for a specific route
            if single_route is not None and route_id != single_route:
                continue

            town = route_elem.get('town', 'Town03')
            waypoints = []

            for wp_elem in route_elem.iter('waypoint'):
                waypoint = Waypoint(
                    x=float(wp_elem.get('x', 0)),
                    y=float(wp_elem.get('y', 0)),
                    z=float(wp_elem.get('z', 0)),
                    pitch=float(wp_elem.get('pitch', 0)),
                    roll=float(wp_elem.get('roll', 0)),
                    yaw=float(wp_elem.get('yaw', 0))
                )

                # Parse road option if present
                road_option_str = wp_elem.get('connection', 'LANEFOLLOW')
                try:
                    waypoint.road_option = RoadOption[road_option_str.upper()]
                except KeyError:
                    waypoint.road_option = RoadOption.LANEFOLLOW

                waypoints.append(waypoint)

            route = Route(id=route_id, town=town, waypoints=waypoints)
            routes.append(route)

            logger.info(f"[RouteParser] Parsed route {route_id} with {len(waypoints)} waypoints")

        return routes

    @staticmethod
    def parse_inline_route(route_config: List[List[float]], route_id: str = "inline") -> Route:
        """
        Parse an inline route definition from YAML config.

        Parameters
        ----------
        route_config : List[List[float]]
            List of waypoint coordinates, each as [x, y, z, yaw] or [x, y, z].
        route_id : str
            Identifier for the route.

        Returns
        -------
        Route
            Parsed Route object.
        """
        waypoints = []

        for wp_data in route_config:
            if len(wp_data) >= 3:
                waypoint = Waypoint(
                    x=float(wp_data[0]),
                    y=float(wp_data[1]),
                    z=float(wp_data[2]),
                    yaw=float(wp_data[3]) if len(wp_data) > 3 else 0.0
                )
                waypoints.append(waypoint)

        return Route(id=route_id, town="", waypoints=waypoints)

    @staticmethod
    def get_route_from_config(vehicle_config: Dict[str, Any], base_path: str = "") -> Optional[Route]:
        """
        Extract route from vehicle configuration.

        Supports three formats:
        1. route_file + route_id: Load from XML file
        2. route: Inline list of waypoints
        3. destination: Single destination point (legacy)

        Parameters
        ----------
        vehicle_config : Dict
            Vehicle configuration dictionary from YAML.
        base_path : str
            Base path for resolving relative route file paths.

        Returns
        -------
        Optional[Route]
            Parsed Route object, or None if no route specified.
        """
        # Check for route file reference
        if 'route_file' in vehicle_config:
            route_file = vehicle_config['route_file']
            if not os.path.isabs(route_file):
                route_file = os.path.join(base_path, route_file)

            route_id = str(vehicle_config.get('route_id', '0'))
            routes = RouteParser.parse_routes_file(route_file, single_route=route_id)

            if routes:
                return routes[0]
            else:
                logger.warning(f"[RouteParser] Route {route_id} not found in {route_file}")
                return None

        # Check for inline route
        if 'route' in vehicle_config and vehicle_config['route']:
            return RouteParser.parse_inline_route(vehicle_config['route'])

        # Check for legacy destination format
        if 'destination' in vehicle_config:
            dest = vehicle_config['destination']
            waypoint = Waypoint(
                x=float(dest[0]),
                y=float(dest[1]),
                z=float(dest[2]) if len(dest) > 2 else 0.0
            )
            return Route(id="destination", town="", waypoints=[waypoint])

        return None


def create_sample_route_xml(output_path: str, routes: List[Dict]) -> None:
    """
    Create a sample route XML file.

    Parameters
    ----------
    output_path : str
        Path to write the XML file.
    routes : List[Dict]
        List of route definitions, each with 'id', 'town', and 'waypoints'.

    Example
    -------
    >>> create_sample_route_xml("routes/test.xml", [
    ...     {
    ...         'id': '1',
    ...         'town': 'Town03',
    ...         'waypoints': [
    ...             {'x': -84.8, 'y': 100.0, 'z': 0.3, 'yaw': 90},
    ...             {'x': -78.0, 'y': 128.0, 'z': 0.3, 'yaw': 0},
    ...             {'x': -28.8, 'y': 134.7, 'z': 0.3, 'yaw': 0}
    ...         ]
    ...     }
    ... ])
    """
    root = ET.Element('routes')

    for route_def in routes:
        route_elem = ET.SubElement(root, 'route')
        route_elem.set('id', str(route_def.get('id', '0')))
        route_elem.set('town', route_def.get('town', 'Town03'))

        for wp in route_def.get('waypoints', []):
            wp_elem = ET.SubElement(route_elem, 'waypoint')
            wp_elem.set('x', str(wp.get('x', 0)))
            wp_elem.set('y', str(wp.get('y', 0)))
            wp_elem.set('z', str(wp.get('z', 0)))
            wp_elem.set('pitch', str(wp.get('pitch', 0)))
            wp_elem.set('roll', str(wp.get('roll', 0)))
            wp_elem.set('yaw', str(wp.get('yaw', 0)))

            if 'connection' in wp:
                wp_elem.set('connection', wp['connection'])

    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    tree.write(output_path, encoding='utf-8', xml_declaration=True)

    logger.info(f"[RouteParser] Created route XML: {output_path}")


if __name__ == "__main__":
    # Test the parser
    import sys

    if len(sys.argv) > 1:
        route_file = sys.argv[1]
        routes = RouteParser.parse_routes_file(route_file)
        for route in routes:
            print(f"Route {route.id} ({route.town}): {len(route.waypoints)} waypoints")
            for i, wp in enumerate(route.waypoints):
                print(f"  [{i}] x={wp.x:.1f}, y={wp.y:.1f}, z={wp.z:.1f}, yaw={wp.yaw:.1f}")
    else:
        # Create sample route file
        sample_routes = [
            {
                'id': '1',
                'town': 'Town03',
                'waypoints': [
                    {'x': -84.8, 'y': 100.0, 'z': 0.3, 'yaw': 90},
                    {'x': -78.0, 'y': 128.0, 'z': 0.3, 'yaw': 0},
                    {'x': -28.8, 'y': 134.7, 'z': 0.3, 'yaw': 0}
                ]
            }
        ]
        create_sample_route_xml("/tmp/sample_route.xml", sample_routes)
        print("Created sample route file: /tmp/sample_route.xml")

        # Parse it back
        routes = RouteParser.parse_routes_file("/tmp/sample_route.xml")
        print(f"Parsed {len(routes)} routes")
