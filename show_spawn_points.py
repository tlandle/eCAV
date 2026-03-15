#!/usr/bin/env python3
"""
show_spawn_points.py
--------------------
Parse a scenario XML and its .py behavior file, then draw labelled markers
and paths in the CARLA spectator view for every vehicle.

- Ego gets a road-network trace (follows CARLA waypoints from spawn).
- Actors with waypoints defined in the .py file get those paths drawn.
- Stationary actors (velocity=0 in .py) get just a spawn marker.

Usage:
    python show_spawn_points.py scenario_3
    python show_spawn_points.py scenario_3 --trace-distance 120
    python show_spawn_points.py --xml path/to/custom.xml
"""

import argparse
import math
import re
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

# ── paths ────────────────────────────────────────────────────────────────
ROOT = Path(__file__).resolve().parent
SCENARIO_DIR = ROOT / "opencda/scenario_testing/scenarios"

# ── colours (one per vehicle index) ──────────────────────────────────────
VEHICLE_COLOURS = [
    carla.Color(0, 255, 0),       # 0  ego       green
    carla.Color(255, 60, 60),     # 1  actor 1   red
    carla.Color(255, 165, 0),     # 2  actor 2   orange
    carla.Color(60, 160, 255),    # 3  actor 3   blue
    carla.Color(200, 60, 255),    # 4  actor 4   purple
    carla.Color(255, 255, 60),    # 5  actor 5   yellow
    carla.Color(60, 255, 255),    # 6  actor 6   cyan
    carla.Color(255, 120, 180),   # 7  actor 7   pink
]

MARKER_LIFE = 180.0
TRACE_STEP = 2.0

LOC_RE = re.compile(
    r"carla\.Location\(\s*x\s*=\s*([+-]?[\d.]+)\s*,\s*"
    r"y\s*=\s*([+-]?[\d.]+)\s*,\s*"
    r"z\s*=\s*([+-]?[\d.]+)\s*\)"
)


def get_vehicle_color(index):
    return VEHICLE_COLOURS[index % len(VEHICLE_COLOURS)]


# ── XML parsing ──────────────────────────────────────────────────────────

def parse_scenario_xml(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()
    scenario = root.find("scenario")
    town = scenario.attrib.get("town", "Town03")

    ego_el = scenario.find("ego_vehicle")
    ego = {
        "label": "EGO",
        "model": ego_el.attrib.get("model", ""),
        "x": float(ego_el.attrib["x"]),
        "y": float(ego_el.attrib["y"]),
        "z": float(ego_el.attrib["z"]),
        "yaw": float(ego_el.attrib.get("yaw", 0)),
    }

    actors = []
    for i, el in enumerate(scenario.findall("other_actor")):
        actors.append({
            "label": f"ACTOR_{i+1}",
            "model": el.attrib.get("model", ""),
            "x": float(el.attrib["x"]),
            "y": float(el.attrib["y"]),
            "z": float(el.attrib["z"]),
            "yaw": float(el.attrib.get("yaw", 0)),
        })

    return town, ego, actors


# ── Python scenario file parsing ─────────────────────────────────────────

def parse_scenario_py(py_path):
    """Extract per-actor velocities and waypoint plans from scenario .py.

    Looks for patterns like:
        self.vehicle_01_velocity = 10
        waypoint = [carla.Location(x=..., y=..., z=...), ...]
        drive_behavior = WaypointFollower(actor, velocity, plan=waypoint)

    Returns:
        velocities: dict  actor_index(0-based) → speed (number)
        waypoint_plans: list of lists of (x, y, z) — one per actor that
                        has an explicit plan.  The list index corresponds
                        to the actor whose WaypointFollower has plan=.
    """
    text = py_path.read_text()

    # ── velocities ──
    # Match: self.vehicle_01_velocity = 10
    vel_re = re.compile(r"self\.vehicle_(\d+)_velocity\s*=\s*([+-]?\d+\.?\d*)")
    velocities = {}
    for m in vel_re.finditer(text):
        idx = int(m.group(1)) - 1  # vehicle_01 → actor index 0
        velocities[idx] = float(m.group(2))

    # ── waypoint plans ──
    # Find lines with: waypoint = [carla.Location(...), ...]
    # Then find the next WaypointFollower that uses plan=
    wp_plan_re = re.compile(
        r"waypoint\s*=\s*\[(.*?)\]",
        re.DOTALL,
    )
    plans = []
    for m in wp_plan_re.finditer(text):
        locs = LOC_RE.findall(m.group(1))
        if locs:
            plans.append([(float(x), float(y), float(z)) for x, y, z in locs])

    # Figure out which actor index each plan belongs to by looking at
    # the surrounding code context.  In scenario_3.py the pattern is:
    #   if i == 0:  ← actor index 0
    #       waypoint = [...]
    #       drive_behavior = WaypointFollower(actor, velocity, plan=waypoint)
    #
    # We parse the `if i == N:` blocks to map plans to actor indices.
    actor_plans = {}
    # Find all `if i == N:` ... `waypoint = [...]` associations
    block_re = re.compile(
        r"if\s+i\s*==\s*(\d+)\s*:.*?waypoint\s*=\s*\[(.*?)\]",
        re.DOTALL,
    )
    for m in block_re.finditer(text):
        idx = int(m.group(1))
        locs = LOC_RE.findall(m.group(2))
        if locs:
            actor_plans[idx] = [(float(x), float(y), float(z)) for x, y, z in locs]

    # If no block-pattern matches but we found plans, assign plan 0 to actor 0
    if not actor_plans and plans:
        actor_plans[0] = plans[0]

    return velocities, actor_plans


# ── drawing helpers ──────────────────────────────────────────────────────

def draw_heading_arrow(debug, loc, yaw_deg, length=5.0, color=None):
    yaw_rad = math.radians(yaw_deg)
    dx = math.cos(yaw_rad) * length
    dy = math.sin(yaw_rad) * length
    end = carla.Location(x=loc.x + dx, y=loc.y + dy, z=loc.z)
    debug.draw_arrow(loc, end, thickness=0.25, arrow_size=0.6,
                     color=color, life_time=MARKER_LIFE)


def draw_vehicle_marker(debug, info, color):
    underground = info["z"] < -100
    z = 0.5 if underground else max(info["z"], 0.5)
    loc = carla.Location(x=info["x"], y=info["y"], z=z)

    top = carla.Location(x=info["x"], y=info["y"], z=z + 8.0)
    debug.draw_line(loc, top, thickness=0.08, color=color, life_time=MARKER_LIFE)
    debug.draw_point(loc, size=0.25, color=color, life_time=MARKER_LIFE)

    model_short = info.get("model", "").split(".")[-1] if info.get("model") else ""
    label = f"{info['label']}  ({info['x']:.1f}, {info['y']:.1f})"
    if model_short:
        label += f"  [{model_short}]"
    debug.draw_string(
        carla.Location(x=info["x"], y=info["y"], z=z + 9.0),
        label, draw_shadow=True, color=color, life_time=MARKER_LIFE,
    )
    draw_heading_arrow(debug, loc, info["yaw"], length=4.0, color=color)


def trace_road_path(carla_map, start_x, start_y, start_z, yaw_deg, distance,
                    debug, color, step=TRACE_STEP, arrow_interval=10.0):
    """Trace road from spawn using CARLA waypoints. Picks straight at junctions.

    Draws a thick continuous line with large direction arrows every
    `arrow_interval` metres.
    """
    z = max(start_z, 0.3)
    start_loc = carla.Location(x=start_x, y=start_y, z=z)
    wp = carla_map.get_waypoint(start_loc, project_to_road=True,
                                lane_type=carla.LaneType.Driving)
    if wp is None:
        return []

    points = []
    prev_loc = None
    travelled = 0.0
    since_arrow = 0.0

    while travelled < distance:
        loc = wp.transform.location
        draw_loc = carla.Location(x=loc.x, y=loc.y, z=loc.z + 0.3)
        points.append((loc.x, loc.y))

        debug.draw_point(draw_loc, size=0.1, color=color, life_time=MARKER_LIFE)

        if prev_loc is not None:
            # Thick connecting line
            debug.draw_line(prev_loc, draw_loc, thickness=0.12, color=color,
                            life_time=MARKER_LIFE)

            # Large direction arrow at intervals
            since_arrow += step
            if since_arrow >= arrow_interval:
                debug.draw_arrow(prev_loc, draw_loc, thickness=0.2,
                                 arrow_size=0.6, color=color, life_time=MARKER_LIFE)
                since_arrow = 0.0

        prev_loc = draw_loc

        next_wps = wp.next(step)
        if not next_wps:
            break
        if len(next_wps) == 1:
            wp = next_wps[0]
        else:
            cur_yaw = wp.transform.rotation.yaw
            wp = min(next_wps, key=lambda c: abs(((c.transform.rotation.yaw - cur_yaw + 180) % 360) - 180))
        travelled += step

    return points


def draw_waypoint_plan(debug, spawn, plan_points, color, dot_spacing=3.0):
    """Draw spawn → wp1 → wp2 → ... as connected lines with dots.

    spawn: vehicle dict with x, y, z
    plan_points: list of (x, y, z) tuples from the .py file
    dot_spacing: metres between interpolated dots along each segment
    Returns (x,y) list for bounding box.
    """
    spawn_z = 0.5 if spawn["z"] < -100 else max(spawn["z"], 0.5)

    # Build full path: spawn position → each waypoint
    full_path = [(spawn["x"], spawn["y"], spawn_z)] + \
                [(x, y, max(z, 0.5)) for x, y, z in plan_points]

    points = []
    for j, (x, y, z) in enumerate(full_path):
        loc = carla.Location(x=x, y=y, z=z)
        points.append((x, y))

        if j > 0:  # skip spawn (already has a marker)
            debug.draw_point(loc, size=0.2, color=color, life_time=MARKER_LIFE)
            label = f"WP[{j}] ({x:.1f}, {y:.1f})"
            debug.draw_string(
                carla.Location(x=x, y=y, z=z + 2.5),
                label, draw_shadow=True, color=color, life_time=MARKER_LIFE,
            )

        # Connect to previous with a thick line + interpolated dots
        if j > 0:
            px, py, pz = full_path[j - 1]
            prev_loc = carla.Location(x=px, y=py, z=pz)

            # Thick connecting line
            debug.draw_line(prev_loc, loc, thickness=0.15, color=color,
                            life_time=MARKER_LIFE)

            # Large direction arrow at the midpoint
            mx = (px + x) / 2
            my = (py + y) / 2
            mz = (pz + z) / 2
            mid = carla.Location(x=mx, y=my, z=mz)
            debug.draw_arrow(mid, loc, thickness=0.2, arrow_size=0.6,
                             color=color, life_time=MARKER_LIFE)

            # Interpolated dots along the segment
            seg_len = math.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
            n_dots = max(1, int(seg_len / dot_spacing))
            for d in range(1, n_dots):
                t = d / n_dots
                dx = px + t * (x - px)
                dy = py + t * (y - py)
                dz = pz + t * (z - pz)
                debug.draw_point(
                    carla.Location(x=dx, y=dy, z=dz),
                    size=0.06, color=color, life_time=MARKER_LIFE,
                )

    return points


# ── main ─────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Draw scenario spawn points and vehicle routes in CARLA")
    ap.add_argument("scenario", nargs="?", default=None,
                    help="Scenario name, e.g. scenario_3 (looks in standard dirs)")
    ap.add_argument("--xml", default=None,
                    help="Path to scenario XML (overrides positional arg)")
    ap.add_argument("--trace-distance", type=float, default=80.0,
                    help="Distance (m) to trace ego's road path (default: 80)")
    ap.add_argument("--fly-to", default="overview", choices=["ego", "overview"],
                    help="Move spectator camera")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()

    # ── resolve paths ────────────────────────────────────────────────────
    if args.xml:
        scenario_xml = Path(args.xml)
    elif args.scenario:
        scenario_xml = SCENARIO_DIR / f"{args.scenario}.xml"
    else:
        ap.error("Provide a scenario name or --xml path")

    if not scenario_xml.exists():
        print(f"ERROR: {scenario_xml} not found", file=sys.stderr)
        sys.exit(1)

    # Auto-detect .py file (same directory, same stem)
    scenario_py = scenario_xml.with_suffix(".py")

    # ── parse ────────────────────────────────────────────────────────────
    town, ego, actors = parse_scenario_xml(scenario_xml)
    vehicles = [ego] + actors

    # Parse .py for velocities and waypoint plans
    velocities = {}
    actor_plans = {}
    if scenario_py.exists():
        velocities, actor_plans = parse_scenario_py(scenario_py)
        print(f"Parsed:   {scenario_py.name}")
        if velocities:
            print(f"  Velocities: {velocities}")
        if actor_plans:
            print(f"  Waypoint plans for actors: {list(actor_plans.keys())}")
    else:
        print(f"WARNING: {scenario_py.name} not found, no actor paths available")

    print(f"\nScenario: {scenario_xml.name}")
    print(f"Town:     {town}")
    print(f"Ego trace: {args.trace_distance}m")
    print()
    for i, v in enumerate(vehicles):
        if i == 0:
            role = "ego (road trace)"
        else:
            actor_idx = i - 1  # actors are 0-indexed in velocities
            vel = velocities.get(actor_idx)
            has_plan = actor_idx in actor_plans
            if vel is not None and vel == 0:
                role = "stationary"
            elif has_plan:
                role = f"moving (v={vel} km/h, {len(actor_plans[actor_idx])} waypoints)"
            elif vel is not None and vel > 0:
                role = f"moving (v={vel} km/h, follows lane)"
            else:
                role = "unknown"
        print(f"  [{i}] {v['label']:10s}  ({v['x']:7.1f}, {v['y']:7.1f})  "
              f"yaw={v['yaw']:4.0f}  {role}")
    print()

    # ── connect to CARLA ─────────────────────────────────────────────────
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    current_map = client.get_world().get_map().name
    if town.lower() not in current_map.lower():
        print(f"Loading {town}...")
        client.load_world(town)
        time.sleep(3.0)

    world = client.get_world()
    debug = world.debug
    carla_map = world.get_map()

    # ── draw ─────────────────────────────────────────────────────────────
    all_pts = []

    for i, v in enumerate(vehicles):
        color = get_vehicle_color(i)
        draw_vehicle_marker(debug, v, color)
        all_pts.append((v["x"], v["y"]))

        if i == 0:
            # Ego: road-network trace
            spawn_z = 0.5 if v["z"] < -100 else v["z"]
            pts = trace_road_path(
                carla_map, v["x"], v["y"], spawn_z, v["yaw"],
                args.trace_distance, debug, color,
            )
            all_pts.extend(pts)
            print(f"  {v['label']}: road trace ({len(pts)} pts)")

        else:
            actor_idx = i - 1
            vel = velocities.get(actor_idx)

            if actor_idx in actor_plans:
                # Actor has explicit waypoint plan from .py
                pts = draw_waypoint_plan(debug, v, actor_plans[actor_idx], color)
                all_pts.extend(pts)
                print(f"  {v['label']}: waypoint plan ({len(actor_plans[actor_idx])} waypoints)")

            elif vel is not None and vel == 0:
                # Stationary actor: just the marker
                print(f"  {v['label']}: stationary")

            else:
                # Moving actor without explicit plan: trace road
                spawn_z = 0.5 if v["z"] < -100 else v["z"]
                pts = trace_road_path(
                    carla_map, v["x"], v["y"], spawn_z, v["yaw"],
                    args.trace_distance, debug, color,
                )
                all_pts.extend(pts)
                print(f"  {v['label']}: road trace ({len(pts)} pts)")

    # ── move spectator ───────────────────────────────────────────────────
    spectator = world.get_spectator()

    if args.fly_to == "ego":
        spectator.set_transform(carla.Transform(
            carla.Location(x=ego["x"], y=ego["y"], z=60),
            carla.Rotation(pitch=-90),
        ))
    else:
        all_x = [p[0] for p in all_pts]
        all_y = [p[1] for p in all_pts]
        cx = (min(all_x) + max(all_x)) / 2
        cy = (min(all_y) + max(all_y)) / 2
        span = max(max(all_x) - min(all_x), max(all_y) - min(all_y), 40)
        spectator.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=span * 0.8 + 20),
            carla.Rotation(pitch=-90),
        ))

    # ── legend ───────────────────────────────────────────────────────────
    print()
    print("Legend:")
    for i, v in enumerate(vehicles):
        c = get_vehicle_color(i)
        print(f"  {v['label']:10s}  rgb({c.r},{c.g},{c.b})")
    print(f"\nMarkers visible for {MARKER_LIFE:.0f}s. Use WASD in CARLA to navigate.")


if __name__ == "__main__":
    main()
