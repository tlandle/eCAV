#!/usr/bin/env python3
"""
query_spawn_lanes.py — Query CARLA Town03 lane layout for 8-ego spawn positions.

Run with CARLA server active:
    python query_spawn_lanes.py

Prints lane info at each candidate spawn position and discovers all
valid driving lanes on the N-S road and E-W road near the intersection.
"""
import carla
import math

def lane_info(mp, x, y, z=1.0, label=""):
    loc = carla.Location(x=x, y=y, z=z)
    wp = mp.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp:
        t = wp.transform
        road_id = wp.road_id
        lane_id = wp.lane_id
        section = wp.section_id
        proj_dist = loc.distance(t.location)
        yaw = t.rotation.yaw
        print(f"  {label:30s} query=({x:7.1f},{y:7.1f}) -> road={road_id:3d} "
              f"sec={section} lane={lane_id:+2d} "
              f"pos=({t.location.x:7.1f},{t.location.y:7.1f}) "
              f"yaw={yaw:6.1f}  proj_dist={proj_dist:.2f}m")

        # Also show adjacent lanes
        left = wp.get_left_lane()
        right = wp.get_right_lane()
        if left and left.lane_type == carla.LaneType.Driving:
            lt = left.transform
            print(f"    LEFT  lane: road={left.road_id} lane={left.lane_id:+2d} "
                  f"pos=({lt.location.x:7.1f},{lt.location.y:7.1f}) yaw={lt.rotation.yaw:6.1f}")
        if right and right.lane_type == carla.LaneType.Driving:
            rt = right.transform
            print(f"    RIGHT lane: road={right.road_id} lane={right.lane_id:+2d} "
                  f"pos=({rt.location.x:7.1f},{rt.location.y:7.1f}) yaw={rt.rotation.yaw:6.1f}")
        return wp
    else:
        print(f"  {label:30s} query=({x:7.1f},{y:7.1f}) -> NO WAYPOINT")
        return None


def trace_lanes_along(mp, start_x, start_y, direction_yaw, distance=50, step=10, label=""):
    """Follow a waypoint along a road to show lane layout."""
    loc = carla.Location(x=start_x, y=start_y, z=1.0)
    wp = mp.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    if not wp:
        print(f"  Cannot trace from ({start_x},{start_y})")
        return

    print(f"\n  Lane trace from ({start_x},{start_y}) heading ~{direction_yaw}°, step={step}m:")
    visited = set()
    current = wp
    total = 0
    while total <= distance:
        key = (current.road_id, current.lane_id, round(current.s, 1))
        if key in visited:
            break
        visited.add(key)
        t = current.transform
        print(f"    s={total:5.0f}m  road={current.road_id:3d} lane={current.lane_id:+2d} "
              f"pos=({t.location.x:7.1f},{t.location.y:7.1f}) yaw={t.rotation.yaw:6.1f}")
        nexts = current.next(step)
        if not nexts:
            break
        current = nexts[0]
        total += step


def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    mp = world.get_map()
    print(f"Map: {mp.name}\n")

    # Current 8-ego spawn positions
    print("=== Current 8-ego spawn positions ===")
    spawns = [
        (-84.8,  80.0, "cav1 hero NB (ScenarioRunner)"),
        (-81.0,  80.0, "cav5 NB inner (STUCK)"),
        (-130.0, 134.7, "cav2 EB outer"),
        (-130.0, 131.2, "cav6 EB inner"),
        (-15.0,  127.7, "cav3 WB outer"),
        (-15.0,  131.2, "cav7 WB inner"),
        (-77.5,  175.0, "cav4 SB outer"),
        (-81.3,  175.0, "cav8 SB inner (STUCK)"),
    ]
    for x, y, label in spawns:
        lane_info(mp, x, y, label=label)

    # Scan across N-S road at y=80 to find all lanes
    print("\n=== N-S road cross-section at y=80 (x sweep) ===")
    for x in range(-90, -72):
        lane_info(mp, float(x), 80.0, label=f"x={x}")

    # Scan across N-S road at y=175 to find all lanes
    print("\n=== N-S road cross-section at y=175 (x sweep) ===")
    for x in range(-90, -72):
        lane_info(mp, float(x), 175.0, label=f"x={x}")

    # Scan across E-W road at x=-78 to find all lanes
    print("\n=== E-W road cross-section at x=-78 (y sweep) ===")
    for y in range(120, 145):
        lane_info(mp, -78.0, float(y), label=f"y={y}")

    # Trace lanes along the road from the hero spawn
    print("\n=== Trace NB lane from hero spawn ===")
    trace_lanes_along(mp, -84.8, 80.0, 90, distance=100)

    # Trace SB lane
    print("\n=== Trace SB lane from cav4 spawn ===")
    trace_lanes_along(mp, -77.5, 175.0, 270, distance=100)

    # Cross-section at y=190 (for cav12/cav16 on curved road 69)
    print("\n=== Road 69 cross-section at y=190 (x sweep) ===")
    for x in range(-90, -60):
        lane_info(mp, float(x), 190.0, label=f"x={x}")

    # Cross-section at y=188 (backup)
    print("\n=== Road 69 cross-section at y=188 (x sweep) ===")
    for x in range(-80, -60):
        lane_info(mp, float(x), 188.0, label=f"x={x}")

    # Trace road 69 lane -2 (cav8's corrected lane)
    print("\n=== Trace road 69 lane -2 from cav8 corrected spawn ===")
    trace_lanes_along(mp, -71.5, 175.0, 247, distance=50, step=5)

    # 16-ego candidates
    print("\n=== 16-ego candidate replacement positions ===")
    candidates = [
        (-88.1,  80.0, "cav5: NB lane -2 (verified)"),
        (-88.1,  65.0, "cav13: NB lane -2 at y=65"),
        (-71.5, 175.0, "cav8: road 69 lane -2 (verified)"),
        (-69.0, 188.0, "cav12: est lane -1 ~15m behind cav4"),
        (-66.0, 188.0, "cav16: est lane -2 ~15m behind cav8"),
        (-70.0, 185.0, "cav12 alt: lane -1 ~12m behind"),
        (-67.0, 185.0, "cav16 alt: lane -2 ~12m behind"),
    ]
    for x, y, label in candidates:
        lane_info(mp, x, y, label=label)


if __name__ == "__main__":
    main()
