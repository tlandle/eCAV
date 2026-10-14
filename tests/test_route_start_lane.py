"""Route start lane at a junction exit with overlapping lanes (Town03 road 24
exit). Runs standalone under opencda310 (no pytest): python tests/test_route_start_lane.py

The nearest lane centre to a hero that is a few decimetres off its lane in
the junction is a lane of another road, often one running the other way.
route_start_waypoint must keep the nearest lane only when it is aligned with
the heading, and otherwise start the route from the first aligned route
waypoint ahead. The replayed positions are the hero's ground truth from the
policy_smoke oracle run that drifted off the road (2026-10-14).
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import carla  # noqa: E402
from ecav.core.plan.behavior_agent import route_start_waypoint  # noqa: E402
from ecav.core.plan.global_route_planner import GlobalRoutePlanner  # noqa: E402
from ecav.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO  # noqa: E402

XODR = '/home/atlas/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr'
m = carla.Map('Town03', open(XODR).read())
grp = GlobalRoutePlanner(GlobalRoutePlannerDAO(m, 4.5))
grp.setup()
# the hero's route: north on road 4, left through the junction, east on road 24
route = grp.trace_route(carla.Location(x=-84.8, y=80.0, z=0.5), carla.Location(x=-28.8, y=134.7, z=0.5))
remaining = [r for r in route if r[0].transform.location.x > -82.0]


def loc(x, y):
    return carla.Location(x=x, y=y, z=0.5)


# 1. positions along the junction exit where the nearest lane is another road
cases = [((-80.6, 132.3), 33.0), ((-77.4, 133.9), 20.0), ((-73.0, 133.8), 10.0), ((-68.8, 131.4), 5.0)]
for (x, y), yaw in cases:
    nearest = m.get_waypoint(loc(x, y))
    start = route_start_waypoint(m, loc(x, y), yaw, remaining)
    diff = (start.transform.rotation.yaw - yaw + 180.0) % 360.0 - 180.0
    ahead = (start.transform.location.x - x) * math.cos(math.radians(yaw)) + \
            (start.transform.location.y - y) * math.sin(math.radians(yaw))
    print(f"({x},{y}) yaw {yaw:.0f}: nearest {nearest.road_id}/{nearest.lane_id} yaw {nearest.transform.rotation.yaw:.0f} "
          f"-> start {start.road_id}/{start.lane_id} yaw {start.transform.rotation.yaw:.0f} ahead {ahead:+.1f} m")
    assert abs(diff) <= 60.0, f"start lane not aligned with the heading at ({x},{y})"
    assert ahead >= 0.0, f"start waypoint behind the vehicle at ({x},{y})"
    assert nearest.road_id != start.road_id or abs(diff) <= 60.0
    # the route traced from the chosen start reaches road 24 eastbound without a detour
    new_route = grp.trace_route(start.transform.location, carla.Location(x=-37.0, y=134.8, z=0.5))
    pts = [(r[0].transform.location.x, r[0].transform.location.y) for r in new_route]
    length = sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts)))
    assert length < 60.0, f"route from ({x},{y}) is {length:.0f} m: a detour"

# 2. the nearest lane is kept when it is aligned (on road 24 after the junction)
start = route_start_waypoint(m, loc(-55.0, 134.9), 0.0, remaining)
assert (start.road_id, start.lane_id) == (24, -1), (start.road_id, start.lane_id)

# 3. without a heading the nearest lane is returned unchanged
start = route_start_waypoint(m, loc(-80.6, 132.3), None, remaining)
nearest = m.get_waypoint(loc(-80.6, 132.3))
assert (start.road_id, start.lane_id) == (nearest.road_id, nearest.lane_id)

# 4. misaligned nearest lane and no route waypoint ahead: the nearest lane is the fallback
start = route_start_waypoint(m, loc(-80.6, 132.3), 33.0, [])
assert (start.road_id, start.lane_id) == (nearest.road_id, nearest.lane_id)
print("ok")
