# -*- coding: utf-8 -*-
"""LocalPlanner: the lateral target must lie a minimum distance ahead.

Replays the gate run_4 situation of 2026-10-14 (openscenario_3_edge_late_fusion,
sensys_nov27_gate1, 0 ms, SBA on) offline on the Town03 OpenDRIVE: the hero
had completed its left turn, was held to a full stop by an RSS proper
response at the junction exit (-62.5, 136.4), 1 m right of the eastbound
lane centre, and restarted. generate_trajectory spaces points by the motion
predicted per trajectory_dt, so at standstill the old target (trajectory
point 1) was 0.17 m ahead and 1.0 m to the side (heading error 81 deg,
full lock); with the turn's history waypoints in the spline the path start
was 1.8 m behind (error 153 deg). The vehicle pulled away under full lock,
crossed the lane, and the target fell behind it until it hit a building at
(-55, 121). The fix picks the first trajectory point at least
max(2.5 m, 0.4 s * v) ahead of the vehicle.

Run: python tests/test_local_planner_lookahead.py  (no CARLA server; no pytest in opencda310)
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import carla  # noqa: E402
from ecav.core.plan.local_planner_behavior import LocalPlanner  # noqa: E402
from ecav.core.plan.global_route_planner import GlobalRoutePlanner  # noqa: E402
from ecav.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO  # noqa: E402

XODR = '/home/atlas/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr'
CFG = {'buffer_size': 12, 'trajectory_update_freq': 15, 'waypoint_update_freq': 9, 'min_dist': 3,
       'trajectory_dt': 0.20, 'debug': False, 'debug_trajectory': False}


class _Veh:
    bounding_box = carla.BoundingBox(carla.Location(0, 0, 0.9), carla.Vector3D(2.35, 1.0, 0.9))

    def __init__(self):
        self.loc = carla.Location(0, 0, 0)

    def get_location(self):
        return self.loc


class _Agent:
    def __init__(self):
        self.vehicle = _Veh()


def _planner_on_hero_route():
    m = carla.Map('Town03', open(XODR).read())
    agent = _Agent()
    lp = LocalPlanner(agent, m, CFG)
    grp = GlobalRoutePlanner(GlobalRoutePlannerDAO(m, 4.5))
    grp.setup()
    start = m.get_waypoint(carla.Location(x=-84.8, y=80.0, z=0.5)).transform.location
    lp.set_global_plan(grp.trace_route(start, carla.Location(x=-28.8, y=134.7, z=0.5)), clean=True)
    return lp, agent


def _dense(points, step=0.5):
    """Interpolate the pose list at `step` metres so every route waypoint is
    passed within the planner's pop distance, as in a 20 Hz run."""
    out = [points[0]]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
        out += [(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n) for k in range(1, n + 1)]
    return out


def _drive(lp, agent, points, yaw_deg, speed_kmh):
    """Feed poses (pop the buffer as the vehicle passes) and return the last
    target relative to the vehicle: (forward_m, lateral_m, angle_deg)."""
    tgt = None
    for (x, y) in _dense(points):
        agent.vehicle.loc = carla.Location(x=x, y=y, z=0.0)
        lp.update_information(carla.Transform(carla.Location(x=x, y=y, z=0.0), carla.Rotation(yaw=yaw_deg)), speed_kmh)
        rx, ry, rk, _ = lp.generate_path()
        _, tgt = lp.run_step(rx, ry, rk, target_speed=47.0)
    x, y = points[-1]
    yr = math.radians(yaw_deg)
    dx, dy = tgt.x - x, tgt.y - y
    fwd = dx * math.cos(yr) + dy * math.sin(yr)
    lat = -dx * math.sin(yr) + dy * math.cos(yr)
    return fwd, lat, math.degrees(math.atan2(lat, fwd))


# the hero's ground-truth poses through the turn to the stop (gate run_4, ticks 100..177)
TURN = [(-84.6, 99.5), (-84.5, 107.2), (-84.4, 116.9), (-84.5, 123.6), (-83.5, 129.5), (-78.9, 134.9),
        (-74.2, 136.8), (-71.7, 137.1), (-66.8, 136.9), (-64.3, 136.6), (-62.9, 136.4), (-62.5, 136.4)]


def test_standstill_restart_after_turn_targets_ahead():
    lp, agent = _planner_on_hero_route()
    _drive(lp, agent, TURN[:-1], 20.0, 40.0)
    fwd, lat, ang = _drive(lp, agent, TURN[-1:], 0.0, 0.0)  # stopped, heading east, 1 m right of centre
    assert fwd >= 2.5, (fwd, lat, ang)
    assert abs(ang) < 35.0, (fwd, lat, ang)


def test_standstill_on_straight_lane_targets_ahead():
    for x in (-60.0, -55.0, -50.0):
        lp, agent = _planner_on_hero_route()
        _drive(lp, agent, TURN + [(x, 136.3)], 10.0, 30.0)
        fwd, lat, ang = _drive(lp, agent, [(x, 136.3)], 0.0, 0.0)
        assert fwd >= 2.5 and abs(ang) < 35.0, (x, fwd, lat, ang)


def test_moving_target_is_unchanged():
    lp, agent = _planner_on_hero_route()
    _drive(lp, agent, TURN + [(-55.0, 135.3)], 10.0, 30.0)
    fwd, lat, ang = _drive(lp, agent, [(-55.0, 135.3)], 0.0, 47.0)
    # at 47 km/h the second sample (about 0.4 s ahead) is the target, as before
    assert 4.0 <= fwd <= 7.0, (fwd, lat, ang)


if __name__ == '__main__':
    lp, agent = _planner_on_hero_route()
    _drive(lp, agent, TURN[:-1], 20.0, 40.0)
    print('restart after turn, stopped 1 m off centre: fwd=%.2f lat=%.2f ang=%.0f' % _drive(lp, agent, TURN[-1:], 0.0, 0.0))
    for x in (-60.0, -55.0, -50.0):
        lp, agent = _planner_on_hero_route()
        _drive(lp, agent, TURN + [(x, 136.3)], 10.0, 30.0)
        print('stopped at x=%.0f: fwd=%.2f lat=%.2f ang=%.0f' % ((x,) + _drive(lp, agent, [(x, 136.3)], 0.0, 0.0)))
    lp, agent = _planner_on_hero_route()
    _drive(lp, agent, TURN + [(-55.0, 135.3)], 10.0, 30.0)
    print('moving 47 km/h: fwd=%.2f lat=%.2f ang=%.0f' % _drive(lp, agent, [(-55.0, 135.3)], 0.0, 47.0))
