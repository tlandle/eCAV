#!/usr/bin/env python
"""Generate the column scenarios: N sensor-equipped vehicles approach the straight crossing in one coupled world.

    python scripts/ispass/gen_column_scenario.py            # writes scp_column.yaml, scp_column_{2,4,6,8,16}ego.yaml
                                                            # and the scenario modules of the same names

The single-vehicle scenario scp_edge_late_fusion (as committed) is the base. Vehicle 1 is the scenario runner's
vehicle (its start is moved by EGO_SPAWN_Y, -30 in the density study). Vehicles 2 to N copy its configuration and
start behind it on the same lane, SPACING metres apart (Town03 road 3, lane -1, straight from y -120 to -20). From
the ninth vehicle on, the second northbound lane (lane -2) is filled the same way, abreast of the first column. A
smaller N is a subset of a larger N. Every vehicle has the leader's sensors, tracker, predictor, planner, target
speed and destination lane, so each vehicle has to react to the vehicle ahead with its own state source.
Lane centres come from the map's OpenDRIVE file, read without a simulator.
"""
import os, re, subprocess, sys
import carla

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = os.path.join(ROOT, "ecav", "scenario_testing", "config_yaml"); MOD = os.path.join(ROOT, "ecav", "scenario_testing")
BASE = "scp_edge_late_fusion"; NAME = "scp_column"; NS = [1, 2, 4, 6, 8, 16]
LEAD_Y = -30.0; SPACING = 10.0; PER_LANE = 8
SPEED = 40      # km/h, the target speed of every follower; test_runner --speeds sets the leader's
FOLLOW_LANE = False
XODR = os.path.expanduser("~/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town03.xodr")


def slots():
    """Start poses (x, y, yaw) of vehicles 2..16, in filling order."""
    m = carla.Map("Town03", open(XODR).read()); out = []
    if FOLLOW_LANE:
        # one lane, any length: follower k stands k * SPACING metres behind the leader along the lane, through
        # the junction and the bend south of road 3 (the straightest predecessor at every 1 m step)
        # With --per-lane P below the largest N, vehicles P+1.. fill the lane to the right, abreast of the first P.
        import math
        wp = m.get_waypoint(carla.Location(-84.8, LEAD_Y, 0.0)); assert abs(wp.transform.location.x + 84.9) < 0.6, wp.transform.location
        dist = 0.0; lane0 = [wp]
        for k in range(1, min(PER_LANE, max(NS))):
            while dist < k * SPACING - 1e-6:
                prev = wp.previous(1.0); assert prev, "the lane ends behind the leader"
                wp = min(prev, key=lambda w: abs(((w.transform.rotation.yaw - wp.transform.rotation.yaw + 180) % 360) - 180)); dist += 1.0
            lane0.append(wp)
        for w in lane0[1:]:
            t = w.transform; out.append((round(t.location.x, 2), round(t.location.y, 2), round(t.rotation.yaw % 360, 1)))
        for w in lane0[:max(0, max(NS) - PER_LANE)]:
            t = w.transform; yaw = math.radians(t.rotation.yaw)
            r = m.get_waypoint(carla.Location(t.location.x - 3.5 * math.sin(yaw), t.location.y + 3.5 * math.cos(yaw), 0.0))
            off = math.hypot(r.transform.location.x - t.location.x, r.transform.location.y - t.location.y)
            assert 3.0 < off < 4.0 and abs(((r.transform.rotation.yaw - t.rotation.yaw + 180) % 360) - 180) < 5, ("no lane to the right", t.location, off)
            out.append((round(r.transform.location.x, 2), round(r.transform.location.y, 2), round(r.transform.rotation.yaw % 360, 1)))
        return out
    for lane in (0, 1):
        if lane * PER_LANE >= max(NS):
            break
        for k in range(PER_LANE):
            if lane == 0 and k == 0: continue            # the scenario runner's vehicle
            wp = m.get_waypoint(carla.Location(-84.8, LEAD_Y - k * SPACING, 0.0))
            assert wp.road_id == 3 and wp.lane_id == -1 and not wp.is_junction, (k, wp.road_id, wp.lane_id)
            if lane == 1:
                wp = wp.get_right_lane() if wp.get_right_lane() and wp.get_right_lane().lane_type == carla.LaneType.Driving else wp.get_left_lane()
                assert wp is not None and wp.lane_type == carla.LaneType.Driving and wp.lane_id == -2, "no second northbound lane"
            t = wp.transform
            out.append((round(t.location.x, 2), round(t.location.y, 2), round(t.rotation.yaw % 360, 1)))
    return out


def main():
    # Variant for a column that meets the crossing car at speed (the leader then starts at EGO_SPAWN_Y, farther
    # south, and the followers have to start behind it):
    #   gen_column_scenario.py --name scp_colx60 --lead-y -90 --per-lane 4 --ns 4 8 --speed 60
    import argparse
    global NAME, NS, LEAD_Y, PER_LANE, SPEED, SPACING, FOLLOW_LANE
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default=NAME); ap.add_argument("--ns", type=int, nargs="+", default=NS)
    ap.add_argument("--lead-y", type=float, default=LEAD_Y); ap.add_argument("--per-lane", type=int, default=PER_LANE)
    ap.add_argument("--speed", type=int, default=SPEED); ap.add_argument("--spacing", type=float, default=SPACING)
    ap.add_argument("--follow-lane", action="store_true", help="one lane of any length: followers stand along the lane behind the leader")
    a = ap.parse_args()
    NAME, NS, LEAD_Y, PER_LANE, SPEED, SPACING, FOLLOW_LANE = a.name, a.ns, a.lead_y, a.per_lane, a.speed, a.spacing, a.follow_lane
    if FOLLOW_LANE and a.per_lane == 8 and "--per-lane" not in sys.argv:
        PER_LANE = max(NS)
    text = subprocess.run(["git", "show", f"HEAD:ecav/scenario_testing/config_yaml/{BASE}.yaml"], cwd=ROOT, capture_output=True, text=True, check=True).stdout
    head, sep, block = text.partition("      vehicles:\n")
    assert sep and block.startswith("        - <<: *vehicle_base\n          name: cav1\n"), "unexpected vehicle block"
    assert block.count("        - <<: *vehicle_base") == 1
    block = block.rstrip("\n") + "\n"
    dest = re.search(r"          destination: \[(-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\]", block)
    s = slots()
    for n in NS:
        name = NAME if n == 1 else f"{NAME}_{n}ego"
        h = head.replace(f"scenario_name: {BASE}", f"scenario_name: {name}")
        h = re.sub(r"(edge_base: &edge_base\n  max_capacity: )\d+", r"\g<1>32", h)
        # the scenario runner's watchdog: a tick with many sensor-equipped vehicles takes seconds
        h, k = re.subn(r"(\n  timeout: )10\b", r"\g<1>600", h); assert k == 1, "scenario_runner.timeout"
        out = h + sep + block
        for i in range(n - 1):
            x, y, yaw = s[i]
            b = block.replace("name: cav1\n", f"name: cav{i + 2}\n          spawn_position: [{x}, {y}, 1.0, 0, {yaw}, 0]  # column slot {i + 1}\n")
            b = b.replace("                visualize: 4\n", "                visualize: 0\n")
            b, k = __import__("re").subn(r"(          behavior:\n            <<: \*base_behavior\n)", r"\g<1>            max_speed: %d\n" % SPEED, b); assert k == 1, "behavior block"
            if i + 1 >= PER_LANE:    # the second lane keeps its lane through the junction
                b = b.replace(dest.group(0), f"          destination: [-88.6, {dest.group(2)}, {dest.group(3)}]")
            out += b
        open(os.path.join(CFG, name + ".yaml"), "w").write(out)
        open(os.path.join(MOD, name + ".py"), "w").write(
            '# -*- coding: utf-8 -*-\n"""Column of %d sensor-equipped vehicle%s on the straight crossing. Generated by\n'
            'scripts/ispass/gen_column_scenario.py; the scenario logic is scp_edge_late_fusion."""\n'
            'from ecav.scenario_testing.scp_edge_late_fusion import *  # noqa: F401,F403\n'
            'from ecav.scenario_testing.scp_edge_late_fusion import run_scenario, run_vehicle, exec_scenario_runner  # noqa: F401\n'
            % (n, "" if n == 1 else "s"))
        print(name, "vehicles", n, "slots", s[:n - 1][-2:])


if __name__ == "__main__":
    main()
