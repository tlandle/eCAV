#!/usr/bin/env python3
"""Generate the scale_town06 distributed scale scenario for N ego vehicles.

Writes, for one configuration (the files are overwritten, one configuration at a time):
    ecav/scenario_testing/config_yaml/scale_town06.yaml
    ecav/scenario_testing/scenarios/scenario_scale_town06.xml
The scenario class (scenarios/scenario_scale_town06.py) and the runner
(ecav/scenario_testing/scale_town06.py) are fixed files and read N from these two.

N ego vehicles are placed at rest on the straight carriageways of Town06, one slot every
--spacing metres along each driving lane, nearest slots to --centre first, so a small N is
one platoon block under the roadside units and a large N fills the map. Each ego follows
its lane to a destination --route-len metres ahead with the planner's car following.
Slots come from the OpenDRIVE file, so no CARLA server is needed.

Layouts:
    --layout dense     slots nearest to --centre first: one block whose local density does not
                       depend on N (the worst case: every vehicle is in range of most others)
    --layout spread    N slots taken evenly over every usable slot of the map, so the local
                       density grows with N only as the map fills. RSUs are placed so that
                       every vehicle is within --rsu-radius of one (--rsus is then a cap, 0 for none).
Range gates (both layouts, both execution modes, written into the YAML):
    --sense-range M    ground-truth object list of a vehicle without sensors is limited to M metres
                       (default: the vehicle lidar range of the base YAML). 0 keeps every vehicle.
    --recipient-range M  the edge sends a vehicle the forecasts within M metres (default 150:
                       the 54 m planned path plus two vehicles closing at the speed limit over
                       the 3 s forecast, with margin). 0 sends every forecast to every vehicle.

Variants:
    --perception off   exact-state edge manager (oracle), no sensors on vehicles or RSUs,
                       clients run without --apply_ml: planning and orchestration cost only
    --perception on    camera and lidar on every vehicle and RSU, late fusion edge manager,
                       clients run with --apply_ml

    python scripts/ispass/gen_scale_town06.py -n 32 --rsus 2 --perception off
    python scripts/ispass/gen_scale_town06.py -n 8 --perception on --carla-host 10.0.0.4
"""
import argparse
import copy
import glob
import math
import os
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / "ecav/scenario_testing/config_yaml"
SCN = ROOT / "ecav/scenario_testing/scenarios"
BASE_YAML = CFG / "scale_town06_base.yaml"   # Town06 base: vehicle, RSU, edge and controller blocks
NAME = "scale_town06"
EGO_MODEL = "vehicle.lincoln.mkz_2017"


def find_xodr(arg):
    if arg:
        return arg
    cands = [os.path.expanduser("~/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive/Town06.xodr"),
             "/opt/carla-simulator/CarlaUE4/Content/Carla/Maps/OpenDrive/Town06.xodr"]
    cands += glob.glob(os.path.expanduser("~/carla*/CarlaUE4/Content/Carla/Maps/OpenDrive/Town06.xodr"))
    for c in cands:
        if os.path.isfile(c):
            return c
    sys.exit("Town06.xodr not found; pass --xodr")


def slots(xodr, spacing, route_len, min_seg, centre, min_route):
    """Spawn slots (x, y, z, yaw, dest_xyz) on straight non-junction driving lanes, nearest to centre first."""
    import carla
    cmap = carla.Map("Town06", open(xodr).read())
    lanes = {}
    for w in cmap.generate_waypoints(2.0):
        if w.lane_type == carla.LaneType.Driving and not w.is_junction:
            lanes.setdefault((w.road_id, w.lane_id), []).append(w)
    out = []
    for (road, lane), wps in lanes.items():
        wps.sort(key=lambda w: w.s)
        if lane > 0:              # positive lanes run against s
            wps = wps[::-1]
        a, b = wps[0].transform, wps[-1].transform
        length = abs(wps[-1].s - wps[0].s)
        dyaw = abs((b.rotation.yaw - a.rotation.yaw + 180) % 360 - 180)
        if length < min_seg or dyaw > 3.0:
            continue
        # leave 8 m at the segment start and 25 m before the junction at its end
        d = 8.0
        while d <= length - 25.0:
            w = wps[min(int(round(d / 2.0)), len(wps) - 1)]
            dest = w
            yaw0 = w.transform.rotation.yaw
            gone = 0.0
            while gone < route_len:   # straight on only: stop where no successor keeps the heading
                nxt = dest.next(5.0)
                if not nxt:
                    break
                best = min(nxt, key=lambda n: abs((n.transform.rotation.yaw - yaw0 + 180) % 360 - 180))
                if abs((best.transform.rotation.yaw - yaw0 + 180) % 360 - 180) > 2.0:
                    break
                dest = best
                gone += 5.0
            if gone < min_route:      # a lane that leaves the carriageway soon: not a slot
                d += spacing
                continue
            t = w.transform
            dl = dest.transform.location
            out.append(dict(x=t.location.x, y=t.location.y, z=t.location.z + 0.5, yaw=t.rotation.yaw,
                            dest=[round(dl.x, 2), round(dl.y, 2), round(dl.z + 0.5, 2)],
                            road=road, lane=lane, s=round(d, 1), route=gone))
            d += spacing
    # Row-major fill outward from the centre: rows (same road and s) nearest first.
    out.sort(key=lambda p: (round(math.hypot(p["x"] - centre[0], p["y"] - centre[1]) / spacing), p["road"], p["s"], p["lane"]))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--num-egos", type=int, required=True)
    ap.add_argument("--rsus", type=int, default=2)
    ap.add_argument("--perception", choices=["on", "off"], default="off")
    ap.add_argument("--manager-type", default=None, help="edge manager type (default: oracle without perception, late_fusion with)")
    ap.add_argument("--speed-kmh", type=int, default=40)
    ap.add_argument("--spacing", type=float, default=12.0, help="metres between slots along a lane")
    ap.add_argument("--route-len", type=float, default=250.0)
    ap.add_argument("--min-route", type=float, default=150.0, help="shortest straight route ahead of a slot, metres")
    ap.add_argument("--min-seg", type=float, default=80.0, help="shortest lane segment used, metres")
    ap.add_argument("--centre", type=float, nargs=2, default=[350.0, 145.0])
    ap.add_argument("--layout", choices=["dense", "spread"], default="dense")
    ap.add_argument("--rsu-radius", type=float, default=100.0, help="spread layout: every vehicle within this distance of an RSU")
    ap.add_argument("--sense-range", type=float, default=None, help="metres; default the base YAML's vehicle lidar range; 0 unlimited")
    ap.add_argument("--recipient-range", type=float, default=150.0, help="metres; 0 unlimited")
    ap.add_argument("--carla-host", default="localhost", help="CARLA host as the clients reach it (scenario_runner.host, world.client_host)")
    ap.add_argument("--carla-port", type=int, default=2000)
    ap.add_argument("--xodr", default=None)
    ap.add_argument("--timeout", type=int, default=1800, help="scenario_runner watchdog, seconds without a tick")
    a = ap.parse_args(argv)
    N, R = a.num_egos, a.rsus
    percep = a.perception == "on"

    xodr = find_xodr(a.xodr)
    for spacing, min_route in ((a.spacing, a.min_route), (10.0, 120.0), (9.0, 100.0)):
        sl = slots(xodr, spacing, a.route_len, a.min_seg, a.centre, min_route)
        if len(sl) >= N:
            break
    else:
        sys.exit(f"only {len(sl)} slots on Town06 at {spacing:g} m spacing; N is too large for this map")
    a.spacing = spacing
    if a.layout == "spread":
        # even stride over the slots in road order (road, s, lane), so the fleet covers the map
        order = sorted(sl, key=lambda p: (p["road"], p["s"], p["lane"]))
        sl = [order[int(i * len(order) / N)] for i in range(N)]
    else:
        sl = sl[:N]

    # ---- scenario XML: one ego entry per vehicle, role names hero, hero1, hero2, ...
    lines = ['<?xml version="1.0"?>', "<scenarios>",
             f"    <!-- Generated by scripts/ispass/gen_scale_town06.py: {N} egos, {R} RSUs, perception {a.perception}. -->",
             '    <scenario name="Scenario_ScaleTown06" type="Scenario_ScaleTown06" town="Town06">']
    for k, p in enumerate(sl):
        role = "hero" if k == 0 else f"hero{k}"
        lines.append(f'        <ego_vehicle x="{p["x"]:.2f}" y="{p["y"]:.2f}" z="{p["z"]:.2f}" yaw="{p["yaw"]:.2f}" '
                     f'model="{EGO_MODEL}" rolename="{role}" />')
    lines += ['        <weather cloudiness="10" precipitation="0" precipitation_deposits="0" wind_intensity="0" '
              'sun_azimuth_angle="0" sun_altitude_angle="75" fog_density="0" />', "    </scenario>", "</scenarios>", ""]
    (SCN / "scenario_scale_town06.xml").write_text("\n".join(lines))

    # ---- YAML from the Town06 base
    d = yaml.safe_load(BASE_YAML.read_text())
    d["description"] = (f"Scale scenario on Town06, generated by scripts/ispass/gen_scale_town06.py: {N} ego vehicles "
                        f"at {a.spacing:g} m spacing on the straight carriageways, {R} RSUs, one edge, perception {a.perception}. "
                        "Every ego is its own client process in distributed mode.")
    d["scenario_name"] = NAME
    d["perception_active"] = percep
    d["ego_vehicle_max_speed"] = a.speed_kmh
    d["world"]["client_host"] = a.carla_host
    d["world"]["client_port"] = a.carla_port
    sr = d["scenario_runner"]
    sr.update(scenario="Scenario_ScaleTown06", num_actors=N,
              configFile="ecav/scenario_testing/scenarios/scenario_scale_town06.xml",
              additionalScenario="ecav/scenario_testing/scenarios/scenario_scale_town06.py",
              host=a.carla_host, port=a.carla_port, timeout=a.timeout,
              openscenarioparams=[f"ego_vehicle_max_speed={a.speed_kmh}"])
    edge = d["scenario"]["edge_list"][0]
    for key in ("per_tick_age_log", "closing_speed_mps"):
        edge.pop(key, None)
    edge["manager_type"] = a.manager_type or ("late_fusion" if percep else "oracle")
    edge["max_capacity"] = max(N, 10)
    edge["latency"] = 0.0
    d["edge_base"]["max_capacity"] = max(N, 10)

    def set_perception(sensing):
        s = copy.deepcopy(sensing)
        s["perception"]["activate"] = percep
        s["perception"]["camera"]["visualize"] = 0
        s["perception"]["lidar"]["visualize"] = False
        return s

    vt = edge["vehicles"][0]
    v_sensing = set_perception(vt["sensing"])
    sense = a.sense_range if a.sense_range is not None else float(vt["sensing"]["perception"]["lidar"].get("range", 50))
    if sense > 0:
        v_sensing["perception"]["deactivate_range_m"] = sense
    if a.recipient_range > 0:
        edge["recipient_range_m"] = a.recipient_range
    v_behavior = copy.deepcopy(vt["behavior"])
    v_behavior.update(max_speed=a.speed_kmh, speed_lim_dist=3, overtake_allowed=False, start_hold_ticks=0)
    v_behavior["local_planner"].update(buffer_size=12, waypoint_update_freq=9)
    v_controller = copy.deepcopy(vt["controller"])
    v_controller["args"]["max_brake"] = 1.0
    vehicles = []
    for k, p in enumerate(sl):
        v = {key: val for key, val in vt.items() if key not in ("sensing", "behavior", "controller", "name", "destination")}
        v.update(sensing=v_sensing, behavior=v_behavior, controller=v_controller, name=f"cav{k + 1}", destination=p["dest"])
        vehicles.append(v)             # shared sub-objects are written once and referenced by YAML aliases
    edge["vehicles"] = vehicles

    rt = edge["rsus"][0]
    r_sensing = set_perception(rt["sensing"])
    if a.layout == "spread":
        # greedy cover: a vehicle farther than --rsu-radius from every RSU gets one at the roadside
        # next to it (10 m to the right of its lane); --rsus caps the count
        rsu_pos = []
        for p in sl:
            if len(rsu_pos) >= R:
                break
            if all(math.hypot(p["x"] - q[0], p["y"] - q[1]) > a.rsu_radius for q in rsu_pos):
                yaw = math.radians(p["yaw"])
                rsu_pos.append([round(p["x"] - 10.0 * math.sin(yaw), 1), round(p["y"] + 10.0 * math.cos(yaw), 1), 7.0])
        R = len(rsu_pos)
    else:
        rsu_pos = [[float(a.centre[0] - 47.5 * (R - 1) + 95.0 * j), 155.0, 7.0] for j in range(R)]
    rsus = []
    for j in range(R):
        r = {key: val for key, val in rt.items() if key not in ("sensing", "name", "spawn_position", "id")}
        r.update(sensing=r_sensing, name=f"rsu{j + 1}", id=-(j + 1), spawn_position=rsu_pos[j])
        rsus.append(r)
    edge["rsus"] = rsus

    (CFG / f"{NAME}.yaml").write_text(
        f"# Generated by scripts/ispass/gen_scale_town06.py -n {N} --rsus {R} --perception {a.perception}; do not edit.\n"
        + yaml.dump(d, sort_keys=False, default_flow_style=None, width=120))
    xs = [p["x"] for p in sl]; ys = [p["y"] for p in sl]
    def within(r):
        return sum(sum(1 for q in sl if q is not p and math.hypot(p["x"] - q["x"], p["y"] - q["y"]) <= r) for p in sl) / max(N, 1)
    zone = [sum(1 for p in sl if math.hypot(p["x"] - q[0], p["y"] - q[1]) <= a.rsu_radius) for q in rsu_pos]
    print(f"RSUS={R}")
    print(f"layout {a.layout}: mean vehicles within {sense:g} m of a vehicle {within(sense if sense > 0 else 1e9):.1f}, "
          f"within {a.recipient_range:g} m {within(a.recipient_range if a.recipient_range > 0 else 1e9):.1f}; "
          f"vehicles within {a.rsu_radius:g} m of an RSU: mean {sum(zone) / max(len(zone), 1):.1f}, max {max(zone) if zone else 0}")
    print(f"scale_town06: {N} egos on {len({(p['road'], p['lane']) for p in sl})} lanes, x {min(xs):.0f}..{max(xs):.0f}, "
          f"y {min(ys):.0f}..{max(ys):.0f}; {R} RSUs; perception {a.perception}; manager {edge['manager_type']}; "
          f"shortest route {min(p['route'] for p in sl):.0f} m; CARLA {a.carla_host}:{a.carla_port}")


if __name__ == "__main__":
    main()
