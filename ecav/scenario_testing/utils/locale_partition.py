#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Q7 deployability study: partition a CARLA town into conflict-anchored
locales and measure the boundary geometry the migration protocol must serve.

Rule: locales anchor on junction centers (conflict zones); a boundary sits at
the midpoint of each inter-junction road, i.e. at maximal distance from both
adjacent conflicts. The boundary-to-conflict separation of that boundary is
half the inter-junction road length. Each protocol needs separation
>= D* = v x T_recover to re-establish state before the conflict; the fraction
of a map's boundaries meeting that is its deployable share.

Usage:
  python locale_partition.py --towns Town01 Town03 --out <dir> [--host localhost]
"""
from __future__ import annotations

import argparse
import csv
import math
import os
from collections import defaultdict

import carla

# Protocol recovery times (s): blocking transfer + track re-maturation etc.
# Sources: measured transfer 21-55 ms; tracker maturation ~2 s; cold
# re-acquisition ~1 s + history depth ~2 s (see scale_out_evaluation.md Q7).
PROTOCOLS = {
    "predictive latent (ours)": 0.05,
    "reactive (EdgeWarp timing)": 2.05,
    "cold (no migration)": 4.0,
}
SPEEDS = {"residential 8.3 m/s": 8.3, "arterial 14 m/s": 14.0}


def analyze_town(client, town):
    world = client.load_world(town)
    cmap = world.get_map()
    wps = cmap.generate_waypoints(2.0)

    junctions = {}
    for wp in wps:
        if wp.is_junction:
            j = wp.get_junction()
            if j is not None and j.id not in junctions:
                loc = j.bounding_box.location
                junctions[j.id] = (loc.x, loc.y)

    # One representative lane per road: group by road_id, keep the lane with
    # the smallest |lane_id| so parallel lanes don't double-count the road.
    lanes = defaultdict(list)
    for wp in wps:
        if not wp.is_junction:
            lanes[(wp.road_id, wp.lane_id)].append(wp)
    roads = {}
    for (rid, lid), pts in lanes.items():
        if rid not in roads or abs(lid) < abs(roads[rid][0]):
            roads[rid] = (lid, pts)

    rows = []
    for rid, (lid, pts) in roads.items():
        pts = sorted(pts, key=lambda w: w.s)
        length = pts[-1].s - pts[0].s
        if length < 4.0 or not junctions:
            continue
        mid = pts[len(pts) // 2]
        mx, my = mid.transform.location.x, mid.transform.location.y
        # nearest conflict anchors for drawing/QA
        d_anchor = min(math.hypot(mx - x, my - y)
                       for x, y in junctions.values())
        rows.append({
            "town": town, "road_id": rid, "length_m": round(length, 1),
            "separation_m": round(length / 2.0, 1),
            "boundary_x": round(mx, 1), "boundary_y": round(my, 1),
            "dist_to_nearest_anchor_m": round(d_anchor, 1),
        })
    return junctions, roads, rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--towns", nargs="+", default=["Town01", "Town03"])
    ap.add_argument("--out", required=True)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(120.0)

    all_rows = []
    draw = {}
    for town in args.towns:
        junctions, roads, rows = analyze_town(client, town)
        all_rows.extend(rows)
        draw[town] = (junctions, {
            rid: [(w.transform.location.x, w.transform.location.y)
                  for w in sorted(pts, key=lambda w: w.s)]
            for rid, (lid, pts) in roads.items()})
        seps = sorted(r["separation_m"] for r in rows)
        print(f"{town}: {len(junctions)} conflict anchors, "
              f"{len(rows)} inter-conflict roads, "
              f"separation median={seps[len(seps)//2]:.0f}m "
              f"min={seps[0]:.0f}m max={seps[-1]:.0f}m")
        for pname, tr in PROTOCOLS.items():
            for sname, v in SPEEDS.items():
                dstar = v * tr
                ok = sum(1 for s in seps if s >= dstar)
                print(f"  {pname:28s} @ {sname:20s} D*={dstar:5.1f}m "
                      f"deployable {100.0*ok/len(seps):5.1f}% of boundaries")

    with open(os.path.join(args.out, "locale_partition.csv"), "w",
              newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader()
        w.writerows(all_rows)

    import pickle
    with open(os.path.join(args.out, "draw_data.pkl"), "wb") as f:
        pickle.dump(draw, f)
    print("wrote", os.path.join(args.out, "locale_partition.csv"))


if __name__ == "__main__":
    main()
