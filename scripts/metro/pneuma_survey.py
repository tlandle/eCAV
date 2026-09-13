#!/usr/bin/env python3
"""pNEUMA calibration survey for the metropolitan replay (experiment 2, step 1).

ONE partition implementation for step 1 (pNEUMA) and step 3 (the SUMO
extension): the Q7 rule on a SUMO network built by netconvert from the Athens
OSM extract (locales anchored on junction nodes, boundaries at inter-junction
road midpoints, coarser levels by fixed-size tiling of the anchors so the
locale SIZE is what is held). The same implementation is run on CARLA's Town03
(netconvert --opendrive-files) to reproduce the committed Q7 numbers
(invalidation condition 4).

Steps
  1. parse one pNEUMA CSV (all drones, one date, one 30 min slot); bbox
  2. OSM extract of the bbox (surface streets + links, cached) -> netconvert
     (--geometry.remove --junctions.join) -> athens.net.xml (cached)
  3. partition on the net; lane-km from edge length x lane count
  4. lat/lon -> net x/y (net projection); 5 Hz cycle grid BEFORE counting; the
     25 Hz crossing rate reported once beside it
  5. statistics: instantaneous active tracks per cycle (median, p95),
     vehicles per lane-km, crossings per track per second, near-boundary
     fraction at 20/30/50 m, neighbour degree, per-boundary crossings per
     minute and observation coverage; cars+taxis+buses+trucks tracked,
     motorcycles separate
Usage: pneuma_survey.py <csv> <out_json> [--tile 300] [--work DIR]
Env: metro_env with SUMO_HOME set (netconvert on PATH, sumolib).
"""
import argparse, json, math, os, subprocess, sys, time, urllib.request, urllib.parse
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree
import sumolib

TRACKED = {"Car", "Taxi", "Bus", "Medium Vehicle", "Heavy Vehicle"}
TWO_WHEEL = {"Motorcycle"}
CYCLE_S = 0.2
MIN_ROAD_M = 4.0

# ----------------------------------------------------------------- pNEUMA
def parse_pneuma(path, max_rows=None):
    tracks = []
    with open(path, encoding="utf-8", errors="replace") as f:
        header = f.readline()
        for i, line in enumerate(f):
            if max_rows and i >= max_rows: break
            parts = [p.strip() for p in line.strip().strip(";").split(";")]
            if len(parts) < 10: continue
            tid, vtype = parts[0], parts[1]
            vals = np.array([float(v) for v in parts[4:] if v != ""], dtype=np.float64)
            n = len(vals) // 6
            if n < 2: continue
            v = vals[:n * 6].reshape(n, 6)   # lat, lon, speed, lon_acc, lat_acc, time
            tracks.append((tid, vtype, v[:, 5], v[:, 0], v[:, 1], v[:, 2]))
    return header, tracks

def subsample_to_grid(t, cols, dt=CYCLE_S):
    k = np.round(t / dt).astype(np.int64)
    keep = np.r_[True, k[1:] != k[:-1]]
    return k[keep], [c[keep] for c in cols]

# ----------------------------------------------------------------- network
def fetch_osm(bbox, path):
    if os.path.exists(path): return
    S, W, N, E = bbox
    q = f'''[out:xml][timeout:180];
(way["highway"~"^(motorway|trunk|primary|secondary|tertiary|residential|unclassified|living_street|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link)$"]({S},{W},{N},{E}););
(._;>;);
out body;'''
    data = urllib.parse.urlencode({"data": q}).encode()
    with urllib.request.urlopen(urllib.request.Request("https://overpass-api.de/api/interpreter", data=data,
                                headers={"User-Agent": "ecav-metro-survey"}), timeout=300) as r:
        open(path, "wb").write(r.read())

def build_net(osm_path, net_path, log_path):
    """netconvert with geometry-node removal and junction joining so a
    dual-carriageway crossing is one junction node and a way split is none."""
    if os.path.exists(net_path): return
    cmd = ["netconvert", "--osm-files", osm_path, "-o", net_path, "--geometry.remove", "--junctions.join",
           "--junctions.join-dist", "20", "--roundabouts.guess", "--ramps.guess", "--remove-edges.isolated",
           "--keep-edges.by-vclass", "passenger", "--no-turnarounds", "--proj.utm"]
    with open(log_path, "w") as lg:
        subprocess.run(cmd, check=True, stdout=lg, stderr=subprocess.STDOUT)

def partition(net, step_m=2.0):
    """Q7 rule on a SUMO net. Returns points (x, y, anchor label), anchors
    {node_id: (x, y)}, roads {key: (A, B, mid_xy, length)}, lane_km."""
    nodes = {n.getID(): n for n in net.getNodes()}
    def neighbours(n):
        out = set()
        for e in n.getIncoming(): out.add(e.getFromNode().getID())
        for e in n.getOutgoing(): out.add(e.getToNode().getID())
        out.discard(n.getID()); return out
    anchors = {nid: n.getCoord() for nid, n in nodes.items() if n.getType() != "dead_end" and len(neighbours(n)) >= 3}
    lane_km = 0.0; seg = {}
    for e in net.getEdges(withInternal=False):
        a, b = e.getFromNode().getID(), e.getToNode().getID()
        lane_km += e.getLength() * e.getLaneNumber() / 1000.0
        key = frozenset((a, b))
        if key not in seg or e.getLength() > seg[key][2]:
            seg[key] = (a, b, e.getLength(), [tuple(p) for p in e.getShape()])
    # chain merge: absorb non-anchor nodes with exactly two road segments
    adj = defaultdict(set)
    for key in seg:
        a, b = tuple(key); adj[a].add(key); adj[b].add(key)
    changed = True
    while changed:
        changed = False
        for nid in list(adj):
            if nid in anchors or len(adj[nid]) != 2: continue
            k1, k2 = list(adj[nid])
            if k1 == k2 or k1 not in seg or k2 not in seg: continue
            a1, b1, l1, s1 = seg[k1]; a2, b2, l2, s2 = seg[k2]
            o1 = a1 if b1 == nid else b1; o2 = a2 if b2 == nid else b2
            if o1 == o2 or o1 == nid or o2 == nid: continue
            s1 = s1 if b1 == nid else s1[::-1]; s2 = s2 if a2 == nid else s2[::-1]
            newkey = frozenset((o1, o2))
            if newkey in seg: continue
            seg[newkey] = (o1, o2, l1 + l2, s1 + s2[1:])
            del seg[k1]; del seg[k2]
            for o in (o1, o2):
                adj[o].discard(k1); adj[o].discard(k2); adj[o].add(newkey)
            adj[nid].clear(); changed = True
    X, Y, LAB = [], [], []; roads = {}
    for key, (a, b, L, shape) in seg.items():
        if a not in anchors and b not in anchors: continue
        sh = np.array(shape, dtype=float)
        d = np.r_[0.0, np.cumsum(np.hypot(np.diff(sh[:, 0]), np.diff(sh[:, 1])))]
        total = d[-1] if d[-1] > 0 else L
        ss = np.arange(0.0, total + 1e-6, step_m)
        xs = np.interp(ss, d, sh[:, 0]); ys = np.interp(ss, d, sh[:, 1])
        for s_, x_, y_ in zip(ss, xs, ys):
            near_a = s_ <= total / 2.0
            lab = (a if near_a else b) if (a in anchors and b in anchors) else (a if a in anchors else b)
            X.append(x_); Y.append(y_); LAB.append(lab)
        if a in anchors and b in anchors and a != b and total >= MIN_ROAD_M:   # Q7's own minimum (locale_partition.py: length < 4 m skipped)
            mx, my = np.interp(total / 2.0, d, sh[:, 0]), np.interp(total / 2.0, d, sh[:, 1])
            roads[key] = (a, b, (float(mx), float(my)), float(total))
    return dict(X=np.array(X), Y=np.array(Y), LAB=np.array(LAB), anchors=anchors, roads=roads, lane_km=lane_km,
                n_nodes=len(nodes), n_edges=len(net.getEdges(withInternal=False)))

def tile_anchors(anchors, tile_m):
    ids = sorted(anchors)
    if tile_m <= 0: return {i: k for k, i in enumerate(ids)}
    x0 = min(anchors[i][0] for i in ids); y0 = min(anchors[i][1] for i in ids)
    tiles = {}
    for i in ids:
        key = (int((anchors[i][0] - x0) // tile_m), int((anchors[i][1] - y0) // tile_m)); tiles.setdefault(key, len(tiles))
    return {i: tiles[(int((anchors[i][0] - x0) // tile_m), int((anchors[i][1] - y0) // tile_m))] for i in ids}

# ----------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv"); ap.add_argument("out")
    ap.add_argument("--tile", type=int, default=300, help="locale size in m (fixed-size tiling of anchors)")
    ap.add_argument("--work", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--max_rows", type=int, default=None); ap.add_argument("--pad_deg", type=float, default=0.002)
    ap.add_argument("--osm", default=None, help="existing OSM extract (default: fetch for the data bbox)")
    a = ap.parse_args()
    t0 = time.time()
    header, tracks = parse_pneuma(a.csv, a.max_rows)
    types = defaultdict(int)
    for tr in tracks: types[tr[1]] += 1
    lats = np.concatenate([tr[3] for tr in tracks]); lons = np.concatenate([tr[4] for tr in tracks])
    bbox = (float(lats.min()) - a.pad_deg, float(lons.min()) - a.pad_deg, float(lats.max()) + a.pad_deg, float(lons.max()) + a.pad_deg)
    print(f"parsed {len(tracks)} tracks in {time.time()-t0:.0f}s; types {dict(types)}; bbox {bbox}", flush=True)
    osm_path = a.osm or f"{a.work}/athens_pneuma.osm"; net_path = osm_path[:-4] + ".net.xml"
    fetch_osm(bbox, osm_path); build_net(osm_path, net_path, net_path + ".netconvert.log")
    net = sumolib.net.readNet(net_path, withInternal=False)
    P = partition(net); tree = cKDTree(np.c_[P["X"], P["Y"]])
    print(f"net: {P['n_nodes']} nodes, {P['n_edges']} edges, {len(P['anchors'])} junction anchors, {len(P['roads'])} inter-anchor roads, lane_km {P['lane_km']:.1f}, points {len(P['LAB'])} ({time.time()-t0:.0f}s)", flush=True)
    seps = np.array([r[3] / 2.0 for r in P["roads"].values()])
    T5, T25, d_all = [], [], []
    for tid, vt, t, lat, lon, spd in tracks:
        xy = np.array([net.convertLonLat2XY(lo, la) for lo, la in zip(lon, lat)], dtype=float)
        d, idx = tree.query(xy); d_all.append(d)
        k25 = np.round(t / 0.04).astype(np.int64)
        k5, (x5, y5, d5, i5) = subsample_to_grid(t, [xy[:, 0], xy[:, 1], d, idx])
        T5.append((tid, vt, k5, x5, y5, d5, i5)); T25.append((tid, vt, k25, d, idx))
    d_all = np.concatenate(d_all)
    align = {"median_m": float(np.median(d_all)), "p90_m": float(np.percentile(d_all, 90)), "frac_within_10m": float((d_all <= 10).mean())}
    print(f"alignment to the nearest road point: {align}", flush=True)
    OFFNET = 15.0
    out = {"csv": os.path.basename(a.csv), "n_tracks": len(tracks), "types": dict(types), "bbox": bbox, "alignment": align,
           "net": {"nodes": P["n_nodes"], "edges": P["n_edges"], "junction_anchors": len(P["anchors"]), "inter_anchor_roads": len(P["roads"]),
                   "separation_median_m": float(np.median(seps)) if len(seps) else None, "lane_km": P["lane_km"]},
           "cadence": {"cycle_s": CYCLE_S, "native_s": 0.04}, "levels": {}}
    act = defaultdict(int)
    for tid, vt, k5, x5, y5, d5, i5 in T5:
        if vt not in TRACKED: continue
        for k, d in zip(k5, d5):
            if d <= OFFNET: act[int(k)] += 1
    actv = np.array(sorted(act.values()))
    out["active_tracks_per_cycle"] = {"median": float(np.median(actv)), "p95": float(np.percentile(actv, 95)), "max": int(actv.max()), "cycles": len(actv), "session_s": len(actv) * CYCLE_S}
    out["vehicles_per_lane_km_median"] = float(np.median(actv) / P["lane_km"]) if P["lane_km"] > 0 else None
    for tile in (a.tile,):
        grp = tile_anchors(P["anchors"], tile)
        lab_of_pt = np.array([grp.get(l, -1) for l in P["LAB"]])
        b_pts, b_pair = [], []
        for key, (ja, jb, xy, L) in P["roads"].items():
            if grp.get(ja) != grp.get(jb): b_pts.append(xy); b_pair.append((grp.get(ja), grp.get(jb)))
        b_tree = cKDTree(np.array(b_pts)) if b_pts else None
        adj = defaultdict(set)
        for ga, gb in b_pair: adj[ga].add(gb); adj[gb].add(ga)
        deg = [len(v) for v in adj.values()]
        L_count = len(set(grp.values()))
        stats = {"tile_m": tile, "L": L_count, "boundaries": len(b_pts), "neighbour_degree_mean": float(np.mean(deg)) if deg else 0.0,
                 "generator_degree_at_L": 2.6 + 0.4 * math.log(L_count / 16.0 + 1.0)}
        for cls_name, cls in (("tracked", TRACKED), ("two_wheel", TWO_WHEEL)):
            cross5 = tc5 = cross25 = tc25 = 0; nb = {20: 0, 30: 0, 50: 0}
            b_cross = defaultdict(int); b_seen = set()
            for (tid, vt, k5, x5, y5, d5, i5), (_, _, k25, d25, i25) in zip(T5, T25):
                if vt not in cls: continue
                labs = np.where(d5 <= OFFNET, lab_of_pt[i5], -1); valid = labs >= 0
                tc5 += int(valid.sum())
                ch = (labs[1:] != labs[:-1]) & valid[1:] & valid[:-1]; cross5 += int(ch.sum())
                if b_tree is not None and valid.any():
                    db, ib = b_tree.query(np.c_[x5[valid], y5[valid]])
                    for r in nb: nb[r] += int((db <= r).sum())
                    b_seen.update(set(ib[db <= 30].tolist()))
                    for j in ib[np.r_[False, ch][valid]].tolist(): b_cross[j] += 1
                labs25 = np.where(d25 <= OFFNET, lab_of_pt[i25], -1); v25 = labs25 >= 0
                tc25 += int(v25.sum()); cross25 += int(((labs25[1:] != labs25[:-1]) & v25[1:] & v25[:-1]).sum())
            sec5 = tc5 * CYCLE_S; sec25 = tc25 * 0.04
            stats[cls_name] = {"track_cycles": tc5, "crossings_5hz": cross5, "crossing_rate_per_track_s_5hz": (cross5 / sec5) if sec5 else None,
                               "crossings_25hz": cross25, "crossing_rate_per_track_s_25hz": (cross25 / sec25) if sec25 else None,
                               "near_boundary_frac": {str(r): (nb[r] / tc5 if tc5 else None) for r in nb},
                               "boundaries_observed_frac": (len(b_seen) / len(b_pts)) if b_pts else None,
                               "boundaries_with_crossings_frac": (len(b_cross) / len(b_pts)) if b_pts else None,
                               "crossings_per_boundary_per_min_median": float(np.median([c / (len(actv) * CYCLE_S / 60.0) for c in b_cross.values()])) if b_cross else 0.0}
        stats["generator"] = {"crossing_rate_per_track_s": 0.013, "near_boundary_frac": 0.22}
        out["levels"][f"tile{tile}"] = stats
        tr_ = stats["tracked"]
        print(f"tile={tile} m: L={L_count} boundaries={len(b_pts)} degree={stats['neighbour_degree_mean']:.2f} (gen {stats['generator_degree_at_L']:.2f}) | tracked: crossing/track/s 5Hz {tr_['crossing_rate_per_track_s_5hz']:.4f} 25Hz {tr_['crossing_rate_per_track_s_25hz']:.4f} (gen 0.013) near20/30/50 {tr_['near_boundary_frac']['20']:.3f}/{tr_['near_boundary_frac']['30']:.3f}/{tr_['near_boundary_frac']['50']:.3f} (gen 0.22) observed {tr_['boundaries_observed_frac']:.2f}", flush=True)
    json.dump(out, open(a.out, "w"), indent=1)
    print(f"active tracks/cycle median {out['active_tracks_per_cycle']['median']:.0f} p95 {out['active_tracks_per_cycle']['p95']:.0f}; veh/lane-km {out['vehicles_per_lane_km_median']:.2f}; written {a.out} in {time.time()-t0:.0f}s")

if __name__ == "__main__":
    main()
