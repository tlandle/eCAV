#!/usr/bin/env python3
"""pNEUMA district survey for the metropolitan replay (experiment 2).

Input: one pNEUMA CSV (all drones, one date, one 30 min slot). Steps:
  1. parse trajectories, subsample to the 0.2 s edge-cycle grid (5 Hz); keep the
     25 Hz samples for the one-time cadence comparison
  2. bbox -> Athens OSM extract (surface streets, cached) -> carla.Osm2Odr ->
     OpenDRIVE (cached) -> client-side carla.Map; georeference from the xodr
     <geoReference> proj string via pyproj; CARLA frame y = -y_proj
  3. alignment check: distance of trajectory points to the nearest driving
     waypoint (median, p90) - the frame is wrong if the median is not metres
  4. Q7 partition: one locale per junction anchor, boundary at each
     inter-junction road midpoint; coarser levels by FIXED-SIZE tiling of the
     anchors (locale size held, count follows the area: the paper's sweep)
  5. statistics per partition level: active tracks per cycle (median, p95) by
     class; crossings per track per second at 5 Hz (and 25 Hz once);
     near-boundary fraction at 20/30/50 m; neighbour degree; per-boundary
     crossings per minute and observation coverage; vehicles per lane-km
Usage: pneuma_survey.py <csv> <out_json> [--tiles 0,300,200,400]  (0 = per-junction)
Env: metro_env (carla, pyproj, scipy, shapely).
"""
import argparse, json, math, os, re, sys, time, urllib.request, urllib.parse
from collections import defaultdict
import numpy as np
import carla
from scipy.spatial import cKDTree
from pyproj import Transformer

TRACKED = {"Car", "Taxi", "Bus", "Medium Vehicle", "Heavy Vehicle"}
TWO_WHEEL = {"Motorcycle"}
CYCLE_S = 0.2

def parse_pneuma(path, max_rows=None):
    """Yield (track_id, vtype, t[], lat[], lon[], speed[]) at native 25 Hz."""
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
    """Keep the sample nearest to each multiple of dt (one per cycle)."""
    k = np.round(t / dt).astype(np.int64)
    keep = np.r_[True, k[1:] != k[:-1]]
    return k[keep], [c[keep] for c in cols]

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

def osm_to_xodr(osm_path, xodr_path):
    if os.path.exists(xodr_path): return open(xodr_path).read()
    st = carla.Osm2OdrSettings(); st.use_offsets = False; st.center_map = False
    st.generate_traffic_lights = False; st.default_lane_width = 3.5
    xodr = carla.Osm2Odr.convert(open(osm_path, encoding="utf-8").read(), st)
    open(xodr_path, "w").write(xodr); return xodr

def build_partition(m):
    """Return dict with waypoint arrays, per-waypoint junction-locale label,
    anchors, roads (rid -> (jA, jB, midpoint xy)), lane_km."""
    wps = m.generate_waypoints(2.0)
    junc_center = {}
    lane_pts = defaultdict(list)   # (road, lane) -> [wp]
    for wp in wps:
        if wp.is_junction:
            j = wp.get_junction()
            if j is not None and j.id not in junc_center:
                bb = j.bounding_box.location; junc_center[j.id] = (bb.x, bb.y)
        else:
            lane_pts[(wp.road_id, wp.lane_id)].append(wp)
    # lane-km over all driving lanes (2 m spacing)
    lane_km = sum(2.0 * max(0, len(v) - 1) for v in lane_pts.values()) / 1000.0
    # representative lane per road for the road graph
    roads = {}
    for (rid, lid), pts in lane_pts.items():
        if rid not in roads or abs(lid) < abs(roads[rid][0]): roads[rid] = (lid, pts)
    def end_junction(wp, forward):
        nb = wp.next(3.0) if forward else wp.previous(3.0)
        for w in nb:
            if w.is_junction:
                j = w.get_junction()
                if j is not None: return j.id
        return None
    road_info = {}
    for rid, (lid, pts) in roads.items():
        # order along s
        pts = sorted(pts, key=lambda w: w.s)
        ja, jb = end_junction(pts[0], False), end_junction(pts[-1], True)
        if lid > 0:   # left lanes travel against s; ends swap
            ja, jb = jb, ja
        road_info[rid] = (ja, jb, pts)
    # label every waypoint (all lanes) by the road-half rule
    X, Y, LAB = [], [], []
    for (rid, lid), pts in lane_pts.items():
        ja, jb, _ = road_info[rid]
        if ja is None and jb is None: continue
        pts = sorted(pts, key=lambda w: w.s)
        s_mid = 0.5 * (pts[0].s + pts[-1].s)
        for wp in pts:
            lab = (ja if wp.s <= s_mid else jb) if (ja is not None and jb is not None) else (ja if ja is not None else jb)
            X.append(wp.transform.location.x); Y.append(wp.transform.location.y); LAB.append(lab)
    for wp in wps:
        if wp.is_junction:
            j = wp.get_junction()
            if j is None: continue
            X.append(wp.transform.location.x); Y.append(wp.transform.location.y); LAB.append(j.id)
    boundaries = {}   # rid -> (ja, jb, (x, y))
    for rid, (ja, jb, pts) in road_info.items():
        if ja is None or jb is None or ja == jb: continue
        mid = pts[len(pts) // 2].transform.location
        boundaries[rid] = (ja, jb, (mid.x, mid.y))
    return dict(X=np.array(X), Y=np.array(Y), LAB=np.array(LAB), anchors=junc_center,
                boundaries=boundaries, lane_km=lane_km, n_wp=len(wps))

def group_anchors(anchors, tile_m):
    """Fixed-size tiling: every junction anchor belongs to the tile_m x tile_m
    square containing it; a locale is the union of the junction locales whose
    anchors share a tile, so boundaries stay at inter-junction road midpoints
    (Q7 rule) and the locale SIZE is what is held, not the count."""
    ids = sorted(anchors)
    if tile_m <= 0: return {i: k for k, i in enumerate(ids)}
    x0 = min(anchors[i][0] for i in ids); y0 = min(anchors[i][1] for i in ids)
    tiles = {}
    for i in ids:
        key = (int((anchors[i][0] - x0) // tile_m), int((anchors[i][1] - y0) // tile_m))
        tiles.setdefault(key, len(tiles))
    return {i: tiles[(int((anchors[i][0] - x0) // tile_m), int((anchors[i][1] - y0) // tile_m))] for i in ids}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv"); ap.add_argument("out")
    ap.add_argument("--tiles", default="300", help="fixed tile size in m (the paper's locale size); 0 = per-junction, not used")
    ap.add_argument("--work", default=os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument("--max_rows", type=int, default=None)
    ap.add_argument("--pad_deg", type=float, default=0.002)
    a = ap.parse_args()
    t0 = time.time()
    header, tracks = parse_pneuma(a.csv, a.max_rows)
    types = defaultdict(int)
    for tr in tracks: types[tr[1]] += 1
    lats = np.concatenate([tr[3] for tr in tracks]); lons = np.concatenate([tr[4] for tr in tracks])
    bbox = (float(lats.min()) - a.pad_deg, float(lons.min()) - a.pad_deg, float(lats.max()) + a.pad_deg, float(lons.max()) + a.pad_deg)
    print(f"parsed {len(tracks)} tracks in {time.time()-t0:.0f}s; types {dict(types)}; bbox {bbox}", flush=True)
    osm_path = f"{a.work}/athens.osm"; xodr_path = f"{a.work}/athens.xodr"
    fetch_osm(bbox, osm_path); xodr = osm_to_xodr(osm_path, xodr_path)
    m = carla.Map("athens", xodr)
    proj = re.search(r"<geoReference><!\[CDATA\[(.*?)\]\]></geoReference>", xodr, re.S)
    proj_str = proj.group(1).strip() if proj else "+proj=tmerc +lat_0=%f +lon_0=%f +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs" % ((bbox[0]+bbox[2])/2, (bbox[1]+bbox[3])/2)
    tf = Transformer.from_crs("EPSG:4326", proj_str, always_xy=True)
    print(f"map ready in {time.time()-t0:.0f}s; proj: {proj_str[:80]}", flush=True)
    P = build_partition(m)
    tree = cKDTree(np.c_[P["X"], P["Y"]])
    print(f"partition: {len(P['anchors'])} junction anchors, {len(P['boundaries'])} boundaries, lane_km {P['lane_km']:.1f}, labelled wps {len(P['LAB'])}", flush=True)
    # trajectories -> CARLA frame at 5 Hz (and 25 Hz for the cadence check)
    T5, T25 = [], []
    d_all = []
    for tid, vt, t, lat, lon, spd in tracks:
        x, y = tf.transform(lon, lat); y = -np.asarray(y); x = np.asarray(x)
        d, idx = tree.query(np.c_[x, y]); d_all.append(d)
        k25 = np.round(t / 0.04).astype(np.int64)
        k5, (x5, y5, d5, i5) = subsample_to_grid(t, [x, y, d, idx])
        T5.append((tid, vt, k5, x5, y5, d5, i5)); T25.append((tid, vt, k25, d, idx))
    d_all = np.concatenate(d_all)
    align = {"median_m": float(np.median(d_all)), "p90_m": float(np.percentile(d_all, 90)), "frac_within_10m": float((d_all <= 10).mean())}
    print(f"alignment to nearest waypoint: {align}", flush=True)
    OFFNET = 15.0
    tiles = [int(v) for v in a.tiles.split(",")]
    nJ = len(P["anchors"])
    out = {"csv": os.path.basename(a.csv), "n_tracks": len(tracks), "types": dict(types), "bbox": bbox, "alignment": align,
           "junction_anchors": nJ, "boundaries_junction_level": len(P["boundaries"]), "lane_km": P["lane_km"], "levels": {}}
    # active tracks per cycle (tracked classes), independent of the level
    act = defaultdict(int)
    for tid, vt, k5, x5, y5, d5, i5 in T5:
        if vt not in TRACKED: continue
        for k, d in zip(k5, d5):
            if d <= OFFNET: act[int(k)] += 1
    actv = np.array(sorted(act.values()))
    out["active_tracks_per_cycle"] = {"median": float(np.median(actv)), "p95": float(np.percentile(actv, 95)), "max": int(actv.max()), "cycles": len(actv)}
    out["vehicles_per_lane_km_median"] = float(np.median(actv) / P["lane_km"]) if P["lane_km"] > 0 else None
    for tile in tiles:
        grp = group_anchors(P["anchors"], tile)
        lab_of_wp = np.array([grp.get(l, -1) for l in P["LAB"]])
        # boundaries at this level: road midpoints whose end anchors fall in different groups
        b_pts, b_pair = [], []
        for rid, (ja, jb, xy) in P["boundaries"].items():
            if grp.get(ja) != grp.get(jb): b_pts.append(xy); b_pair.append((grp.get(ja), grp.get(jb)))
        b_tree = cKDTree(np.array(b_pts)) if b_pts else None
        adj = defaultdict(set)
        for ga, gb in b_pair: adj[ga].add(gb); adj[gb].add(ga)
        deg = [len(v) for v in adj.values()]
        stats = {"tile_m": tile, "L": int(len(set(grp.values()))), "boundaries": len(b_pts), "neighbour_degree_mean": float(np.mean(deg)) if deg else 0.0,
                 "locale_area_km2_mean": None}
        for cls_name, cls in (("tracked", TRACKED), ("two_wheel", TWO_WHEEL)):
            cross5 = 0; tc5 = 0; nb = {20: 0, 30: 0, 50: 0}; cross25 = 0; tc25 = 0
            b_cross = defaultdict(int); b_seen = set()
            for (tid, vt, k5, x5, y5, d5, i5), (_, _, k25, d25, i25) in zip(T5, T25):
                if vt not in cls: continue
                on = d5 <= OFFNET
                labs = np.where(on, lab_of_wp[i5], -1)
                valid = labs >= 0
                tc5 += int(valid.sum())
                ch = (labs[1:] != labs[:-1]) & valid[1:] & valid[:-1]
                cross5 += int(ch.sum())
                if b_tree is not None and valid.any():
                    db, ib = b_tree.query(np.c_[x5[valid], y5[valid]])
                    for r in nb: nb[r] += int((db <= r).sum())
                    for j in set(ib[db <= 30].tolist()): b_seen.add(j)
                    for j in ib[np.r_[False, ch][valid]].tolist(): b_cross[j] += 1
                labs25 = np.where(d25 <= OFFNET, lab_of_wp[i25], -1); v25 = labs25 >= 0
                tc25 += int(v25.sum()); cross25 += int(((labs25[1:] != labs25[:-1]) & v25[1:] & v25[:-1]).sum())
            sec5 = tc5 * CYCLE_S; sec25 = tc25 * 0.04
            stats[cls_name] = {"track_cycles": tc5, "crossings_5hz": cross5, "crossing_rate_per_track_s_5hz": (cross5 / sec5) if sec5 else None,
                               "crossings_25hz": cross25, "crossing_rate_per_track_s_25hz": (cross25 / sec25) if sec25 else None,
                               "near_boundary_frac": {str(r): (nb[r] / tc5 if tc5 else None) for r in nb},
                               "boundaries_observed_frac": (len(b_seen) / len(b_pts)) if b_pts else None,
                               "boundaries_with_crossings_frac": (len(b_cross) / len(b_pts)) if b_pts else None,
                               "crossings_per_boundary_per_min_median": float(np.median([c / (len(actv) * CYCLE_S / 60.0) for c in b_cross.values()])) if b_cross else 0.0}
        out["levels"][f"tile{tile}"] = stats
        print(f"tile={tile} L={stats['L']}: boundaries {stats['boundaries']} degree {stats['neighbour_degree_mean']:.2f} | tracked crossing/track/s 5Hz {stats['tracked']['crossing_rate_per_track_s_5hz']:.4f} 25Hz {stats['tracked']['crossing_rate_per_track_s_25hz']:.4f} near30 {stats['tracked']['near_boundary_frac']['30']:.3f} observed {stats['tracked']['boundaries_observed_frac']:.2f}", flush=True)
    json.dump(out, open(a.out, "w"), indent=1)
    print(f"written {a.out} in {time.time()-t0:.0f}s")

if __name__ == "__main__":
    main()
