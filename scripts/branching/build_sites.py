# Branching sites: locale polygons + actor placements from the OpenDRIVE via the
# client-side carla.Map (no server). Output: branching/branching_sites.json.
#   approach locale  = road rectangle (far end -> junction) UNION the junction box,
#                      so the exit test fires at the branch mouths after the turn
#   branch locale    = 60 m rectangle from the branch mouth outward (long enough
#                      to contain MTR mode endpoints at the trigger tick)
#   placements (path distance along the lane, via waypoint.previous/next):
#     NPC spawn: approach inward lane, NPC_M back from the mouth
#     blocker:   branch inward lane, BLOCKER_M back from the mouth (stopped)
#     CAV:       branch inward lane, BLOCKER_M + CAV_GAP_M back (may lie upstream
#                of the branch's far junction; the route planner brings it in)
#     conflict:  branch OUTWARD lane, CONFLICT_M past the mouth (the lane the
#                CAV overtakes into and the NPC arrives in)
#     RSU:       RSU_LAT_M beside the conflict point, z=3
#   lane rules: two-way single lane -> standard; two-way dual -> inner inward lane
#   (+ second blocker in the outer inward lane); one-way outward -> CAV+blocker in
#   the outer outward lane (-2), NPC keeps its natural lane -1 (same-direction
#   overtake). Every non-standard branch is flagged conflict_type != "oncoming".
import json, math, sys, os
import carla
from shapely.geometry import Polygon, LineString, MultiPoint
from shapely.ops import unary_union
W = os.path.dirname(os.path.abspath(__file__))
XODR = "/home/atlas/carla-0.9.15/CarlaUE4/Content/Carla/Maps/OpenDrive"
SITES = {"k2": ("Town01", "87", "24"), "k3": ("Town04", "1452", "18"), "k4": ("Town03", "861", "62")}
CONFLICT_M, BLOCKER_M, CAV_GAP_M, NPC_M = 15.0, 19.0, 60.0, 65.0
PREFER_FEEDER = {"k4": "20"}   # k4: road 62 is 37 m; the run-in continues through T-junction 1820 onto road 20 (S)
_PREF = None
BRANCH_LEN, HALF_W, RSU_LAT_M, BOX_PAD, RSU_APPR_M = 60.0, 8.0, 10.0, 3.0, 30.0
sys.path.insert(0, W); from junction_sites import arms  # xodr arm survey (ids, lengths)

def yaw_of(wp): return wp.transform.rotation.yaw
def xy(wp): return (round(wp.transform.location.x, 2), round(wp.transform.location.y, 2))
def mouth_waypoints(m, rid, jc):
    """All lanes of road rid at the junction end: {lane_id: (wp, inward_bool)}."""
    best = {}
    for wp in m.generate_waypoints(1.0):
        if str(wp.road_id) != rid or wp.lane_type != carla.LaneType.Driving: continue
        d = math.hypot(wp.transform.location.x - jc[0], wp.transform.location.y - jc[1])
        if wp.lane_id not in best or d < best[wp.lane_id][1]: best[wp.lane_id] = (wp, d)
    out = {}
    for lid, (wp, d) in best.items():
        f = wp.transform.get_forward_vector()
        tow = (jc[0] - wp.transform.location.x, jc[1] - wp.transform.location.y)
        out[lid] = (wp, (f.x * tow[0] + f.y * tow[1]) > 0)
    return out
def _straightest(cands, cur):
    """Prefer the candidate whose heading is closest to the current one (straight
    through junctions), non-junction roads first on ties; a per-site preferred
    feeder road wins outright."""
    if _PREF is not None and len(cands) > 1:
        # look through the junction: pick the candidate whose backward chain
        # reaches the preferred feeder road within 60 m
        def reaches(w):
            cur, left = w, 60.0
            while left > 0:
                if str(cur.road_id) == _PREF: return True
                prev = cur.previous(3.0)
                if not prev: return False
                cur = prev[0]; left -= 3.0
            return False
        for w in cands:
            if reaches(w): return w
    cy = cur.transform.rotation.yaw
    def key(w):
        d = abs((w.transform.rotation.yaw - cy + 180.0) % 360.0 - 180.0)
        return (round(d / 5.0), 1 if w.is_junction else 0)
    return sorted(cands, key=key)[0]
def back(wp, d):
    """Waypoint d metres BEHIND wp along its lane (previous chain across junctions)."""
    cur, left = wp, d
    while left > 0:
        step = min(left, 5.0); prev = cur.previous(step)
        if not prev: return cur, d - left
        # prefer staying on a non-junction road, else the first
        cur = _straightest(prev, cur); left -= step
    return cur, d
def fwd(wp, d):
    cur, left = wp, d
    while left > 0:
        step = min(left, 5.0); nxt = cur.next(step)
        if not nxt: return cur, d - left
        cur = _straightest(nxt, cur); left -= step
    return cur, d
def chain_back(wp, d, step=2.0):
    pts=[(wp.transform.location.x, wp.transform.location.y)]; cur=wp; left=d
    while left > 0:
        prev=cur.previous(min(step,left))
        if not prev: break
        cur=_straightest(prev, cur); pts.append((cur.transform.location.x, cur.transform.location.y)); left-=step
    return pts
def chain_fwd(wp, d, step=2.0):
    pts=[(wp.transform.location.x, wp.transform.location.y)]; cur=wp; left=d
    while left > 0:
        nxt=cur.next(min(step,left))
        if not nxt: break
        cur=_straightest(nxt, cur); pts.append((cur.transform.location.x, cur.transform.location.y)); left-=step
    return pts
def corridor(pts, halfw): return LineString(pts).buffer(halfw, cap_style=2, join_style=2)
def ring(poly): return [[round(a,1), round(b,1)] for a,b in list(poly.exterior.coords)[:-1]]
def rect(x, y, hdg, length, halfw):
    c, s = math.cos(hdg), math.sin(hdg); nx, ny = -s, c
    return [(x + nx * halfw, y + ny * halfw), (x - nx * halfw, y - ny * halfw),
            (x - nx * halfw + c * length, y - ny * halfw + s * length), (x + nx * halfw + c * length, y + ny * halfw + s * length)]
def lateral(pt, hdg_deg, d): h = math.radians(hdg_deg) + math.pi / 2; return (pt[0] + math.cos(h) * d, pt[1] + math.sin(h) * d)

out = {}
for key, (town, jid, appr) in SITES.items():
    m = carla.Map(town, open(f"{XODR}/{town}.xodr").read())
    _PREF = PREFER_FEEDER.get(key)
    A = arms(town, jid)                       # (rid, L, deg_xodr, jx, jy, drv, wsum)
    mouths = {rid: (jx, -jy) for rid, L, deg, jx, jy, drv, wsum in A}   # CARLA frame
    jc = (sum(p[0] for p in mouths.values()) / len(mouths), sum(p[1] for p in mouths.values()) / len(mouths))
    boxr = max(math.hypot(p[0] - jc[0], p[1] - jc[1]) for p in mouths.values()) + BOX_PAD
    jbox = MultiPoint(list(mouths.values())).convex_hull.buffer(BOX_PAD, join_style=2)
    site = {"town": town, "junction": jid, "approach_road": appr, "junction_center": [round(jc[0], 1), round(jc[1], 1)],
            "junction_box": ring(jbox), "locales": []}
    for rid, L, deg, jx, jy, drv, wsum in A:
        H = -deg                                # outward heading, CARLA degrees
        lanes = mouth_waypoints(m, rid, jc)
        inward = {l: w for l, (w, i) in lanes.items() if i}; outward = {l: w for l, (w, i) in lanes.items() if not i}
        if rid == appr:
            far = (mouths[rid][0] + math.cos(math.radians(H)) * min(L, NPC_M + 5), mouths[rid][1] + math.sin(math.radians(H)) * min(L, NPC_M + 5))
            # inward lane = the one adjacent to the centreline if several (inner lane); NPC keeps it
            lin = sorted(inward, key=abs)
            if not lin: raise SystemExit(f"{key}: approach {rid} has no inward lane")
            wp_in = inward[lin[0]]
            spawn_wp, got = back(wp_in, NPC_M)   # may run through the upstream junction
            lane_pts = chain_back(wp_in, NPC_M + 8.0)[::-1]      # far -> mouth along the inward lane (through the upstream junction)
            poly = unary_union([corridor(lane_pts, HALF_W), jbox])
            rsu_wp, _ = back(wp_in, RSU_APPR_M)
            lout_a = sorted(outward, key=abs)
            if not lout_a: raise SystemExit(f"{key}: approach {rid} has no outward lane")
            dest_wp, _ = fwd(outward[lout_a[0]], min(60.0, L - 5))
            site["cav_destination"] = [*xy(dest_wp), 0.5]
            site["locales"].append({"id": "locale_approach", "road": rid, "length_m": round(L, 1),
                "polygon": ring(poly), "rsu": [round(v, 1) for v in lateral(xy(rsu_wp), yaw_of(rsu_wp) + 180.0, RSU_LAT_M)],
                "lanes_inward": [str(l) for l in lin], "npc_lane": str(lin[0]),
                "npc_spawn": [*xy(spawn_wp), round(yaw_of(spawn_wp), 1)], "npc_back_m": round(got, 1), "npc_spawn_road": str(spawn_wp.road_id),
                "approach_yaw_at_mouth": round(yaw_of(wp_in), 1), "npc_spawn_yaw": round(yaw_of(spawn_wp), 1),
                "mouth": [round(mouths[rid][0], 1), round(mouths[rid][1], 1)]})
        else:
            lin, lout = sorted(inward, key=abs), sorted(outward, key=abs)
            if not lout: raise SystemExit(f"{key}: branch {rid} has no outward lane (not a destination)")
            poly = corridor(chain_fwd(outward[lout[0]], BRANCH_LEN), HALF_W)
            entry = {"id": f"locale_branch_{rid}", "road": rid, "branch_len_m": round(L, 1), "outward_hdg_deg": round(H, 1),
                     "polygon": ring(poly), "lanes_inward": [str(l) for l in lin], "lanes_outward": [str(l) for l in lout],
                     "mouth": [round(mouths[rid][0], 1), round(mouths[rid][1], 1)]}
            if lin and lout:
                # two-way: CAV+blocker in the INNER inward lane; conflict in the INNER outward lane
                wi, wo = inward[lin[0]], outward[lout[0]]
                entry["conflict_type"] = "oncoming" if (len(lin) == 1 and len(lout) == 1) else "oncoming_dual"
                extra_blockers = [back(inward[l], BLOCKER_M)[0] for l in lin[1:]]  # block the outer inward lanes too
            elif lout and not lin:
                # one-way outward: CAV+blocker in the OUTER outward lane, NPC keeps the inner lane
                wi, wo = outward[lout[-1]], outward[lout[0]]
                entry["conflict_type"] = "same_direction"; extra_blockers = []
                # CAV drives outward here: it starts near the mouth, blocker ahead of it
                bl, _ = fwd(wi, BLOCKER_M); cav, _ = fwd(wi, max(2.0, BLOCKER_M - 12.0))
                cp, _ = fwd(wo, CONFLICT_M)
                re_wp, _ = fwd(wo, min(L - 3.0, 45.0)); entry["npc_route_end"] = [*xy(re_wp), 0.5]
                entry.update({"blocker": [*xy(bl), round(yaw_of(bl), 1)], "cav": [*xy(cav), round(yaw_of(cav), 1)], "cav_back_m": 0.0, "cav_road": str(cav.road_id),
                              "conflict_point": [*xy(cp)], "rsu": [round(v, 1) for v in lateral(xy(cp), H, RSU_LAT_M)], "extra_blockers": []})
                site["locales"].append(entry); continue
            else:
                raise SystemExit(f"{key}: branch {rid} has no outward lane (not a destination)")
            bl, _ = back(wi, BLOCKER_M); cav, got = back(wi, BLOCKER_M + CAV_GAP_M); cp, _ = fwd(wo, CONFLICT_M)
            re_wp, _ = fwd(wo, min(L - 3.0, 45.0)); entry["npc_route_end"] = [*xy(re_wp), 0.5]
            entry.update({"blocker": [*xy(bl), round(yaw_of(bl), 1)], "cav": [*xy(cav), round(yaw_of(cav), 1)], "cav_back_m": round(got, 1),
                          "cav_road": str(cav.road_id), "conflict_point": [*xy(cp)], "rsu": [round(v, 1) for v in lateral(xy(cp), H, RSU_LAT_M)],
                          "extra_blockers": [[*xy(b), round(yaw_of(b), 1)] for b in extra_blockers]})
            site["locales"].append(entry)
    # NPC approach lane per destination (which approach lane connects to each branch)
    import xml.etree.ElementTree as ET
    root = ET.parse(f"{XODR}/{town}.xodr").getroot(); roads = {r.get("id"): r for r in root.findall("road")}
    j = [x for x in root.findall("junction") if x.get("id") == jid][0]
    lane_by_dst = {}
    for c in j.findall("connection"):
        if c.get("incomingRoad") != appr: continue
        cr = roads[c.get("connectingRoad")]; lk = cr.find("link")
        dsts = [e.get("elementId") for e in lk if e.get("elementType") == "road" and e.get("elementId") != appr]
        for d in dsts:
            for ll in c.findall("laneLink"): lane_by_dst.setdefault(d, set()).add(ll.get("from"))
    site["npc_lane_by_branch"] = {}
    for l in site["locales"]:
        if l["id"].startswith("locale_branch_"):
            cands = lane_by_dst.get(l["road"], set())
            # prefer the lane reaching the most destinations (fewest tells)
            best = max(cands, key=lambda ln: sum(ln in v for v in lane_by_dst.values())) if cands else None
            site["npc_lane_by_branch"][l["id"]] = best
    out[key] = site
    print(f"\n== {key}: {town} j{jid} approach road {appr}, centre {site['junction_center']}, box r={boxr:.1f} ==")
    for l in site["locales"]:
        if l["id"] == "locale_approach":
            print(f"  approach road {l['road']} ({l['length_m']} m) npc_lane {l['npc_lane']} spawn {l['npc_spawn']} on road {l['npc_spawn_road']} back {l['npc_back_m']} m (yaw at mouth {l['approach_yaw_at_mouth']}, at spawn {l['npc_spawn_yaw']}); rsu {l['rsu']}; poly {len(l['polygon'])} pts; npc lane by branch {site['npc_lane_by_branch']}")
        else:
            print(f"  {l['id']:18s} road {l['road']:>3s} L={l['branch_len_m']:5.1f} hdg {l['outward_hdg_deg']:7.1f} type={l['conflict_type']:14s} cav {l['cav']} (road {l.get('cav_road','?')}, back {l['cav_back_m']}) blocker {l['blocker']} +{len(l['extra_blockers'])} conflict {l['conflict_point']}")
# overlap check between locales at each site
for key, site in out.items():
    polys = {l["id"]: Polygon(l["polygon"]) for l in site["locales"]}
    ids = list(polys)
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a = polys[ids[i]].intersection(polys[ids[j]]).area
            if a > 1.0: print(f"  OVERLAP {key}: {ids[i]} x {ids[j]} = {a:.1f} m^2")
json.dump(out, open(f"{W}/branching_sites.json", "w"), indent=1)
print("\nwritten", f"{W}/branching_sites.json")
