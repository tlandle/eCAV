# Candidate district for the metropolitan replay: fetch an OSM extract (surface
# streets only), convert to OpenDRIVE with carla.Osm2Odr (client side), load it
# with the client-side carla.Map and survey junction degree + inter-junction
# road lengths (the Q7 separation statistic). Usage: district_survey.py NAME S W N E
import sys, os, math, json, time, urllib.request, urllib.parse
import carla
from collections import defaultdict
name, S, W, N, E = sys.argv[1], *map(float, sys.argv[2:6])
OUT = os.path.dirname(os.path.abspath(__file__))
osm_path = f"{OUT}/{name}.osm"
if not os.path.exists(osm_path):
    q = f'''[out:xml][timeout:180];
(way["highway"~"^(primary|secondary|tertiary|residential|unclassified|living_street|primary_link|secondary_link|tertiary_link)$"]({S},{W},{N},{E}););
(._;>;);
out body;'''
    data = urllib.parse.urlencode({"data": q}).encode()
    t0 = time.time()
    with urllib.request.urlopen(urllib.request.Request("https://overpass-api.de/api/interpreter", data=data, headers={"User-Agent": "ecav-metro-survey"}), timeout=300) as r:
        open(osm_path, "wb").write(r.read())
    print(f"fetched {osm_path} {os.path.getsize(osm_path)/1e6:.1f} MB in {time.time()-t0:.0f}s")
osm = open(osm_path, encoding="utf-8").read()
st = carla.Osm2OdrSettings()
st.use_offsets = False; st.generate_traffic_lights = False; st.center_map = True
st.default_lane_width = 3.5
t0 = time.time(); xodr = carla.Osm2Odr.convert(osm, st); print(f"Osm2Odr: {len(xodr)/1e6:.1f} MB xodr in {time.time()-t0:.0f}s")
open(f"{OUT}/{name}.xodr", "w").write(xodr)
m = carla.Map(name, xodr)
wps = m.generate_waypoints(2.0)
print("waypoints", len(wps))
# roads outside junctions: length per road (one lane per road)
road_pts = defaultdict(list); junc_of_wp = {}
for wp in wps:
    if wp.is_junction: continue
    road_pts[wp.road_id].append(wp)
# arms per junction: roads whose lane continues into a junction (next or previous 2 m)
arms = defaultdict(dict)   # jid -> road_id -> heading deg
def norm(a): return (a + 180.0) % 360.0 - 180.0
for rid, pts in road_pts.items():
    for wp in pts:
        for nb, sign in ((wp.next(2.0), 1), (wp.previous(2.0), -1)):
            for w2 in nb:
                if w2.is_junction:
                    j = w2.get_junction()
                    if j is None: continue
                    h = wp.transform.rotation.yaw if sign == 1 else wp.transform.rotation.yaw + 180.0
                    arms[j.id].setdefault(rid, norm(h))
def cluster(hs, tol=25.0):
    cl = []
    for h in sorted(hs):
        for c in cl:
            if abs(norm(h - c[0])) <= tol: c.append(h); break
        else: cl.append([h])
    return len(cl)
deg = {jid: cluster(list(v.values())) for jid, v in arms.items()}
hist = defaultdict(int)
for d in deg.values(): hist[d] += 1
road_len = {rid: 2.0 * (len(pts) - 1) for rid, pts in road_pts.items() if len(pts) > 1}
lens = sorted(road_len.values()); med = lens[len(lens)//2] if lens else float('nan')
res = {"name": name, "bbox": [S, W, N, E], "junctions": len(deg), "degree_hist": dict(sorted(hist.items())),
       "roads": len(road_len), "road_len_median_m": med, "separation_median_m": med / 2.0,
       "roads_ge_60m_frac": sum(l >= 60 for l in lens) / max(1, len(lens))}
print(json.dumps(res, indent=1)); json.dump(res, open(f"{OUT}/{name}_survey.json", "w"), indent=1)
