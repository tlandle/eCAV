# scenario_1_branch_{k}.xml + openscenario_1_branch_{k}.yaml from branching_sites.json
# and the flow templates. All CAV configs sit under edge 0 'vehicles' (the
# VehicleManager reads edge_list[0]['vehicles'][vehicle_index]); each edge names
# its members by global index in 'member_vehicles' (sim_api overlay).
import copy, json, os, sys, yaml
W = os.path.dirname(os.path.abspath(__file__))
sites = json.load(open(f"{W}/branching_sites.json"))
tmpl = yaml.safe_load(open(f"{W}/flow_template.yaml"))
NPC_MODEL, CAV_MODEL, BLOCKER_MODEL = "vehicle.nissan.seat.leon", "vehicle.nissan.patrol", "vehicle.carlamotors.carlacola"
for key, site in sites.items():
    town = site["town"]; appr = [l for l in site["locales"] if l["id"] == "locale_approach"][0]
    branches = [l for l in site["locales"] if l["id"].startswith("locale_branch_")]
    # ---- XML: egos in branch order, then NPC (hidden), then blockers
    x = [f'<?xml version="1.0"?>', "<scenarios>", f'    <scenario name="Scenario_1" type="Scenario_1" town="{town}">']
    for b in branches:
        cx, cy, cyaw = b["cav"]; x.append(f'        <ego_vehicle x="{cx:.2f}" y="{cy:.2f}" z="0.3" yaw="{cyaw:.1f}" model="{CAV_MODEL}" />  <!-- recipient CAV, {b["id"]} -->')
    sx, sy, syaw = appr["npc_spawn"]; x.append(f'        <other_actor x="{sx:.2f}" y="{sy:.2f}" z="-500" yaw="{syaw:.1f}" model="{NPC_MODEL}" />  <!-- NPC (scripted road user), hidden until released -->')
    for b in branches:
        bx, by, byaw = b["blocker"]; x.append(f'        <other_actor x="{bx:.2f}" y="{by:.2f}" z="0.3" yaw="{byaw:.1f}" model="{BLOCKER_MODEL}" />  <!-- blocker, {b["id"]} -->')
        for ex, ey, eyaw in b.get("extra_blockers", []):
            x.append(f'        <other_actor x="{ex:.2f}" y="{ey:.2f}" z="0.3" yaw="{eyaw:.1f}" model="{BLOCKER_MODEL}" />  <!-- outer-lane blocker, {b["id"]} -->')
    x += ['        <weather cloudiness="0" precipitation="0" precipitation_deposits="0" wind_intensity="0" sun_azimuth_angle="0" sun_altitude_angle="75" />', "    </scenario>", "</scenarios>"]
    open(f"{W}/scenario_1_branch_{key}.xml", "w").write("\n".join(x) + "\n")
    n_actors = len(branches) + 1 + len(branches) + sum(len(b.get("extra_blockers", [])) for b in branches)
    # ---- YAML
    y = copy.deepcopy(tmpl)
    y["scenario_name"] = f"openscenario_1_branch_{key}"
    y["description"] = f"Branching junction {key}: {town} junction {site['junction']}, approach road {site['approach_road']}, {len(branches)} branch locales."
    sr = y["scenario_runner"]
    sr["town"] = town if str(sr.get("town", "")).startswith("Town") else town.lower()
    sr["configFile"] = f"ecav/scenario_testing/scenarios/scenario_1_branch_{key}.xml"
    sr["additionalScenario"] = "ecav/scenario_testing/scenarios/scenario_1_branch.py"
    sr["num_actors"] = n_actors
    e0 = copy.deepcopy(y["scenario"]["edge_list"][0]); vt = copy.deepcopy(e0["vehicles"][0])
    # perception active (campaign decision): non-connected traffic from WorldFusion, no GT injection
    e0["gt_detection_injection"]["enabled"] = False
    def mk_edge(lid, rsu_xy, polygon, members, extra):
        e = copy.deepcopy(e0)
        e["world_anchor"] = [round(rsu_xy[0], 1), round(rsu_xy[1], 1), 0.0, 0.0, 0.0, 0.0]
        e["rsus"][0]["spawn_position"] = [round(rsu_xy[0], 1), round(rsu_xy[1], 1), 3.0]; e["rsus"][0]["name"] = f"rsu_{lid}"
        e["member_vehicles"] = members
        e["locale"] = {"id": lid, "polygon": [[round(a, 1), round(b, 1)] for a, b in polygon]}; e["locale"].update(extra)
        return e
    vehicles = []
    for i, b in enumerate(branches):
        v = copy.deepcopy(vt); v["name"] = f"cav_{b['id'][14:]}"; v["destination"] = [round(c, 1) for c in site["cav_destination"]]
        vehicles.append(v)
    edges = [mk_edge("locale_approach", appr["rsu"], appr["polygon"], [], {"neighbors": [b["id"] for b in branches]})]
    edges[0]["vehicles"] = vehicles
    for i, b in enumerate(branches):
        edges.append(mk_edge(b["id"], b["rsu"], b["polygon"], [i], {"neighbors": ["locale_approach"], "conflict_point": [round(c, 1) for c in b["conflict_point"]], "conflict_type": b["conflict_type"]}))
        edges[-1]["vehicles"] = []
    # RSU ids must be unique across edges (sim_api keys rsu_managers by id)
    for i, e in enumerate(edges): e["rsus"][0]["id"] = -(i + 1)
    y["scenario"]["edge_list"] = edges
    open(f"{W}/openscenario_1_branch_{key}.yaml", "w").write(yaml.safe_dump(y, sort_keys=False, width=120))
    print(f"{key}: {town} edges={len(edges)} egos={len(branches)} actors={n_actors} town_field={sr['town']} | " + ", ".join(f"{b['id'][14:]}:{b['conflict_type']}" for b in branches))
