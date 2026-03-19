import json, os, glob
import numpy as np

sweep_dir = "experiment_results/openscenario_3_edge_late_fusion/20260302_230125"
files = sorted(glob.glob(os.path.join(sweep_dir, "**", "simulation_metrics.json"), recursive=True))
files = [f for f in files if "/evaluation_output/" not in f]

print(f"Total metrics files found: {len(files)}")

def fmt(val, fmt_str="{:.1f}"):
    if val in ("?", None):
        return "?"
    try:
        return fmt_str.format(float(val))
    except (ValueError, TypeError):
        return str(val)

print("=" * 80)
print("N=1, latency=0 runs - detailed brake attribution analysis")
print("=" * 80)

for filepath in files:
    with open(filepath) as f:
        d = json.load(f)

    lat = d.get("config_latency_ms", -1)
    if lat is None:
        lat = -1
    lat = int(lat)
    mgr = d.get("config_manager_type", "")
    anch = d.get("config_anchoring", False)

    if lat != 0 or "late_fusion" not in mgr:
        continue

    rel = os.path.relpath(filepath, sweep_dir)
    anch_str = "on" if anch else "off"

    print(f"\n--- {rel} (late_fusion anch={anch_str} lat=0) ---")
    print(f"  s_coll={d.get('s_coll')}, s_ghost={d.get('s_ghost')}, s_fp={d.get('s_fp')}, "
          f"s_prog={d.get('s_prog')}, s_op={d.get('s_op')}")
    print(f"  total_ghost_brake_gt={d.get('total_ghost_brake_gt')}, "
          f"total_other_fp_gt={d.get('total_other_fp_gt')}, "
          f"total_tp_gt={d.get('total_tp_gt')}")
    print(f"  avg_speed_mps={d.get('overall_avg_speed_mps', 0):.2f}, "
          f"collision_count={d.get('collision_count')}")

    vehicles = d.get("vehicles", {})
    for vid, veh in vehicles.items():
        ba = veh.get("brake_attributions", [])
        total = veh.get("total_brake_events", 0)
        ghost_gt = veh.get("ghost_brake_gt", 0)
        fp_gt = veh.get("other_fp_gt", 0)
        tp_gt = veh.get("true_positive_gt", 0)

        if total == 0:
            continue

        print(f"  Vehicle {vid}: total_brakes={total}, ghost_gt={ghost_gt}, fp_gt={fp_gt}, tp_gt={tp_gt}")

        for b in ba:
            cls = b.get("gt_brake_class", "?")
            cid = b.get("trigger_carla_id", "?")
            ghost_r = b.get("ghost_reason", "?")
            ego_dist = b.get("ego_dist_m", "?")
            obs_speed = b.get("obstacle_speed", "?")
            ttc = b.get("ttc", "?")
            gt_matched = b.get("gt_matched_actor_id", "?")
            gt_dist = b.get("gt_match_dist_m", "?")
            gt_spd = b.get("gt_actor_speed", "?")
            gt_prov = b.get("gt_provenance", "?")
            gt_dca = b.get("gt_dca_m", "?")
            src = b.get("source_tick", "?")
            trig = b.get("trigger_tick", "?")

            ghost_r_s = str(ghost_r) if ghost_r else "?"

            print(f"    {str(cls):15s} cid={str(cid):>6} ghost_r={ghost_r_s:15s} "
                  f"ego_d={fmt(ego_dist):>6}m obs_spd={fmt(obs_speed):>5} ttc={fmt(ttc, '{:.2f}'):>5}s "
                  f"gt_id={gt_matched} gt_d={fmt(gt_dist):>5}m gt_spd={fmt(gt_spd, '{:.2f}')} "
                  f"prov={gt_prov} dca={fmt(gt_dca, '{:.2f}')}")

print("\n" + "=" * 80)
print("N=1, latency=0, oracle runs for comparison")
print("=" * 80)

for filepath in files:
    with open(filepath) as f:
        d = json.load(f)

    lat = d.get("config_latency_ms", -1)
    if lat is None:
        lat = -1
    lat = int(lat)
    mgr = d.get("config_manager_type", "")
    anch = d.get("config_anchoring", False)

    if lat != 0 or "oracle" not in mgr:
        continue

    rel = os.path.relpath(filepath, sweep_dir)
    anch_str = "on" if anch else "off"

    print(f"\n--- {rel} oracle anch={anch_str} ---")
    print(f"  s_coll={d.get('s_coll')}, s_ghost={d.get('s_ghost')}, s_fp={d.get('s_fp')}, "
          f"s_prog={d.get('s_prog')}, s_op={d.get('s_op')}")
    print(f"  total_ghost_brake_gt={d.get('total_ghost_brake_gt')}, "
          f"total_other_fp_gt={d.get('total_other_fp_gt')}, "
          f"total_tp_gt={d.get('total_tp_gt')}")

    vehicles = d.get("vehicles", {})
    for vid, veh in vehicles.items():
        ba = veh.get("brake_attributions", [])
        total = veh.get("total_brake_events", 0)
        ghost_gt = veh.get("ghost_brake_gt", 0)
        fp_gt = veh.get("other_fp_gt", 0)
        tp_gt = veh.get("true_positive_gt", 0)

        if total == 0:
            print(f"  Vehicle {vid}: total_brakes=0 (clean)")
            continue

        print(f"  Vehicle {vid}: total_brakes={total}, ghost_gt={ghost_gt}, fp_gt={fp_gt}, tp_gt={tp_gt}")
        for b in ba:
            cls = b.get("gt_brake_class", "?")
            cid = b.get("trigger_carla_id", "?")
            ego_dist = b.get("ego_dist_m", "?")
            gt_prov = b.get("gt_provenance", "?")
            gt_dca = b.get("gt_dca_m", "?")
            print(f"    {str(cls):15s} cid={str(cid):>6} ego_d={fmt(ego_dist):>6}m "
                  f"prov={gt_prov} dca={fmt(gt_dca, '{:.2f}')}")

# Also show vips_temporal at lat=0 for comparison
print("\n" + "=" * 80)
print("N=1, latency=0, vips_temporal runs for comparison")
print("=" * 80)

for filepath in files:
    with open(filepath) as f:
        d = json.load(f)

    lat = d.get("config_latency_ms", -1)
    if lat is None:
        lat = -1
    lat = int(lat)
    mgr = d.get("config_manager_type", "")
    anch = d.get("config_anchoring", False)

    if lat != 0 or "vips" not in mgr:
        continue

    rel = os.path.relpath(filepath, sweep_dir)
    anch_str = "on" if anch else "off"

    print(f"\n--- {rel} vips_temporal anch={anch_str} ---")
    print(f"  s_coll={d.get('s_coll')}, s_ghost={d.get('s_ghost')}, s_fp={d.get('s_fp')}, "
          f"s_prog={d.get('s_prog')}, s_op={d.get('s_op')}")
    print(f"  total_ghost_brake_gt={d.get('total_ghost_brake_gt')}, "
          f"total_other_fp_gt={d.get('total_other_fp_gt')}, "
          f"total_tp_gt={d.get('total_tp_gt')}")
    print(f"  avg_speed_mps={d.get('overall_avg_speed_mps', 0):.2f}")

    vehicles = d.get("vehicles", {})
    for vid, veh in vehicles.items():
        ba = veh.get("brake_attributions", [])
        total = veh.get("total_brake_events", 0)
        if total > 0:
            print(f"  Vehicle {vid}: total_brakes={total}")
            for b in ba:
                cls = b.get("gt_brake_class", "?")
                cid = b.get("trigger_carla_id", "?")
                ghost_r = b.get("ghost_reason", "?")
                ego_dist = b.get("ego_dist_m", "?")
                gt_prov = b.get("gt_provenance", "?")
                gt_dca = b.get("gt_dca_m", "?")
                gt_spd = b.get("gt_actor_speed", "?")
                ghost_r_s = str(ghost_r) if ghost_r else "?"
                print(f"    {str(cls):15s} cid={str(cid):>6} ghost_r={ghost_r_s:15s} "
                      f"ego_d={fmt(ego_dist):>6}m gt_spd={fmt(gt_spd, '{:.2f}')} "
                      f"prov={gt_prov} dca={fmt(gt_dca, '{:.2f}')}")
