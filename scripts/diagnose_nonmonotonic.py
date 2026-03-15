#!/usr/bin/env python3
"""
Diagnostic analysis for the non-monotonic collision pattern.

Reads simulation_metrics.json files from a sweep directory and produces:
1. Summary table: per-config collision rate, brake counts, AoI
2. Per-run brake attribution details for the collision zone (lat=100-400)
3. Confound A check: edge prediction acceptance (if available)
4. Confound B check: collision actor identity vs brake-triggering actor

Usage:
    python scripts/diagnose_nonmonotonic.py <sweep_dir> [--anch-only] [--csv out.csv]
"""
import json
import glob
import os
import sys
import argparse
from collections import defaultdict

DT_MS = 50.0  # ms per tick


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('sweep_dir', help='Sweep result directory')
    p.add_argument('--anch-only', action='store_true',
                   help='Only show anchoring=ON runs')
    p.add_argument('--csv', default=None, help='Write CSV output')
    p.add_argument('--latency-range', nargs=2, type=float, default=[0, 700],
                   metavar=('MIN', 'MAX'), help='Latency range to show (ms)')
    return p.parse_args()


def load_runs(sweep_dir):
    """Load all simulation_metrics.json files, excluding evaluation_output copies."""
    pattern = os.path.join(sweep_dir, '**', 'simulation_metrics.json')
    files = glob.glob(pattern, recursive=True)
    files = [f for f in files if 'evaluation_output' not in f]

    runs = []
    for f in sorted(files):
        with open(f) as fh:
            d = json.load(fh)

        run_name = f.split('/')[-2]  # e.g. "run_1"
        lat = d.get('config_latency_ms', 0.0)
        mgr = d.get('config_manager_type', 'unknown')
        anch = d.get('config_anchoring', False)
        s_coll = d.get('s_coll', None)
        s_ghost = d.get('s_ghost', None)
        s_fp = d.get('s_fp', None)
        s_prog = d.get('s_prog', None)
        s_op = d.get('s_op', None)

        # Aggregate brake stats from all vehicles
        all_ba = []
        col_actor_id = None
        col_frame = None
        edge_stats = {}
        for vid, vdata in d.get('vehicles', {}).items():
            ba = vdata.get('brake_attributions', [])
            all_ba.extend(ba)

            safety = vdata.get('safety', {})
            if safety.get('collision_count', 0) > 0:
                col_actor_id = safety.get('collision_other_actor_id')
                ch = safety.get('collision_history', [])
                if ch:
                    entry = ch[0]
                    if isinstance(entry, dict):
                        col_frame = entry.get('frame')
                        col_actor_id = entry.get('other_actor_id', col_actor_id)
                    elif isinstance(entry, (list, tuple)):
                        col_frame = entry[0]

            # Edge acceptance counters (Confound A)
            for k in ['edge_ticks_total', 'edge_ticks_with_preds',
                       'edge_ticks_with_collision', 'edge_preds_received_total',
                       'edge_preds_ego_filtered', 'edge_collisions_total']:
                if k in vdata:
                    edge_stats[k] = edge_stats.get(k, 0) + vdata[k]

        # AoI from edge profiler
        aoi_p99 = None
        for eid, edata in d.get('edges', {}).items():
            aoi_p99 = edata.get('aoi_p99_ticks')
            break

        # First brake details
        first_ba = all_ba[0] if all_ba else None

        runs.append({
            'path': f,
            'run': run_name,
            'lat_ms': lat,
            'mgr': mgr,
            'anch': anch,
            's_coll': s_coll,
            's_ghost': s_ghost,
            's_fp': s_fp,
            's_prog': s_prog,
            's_op': s_op,
            'n_brakes': len(all_ba),
            'n_tp': sum(1 for a in all_ba if a.get('gt_brake_class') == 'true_positive'),
            'n_ghost': sum(1 for a in all_ba if a.get('gt_brake_class') == 'self_ghost'),
            'n_fp': sum(1 for a in all_ba if a.get('gt_brake_class') == 'other_fp'),
            'first_ba': first_ba,
            'all_ba': all_ba,
            'aoi_p99_ticks': aoi_p99,
            'col_actor_id': col_actor_id,
            'col_frame': col_frame,
            'edge_stats': edge_stats,
        })
    return runs


def print_summary_table(runs, lat_min, lat_max, anch_only):
    """Print collision rate summary grouped by (manager, anchoring, latency)."""
    groups = defaultdict(list)
    for r in runs:
        if anch_only and not r['anch']:
            continue
        if not (lat_min <= r['lat_ms'] <= lat_max):
            continue
        key = (r['mgr'], r['anch'], r['lat_ms'])
        groups[key].append(r)

    print("\n" + "=" * 130)
    print(f"{'Manager':<15} {'Anch':<6} {'Lat(ms)':<8} {'Runs':<5} "
          f"{'Collide':<8} {'Rate':<6} {'TP':<5} {'Ghost':<6} {'FP':<5} "
          f"{'1st_tick':<9} {'1st_AoI':<8} {'1st_TTC':<8} {'AoI_p99':<8} "
          f"{'ColActor':<10} {'1stGtActor':<11}")
    print("-" * 130)

    for key in sorted(groups.keys(), key=lambda k: (k[0], k[1], k[2])):
        mgr, anch, lat = key
        group = groups[key]
        n = len(group)
        n_collide = sum(1 for r in group if r['s_coll'] == 0)
        rate = n_collide / n if n > 0 else 0

        # Averages
        tp_avg = sum(r['n_tp'] for r in group) / n
        ghost_avg = sum(r['n_ghost'] for r in group) / n
        fp_avg = sum(r['n_fp'] for r in group) / n

        # First brake info
        first_ticks = [r['first_ba']['trigger_tick'] for r in group
                       if r['first_ba'] is not None]
        first_tick = f"{sum(first_ticks)/len(first_ticks):.0f}" if first_ticks else "-"

        first_aois = []
        for r in group:
            if r['first_ba'] and r['first_ba'].get('source_tick') is not None:
                aoi = r['first_ba']['trigger_tick'] - r['first_ba']['source_tick']
                first_aois.append(aoi)
        first_aoi = f"{sum(first_aois)/len(first_aois):.1f}" if first_aois else "-"

        first_ttcs = [r['first_ba']['ttc'] for r in group
                      if r['first_ba'] and r['first_ba'].get('ttc') is not None]
        first_ttc = f"{sum(first_ttcs)/len(first_ttcs):.2f}" if first_ttcs else "-"

        aoi_p99s = [r['aoi_p99_ticks'] for r in group if r['aoi_p99_ticks'] is not None]
        aoi_p99 = f"{sum(aoi_p99s)/len(aoi_p99s):.1f}" if aoi_p99s else "-"

        # Collision actor (new field — may be None for old runs)
        col_actors = [str(r['col_actor_id']) for r in group if r['col_actor_id'] is not None]
        col_actor = ",".join(col_actors) if col_actors else "N/A"

        # First brake GT actor
        gt_actors = set()
        for r in group:
            if r['first_ba'] and r['first_ba'].get('gt_matched_actor_id'):
                gt_actors.add(r['first_ba']['gt_matched_actor_id'])
        gt_actor_str = ",".join(str(a) for a in sorted(gt_actors)) if gt_actors else "-"

        marker = " ***" if n_collide > 0 and n_collide < n else ""
        if n_collide == n:
            marker = " ALL"
        if n_collide == 0:
            marker = ""

        print(f"{mgr:<15} {str(anch):<6} {lat:<8.0f} {n:<5} "
              f"{n_collide:<8} {rate:<6.2f} {tp_avg:<5.1f} {ghost_avg:<6.1f} {fp_avg:<5.1f} "
              f"{first_tick:<9} {first_aoi:<8} {first_ttc:<8} {aoi_p99:<8} "
              f"{col_actor:<10} {gt_actor_str:<11}{marker}")

    print("=" * 130)


def print_collision_zone_detail(runs, anch_only):
    """Detailed brake trace for the non-monotonic zone (lat=100-400)."""
    print("\n\n" + "=" * 100)
    print("COLLISION ZONE DETAIL (anch=ON, lat=100-400)")
    print("=" * 100)

    for r in sorted(runs, key=lambda x: (x['lat_ms'], x['run'])):
        if anch_only and not r['anch']:
            continue
        if not (100 <= r['lat_ms'] <= 400):
            continue
        if not r['anch']:
            continue

        collided = "COLLIDE" if r['s_coll'] == 0 else "safe"
        print(f"\n--- lat={r['lat_ms']:.0f}ms {r['run']} → {collided} ---")

        ba = r['all_ba']
        if not ba:
            print("  No brake events")
            continue

        # Show first brake, last brake, and summary
        first = ba[0]
        last = ba[-1]
        src_range = f"{first.get('source_tick','?')}-{last.get('source_tick','?')}"
        trg_range = f"{first.get('trigger_tick','?')}-{last.get('trigger_tick','?')}"

        print(f"  {len(ba)} brakes | src_ticks={src_range} | trg_ticks={trg_range}")
        print(f"  First: obs=({first.get('obs_x',0):.1f},{first.get('obs_y',0):.1f}) "
              f"ego=({first.get('ego_x',0):.1f},{first.get('ego_y',0):.1f}) "
              f"TTC={first.get('ttc',0):.2f}s class={first.get('gt_brake_class')} "
              f"gt_actor={first.get('gt_matched_actor_id')}")
        print(f"  Last:  obs=({last.get('obs_x',0):.1f},{last.get('obs_y',0):.1f}) "
              f"ego=({last.get('ego_x',0):.1f},{last.get('ego_y',0):.1f}) "
              f"TTC={last.get('ttc',0):.2f}s class={last.get('gt_brake_class')} "
              f"gt_actor={last.get('gt_matched_actor_id')}")

        # Ego velocity estimate from position change
        if len(ba) >= 2:
            dy = ba[-1].get('ego_y', 0) - ba[0].get('ego_y', 0)
            dt = (ba[-1].get('trigger_tick', 0) - ba[0].get('trigger_tick', 0)) * 0.05
            if dt > 0:
                v_ego = dy / dt
                print(f"  Ego avg speed during braking: {v_ego:.1f} m/s ({v_ego*3.6:.0f} km/h)")

        # Distance to conflict point (NPC at y≈128, ego going north)
        conflict_y = 128.0  # approximate NPC y-coordinate
        ego_y_first = first.get('ego_y', 0)
        ego_y_last = last.get('ego_y', 0)
        print(f"  Dist to conflict: first={conflict_y - ego_y_first:.1f}m, "
              f"last={conflict_y - ego_y_last:.1f}m")

        if r['col_actor_id'] is not None:
            print(f"  Collision actor: {r['col_actor_id']}")

        # Edge acceptance stats if available
        es = r['edge_stats']
        if es.get('edge_ticks_total', 0) > 0:
            frac_preds = es['edge_ticks_with_preds'] / es['edge_ticks_total']
            frac_coll = es['edge_ticks_with_collision'] / es['edge_ticks_total']
            avg_preds = es['edge_preds_received_total'] / es['edge_ticks_total']
            avg_ego_filt = es['edge_preds_ego_filtered'] / es['edge_ticks_total']
            print(f"  Edge acceptance: {frac_preds:.1%} ticks w/preds, "
                  f"{avg_preds:.1f} avg preds/tick, "
                  f"{avg_ego_filt:.1f} ego-filtered/tick, "
                  f"{frac_coll:.1%} ticks w/collision")


def print_confound_analysis(runs):
    """Summarize confound checks."""
    print("\n\n" + "=" * 100)
    print("CONFOUND ANALYSIS")
    print("=" * 100)

    # Confound A: Edge prediction acceptance by latency
    print("\n--- Confound A: Edge Prediction Acceptance ---")
    by_lat = defaultdict(list)
    for r in runs:
        if not r['anch']:
            continue
        es = r['edge_stats']
        if es.get('edge_ticks_total', 0) > 0:
            by_lat[r['lat_ms']].append(es)

    if by_lat:
        print(f"{'Lat(ms)':<10} {'TotalTicks':<12} {'WithPreds%':<12} "
              f"{'AvgPreds':<10} {'EgoFilt':<10} {'WithColl%':<12}")
        for lat in sorted(by_lat.keys()):
            stats_list = by_lat[lat]
            n = len(stats_list)
            total = sum(s['edge_ticks_total'] for s in stats_list) / n
            with_preds = sum(s['edge_ticks_with_preds'] for s in stats_list) / n
            avg_preds = sum(s['edge_preds_received_total'] for s in stats_list) / (
                sum(s['edge_ticks_total'] for s in stats_list) or 1)
            ego_filt = sum(s['edge_preds_ego_filtered'] for s in stats_list) / (
                sum(s['edge_ticks_total'] for s in stats_list) or 1)
            with_coll = sum(s['edge_ticks_with_collision'] for s in stats_list) / n
            print(f"{lat:<10.0f} {total:<12.0f} {with_preds/total*100:<12.1f} "
                  f"{avg_preds:<10.1f} {ego_filt:<10.1f} {with_coll/total*100:<12.1f}")
    else:
        print("  No edge acceptance data available (runs predate the instrumentation)")

    # Confound B: Collision actor identity
    print("\n--- Confound B: Collision Actor Identity ---")
    print("  Checking if collision counterpart matches the brake-triggering GT actor...")
    for r in sorted(runs, key=lambda x: x['lat_ms']):
        if not r['anch'] or r['s_coll'] != 0:  # only colliding runs
            continue
        col_id = r['col_actor_id']
        gt_actors = set(a.get('gt_matched_actor_id') for a in r['all_ba']
                        if a.get('gt_brake_class') == 'true_positive')
        if col_id is not None:
            match = "MATCH" if col_id in gt_actors else "MISMATCH"
            print(f"  lat={r['lat_ms']:.0f}ms {r['run']}: "
                  f"collision_actor={col_id}, brake_gt_actors={gt_actors} → {match}")
        else:
            print(f"  lat={r['lat_ms']:.0f}ms {r['run']}: "
                  f"collision_actor=N/A (pre-instrumentation), "
                  f"brake_gt_actors={gt_actors}")


def write_csv(runs, path, anch_only):
    """Write detailed CSV for further analysis."""
    import csv
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['manager', 'anchoring', 'latency_ms', 'run',
                     'collided', 'n_brakes', 'n_tp', 'n_ghost', 'n_fp',
                     'first_src_tick', 'first_trg_tick', 'first_aoi_ticks',
                     'first_ttc_s', 'first_obs_x', 'first_obs_y',
                     'first_ego_x', 'first_ego_y', 'first_gt_class',
                     'first_gt_actor', 'aoi_p99_ticks',
                     'collision_actor_id', 'collision_frame',
                     'edge_ticks_total', 'edge_ticks_with_preds',
                     'edge_preds_per_tick', 'edge_ego_filtered_per_tick'])
        for r in sorted(runs, key=lambda x: (x['mgr'], x['anch'], x['lat_ms'], x['run'])):
            if anch_only and not r['anch']:
                continue
            ba = r['first_ba']
            es = r['edge_stats']
            etotal = es.get('edge_ticks_total', 0)
            w.writerow([
                r['mgr'], r['anch'], r['lat_ms'], r['run'],
                1 if r['s_coll'] == 0 else 0,
                r['n_brakes'], r['n_tp'], r['n_ghost'], r['n_fp'],
                ba.get('source_tick') if ba else '',
                ba.get('trigger_tick') if ba else '',
                (ba['trigger_tick'] - ba['source_tick']) if ba and ba.get('source_tick') is not None else '',
                ba.get('ttc') if ba else '',
                ba.get('obs_x') if ba else '',
                ba.get('obs_y') if ba else '',
                ba.get('ego_x') if ba else '',
                ba.get('ego_y') if ba else '',
                ba.get('gt_brake_class') if ba else '',
                ba.get('gt_matched_actor_id') if ba else '',
                r['aoi_p99_ticks'] or '',
                r['col_actor_id'] or '',
                r['col_frame'] or '',
                etotal,
                es.get('edge_ticks_with_preds', 0),
                f"{es['edge_preds_received_total']/etotal:.1f}" if etotal > 0 else '',
                f"{es['edge_preds_ego_filtered']/etotal:.1f}" if etotal > 0 else '',
            ])
    print(f"\nCSV written to {path}")


def main():
    args = parse_args()
    runs = load_runs(args.sweep_dir)
    if not runs:
        print(f"No simulation_metrics.json found in {args.sweep_dir}")
        sys.exit(1)

    print(f"Loaded {len(runs)} runs from {args.sweep_dir}")

    lat_min, lat_max = args.latency_range
    print_summary_table(runs, lat_min, lat_max, args.anch_only)
    print_collision_zone_detail(runs, args.anch_only)
    print_confound_analysis(runs)

    if args.csv:
        write_csv(runs, args.csv, args.anch_only)


if __name__ == '__main__':
    main()
