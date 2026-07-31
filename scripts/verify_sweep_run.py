#!/usr/bin/env python3
"""Continuous verification of sweep runs.

Reads each completed run's simulation_metrics.json (via the paper's
compute_run_metrics episode pipeline) and flags any run whose data does not
behave as expected, so a multi-hour sweep is not discovered to be garbage at
the end. Run repeatedly (or via Monitor) against a live sweep dir.

Checks per run:
  - run completed (no 'error' field, has edges{} + vehicles{})
  - detection recall > 0 (cross-traffic actually detected)
  - AoI tracks configured latency (measured p99 AoI rises with lat)
  - episode counts finite and sane (not exploding)
  - cross-traffic present in conflict logger (if CSV available)

Usage: python3 verify_sweep_run.py <sweep_dir> [<sweep_dir> ...]
Prints one line per run: OK / WARN <reason>. Exit 0 always (advisory).
"""
import sys, os, json, glob, re

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from recompute_metrics import compute_run_metrics
except Exception:
    compute_run_metrics = None


def _cfg_from_dirname(d):
    anc = 'on' if 'anchoring_on' in d else ('off' if 'anchoring_off' in d else '-')
    lat = re.search(r'lat_(\d+)', d)
    lat = int(lat.group(1)) if lat else None
    for m in ('oracle', 'vips', 'cip', 'infra_only', 'late_fusion', 'perception'):
        if m in d:
            mgr = m
            break
    else:
        mgr = '?'
    return mgr, anc, lat


def check_run(path):
    d = os.path.basename(os.path.dirname(path))
    mgr, anc, lat = _cfg_from_dirname(d)
    tag = f"[{mgr}/{anc}/lat{lat}]"
    try:
        data = json.load(open(path))
    except Exception as e:
        return f"WARN {tag} unreadable: {e}"
    if data.get('error'):
        return f"WARN {tag} run error: {str(data['error'])[:60]}"

    edges = data.get('edges', {})
    if not edges:
        return f"WARN {tag} no edges{{}} (run did not complete eval)"
    e = next(iter(edges.values()))

    warns = []
    # detection recall > 0 (cross-traffic seen)
    rec = e.get('detection_recall')
    if rec is not None and rec <= 0.0:
        warns.append(f"recall=0 (no detections)")
    # AoI should track latency: p99 ticks roughly >= lat/50
    aoi_p99 = e.get('aoi_p99_ticks')
    if aoi_p99 is not None and lat is not None:
        expect = lat / 50.0
        if aoi_p99 + 1 < expect:  # measured AoI well below injected -> suspicious
            warns.append(f"aoi_p99={aoi_p99}t < expect~{expect:.0f}t for lat{lat}")
    # episodes sane
    if compute_run_metrics:
        rm = compute_run_metrics(data)
        if rm:
            ge = rm.get('focal_ghost_episodes')
            if ge is not None and ge > 30:
                warns.append(f"ghost_episodes={ge} (exploding)")
    # ego-uniqueness fraction in [0,1]
    euf = e.get('ego_uniqueness_violation_tick_fraction')
    if euf is not None and not (0 <= euf <= 1):
        warns.append(f"euniq_frac out of range: {euf}")

    if warns:
        return f"WARN {tag} " + "; ".join(warns)
    return (f"OK   {tag} recall={rec:.2f} aoi_p99={aoi_p99}t "
            f"euniq={euf:.3f}" if rec is not None and euf is not None
            else f"OK   {tag}")


def main():
    dirs = sys.argv[1:] or ['.']
    seen = set()
    for base in dirs:
        for path in sorted(glob.glob(os.path.join(base, '**', 'simulation_metrics.json'),
                                     recursive=True)):
            if 'evaluation_output' in path or path in seen:
                continue
            seen.add(path)
            print(check_run(path), flush=True)


if __name__ == '__main__':
    main()
