#!/usr/bin/env python3
"""Load-coupled freshness summary across the metro replays: per point and
admission row, each arm's mechanism-only and load-coupled within-limit
fraction (2 s post-crossing window), the fraction of consumptions taken at
rate_k > 1, the rate_k distribution over owned locale-cycles, and the executed
pass p95 beside the owned-only pass p95 (which fallback is admission and which
is replication). Usage: coupled_summary.py <metro_dir>"""
import glob, json, os, re, sys
D = sys.argv[1]
files = [(f"pneuma {os.path.basename(f).split('_')[3].split('.')[0]}", f) for f in sorted(glob.glob(f"{D}/pneuma_0900_replay_*.json")) if re.search(r"replay_(demand|admit92|admit50|deployed)\.json$", f)]
def _tr(f): m = re.search(r"_t(\d+)_s1_(\w+)\.json", f); return int(m.group(1)), m.group(2)
files += [("ext t%d %s" % _tr(f), f) for f in sorted(glob.glob(f"{D}/ext/replay_ext_L*_t*_s1_*.json"), key=lambda f: (_tr(f)[0], f))]
print(f"{'point/row':22s} {'K':>3s} {'arm':12s} {'load%':>6s} {'mech':>6s} {'COUPL':>6s} {'rk>1':>6s} {'rk mean':>7s} {'pass p95':>8s} {'owned':>6s}  rate_k dist (1/2/4/5/10 over owned locale-cycles)   owned-pass model")
for tag, f in files:
    j = json.load(open(f)); o = j["n_owned_per_locale_cycle"]; fc = j.get("freshness_coupling") or {}
    for arm in ("warm", "replall", "delta_ready", "delta_state", "reactive"):
        r = j["arms"][arm]; rd = r["rate_k_frac_owned_locale_cycles"]
        print(f"{tag:22s} {str(o.get('K_admit')):>3s} {arm:12s} {100*r['p95_load_frac']:6.1f} {r['within_limit_postcross_frac']:6.3f} {r['within_limit_postcross_coupled']:6.3f} {r['rate_k_frac_gt1_at_consumption']:6.3f} {r['rate_k_mean_at_consumption']:7.2f} {r['pass_ms_p95_executed']:8.0f} {r['pass_ms_p95_owned_only'] or 0:6.0f}  " + "/".join(f"{(rd[k] or 0):.2f}" for k in ("1","2","4","5","10")) + f"   {(fc.get('owned_pass_model') or 'cap pattern (pre-switch)')[:28]}")
