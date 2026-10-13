#!/usr/bin/env python3
"""Reviewer C4: bootstrap 95% intervals on the landing study's p99(Delta_use)
per architecture and N. Same per-tick composition as scripts/landing_ns3.py
(_stages), same source CSV (paper2/paper2_figures/aoi_composed.csv). Resamples
ticks within each N cell; if the CSV carries a locale column (rsu), also
resamples locales (block bootstrap) so the interval reflects locale-to-locale
variation rather than tick count alone."""
import csv, os, sys, collections, random
import numpy as np
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AOI = os.path.join(ROOT, "paper2", "paper2_figures", "aoi_composed.csv")
Ns = [4, 8, 16, 24, 31]
PLAN_MS = 5.0
def _c(r, k, d=0.0):
    try: return float(r[k])
    except Exception: return d
def duse(arch, r):
    ul = _c(r,'network_ul_ms'); dl = _c(r,'network_dl_ms')
    det = _c(r,'detection_ms'); trk = _c(r,'tracking_ms'); prd = _c(r,'prediction_ms')
    sp = _c(r,'serialize_pred_ms') + _c(r,'deserialize_pred_ms'); cons = _c(r,'consume_lag_ms', 25.0)
    if arch == 'oracle': return det + trk + prd + 9.0 + cons
    if arch == 'object_i2v': return det + trk + prd + 2.0 + ul + dl + cons
    if arch == 'coop_pred': return det + trk + prd + sp + ul + dl + cons
    if arch == 'coop_planning': return det + trk + prd + PLAN_MS + 0.5 + ul + dl + cons
    if arch == 'intermediate':
        aoi = _c(r,'aoi_total_ms'); net = ul + dl
        return max(0.0, aoi - net - cons) + net + cons
    return float('nan')
rows = list(csv.DictReader(open(AOI)))
cols = rows[0].keys()
locale_col = next((c for c in ('rsu', 'locale', 'zone') if c in cols), None)
print("columns:", ", ".join(cols)); print("locale column:", locale_col); print("ticks:", len(rows))
byN = collections.defaultdict(list)
for r in rows:
    try: byN[int(float(r['N']))].append(r)
    except Exception: pass
# nearest-N binning as landing_ns3 does implicitly via exact N; report which Ns exist
print("N values present:", sorted(byN)[:20])
rng = random.Random(0); B = 1000
out = ["| arch | N | ticks | locales | p99 (ms) | 95% CI ticks | 95% CI locales |", "|---|---|---|---|---|---|---|"]
for arch in ['oracle', 'object_i2v', 'coop_pred', 'coop_planning', 'intermediate']:
    for n in Ns:
        rs = byN.get(n, [])
        if not rs: continue
        v = np.array([duse(arch, r) for r in rs]); v = v[~np.isnan(v)]
        if len(v) == 0: continue
        p99 = np.percentile(v, 99)
        bt = [np.percentile(rng.choices(list(v), k=len(v)), 99) for _ in range(B)]
        ci_t = (np.percentile(bt, 2.5), np.percentile(bt, 97.5))
        ci_l = ("", "")
        nloc = ""
        if locale_col:
            groups = collections.defaultdict(list)
            for r in rs: groups[r[locale_col]].append(duse(arch, r))
            keys = list(groups); nloc = len(keys)
            bl = []
            for _ in range(B):
                pick = rng.choices(keys, k=len(keys))
                pooled = np.concatenate([np.array(groups[k]) for k in pick])
                pooled = pooled[~np.isnan(pooled)]
                bl.append(np.percentile(pooled, 99))
            ci_l = (np.percentile(bl, 2.5), np.percentile(bl, 97.5))
        out.append(f"| {arch} | {n} | {len(v)} | {nloc} | {p99:.0f} | [{ci_t[0]:.0f}, {ci_t[1]:.0f}] | " + (f"[{ci_l[0]:.0f}, {ci_l[1]:.0f}]" if locale_col else "n/a") + " |")
text = "\n".join(out); print(text)
open(os.path.join(ROOT, "safety_envelope_paper", "analysis", "landing_ci.md"), "w").write("# Bootstrap intervals on landing p99(Delta_use)\n\n" + text + "\n")
