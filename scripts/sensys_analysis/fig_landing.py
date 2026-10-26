#!/usr/bin/env python3
"""Landing study figure and table from the per-tick stage composition of landing_ci.py
(scripts/sensys_analysis/landing_ci.py, same source CSV and stage sums as scripts/landing_ns3.py).
Outputs, under safety_envelope_paper/analysis/fig_landing/:
  arch_landing.pdf            p99(Delta_use) against N per architecture, block-bootstrap 95% interval over locales
  arch_landing_breakdown.pdf  mean stage components at N=16 per architecture
  tbl-arch-payload.tex        measured stage terms at N=16 and p99 at N=16 and N=24 with intervals
  landing.csv                 the numbers behind both
Run: python scripts/sensys_analysis/fig_landing.py
"""
import csv, os, sys, collections, random
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AOI = os.path.join(ROOT, "paper2", "paper2_figures", "aoi_composed.csv")
OUT = os.path.join(ROOT, "safety_envelope_paper", "analysis", "fig_landing")
os.makedirs(OUT, exist_ok=True)
Ns = [4, 8, 16, 24]
PLAN_MS = 5.0
ARCHS = [('oracle', 'Exact-state reference'), ('object_i2v', 'I2V object sharing'), ('coop_pred', 'V2X2V prediction'),
         ('coop_planning', 'Cooperative planning (CIP)'), ('intermediate', 'Intermediate feature fusion')]
COMP = ['detect', 'track+pred', 'serialize', 'network', 'consume']
CLAB = {'detect': 'frontend compute', 'track+pred': 'tracking and prediction', 'serialize': 'serialization',
        'network': 'network transfer', 'consume': 'planner consumption'}
CCOL = {'detect': '#4c78a8', 'track+pred': '#72b7b2', 'serialize': '#f2cf5b', 'network': '#e45756', 'consume': '#9d755d'}
MK = {'oracle': 'D', 'object_i2v': '^', 'coop_pred': 'o', 'coop_planning': 'X', 'intermediate': 's'}
LC = {'oracle': '#2ca02c', 'object_i2v': '#ff7f0e', 'coop_pred': '#1f77b4', 'coop_planning': '#9467bd', 'intermediate': '#d62728'}
plt.rcParams.update({'font.size': 8, 'axes.labelsize': 8, 'legend.fontsize': 6.5, 'xtick.labelsize': 7.5, 'ytick.labelsize': 7.5,
                     'figure.dpi': 150, 'lines.linewidth': 1.4, 'lines.markersize': 4})

def _c(r, k, d=0.0):
    try: return float(r[k])
    except Exception: return d

def stages(arch, r):
    ul = _c(r, 'network_ul_ms'); dl = _c(r, 'network_dl_ms')
    det = _c(r, 'detection_ms'); trk = _c(r, 'tracking_ms'); prd = _c(r, 'prediction_ms')
    sp = _c(r, 'serialize_pred_ms') + _c(r, 'deserialize_pred_ms'); cons = _c(r, 'consume_lag_ms', 25.0)
    if arch == 'oracle': return {'detect': det, 'track+pred': trk + prd, 'serialize': 0.0, 'network': 9.0, 'consume': cons}
    if arch == 'object_i2v': return {'detect': det, 'track+pred': trk + prd, 'serialize': 2.0, 'network': ul + dl, 'consume': cons}
    if arch == 'coop_pred': return {'detect': det, 'track+pred': trk + prd, 'serialize': sp, 'network': ul + dl, 'consume': cons}
    if arch == 'coop_planning': return {'detect': det, 'track+pred': trk + prd + PLAN_MS, 'serialize': 0.5, 'network': ul + dl, 'consume': cons}
    if arch == 'intermediate':
        aoi = _c(r, 'aoi_total_ms'); net = ul + dl
        return {'detect': max(0.0, aoi - net - cons), 'track+pred': 0.0, 'serialize': 0.0, 'network': net, 'consume': cons}
    return None

rows = list(csv.DictReader(open(AOI)))
cols = rows[0].keys()
locale_col = next((c for c in ('rsu', 'locale', 'zone') if c in cols), None)
byN = collections.defaultdict(list)
for r in rows:
    try: byN[int(float(r['N']))].append(r)
    except Exception: pass
print("N present:", sorted(byN), "locale column:", locale_col)
rng = random.Random(0); B = 1000
res = {}   # (arch, N) -> dict(p99, lo, hi, means)
for a, _ in ARCHS:
    for n in Ns:
        rs = byN.get(n, [])
        if not rs: continue
        st = [stages(a, r) for r in rs]
        v = np.array([sum(s.values()) for s in st])
        p99 = float(np.percentile(v, 99))
        if locale_col:
            groups = collections.defaultdict(list)
            for r, s in zip(rs, st): groups[r[locale_col]].append(sum(s.values()))
            keys = list(groups); bl = []
            for _ in range(B):
                pick = rng.choices(keys, k=len(keys))
                bl.append(np.percentile(np.concatenate([np.array(groups[k]) for k in pick]), 99))
        else:
            bl = [np.percentile(rng.choices(list(v), k=len(v)), 99) for _ in range(B)]
        lo, hi = float(np.percentile(bl, 2.5)), float(np.percentile(bl, 97.5))
        means = {c: float(np.mean([s[c] for s in st])) for c in COMP}
        res[(a, n)] = dict(p99=p99, lo=lo, hi=hi, means=means, ticks=len(rs))
with open(os.path.join(OUT, 'landing.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['arch', 'N', 'ticks', 'p99_ms', 'lo', 'hi', *COMP])
    for (a, n), d in sorted(res.items()): w.writerow([a, n, d['ticks'], round(d['p99'], 1), round(d['lo'], 1), round(d['hi'], 1), *[round(d['means'][c], 1) for c in COMP]])

fig, ax = plt.subplots(figsize=(3.4, 2.4))
for a, lab in ARCHS:
    xs = [n for n in Ns if (a, n) in res]
    ys = [res[(a, n)]['p99'] for n in xs]
    err = [[res[(a, n)]['p99'] - res[(a, n)]['lo'] for n in xs], [res[(a, n)]['hi'] - res[(a, n)]['p99'] for n in xs]]
    ax.errorbar(xs, ys, yerr=err, color=LC[a], marker=MK[a], capsize=2, label=lab)
ax.set_xlabel(r'connected CAVs per RSU/MEC, $N$'); ax.set_ylabel(r'$p_{99}(\Delta_{\mathrm{use}})$ (ms)')
ax.set_xscale('log', base=2); ax.set_xticks(Ns); ax.set_xticklabels([str(n) for n in Ns])
ax.set_ylim(100, 360); ax.grid(alpha=0.25); ax.legend(loc='upper left', frameon=False)
fig.tight_layout(); fig.savefig(os.path.join(OUT, 'arch_landing.pdf'))

fig2, ax2 = plt.subplots(figsize=(3.4, 2.3))
N0 = 16; xs = np.arange(len(ARCHS)); bottoms = np.zeros(len(ARCHS))
for c in COMP:
    vals = [res[(a, N0)]['means'][c] for a, _ in ARCHS]
    ax2.bar(xs, vals, bottom=bottoms, color=CCOL[c], label=CLAB[c], width=0.7); bottoms += np.array(vals)
ax2.set_xticks(xs); ax2.set_xticklabels([l.replace(' (CIP)', '\n(CIP)').replace('Intermediate feature fusion', 'Intermediate\nfeature fusion').replace('Exact-state reference', 'Exact-state\nreference') for _, l in ARCHS], fontsize=6.3)
ax2.set_ylabel(r'mean stage cost (ms), $N{=}16$'); ax2.legend(ncol=2, loc='upper left', frameon=False)
ax2.set_ylim(0, max(bottoms) * 1.45); ax2.grid(axis='y', alpha=0.25)
fig2.tight_layout(); fig2.savefig(os.path.join(OUT, 'arch_landing_breakdown.pdf'))

with open(os.path.join(OUT, 'tbl-arch-payload.tex'), 'w') as f:
    f.write("% GENERATED by scripts/sensys_analysis/fig_landing.py from the landing study reduction; do not edit by hand.\n")
    f.write("\\begin{table}[t]\n\\centering\\footnotesize\n")
    f.write("\\caption{Measured stage terms behind \\cref{fig:arch-landing}, in ms. \\textmd{Mean frontend compute, tracking and prediction, and network transfer per tick at $N{=}16$, and the planner-side $p_{99}(\\Delta_{\\mathrm{use}})$ at $N{=}16$ and $N{=}24$ with its bootstrap 95\\% interval over locales. The network boundary is the pipeline step after which the shared state crosses the link ($D$, $P$, $B$ as in \\cref{fig:arch-model}).}}\n")
    f.write("\\label{tab:arch-payload}\n\\resizebox{\\columnwidth}{!}{\\begin{tabular}{@{}llrrrrr@{}}\n\\toprule\n")
    f.write("\\textbf{Architecture} & \\textbf{Boundary} & \\textbf{front} & \\textbf{track+} & \\textbf{net} & $\\mathbf{p_{99}}$, $N{=}16$ & $\\mathbf{p_{99}}$, $N{=}24$ \\\\\n")
    f.write(" & after & & \\textbf{pred} & & & \\\\\n\\midrule\n")
    cut = {'oracle': 'none', 'object_i2v': '$D$', 'coop_pred': '$P$', 'coop_planning': '$B$', 'intermediate': 'features'}
    for a, lab in ARCHS:
        m = res[(a, 16)]['means']; d16 = res[(a, 16)]; d24 = res[(a, 24)]
        f.write(f"{lab} & {cut[a]} & {m['detect']:.0f} & {m['track+pred']:.0f} & {m['network']:.0f} & {d16['p99']:.0f} {{\\scriptsize[{d16['lo']:.0f}, {d16['hi']:.0f}]}} & {d24['p99']:.0f} {{\\scriptsize[{d24['lo']:.0f}, {d24['hi']:.0f}]}} \\\\\n")
    f.write("\\bottomrule\n\\end{tabular}}\n\\end{table}\n")
for (a, n), d in sorted(res.items()): print(f"{a:14s} N={n:2d} p99={d['p99']:.0f} [{d['lo']:.0f}, {d['hi']:.0f}]")
print("wrote", OUT)
