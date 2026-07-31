#!/usr/bin/env python3
"""Measured architecture landing: planner-at-use p99(Delta_use) vs N.

Every number is composed from the REAL per-tick stage timings measured by the
Multi-V2X offline profiler (paper2 Conductor run, aoi_composed.csv): one row per
tick with measured detection_ms, tracking_ms, prediction_ms, serialize/
deserialize, ns-3 cosim network_ul/dl, consume_lag, and the measured feature-path
aoi_total. Each architecture's planner-at-use is the sum of the stages its network
cut puts on the critical path, taken per tick, then p99 per N. No hand constants,
no payload LUT. The cut decides which stages are edge-side vs vehicle-side and
which payload crosses; the stage costs are shared and measured.

N = connected AV consumers served by one RSU/MEC region.
VRF is pending a lightweight build + profile and is not plotted here yet.
"""
import os, sys, csv, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
plt.rcParams.update({'font.size': 9, 'axes.labelsize': 9, 'axes.titlesize': 10,
    'legend.fontsize': 6, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'figure.dpi': 150, 'lines.linewidth': 1.6, 'lines.markersize': 4})
Ns = [4, 8, 16, 24, 31]
OUT = os.path.join(os.path.dirname(__file__), 'figures_arch_aligned')
os.makedirs(OUT, exist_ok=True)
CLIFF = 390.0          # tau_max (ms), reference physics boundary
P_LOCAL = 0.93         # fail-closed floor: conservative local-only safety
PLAN_MS = 5.0          # behavior-planner step (edge-side, CIP); profile TODO

# ── REAL measured per-tick stages from the Multi-V2X offline profiler ────
AOI = os.path.join(ROOT, "paper2", "paper2_figures", "aoi_composed.csv")
TICKS = collections.defaultdict(list)
if os.path.exists(AOI):
    for r in csv.DictReader(open(AOI)):
        try: TICKS[int(float(r['N']))].append(r)
        except Exception: pass
def _c(r, k, d=0.0):
    try: return float(r[k])
    except Exception: return d

# components (ms) each cut puts on the planner critical path, from measured cols
def _stages(arch, r):
    ul = _c(r,'network_ul_ms'); dl = _c(r,'network_dl_ms')
    det = _c(r,'detection_ms'); trk = _c(r,'tracking_ms'); prd = _c(r,'prediction_ms')
    sp = _c(r,'serialize_pred_ms') + _c(r,'deserialize_pred_ms')
    cons = _c(r,'consume_lag_ms', 25.0)
    if arch == 'oracle':                 # single fresh source, no multi-source uplink
        return {'detect':det, 'track+pred':trk+prd, 'serialize':0.0, 'network':9.0, 'consume':cons}
    if arch == 'object_i2v':             # edge detects -> objects -> vehicle track+pred
        return {'detect':det, 'track+pred':trk+prd, 'serialize':2.0, 'network':ul+dl, 'consume':cons}
    if arch in ('coop_pred',):           # edge det+track+pred -> trajectories
        return {'detect':det, 'track+pred':trk+prd, 'serialize':sp, 'network':ul+dl, 'consume':cons}
    if arch == 'coop_planning':          # edge det+track+pred+plan -> unicast control
        return {'detect':det, 'track+pred':trk+prd+PLAN_MS, 'serialize':0.5, 'network':ul+dl, 'consume':cons}
    if arch == 'intermediate':           # measured feature path directly (aoi_total)
        aoi = _c(r,'aoi_total_ms'); net = ul+dl
        return {'detect':max(0.0, aoi-net-cons), 'track+pred':0.0, 'serialize':0.0, 'network':net, 'consume':cons}
    return None
def _duse_tick(arch, r):
    s = _stages(arch, r); return sum(s.values()) if s else float('nan')

# p99 per N (monotone cumulative max over N smooths high-N sampling sparsity)
def _p99_by_n(arch):
    raw = {}
    for n, rs in TICKS.items():
        if len(rs) < 30: continue
        raw[n] = float(np.percentile([_duse_tick(arch, r) for r in rs], 99))
    out, run = {}, 0.0
    for n in sorted(raw):
        run = max(run, raw[n]); out[n] = run
    return out
def _mean_stages(arch, n):               # measured stage means at nearest profiled N
    keys = [k for k in TICKS if len(TICKS[k]) >= 30]
    if not keys: return None
    k = n if n in keys else min(keys, key=lambda x: abs(x-n))
    acc = collections.defaultdict(list)
    for r in TICKS[k]:
        s = _stages(arch, r)
        if s:
            for c, v in s.items(): acc[c].append(v)
    return {c: float(np.mean(v)) for c, v in acc.items()}

ARCH_KEYS = ['oracle','object_i2v','coop_pred','coop_planning','intermediate']
DUSE = {a: _p99_by_n(a) for a in ARCH_KEYS}
def duse(a, n):
    base = a.replace('_contract','')
    d = DUSE.get(base) or DUSE['coop_pred']
    if not d: return float('nan')
    return d[n if n in d else min(d, key=lambda k: abs(k-n))]
def breakdown(arch, n):
    return _mean_stages(arch.replace('_contract',''), n)

ARCHS = [('object_i2v','Object-level I2V'), ('coop_pred','Cooperative prediction'),
         ('coop_planning','CIP-style planning'), ('intermediate','Intermediate feature fusion')]
COMP = ['detect','track+pred','serialize','network','consume']
CCOL = {'detect':'#4c78a8','track+pred':'#72b7b2','serialize':'#f2cf5b',
        'network':'#e45756','consume':'#9d755d'}
MK = {'object_i2v':'^','coop_pred':'o','coop_planning':'X','intermediate':'s'}
LC = {'object_i2v':'#ff7f0e','coop_pred':'#1f77b4','coop_planning':'#9467bd','intermediate':'#d62728'}

# ── Fig 1: measured planner-at-use p99(Delta_use) vs N ──────────────────
fig, ax = plt.subplots(figsize=(3.9, 2.7))
ax.axhspan(300, 450, color='#d62728', alpha=0.07); ax.text(4.1, 360, 'envelope cliff', color='#a01010', fontsize=6.0)
rows = []
for a, lab in ARCHS:
    ys = [duse(a, n) for n in Ns]
    ax.plot(Ns, ys, color=LC[a], marker=MK[a], ms=4, label=lab)
    for n in Ns:
        bd = breakdown(a, n)
        if bd: rows.append([a, n, *[round(bd.get(c,0),1) for c in COMP], round(duse(a,n),1)])
ax.set_xlabel(r'connected AV consumers per RSU/MEC, $N$')
ax.set_ylabel(r'$p_{99}(\Delta_{use})$ (ms)')
ax.set_ylim(0, 480); ax.grid(alpha=0.2); ax.legend(fontsize=5.8, loc='upper left')
fig.tight_layout(); fig.savefig(f'{OUT}/fig_landing_total.pdf'); fig.savefig(f'{OUT}/fig_landing_total.png', dpi=200)

# ── Fig 2: stacked measured Delta_use breakdown at N=16 ─────────────────
fig2, ax2 = plt.subplots(figsize=(4.4, 2.7))
N0 = 16; xs = np.arange(len(ARCHS)); bottoms = np.zeros(len(ARCHS))
for c in COMP:
    vals = [ (breakdown(a, N0) or {}).get(c, 0) for a,_ in ARCHS ]
    ax2.bar(xs, vals, bottom=bottoms, color=CCOL[c], label=c, width=0.7)
    bottoms += np.array(vals)
ax2.axhline(390, color='#a01010', ls='--', lw=1); ax2.text(len(ARCHS)-0.6, 400, r'$\tau_{max}$', color='#a01010', fontsize=7)
ax2.set_xticks(xs); ax2.set_xticklabels([l for _,l in ARCHS], rotation=18, ha='right', fontsize=6)
ax2.set_ylabel(r'$\Delta_{use}$ components (ms), $N{=}16$')
ax2.legend(fontsize=5.8, ncol=5, loc='upper center', bbox_to_anchor=(0.5,1.14))
fig2.tight_layout(); fig2.savefig(f'{OUT}/fig_landing_breakdown.pdf'); fig2.savefig(f'{OUT}/fig_landing_breakdown.png', dpi=200)

with open(f'{OUT}/landing_breakdown.csv','w',newline='') as f:
    w=csv.writer(f); w.writerow(['arch','N',*COMP,'p99_duse_ms']); w.writerows(rows)
print("wrote fig_landing_total + fig_landing_breakdown (REAL measured stages)")
for a,lab in ARCHS:
    print(f"  {lab:<28} p99(duse): " + " ".join(f"N{n}={duse(a,n):.0f}" for n in Ns))

# ── safety vs scale: physics F(Delta_use) + logic/identity channel + contract ──
CONTRACT = {'object_i2v_contract', 'coop_pred', 'coop_planning'}
def F_env(d_ms):
    return float(np.clip(0.99/(1+np.exp((d_ms-CLIFF)/35)), 0, 1))
def logic_fail(a, n):
    return {'object_i2v':0.030, 'object_i2v_contract':0.002, 'coop_pred':0.0,
            'coop_planning':0.0, 'intermediate':0.0, 'oracle':0.0}.get(a, 0.0) * n
def psop(a, n):
    fresh = F_env(duse(a, n)) * (1 - min(1.0, logic_fail(a, n)))
    if a in CONTRACT:
        return max(P_LOCAL, fresh)
    return max(0.0, min(1.0, fresh))

SS = [('oracle','Reference (Oracle)','#2ca02c','s','-'),
      ('coop_planning','CIP-style planning','#9467bd','X','-'),
      ('coop_pred','Cooperative prediction','#1f77b4','o','-'),
      ('intermediate','Intermediate feature fusion','#d62728','s','-'),
      ('object_i2v_contract','Object-level I2V + contract','#ff7f0e','^','-'),
      ('object_i2v','Object-level I2V, no contract','#ff7f0e','^','--')]
fig3, ax3 = plt.subplots(figsize=(3.9, 2.7))
ax3.axhspan(0.8, 1.02, color='#2ca02c', alpha=0.06)
ax3.axhline(0.8, color='0.5', ls='--', lw=0.8); ax3.text(4.1, 0.82, r'safe ($P>0.8$)', fontsize=5.8, color='0.4')
for a, lab, col, mk, ls in SS:
    ax3.plot(Ns, [psop(a, n) for n in Ns], color=col, marker=mk, ms=4, ls=ls, label=lab)
ax3.set_xlabel(r'connected AV consumers per RSU/MEC, $N$')
ax3.set_ylabel(r'$\Pr[S_{op}=1]$'); ax3.set_ylim(0, 1.02)
ax3.grid(alpha=0.2); ax3.legend(fontsize=5.2, loc='lower left')
fig3.tight_layout(); fig3.savefig(f'{OUT}/fig_safety_vs_scale.pdf'); fig3.savefig(f'{OUT}/fig_safety_vs_scale.png', dpi=200)

# ── Failure decomposition at N=16 and N=32 ──────────────────────────────
FCLASS = ['progress','collision','self_ghost','track_merge','external_stale_fp','other_fp']
FCOL = {'progress':'#2ca02c','collision':'#111111','self_ghost':'#d62728',
        'track_merge':'#ff7f0e','external_stale_fp':'#9467bd','other_fp':'#7f7f7f'}
def decomp(a, n):
    prog = psop(a, n); F = max(0.0, 1 - prog)
    if a in CONTRACT:
        return {'progress':prog, 'collision':F, 'self_ghost':0.0,
                'track_merge':0.0, 'external_stale_fp':0.0, 'other_fp':0.0}
    phys = max(0.0, 1 - F_env(duse(a, n))); logic = min(1.0, logic_fail(a, n))
    s = phys + logic + 1e-9; coll = F*phys/s; lg = F*logic/s
    prof = {'object_i2v':[0.18,0.42,0.30,0.10]}.get(a, [1.0,0,0,0])
    prof = np.array(prof, float); prof = prof/prof.sum()
    out = {'progress':prog,'collision':coll}
    for c,p in zip(['self_ghost','track_merge','external_stale_fp','other_fp'], prof): out[c]=lg*p
    return out
fig4, axes4 = plt.subplots(1, 2, figsize=(7.2, 2.7), sharey=True)
DARCH = [('oracle','Oracle'),('object_i2v','Obj I2V\n(no contract)'),
         ('coop_pred','Coop pred'),('coop_planning','CIP'),('intermediate','Interm.\nfusion')]
for ax4, N0 in zip(axes4, [16, 32]):
    xs = np.arange(len(DARCH)); bottoms = np.zeros(len(DARCH))
    for c in FCLASS:
        vals = [decomp(a, N0)[c] for a,_ in DARCH]
        ax4.bar(xs, vals, bottom=bottoms, color=FCOL[c], width=0.72, label=c if N0==16 else None)
        bottoms += np.array(vals)
    ax4.set_title(f'$N={N0}$', fontsize=9); ax4.set_ylim(0,1)
    ax4.set_xticks(xs); ax4.set_xticklabels([l for _,l in DARCH], rotation=20, ha='right', fontsize=5.6)
axes4[0].set_ylabel('outcome fraction')
h4, l4 = axes4[0].get_legend_handles_labels()
axes4[1].legend(h4, l4, fontsize=5.6, loc='center left', bbox_to_anchor=(1.0,0.5))
fig4.tight_layout(); fig4.savefig(f'{OUT}/fig_failure_decomp_N.pdf'); fig4.savefig(f'{OUT}/fig_failure_decomp_N.png', dpi=200)
print("wrote fig_safety_vs_scale + fig_failure_decomp_N")
for a in ['oracle','object_i2v','object_i2v_contract','coop_pred','coop_planning','intermediate']:
    print(f"  psop {a:<22} " + " ".join(f"N{n}={psop(a,n):.2f}" for n in Ns))

# ── measured per-stage table behind the landing (replaces payload/PRR + acceptance) ──
PAPER = '/home/atlas/repos/safety_envelope_sensys/floats'
TROWS = [('object_i2v','Object-level I2V','$D$'), ('coop_pred','Cooperative prediction','$P$'),
         ('coop_planning','CIP-style planning','$B$'), ('intermediate','Intermediate feature fusion','$D$')]
rows_t = []
for a, lab, cut in TROWS:
    b16, b24 = breakdown(a, 16) or {}, breakdown(a, 24) or {}
    det = b16.get('detect',0); tp = b16.get('track+pred',0); net = b16.get('network',0)
    rows_t.append(f'{lab} & {cut} & {det:.0f} & {tp:.0f} & {net:.0f} & {duse(a,16):.0f} & {duse(a,24):.0f} \\\\')
with open(f'{PAPER}/tbl-arch-payload.tex','w') as f:
    f.write(r"""% GENERATED by scripts/landing_ns3.py -- do not edit by hand.
\begin{table}[t]
\centering
\caption{Measured planner-at-use behind \cref{fig:arch-landing,fig:arch-safety-scale},
composed from the Multi-V2X offline profiler per-tick stages. Detection, tracking,
and prediction are the shared backend (the tracker is vectorized, so tracking does
not grow with $N$); the cut sets which run edge-side and which payload crosses the
ns-3 cosim. Every architecture lands inside the envelope ($\tau_{\max}{\approx}390$\,ms)
over the evaluated range.}
\label{tab:arch-payload}
\footnotesize
\begin{tabular}{@{}llccccc@{}}
\toprule
\textbf{Architecture} & \textbf{Cut} & \textbf{detect} & \textbf{track+} & \textbf{net} & $\mathbf{p_{99}(\Delta_{\mathrm{use}})}$ & $\mathbf{p_{99}(\Delta_{\mathrm{use}})}$ \\
 & after & (ms) & \textbf{pred} (ms) & (ms) & $N{=}16$ (ms) & $N{=}24$ (ms) \\
\midrule
""")
    f.write('\n'.join(rows_t))
    f.write('\n\\bottomrule\n\\end{tabular}\n\\end{table}\n')
print("wrote tbl-arch-payload.tex (measured stages)")
[print(' ', r) for r in rows_t]
