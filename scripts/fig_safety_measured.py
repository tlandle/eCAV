#!/usr/bin/env python3
"""Figure 2 reference physics envelope, from REAL closed-loop runs only.

Two-axis taxonomy. Data sources (all measured success_rate, no synthetic):
  V2X2V cooperative prediction (+/- EUI contract) = late_fusion manager,
      OLD run 20260419_120000, speed 50 (edge tracks+predicts+broadcasts).
  V2X2V reference / Oracle = oracle manager.
  I2V object-level perception = infra_only manager, boundary run, speed 40.
  I2V cooperative planning (CIP) = cip manager, boundary run, speed 40.

NOTE the speed mismatch: the I2V cuts (infra_only/cip/oracle) are speed 40, the
V2X2V cooperative-prediction sweep is speed 50, so their physics cliffs sit at
slightly different latencies. Flagged in the caption until a matched-speed run
exists.
"""
import os, glob, json, collections
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(os.path.dirname(__file__), 'figures_arch_aligned')
OLD = os.path.join(ROOT, 'safety_envelope_paper/experiment_results/20260419_120000')
BND = os.path.join(ROOT, 'experiment_results/openscenario_3_edge_late_fusion_boundary/20260602_201759')
plt.rcParams.update({'font.size': 9, 'axes.labelsize': 9, 'legend.fontsize': 6,
    'xtick.labelsize': 8, 'ytick.labelsize': 8, 'figure.dpi': 150,
    'lines.linewidth': 1.6, 'lines.markersize': 4})

def load(run_glob):
    g = collections.defaultdict(list)
    for f in glob.glob(run_glob, recursive=True):
        try: m = json.load(open(f))
        except Exception: continue
        if m.get('config_manager_type') is None:
            continue
        g[(m['config_manager_type'], str(m.get('config_anchoring')),
           int(m.get('config_latency_ms', -1)), m.get('config_ego_count'))].append(
               float(m.get('success_rate', np.nan)))
    return g
GOLD = load(OLD + '/*/simulation_metrics.json')
GBND = load(BND + '/**/simulation_metrics.json')
def S(g, mt, anc, lat, ego=None):
    # ego=None -> aggregate over any ego (boundary run has ego=None)
    vals = [np.mean([x for x in v if x == x])
            for k, v in g.items() if k[0]==mt and k[1]==anc and k[2]==lat and (ego is None or k[3]==ego) and v]
    vals = [x for x in vals if x == x]
    return float(np.mean(vals)) if vals else np.nan

LATS = [0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500]
# (graph_source, mgr, anchoring, ego, label, color, ls, marker)
CURVES = [
    (GBND, 'oracle',         'True',  None, 'Reference (Oracle), V2X2V',                 '#2ca02c', '-',  's'),
    (GOLD, 'late_fusion',    'True',  1,    'V2X2V cooperative prediction + contract',   '#1f77b4', '-',  'o'),
    (GOLD, 'late_fusion',    'False', 1,    'V2X2V cooperative prediction, no contract', '#1f77b4', '--', 'o'),
    (GBND, 'infra_only',     'True',  None, 'I2V object-level perception',               '#ff7f0e', '-',  '^'),
    (GBND, 'cip',            'True',  None, 'I2V cooperative planning (CIP)',            '#9467bd', ':',  'X'),
]
fig, ax = plt.subplots(figsize=(4.3, 2.8))
ax.axvspan(300, 450, color='0.88')
ax.text(303, 0.04, 'physics boundary', fontsize=5.6, color='0.45')
for g, mt, anc, ego, lab, col, ls, mk in CURVES:
    ys = [S(g, mt, anc, L, ego) for L in LATS]
    xs = [L for L, y in zip(LATS, ys) if y == y]; yy = [y for y in ys if y == y]
    if xs:
        ax.plot(xs, yy, color=col, ls=ls, marker=mk, ms=3.5, label=lab)
ax.set_xlabel(r'configured end-to-end latency $\approx p_{99}(\Delta_{use})$ (ms)')
ax.set_ylabel(r'$\Pr[S_{op}=1]$'); ax.set_ylim(-0.03, 1.05)
ax.grid(alpha=0.2); ax.legend(fontsize=5.4, loc='center left', bbox_to_anchor=(0.0, 0.40))
fig.tight_layout(); fig.savefig(f'{OUT}/fig_ref_envelope_measured.pdf'); fig.savefig(f'{OUT}/fig_ref_envelope_measured.png', dpi=200)
print("wrote fig_ref_envelope_measured (real runs)")
for g, mt, anc, ego, lab, *_ in CURVES:
    print(f"  {lab:<42}", {L: round(S(g, mt, anc, L, ego), 2) for L in [0,100,200,300,350,400,450] if S(g,mt,anc,L,ego)==S(g,mt,anc,L,ego)})
