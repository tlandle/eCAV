"""Generate additional paper-ready evaluation plots."""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

OUT = Path('paper2_figures')
plt.rcParams.update({
    'font.size': 10, 'font.family': 'serif',
    'axes.labelsize': 11, 'axes.titlesize': 12,
    'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'legend.fontsize': 8, 'figure.dpi': 150,
})

# Load all data
dfs = []
for f in ['ablation_rsu250_lightweight', 'ablation_rsu250_random',
          'ablation_rsu250_causal', 'ablation_rsu250_oracle_total',
          'ablation_rsu250_oracle_occluded', 'ablation_rsu250_joint']:
    try:
        dfs.append(pd.read_csv(OUT / f'{f}.csv'))
    except: pass
df = pd.concat(dfs, ignore_index=True)
df = df[df['tick'] >= 5].drop_duplicates(['config', 'tick'])

# ── Plot 5: Deadline Compliance Curve ─────────────────────────
fig, ax = plt.subplots(figsize=(5.5, 3.5))

# For each selector at each K, compute deadline compliance
selectors = {
    'Random': ['k_cav=0_lightweight', 'k_cav=2_random', 'k_cav=4_random', 'k_cav=8_random', 'static_all'],
    'Feature-norm': ['k_cav=0_lightweight', 'k_cav=2_lightweight', 'k_cav=4_lightweight', 'k_cav=8_lightweight', 'static_all'],
    'Causal (ours)': ['k_cav=0_lightweight', 'k_cav=2_causal', 'k_cav=4_causal', 'k_cav=8_causal', 'static_all'],
}
k_labels = [0, 2, 4, 8, 22]
colors = {'Random': '#ff7f0e', 'Feature-norm': '#d62728', 'Causal (ours)': '#2ca02c'}

for sel_name, configs in selectors.items():
    compliance = []
    for cfg in configs:
        sub = df[df['config'] == cfg]
        if len(sub) > 0:
            compliance.append((sub['edge_ms'] <= 100).mean())
        else:
            compliance.append(np.nan)
    ax.plot(k_labels, compliance, 'o-', label=sel_name,
            color=colors[sel_name], linewidth=2, markersize=6)

# Add joint controller point
joint = df[df['config'] == 'joint']
if len(joint) > 0:
    joint_comp = (joint['edge_ms'] <= 100).mean()
    joint_k = joint['n_cav_kept'].mean()
    ax.scatter([joint_k], [joint_comp], marker='*', s=200, color='#8c564b',
               zorder=10, label=f'Joint (K$_{{cav}}$={joint_k:.1f})')

ax.axhline(1.0, color='gray', linestyle=':', alpha=0.3)
ax.set_xlabel('$K_{cav}$ (CAVs selected beyond RSU)')
ax.set_ylabel('Deadline compliance rate')
ax.set_title('Fraction of ticks meeting 100ms deadline')
ax.set_ylim(-0.05, 1.05)
ax.set_xticks([0, 2, 4, 8, 22])
ax.legend(loc='lower left')
fig.tight_layout()
fig.savefig(OUT / 'fig_deadline_compliance.pdf')
fig.savefig(OUT / 'fig_deadline_compliance.png', dpi=200)
print('Saved fig_deadline_compliance')

# ── Plot 6: Per-tick edge latency distributions (box plot) ────
fig, ax = plt.subplots(figsize=(7, 3.5))

box_configs = [
    ('RSU only', 'k_cav=0_lightweight'),
    ('Random\nK=2', 'k_cav=2_random'),
    ('Causal\nK=2', 'k_cav=2_causal'),
    ('Random\nK=4', 'k_cav=4_random'),
    ('Causal\nK=4', 'k_cav=4_causal'),
    ('Random\nK=8', 'k_cav=8_random'),
    ('Causal\nK=8', 'k_cav=8_causal'),
    ('Joint', 'joint'),
    ('Static\nall', 'static_all'),
]

data_for_box = []
labels_for_box = []
for label, cfg in box_configs:
    sub = df[df['config'] == cfg]['edge_ms']
    if len(sub) > 0:
        data_for_box.append(sub.values)
        labels_for_box.append(label)

bp = ax.boxplot(data_for_box, labels=labels_for_box, patch_artist=True,
                showfliers=False, medianprops=dict(color='black', linewidth=1.5))

# Color by selector type
box_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#ff7f0e', '#2ca02c',
              '#ff7f0e', '#2ca02c', '#8c564b', '#7f7f7f']
for patch, color in zip(bp['boxes'], box_colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

ax.axhline(100, color='black', linestyle='--', linewidth=1, alpha=0.7, label='100ms deadline')
ax.set_ylabel('Edge latency (ms)')
ax.set_title('Per-tick latency distribution by selector and budget')
ax.legend(loc='upper left')
fig.tight_layout()
fig.savefig(OUT / 'fig_latency_boxplot.pdf')
fig.savefig(OUT / 'fig_latency_boxplot.png', dpi=200)
print('Saved fig_latency_boxplot')

# ── Plot 7: Occluded recall gap visualization ─────────────────
fig, ax = plt.subplots(figsize=(6, 4))

# Bar chart: recall_occluded for each selector at K=2
k2_configs = [
    ('RSU\nonly', 'k_cav=0_lightweight', '#1f77b4'),
    ('Feature\nnorm', 'k_cav=2_lightweight', '#d62728'),
    ('Random', 'k_cav=2_random', '#ff7f0e'),
    ('Causal\n(ours)', 'k_cav=2_causal', '#2ca02c'),
    ('Oracle\n(total)', 'k_cav=2_oracle_total', '#9467bd'),
    ('Oracle\n(occ)', 'k_cav=2_oracle_occluded', '#e377c2'),
    ('Static\nall', 'static_all', '#7f7f7f'),
]

x = np.arange(len(k2_configs))
bars_total = []
bars_occ = []
labels = []
colors = []

for label, cfg, color in k2_configs:
    sub = df[df['config'] == cfg]
    if len(sub) == 0: continue
    tp = sub['det_tp'].sum()
    fn = sub['det_fn'].sum()
    tp_o = sub['det_tp_occluded'].sum()
    fn_o = sub['det_fn_occluded'].sum()
    bars_total.append(tp / (tp + fn) if (tp + fn) > 0 else 0)
    bars_occ.append(tp_o / (tp_o + fn_o) if (tp_o + fn_o) > 0 else 0)
    labels.append(label)
    colors.append(color)

x = np.arange(len(labels))
w = 0.35
ax.bar(x - w/2, bars_total, w, label='Total recall', color=colors, alpha=0.6, edgecolor='black', linewidth=0.5)
ax.bar(x + w/2, bars_occ, w, label='Occluded recall', color=colors, alpha=1.0, edgecolor='black', linewidth=0.5)

ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=8)
ax.set_ylabel('Recall')
ax.set_title('Total vs RSU-occluded recall at $K_{cav}=2$')
ax.legend()
ax.set_ylim(0, 0.65)
fig.tight_layout()
fig.savefig(OUT / 'fig_occluded_recall_gap.pdf')
fig.savefig(OUT / 'fig_occluded_recall_gap.png', dpi=200)
print('Saved fig_occluded_recall_gap')

# ── Plot 8: Oracle gap closure ────────────────────────────────
fig, ax = plt.subplots(figsize=(5, 3.5))

# For each K, show: random r_occ, causal r_occ, oracle r_occ as grouped bars
k_vals = [2, 4, 8]
random_occ = []
causal_occ = []
oracle_occ = []

for k in k_vals:
    for sel, lst in [('random', random_occ), ('causal', causal_occ), ('oracle_total', oracle_occ)]:
        cfg = f'k_cav={k}_{sel}'
        sub = df[df['config'] == cfg]
        if len(sub) > 0:
            tp_o = sub['det_tp_occluded'].sum()
            fn_o = sub['det_fn_occluded'].sum()
            lst.append(tp_o / (tp_o + fn_o) if (tp_o + fn_o) > 0 else 0)
        else:
            lst.append(0)

x = np.arange(len(k_vals))
w = 0.25
ax.bar(x - w, random_occ, w, label='Random', color='#ff7f0e')
ax.bar(x, causal_occ, w, label='Causal (ours)', color='#2ca02c')
ax.bar(x + w, oracle_occ, w, label='Oracle', color='#9467bd')

# Static all baseline
static_sub = df[df['config'] == 'static_all']
if len(static_sub) > 0:
    tp_o = static_sub['det_tp_occluded'].sum()
    fn_o = static_sub['det_fn_occluded'].sum()
    static_occ = tp_o / (tp_o + fn_o)
    ax.axhline(static_occ, color='#7f7f7f', linestyle='--', linewidth=1.5,
               label=f'Static all (K=22): {static_occ:.3f}')

ax.set_xticks(x)
ax.set_xticklabels([f'K={k}' for k in k_vals])
ax.set_ylabel('RSU-occluded recall')
ax.set_title('Occluded recall: how much of the oracle gap does causal close?')
ax.legend(loc='lower right', fontsize=8)
ax.set_ylim(0, 0.55)

# Annotate gap closure percentages
for i, k in enumerate(k_vals):
    gap = oracle_occ[i] - random_occ[i]
    closed = causal_occ[i] - random_occ[i]
    pct = closed / gap * 100 if gap > 0 else 0
    ax.annotate(f'{pct:.0f}%', (x[i], causal_occ[i] + 0.01),
                ha='center', fontsize=8, fontweight='bold', color='#2ca02c')

fig.tight_layout()
fig.savefig(OUT / 'fig_oracle_gap_closure.pdf')
fig.savefig(OUT / 'fig_oracle_gap_closure.png', dpi=200)
print('Saved fig_oracle_gap_closure')

print('\nAll v2 plots saved.')
