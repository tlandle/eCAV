"""Generate paper-ready evaluation plots from ablation data."""
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

OUT = Path('paper2_figures')
OUT.mkdir(exist_ok=True)

# Style
plt.rcParams.update({
    'font.size': 10, 'font.family': 'serif',
    'axes.labelsize': 11, 'axes.titlesize': 12,
    'xtick.labelsize': 9, 'ytick.labelsize': 9,
    'legend.fontsize': 8, 'figure.dpi': 150,
})
COLORS = {
    'RSU only': '#1f77b4',
    'Random': '#ff7f0e',
    'Feature-norm': '#d62728',
    'Causal (ours)': '#2ca02c',
    'Oracle': '#9467bd',
    'Joint': '#8c564b',
    'Static all': '#7f7f7f',
}

# ── Load data ─────────────────────────────────────────────────
dfs = []
for f in ['ablation_rsu250_lightweight', 'ablation_rsu250_random',
          'ablation_rsu250_causal', 'ablation_rsu250_oracle_total',
          'ablation_rsu250_oracle_occluded', 'ablation_rsu250_joint']:
    try:
        dfs.append(pd.read_csv(OUT / f'{f}.csv'))
    except FileNotFoundError:
        pass
df = pd.concat(dfs, ignore_index=True)
df = df[df['tick'] >= 5].drop_duplicates(['config', 'tick'])

def agg(df_sub):
    return pd.Series({
        'cav_kept': df_sub['n_cav_kept'].mean(),
        'recall': df_sub['det_tp'].sum() / (df_sub['det_tp'].sum() + df_sub['det_fn'].sum()),
        'r_occ': df_sub['det_tp_occluded'].sum() / max(df_sub['det_tp_occluded'].sum() + df_sub['det_fn_occluded'].sum(), 1),
        'edge_mean': df_sub['edge_ms'].mean(),
        'edge_p95': np.percentile(df_sub['edge_ms'], 95),
        'fusion_ms': df_sub['fusion_ms'].mean(),
        'filter_ms': df_sub['filter_ms'].mean(),
        'det_ms': df_sub['detection_ms'].mean(),
        'trk_ms': df_sub['tracking_ms'].mean(),
        'pred_ms': df_sub['prediction_ms'].mean(),
        'miss_rate': (df_sub['edge_ms'] > 100).mean(),
    })

g = df.groupby('config').apply(agg).reset_index()

# ── Plot 1: Deadline Cliff (stacked bar) ──────────────────────
fig, ax = plt.subplots(figsize=(6, 3.5))
# Use static data at different K values
cliff_configs = ['k_cav=0_lightweight', 'k_cav=2_random', 'k_cav=4_random',
                 'k_cav=8_random', 'static_all']
cliff_labels = ['RSU only\n(K=0)', 'K=2', 'K=4', 'K=8', 'All\n(K=22)']
cliff_data = g[g['config'].isin(cliff_configs)].set_index('config').loc[cliff_configs]

x = np.arange(len(cliff_labels))
w = 0.6
bottom = np.zeros(len(x))
stages = [('filter_ms', 'Filter', '#1f77b4'),
          ('fusion_ms', 'Fusion', '#ff7f0e'),
          ('det_ms', 'Detection', '#2ca02c'),
          ('trk_ms', 'Tracking', '#d62728'),
          ('pred_ms', 'Prediction', '#9467bd')]
for col, label, color in stages:
    vals = cliff_data[col].values
    ax.bar(x, vals, w, bottom=bottom, label=label, color=color)
    bottom += vals

ax.axhline(100, color='black', linestyle='--', linewidth=1, alpha=0.7, label='100ms deadline')
ax.set_ylabel('Edge latency (ms)')
ax.set_xticks(x)
ax.set_xticklabels(cliff_labels)
ax.set_xlabel('Contributors fused')
ax.legend(loc='upper left', ncol=2, framealpha=0.9)
ax.set_title('Per-stage latency scaling (rsu_250, N=23)')
fig.tight_layout()
fig.savefig(OUT / 'fig_deadline_cliff.pdf')
fig.savefig(OUT / 'fig_deadline_cliff.png', dpi=200)
print('Saved fig_deadline_cliff')

# ── Plot 2: Selector Comparison (recall vs K_cav) ─────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.8))

selectors = {
    'Feature-norm': ['k_cav=2_lightweight', 'k_cav=4_lightweight', 'k_cav=8_lightweight'],
    'Random': ['k_cav=2_random', 'k_cav=4_random', 'k_cav=8_random'],
    'Causal (ours)': ['k_cav=2_causal', 'k_cav=4_causal', 'k_cav=8_causal'],
    'Oracle': ['k_cav=2_oracle_total', 'k_cav=4_oracle_total', 'k_cav=8_oracle_total'],
}
k_vals = [2, 4, 8]

for sel_name, configs in selectors.items():
    sel_data = g[g['config'].isin(configs)].set_index('config').loc[configs]
    ax1.plot(k_vals, sel_data['recall'].values, 'o-', label=sel_name,
             color=COLORS.get(sel_name, 'gray'), linewidth=2, markersize=6)
    ax2.plot(k_vals, sel_data['r_occ'].values, 'o-', label=sel_name,
             color=COLORS.get(sel_name, 'gray'), linewidth=2, markersize=6)

# Add RSU-only and static-all baselines
rsu_row = g[g['config'] == 'k_cav=0_lightweight'].iloc[0]
static_row = g[g['config'] == 'static_all'].iloc[0]

for ax in [ax1, ax2]:
    metric = 'recall' if ax == ax1 else 'r_occ'
    ax.axhline(rsu_row[metric], color=COLORS['RSU only'], linestyle=':', linewidth=1.5, alpha=0.8)
    ax.axhline(static_row[metric], color=COLORS['Static all'], linestyle='--', linewidth=1.5, alpha=0.8)
    ax.text(8.3, rsu_row[metric], 'RSU only', fontsize=7, va='center', color=COLORS['RSU only'])
    ax.text(8.3, static_row[metric], 'All (K=22)', fontsize=7, va='center', color=COLORS['Static all'])

ax1.set_xlabel('$K_{cav}$ (CAVs selected beyond RSU)')
ax1.set_ylabel('Total recall')
ax1.set_title('(a) Total recall vs contributor budget')
ax1.set_xticks(k_vals)
ax1.legend(loc='lower right')

ax2.set_xlabel('$K_{cav}$ (CAVs selected beyond RSU)')
ax2.set_ylabel('RSU-occluded recall')
ax2.set_title('(b) Occluded recall vs contributor budget')
ax2.set_xticks(k_vals)
ax2.legend(loc='lower right')

fig.tight_layout()
fig.savefig(OUT / 'fig_selector_comparison.pdf')
fig.savefig(OUT / 'fig_selector_comparison.png', dpi=200)
print('Saved fig_selector_comparison')

# ── Plot 3: Quality-Latency Pareto ────────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3.8))

all_configs = {
    'RSU only': 'k_cav=0_lightweight',
    'Random K=2': 'k_cav=2_random',
    'Random K=4': 'k_cav=4_random',
    'Random K=8': 'k_cav=8_random',
    'Feature-norm K=2': 'k_cav=2_lightweight',
    'Feature-norm K=4': 'k_cav=4_lightweight',
    'Feature-norm K=8': 'k_cav=8_lightweight',
    'Causal K=2': 'k_cav=2_causal',
    'Causal K=4': 'k_cav=4_causal',
    'Causal K=8': 'k_cav=8_causal',
    'Joint': 'joint',
    'Static all': 'static_all',
}

for label, cfg in all_configs.items():
    row = g[g['config'] == cfg]
    if row.empty:
        continue
    row = row.iloc[0]
    if 'Random' in label:
        color, marker = COLORS['Random'], 's'
    elif 'Feature' in label:
        color, marker = COLORS['Feature-norm'], 'x'
    elif 'Causal' in label:
        color, marker = COLORS['Causal (ours)'], 'D'
    elif 'Joint' in label:
        color, marker = COLORS['Joint'], '*'
    elif 'RSU' in label:
        color, marker = COLORS['RSU only'], 'o'
    else:
        color, marker = COLORS['Static all'], '^'

    ax1.scatter(row['edge_mean'], row['recall'], c=color, marker=marker, s=80, zorder=5)
    ax2.scatter(row['edge_mean'], row['r_occ'], c=color, marker=marker, s=80, zorder=5)

    if label in ['RSU only', 'Joint', 'Static all', 'Causal K=2']:
        for ax, metric in [(ax1, 'recall'), (ax2, 'r_occ')]:
            ax.annotate(label, (row['edge_mean'], row[metric]),
                       textcoords='offset points', xytext=(5, 5), fontsize=7)

for ax in [ax1, ax2]:
    ax.axvline(100, color='black', linestyle='--', linewidth=1, alpha=0.5)
    ax.set_xlabel('Edge latency (ms)')

ax1.set_ylabel('Total recall')
ax1.set_title('(a) Quality-latency tradeoff: total recall')
ax2.set_ylabel('RSU-occluded recall')
ax2.set_title('(b) Quality-latency tradeoff: occluded recall')

# Legend
from matplotlib.lines import Line2D
handles = [
    Line2D([0], [0], marker='o', color='w', markerfacecolor=COLORS['RSU only'], markersize=8, label='RSU only'),
    Line2D([0], [0], marker='s', color='w', markerfacecolor=COLORS['Random'], markersize=8, label='Random'),
    Line2D([0], [0], marker='x', color='w', markeredgecolor=COLORS['Feature-norm'], markersize=8, label='Feature-norm'),
    Line2D([0], [0], marker='D', color='w', markerfacecolor=COLORS['Causal (ours)'], markersize=8, label='Causal (ours)'),
    Line2D([0], [0], marker='*', color='w', markerfacecolor=COLORS['Joint'], markersize=10, label='Joint'),
    Line2D([0], [0], marker='^', color='w', markerfacecolor=COLORS['Static all'], markersize=8, label='Static all'),
]
ax2.legend(handles=handles, loc='lower right', fontsize=7)

fig.tight_layout()
fig.savefig(OUT / 'fig_pareto.pdf')
fig.savefig(OUT / 'fig_pareto.png', dpi=200)
print('Saved fig_pareto')

# ── Plot 4: Joint Controller Budget Allocation ────────────────
joint_df = pd.read_csv(OUT / 'ablation_rsu250_joint.csv')
joint_df = joint_df[joint_df['config'] == 'joint']

if len(joint_df) > 5:
    fig, ax = plt.subplots(figsize=(6, 3.5))
    ticks = joint_df['tick'].values
    ax.stackplot(ticks,
                 joint_df['filter_ms'].values,
                 joint_df['fusion_ms'].values,
                 joint_df['detection_ms'].values,
                 joint_df['tracking_ms'].values,
                 joint_df['prediction_ms'].values,
                 labels=['Filter', 'Fusion', 'Detection', 'Tracking', 'Prediction'],
                 colors=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd'],
                 alpha=0.8)
    ax.axhline(100, color='black', linestyle='--', linewidth=1, label='100ms deadline')
    ax.set_xlabel('Tick')
    ax.set_ylabel('Edge latency (ms)')
    ax.set_title('Joint controller: per-tick stage breakdown')
    ax.legend(loc='upper right', ncol=2, fontsize=7)
    ax.set_ylim(0, 150)
    fig.tight_layout()
    fig.savefig(OUT / 'fig_joint_breakdown.pdf')
    fig.savefig(OUT / 'fig_joint_breakdown.png', dpi=200)
    print('Saved fig_joint_breakdown')

print('\nAll plots saved to paper2_figures/')
