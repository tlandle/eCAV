"""Single source of paper figures for the SEC submission.

Reads measured CSVs and renders every main-text figure with a shared
style. Run with no args to produce every figure whose input is present;
pass ``--fig 2,3`` to select a subset.

Data inputs (paths relative to repo root):
  paper2/paper2_figures/offline_profile_v3_allrsu_a10.csv
      - Fig 2 (per-stage latency vs N, static policy on A10)
      - Fig 3 (deadline compliance vs N across policies)
      - Fig 7 (per-tick latency boxplot at peak N)
      - Fig 8 (prediction quality vs N, occluded vs visible)
  paper2/reproduce/data/ablation_rsu250_{random,lightweight,causal,
      oracle_total,oracle_occluded,joint}.csv
      - Fig 4 (RSU-occluded recall vs K_cav by selector)
      - Fig 5 (oracle gap closure)
      - Fig 6 (quality-latency Pareto)
  paper2/paper2_figures/closed_loop_matrix.csv   [pending PACE run]
      - Fig 9 (2x2 outcome matrix)
  paper2/paper2_figures/closed_loop_correlation.csv  [pending]
      - Fig 10 (success rate vs high-risk miss rate)

Outputs land in:
  paper2/paper2_figures/<name>.{pdf,png}
  cooperative_world_model_prediction/floats/<name>.pdf   (paper copy)
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Callable

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ---- Paths ---------------------------------------------------------------

REPO = Path("/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox")
PAPER_REPO = Path("/home/atlas/repos/cooperative_world_model_prediction")

OUT_SCRIPT = REPO / "paper2/paper2_figures"
OUT_PAPER = PAPER_REPO / "floats"
OUT_SCRIPT.mkdir(parents=True, exist_ok=True)
OUT_PAPER.mkdir(parents=True, exist_ok=True)

# Aggregate cross-locale sweep across N={4,8,16,24,32}. After the
# patched-profiler all-rsu+joint sweep finishes on the A10 this gets
# overwritten with fresh data that includes the joint policy across N.
ALLRSU_CSV = OUT_SCRIPT / "offline_profile_v3_allrsu_a10.csv"

# Per-locale selector_sweeps CSVs (causal/random/lightweight/oracle_*)
# from the patched-profiler runs. Oracle CSVs are reused from the
# pre-patch sweep because oracle reports recall, not edge timing.
ABLATION_DIR = REPO / "paper2/reproduce/data"

# Single-locale joint detail. Once ALLRSU_CSV has joint policy rows this
# becomes redundant, but keep it as a fallback for figures rendered before
# the all-rsu sweep finishes.
FILTER_JOINT_CSV = ABLATION_DIR / "ablation_rsu250_joint.csv"

CLOSED_LOOP_MATRIX_CSV = OUT_SCRIPT / "closed_loop_matrix.csv"
CLOSED_LOOP_CORR_CSV = OUT_SCRIPT / "closed_loop_correlation.csv"

DEADLINE_MS = 130.0  # internal compute SLO; physical AoI envelope is 300 ms


# ---- Shared style --------------------------------------------------------

plt.rcParams.update({
    "font.size": 10,
    "font.family": "serif",
    "axes.labelsize": 11,
    "axes.titlesize": 12,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "figure.dpi": 150,
})

POLICY_STYLE = {
    "Static":        dict(marker="o", color="#d62728", label="Static"),
    "Filter-only":   dict(marker="s", color="#ff7f0e", label="Filter only"),
    "Pred-only":     dict(marker="^", color="#9467bd", label="Prediction only (amort+risk)"),
    "Joint":         dict(marker="P", color="#2ca02c", label="Joint"),
}

SELECTOR_STYLE = {
    "random":          dict(marker="o", color="#888888", label="Random"),
    "lightweight":     dict(marker="s", color="#1f77b4", label="Feature-norm"),
    "causal_v4":       dict(marker="P", color="#2ca02c", label="Occlusion-aware (ours)"),
    "oracle_occluded": dict(marker="^", color="#ff7f0e", label="Oracle (occluded)"),
}


def _save(fig, name):
    for path in (OUT_SCRIPT, OUT_PAPER):
        fig.savefig(path / f"{name}.pdf", bbox_inches="tight")
    fig.savefig(OUT_SCRIPT / f"{name}.png", dpi=150, bbox_inches="tight")
    print(f"  wrote {name}.pdf/png")
    plt.close(fig)


def _load_controlled_allrsu() -> pd.DataFrame | None:
    """Load the combined controlled-N sweep, mapping (config, policy)
    pairs to one of four named policies for the paper figures:

      Static       - no filter + static MTR (config=static_all, policy=static)
      Filter-only  - causal filter K=4 + static MTR (config=k_cav=4_causal,
                     policy=static)
      Pred-only    - no filter + adaptive MTR (amortization + risk-budget)
                     (config=pred_only_adaptive or policy=amort_only/risk_only)
      Joint        - causal filter + adaptive MTR (config=joint)
    """
    frames = []
    if ALLRSU_CSV.exists():
        frames.append(pd.read_csv(ALLRSU_CSV))
    if FILTER_JOINT_CSV.exists():
        frames.append(pd.read_csv(FILTER_JOINT_CSV))
    if not frames:
        return None
    df = pd.concat(frames, ignore_index=True, sort=False)

    if "n_target" in df.columns:
        # Drop "natural" tag rows but keep rows with missing n_target (joint
        # controller rows don't use --n-sweep; their effective N is the
        # natural zone density).
        df = df[df["n_target"].astype(str) != "natural"].copy()
        df["n_target"] = pd.to_numeric(df["n_target"], errors="coerce")
        # For rows without n_target (joint path), backfill with the measured
        # N so they bin naturally alongside the controlled-sweep rows.
        df["n_target"] = df["n_target"].fillna(df["N"])
        # Effective n_target = min(N, n_target).  When the requested
        # truncation exceeds the natural agent count (e.g. --n-sweep 31 on
        # an RSU whose max is 29), the truncation is a no-op and the row
        # really represents the natural N -- we want it in the dense-tail
        # bin, not dropped.
        df["n_target"] = df[["N", "n_target"]].min(axis=1)
        df["n_target"] = df["n_target"].astype(int)
    else:
        df["n_target"] = df["N"]

    def label(row):
        cfg = str(row.get("config", ""))
        pol = str(row.get("policy", ""))
        if cfg == "joint" or pol == "joint":
            return "Joint"
        if cfg.startswith("k_cav=") and pol == "static":
            return "Filter-only"
        if cfg == "pred_only_adaptive" or pol == "adaptive":
            return "Pred-only"
        if cfg == "static_all" and pol == "static":
            return "Static"
        return None  # unclassified (e.g. legacy amort_only / risk_only rows)
    df["policy_label"] = df.apply(label, axis=1)
    df = df[df["policy_label"].notna()].copy()
    return df


def _load_selector_csvs(locales: tuple[str, ...] = ("rsu250",)) -> pd.DataFrame | None:
    """Load ablation_<locale>_*.csv across the requested locales and parse
    K_cav + selector from the `config` column.

    Config strings take two forms:
      static_all              -> baseline with all CAVs fused (K = N)
      k_cav=<K>_<selector>    -> selector filter retaining K contributors
        where <selector> can be random, lightweight, causal,
        oracle_total, oracle_occluded, etc. (may contain underscores)
    """
    import re
    selectors = ["random", "lightweight", "causal", "causal_v3", "causal_v4",
                 "oracle_total", "oracle_occluded", "joint"]
    dfs = []
    for loc in locales:
        for s in selectors:
            p = ABLATION_DIR / f"ablation_{loc}_{s}.csv"
            if not p.exists():
                continue
            dfs.append(pd.read_csv(p))
    if not dfs:
        return None
    df = pd.concat(dfs, ignore_index=True, sort=False)
    df = df[df["tick"] >= 5].drop_duplicates(["rsu", "config", "tick"]) if "rsu" in df.columns else df[df["tick"] >= 5].drop_duplicates(["config", "tick"])

    # Parse config -> (kcav, selector). static_all rows keep kcav=NaN and
    # selector='static_all'; those are baselines, not selector curves.
    pat = re.compile(r"k_cav=(\d+)_(.+)")
    def parse(cfg):
        if cfg == "static_all":
            return (np.nan, "static_all")
        m = pat.match(str(cfg))
        if not m:
            return (np.nan, "unknown")
        return (int(m.group(1)), m.group(2))
    df[["kcav", "selector"]] = df["config"].apply(
        lambda c: pd.Series(parse(c)))
    # Numeric on the K column for pivot/sort usability.
    df["kcav"] = pd.to_numeric(df["kcav"], errors="coerce")

    # Derive per-row recalls while we have the raw columns in one place.
    df["total_recall"] = df.det_tp / (df.det_tp + df.det_fn).replace(0, np.nan)
    df["occluded_recall"] = df.det_tp_occluded / (
        df.det_tp_occluded + df.det_fn_occluded).replace(0, np.nan)
    return df


# ---- Fig 2: per-stage latency vs N -------------------------------------

def fig2_cliff_per_stage():
    df = _load_controlled_allrsu()
    if df is None:
        print("[fig2] missing allrsu CSV, skipping")
        return
    # Restrict to a single locale so the K_cav axis is apples-to-apples.
    # rsu_250 (Locale A, Town05) has the highest per-stage cost in the
    # dataset and exhibits the sharpest cliff: compliance drops from 100%
    # at K_cav=15 to 44% at K_cav=22 as fusion saturates the deadline.
    # Mixing locales produces non-monotonic cliffs because per-stage
    # characteristics (track density, MTR cost) differ.
    s = df[(df.policy_label == "Static") & (df.rsu == "rsu_250")].copy()
    s["k_cav"] = (s["K"] - 1).astype(int)
    # K_cav values that surround the cliff knee on rsu_250: linear climb
    # below 17, then breach across 19-23.
    target_kcav = [3, 11, 15, 17, 19, 21, 23]
    s = s[s.k_cav.isin(target_kcav)]

    g = s.groupby("k_cav").agg(
        vehicle_bb=("vehicle_backbone_ms", "mean"),
        fusion=("fusion_ms", "mean"),
        detection=("detection_ms", "mean"),
        tracking=("tracking_ms", "mean"),
        prediction=("prediction_ms", "mean"),
        edge_p95=("edge_ms", lambda x: np.percentile(x, 95)),
        n_target_med=("n_target", "median"),
    ).reset_index().sort_values("k_cav")

    def uplink_ms(n):
        return 12 + 0.85 * n
    g["uplink"] = uplink_ms(g.n_target_med)
    g["downlink"] = 9.0
    g["serdes"] = 6.0
    g["consume"] = 25.0
    fig, ax = plt.subplots(figsize=(5.8, 3.5))
    xpos = np.arange(len(g))
    width = 0.62
    components = [
        ("Edge: Fusion",      g.fusion,     "#1f77b4"),
        ("Edge: Detection",   g.detection,  "#9467bd"),
        ("Edge: Tracking",    g.tracking,   "#2ca02c"),
        ("Edge: MTR pred.",   g.prediction, "#d62728"),
        ("Vehicle backbone",  g.vehicle_bb, "#888888"),
        ("Uplink",            g.uplink,     "#1ca3a3"),
        ("Downlink",          g.downlink,   "#8c564b"),
        ("Serialize/des.",    g.serdes,     "#9bb1ff"),
        ("Consume lag",       g.consume,    "#ff7f0e"),
    ]
    bottoms = np.zeros(len(g))
    for label, vals, color in components:
        v = np.asarray(vals.values)
        ax.bar(xpos, v, width, bottom=bottoms, label=label,
               color=color, edgecolor="black", linewidth=0.4)
        bottoms = bottoms + v
    totals = bottoms.copy()  # composed mean per K_cav (sum of stage means + reserve)
    # Composed p95: replace edge_mean (=fusion+det+track+pred) with edge_p95
    # for the edge-side contribution.  Stacked bar shows mean breakdown;
    # diamond marker above shows the p95 that drives envelope compliance.
    edge_mean_sum = g.fusion + g.detection + g.tracking + g.prediction
    composed_p95 = (totals - edge_mean_sum + g.edge_p95).values
    ax.plot(xpos, composed_p95, "D", color="#cc5500", markersize=8,
            markeredgecolor="black", markeredgewidth=0.5,
            label="Composed AoI (p95)", zorder=5)
    for x_i, y_i in zip(xpos, composed_p95):
        ax.annotate(f"{y_i:.0f}", (x_i, y_i),
                    xytext=(0, 5), textcoords="offset points",
                    ha="center", fontsize=8, fontweight="bold",
                    color="#cc5500")
    ax.axhline(300, ls=":", color="#a33", lw=1.4, label="300 ms AoI envelope")
    ax.set_xticks(xpos)
    ax.set_xticklabels([str(int(k)) for k in g.k_cav])
    ax.set_xlabel("Selected CAV contributors $K_{\\mathrm{cav}}$")
    ax.set_ylabel("Composed planner-at-use AoI (ms)")
    ax.set_ylim(0, max(340, composed_p95.max() + 30))
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=6.5, ncol=3, loc="upper center",
              bbox_to_anchor=(0.5, -0.18), frameon=False,
              handlelength=1.4, columnspacing=0.9, handletextpad=0.4)
    fig.tight_layout()
    _save(fig, "fig_cliff_per_stage")


# ---- Fig 3: AoI envelope compliance vs N, by policy --------------------

def fig3_cross_scale_compliance():
    """Compliance against the 300 ms physical AoI envelope, vs N, per policy.

    The compliance metric is the fraction of ticks for which the
    end-to-end planner-at-use AoI stays under the 300 ms physical
    envelope (urban approach speed, ~one lane of brake-margin at
    14 m/s). The 100 ms compute SLO is the controller's internal
    target, not the safety-critical deadline; reporting compliance
    against that smaller threshold understates the joint controller
    because the controller is not built to drive compute below 100 ms,
    it is built to keep AoI under 300 ms.

    Reads aoi_composed.csv (which already includes the network
    components) so that "compliance" reflects what the planner
    actually observes.
    """
    aoi_path = OUT_SCRIPT / "aoi_composed.csv"
    if not aoi_path.exists():
        print(f"[fig3] missing {aoi_path}; skipping")
        return
    df = pd.read_csv(aoi_path)
    policy_col = "policy_label" if "policy_label" in df.columns else "policy"
    if policy_col not in df.columns or "aoi_total_ms" not in df.columns:
        print("[fig3] aoi_composed missing policy_label or aoi_total_ms; skipping")
        return

    AOI_ENVELOPE_MS = 300.0
    N_TARGETS = np.array([4, 8, 16, 24, 28])
    df = df.copy()
    df["aoi_compliant"] = (df["aoi_total_ms"] <= AOI_ENVELOPE_MS).astype(float)
    df["n_bin"] = df["n_target"].apply(
        lambda n: int(N_TARGETS[np.argmin(np.abs(N_TARGETS - n))]))
    # Require enough samples per (policy, N) bin to report a compliance
    # percentage; small bins (e.g. only Pred-only happens to land at N=32
    # with a handful of ticks) produce spurious 100% points.
    MIN_TICKS = 30
    g = (df.groupby([policy_col, "n_bin"])
           .agg(compliance_pct=("aoi_compliant", lambda x: x.mean() * 100),
                n=("aoi_compliant", "size"))
           .reset_index())
    g.loc[g["n"] < MIN_TICKS, "compliance_pct"] = np.nan
    pivot = g.pivot(index="n_bin", columns=policy_col,
                    values="compliance_pct").reindex(N_TARGETS)

    fig, ax = plt.subplots(figsize=(5.8, 2.4))
    x_pos = np.arange(len(N_TARGETS))
    for policy, style in POLICY_STYLE.items():
        if policy not in pivot.columns:
            continue
        ys = pivot[policy].values
        lw = 2.2 if policy == "Joint" else 1.6
        ax.plot(x_pos, ys, **style, lw=lw, markersize=8,
                markeredgecolor="black", markeredgewidth=0.5)
    ax.set_xticks(x_pos)
    ax.set_xticklabels([str(int(n)) for n in N_TARGETS])
    ax.set_xlabel("Cooperating agents $N$")
    ax.set_ylabel("AoI envelope compliance (%)")
    ax.set_ylim(60, 102)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", framealpha=0.95, fontsize=9)
    _save(fig, "fig_cross_scale_compliance")


# ---- Fig 4: selector recall (total + occluded) vs K_cav ----------------

def fig4_selector_quality():
    df = _load_selector_csvs(locales=("rsu93",))
    if df is None:
        print("[fig4] missing ablation CSVs, skipping")
        return
    # Selector curves (K-sweep) vs baselines (static_all = all CAVs).
    sel_df = df[df.kcav.notna()].copy()
    base_df = df[df.selector == "static_all"]
    base_total = base_df.total_recall.mean() if not base_df.empty else np.nan
    base_occ = base_df.occluded_recall.mean() if not base_df.empty else np.nan

    g = sel_df.groupby(["selector", "kcav"])[
        ["total_recall", "occluded_recall"]].mean().reset_index()

    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.6), sharex=True)
    for ax, metric, base_val, title in [
        (axes[0], "total_recall",    base_total, "(a)"),
        (axes[1], "occluded_recall", base_occ,  "(b)")]:
        for sel, style in SELECTOR_STYLE.items():
            sub = g[g.selector == sel].sort_values("kcav")
            if sub.empty:
                continue
            ax.plot(sub.kcav, sub[metric], **style, lw=1.6, markersize=7)
        if np.isfinite(base_val):
            ax.axhline(base_val, ls="--", color="#444", lw=1.1,
                       label=f"Static-all (K=N): {base_val:.2f}")
        ax.set_xlabel("$K_{\\mathrm{cav}}$")
        ax.grid(True, alpha=0.3)
        ax.set_title(title)
    axes[0].set_ylabel("Recall")
    axes[0].legend(loc="lower right", framealpha=0.95, fontsize=8)
    axes[1].legend(loc="lower right", framealpha=0.95, fontsize=8)
    fig.tight_layout()
    _save(fig, "fig_selector_comparison")


# ---- Fig 5: oracle gap closure ----------------------------------------

def fig5_oracle_gap():
    """Causal-selector gap closure vs oracle on RSU-occluded recall.

    Bar height is the fraction of the random-to-oracle gap the causal
    selector closes; the caption-ready table underneath reports the
    absolute recall values so the percentages are interpretable.
    """
    df = _load_selector_csvs()
    if df is None:
        print("[fig5] missing ablation CSVs, skipping")
        return
    sel = df[df.kcav.notna()]
    g = sel.groupby(["selector", "kcav"])["occluded_recall"].mean().unstack("selector")
    if not {"random", "causal", "oracle_occluded"}.issubset(g.columns):
        print("[fig5] need random / causal / oracle_occluded selectors — skipping")
        return

    gap = (g.causal - g.random) / (g.oracle_occluded - g.random) * 100
    # Compact table of absolute recalls for caption readers.
    recall_tbl = g[["random", "causal", "oracle_occluded"]].round(3)

    fig, ax = plt.subplots(figsize=(5.5, 3.8))
    xs = np.arange(len(gap))
    ax.bar(xs, gap.values, color="#2ca02c", edgecolor="black", linewidth=0.6)
    for i, v in enumerate(gap.values):
        if np.isfinite(v):
            ax.text(i, v + 2, f"{v:.0f}%", ha="center", va="bottom", fontsize=9)
    # Absolute recalls below each bar. Skip rows where any selector is
    # missing (prints "rand=nan..." is a reviewer-visible artifact).
    def _fmt(v):
        return "---" if pd.isna(v) else f"{v:.2f}"
    for i, k in enumerate(gap.index):
        r = recall_tbl.loc[k]
        if recall_tbl.loc[k].isna().any():
            continue
        label = (f"rand={_fmt(r.random)}\n"
                 f"causal={_fmt(r.causal)}\n"
                 f"oracle={_fmt(r.oracle_occluded)}")
        ax.text(i, -12, label, ha="center", va="top", fontsize=7, color="#333")
    ax.set_xticks(xs)
    ax.set_xticklabels([str(int(k)) for k in gap.index])
    ax.set_xlabel("$K_{\\mathrm{cav}}$")
    ax.set_ylabel("Random-to-oracle gap closed (%)")
    ax.set_ylim(-35, 110)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    _save(fig, "fig_oracle_gap_closure")


# ---- Fig 6: quality-latency Pareto ------------------------------------

def fig6_pareto():
    """Quality-latency tradeoff across deployable selectors.

    Markers are individual (selector, K_cav) configurations. The shaded
    band marks the deadline-compliant region (mean edge latency below
    the internal compute SLO). Within that region, Causal K_cav=2 sits highest
    on the recall axis, which is the paper's headline operating point.
    Oracle recall is a horizontal upper bound (offline only, no
    deployable latency).
    """
    df = _load_selector_csvs()
    if df is None:
        print("[fig6] missing ablation CSVs, skipping")
        return
    sel = df[df.kcav.notna()]
    g = sel.groupby(["selector", "kcav"]).agg(
        latency=("edge_ms", "mean"),
        occluded_recall=("occluded_recall", "mean"),
        n_ticks=("edge_ms", "size"),
    ).reset_index()
    # Drop (selector, K_cav) points with too few samples to avoid spurious
    # outliers (single-tick configurations leak into the per-K aggregation).
    g = g[g.n_ticks >= 30].reset_index(drop=True)

    deployable = ["random", "lightweight", "causal_v4"]

    fig, ax = plt.subplots(figsize=(5.8, 3.9))

    # Shade the deadline-compliant region.
    ax.axvspan(0, DEADLINE_MS, color="#2ca02c", alpha=0.06,
               zorder=0)
    ax.axvline(DEADLINE_MS, ls=":", color="#555", lw=1.3,
               label=f"{int(DEADLINE_MS)} ms internal compute SLO")

    # Markers only (no connecting lines): each (selector, K) is a discrete
    # configuration, not a continuous sweep.
    for sel_name in deployable:
        sub = g[g.selector == sel_name]
        if sub.empty or sel_name not in SELECTOR_STYLE:
            continue
        st = SELECTOR_STYLE[sel_name]
        ax.scatter(sub.latency, sub.occluded_recall, s=80,
                   color=st["color"], edgecolor="black", linewidth=0.6,
                   marker=st.get("marker", "o"),
                   label=st["label"], zorder=3)
        for _, r in sub.iterrows():
            ax.annotate(f"$K_{{cav}}{{=}}{int(r.kcav)}$",
                        (r.latency, r.occluded_recall),
                        fontsize=7, xytext=(5, 4), textcoords="offset points")

    # Highlight the headline operating point: Causal at K_cav=2.
    causal_k4 = g[(g.selector == "causal_v4") & (g.kcav == 4)]
    if not causal_k4.empty:
        x = float(causal_k4.latency.iloc[0])
        y = float(causal_k4.occluded_recall.iloc[0])
        ax.annotate("Occlusion-aware $K_{cav}{=}4$\n(headline operating point)",
                    (x, y), xytext=(20, -32), textcoords="offset points",
                    fontsize=8, ha="left",
                    arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=1.0))

    # Oracle as a recall-axis upper bound (no latency semantics).
    oracle_rows = g[g.selector == "oracle_occluded"]
    if not oracle_rows.empty:
        oracle_best = oracle_rows.occluded_recall.max()
        ax.axhline(oracle_best, ls="--", color="#d62728", lw=1.3, alpha=0.85,
                   label=f"Oracle upper bound ({oracle_best:.2f})")

    ax.set_xlabel("Mean edge latency (ms)")
    ax.set_ylabel("RSU-occluded recall")
    ax.grid(True, alpha=0.3)
    ax.legend(framealpha=0.95, fontsize=8, loc="lower left")
    fig.tight_layout()
    _save(fig, "fig_pareto")


# ---- Fig 7: per-tick latency distribution ------------------------------

def fig7_latency_boxplot():
    """Per-publish-cycle edge-compute distribution by policy at Locale A
    (N=23). Filter-only and Joint use K_cav=4 for matched-K comparison.

    Single-locale single-N because the all-rsu sweep does not include
    filter-only data (config=k_cav=4_causal + policy=static). Pulling
    Filter-only from the per-locale selector CSV; the other three from
    the joint CSV.
    """
    joint_csv = ABLATION_DIR / "ablation_rsu250_joint.csv"
    causal_csv = ABLATION_DIR / "ablation_rsu250_causal.csv"
    if not joint_csv.exists() or not causal_csv.exists():
        print("[fig7] missing joint or causal selector CSV, skipping")
        return
    j = pd.read_csv(joint_csv)
    j = j[j.tick >= 5]
    c = pd.read_csv(causal_csv)
    c = c[c.tick >= 5]

    # Compose per-cycle AoI: edge + vehicle backbone (max-per-agent) +
    # uplink (linear in K_cav from ns-3 LUT) + downlink (~9 ms multicast) +
    # serialize/deserialize (~6 ms) + planner consume lag (~25 ms mean).
    def aoi_series(df_sub):
        if df_sub.empty:
            return df_sub.edge_ms.values
        kc = df_sub.n_cav_kept.fillna(0).astype(float)
        uplink = 12 + 0.85 * kc
        vbb = df_sub.vehicle_backbone_ms.fillna(8).astype(float)
        return (df_sub.edge_ms.values + vbb.values
                + uplink.values + 9.0 + 6.0 + 25.0)

    series = {
        "Static":      aoi_series(j[(j.config == "static_all")          & (j.policy == "static")]),
        "Filter-only": aoi_series(c[(c.config == "k_cav=4_causal")      & (c.policy == "static")]),
        "Pred-only":   aoi_series(j[(j.config == "pred_only_adaptive")  & (j.policy == "adaptive")]),
        "Joint":       aoi_series(j[(j.config == "joint")               & (j.policy == "joint")]),
    }
    policies = [k for k, v in series.items() if len(v) > 0]
    if not policies:
        print("[fig7] no policy data, skipping")
        return
    labels = [POLICY_STYLE[p]["label"] for p in policies]
    data = [series[p] for p in policies]

    fig, ax = plt.subplots(figsize=(3.4, 3.0))
    bp = ax.boxplot(data, tick_labels=labels, patch_artist=True,
                    showfliers=False, widths=0.55)
    for patch, p in zip(bp["boxes"], policies):
        patch.set_facecolor(POLICY_STYLE[p]["color"])
        patch.set_alpha(0.65)
    ax.axhline(300, ls="--", color="#d62728", lw=1.2,
               label="300 ms AoI envelope")
    ax.set_ylabel("Composed planner-at-use AoI (ms)", fontsize=9)
    ax.tick_params(axis="x", rotation=20, labelsize=8)
    ax.tick_params(axis="y", labelsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax.legend(loc="upper right", fontsize=6.5, framealpha=0.95)
    fig.tight_layout()
    _save(fig, "fig_latency_boxplot")


# ---- Fig 8: prediction + detection quality by risk tier ----------------

def fig8_prediction_quality():
    """Prediction error (ADE) at 0.25 s horizon by policy vs N.

    Bins natural-N data to the controlled targets {4,8,16,24,32} so the
    four policies align on the same x positions (otherwise the natural-N
    Joint and Pred-only curves swing through every integer N=3..28 while
    Static/Filter-only only have data at four points, and the noisy
    per-N samples dominate the visual). The x-axis is categorical so
    each binned N has equal spacing.
    """
    df = _load_controlled_allrsu()
    if df is None:
        print("[fig8] missing allrsu CSV, skipping")
        return
    df = df[df.pred_ade.notna()].copy()

    N_TARGETS = np.array([4, 8, 16, 24, 28])
    df["n_bin"] = df["n_target"].apply(
        lambda n: int(N_TARGETS[np.argmin(np.abs(N_TARGETS - n))]))

    g = df.groupby(["policy_label", "n_bin"]).agg(
        ade_mean=("pred_ade", "mean"),
        ade_p95=("pred_ade", lambda x: np.percentile(x, 95)),
        n_samples=("pred_ade", "count"),
    ).reset_index()

    fig, ax = plt.subplots(figsize=(5.8, 3.8))
    pivot = g.pivot(index="n_bin", columns="policy_label",
                    values="ade_mean").reindex(N_TARGETS)
    x_pos = np.arange(len(N_TARGETS))
    for policy, style in POLICY_STYLE.items():
        if policy not in pivot.columns:
            continue
        lw = 2.2 if policy == "Joint" else 1.5
        ax.plot(x_pos, pivot[policy].values, **style, lw=lw, markersize=8,
                markeredgecolor="black", markeredgewidth=0.5)
    ax.set_xticks(x_pos)
    ax.set_xticklabels([str(int(n)) for n in N_TARGETS])
    ax.set_xlabel("Cooperating agents $N$")
    ax.set_ylabel("Prediction ADE (m) at 0.25 s horizon")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, framealpha=0.95, loc="best")
    fig.tight_layout()
    _save(fig, "fig_prediction_quality")


# ---- Fig 8b: detection coverage (cumulative recall) -----------------------

def fig8b_detection_coverage():
    """Detection recall vs N, aggregated cumulatively (not per-tick mean).

    Per-tick `FN / (TP + FN)` has small-integer denominators and flips
    wildly (0% -> 50% -> 100%) on zones with few GT per tick; mean over
    ticks is misleading. This aggregates `sum(TP)` and `sum(FN)` over
    all ticks per (policy, N) bin, then divides, which is the correct
    way to report detection recall at a scene scale.
    """
    df = _load_controlled_allrsu()
    if df is None:
        print("[fig8b] missing allrsu CSV, skipping")
        return
    g = df.groupby(["policy_label", "n_target"]).agg(
        tp_occ=("det_tp_occluded", "sum"),
        fn_occ=("det_fn_occluded", "sum"),
        tp_vis=("det_tp_visible",  "sum"),
        fn_vis=("det_fn_visible",  "sum"),
    ).reset_index()
    g["recall_occ"] = g.tp_occ / (g.tp_occ + g.fn_occ).replace(0, np.nan)
    g["recall_vis"] = g.tp_vis / (g.tp_vis + g.fn_vis).replace(0, np.nan)

    fig, axes = plt.subplots(1, 2, figsize=(8.8, 3.6), sharex=True, sharey=True)
    for ax, col, title in [
        (axes[0], "recall_occ", "(a)"),
        (axes[1], "recall_vis", "(b)")]:
        pivot = g.pivot(index="n_target", columns="policy_label", values=col)
        for policy, style in POLICY_STYLE.items():
            if policy not in pivot.columns:
                continue
            lw = 2.2 if policy == "Joint" else 1.5
            ax.plot(pivot.index, pivot[policy], **style, lw=lw, markersize=6)
        ax.set_xscale("log", base=2)
        ax.set_xticks(pivot.index.tolist())
        ax.set_xticklabels([str(int(n)) for n in pivot.index])
        ax.set_xlabel("Cooperating agents $N$")
        ax.grid(True, alpha=0.3)
        ax.set_title(title, fontsize=10)
        ax.set_ylim(0, 1)
    axes[0].set_ylabel("Recall")
    axes[0].legend(fontsize=8, framealpha=0.95, loc="best")
    fig.tight_layout()
    _save(fig, "fig_detection_coverage")


# ---- Fig 9: closed-loop 2x2 matrix (pending data) ---------------------

def fig_aoi_vs_n():
    """Planner-at-use AoI vs N, per policy, with component decomposition.

    Reads aoi_composed.csv produced by compose_aoi.py. Left panel: p95 AoI
    vs N, one curve per policy. Reference lines at the 130 ms internal
    compute SLO and the 300 ms physical-envelope bound. Right panel:
    stacked-bar decomposition at a representative dense N (N=24).
    """
    aoi_path = OUT_SCRIPT / "aoi_composed.csv"
    if not aoi_path.exists():
        print(f"[fig_aoi] waiting on {aoi_path}; skipping")
        return
    df = pd.read_csv(aoi_path)
    policy_col = "policy_label" if "policy_label" in df.columns else "policy"
    if policy_col not in df.columns or "n_target" not in df.columns:
        print("[fig_aoi] missing policy_label / n_target; skipping")
        return

    # Bin n_target into the shared fixed targets so the Joint policy
    # (measured at natural zone density, so its natural N lands at many
    # values between 5 and 29) aligns with Static / Filter-only /
    # Pred-only (run with --n-sweep {4,8,16,24,32}). Joint's min observed
    # N is 5, so the N=4 bin is honestly empty for Joint.
    N_TARGETS = np.array([4, 8, 16, 24, 28])
    def bin_nearest(n):
        return int(N_TARGETS[np.argmin(np.abs(N_TARGETS - n))])
    df = df.copy()
    df["n_bin"] = df["n_target"].apply(bin_nearest)

    # Percentile AoI per (policy, N_bin).
    g = df.groupby([policy_col, "n_bin"]).agg(
        aoi_p50=("aoi_total_ms", lambda x: np.percentile(x, 50)),
        aoi_p95=("aoi_total_ms", lambda x: np.percentile(x, 95)),
        edge_mean=("edge_ms", "mean"),
        ul_mean=("network_ul_ms", "mean"),
        dl_mean=("network_dl_ms", "mean"),
        ser_mean=("serialize_feature_ms", "mean"),
        des_mean=("deserialize_feature_ms", "mean"),
        pred_ser_mean=("serialize_pred_ms", "mean"),
        pred_des_mean=("deserialize_pred_ms", "mean"),
        consume_mean=("consume_lag_ms", "mean"),
        n_ticks=("aoi_total_ms", "count"),
    ).reset_index().rename(columns={"n_bin": "n_target"})
    # Drop bins with too few samples; otherwise tiny outliers (e.g. only
    # Pred-only with 7 ticks at the dense tail) produce spurious points.
    g = g[g.n_ticks >= 30].reset_index(drop=True)

    fig, axes = plt.subplots(2, 1, figsize=(3.4, 5.4))

    # (a) p95 AoI vs N per policy.
    ax = axes[0]
    piv = g.pivot(index="n_target", columns=policy_col, values="aoi_p95")
    for policy, style in POLICY_STYLE.items():
        if policy not in piv.columns:
            continue
        lw = 2.0 if policy == "Joint" else 1.4
        ax.plot(piv.index, piv[policy], **style, lw=lw, markersize=5)
    ax.axhline(300.0, ls="--", color="#d62728", lw=1.1,
               label="300 ms envelope")
    ax.set_xscale("log", base=2)
    ax.set_xticks(piv.index.tolist())
    ax.set_xticklabels([str(int(n)) for n in piv.index])
    ax.set_xlabel("Cooperating agents $N$", fontsize=9)
    ax.set_ylabel("p95 AoI (ms)", fontsize=9)
    ax.set_title("(a)", fontsize=9, loc="left")
    ax.tick_params(axis="both", labelsize=8)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=6.5, ncol=2, loc="upper center",
              bbox_to_anchor=(0.5, -0.28), frameon=False, handlelength=1.6,
              columnspacing=1.0, handletextpad=0.4)

    # (b) Stacked decomposition at a representative dense N.
    ax = axes[1]
    target_N = 24 if 24 in g.n_target.unique() else int(g.n_target.max())
    sub = g[g.n_target == target_N].set_index(policy_col)
    components = [
        ("Ser+Des",
         lambda r: r.ser_mean + r.des_mean + r.pred_ser_mean + r.pred_des_mean,
         "#9467bd"),
        ("Uplink", lambda r: r.ul_mean, "#1f77b4"),
        ("Edge compute", lambda r: r.edge_mean, "#d62728"),
        ("Downlink", lambda r: r.dl_mean, "#2ca02c"),
        ("Consume lag", lambda r: r.consume_mean, "#ff7f0e"),
    ]
    policies_order = [p for p in POLICY_STYLE if p in sub.index]
    bottoms = np.zeros(len(policies_order))
    for label, getter, color in components:
        vals = np.array([getter(sub.loc[p]) for p in policies_order])
        ax.bar(policies_order, vals, bottom=bottoms, label=label,
               color=color, edgecolor="black", linewidth=0.4)
        bottoms = bottoms + vals
    # p95 markers: mean bars can sit comfortably under 300 ms even when p95
    # breaches.  Show both so the breaching policies are visible.
    p95_vals = np.array([sub.loc[p, "aoi_p95"] for p in policies_order])
    xpos_b = np.arange(len(policies_order))
    ax.plot(xpos_b, p95_vals, "D", color="#cc5500", markersize=7,
            markeredgecolor="black", markeredgewidth=0.5, zorder=5,
            label="AoI p95")
    for x_i, y_i in zip(xpos_b, p95_vals):
        ax.annotate(f"{y_i:.0f}", (x_i, y_i),
                    xytext=(0, 4), textcoords="offset points",
                    ha="center", fontsize=7, fontweight="bold",
                    color="#cc5500")
    ax.axhline(300.0, ls="--", color="#d62728", lw=1.0)
    ax.set_ylim(0, max(340, p95_vals.max() + 30))
    ax.set_ylabel("AoI component (ms)", fontsize=9)
    ax.set_title(f"(b) $N={target_N}$", fontsize=9, loc="left")
    ax.tick_params(axis="x", rotation=15, labelsize=8)
    ax.tick_params(axis="y", labelsize=8)
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=6.5, ncol=5, loc="upper center",
              bbox_to_anchor=(0.5, -0.22), frameon=False, handlelength=1.2,
              columnspacing=0.7, handletextpad=0.3)

    fig.tight_layout()
    fig.subplots_adjust(hspace=0.85)
    _save(fig, "fig_aoi_vs_n")


def fig9_closed_loop_matrix():
    if not CLOSED_LOOP_MATRIX_CSV.exists():
        print("[fig9] waiting on closed-loop data; skipping")
        return
    df = pd.read_csv(CLOSED_LOOP_MATRIX_CSV)
    has_ttc = "min_ttc_s" in df.columns
    # 2x2 grid sized to fit a single IEEE column. Visibility on rows
    # (de-occluded / occluded), load on columns (low N=4 / stressed N=12).
    fig, axes = plt.subplots(2, 2, figsize=(3.4, 3.2),
                             sharey=True, sharex=True)
    axes = axes.flatten()
    cells = [
        ("deocc", "low",  "De-occ., low ($N{=}4$)"),
        ("deocc", "high", "De-occ., stressed ($N{=}12$)"),
        ("occ",   "low",  "Occluded, low ($N{=}4$)"),
        ("occ",   "high", "Occluded, stressed ($N{=}12$)"),
    ]
    method_order = ["local_lf", "static_wf", "adaptive_wf"]
    method_labels = ["LF", "Static", "Adaptive"]
    method_colors = ["#888888", "#3a7ec1", "#c33a3a"]
    for ax, (vis, load, title) in zip(axes, cells):
        sub = df[(df.visibility == vis) & (df.load == load)]
        vals = [sub[sub.method == m].success_rate.mean() for m in method_order]
        ttcs = [sub[sub.method == m].min_ttc_s.mean() if has_ttc else float("nan")
                for m in method_order]
        bars = ax.bar(range(len(method_order)), vals,
                      color=method_colors, edgecolor="black", linewidth=0.5)
        for bar, val, ttc in zip(bars, vals, ttcs):
            if np.isnan(val):
                continue
            label = f"{val:.2f}\n({ttc:.1f}s)" if has_ttc and not np.isnan(ttc) \
                else f"{val:.2f}"
            ax.text(bar.get_x() + bar.get_width() / 2, val + 0.02,
                    label, ha="center", va="bottom", fontsize=7)
        ax.set_xticks(range(len(method_order)))
        ax.set_xticklabels(method_labels, fontsize=8)
        ax.set_ylim(0, 1.20)
        ax.set_yticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.set_title(title, fontsize=8.5)
        ax.grid(axis="y", alpha=0.3)
    for ax in (axes[0], axes[2]):
        ax.set_ylabel("Success rate", fontsize=8)
    fig.tight_layout()
    _save(fig, "fig_closed_loop_matrix")


def fig10_closed_loop_correlation():
    if not CLOSED_LOOP_CORR_CSV.exists():
        print("[fig10] waiting on closed-loop data; skipping")
        return
    df = pd.read_csv(CLOSED_LOOP_CORR_CSV)
    if "aoi_p99_ms" not in df.columns:
        print("[fig10] correlation CSV missing aoi_p99_ms; skipping")
        return
    fig, ax = plt.subplots(figsize=(3.6, 3.2))
    method_style = {
        "local_lf":    ("#888888", "o", "Late fusion"),
        "static_wf":   ("#3a7ec1", "s", "Static cooperative"),
        "adaptive_wf": ("#c33a3a", "^", "Adaptive cooperative"),
    }
    load_size = {"low": 110, "high": 230}
    # Envelope shading: above 300 ms is the unsafe region
    ax.axvspan(300, 600, color="red", alpha=0.07, zorder=0)
    ax.axvline(300, color="red", linestyle="--", linewidth=1.2,
               label=r"physical envelope $Z=300$ ms", zorder=1)
    for method, (color, marker, label) in method_style.items():
        for vis in ["deocc", "occ"]:
            for load, size in load_size.items():
                sub = df[(df.method == method)
                         & (df.visibility == vis)
                         & (df.load == load)]
                if sub.empty:
                    continue
                ax.scatter(sub.aoi_p99_ms, sub.success_rate * 100,
                           s=size, marker=marker, color=color,
                           edgecolor="black", linewidth=0.6,
                           alpha=0.92, zorder=3)
                # Per-point condition annotation
                for _, r in sub.iterrows():
                    cond = ("clear" if r.visibility == "deocc" else "occ") \
                           + "/" + ("low" if r.load == "low" else "stress")
                    ax.annotate(cond, (r.aoi_p99_ms, r.success_rate * 100),
                                xytext=(7, 4), textcoords="offset points",
                                fontsize=8, color="#333", zorder=4)
    # Compact legend in the upper-right (high-AoI / high-success corner stays
    # empty by construction since failures live at high AoI / low success).
    method_handles = [plt.Line2D([0], [0], marker=m, color="w",
                                 markerfacecolor=c, markersize=6,
                                 markeredgecolor="black", linewidth=0,
                                 label=l)
                      for c, m, l in method_style.values()]
    method_handles.append(plt.Line2D([0], [0], marker="o", color="w",
                                     markerfacecolor="gray", markersize=4,
                                     markeredgecolor="black", linewidth=0,
                                     label=r"low $N{=}4$"))
    method_handles.append(plt.Line2D([0], [0], marker="o", color="w",
                                     markerfacecolor="gray", markersize=7,
                                     markeredgecolor="black", linewidth=0,
                                     label=r"stress $N{=}12$"))
    ax.legend(handles=method_handles, loc="upper right",
              fontsize=7.5, framealpha=0.92, ncol=1, handlelength=1.2,
              borderpad=0.4, labelspacing=0.3)
    ax.set_xlabel("Edge AoI p99 at planner use time (ms)")
    ax.set_ylabel("Scenario success rate (%)")
    ax.set_ylim(-5, 105)
    ax.set_xlim(50, 550)
    ax.grid(True, alpha=0.3, zorder=0)
    fig.tight_layout()
    _save(fig, "fig_closed_loop_correlation")


# ---- Driver -----------------------------------------------------------

FIGS: dict[int, Callable[[], None]] = {
    2: fig2_cliff_per_stage,
    3: fig3_cross_scale_compliance,
    4: fig4_selector_quality,
    5: fig5_oracle_gap,
    6: fig6_pareto,
    7: fig7_latency_boxplot,
    8: fig8_prediction_quality,
    81: fig8b_detection_coverage,
    82: fig_aoi_vs_n,
    9: fig9_closed_loop_matrix,
    10: fig10_closed_loop_correlation,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fig", default="all",
                    help="Comma-separated figure numbers or 'all' (default).")
    args = ap.parse_args()
    if args.fig == "all":
        selected = sorted(FIGS)
    else:
        selected = [int(x) for x in args.fig.split(",") if x.strip()]
    for n in selected:
        fn = FIGS.get(n)
        if fn is None:
            print(f"[skip] unknown figure {n}")
            continue
        print(f"[fig {n}] {fn.__name__}")
        try:
            fn()
        except Exception as e:
            print(f"  FAILED: {e}")
            import traceback; traceback.print_exc()


if __name__ == "__main__":
    main()
