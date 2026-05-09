# SEC 2026 Paper: Reproduction Guide

This directory contains the data and scripts needed to reproduce every
figure and table in the SEC 2026 WorldFusion paper.

## Directory layout

```
paper2/reproduce/
├── README.md                       # This file
├── data/                           # CSV outputs from the offline profiler
│   ├── ablation_rsu250_causal.csv          # Zone A, causal selector, K sweep
│   ├── ablation_rsu250_random.csv          # Zone A, random selector, K sweep
│   ├── ablation_rsu250_lightweight.csv     # Zone A, feature-norm selector
│   ├── ablation_rsu250_oracle_total.csv    # Zone A, oracle (total recall)
│   ├── ablation_rsu250_oracle_occluded.csv # Zone A, oracle (occluded recall)
│   ├── ablation_rsu250_joint.csv           # Zone A, static + joint + pred_only
│   ├── ablation_rsu40_causal.csv           # Zone B versions
│   ├── ablation_rsu40_random.csv
│   ├── ablation_rsu40_lightweight.csv
│   ├── ablation_mechanisms.csv             # amort_only + risk_only (Zone A)
│   └── multiv2x_n_per_zone.csv             # N distribution across 56 zones
└── scripts/                        # Wrapper scripts that invoke the profiler
    ├── run_all.sh                          # Reproduce everything end-to-end
    ├── run_selector_sweeps.sh              # All selector + K sweeps
    ├── run_joint_ablation.sh               # static + joint + pred_only
    ├── run_mechanism_ablation.sh           # amort_only + risk_only
    └── run_n_histogram.sh                  # Multi-V2X N distribution
```

The actual profiler implementation lives in `paper2/paper2_offline_profiler_v3.py`.
Plot scripts live in `paper2/paper2_plot_evaluation.py` and
`paper2/paper2_plot_evaluation_v2.py`.


## What each CSV produces

| CSV | Paper artifact |
|-----|----------------|
| `ablation_rsu250_*` | Fig. selector_comparison, Fig. pareto, Fig. oracle_gap_closure, Fig. deadline_cliff |
| `ablation_rsu250_joint.csv` | Fig. deadline_compliance, Fig. latency_boxplot, joint-controller results |
| `ablation_mechanisms.csv` | Mechanism ablation table (Tab. mech-ablation) |
| `ablation_rsu40_*` | Cross-zone validation text (Zone B) |
| `multiv2x_n_per_zone.csv` | N distribution figure and text |


## Reproducing from scratch

1. Multi-V2X dataset must be at `/data1/Datasets/Multi-V2X` (NVMe).
2. Activate the `opencda310` conda env.
3. Run `bash paper2/reproduce/scripts/run_all.sh`.

Hardware assumption: NVIDIA A10 24 GB (or equivalent). On a 16 GB card
(e.g. RTX 4080 SUPER) you must close any other GPU consumers (CARLA
etc.) before running. Running all ablations takes approximately 4 hours
on an A10.


## Invoking the profiler directly

The profiler's relevant flags:

- `--data-root PATH`: Multi-V2X dataset root
- `--town NAME`: town subdirectory name (e.g.
  `Town05__2023_11_13_23_03_07`)
- `--rsu NAME`: RSU zone (e.g. `rsu_250` for Zone A, `rsu_40` for Zone B)
- `--max-frames N`: per-config frame budget (we use 50)
- `--sweep-k-cav "0,2,4,8"`: K_cav values to sweep for the selector
- `--filter-mode MODE`: `causal`, `random`, `lightweight`,
  `oracle_total`, `oracle_occluded`, or `backbone`
- `--joint`: additionally runs the joint controller and pred_only_adaptive
- `--out PATH`: output CSV

The `paper2_ablation_mechanisms.py` script calls `profile_config`
directly to isolate `amort_only` and `risk_only` predictor policies,
which the main CLI does not expose.


## Non-determinism

The profiler is deterministic given a fixed frame window. However:

- `max_frames` chooses a centered window, so different values pick
  different frames.
- Default is 50. All results in the paper use 50 frames starting from
  the middle of each zone sequence.
- Tracking state warms up over the first 5 ticks; all reported metrics
  use `tick >= 5` only.
