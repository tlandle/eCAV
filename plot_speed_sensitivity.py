#!/usr/bin/env python3
import json, sys
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from json import JSONDecodeError

if len(sys.argv) < 2:
    print("Usage: plot_olt_speed.py <exp_root1> [<exp_root2> …]")
    sys.exit(1)

# Collect experiment roots from CLI args
exp_roots = [Path(p).expanduser() for p in sys.argv[1:]]

rows = []
for root in exp_roots:
    # Derive kind from folder name, e.g. "Local", "Naive", "Beacon"
    kind = root.name.split("_")[0].capitalize()
    for jf in root.rglob("simulation_metrics.json"):
        # Skip any evaluation_output subfolders
        if "evaluation_output" in jf.parts:
            continue
        try:
            data = json.load(open(jf))
        except (JSONDecodeError, OSError) as e:
            print(f"[skip] {jf} – bad JSON: {e}")
            continue

        try:
            oncoming = float(data["oncoming_speed_kmh"])
            success  = float(data.get("success_rate", 0.0))
        except (KeyError, ValueError, TypeError) as e:
            print(f"[skip] {jf} – missing or invalid field: {e}")
            continue

        rows.append({
            "Kind": kind,
            "Oncoming Speed (km/h)": oncoming,
            "Success": success,
        })

# Build DataFrame
df = pd.DataFrame(rows)
if df.empty:
    print("No valid data found – nothing to plot.")
    sys.exit(0)
df = df.sort_values(["Kind", "Oncoming Speed (km/h)"])

# Plotting
sns.set_style("whitegrid")
plt.figure(figsize=(6,4))
sns.lineplot(
    data=df,
    x="Oncoming Speed (km/h)",
    y="Success",
    hue="Kind",
    marker="o",
    err_style="bars"
)

plt.title("Occluded Left Turn –\nSuccess Rate vs Oncoming Vehicle Speed (200 ms Latency)")
plt.xlabel("Oncoming Vehicle Speed (km/h)")
plt.ylabel("Scenario Success Rate")
plt.ylim(0, 1.05)
plt.xticks(np.arange(0, df["Oncoming Speed (km/h)"].max()+1, 5))
plt.tight_layout()
plt.savefig("olt_success_vs_oncoming_speed.png", dpi=300)
print("Wrote olt_success_vs_oncoming_speed.png")
