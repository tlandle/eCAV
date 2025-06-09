#!/usr/bin/env python3
import json, argparse, pandas as pd, matplotlib.pyplot as plt
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument("exp_root")
args = p.parse_args()

rows = []
for jf in Path(args.exp_root).rglob("simulation_metrics.json"):
    m = json.load(open(jf))
    rows.append({
        "latency"     : m["edge_latency_s"],
        "target_spd"  : m.get("target_speed_kmh"),  # if you recorded it
        "avg_speed"   : m["vehicles"]["1169"]["avg_speed_mps"] * 3.6,  # km/h
        "success"     : m["success_rate"],
    })
df = pd.DataFrame(rows)

# ───────── plots ──────────────────────────────────────────────────────────
plt.figure(); sns.lineplot(data=df, x="latency", y="success", marker="o")
plt.title("Scenario Success vs Latency"); plt.ylabel("Success Rate"); plt.savefig("success_vs_latency.png")

plt.figure(); sns.lineplot(data=df, x="target_spd", y="success", marker="o")
plt.title("Scenario Success vs Target Speed"); plt.ylabel("Success Rate"); plt.savefig("success_vs_target.png")

plt.figure(); sns.lineplot(data=df, x="avg_speed", y="success", marker="o")
plt.title("Scenario Success vs Average Speed"); plt.ylabel("Success Rate"); plt.savefig("success_vs_avg.png")

lat_groups = df.groupby("latency")
stack = pd.DataFrame({
    "comm":   lat_groups["latency"].first(),   # replace with metrics you stored
    "track":  lat_groups["latency"].first(),
    "pred":   lat_groups["latency"].first(),
})
succ = lat_groups["success"].mean()

fig, ax1 = plt.subplots()
stack.plot(kind="bar", stacked=True, ax=ax1, width=0.6,
           color=["skyblue", "gold", "salmon"])
ax1.set_ylabel("Latency (ms)")
ax2 = ax1.twinx()
ax2.plot(succ.index.astype(str), succ.values, "k--o", label="Success Rate")
ax2.set_ylim(0, 1.05); ax2.set_ylabel("Scenario Success Rate")
fig.tight_layout(); fig.savefig("latency_breakdown.png")
