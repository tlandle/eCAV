#!/usr/bin/env python3
"""Reviewer C4/C5 on the measured April closed-loop campaign: per-configuration
P(S_op) with Wilson 95% intervals, and tau_max at thresholds 0.8/0.9/0.95/0.99,
using the paper's own focal-vehicle success definition (scripts/recompute_metrics.compute_run_metrics)."""
import sys, os, re, json, glob, math, collections
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from recompute_metrics import compute_run_metrics
PAT = re.compile(r"mgr_(?P<mgr>.+?)__anchoring_(?P<anch>on|off)__lat_(?P<lat>\d+)__ego_(?P<n>\d+)__run_(?P<run>\d+)__openscenario_(?P<scn>\w+)$")
THR = (0.8, 0.9, 0.95, 0.99)
def wilson(k, n, z=1.96):
    p = k/n; d = 1 + z*z/n; c = (p + z*z/(2*n))/d; h = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/d
    return p, c-h, c+h
camp = sys.argv[1] if len(sys.argv) > 1 else '20260419_120000'
cells = collections.defaultdict(list)
for f in glob.glob(os.path.join(ROOT, 'safety_envelope_paper', 'experiment_results', camp, '*', 'simulation_metrics.json')):
    m = PAT.match(os.path.basename(os.path.dirname(f)))
    if not m: continue
    g = m.groupdict()
    try: s = compute_run_metrics(json.load(open(f)))['s_op']
    except Exception as e: continue
    cells[(g['mgr'], g['anch'], g['scn'], int(g['n']), int(g['lat']))].append(int(s))
out = [f"# {camp}: Wilson intervals and tau_max (focal-vehicle S_op)\n", "| mgr | contract | scenario | N | lat | seeds | P(S_op) | 95% CI |", "|---|---|---|---|---|---|---|---|"]
for k in sorted(cells):
    v = cells[k]; p, lo, hi = wilson(sum(v), len(v))
    out.append(f"| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {k[4]} | {len(v)} | {p:.2f} | [{lo:.2f}, {hi:.2f}] |")
out += ["\n## tau_max: largest configured latency with P(S_op) >= threshold\n", "| mgr | contract | scenario | N | " + " | ".join(f"{t}" for t in THR) + " |", "|---|---|---|---|" + "---|"*len(THR)]
groups = collections.defaultdict(dict)
for (mgr, anch, scn, n, lat), v in cells.items(): groups[(mgr, anch, scn, n)][lat] = sum(v)/len(v)
for gk in sorted(groups):
    row = [str(max([l for l, p in groups[gk].items() if p >= t], default='none')) for t in THR]
    out.append(f"| {gk[0]} | {gk[1]} | {gk[2]} | {gk[3]} | " + " | ".join(row) + " |")
text = "\n".join(out)
open(os.path.join(ROOT, 'safety_envelope_paper', 'analysis', f'threshold_table_{camp}.md'), 'w').write(text + "\n")
print("\n".join(l for l in out if l.startswith('| ') and (' | 1 | ' in l) or l.startswith('#') or l.startswith('| mgr')))
