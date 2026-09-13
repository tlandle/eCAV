#!/usr/bin/env python3
"""Metropolitan replay engine (NSDI experiment 2, fig:metro as generated).

Walks 5 Hz traces (pNEUMA CSV or SUMO FCD) through the Q7 partition of a SUMO
network and charges, per locale per 200 ms edge cycle, the DESTINATION
processing each placement arm induces on locales that do not own the track
(record import, keyed delta apply, prediction refresh), from the measured
per-operation batch curves. Reports p95 load over pooled locale-cycles as a
fraction of the 70 ms cycle slack, per-locale max, the within-age-limit
fraction of consumptions, and bytes/s as a secondary column.

Arms (paper 5.9): warm (Khonsu computed trigger), replall, delta_state,
delta_ready, reactive. Definitions fixed by the paper side (design doc):
  age at use = t_c - t_o, within limit iff < tau (0.3 s; 0.4 s secondary);
  FIGURE axis = consumptions within W = 10 cycles (2 s) after a locale crossing
  (the window where destination state quality matters: ~2 s rebuilds the
  ten-frame predictor input; branching conflicts sit ~2 s after entry);
  pooled owner-cycle population kept for the text; W = 5 and 15 as CSV
  sensitivity columns;
  p95 pooled over (locale, cycle); batching = one call per N records per
  cycle at the batch-N cost; the runtime's prediction call pattern with the
  agent-set cap (cap=1: fastest alone + the rest batched) is charged.
Costs: --costs JSON from scripts/metro/microbench_ops.py (measured); with
--costs generator the gen_batchcost placeholders are used and every output
row is flagged cost_source=generator (never a paper number).
"""
import argparse, json, math, os, sys, time, hashlib
from collections import defaultdict
import numpy as np
from scipy.spatial import cKDTree
import sumolib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pneuma_survey as ps

CYCLE = 0.2; SLACK_MS = 70.0
ARMS = ("warm", "replall", "delta_state", "delta_ready", "reactive")
GEN_COSTS = {"create": (0.9, 3.1), "delta": (0.25, 0.15), "predict": (8.0, 2.5)}   # FIXED, PER (placeholders)

class Costs:
    def __init__(self, path):
        self.source = "generator" if path == "generator" else "measured"
        self.curve = {}
        if self.source == "measured":
            j = json.load(open(path)); self.meta = {"file": os.path.basename(path), "sha": hashlib.sha256(open(path, "rb").read()).hexdigest()[:16], "device": j.get("device")}
            for op, d in j["ops"].items():
                rows = {int(k): v["median_ms"] for k, v in d["rows"].items()}
                self.curve[op] = (sorted(rows), [rows[k] for k in sorted(rows)], d["fit_fixed_ms"], d["fit_per_record_ms"])
        else:
            self.meta = {"file": "gen_batchcost placeholders"}
            for op, (fx, pr) in GEN_COSTS.items():
                self.curve[op] = ([1, 2, 4, 8, 16, 32], [fx + pr * n for n in (1, 2, 4, 8, 16, 32)], fx, pr)
    def ms(self, op, n):
        """Cost of ONE call on a batch of n records: interpolated on the measured
        batch points, linear fit beyond the largest measured batch."""
        if n <= 0: return 0.0
        xs, ys, fx, pr = self.curve[op]
        if n <= xs[-1]: return float(np.interp(n, xs, ys))
        return fx + pr * n
    def predict_call_pattern(self, n, cap=1, pad=True):
        """Runtime pattern under the agent-set cap: cap tracks alone (each padded to
        N=2 under the freeze-1q fix, so charged at the batch-2 cost) + the rest batched."""
        if n <= 0: return 0.0
        k = min(cap, n); iso = self.ms("predict", 2 if pad else 1)
        return k * iso + (self.ms("predict", n - k) if n - k > 0 else 0.0)

def load_pneuma(csv, net, tree, offnet=15.0, max_rows=None):
    _, tracks = ps.parse_pneuma(csv, max_rows); out = []
    for tid, vt, t, lat, lon, spd in tracks:
        if vt not in ps.TRACKED: continue
        xy = np.array([net.convertLonLat2XY(lo, la) for lo, la in zip(lon, lat)], dtype=float)
        d, idx = tree.query(xy)
        k5, (x5, y5, d5, i5) = ps.subsample_to_grid(t, [xy[:, 0], xy[:, 1], d, idx])
        out.append((str(tid), k5, x5, y5, d5 <= offnet, i5))
    return out

def load_fcd(path, tree, offnet=15.0):
    """SUMO --fcd-output at 0.2 s (x, y in net coordinates)."""
    import xml.etree.ElementTree as ET
    per = defaultdict(list)
    for ev, el in ET.iterparse(path, events=("end",)):
        if el.tag == "timestep":
            k = int(round(float(el.get("time")) / CYCLE))
            for v in el.findall("vehicle"): per[v.get("id")].append((k, float(v.get("x")), float(v.get("y"))))
            el.clear()
    out = []
    for vid, rows in per.items():
        rows.sort(); k = np.array([r[0] for r in rows]); x = np.array([r[1] for r in rows]); y = np.array([r[2] for r in rows])
        d, idx = tree.query(np.c_[x, y]); out.append((vid, k, x, y, d <= offnet, idx))
    return out

def run(args):
    t0 = time.time()
    net = sumolib.net.readNet(args.net, withInternal=False)
    P = ps.partition(net); tree = cKDTree(np.c_[P["X"], P["Y"]])
    grp = ps.tile_anchors(P["anchors"], args.tile); lab_of_pt = np.array([grp.get(l, -1) for l in P["LAB"]])
    L = len(set(grp.values()))
    # boundaries + neighbours per locale
    b_pts, b_loc = [], []; nbrs = defaultdict(set)
    for key, (ja, jb, xy, ln) in P["roads"].items():
        ga, gb = grp.get(ja), grp.get(jb)
        if ga != gb: b_pts.append(xy); b_loc.append((ga, gb)); nbrs[ga].add(gb); nbrs[gb].add(ga)
    b_tree = cKDTree(np.array(b_pts))
    nbrs_topo = {k: set(v) for k, v in nbrs.items()}
    costs = Costs(args.costs)
    traces = load_fcd(args.trace, tree) if args.trace.endswith(".xml") else load_pneuma(args.trace, net, tree, max_rows=args.max_rows)
    if args.subsample and args.subsample < len(traces):
        rng = np.random.default_rng(args.seed); traces = [traces[i] for i in sorted(rng.choice(len(traces), args.subsample, replace=False))]
    # per-track per-cycle records: locale, position, CV velocity, near-boundary, boundary id
    kmin = min(int(t[1][0]) for t in traces); kmax = max(int(t[1][-1]) for t in traces)
    ncyc = kmax - kmin + 1
    # state per track: sequence dict cycle -> (loc, x, y, vx, vy, dist_b, bid)
    seqs = {}
    for tid, k5, x5, y5, on, i5 in traces:
        labs = np.where(on, lab_of_pt[i5], -1)
        vx = np.gradient(x5, k5 * CYCLE) if len(k5) > 1 else np.zeros_like(x5); vy = np.gradient(y5, k5 * CYCLE) if len(k5) > 1 else np.zeros_like(y5)
        db, ib = b_tree.query(np.c_[x5, y5])
        seqs[tid] = (k5 - kmin, labs, x5, y5, vx, vy, db, ib)
    # Replication candidates per locale. 'observed' (default): neighbours across
    # boundaries that carry traffic in THIS trace (any track within 30 m during
    # the session), so replicas follow the crossings the data contains (paper
    # side, 2026-09-13: the measured point must not inherit the topological
    # degree of unobserved side streets; the SUMO extension puts traffic on every
    # street, so there observed ~ topological, a stated difference).
    seen_b = set()
    for tid, (kk, labs, xs, ys, vxs, vys, dbs, ibs) in seqs.items():
        seen_b.update(set(ibs[dbs <= 30.0].tolist()))
    if args.nbr_mode == "observed":
        nbrs = defaultdict(set)
        for j in seen_b:
            ga, gb = b_loc[j]; nbrs[ga].add(gb); nbrs[gb].add(ga)
    deg_topo = float(np.mean([len(v) for v in nbrs_topo.values()])) if nbrs_topo else 0.0
    deg_used = float(np.mean([len(v) for v in nbrs.values()])) if nbrs else 0.0
    print(f"partition L={L}, boundaries {len(b_pts)} (observed {len(seen_b)}), degree topological {deg_topo:.2f} used({args.nbr_mode}) {deg_used:.2f}, tracks {len(traces)}, cycles {ncyc} ({time.time()-t0:.0f}s)", flush=True)
    xfer_s = args.xfer_ms / 1000.0; fold = 3 * CYCLE; lead = min(2.5, max(fold + 0.35, xfer_s + fold + 0.35))
    lead_cycles = int(round(lead / CYCLE)); nb_m = args.nb_m; tau = args.tau
    results = {}
    for arm in args.arms.split(","):
        # per (locale, cycle) op counters: imports, deltas, predict replicas
        imp = defaultdict(int); dlt = defaultdict(int); prd = defaultdict(int); byts = defaultdict(float)
        consume = []   # (age_s) per consumption (pooled owner-cycle population)
        consume_pc = []  # ages of consumptions within POSTCROSS_W cycles after a crossing (handoff window)
        # per-track replica state per locale: cycle installed
        for tid, (kk, labs, xs, ys, vxs, vys, dbs, ibs) in seqs.items():
            replicas = {}          # locale -> (cycle installed, has_forecast_cycle or None)
            prepared_dst = None; prepare_cycle = None
            owner = None; owner_since = None; forecast_ready_at = None; prev_owner_exists = False
            for j in range(len(kk)):
                k = int(kk[j]); loc = int(labs[j])
                if loc < 0: continue
                # ---- crossing / ownership
                if owner is None or loc != owner:
                    prev = owner; owner = loc; owner_since = k; prev_owner_exists = prev is not None
                    # forecast availability at the new owner
                    if prev is None:
                        forecast_ready_at = k + args.cold_cycles                      # first acquisition
                    elif arm == "warm":
                        forecast_ready_at = k if loc in replicas else k + args.cold_cycles
                    elif arm in ("replall", "delta_ready"):
                        forecast_ready_at = k if loc in replicas else k + args.cold_cycles
                    elif arm == "delta_state":
                        forecast_ready_at = (k + 1) if loc in replicas else k + args.cold_cycles   # state-ready, one prediction cycle
                    elif arm == "reactive":
                        forecast_ready_at = k + 1 + int(math.ceil(xfer_s / CYCLE)); imp[(loc, k)] += 1; byts[(loc, k)] += args.record_bytes
                    replicas = {}; prepared_dst = None
                # ---- consumption at the owner
                age = CYCLE if (forecast_ready_at is not None and k >= forecast_ready_at) else (k - owner_since + 1) * CYCLE + CYCLE
                consume.append(age)
                if prev_owner_exists and k - owner_since < 15: consume_pc.append((age, k - owner_since))   # (age, cycles since crossing)
                # ---- replication policy (destination-side work for non-owners)
                near = dbs[j] <= nb_m
                if arm == "warm":
                    if prepared_dst is None:
                        # computed trigger: CV projection exits the owner within lead
                        exit_loc = None
                        for s in range(1, lead_cycles + 1):
                            px, py = xs[j] + vxs[j] * s * CYCLE, ys[j] + vys[j] * s * CYCLE
                            d_, i_ = tree.query([px, py])
                            if d_ <= 15.0 and lab_of_pt[i_] >= 0 and lab_of_pt[i_] != owner: exit_loc = int(lab_of_pt[i_]); break
                        if exit_loc is not None:
                            prepared_dst = exit_loc; prepare_cycle = k; replicas[exit_loc] = (k, k)
                            imp[(exit_loc, k)] += 1; byts[(exit_loc, k)] += args.record_bytes
                    for rl in list(replicas):
                        prd[(rl, k)] += 1                                   # destination predicts on its replica each cycle
                        if k - replicas[rl][0] > args.replica_ttl_cycles: del replicas[rl]
                elif arm in ("replall", "delta_state", "delta_ready"):
                    if near:
                        for nl in nbrs.get(owner, ()):
                            if nl not in replicas:
                                replicas[nl] = (k, k); imp[(nl, k)] += 1; byts[(nl, k)] += args.record_bytes
                            elif arm == "replall":
                                imp[(nl, k)] += 1; byts[(nl, k)] += args.record_bytes
                            else:
                                dlt[(nl, k)] += 1; byts[(nl, k)] += args.delta_bytes
                            if arm in ("replall", "delta_ready"): prd[(nl, k)] += 1
                    else:
                        for rl in list(replicas):
                            if k - replicas[rl][0] > args.replica_ttl_cycles: del replicas[rl]
                # reactive: nothing before the crossing
        # ---- per (locale, cycle) load in ms
        keys = set(imp) | set(dlt) | set(prd)
        load = {}
        for key in keys:
            ms = costs.ms("create", imp.get(key, 0)) if imp.get(key, 0) else 0.0
            ms += costs.ms("delta", dlt.get(key, 0)) if dlt.get(key, 0) else 0.0
            ms += costs.predict_call_pattern(prd.get(key, 0), cap=args.cap, pad=args.pad_isolated) if prd.get(key, 0) else 0.0
            load[key] = ms
        # pooled over ALL locale-cycles (zero where no replication work)
        pooled = np.zeros(L * ncyc);
        for (loc, k), ms in load.items():
            if 0 <= loc < L and 0 <= k < ncyc: pooled[loc * ncyc + k] = ms
        per_loc_max = max((ms for ms in load.values()), default=0.0)
        ages = np.array(consume); pc = np.array(consume_pc).reshape(-1, 2)
        def within_w(w): 
            sel = pc[pc[:, 1] < w, 0] if len(pc) else pc
            return (float((sel < tau).mean()) if len(sel) else None, int(len(sel)))
        w5, w10, w15 = within_w(5), within_w(args.postcross_w), within_w(15)
        ages_pc = pc[pc[:, 1] < args.postcross_w, 0] if len(pc) else pc
        bytes_per_s = sum(byts.values()) / (ncyc * CYCLE) / max(L, 1)
        results[arm] = {"p95_load_frac": float(np.percentile(pooled, 95) / SLACK_MS), "p99_load_frac": float(np.percentile(pooled, 99) / SLACK_MS),
                        "mean_load_frac": float(pooled.mean() / SLACK_MS), "max_locale_cycle_frac": per_loc_max / SLACK_MS,
                        "within_limit_frac": float((ages < tau).mean()) if len(ages) else None,
                        "within_limit_frac_400": float((ages < 0.4).mean()) if len(ages) else None,
                        "within_limit_postcross_frac": w10[0], "postcross_consumptions": w10[1], "postcross_w_cycles": args.postcross_w,
                        "within_limit_postcross_w5": w5[0], "within_limit_postcross_w15": w15[0],   # sensitivity columns (CSV only, not the figure)
                        "consumptions": int(len(ages)), "imports": int(sum(imp.values())), "deltas": int(sum(dlt.values())), "predict_replicas": int(sum(prd.values())),
                        "bytes_per_s_per_locale": bytes_per_s}
        r = results[arm]
        print(f"{arm:12s} p95 load {r['p95_load_frac']*100:6.1f}% of slack  max {r['max_locale_cycle_frac']*100:6.1f}%  within {r['within_limit_frac']*100:5.1f}% (post-crossing window {(r['within_limit_postcross_frac'] or 0)*100:5.1f}%)  imports {r['imports']} deltas {r['deltas']} predict-replicas {r['predict_replicas']}  bytes/s/locale {bytes_per_s:.0f}", flush=True)
    out = {"trace": os.path.basename(args.trace), "trace_sha": hashlib.sha256(open(args.trace, "rb").read(1 << 26)).hexdigest()[:16], "net": os.path.basename(args.net),
           "tile_m": args.tile, "L": L, "boundaries": len(b_pts), "boundaries_observed": len(seen_b), "degree_topological": deg_topo, "degree_used": deg_used, "nbr_mode": args.nbr_mode,
           "tracks": len(traces), "cycles": ncyc, "lead_s": lead, "nb_m": nb_m, "tau_s": tau,
           "cost_source": costs.source, "cost_meta": costs.meta, "cap": args.cap, "pad_isolated": args.pad_isolated, "arms": results}
    json.dump(out, open(args.out, "w"), indent=1); print("written", args.out, f"({time.time()-t0:.0f}s)")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True); ap.add_argument("--net", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--costs", default="generator", help="microbench JSON or 'generator' (placeholders, flagged)")
    ap.add_argument("--arms", default=",".join(ARMS)); ap.add_argument("--tile", type=int, default=300)
    ap.add_argument("--nb_m", type=float, default=20.0, help="near-boundary band for the replication arms (20 m = the corridor's own band)")
    ap.add_argument("--nbr_mode", default="observed", choices=("observed", "topological"), help="replication neighbours: across observed boundaries (default) or all topological neighbours")
    ap.add_argument("--tau", type=float, default=0.3); ap.add_argument("--xfer_ms", type=float, default=40.0)
    ap.add_argument("--cold_cycles", type=int, default=4, help="cycles until a cold destination has a forecast (fold-in 3 + prediction 1)")
    ap.add_argument("--replica_ttl_cycles", type=int, default=25); ap.add_argument("--cap", type=int, default=1)
    ap.add_argument("--pad_isolated", type=int, default=1, help="1 = isolated calls padded to N=2 (freeze-1q fix), charged at the batch-2 cost")
    ap.add_argument("--postcross_w", type=int, default=10, help="handoff window (cycles after a crossing) for the second freshness population")
    ap.add_argument("--record_bytes", type=float, default=2600.0); ap.add_argument("--delta_bytes", type=float, default=120.0)
    ap.add_argument("--subsample", type=int, default=None); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--max_rows", type=int, default=None)
    run(ap.parse_args())

if __name__ == "__main__":
    main()
