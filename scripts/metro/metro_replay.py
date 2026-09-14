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
  FIGURE right axis = the LOAD-COUPLED within-limit fraction: the deployed
  adaptive controller's rate fallback (WorldFusionAdaptiveEdge.run_step lines
  168-176 and 213-222) replayed per locale on pre_pred + owned pass + induced
  load; the mechanism-only fraction stays as a CSV column and excludes
  load-induced rate fallback (paper side, 2026-09-13);
  p95 pooled over (locale, cycle); batching = one call per N records per
  cycle at the batch-N cost; the runtime's prediction call pattern with the
  agent-set cap (cap=1: fastest alone + the rest batched) is charged.
Costs: --costs JSON from scripts/metro/microbench_ops.py (measured); with
--costs generator the gen_batchcost placeholders are used and every output
row is flagged cost_source=generator (never a paper number).
"""
import argparse, json, math, os, sys, time, hashlib
from array import array
from collections import deque
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
                xs = sorted(rows); ys = [rows[k] for k in xs]
                # MONOTONE ENVELOPE for charging: the measured medians are not
                # monotone at small N (predict 43.0 ms at N=2, 38.8 at N=4, 5-rep
                # noise around a flat region), and a marginal charge from a
                # non-monotone curve can go negative. Charging uses the cumulative
                # maximum of the measured points (stated in the doc); the raw
                # points are what the appendix table reports.
                ys = list(np.maximum.accumulate(np.array(ys, dtype=float)))
                self.curve[op] = (xs, ys, d["fit_fixed_ms"], d["fit_per_record_ms"])
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
    grp = ps.tile_anchors(P["anchors"], args.tile)
    # WEAK-SCALING CROP (2026-09-14): keep only the tiles inside a window of the
    # tiling (tile coordinates); other anchors become unlabelled (-1) = off-network
    # for the replay; boundaries to unlabelled tiles are dropped below. The trace
    # is unchanged, so density inside the window is the full network's.
    crop_meta = None
    if args.tile_square or args.tile_window:
        ids = sorted(P["anchors"]); x0 = min(P["anchors"][i][0] for i in ids); y0 = min(P["anchors"][i][1] for i in ids)
        tc = {i: (int((P["anchors"][i][0] - x0) // args.tile), int((P["anchors"][i][1] - y0) // args.tile)) for i in ids}
        if args.tile_window:
            kx0, ky0, kx1, ky1 = [int(v) for v in args.tile_window.split(",")]
        else:
            k = int(args.tile_square); xs_ = [c[0] for c in tc.values()]; ys_ = [c[1] for c in tc.values()]
            cx = (min(xs_) + max(xs_)) // 2; cy = (min(ys_) + max(ys_)) // 2
            kx0, ky0 = cx - k // 2, cy - k // 2; kx1, ky1 = kx0 + k - 1, ky0 + k - 1
        keep = sorted({grp[i] for i in ids if kx0 <= tc[i][0] <= kx1 and ky0 <= tc[i][1] <= ky1})
        remap = {g: n for n, g in enumerate(keep)}
        grp = {i: remap.get(grp[i], -1) for i in ids}
        crop_meta = {"tile_window": [kx0, ky0, kx1, ky1], "tiles_in_window": (kx1 - kx0 + 1) * (ky1 - ky0 + 1), "locales_kept": len(keep)}
    lab_of_pt = np.array([grp.get(l, -1) for l in P["LAB"]])
    L = len(set(v for v in grp.values() if v >= 0))
    # tile coordinates per locale (for window selection and interior reporting)
    _ids = sorted(P["anchors"]); _x0 = min(P["anchors"][i][0] for i in _ids); _y0 = min(P["anchors"][i][1] for i in _ids)
    locale_tile = {}
    for i in _ids:
        g = grp.get(i, -1)
        if g >= 0 and g not in locale_tile: locale_tile[g] = (int((P["anchors"][i][0] - _x0) // args.tile), int((P["anchors"][i][1] - _y0) // args.tile))
    interior = None
    if crop_meta is not None:
        kx0, ky0, kx1, ky1 = crop_meta["tile_window"]
        interior = sorted(g for g, (tx, ty) in locale_tile.items() if kx0 < tx < kx1 and ky0 < ty < ky1)
        crop_meta["interior_locales"] = len(interior)
    # boundaries + neighbours per locale
    b_pts, b_loc = [], []; nbrs = defaultdict(set)
    for key, (ja, jb, xy, ln) in P["roads"].items():
        ga, gb = grp.get(ja, -1), grp.get(jb, -1)
        if ga != gb and ga >= 0 and gb >= 0: b_pts.append(xy); b_loc.append((ga, gb)); nbrs[ga].add(gb); nbrs[gb].add(ga)
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
    # owned tracks per (locale, cycle): the destination's own prediction batch that
    # imported replicas JOIN; the marginal charge is pattern(owned + replicas) -
    # pattern(owned). Standalone (replicas as their own batch) kept as a column.
    owned = defaultdict(int)
    for tid, (kk, labs, xs, ys, vxs, vys, dbs, ibs) in seqs.items():
        for k, loc in zip(kk, labs):
            if loc >= 0: owned[(int(loc), int(k))] += 1
    # trace statistics beside the generator's constants (same definitions as the
    # pNEUMA survey: 5 Hz cycle grid, crossings per track-second, near-boundary
    # fraction of track-cycles at 20/30/50 m, active tracks per cycle, veh/lane-km)
    _cross = _tc = 0; _nb = {20: 0, 30: 0, 50: 0}; _act = defaultdict(int)
    for tid, (kk, labs, xs, ys, vxs, vys, dbs, ibs) in seqs.items():
        valid = labs >= 0; _tc += int(valid.sum())
        _cross += int(((labs[1:] != labs[:-1]) & valid[1:] & valid[:-1]).sum())
        for r in _nb: _nb[r] += int((dbs[valid] <= r).sum())
        for k in kk[valid]: _act[int(k)] += 1
    _actv = np.array(sorted(_act.values())) if _act else np.zeros(1)
    trace_stats = {"crossing_rate_per_track_s_5hz": (_cross / (_tc * CYCLE)) if _tc else None,
                   "near_boundary_frac": {str(r): (_nb[r] / _tc if _tc else None) for r in _nb},
                   "active_tracks_p50": float(np.median(_actv)), "active_tracks_p95": float(np.percentile(_actv, 95)),
                   "vehicles_per_lane_km": (float(np.median(_actv) / P["lane_km"]) if (P["lane_km"] > 0 and crop_meta is None) else None), "lane_km": (P["lane_km"] if crop_meta is None else None),
                   "generator": {"crossing_rate_per_track_s": 0.013, "near_boundary_frac": 0.22, "degree_at_L": 2.6 + math.log(L / 16.0 + 1.0) * 0.4}}
    _vlk = (f"{trace_stats['vehicles_per_lane_km']:.2f}" if trace_stats['vehicles_per_lane_km'] is not None else "n/a (crop)")
    print(f"trace stats: crossings/track/s {trace_stats['crossing_rate_per_track_s_5hz']:.4f} (gen 0.013) near20 {trace_stats['near_boundary_frac']['20']:.3f} (gen 0.22) active p50 {trace_stats['active_tracks_p50']:.0f} p95 {trace_stats['active_tracks_p95']:.0f} veh/lane-km {_vlk} degree used {deg_used:.2f} (gen {trace_stats['generator']['degree_at_L']:.2f})", flush=True)
    _ov = np.array(list(owned.values())) if owned else np.zeros(1)
    _pl = np.zeros(L)
    for (loc, k), v in owned.items():
        if 0 <= loc < L: _pl[loc] += v
    per_locale_tracks_mean = (_pl / max(ncyc, 1)).tolist()
    owned_stats = {"p50": float(np.median(_ov)), "p95": float(np.percentile(_ov, 95)), "max": int(_ov.max()), "locale_cycles_with_tracks": int(len(owned))}
    # ADMISSION CONTROL (paper 5.1; the runtime's budgeted predictor): the destination
    # admits K_admit tracks to full prediction per cycle, the largest N whose call
    # pattern fits the prediction budget; the rest get the fallback. The owned pass
    # is pattern(min(N_owned, K_admit)); an added track is charged marginally above
    # that ("on-top" face); if replicas compete for the K slots they displace owned
    # tracks ("displacement" face). --admit_budget_ms 0 disables admission (demand).
    K_ADMIT = None; K_PATTERN = None
    if args.admit_budget_ms > 0:
        # paper side's rule (5.1): K_admit = the largest N whose single batched call
        # predict(N) fits the prediction budget (92 ms -> 16 on the measured curve).
        # The two-call cap pattern (predict(2) + predict(N-1)) gives a smaller K,
        # reported as a sensitivity value, not used for the figure.
        K_PATTERN = max([n for n in range(1, 2000) if costs.predict_call_pattern(n, cap=args.cap, pad=args.pad_isolated) <= args.admit_budget_ms] or [1])
        K_single = max([n for n in range(1, 2000) if costs.ms("predict", n) <= args.admit_budget_ms] or [1])
        # DEPLOYED rule (the runtime's own): budget_{t+1} = deadline - (pre_pred + prediction_t)
        # (WorldFusionAdaptiveEdge.run_step lines 168/193-194 measure the WHOLE pass, then set
        # the predictor's budget for the next tick); K_t = max(int((budget_t - 25) / avg), 1)
        # with avg = mean over the issued calls of (elapsed - 25) / n (_risk_budget_select);
        # iterated on the measured curve until it repeats; report the cycle.
        K_DEPLOYED_CYCLE = None
        if args.admit_rule == "deployed" or True:
            def calls_for(K):
                iso = costs.ms("predict", 2 if args.pad_isolated else 1); rest = costs.ms("predict", K - 1) if K > 1 else 0.0
                per = [(iso - 25.0) / (2 if args.pad_isolated else 1)] + ([(rest - 25.0) / (K - 1)] if K > 1 else [])
                return iso + rest, float(np.mean(per))
            K, seen, seq = 1, {}, []
            for it in range(50):
                pred_ms, avg = calls_for(K); budget = max(5.0, args.deadline_ms - (args.pre_pred_ms + pred_ms))
                seq.append(K)
                Kn = max(int((budget - 25.0) / max(avg, 0.1)), 1)
                if Kn in seen and seen[Kn] >= 0: 
                    cyc = seq[seen[Kn]:]; K_DEPLOYED_CYCLE = sorted(set(cyc)); break
                seen[Kn] = len(seq); K = Kn
            if K_DEPLOYED_CYCLE is None: K_DEPLOYED_CYCLE = [K]
        K_DEPLOYED = int(round(np.mean(K_DEPLOYED_CYCLE)))
        K_ADMIT = args.k_admit if args.k_admit > 0 else {"single": K_single, "pattern": K_PATTERN, "deployed": K_DEPLOYED}[args.admit_rule]
    def admitted(no): return no if K_ADMIT is None else min(no, K_ADMIT)
    _ratio = np.array([min(1.0, admitted(v) / v) for v in _ov if v > 0]) if K_ADMIT else np.ones(1)
    owned_stats.update({"K_admit": K_ADMIT, "K_admit_cap_pattern": K_PATTERN, "K_deployed_cycle": K_DEPLOYED_CYCLE if args.admit_budget_ms > 0 else None, "admit_rule": args.admit_rule, "admit_budget_ms": args.admit_budget_ms,
                        "admission_ratio_p50": float(np.median(_ratio)), "admission_ratio_p95_low": float(np.percentile(_ratio, 5)), "admission_ratio_mean": float(_ratio.mean())})
    print(f"N_owned per locale-cycle: p50 {owned_stats['p50']:.0f} p95 {owned_stats['p95']:.0f} max {owned_stats['max']} | K_admit={K_ADMIT} ({args.admit_rule} rule, budget {args.admit_budget_ms} ms; cap-pattern K {K_PATTERN}; deployed-rule cycle {owned_stats.get('K_deployed_cycle')}) admission ratio p50 {owned_stats['admission_ratio_p50']:.2f} 5th pct {owned_stats['admission_ratio_p95_low']:.2f}", flush=True)
    results = {}; _dump_pooled = {}
    for arm in args.arms.split(","):
        # per (locale, cycle) op counters: imports, deltas, predict replicas
        imp = defaultdict(int); dlt = defaultdict(int); prd = defaultdict(int); byts = defaultdict(float)
        cross_in = defaultdict(int)          # actual crossings INTO a locale per (locale, cycle)
        prepared_into = defaultdict(set)     # locale -> set of tracks prepared into it (warm: predicted to cross)
        prepared_crossed = defaultdict(set)  # locale -> prepared tracks that did cross into it
        # per consumption: mechanism age, owner locale, cycle, cycles since a crossing (-1 = no prior owner or past 15)
        c_age = array("f"); c_own = array("i"); c_k = array("i"); c_since = array("i")
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
                    if prev is not None:
                        cross_in[(loc, k)] += 1
                        if tid in prepared_into.get(loc, ()): prepared_crossed[loc].add(tid)
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
                c_age.append(age); c_own.append(owner); c_k.append(k); c_since.append((k - owner_since) if (prev_owner_exists and k - owner_since < 15) else -1)
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
                            prepared_into[exit_loc].add(tid)
                    for rl in list(replicas):
                        prd[(rl, k)] += 1                                   # destination predicts on its replica each cycle
                        if k - replicas[rl][0] > args.replica_ttl_cycles: del replicas[rl]
                elif arm in ("replall", "delta_state", "delta_ready"):
                    if near:
                        for nl in nbrs.get(owner, ()):
                            if nl not in replicas:
                                replicas[nl] = (k, k); imp[(nl, k)] += 1; byts[(nl, k)] += args.record_bytes
                                prepared_into[nl].add(tid)
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
        load = {}; load_sa = {}; batch_sizes = []; displaced_frac = []
        for key in keys:
            base = costs.ms("create", imp.get(key, 0)) if imp.get(key, 0) else 0.0
            base += costs.ms("delta", dlt.get(key, 0)) if dlt.get(key, 0) else 0.0
            nr = prd.get(key, 0); no = owned.get(key, 0)
            p_sa = costs.predict_call_pattern(nr, cap=args.cap, pad=args.pad_isolated) if nr else 0.0
            # marginal (paper side): added tracks join the BATCHED REMAINDER of the
            # owned pass (predict(2) for the isolated+pad track cancels out), so
            # marginal = predict(N_owned-1+N_added) - predict(N_owned-1); a locale
            # with no owned tracks runs its pass only for the replicas (pattern(N_added)).
            na = admitted(no)
            p_mg = max(0.0, costs.predict_call_pattern(na + nr, cap=args.cap, pad=args.pad_isolated) - costs.predict_call_pattern(na, cap=args.cap, pad=args.pad_isolated)) if nr else 0.0
            if nr:
                batch_sizes.append(na + nr)
                if K_ADMIT is not None and no > 0:
                    displaced_frac.append(max(0, na + nr - K_ADMIT) / no if (na + nr) > K_ADMIT else 0.0)   # owned tracks losing their slot if replicas compete
            load_sa[key] = base + p_sa
            load[key] = base + (p_mg if args.charge == "marginal" else p_sa)
        # pooled over ALL locale-cycles (zero where no replication work)
        pooled = np.zeros(L * ncyc);
        for (loc, k), ms in load.items():
            if 0 <= loc < L and 0 <= k < ncyc: pooled[loc * ncyc + k] = ms
        per_loc_max = max((ms for ms in load.values()), default=0.0)
        if args.dump_pooled: _dump_pooled.setdefault("arms", {})[arm] = pooled.astype(np.float32)
        # locality reporting: (a) interior locales of a crop (all neighbours inside the window);
        # (b) --report_windows: pooled p95 over centred k x k windows of THIS run (no cropping;
        # the reference in which only sampling changes with L)
        p95_interior = None
        if interior:
            _sel = np.concatenate([pooled[l * ncyc:(l + 1) * ncyc] for l in interior]); p95_interior = float(np.percentile(_sel, 95) / SLACK_MS)
        window_p95 = {}
        if args.report_windows:
            _txs = [t[0] for t in locale_tile.values()]; _tys = [t[1] for t in locale_tile.values()]
            _cx = (min(_txs) + max(_txs)) // 2; _cy = (min(_tys) + max(_tys)) // 2
            for kk in [int(v) for v in args.report_windows.split(",") if v]:
                x0_, y0_ = _cx - kk // 2, _cy - kk // 2; x1_, y1_ = x0_ + kk - 1, y0_ + kk - 1
                locs = sorted(g for g, (tx, ty) in locale_tile.items() if x0_ <= tx <= x1_ and y0_ <= ty <= y1_)
                inner = sorted(g for g, (tx, ty) in locale_tile.items() if x0_ < tx < x1_ and y0_ < ty < y1_)
                if locs:
                    _selw = np.concatenate([pooled[l * ncyc:(l + 1) * ncyc] for l in locs])
                    _seli = np.concatenate([pooled[l * ncyc:(l + 1) * ncyc] for l in inner]) if inner else None
                    window_p95[str(kk)] = {"L": len(locs), "p95_load_frac": float(np.percentile(_selw, 95) / SLACK_MS), "tracks_per_locale": float(np.mean([per_locale_tracks_mean[l] for l in locs])),
                                           "L_interior": len(inner), "p95_load_frac_interior": (float(np.percentile(_seli, 95) / SLACK_MS) if _seli is not None else None)}
        pooled_sa = np.zeros(L * ncyc)
        for (loc, k), ms in load_sa.items():
            if 0 <= loc < L and 0 <= k < ncyc: pooled_sa[loc * ncyc + k] = ms
        ages = np.frombuffer(c_age, dtype=np.float32).astype(float); c_own_a = np.frombuffer(c_own, dtype=np.int32); c_k_a = np.frombuffer(c_k, dtype=np.int32); c_since_a = np.frombuffer(c_since, dtype=np.int32)
        # LOAD-COUPLED freshness (paper side, 2026-09-13). The deployed adaptive
        # controller (WorldFusionAdaptiveEdge.run_step lines 168-176, 213-222) keeps a
        # 20-pass compute history per locale and, once >= 5 passes are in it, sets
        # rate_k = 1/2/4/5/10 from the p95 of the WHOLE pass against the deadline
        # (<= 1x / 2x / 4x / 5x / else); at rate_k the locale executes one pass every
        # rate_k cycles and the agents hold the last delivered forecast. Whole pass at
        # (locale, cycle) = pre_pred + owned prediction pass (pattern(admitted N_owned))
        # + the arm's induced load as charged (ms). The controller is replayed per
        # locale; a consumption's coupled age = max(mechanism age, (cycle - last
        # executed pass + 1) x CYCLE). Induced work at skipped cycles is not deferred
        # (the lighter assumption for the replication arms).
        # Owned-pass model per row (paper side, 2026-09-13): the K = 16 row IS the
        # single-batch capacity model, for admission AND for its owned pass
        # (predict(K) as one batch, 89 ms at 16); the deployed row keeps the cap
        # pattern throughout (predict(2) + predict(K-1)); the demand row (no
        # admission) keeps the cap pattern on every owned track by definition.
        _single_owned = (args.admit_rule == "single" and K_ADMIT is not None)
        _pat_cache = {}
        def _owned_pass(na):
            if na not in _pat_cache: _pat_cache[na] = (costs.ms("predict", na) if _single_owned else costs.predict_call_pattern(na, cap=args.cap, pad=args.pad_isolated)) if na else 0.0
            return _pat_cache[na]
        RATE_STEPS = ((1.0, 1), (2.0, 2), (4.0, 4), (5.0, 5), (None, 10))
        def _p95_20(hist):   # np.percentile(list, 95) (linear interpolation) on <= 20 values
            s_ = sorted(hist); pos = 0.95 * (len(s_) - 1); lo = int(pos)
            return s_[lo] if lo + 1 >= len(s_) else s_[lo] + (s_[lo + 1] - s_[lo]) * (pos - lo)
        owned_pass_ms = np.zeros((L, ncyc))
        for (loc, k), no in owned.items():
            if 0 <= loc < L and 0 <= k < ncyc: owned_pass_ms[loc, k] = _owned_pass(admitted(no))
        T_all = args.pre_pred_ms + owned_pass_ms
        for (loc, k), ms in load.items():
            if 0 <= loc < L and 0 <= k < ncyc: T_all[loc, k] += ms
        last_exec = np.zeros((L, ncyc), dtype=np.int32); rate_of = np.ones((L, ncyc), dtype=np.int8); pass_ms_exec = []
        for loc in range(L):
            hist = deque(maxlen=20); rk = 1; since = 0; le = 0; Trow = T_all[loc]
            for k in range(ncyc):
                run_pass = True
                if rk > 1:
                    since += 1
                    if since < rk: run_pass = False
                    else: since = 0
                if run_pass:
                    le = k; hist.append(Trow[k]); pass_ms_exec.append(Trow[k])
                    if len(hist) >= 5:
                        p95 = _p95_20(hist)
                        for mult, r_ in RATE_STEPS:
                            if mult is None or p95 <= args.deadline_ms * mult: rk = r_; break
                last_exec[loc, k] = le; rate_of[loc, k] = rk
        ages_c = np.maximum(ages, (c_k_a - last_exec[c_own_a, c_k_a] + 1) * CYCLE) if len(ages) else ages
        rk_cons = rate_of[c_own_a, c_k_a] if len(ages) else np.ones(0, dtype=np.int8)
        _own_mask = owned_pass_ms > 0; rk_owned = rate_of[_own_mask]
        rate_dist = {str(r_): (float((rk_owned == r_).mean()) if rk_owned.size else None) for r_ in (1, 2, 4, 5, 10)}
        def within_w(w, a=ages):
            sel = a[(c_since_a >= 0) & (c_since_a < w)] if len(a) else a
            return (float((sel < tau).mean()) if len(sel) else None, int(len(sel)))
        w5, w10, w15 = within_w(5), within_w(args.postcross_w), within_w(15)
        w5c, w10c, w15c = within_w(5, ages_c), within_w(args.postcross_w, ages_c), within_w(15, ages_c)
        bytes_per_s = sum(byts.values()) / (ncyc * CYCLE) / max(L, 1)
        # INSTRUMENTATION (2026-09-14): counts per (locale, cycle) behind the charged load
        def _mp(vals):
            a = np.array(list(vals), dtype=float) if len(vals) else np.zeros(0)
            return {"mean": float(a.mean()) if a.size else None, "p95": float(np.percentile(a, 95)) if a.size else None, "max": float(a.max()) if a.size else None, "n": int(a.size)}
        _ptc = [len(prepared_into[l]) for l in range(L)]
        _pc = sum(len(prepared_crossed[l]) for l in range(L)); _pi = sum(_ptc)
        instr = {"prepares_per_dest_cycle": _mp(imp.values()), "refreshes_per_dest_cycle": _mp(prd.values()),
                 "crossing_burst_per_locale_cycle": _mp(cross_in.values()),
                 "predicted_to_cross_per_locale": _mp(_ptc), "false_prep_frac": (1.0 - _pc / _pi) if _pi else None,
                 "prepared_tracks_total": _pi, "prepared_tracks_crossed": _pc,
                 "locale_cycles_with_imports": int(len(imp)), "locale_cycles_with_refreshes": int(len(prd)), "locale_cycles_total": int(L * ncyc),
                 # paper-side registry names (2026-09-14): unconditional means over ALL locale-cycles,
                 # the fraction of locale-cycles with any prepare/import, p95 given any, never-crossed share
                 "registry": {"prepares_per_dest_cycle": sum(imp.values()) / max(L * ncyc, 1), "prepare_cycle_frac": len(imp) / max(L * ncyc, 1),
                              "prepares_p95_given_any": _mp(imp.values())["p95"], "predicted_crossers_per_locale": float(np.mean(_ptc)) if _ptc else None,
                              "refreshes_per_dest_cycle": sum(prd.values()) / max(L * ncyc, 1), "refreshes_p95_given_any": _mp(prd.values())["p95"],
                              "crossing_burst_p95": _mp(cross_in.values())["p95"], "never_crossed_frac": (1.0 - _pc / _pi) if _pi else None}}
        results[arm] = {"p95_load_frac": float(np.percentile(pooled, 95) / SLACK_MS), "p99_load_frac": float(np.percentile(pooled, 99) / SLACK_MS),
                        "mean_load_frac": float(pooled.mean() / SLACK_MS), "max_locale_cycle_frac": per_loc_max / SLACK_MS,
                        "p95_load_frac_standalone": float(np.percentile(pooled_sa, 95) / SLACK_MS), "charge": args.charge,
                        "displaced_owned_frac_mean": float(np.mean(displaced_frac)) if displaced_frac else None,
                        "displaced_owned_frac_p95": float(np.percentile(displaced_frac, 95)) if displaced_frac else None,
                        "batch_owned_plus_added_p50": float(np.median(batch_sizes)) if batch_sizes else None,
                        "batch_owned_plus_added_p95": float(np.percentile(batch_sizes, 95)) if batch_sizes else None,
                        "within_limit_frac": float((ages < tau).mean()) if len(ages) else None,
                        "within_limit_frac_400": float((ages < 0.4).mean()) if len(ages) else None,
                        "within_limit_postcross_frac": w10[0], "postcross_consumptions": w10[1], "postcross_w_cycles": args.postcross_w,
                        "within_limit_postcross_w5": w5[0], "within_limit_postcross_w15": w15[0],   # sensitivity columns (CSV only, not the figure)
                        "within_limit_postcross_coupled": w10c[0], "within_limit_postcross_coupled_w5": w5c[0], "within_limit_postcross_coupled_w15": w15c[0],   # FIGURE right axis (load-coupled)
                        "within_limit_frac_coupled": float((ages_c < tau).mean()) if len(ages_c) else None,
                        "rate_k_frac_owned_locale_cycles": rate_dist, "rate_k_mean_at_consumption": float(rk_cons.mean()) if rk_cons.size else None,
                        "rate_k_frac_gt1_at_consumption": float((rk_cons > 1).mean()) if rk_cons.size else None,
                        "pass_ms_p95_executed": float(np.percentile(pass_ms_exec, 95)) if pass_ms_exec else None,
                        "pass_ms_p95_owned_only": float(np.percentile((args.pre_pred_ms + owned_pass_ms)[_own_mask], 95)) if _own_mask.any() else None,
                        "consumptions": int(len(ages)), "imports": int(sum(imp.values())), "deltas": int(sum(dlt.values())), "predict_replicas": int(sum(prd.values())),
                        "bytes_per_s_per_locale": bytes_per_s, "instrumentation": instr,
                        "p95_load_frac_interior": p95_interior, "window_p95": window_p95}
        r = results[arm]
        print(f"{arm:12s} p95 load {r['p95_load_frac']*100:6.1f}% of slack ({args.charge}; standalone {r['p95_load_frac_standalone']*100:6.1f}%; batch admitted+added p50/p95 {r['batch_owned_plus_added_p50']}/{r['batch_owned_plus_added_p95']}; displaced owned frac mean/p95 {r['displaced_owned_frac_mean']}/{r['displaced_owned_frac_p95']})  max {r['max_locale_cycle_frac']*100:6.1f}%  within {r['within_limit_frac']*100:5.1f}% (post-crossing window {(r['within_limit_postcross_frac'] or 0)*100:5.1f}%; COUPLED {(r['within_limit_postcross_coupled'] or 0)*100:5.1f}%, rate_k>1 at {(r['rate_k_frac_gt1_at_consumption'] or 0)*100:4.1f}% of consumptions, pass p95 {r['pass_ms_p95_executed'] or 0:.0f} ms vs owned-only {r['pass_ms_p95_owned_only'] or 0:.0f} ms)  imports {r['imports']} deltas {r['deltas']} predict-replicas {r['predict_replicas']}  bytes/s/locale {bytes_per_s:.0f}", flush=True)
    out = {"trace": os.path.basename(args.trace), "trace_sha": hashlib.sha256(open(args.trace, "rb").read(1 << 26)).hexdigest()[:16], "net": os.path.basename(args.net),
           "tile_m": args.tile, "L": L, "boundaries": len(b_pts), "boundaries_observed": len(seen_b), "degree_topological": deg_topo, "degree_used": deg_used, "nbr_mode": args.nbr_mode,
           "tracks": len(traces), "cycles": ncyc, "lead_s": lead, "nb_m": nb_m, "tau_s": tau,
           "cost_source": costs.source, "cost_meta": costs.meta, "charging_curve": "monotone envelope (cumulative max) of the measured medians", "cap": args.cap, "pad_isolated": args.pad_isolated,
           "python_hash_seed": os.environ.get("PYTHONHASHSEED"), "partition_deterministic": True,
           "crop": crop_meta, "tracks_per_locale": (trace_stats["active_tracks_p50"] / L) if L else None,
           "per_locale_tracks_mean": per_locale_tracks_mean, "locale_tile": {str(g): list(t) for g, t in locale_tile.items()},
           "freshness_coupling": {"rule": "deployed rate fallback (WorldFusionAdaptiveEdge.run_step 168-176, 213-222): per-locale p95 over the last 20 executed passes of pre_pred + owned pass + induced load vs deadline -> rate_k 1/2/4/5/10 (<= 1x/2x/4x/5x/else); one pass per rate_k cycles; coupled age = max(mechanism age, (cycle - last executed pass + 1) x 0.2 s); induced work at skipped cycles not deferred",
                                  "pre_pred_ms": args.pre_pred_ms, "deadline_ms": args.deadline_ms,
                                  "owned_pass_model": ("single batch predict(K_admit) (the K-capacity row's own model)" if (args.admit_rule == "single" and K_ADMIT is not None) else "cap pattern predict(2) + predict(N-1) on the admitted tracks"),
                                  "skipped_cycle_induced_work": "not deferred (the lighter case for replication)"},
           "n_owned_per_locale_cycle": owned_stats, "trace_stats": trace_stats, "arms": results}
    json.dump(out, open(args.out, "w"), indent=1); print("written", args.out, f"({time.time()-t0:.0f}s)")
    if args.dump_pooled:
        # per-(locale, cycle) charged load (ms) per arm, for offline window selection (weak scaling)
        np.savez_compressed(args.dump_pooled, L=L, ncyc=ncyc, per_locale_tracks_mean=np.array(per_locale_tracks_mean), locale_tile=np.array([locale_tile[g] for g in range(L)]),
                            degree_per_locale=np.array([len(nbrs.get(g, ())) for g in range(L)]), **{f"pooled_{a}": v for a, v in _dump_pooled.get("arms", {}).items()})
        print("dumped", args.dump_pooled)

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
    ap.add_argument("--charge", default="marginal", choices=("marginal", "standalone"), help="replica prediction charge: marginal on the destination's owned batch (default) or a standalone batch")
    ap.add_argument("--admit_budget_ms", type=float, default=92.0, help="prediction budget per cycle for admission control (130 ms allocation - ~38 ms fusion+tracking); 0 = no admission (demand)")
    ap.add_argument("--admit_rule", default="single", choices=("single", "pattern", "deployed"), help="K_admit: single batched predict(K) <= budget (paper 5.1); the two-call cap pattern; or the runtime's own feedback rule iterated to its cycle (deployed)")
    ap.add_argument("--deadline_ms", type=float, default=130.0); ap.add_argument("--pre_pred_ms", type=float, default=38.0)
    ap.add_argument("--k_admit", type=int, default=0, help="explicit K_admit override (0 = derive)")
    ap.add_argument("--postcross_w", type=int, default=10, help="handoff window (cycles after a crossing) for the second freshness population")
    ap.add_argument("--tile_square", type=int, default=0, help="weak-scaling crop: keep a k x k window of tiles centred on the tiling (0 = full network)")
    ap.add_argument("--tile_window", default="", help="weak-scaling crop: explicit tile window kx0,ky0,kx1,ky1 (tile coordinates)")
    ap.add_argument("--report_windows", default="", help="comma-separated k: pooled p95 over centred k x k windows of this run (locality reference, no cropping)")
    ap.add_argument("--dump_pooled", default="", help="npz path: per-(locale, cycle) charged load per arm + per-locale density/tiles/degree (offline window selection)")
    ap.add_argument("--record_bytes", type=float, default=2600.0); ap.add_argument("--delta_bytes", type=float, default=120.0)
    ap.add_argument("--subsample", type=int, default=None); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--max_rows", type=int, default=None)
    run(ap.parse_args())

if __name__ == "__main__":
    main()
