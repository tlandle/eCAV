#!/usr/bin/env python3
"""Does the MTR path collapse its modes at batch size 1?

Runs the overlay predictor's _run_mtr on the SAME mature track three ways and
reports, per way, the number of distinct mode endpoints, the score entropy
(uniform over 64 = 4.16 nats; 6 identical modes at 1/64 each is the collapse
signature seen in every freeze-1p MODESROW), and the best-mode displacement:
  A. alone (n=1, the cap=1 isolation call)
  B. alone + one far DUMMY agent (n=2; the dummy 300 m away, stationary)
  C. with a real second agent in the same lane (n=2, the batched regime)
If B restores diversity, the collapse is a batch-size-one property and batch
padding under isolation is a candidate fix; if B stays collapsed the locus is
elsewhere (decoder input at N=1) and needs the code read.
Runs from a worktree with the freeze-1p overlay applied (scripts/metro/run_microbench.sh
style), opencda310, GPU idle. Never during a campaign block.
"""
import argparse, json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "metro"))
from microbench_ops import build_predictor, make_trajectories

def modes_of(pred, tid, ot=None):
    ms = pred.last_mtr_modes.get(int(tid), [])
    pts = {(round(wx, 2), round(wy, 2)) for wx, wy, p in ms}
    ps = np.array([p for wx, wy, p in ms], dtype=float)
    ent = float(-(ps * np.log(np.clip(ps, 1e-12, None))).sum()) if len(ps) else None
    out = {"n_modes": len(ms), "distinct_endpoints": len(pts), "p_top": float(ps.max()) if len(ps) else None, "p_sum": float(ps.sum()) if len(ps) else None, "entropy_nats": ent}
    if ot is not None and len(ms):
        # best-mode direction: endpoint displacement from the track's newest position along its heading (+x here)
        x0, y0 = ot.trajectory[0].location.x, ot.trajectory[0].location.y
        wx, wy, p = max(ms, key=lambda m: m[2])
        out["best_dx_m"] = float(wx - x0); out["best_dy_m"] = float(wy - y0); out["best_forward"] = bool(wx - x0 > 0)
        # every mode ranked by score: (rank, p, dx, dy) so a correct forward mode outranked by a stationary one is visible
        ranked = sorted(ms, key=lambda m: -m[2])
        out["modes_ranked"] = [(i, round(float(p_), 3), round(float(wx_ - x0), 1), round(float(wy_ - y0), 1)) for i, (wx_, wy_, p_) in enumerate(ranked)]
        fw = [(i, p_, dx_) for i, p_, dx_, dy_ in out["modes_ranked"] if dx_ > 5.0]
        out["best_forward_rank"] = fw[0][0] if fw else None; out["best_forward_dx"] = fw[0][2] if fw else None; out["best_forward_p"] = fw[0][1] if fw else None
    return out

PLACEMENTS = {"behind150_adjlane": (-150.0, 3.5), "behind300_adjlane": (-300.0, 3.5)}

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--edge_yaml", default="ecav/scenario_testing/config_yaml/openscenario_1_flow_gt.yaml")
    ap.add_argument("--out", default="evaluation_outputs/diag/mtr_batch1_collapse.json"); ap.add_argument("--reps", type=int, default=5)
    a = ap.parse_args(); os.makedirs(os.path.dirname(a.out), exist_ok=True)
    pred = build_predictor(a.edge_yaml)
    res = {"A_alone": [], "C_real_pair_same_lane": []}
    for r in range(a.reps):
        base = make_trajectories(pred, 1, seed=r)                     # tid 100, 12 m/s along +x
        tid = 100
        # A
        preds = pred._run_mtr(dict(base), 200 + r, 200 + r); res["A_alone"].append(modes_of(pred, tid, base[tid]))
        # B: static dummy at several placements relative to the centre object (drives +x)
        x0 = base[tid].trajectory[0].location.x; y0 = base[tid].trajectory[0].location.y
        for name, (dx, dy) in PLACEMENTS.items():
            dummy = make_trajectories(pred, 1, seed=1000 + r, speed=0.0); dt = dummy[100]
            for tf in dt.trajectory: tf.location.x = x0 + dx; tf.location.y = y0 + dy
            dt.obstacle.track_id = 999; dt.obstacle.carla_id = 9999
            preds = pred._run_mtr({tid: base[tid], 999: dt}, 300 + r, 300 + r)
            res.setdefault("B_dummy_" + name, []).append(modes_of(pred, tid, base[tid]))
        # C: real second agent, same lane, 30 m behind, same speed
        other = make_trajectories(pred, 1, seed=2000 + r); ot = other[100]
        x0 = base[tid].trajectory[0].location.x; y0 = base[tid].trajectory[0].location.y
        for k, tf in enumerate(ot.trajectory): tf.location.x = x0 - 30.0 - 0.6 * k; tf.location.y = y0
        ot.obstacle.track_id = 101; ot.obstacle.carla_id = 5001
        preds = pred._run_mtr({tid: base[tid], 101: ot}, 400 + r, 400 + r); res["C_real_pair_same_lane"].append(modes_of(pred, tid, base[tid]))
    summary = {k: {"distinct_endpoints_mean": float(np.mean([m["distinct_endpoints"] for m in v])), "entropy_mean": float(np.mean([m["entropy_nats"] for m in v if m["entropy_nats"] is not None])),
                   "p_top_mean": float(np.mean([m["p_top"] for m in v if m["p_top"] is not None])),
                   "best_dx_median_m": float(np.median([m["best_dx_m"] for m in v if "best_dx_m" in m])) if any("best_dx_m" in m for m in v) else None,
                   "best_forward_frac": float(np.mean([m["best_forward"] for m in v if "best_forward" in m])) if any("best_forward" in m for m in v) else None} for k, v in res.items()}
    json.dump({"reps": a.reps, "per_rep": res, "summary": summary}, open(a.out, "w"), indent=1)
    for k, v in res.items():
        fr = [m.get("best_forward_rank") for m in v]; fd = [m.get("best_forward_dx") for m in v]
        print(f"{k:28s} forward mode (dx>5 m) present in {sum(1 for r_ in fr if r_ is not None)}/{len(fr)} reps; its rank by score {fr}; its dx {fd}; top-ranked modes rep0 {v[0].get('modes_ranked')}")
    for k, v in summary.items(): print(f"{k:28s} distinct endpoints {v['distinct_endpoints_mean']:.1f}  entropy {v['entropy_mean']:.2f} nats  p_top {v['p_top_mean']:.3f}  best dx median {v['best_dx_median_m']:+.1f} m  forward {v['best_forward_frac']:.2f}")
    print("written", a.out)

if __name__ == "__main__":
    main()
