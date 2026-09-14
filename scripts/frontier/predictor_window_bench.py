#!/usr/bin/env python3
"""Predictor window microbench (design: docs/agent_plans/tracker_window_microbench_design.md, Block B).
MTR forward time per track with past_frames in {10, 20, 40} (the MTREdgePredictor constructor
argument; obj_trajs is built (n, past_frames + 1, 8) from the track history, so the tensor
shapes follow), n tracks in {1, 2, 8} (1 = the isolated call, 2 = the padded isolated call,
8 = a batch), checkpoint fixed, outputs discarded. >= 20 reps after warm-up, cuda synchronize
around each call. Run inside the campaign worktree with the overlay applied
(scripts/diag/run_in_overlay_wt.sh). Rows: past_frames, n_tracks, predict_ms_per_track, rep.
  python scripts/frontier/predictor_window_bench.py --out evaluation_outputs/frontier/predictor_window --reps 20
"""
import argparse, csv, json, os, sys, time, traceback
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "metro"))   # microbench_ops (main checkout script); ecav resolves via PYTHONPATH to the worktree
from microbench_ops import build_predictor, make_trajectories
import torch
PAST = (10, 20, 40); SIZES = (1, 2, 8)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--reps", type=int, default=20); ap.add_argument("--tag", default="frozen1r")
    ap.add_argument("--edge_yaml", default="ecav/scenario_testing/config_yaml/openscenario_1_flow_gt.yaml"); a = ap.parse_args()
    out = os.path.abspath(a.out); os.makedirs(out, exist_ok=True); rows = []; meta = {"cells": {}, "gpu": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None, "load_1min": os.getloadavg()[0]}
    for P in PAST:
        try:
            pred = build_predictor(a.edge_yaml, overrides={"past_frames": P})
        except Exception as e:
            meta["cells"][f"P{P}"] = {"error": repr(e), "trace": traceback.format_exc()[-1500:]}; print(f"past_frames={P}: constructor rejected: {e!r}", flush=True); continue
        sub = int(getattr(pred, "_subsample", 1)); FR = (P + 2) * sub + 4
        for n in SIZES:
            try:
                for w in range(3):
                    pred._run_mtr(make_trajectories(pred, n, seed=100 + w, frames=FR), 500 + w, 500 + w); torch.cuda.synchronize()
                ms = []
                for r in range(a.reps):
                    tracks = make_trajectories(pred, n, seed=1 + r, frames=FR)
                    torch.cuda.synchronize(); t0 = time.perf_counter(); pred._run_mtr(tracks, 600 + r, 600 + r); torch.cuda.synchronize(); ms.append((time.perf_counter() - t0) * 1e3)
                for r, m in enumerate(ms, 1): rows.append({"past_frames": P, "n_tracks": n, "predict_ms_per_track": round(m / n, 4), "rep": r, "predict_ms": round(m, 4), "history_subsample": sub, "frames_per_track": FR})
                meta["cells"][f"P{P}_n{n}"] = {"median_ms": float(np.median(ms)), "p10": float(np.percentile(ms, 10)), "p90": float(np.percentile(ms, 90))}
                print(f"past_frames={P:2d} n={n}: median {np.median(ms):7.2f} ms ({np.median(ms)/n:6.2f}/track) p10 {np.percentile(ms,10):7.2f} p90 {np.percentile(ms,90):7.2f}", flush=True)
            except Exception as e:
                meta["cells"][f"P{P}_n{n}"] = {"error": repr(e), "trace": traceback.format_exc()[-1500:]}; print(f"past_frames={P} n={n}: call rejected: {e!r}", flush=True)
    if rows:
        p = os.path.join(out, f"{a.tag}_predictor_window_rows.csv")
        with open(p, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
        print("written", p)
    json.dump(meta, open(os.path.join(out, "settings.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
