#!/usr/bin/env python3
"""Tracker window microbench (design: docs/agent_plans/tracker_window_microbench_design.md, Block A + A2).
Per-tick tracker.update cost per track at max_window in {1,2,5,10,20} for 1 and 8
tracks (deployed head, enable_time_thresh 1000000), the learned head's predict()
per tracklet at the same depths (harness threshold 5; labelled not deployed), and
the import cost of a record of depth K in {1,2,5,10} (create: latent + serialize +
deserialize + inject into a fresh tracker). Same tracker code path and GPU as the
deployed Mamba tracker. Warm-up 5, >= 20 reps per cell.
  python scripts/frontier/tracker_window_bench.py --out evaluation_outputs/frontier/tracker_window --reps 20
"""
import argparse, csv, json, os, subprocess, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", "..")); sys.path.insert(0, ROOT)
import torch
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
from ecav.core.application.edge.migration.factories import inject_latent_into_tracker, latent_from_tracklet
from ecav.core.application.edge.migration.payload import MigrationPayload
DT = 0.05; WEIGHTS = os.path.join(ROOT, "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")
WINDOWS = (1, 2, 5, 10, 20); SIZES = (1, 8); DEPTHS = (1, 2, 5, 10)

def cfg_for(window, head):
    # the deployed tracker block (corridor/flow yaml) with the window varied
    return {"motion_model_path": WEIGHTS, "filter_thresh": 0.05, "new_track_thresh": 0.2, "match_thresh": 0.95, "max_time_lost": 60,
            "enable_time_thresh": (1000000 if head == "cv" else 5), "max_window": window, "lost_match_dist_m": 8.0, "coast_window": 40, "_spf_live": DT}

def dets_at(n, k, speed=12.0):
    """n detections at frame k: lanes 4 m apart, advancing speed*DT per frame along +x."""
    out = np.zeros((n, 7), dtype=np.float32)
    for i in range(n): out[i] = [50.0 * i + speed * DT * k, 4.0 * i, 0.5, 4.5, 2.0, 1.5, 0.0]
    return out

def sync(): 
    if torch.cuda.is_available(): torch.cuda.synchronize()

def bench_update(window, n, reps, head, device):
    trk = Mamba3DTracker(cfg_for(window, head), device)
    scores = np.full(n, 0.9, dtype=np.float32); k = 0
    for _ in range(max(window, 2) + 5):     # fill the banks to the window depth, then warm up
        trk.update(dets_at(n, k), scores); k += 1
    assert len(trk.tracked_tracklets) == n, f"expected {n} tracklets, got {len(trk.tracked_tracklets)}"
    depth = len(trk.tracked_tracklets[0].memo_bank)
    ms = []
    for r in range(reps):
        d = dets_at(n, k); k += 1
        sync(); t0 = time.perf_counter(); trk.update(d, scores); sync(); ms.append((time.perf_counter() - t0) * 1e3)
    return ms, depth, trk

def bench_predict_head(trk, reps):
    """The learned motion head's predict() per tracklet (only meaningful when memo >= enable_time_thresh)."""
    ms = []
    for r in range(reps):
        sync(); t0 = time.perf_counter()
        for t in trk.tracked_tracklets: t.predict()
        sync(); ms.append((time.perf_counter() - t0) * 1e3)
    return ms

def bench_import(depth, reps, device):
    src_trk = Mamba3DTracker(cfg_for(10, "cv"), device); scores = np.full(1, 0.9, dtype=np.float32)
    for k in range(15): src_trk.update(dets_at(1, k), scores)
    src = src_trk.tracked_tracklets[0]; ms = []; nbytes = 0
    for r in range(reps + 5):
        dst = Mamba3DTracker(cfg_for(10, "cv"), device)
        sync(); t0 = time.perf_counter()
        lat = latent_from_tracklet(src, persistent_vehicle_id=7, history_depth=(None if depth == 10 else depth))
        blob = MigrationPayload(source_locale_id="a", destination_locale_id="b", trigger_time_s=0.0, tracks=[lat]).serialize()
        pl = MigrationPayload.deserialize(blob); inject_latent_into_tracker(dst, pl.tracks[0])
        sync(); dt = (time.perf_counter() - t0) * 1e3
        if r >= 5: ms.append(dt)
        nbytes = len(blob)
    return ms, nbytes

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--reps", type=int, default=20); ap.add_argument("--tag", default="frozen1r")
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); device = "cuda" if torch.cuda.is_available() else "cpu"
    git = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    rows, head_rows, imp_rows = [], [], []
    for w in WINDOWS:
        for n in SIZES:
            ms, depth, trk = bench_update(w, n, a.reps, "cv", device)
            for r, m in enumerate(ms, 1): rows.append({"window_frames": w, "n_tracks": n, "update_ms_per_track": round(m / n, 4), "rep": r, "update_ms": round(m, 4), "memo_depth": depth, "head": "cv", "enable_time_thresh": 1000000, "device": device, "git": git})
            print(f"update window={w:2d} n={n}: median {np.median(ms):.3f} ms ({np.median(ms)/n:.3f}/track) p10 {np.percentile(ms,10):.3f} p90 {np.percentile(ms,90):.3f} memo_depth {depth}", flush=True)
            msh, depth_h, trk_h = bench_update(w, n, 5, "mamba", device); pm = bench_predict_head(trk_h, a.reps)
            for r, m in enumerate(pm, 1): head_rows.append({"window_frames": w, "n_tracks": n, "predict_ms_per_track": round(m / n, 4), "rep": r, "predict_ms": round(m, 4), "memo_depth": depth_h, "head": "mamba", "enable_time_thresh": 5, "learned_head_active": int(depth_h >= 5), "device": device, "git": git})
            print(f"  learned head predict window={w:2d} n={n}: median {np.median(pm):.3f} ms ({np.median(pm)/n:.3f}/track) active={int(depth_h >= 5)}", flush=True)
    for d in DEPTHS:
        ms, nb = bench_import(d, a.reps, device)
        for r, m in enumerate(ms, 1): imp_rows.append({"depth_frames": d, "import_ms": round(m, 4), "bytes": nb, "rep": r, "device": device, "git": git})
        print(f"import depth={d:2d}: median {np.median(ms):.3f} ms p10 {np.percentile(ms,10):.3f} p90 {np.percentile(ms,90):.3f} bytes {nb}", flush=True)
    for name, rs in ((f"{a.tag}_tracker_window_rows.csv", rows), (f"{a.tag}_tracker_window_learnedhead_rows.csv", head_rows), (f"{a.tag}_record_import_depth_rows.csv", imp_rows)):
        p = os.path.join(a.out, name)
        with open(p, "w", newline="") as fh:
            wri = csv.DictWriter(fh, fieldnames=list(rs[0].keys())); wri.writeheader(); [wri.writerow(r) for r in rs]
        print("written", p)
    json.dump({"device": device, "gpu": (torch.cuda.get_device_name(0) if torch.cuda.is_available() else None), "reps": a.reps, "windows": WINDOWS, "sizes": SIZES, "depths": DEPTHS, "git": git,
               "load_1min": os.getloadavg()[0], "cadence_s": DT, "note": "deployed head = constant velocity (enable_time_thresh 1000000); learned-head rows are not deployed"}, open(os.path.join(a.out, "settings.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
