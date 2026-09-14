#!/usr/bin/env python3
"""State frontier by maneuver: offline forced-handoff block (design:
docs/agent_plans/state_frontier_design.md). Eight transferred-state arms x four
maneuvers on the migration harness's trajectories; the destination forecast is
the Mamba tracker's own motion head rolled out open-loop (no MTR).

Rows: maneuver, representation, kb, fde5_m, rep (+ fde3_m, one_step_err_m,
frames_to_first_forecast, coverage, bytes, head, onset_frame, noise_sigma).
Run under opencda310 on a GPU (the mamba kernel):
  python scripts/frontier/state_frontier.py --out evaluation_outputs/frontier/smoke --head cv --onset 10
"""
import argparse, copy, csv, json, os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)
import torch
from ecav.core.application.edge.migration.harness import TRAJECTORIES, _tracker_cfg
from ecav.core.application.edge.migration.factories import inject_latent_into_tracker, latent_from_tracklet
from ecav.core.application.edge.migration.payload import KFState, MigrationPayload
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
import importlib.util
_KF_PATH = os.path.join(ROOT, "ecav/core/application/v2v/baselines/cmp/CMP/AB3Dmot/AB3DMOT_libs/kalman_filter.py")
_spec = importlib.util.spec_from_file_location("cmp_ab3dmot_kalman_filter", _KF_PATH); _kfmod = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_kfmod)
AB3DMOT_KF = _kfmod.KF   # loaded by path: another AB3DMOT_libs package is cached in sys.modules by the harness import chain

DT = 0.05; HANDOFF = 10; WINDOW = 5; H5 = 100; H3 = 60
MANEUVERS = ("straight", "brake", "turn", "lane_change")
ARMS = ("none", "position", "pos_vel", "kalman_state", "hist2", "hist5", "hist10", "no_handoff")
WEIGHTS = os.path.join(ROOT, "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth")

def make_traj(man, total, onset):
    gen = TRAJECTORIES[man]
    return gen(total) if man == "straight" else gen(total, handoff=onset)

def feed(tracker, det):
    return tracker.update(det.reshape(1, -1), np.array([0.95], dtype=np.float32))

def rollout(trk, horizons):
    """Open-loop: predict() then update(None) per frame (the tracker's own dead-reckoning path)."""
    c = copy.deepcopy(trk); out = {}
    for step in range(1, max(horizons) + 1):
        c.predict()
        if c.predicted_last_bbox is None: break
        if step in horizons: out[step] = (float(c.predicted_last_bbox[0]), float(c.predicted_last_bbox[1]))
        c.update(None, frame_id=c.frame_id + step)
    return out

def source_velocity(src):
    m = np.asarray(src.memo_bank, dtype=np.float32); k = max(len(m) - 1, 1)
    return np.array([(m[-1, 0] - m[0, 0]) / (k * DT), (m[-1, 1] - m[0, 1]) / (k * DT)], dtype=np.float32)

def kf_over(pre):
    """AB3DMOT constant-velocity KF over the pre-handoff detections (tracker box layout)."""
    kf = AB3DMOT_KF(np.asarray(pre[0], dtype=np.float64), None, 0, 0)   # (bbox3D, info, ID, cav_id)
    for det in pre[1:]:
        kf.kf.predict(); kf.kf.update(np.asarray(det, dtype=np.float64).reshape(7, 1)); kf.hits += 1
    return kf

def build_destination(arm, cfg, device, src, pre, edge_a):
    """Return (tracker, payload_bytes)."""
    if arm == "no_handoff": return edge_a, 0
    if arm == "none": return Mamba3DTracker(cfg, device=device), 0
    depth = {"position": 1, "pos_vel": 1, "kalman_state": 1, "hist2": 2, "hist5": 5, "hist10": None}[arm]
    vel = None
    if arm == "pos_vel": vel = source_velocity(src)
    latent = latent_from_tracklet(src, persistent_vehicle_id=42, last_observation_t=0.5, history_depth=depth, vel_mps=vel)
    if arm == "kalman_state":
        kf = kf_over(pre); x = kf.kf.x.reshape(-1).astype(np.float32)
        latent.kf_state = KFState(state_vector=x, covariance=kf.kf.P.astype(np.float32), hits=int(kf.hits), anchoring_age=0)
        latent.memo_bank = np.asarray([x[:7]], dtype=np.float32); latent.bbox_3d = x[:7].copy()
        latent.vel_mps = np.array([x[7] / DT, x[8] / DT], dtype=np.float32)   # per-frame velocity -> m/s
    payload = MigrationPayload(source_locale_id="A", destination_locale_id="B", trigger_time_s=HANDOFF * DT, tracks=[latent])
    nbytes = len(payload.serialize())
    dst = Mamba3DTracker(cfg, device=device); inject_latent_into_tracker(dst, latent)
    return dst, nbytes

def run_cell(man, arm, cfg, device, onset, sigma, rng):
    total = HANDOFF + WINDOW + H5 + 1
    traj = make_traj(man, total, onset); truth = traj.copy()
    dets = traj.copy()
    if sigma > 0: dets[:, :2] += rng.normal(0.0, sigma, size=(len(dets), 2)).astype(dets.dtype)
    pre, post = dets[:HANDOFF], dets[HANDOFF:]
    edge_a = Mamba3DTracker(cfg, device=device)
    for det in pre: feed(edge_a, det)
    src = edge_a.tracked_tracklets[0]
    dst, nbytes = build_destination(arm, cfg, device, src, pre, edge_a)
    fde5, fde3, one = [], [], []; cover = 0; first = None
    for f in range(WINDOW):
        det = post[f]; feed(dst, det)
        if not dst.tracked_tracklets: continue
        trk = dst.tracked_tracklets[0]
        if trk.predicted_last_bbox is not None:
            one.append(float(np.hypot(trk.predicted_last_bbox[0] - det[0], trk.predicted_last_bbox[1] - det[1])))
        can = not (trk.predicted_last_bbox is None and len(trk.memo_bank) < 2 and getattr(trk, "_migrated_vel_mps", None) is None)
        last = (float(trk.memo_bank[-1][0]), float(trk.memo_bank[-1][1]))
        if can:
            eps = rollout(trk, (H3, H5)); ep3 = eps.get(H3, last); ep5 = eps.get(H5, last); cover += 1
            if first is None: first = f
        else:
            ep3 = ep5 = last
        g3 = truth[HANDOFF + f + H3]; g5 = truth[HANDOFF + f + H5]
        fde3.append(float(np.hypot(ep3[0] - g3[0], ep3[1] - g3[1]))); fde5.append(float(np.hypot(ep5[0] - g5[0], ep5[1] - g5[1])))
    return {"fde5_m": float(np.mean(fde5)) if fde5 else float("nan"), "fde3_m": float(np.mean(fde3)) if fde3 else float("nan"),
            "one_step_err_m": float(np.mean(one)) if one else float("nan"), "frames_to_first_forecast": first if first is not None else -1,
            "coverage": cover / WINDOW, "bytes": nbytes, "kb": round(nbytes / 1000.0, 3)}

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--head", choices=("cv", "mamba"), default="cv")
    ap.add_argument("--onset", type=int, default=HANDOFF, help="maneuver onset frame (10 = at the handoff, the harness definition)")
    ap.add_argument("--noise_sigma", type=float, default=0.0); ap.add_argument("--reps", type=int, default=1); ap.add_argument("--tag", default="frozen1r")
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True); t0 = time.time()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = _tracker_cfg(WEIGHTS); cfg["_spf_live"] = DT
    if a.head == "cv": cfg["enable_time_thresh"] = 1000000      # the deployed head (flow/corridor yamls)
    rows = []
    for rep in range(1, a.reps + 1):
        rng = np.random.default_rng(1000 + rep)
        for man in MANEUVERS:
            for arm in ARMS:
                r = run_cell(man, arm, cfg, device, a.onset, a.noise_sigma, rng)
                rows.append({"maneuver": man, "representation": arm, "kb": r["kb"], "fde5_m": round(r["fde5_m"], 3), "rep": rep,
                             "fde3_m": round(r["fde3_m"], 3), "one_step_err_m": round(r["one_step_err_m"], 3), "frames_to_first_forecast": r["frames_to_first_forecast"],
                             "coverage": r["coverage"], "bytes": r["bytes"], "head": a.head, "onset_frame": a.onset, "noise_sigma": a.noise_sigma})
    path = os.path.join(a.out, f"{a.tag}_state_frontier_rows.csv")
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
    json.dump({"head": a.head, "onset_frame": a.onset, "noise_sigma": a.noise_sigma, "reps": a.reps, "window": WINDOW, "handoff": HANDOFF, "dt": DT,
               "enable_time_thresh": cfg["enable_time_thresh"], "weights": os.path.relpath(WEIGHTS, ROOT), "device": device, "kf_module": os.path.relpath(_KF_PATH, ROOT), "rows": len(rows), "seconds": round(time.time() - t0, 1)},
              open(os.path.join(a.out, "settings.json"), "w"), indent=1)
    print(f"head={a.head} onset={a.onset} sigma={a.noise_sigma} reps={a.reps} ({time.time()-t0:.0f}s)")
    print(f"{'maneuver':12s} {'arm':13s} {'kb':>6s} {'fde5':>8s} {'fde3':>8s} {'1step':>7s} {'first':>5s} {'cov':>4s}")
    for r in rows:
        if r["rep"] == 1: print(f"{r['maneuver']:12s} {r['representation']:13s} {r['kb']:6.3f} {r['fde5_m']:8.3f} {r['fde3_m']:8.3f} {r['one_step_err_m']:7.3f} {r['frames_to_first_forecast']:5d} {r['coverage']:4.1f}")
    print("written", path)

if __name__ == "__main__":
    main()
