#!/usr/bin/env python3
"""Measured per-operation costs for the metropolitan replay (experiment 2,
step 2): create (record import), delta apply, predict, at batch sizes
{1,2,4,8,16,32}, on the REAL code paths of the freeze-1p overlay. Replaces
gen_batchcost's linear placeholders.

Run from a worktree with the freeze-1p overlay applied (scripts/atlas_branch.sh
style copy), under opencda310, on an IDLE host (the campaign must be finished;
timings on a loaded host are not the paper's numbers).

create N : source tracklets -> latent_from_tracklet x N -> ONE MigrationPayload
           serialize -> deserialize -> inject_latent_into_tracker x N into the
           destination tracker. One serialization and one deserialization per
           batch (the "genuinely batch" rule); the per-record inject is the
           real tracker path.
delta N  : an N-observation delta message (replica id + newest bbox + tick per
           record) -> one pickle round trip -> tracklet.update() on each replica
           (the tracker's own observation-apply path). There is no separate
           delta-replication path in the runner (every resend is a full record);
           this is the minimal real operation and is labelled as such.
predict N: the MTR call on N mature tracks through the overlay predictor's
           _run_mtr (GPU); batch 1 anchors to the campaign's [MTRTIME] n=1.
Output: JSON with per-op per-batch median/p10/p90 ms over REPS, the linear
fit (fixed + per_record), and the generator placeholders beside them.
"""
import argparse, json, os, pickle, sys, time
import numpy as np

def _now(): return time.perf_counter()

def build_tracker(device):
    from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
    cfg = {"motion_model_path": "ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth",
           "filter_thresh": 0.05, "new_track_thresh": 0.2, "match_thresh": 0.95,
           "max_time_lost": 60, "enable_time_thresh": 1000000, "max_window": 10,
           "lost_match_dist_m": 8.0, "coast_window": 40}
    return Mamba3DTracker(cfg, device)

def make_source_tracklets(tracker, n, depth=10, seed=0):
    """n mature tracklets with `depth` memo frames (12 m/s eastbound, 0.05 s ticks)."""
    from ecav.core.tracking.mamba3dmot.tracklet import MambaTracklet3D
    rng = np.random.default_rng(seed); out = []
    for i in range(n):
        x0, y0 = rng.uniform(-200, 200), rng.uniform(-200, 200)
        t = MambaTracklet3D(cfgs=tracker.cfgs, bbox_3d=[x0, y0, 0.5, 4.5, 2.0, 1.5, 0.0], score=0.9, device=tracker.device)
        t.track_id = 1000 + i; t.frame_id = depth; t.start_frame = 0; t.is_activated = True
        t.memo_bank = [np.array([x0 + 0.6 * k, y0, 0.5, 4.5, 2.0, 1.5, 0.0], dtype=np.float32) for k in range(depth)]
        t.diff_memo_bank = [np.zeros(7, dtype=np.float32)] + [np.array([0.6, 0, 0, 0, 0, 0, 0], dtype=np.float32)] * (depth - 1)
        t.memo_tick = list(range(depth)); t._bbox_3d = t.memo_bank[-1].copy()
        t.time_since_update = 0; t.state_flag = 0
        out.append(t)
    return out

def bench_create(tracker_src, tracker_dst, N, reps):
    from ecav.core.application.edge.migration.factories import latent_from_tracklet, inject_latent_into_tracker
    from ecav.core.application.edge.migration.payload import MigrationPayload
    ms = []
    for r in range(reps):
        src = make_source_tracklets(tracker_src, N, seed=r)
        tracker_dst.tracked_tracklets = []
        t0 = _now()
        recs = [latent_from_tracklet(t, persistent_vehicle_id=5000 + i, vel_mps=np.array([12.0, 0.0], dtype=np.float32)) for i, t in enumerate(src)]
        blob = MigrationPayload(source_locale_id="a", destination_locale_id="b", trigger_time_s=0.0, tracks=recs).serialize()
        pl = MigrationPayload.deserialize(blob)
        for rec in pl.tracks:
            inject_latent_into_tracker(tracker_dst, rec)
        ms.append((_now() - t0) * 1e3)
    return ms, len(blob)

def bench_delta(tracker_dst, N, reps):
    """Replicas already installed; apply one newest observation to each."""
    from ecav.core.tracking.mamba3dmot.tracklet import MambaTracklet3D
    from ecav.core.application.edge.migration.factories import inject_latent_into_tracker, latent_from_tracklet
    ms = []
    src = make_source_tracklets(tracker_dst, N, seed=123)
    tracker_dst.tracked_tracklets = []
    for i, t in enumerate(src):
        inject_latent_into_tracker(tracker_dst, latent_from_tracklet(t, persistent_vehicle_id=6000 + i))
    by_id = {t.track_id: t for t in tracker_dst.tracked_tracklets}
    for r in range(reps):
        # delta message: (replica id, newest bbox, tick) per record, one pickle round trip
        msg = [(t.track_id, (t.memo_bank[-1] + np.array([0.6, 0, 0, 0, 0, 0, 0], dtype=np.float32)), r + 100) for t in by_id.values()]
        t0 = _now()
        recs = pickle.loads(pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL))
        for tid, bbox, tick in recs:
            rep = by_id[tid]
            obs = MambaTracklet3D(cfgs=tracker_dst.cfgs, bbox_3d=bbox.tolist(), score=0.9, device=tracker_dst.device)
            rep.update(obs, tick, source_tick=tick)
        ms.append((_now() - t0) * 1e3)
    return ms, len(pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL))

def build_predictor(edge_cfg_yaml, world_dt=0.05, overrides=None):
    """Construct the overlay MTREdgePredictor exactly as the edge manager does
    (campaign yaml mtr_predictor block), with the budget/adaptation off so
    _run_mtr runs exactly the N tracks handed to it."""
    from omegaconf import OmegaConf
    from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor
    cfg = OmegaConf.load(edge_cfg_yaml)
    mp = OmegaConf.to_container(cfg.scenario.edge_list[0].mtr_predictor, resolve=True)
    if overrides: mp.update(overrides)   # e.g. {'history_subsample': 4} for the spacing check; logged by the caller
    return MTREdgePredictor(
        cmp_root=mp.get('cmp_root'), mtr_checkpoint=mp.get('checkpoint'), cfg_yaml=mp.get('cfg_yaml'),
        intention_points_file=mp.get('intention_points_file'), dataset=mp.get('dataset', 'opv2v'),
        aggregator=mp.get('aggregator'), past_frames=mp.get('past_frames', 10), future_frames=mp.get('future_frames', 50),
        time_interval=mp.get('time_interval', 0.1), history_subsample=mp.get('history_subsample', 1),
        device=mp.get('device', 'cuda'), num_output_steps=mp.get('num_output_steps', 100), output_dt=world_dt,
        lane_map=(mp.get('lane_map') if mp.get('lane_map') != 'auto' else None),
        budget_ms=1000.0, enable_amortization=False, enable_risk_budget=False, ego_vehicles=[])

def make_trajectories(pred, N, seed=0, frames=40, dt=0.05, speed=12.0):
    """N mature ObstacleTrajectory inputs as the edge builds them: a deque of
    Transforms (newest first), obstacle with kf_speed_mps; 12 m/s along +x with
    lateral spread, 0.05 s ticks (the runtime cadence)."""
    from collections import deque
    from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
    from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
    from ecav.ecav_carla import Location, Rotation, Transform
    rng = np.random.default_rng(seed); out = {}
    for i in range(N):
        x0, y0 = rng.uniform(-100, 100), rng.uniform(-40, 40)
        obs = ObstacleVehicle(corners=np.zeros((8, 3)), o3d_bbx=None, track_id=100 + i, tick_id=0)
        obs.kf_speed_mps = speed; obs.kf_vx = speed; obs.kf_vy = 0.0; obs.carla_id = 5000 + i
        ot = ObstacleTrajectory(obs, deque(maxlen=frames))
        for k in range(frames):   # oldest first, appendleft -> newest at index 0
            ot.trajectory.appendleft(Transform(location=Location(x=x0 + speed * dt * k, y=y0, z=0.5), rotation=Rotation(yaw=0.0)))
        out[100 + i] = ot
    return out

def bench_predict(N, reps, edge_cfg_yaml, pred=None):
    """The MTR call on N mature tracks through the overlay predictor's _run_mtr
    (GPU); one warm-up call excluded."""
    import torch
    pred = pred or build_predictor(edge_cfg_yaml)
    tracks = make_trajectories(pred, N, seed=0)
    pred._run_mtr(tracks, 100, 100); torch.cuda.synchronize()
    ms = []
    for r in range(reps):
        tracks = make_trajectories(pred, N, seed=r + 1)
        torch.cuda.synchronize(); t0 = _now()
        pred._run_mtr(tracks, 100 + r, 100 + r)
        torch.cuda.synchronize(); ms.append((_now() - t0) * 1e3)
    return ms

GEN = {"create": (0.9, 3.1), "delta": (0.25, 0.15), "predict": (8.0, 2.5)}   # gen_batchcost placeholders (FIXED, PER)

def fit(batches, med):
    A = np.c_[np.ones(len(batches)), np.array(batches, dtype=float)]
    coef, *_ = np.linalg.lstsq(A, np.array(med), rcond=None)
    return float(coef[0]), float(coef[1])

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True); ap.add_argument("--reps", type=int, default=10)
    ap.add_argument("--batches", default="1,2,4,8,16,32"); ap.add_argument("--ops", default="create,delta")
    ap.add_argument("--device", default="cpu"); ap.add_argument("--edge_yaml", default="ecav/scenario_testing/config_yaml/openscenario_1_flow_gt.yaml")
    a = ap.parse_args()
    batches = [int(b) for b in a.batches.split(",")]; ops = a.ops.split(",")
    res = {"host": os.uname().nodename, "device": a.device, "reps": a.reps, "batches": batches, "ops": {}, "generator_placeholders": GEN,
           "load_1min": os.getloadavg()[0]}
    if any(o in ops for o in ("create", "delta")):
        src = build_tracker(a.device); dst = build_tracker(a.device)
        for op in [o for o in ops if o in ("create", "delta")]:
            rows = {}
            for N in batches:
                ms, nbytes = (bench_create(src, dst, N, a.reps) if op == "create" else bench_delta(dst, N, a.reps))
                rows[N] = {"median_ms": float(np.median(ms)), "p10_ms": float(np.percentile(ms, 10)), "p90_ms": float(np.percentile(ms, 90)),
                           "per_record_ms": float(np.median(ms) / N), "bytes": nbytes}
                print(f"{op} N={N:2d}: median {rows[N]['median_ms']:.3f} ms ({rows[N]['per_record_ms']:.3f}/record) bytes {nbytes}", flush=True)
            fx, pr = fit(batches, [rows[N]["median_ms"] for N in batches])
            res["ops"][op] = {"rows": rows, "fit_fixed_ms": fx, "fit_per_record_ms": pr}
    if "predict" in ops:
        rows = {}; pred = build_predictor(a.edge_yaml)
        for N in batches:
            ms = bench_predict(N, a.reps, a.edge_yaml, pred=pred)
            rows[N] = {"median_ms": float(np.median(ms)), "p10_ms": float(np.percentile(ms, 10)), "p90_ms": float(np.percentile(ms, 90)), "per_record_ms": float(np.median(ms) / N)}
            print(f"predict N={N:2d}: median {rows[N]['median_ms']:.2f} ms", flush=True)
        fx, pr = fit(batches, [rows[N]["median_ms"] for N in batches])
        res["ops"]["predict"] = {"rows": rows, "fit_fixed_ms": fx, "fit_per_record_ms": pr}
    json.dump(res, open(a.out, "w"), indent=1); print("written", a.out)

if __name__ == "__main__":
    main()
