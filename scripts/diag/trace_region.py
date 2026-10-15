"""Replay a run's edge tracker and print every track and measurement inside a
ground region per frame: how a track's published velocity arose.

    python scripts/diag/trace_region.py RUN_DIR --ticks T0 T1 --region XMIN XMAX YMIN YMAX
"""
import argparse, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay_edge_tracker as R  # noqa: E402
import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--ticks", type=int, nargs=2, required=True)
    ap.add_argument("--region", type=float, nargs=4, required=True)
    ap.add_argument("--nms-thresh", type=float, default=5.0)
    ap.add_argument("--max-age", type=int, default=6)
    ap.add_argument("--ext", type=float, nargs=3, default=[4.5, 2.0, 1.5])
    ap.add_argument("--beacon-ext", type=float, nargs=3, default=[4.6, 1.95, 1.85])
    a = ap.parse_args()
    xmin, xmax, ymin, ymax = a.region
    frames, _ = R.parse_frames(os.path.join(a.run, "runner_stdout.txt"))
    tracker = R.make_tracker(a.max_age, anchoring=True)
    state = {}
    for tick, fdets in frames:
        bundle = R.build_dets(fdets, tick, tuple(a.ext), tuple(a.beacon_ext), state)
        bundle = R.cross_source_nms(bundle, a.nms_thresh)
        res, _ = tracker.track(bundle, tick)
        if tick < a.ticks[0] or tick > a.ticks[1]:
            continue
        dets = bundle["dets"]
        info = bundle["info"]
        meas = [f"({d[3]:.2f},{d[5]:.2f} c{d[7]:.2f})" for d, i in zip(dets, info)
                if xmin <= d[3] <= xmax and ymin <= d[5] <= ymax]
        rows = res[0] if res and len(res[0]) else np.zeros((0, 16))
        live = {trk.id: trk for trk in tracker.trackers}
        print(f"tick {tick}: meas {' '.join(meas) if meas else '-'}")
        for row in rows:
            x, y = float(row[3]), float(row[5])
            if not (xmin <= x <= xmax and ymin <= y <= ymax):
                continue
            tid = int(row[7])
            pub_v = math.hypot(float(row[10]), float(row[12])) / R.DT
            trk = live.get(tid)
            h = getattr(trk, "pos_hist", None) if trk is not None else None
            vel = trk.kf.x.reshape(-1)[7:10] if trk is not None else [0, 0, 0]
            kf_v = math.hypot(float(vel[0]), float(vel[2])) / R.DT
            hs = ""
            if h:
                (f0, x0, z0), (f1, x1, z1) = h[0], h[-1]
                hs = f" hist n={len(h)} f{f0}..f{f1} ({x0:.2f},{z0:.2f})->({x1:.2f},{z1:.2f}) d={math.hypot(x1-x0, z1-z0):.2f}"
            print(f"   trk {tid}: pos ({x:.2f},{y:.2f}) pub_v {pub_v:.2f} kf_v {kf_v:.2f} tsu {trk.time_since_update if trk else '?'} hits {trk.hits if trk else '?'}{hs}")


if __name__ == "__main__":
    main()
