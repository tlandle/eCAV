"""Observation lineage through tracker coasting: a track that stops receiving
detections keeps the tick of its last matched observation, and that tick equals
the frame clock minus time_since_update. Runs standalone (no pytest)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402


def det(x, y):
    # [h, w, l, x, y, z, theta] in AB3DMOT's KITTI layout: KITTI y is height,
    # KITTI z is CARLA y (the ground-plane columns are 3 and 5)
    return np.array([1.5, 2.0, 4.5, x, 0.0, y, 0.0], dtype=float)


def main():
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 1, "anchoring": False})
    trk = w._tracker
    fed = []
    for tick in range(1, 31):
        dets = [det(-37.0 - 0.65 * tick, 127.7)] if (tick <= 12 or tick >= 20) else []
        fed.append(tick if dets else None)
        arr = np.array(dets) if dets else np.zeros((0, 7))
        if not dets:
            info = np.zeros((0, 3), dtype=np.int64)
        # info rows are [frame_idx, guid, carla_id] (ab3dmot_format.py); -1 = anonymous
        info = np.array([[tick, 100 + tick, -1]] * len(dets), dtype=np.int64).reshape(len(dets), 3)
        trk.track({"dets": arr, "info": info}, tick)
        live = [t for t in trk.trackers if t.hits >= 1]
        if dets:
            assert any(getattr(t, "last_obs_tick", None) == tick for t in live), (tick, "no track stamped this tick")
        for t in live:
            last = getattr(t, "last_obs_tick", None)
            recon = tick - t.time_since_update
            if last is not None:
                assert last == recon, (tick, last, recon)
                if not dets:
                    assert last == 12, (tick, last)
    print("observation lineage holds through a 7-tick gap: last_obs_tick == frame - time_since_update, and 12 during the gap")


if __name__ == "__main__":
    main()


def coasting_velocity():
    """A track lost after convergence is output with its filter velocity,
    its position advancing along it, through the gap."""
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    rows_by_tick = {}
    for tick in range(1, 26):
        dets = [det(-37.0 - 0.65 * tick, 127.7)] if (tick <= 12 or tick >= 20) else []
        arr = np.array(dets) if dets else np.zeros((0, 7))
        info = (np.array([[tick, 100 + tick, -1]] * len(dets), dtype=np.int64).reshape(len(dets), 3)
                if dets else np.zeros((0, 3), dtype=np.int64))
        out, _ = trk.track({"dets": arr, "info": info}, tick)
        rows_by_tick[tick] = out
    # track() returns [array of shape (n_tracks, cols)]
    gap = [np.asarray(rows_by_tick[t][0]) for t in range(13, 20)]
    assert all(g.shape[0] == 1 for g in gap), "track dropped during a 7-tick gap"
    rows = [g[0] for g in gap]
    speeds = [float(np.hypot(r[10], r[11])) for r in rows]
    xs = [float(r[3]) for r in rows]
    assert min(speeds) > 0.3, ("coasting velocity zeroed", speeds)
    assert xs[-1] < xs[0] - 2.0, ("coasting position did not advance", xs)
    print("coasting track keeps its velocity (m/tick %.2f..%.2f) and advances %.1f m over the gap" % (min(speeds), max(speeds), xs[0] - xs[-1]))


if __name__ == "__main__":
    coasting_velocity()


def published_velocity_is_displacement():
    """A stationary object whose measured face alternates by 1.4 m between
    sensors is published at rest; a moving one at its real speed."""
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    last_static = last_moving = None
    for tick in range(1, 41):
        face = -82.4 if tick % 2 else -81.0          # alternating faces of a parked truck
        dets = [det(face, 101.7), det(-37.0 - 0.65 * tick, 127.7)]
        arr = np.array(dets)
        info = np.array([[tick, 100 + i, -1] for i in range(len(dets))], dtype=np.int64)
        out, _ = trk.track({"dets": arr, "info": info}, tick)
        for r in np.asarray(out[0]):
            spd = float(np.hypot(r[10], r[12])) / 0.05
            if abs(r[5] - 101.7) < 3:
                last_static = spd
            else:
                last_moving = spd
    assert last_static is not None and last_static < 1.0, ("parked truck published moving", last_static)
    assert last_moving is not None and 10.0 < last_moving < 16.0, ("crossing car speed", last_moving)
    print("published speed: parked truck with alternating faces %.2f m/s, crossing car %.1f m/s" % (last_static, last_moving))


if __name__ == "__main__":
    published_velocity_is_displacement()
