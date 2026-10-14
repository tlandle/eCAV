"""Observation lineage through tracker coasting: a track that stops receiving
detections keeps the tick of its last matched observation, and that tick equals
the frame clock minus time_since_update. Runs standalone (no pytest)."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402


def det(x, y):
    # [h, w, l, x, y, z, theta] per AB3DMOT's detection layout
    return np.array([1.5, 2.0, 4.5, x, y, 0.0, 0.0], dtype=float)


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
