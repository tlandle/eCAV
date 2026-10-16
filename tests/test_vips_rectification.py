"""VIPS on the late-fusion pipeline: a delayed detection of a tracked moving
car is moved to the ingest tick by the track's velocity; a detection with no
track, or with zero staleness, is left where it is. Runs standalone."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402
from ecav.core.application.edge.edge_manager.edge_manager_vips_temporal_alignment import (  # noqa: E402
    _time_rectify_detections,
)


def det_row(x, y):
    return [1.8, 2.0, 4.5, x, 0.0, y, 0.0, 0.5]


def main():
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    # a car moving at 13 m/s (0.65 m per tick) in +x, tracked for 14 frames with edges
    for k in range(1, 15):
        x = -37.0 + 0.65 * k
        trk.track({"dets": np.array([det_row(x, 127.7)[:7]], dtype=float),
                   "info": np.array([[k, 100 + k, -1]], dtype=np.int64),
                   "edges": np.array([[x - 2.25, x + 2.25, 126.7, 128.7]], dtype=float)}, k)
    x_obs = -37.0 + 0.65 * 14
    dets = {"dets": np.array([det_row(x_obs, 127.7), det_row(50.0, 50.0)], dtype=np.float32),
            "info": np.array([[14, 900, -1], [14, 901, -1]], dtype=np.int64),
            "edges": np.array([[x_obs - 2.25, x_obs + 2.25, 126.7, 128.7], [48, 52, 48, 52]], dtype=float)}
    out = _time_rectify_detections(dets, source_tick=14, current_tick=18, dt=0.05, tracker=trk)
    moved = float(out["dets"][0, 3]) - x_obs
    assert 2.0 < moved < 3.2, moved          # 4 ticks at about 0.65 m per tick
    assert abs(float(out["edges"][0, 0]) - (x_obs - 2.25 + moved)) < 1e-6
    assert float(out["dets"][1, 3]) == 50.0  # no track within 10 m: untouched
    same = _time_rectify_detections(dets, source_tick=18, current_tick=18, dt=0.05, tracker=trk)
    assert float(same["dets"][0, 3]) == np.float32(x_obs)
    print("rectification: delayed detection moved %.2f m along the track's motion, edges with it; untracked and fresh detections untouched" % moved)
    from ecav.core.application.edge.edge_manager import _EDGE_REGISTRY, VIPSLateFusionEdge
    assert _EDGE_REGISTRY["VIPS"] is VIPSLateFusionEdge
    print("registry: VIPS -> VIPSLateFusionEdge")


if __name__ == "__main__":
    main()
