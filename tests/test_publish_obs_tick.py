"""A publish on a tick with no newly drained frame still carries each track's
observation tick: the age logger must never read such a publish as age zero.
Runs standalone."""
import os
import sys
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecav.core.application.edge.edge_manager.edge_manager_base import _BaseEdgeManager  # noqa: E402


class _Trk:
    def __init__(self, tid, last_obs_tick, tsu=0):
        self.id, self.last_obs_tick, self.time_since_update = tid, last_obs_tick, tsu


class _Pred:
    def __init__(self, tid):
        self.obstacle_trajectory = types.SimpleNamespace(obstacle=types.SimpleNamespace(track_id=tid))


class _Predictor:
    def generate_predicted_trajectories(self, trajs, source_tick=None, publish_tick=None):
        return [_Pred(1), _Pred(2)]


def main():
    host = types.SimpleNamespace(_age_logger=types.SimpleNamespace(note_publish=lambda t: None),
                                 tracker=types.SimpleNamespace(trackers=[_Trk(1, 206), _Trk(2, 205, tsu=1)],
                                                               _cur_frame_tick=206))
    wrapped = _BaseEdgeManager._wrap_predictor_for_age_log(host, _Predictor())
    # a frame drained this tick: stamped from the tracks' last observations
    preds = wrapped.generate_predicted_trajectories({}, source_tick=206, publish_tick=214)
    assert [p.obs_tick for p in preds] == [206, 205], [p.obs_tick for p in preds]
    # no frame drained this tick (source_tick None): still stamped, from the
    # tracker's last ingested frame and the tracks' last observations
    preds = wrapped.generate_predicted_trajectories({}, source_tick=None, publish_tick=215)
    assert [p.obs_tick for p in preds] == [206, 205], [p.obs_tick for p in preds]
    print("publish without a new frame keeps obs_tick 206/205 (never age zero)")


if __name__ == "__main__":
    main()
