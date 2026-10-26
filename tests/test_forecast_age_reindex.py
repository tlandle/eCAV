"""Forecast re-indexing by age: the consumed forecast starts at the present,
keeps its length, and continues at the last segment's velocity. Runs
standalone under the simulation env (needs carla)."""
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import carla  # noqa: E402
from ecav.core.plan.forecast_age import shift_trajectory, reindex_forecast_by_age  # noqa: E402


class _Pred:
    def __init__(self, traj, source_tick, origin_tick=None):
        self.predicted_trajectory = traj
        self.source_tick = source_tick
        self.origin_tick = origin_tick


def _traj(n, v=0.65):
    return [carla.Transform(carla.Location(x=-37.0 + v * (j + 1), y=127.7, z=0.0),
                            carla.Rotation()) for j in range(n)]


def main():
    t = _traj(60)
    s = shift_trajectory(t, 14)                   # 700 ms at 50 ms steps
    assert len(s) == 60
    assert abs(s[0].location.x - t[14].location.x) < 1e-9
    assert abs(s[-1].location.x - (t[-1].location.x + 0.65 * 14)) < 1e-6
    s2 = shift_trajectory(t, 70)                  # present beyond the end
    assert len(s2) == 60 and abs(s2[0].location.x - (t[-1].location.x + 0.65 * 11)) < 1e-6
    assert shift_trajectory(t, 0) is t
    p = _Pred(_traj(60), source_tick=100)
    q = reindex_forecast_by_age(p, now_tick=115)  # k = 14
    assert q is not p and q.age_reindex_steps == 14 and len(q.predicted_trajectory) == 60
    assert abs(q.predicted_trajectory[0].location.x - p.predicted_trajectory[14].location.x) < 1e-9
    assert abs(p.predicted_trajectory[0].location.x - (-37.0 + 0.65)) < 1e-4   # original untouched (float32)
    q2 = reindex_forecast_by_age(q, now_tick=116)  # held pred re-indexed from the original
    assert q2.age_reindex_steps == 15
    assert reindex_forecast_by_age(p, now_tick=101) is p           # k = 0
    assert reindex_forecast_by_age(p, now_tick=None) is p
    r = reindex_forecast_by_age(_Pred(_traj(60), source_tick=100, origin_tick=110), now_tick=115)
    assert r.age_reindex_steps == 4                                 # origin wins over source
    print("ok")


if __name__ == "__main__":
    main()
