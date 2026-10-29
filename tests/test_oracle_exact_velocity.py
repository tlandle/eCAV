"""The exact-state reference publishes each actor's exact velocity at the source tick in place
of the tracker's filtered slope, which lags an accelerating actor. Anonymous tracks and tracks
whose actor is not in the snapshot keep the tracker's estimate. Runs standalone."""
import os
import sys
from types import SimpleNamespace
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecav.core.application.edge.edge_manager.edge_manager_oracle_ab3dmot_linear_predictor import exact_velocity_override  # noqa: E402

DT = 0.05


def _track(cid, vx_tick, vy_tick, speed):
    return SimpleNamespace(obstacle=SimpleNamespace(carla_id=cid, kf_vx=vx_tick, kf_vy=vy_tick, kf_speed_mps=speed))


def main():
    # crossing car accelerating: exact 13.0 m/s along -x, the tracker's slope says 11.9 m/s
    crossing = _track(675, -11.9 * DT, 0.0, 11.9)
    anonymous = _track(-1, 0.3, 0.0, 6.0)
    unlisted = _track(900, 0.1, 0.1, 2.8)
    snapshot = {675: {'vx': -13.0, 'vy': 0.0, 'speed': 13.0}, 674: {'vx': 0.0, 'vy': 18.3, 'speed': 18.3}}
    n = exact_velocity_override({1: crossing, 2: anonymous, 3: unlisted}, snapshot, DT)
    assert n == 1, n
    assert abs(crossing.obstacle.kf_vx - (-13.0 * DT)) < 1e-9 and crossing.obstacle.kf_vy == 0.0
    assert crossing.obstacle.kf_speed_mps == 13.0
    # 60 steps of the overridden velocity cover 39 m, the tracker's slope only 35.7 m
    assert abs(60 * crossing.obstacle.kf_vx) - 39.0 < 1e-6 and 60 * 11.9 * DT < 36.0
    assert anonymous.obstacle.kf_vx == 0.3 and anonymous.obstacle.kf_speed_mps == 6.0
    assert unlisted.obstacle.kf_vx == 0.1 and unlisted.obstacle.kf_speed_mps == 2.8
    # a stopped actor keeps a stationary forecast
    waiting = _track(675, 0.4 * DT, 0.0, 0.4)
    assert exact_velocity_override({1: waiting}, {675: {'vx': 0.0, 'vy': 0.0, 'speed': 0.0}}, DT) == 1
    assert waiting.obstacle.kf_speed_mps == 0.0 and waiting.obstacle.kf_vx == 0.0
    # no snapshot: nothing changes
    assert exact_velocity_override({1: crossing}, None, DT) == 0
    print("test_oracle_exact_velocity: ok")


if __name__ == "__main__":
    main()
