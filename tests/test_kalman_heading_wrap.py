# -*- coding: utf-8 -*-
"""Localizer Kalman filter: heading innovation must be wrapped.

A vehicle heading west reports a CARLA yaw of +180 on one tick and -180 on
the next (plus 1 degree of GNSS heading noise). Before the wrap the raw
innovation jumped by 2*pi, the yaw state settled at a wrong angle (-30, -134
or +50 degrees for seeds 1 to 3 of this replay) and the motion model then
carried a speed-proportional position bias (up to 2.8 m lateral, 7 m
longitudinal at 13.9 m/s). In closed loop that bias put the lead
east-approach CAV of the multi-ego scenarios 2.5 m left of its lane
(2026-10-13, experiment_results/*/defects_fix*, filter_yaw_error_mean_deg
155 to 264 for the westbound CAVs, under 4 for every other heading).

Run: python -m pytest tests/test_kalman_heading_wrap.py  (or python tests/...)
"""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from ecav.core.sensing.localization.kalman_filter import KalmanFilter  # noqa: E402


def _wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def replay(seed, yaw_true_deg, v=13.9, ticks=300, wobble_deg=1.0, noise_deg=1.0,
           gnss_sigma_m=0.17, dt=0.05):
    """Replay the filter on synthetic straight-line motion; return
    (mean |yaw error| deg over the second half, mean position error m)."""
    rng = random.Random(seed)
    kf = KalmanFilter(dt)
    x, y = -20.0, 130.9
    yaw_err, pos_err = [], []
    for t in range(ticks):
        yaw_t = yaw_true_deg + rng.gauss(0, wobble_deg)
        if yaw_t > 180.0:
            yaw_t -= 360.0  # CARLA reports yaw in (-180, 180]
        x += v * dt * math.cos(math.radians(yaw_t))
        y += v * dt * math.sin(math.radians(yaw_t))
        meas = yaw_t + rng.gauss(0, noise_deg)  # add_heading_direction_noise: no wrap
        gx, gy = x + rng.gauss(0, gnss_sigma_m), y + rng.gauss(0, gnss_sigma_m)
        if t == 0:
            kf.run_step_init(gx, gy, math.radians(meas), v)
            continue
        xk, yk, yawk, _ = kf.run_step(gx, gy, math.radians(meas), v, 0.0)
        if t >= ticks // 2:
            yaw_err.append(abs(_wrap_deg(math.degrees(yawk) - yaw_t)))
            pos_err.append(math.hypot(xk - x, yk - y))
    return sum(yaw_err) / len(yaw_err), sum(pos_err) / len(pos_err)


def test_westbound_heading_is_tracked():
    for seed in (1, 2, 3):
        yaw_err, pos_err = replay(seed, 179.5)
        assert yaw_err < 3.0, (seed, yaw_err)
        assert pos_err < 0.3, (seed, pos_err)


def test_westbound_matches_northbound():
    west = replay(1, 179.5)
    north = replay(1, 90.0)
    assert abs(west[1] - north[1]) < 0.1, (west, north)


def test_slow_westbound_follower():
    yaw_err, pos_err = replay(1, 179.5, v=7.8)
    assert yaw_err < 3.0 and pos_err < 0.3, (yaw_err, pos_err)


if __name__ == '__main__':
    for seed in (1, 2, 3):
        print('west seed', seed, 'yaw_err_deg=%.2f pos_err_m=%.3f' % replay(seed, 179.5))
    print('north      ', 'yaw_err_deg=%.2f pos_err_m=%.3f' % replay(1, 90.0))
    print('west 7.8m/s', 'yaw_err_deg=%.2f pos_err_m=%.3f' % replay(1, 179.5, v=7.8))
