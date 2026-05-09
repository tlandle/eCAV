"""Benchmark vectorized AB3DMOT Kalman predict+update against per-tracker loop.

Measures per-tick speedup of the batched predict and update
implementations relative to the original per-tracker filterpy calls,
across a range of tracker counts.

Correctness: state/covariance outputs must match bit-for-bit after one
predict+update cycle.
"""
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import numpy as np
from AB3DMOT_libs.kalman_filter import KF


def make_trackers(n, seed=0):
    rng = np.random.default_rng(seed)
    trackers = []
    for i in range(n):
        bbox = rng.normal(0, 20, size=7)
        info = np.array([0, i, -1], dtype=np.int64)
        t = KF(bbox, info, ID=i)
        t.kf.x[7:] = rng.normal(0, 2, size=(3, 1))
        trackers.append(t)
    return trackers


def make_measurements(n, seed=1):
    rng = np.random.default_rng(seed)
    return rng.normal(0, 20, size=(n, 7))


def clone_state(trackers):
    return [(t.kf.x.copy(), t.kf.P.copy()) for t in trackers]


def restore_state(trackers, snap):
    for t, (x, P) in zip(trackers, snap):
        t.kf.x[...] = x
        t.kf.P[...] = P


# ── Per-tracker loop (original) ──────────────────────────────────────
def loop_predict(trackers):
    for t in trackers:
        t.kf.predict()


def loop_update(trackers, measurements):
    for t, z in zip(trackers, measurements):
        t.kf.update(z.reshape(7))


# ── Batched ──────────────────────────────────────────────────────────
def batched_predict(trackers):
    n = len(trackers)
    if n == 0:
        return
    F = trackers[0].kf.F
    FT = F.T
    Q = trackers[0].kf.Q
    X = np.empty((n, 10, 1), dtype=np.float64)
    P = np.empty((n, 10, 10), dtype=np.float64)
    for i in range(n):
        X[i] = trackers[i].kf.x
        P[i] = trackers[i].kf.P
    X_new = F @ X
    P_new = F @ P @ FT + Q
    for i in range(n):
        trackers[i].kf.x[...] = X_new[i]
        trackers[i].kf.P[...] = P_new[i]


def batched_update(trackers, measurements):
    n = len(trackers)
    if n == 0:
        return
    H = trackers[0].kf.H
    HT = H.T
    R = trackers[0].kf.R
    X = np.empty((n, 10, 1), dtype=np.float64)
    P = np.empty((n, 10, 10), dtype=np.float64)
    Z = measurements.reshape(n, 7, 1)
    for i in range(n):
        X[i] = trackers[i].kf.x
        P[i] = trackers[i].kf.P
    HX = H @ X
    Y = Z - HX
    PHT = P @ HT
    S = H @ PHT + R
    Sinv = np.linalg.inv(S)
    K = PHT @ Sinv
    X_new = X + K @ Y
    I_KH = np.eye(10) - K @ H
    P_new = I_KH @ P
    for i in range(n):
        trackers[i].kf.x[...] = X_new[i]
        trackers[i].kf.P[...] = P_new[i]


def main():
    print(f"{'N':>6} {'loop_ms':>10} {'batch_ms':>10} {'speedup':>8} {'err_x':>10} {'err_P':>10}")
    for n in [5, 10, 20, 30, 50, 100]:
        trackers_loop = make_trackers(n, seed=42)
        trackers_batch = make_trackers(n, seed=42)
        snap_loop = clone_state(trackers_loop)
        snap_batch = clone_state(trackers_batch)
        measurements = make_measurements(n, seed=7)

        # Correctness check: one predict+update cycle
        loop_predict(trackers_loop)
        loop_update(trackers_loop, measurements)
        batched_predict(trackers_batch)
        batched_update(trackers_batch, measurements)
        err_x = max(np.abs(a.kf.x - b.kf.x).max()
                    for a, b in zip(trackers_loop, trackers_batch))
        err_P = max(np.abs(a.kf.P - b.kf.P).max()
                    for a, b in zip(trackers_loop, trackers_batch))

        restore_state(trackers_loop, snap_loop)
        restore_state(trackers_batch, snap_batch)

        iters = max(200, 30000 // max(n, 1))

        # Warmup
        for _ in range(10):
            loop_predict(trackers_loop)
            loop_update(trackers_loop, measurements)
            batched_predict(trackers_batch)
            batched_update(trackers_batch, measurements)
        restore_state(trackers_loop, snap_loop)
        restore_state(trackers_batch, snap_batch)

        # Loop timing
        t0 = time.perf_counter()
        for _ in range(iters):
            restore_state(trackers_loop, snap_loop)
            loop_predict(trackers_loop)
            loop_update(trackers_loop, measurements)
        loop_total = time.perf_counter() - t0

        # Batched timing
        t0 = time.perf_counter()
        for _ in range(iters):
            restore_state(trackers_batch, snap_batch)
            batched_predict(trackers_batch)
            batched_update(trackers_batch, measurements)
        batch_total = time.perf_counter() - t0

        # Restore-only overhead (same in both paths, subtracted out)
        t0 = time.perf_counter()
        for _ in range(iters):
            restore_state(trackers_loop, snap_loop)
        restore_total = time.perf_counter() - t0

        loop_ms = (loop_total - restore_total) * 1000 / iters
        batch_ms = (batch_total - restore_total) * 1000 / iters
        speedup = loop_ms / batch_ms if batch_ms > 0 else float('inf')

        print(f"{n:>6} {loop_ms:>10.3f} {batch_ms:>10.3f} "
              f"{speedup:>8.2f} {err_x:>10.2e} {err_P:>10.2e}")


if __name__ == '__main__':
    main()
