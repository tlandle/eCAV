#!/usr/bin/env python3
"""
Micro-benchmark validation suite for MAC model, GPS dropout,
and AB3DMOT birth gate.

27 standalone tests (no CARLA required).

Usage:
    python scripts/micro_benchmarks.py -v                       # all 27
    python scripts/micro_benchmarks.py --section mac -v         # MAC only
    python scripts/micro_benchmarks.py --section gps -v         # GPS only
    python scripts/micro_benchmarks.py --section birth_gate -v  # birth gate only

Exit 0 = all pass. Exit 1 = any failure.
"""

import argparse
import math
import os
import random
import sys
import traceback

import numpy as np

# Add parent dir so we can import repo modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# ─── Simple test framework ──────────────────────────────────────

_PASS = 0
_FAIL = 0
_VERBOSE = False


def _run(name, fn):
    global _PASS, _FAIL
    try:
        fn()
        _PASS += 1
        if _VERBOSE:
            print(f"  PASS  {name}")
    except Exception as e:
        _FAIL += 1
        print(f"  FAIL  {name}")
        if _VERBOSE:
            traceback.print_exc()
        else:
            print(f"        {e}")


# ═════════════════════════════════════════════════════════════════
#  SECTION 1: MAC MODEL (10 tests)
# ═════════════════════════════════════════════════════════════════

def mac_tests():
    from ecav.core.application.edge.latency.mac_model import (
        SbSpsMac, NullMac, LegacyMacAdapter, MACMetrics,
        create_mac_model, reset_global_macs,
    )

    # ── Test 1: NullMac always delivers ──────────────────────────
    def test_null_mac_all_delivered():
        mac = NullMac()
        for t in range(100):
            result = mac.attempt_tick(t, [1, 2, 3])
            assert all(result.values()), f"Tick {t}: not all delivered"
        assert mac.metrics.prr() == 1.0, \
            f"PRR={mac.metrics.prr()}, expected 1.0"
        assert mac.metrics.collision_drops == 0
        assert mac.metrics.fading_drops == 0

    # ── Test 2: SB-SPS pigeonhole (M=2, 3 senders) ──────────────
    def test_sbsps_pigeonhole():
        mac = SbSpsMac(M=2, seed=42, p_loss_base=0.0)
        for t in range(100):
            result = mac.attempt_tick(t, [0, 1, 2])
            delivered_count = sum(1 for v in result.values() if v)
            assert delivered_count <= 1, \
                f"Tick {t}: {delivered_count} delivered with M=2, 3 senders"
        assert mac.metrics.collision_drops > 0, \
            "Expected collision drops with M=2, 3 senders"
        assert mac.metrics.fading_drops == 0, \
            f"Expected 0 fading drops (p_loss_base=0), got {mac.metrics.fading_drops}"

    # ── Test 3: Tick idempotency ─────────────────────────────────
    def test_tick_idempotency():
        mac = SbSpsMac(M=10, seed=42, p_loss_base=0.0)
        r1 = mac.attempt_tick(0, [1, 2, 3])
        r2 = mac.attempt_tick(0, [1, 2, 3])
        assert r1 == r2, f"r1={r1} != r2={r2}"
        assert mac.metrics.attempted == 3, \
            f"Expected 3 attempts, got {mac.metrics.attempted}"

    # ── Test 4: Subset tolerance ─────────────────────────────────
    def test_subset_tolerance():
        mac = SbSpsMac(M=10, seed=42, p_loss_base=0.0)
        full = mac.attempt_tick(0, [1, 2, 3])
        subset = mac.attempt_tick(0, [2])
        assert subset[2] == full[2], \
            f"subset[2]={subset[2]} != full[2]={full[2]}"

    # ── Test 5: New sender raises ────────────────────────────────
    def test_new_sender_raises():
        mac = SbSpsMac(M=10, seed=42, p_loss_base=0.0)
        mac.attempt_tick(0, [1, 2, 3])
        try:
            mac.attempt_tick(0, [1, 99])
            assert False, "Should have raised RuntimeError"
        except RuntimeError as e:
            assert "99" in str(e), f"Error should mention sender 99: {e}"

    # ── Test 6: Burst persistence ────────────────────────────────
    def test_burst_persistence():
        mac = SbSpsMac(M=10, seed=42, p_loss_base=0.0, p_keep=0.4)
        senders = list(range(4))
        for t in range(10000):
            mac.attempt_tick(t, senders)
        mac.metrics.finalize()
        summary = mac.metrics.get_summary()
        hist = summary['burst_histogram']
        assert hist, "Burst histogram is empty"
        max_burst = max(hist.keys())
        assert max_burst > 1, \
            f"Expected multi-tick bursts, max burst length={max_burst}"
        assert summary['mac_burstiness_factor'] > 1.0, \
            f"Burstiness factor={summary['mac_burstiness_factor']}, expected >1.0"

    # ── Test 7: PRR monotonicity ─────────────────────────────────
    def test_prr_monotonicity():
        prrs = {}
        for N in [2, 8, 16]:
            mac = SbSpsMac(M=20, seed=42, p_loss_base=0.02)
            senders = list(range(N))
            for t in range(5000):
                mac.attempt_tick(t, senders)
            prrs[N] = mac.metrics.prr()
        assert prrs[2] > prrs[8] > prrs[16], \
            f"PRR not monotonically decreasing: {prrs}"

    # ── Test 8: Collision dominance at high N ────────────────────
    def test_collision_dominance():
        mac = SbSpsMac(M=20, seed=42, p_loss_base=0.02)
        senders = list(range(16))
        for t in range(10000):
            mac.attempt_tick(t, senders)
        mac.metrics.finalize()
        assert mac.metrics.collision_drops > 2 * mac.metrics.fading_drops, \
            (f"Expected collision >> fading: "
             f"coll={mac.metrics.collision_drops}, "
             f"fading={mac.metrics.fading_drops}")

    # ── Test 9: Shared registry ──────────────────────────────────
    def test_shared_registry():
        reset_global_macs()
        cfg = {"mac": {"type": "sbsps", "M": 20}}
        m1 = create_mac_model(cfg, seed=42)
        m2 = create_mac_model(cfg, seed=42)
        assert m1 is m2, "Same config+seed should return same instance"
        cfg3 = {"mac": {"type": "sbsps", "M": 20}}
        m3 = create_mac_model(cfg3, seed=99)
        assert m1 is not m3, "Different seed should return different instance"
        reset_global_macs()

    # ── Test 10: Legacy adapter ──────────────────────────────────
    def test_legacy_adapter():
        class MockLM:
            def __init__(self):
                self._count = 0
            def should_drop(self):
                self._count += 1
                return (self._count % 3) == 0  # drop every 3rd

        lm = MockLM()
        adapter = LegacyMacAdapter(lm)
        result = adapter.attempt_tick(0, [1, 2, 3])
        # should_drop calls: count=1 → False (delivered),
        # count=2 → False (delivered), count=3 → True (dropped)
        assert result[1] is True, f"Sender 1 should be delivered"
        assert result[2] is True, f"Sender 2 should be delivered"
        assert result[3] is False, f"Sender 3 should be dropped"
        assert adapter.metrics.delivered == 2
        assert adapter.metrics.attempted == 3

    return [
        ("test_null_mac_all_delivered", test_null_mac_all_delivered),
        ("test_sbsps_pigeonhole", test_sbsps_pigeonhole),
        ("test_tick_idempotency", test_tick_idempotency),
        ("test_subset_tolerance", test_subset_tolerance),
        ("test_new_sender_raises", test_new_sender_raises),
        ("test_burst_persistence", test_burst_persistence),
        ("test_prr_monotonicity", test_prr_monotonicity),
        ("test_collision_dominance", test_collision_dominance),
        ("test_shared_registry", test_shared_registry),
        ("test_legacy_adapter", test_legacy_adapter),
    ]


# ═════════════════════════════════════════════════════════════════
#  SECTION 2: GPS DROPOUT (5 tests)
# ═════════════════════════════════════════════════════════════════

class GnssDropoutSim:
    """Standalone reimplementation of GnssSensor._on_gnss_event() dropout
    state machine (localization_manager.py:114-132).

    Uses per-instance random.Random(seed) for determinism (the original
    uses the global random module).
    """

    def __init__(self, dropout_pct, mean_ticks=10, seed=0):
        self._dropout_prob = dropout_pct / 100.0
        self._mean_ticks = max(mean_ticks, 1)
        self._dropout_remaining = 0
        self._rng = random.Random(seed)
        self.lat = 0.0
        self.lon = 0.0

    def step(self, new_lat, new_lon):
        """Process one tick. Returns True if GNSS reading accepted."""
        # Phase 1: currently inside a dropout burst
        if self._dropout_remaining > 0:
            self._dropout_remaining -= 1
            return False

        # Phase 2: burst initiation check
        if self._dropout_prob > 0.0 and self._rng.random() < self._dropout_prob:
            duration = int(
                max(1, self._rng.expovariate(1.0 / self._mean_ticks)))
            self._dropout_remaining = duration
            return False  # first tick of burst is also suppressed

        # Phase 3: normal operation — accept reading
        self.lat = new_lat
        self.lon = new_lon
        return True


def gps_tests():

    # ── Test 11: 0% dropout → all accepted ───────────────────────
    def test_dropout_zero_pct():
        sim = GnssDropoutSim(dropout_pct=0, seed=42)
        accepted = sum(sim.step(i * 0.001, i * 0.001) for i in range(1000))
        assert accepted == 1000, f"Expected 1000 accepted, got {accepted}"

    # ── Test 12: 100% dropout → all suppressed ───────────────────
    def test_dropout_hundred_pct():
        sim = GnssDropoutSim(dropout_pct=100, mean_ticks=10, seed=42)
        sim.lat = 99.0  # sentinel
        sim.lon = 99.0
        accepted = 0
        for i in range(1000):
            if sim.step(i * 0.001, i * 0.001):
                accepted += 1
        assert accepted == 0, f"Expected 0 accepted, got {accepted}"
        assert sim.lat == 99.0, \
            f"lat should never update at 100% dropout, got {sim.lat}"
        assert sim.lon == 99.0, \
            f"lon should never update at 100% dropout, got {sim.lon}"

    # ── Test 13: Bursty dropout (10%, mean=10) ───────────────────
    def test_dropout_bursty():
        sim = GnssDropoutSim(dropout_pct=10, mean_ticks=10, seed=42)
        accepted = []
        for i in range(5000):
            accepted.append(sim.step(i, i))

        # Find burst lengths (consecutive False runs)
        burst_lengths = []
        run_len = 0
        for a in accepted:
            if not a:
                run_len += 1
            else:
                if run_len > 0:
                    burst_lengths.append(run_len)
                run_len = 0
        if run_len > 0:
            burst_lengths.append(run_len)

        assert burst_lengths, "Expected some dropout bursts"
        max_bl = max(burst_lengths)
        assert max_bl > 1, \
            f"Expected multi-tick bursts, max burst length={max_bl}"
        mean_bl = sum(burst_lengths) / len(burst_lengths)
        assert 2 < mean_bl < 30, \
            f"Mean burst length={mean_bl:.1f}, expected 2-30"

    # ── Test 14: Position hold during burst ──────────────────────
    def test_dropout_position_hold():
        sim = GnssDropoutSim(dropout_pct=0, seed=42)
        # Accept one reading to set position
        ok = sim.step(45.0, -73.0)
        assert ok, "First reading should be accepted"
        assert sim.lat == 45.0

        # Force a dropout burst manually
        sim._dropout_remaining = 5
        for i in range(5):
            ok = sim.step(99.0 + i, 99.0 + i)
            assert not ok, f"Tick {i}: should be suppressed during burst"
            assert sim.lat == 45.0, \
                f"lat should hold at 45.0, got {sim.lat}"
            assert sim.lon == -73.0, \
                f"lon should hold at -73.0, got {sim.lon}"

    # ── Test 15: Deterministic (same seed → same sequence) ───────
    def test_dropout_deterministic():
        def run_seq(seed):
            sim = GnssDropoutSim(dropout_pct=20, mean_ticks=5, seed=seed)
            return [sim.step(i, i) for i in range(500)]

        seq_a = run_seq(123)
        seq_b = run_seq(123)
        seq_c = run_seq(456)
        assert seq_a == seq_b, "Same seed should produce identical sequence"
        assert seq_a != seq_c, "Different seeds should produce different sequences"

    return [
        ("test_dropout_zero_pct", test_dropout_zero_pct),
        ("test_dropout_hundred_pct", test_dropout_hundred_pct),
        ("test_dropout_bursty", test_dropout_bursty),
        ("test_dropout_position_hold", test_dropout_position_hold),
        ("test_dropout_deterministic", test_dropout_deterministic),
    ]


# ═════════════════════════════════════════════════════════════════
#  SECTION 3: BIRTH GATE (12 tests)
# ═════════════════════════════════════════════════════════════════

def birth_gate_tests():
    from types import SimpleNamespace
    from AB3DMOT_libs.model import AB3DMOT
    from AB3DMOT_libs.box import Box3D
    from AB3DMOT_libs.kalman_filter import KF

    # ── Helpers ──────────────────────────────────────────────────

    def make_tracker(**overrides):
        """Create AB3DMOT with test-friendly defaults."""
        defaults = {
            'vis': False,
            'ego_com': False,
            'affi_pro': False,
            'dataset': 'KITTI',
            'det_name': 'pvrcnn',
            'anchoring': True,
            'anchoring_epoch': 40,
            'max_age': 3,
            'min_hits': 1,
            'dup_x_max': 8.0,
            'dup_y_max': 2.0,
            'dup_size_ratio': 2.5,
            'cull_consec_ticks': 3,
        }
        defaults.update(overrides)
        cfg = SimpleNamespace(**defaults)
        return AB3DMOT(cfg, cat='Car')

    def add_identified_track(tracker, x, z, theta=0.0,
                             l=4.5, w=1.8, h=1.5, carla_id=100):
        """Inject a fully-initialized identified KF track.

        KF state: [x, y, z, theta, l, w, h] where y=height (KITTI convention).
        Ground plane is indices 0 (KITTI x = CARLA x) and 2 (KITTI z = CARLA y).
        """
        bbox = np.array([x, 0.7, z, theta, l, w, h])
        info = np.array([0, 0, carla_id])
        trk = KF(bbox, info, tracker.ID_count[0])
        trk.carla_id = carla_id
        trk.guid = 0
        trk.near_identified_ticks = 0
        trk.anchoring_age = 0
        trk.hits = 3  # well-established
        tracker.trackers.append(trk)
        tracker.ID_count[0] += 1
        return trk

    def add_anon_track(tracker, x, z, theta=0.0,
                       l=4.5, w=1.8, h=1.5):
        """Inject an anonymous KF track (carla_id=-1)."""
        bbox = np.array([x, 0.7, z, theta, l, w, h])
        info = np.array([0, 0, -1])
        trk = KF(bbox, info, tracker.ID_count[0])
        trk.carla_id = -1
        trk.guid = -1
        trk.near_identified_ticks = 0
        trk.anchoring_age = 0
        tracker.trackers.append(trk)
        tracker.ID_count[0] += 1
        return trk

    def make_det_box(x, z, theta=0.0, l=4.5, w=1.8, h=1.5):
        """Create a Box3D detection at given position."""
        return Box3D(x=x, y=0.7, z=z, h=h, w=w, l=l, ry=theta, s=1.0)

    # ── Test 16: Zone on top ─────────────────────────────────────
    def test_zone_on_top():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        det = make_det_box(x=10, z=20)
        assert tracker._in_exclusion_zone(det), \
            "Detection right on top of identified track should be suppressed"

    # ── Test 17: Zone along track ────────────────────────────────
    def test_zone_along_track():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        # theta=0: along-track = x direction
        det_3m = make_det_box(x=13, z=20)   # 3m ahead
        det_7_9m = make_det_box(x=17.9, z=20)  # 7.9m ahead
        assert tracker._in_exclusion_zone(det_3m), \
            "3m along-track should be inside zone (dup_x_max=8)"
        assert tracker._in_exclusion_zone(det_7_9m), \
            "7.9m along-track should be inside zone (dup_x_max=8)"

    # ── Test 18: Zone cross-track ────────────────────────────────
    def test_zone_cross_track():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        # theta=0: cross-track = z direction
        det_3_5m = make_det_box(x=10, z=23.5)  # 3.5m lateral
        det_1_9m = make_det_box(x=10, z=21.9)  # 1.9m lateral
        assert not tracker._in_exclusion_zone(det_3_5m), \
            "3.5m cross-track should be outside zone (dup_y_max=2)"
        assert tracker._in_exclusion_zone(det_1_9m), \
            "1.9m cross-track should be inside zone (dup_y_max=2)"

    # ── Test 19: Zone angled heading (theta=pi/4) ────────────────
    def test_zone_angled_heading():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=math.pi / 4,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        # 5m along track heading (pi/4 diagonal)
        # In track frame: dx_along=5, dy_cross=0
        # World: dx = 5*cos(pi/4) = 3.536, dz = 5*sin(pi/4) = 3.536
        offset_along = 5.0 / math.sqrt(2)
        det_along = make_det_box(x=10 + offset_along, z=20 + offset_along)
        assert tracker._in_exclusion_zone(det_along), \
            "5m along track (theta=pi/4) should be inside zone (<=8)"

        # 3m perpendicular to heading
        # In track frame: dx_along=0, dy_cross=3
        # World: dx = -3*sin(pi/4) = -2.121, dz = 3*cos(pi/4) = 2.121
        perp_offset = 3.0 / math.sqrt(2)
        det_perp = make_det_box(x=10 - perp_offset, z=20 + perp_offset)
        assert not tracker._in_exclusion_zone(det_perp), \
            "3m perpendicular (theta=pi/4) should be outside zone (>2)"

    # ── Test 20: Zone size guard ─────────────────────────────────
    def test_zone_size_guard():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        det = make_det_box(x=10, z=20, l=12.0)  # truck-sized
        # ratio = 12.0 / 4.5 = 2.667 > 2.5 → size guard triggers
        assert not tracker._in_exclusion_zone(det), \
            "l=12 truck near l=4.5 car: ratio 2.67 > 2.5, should NOT suppress"

    # ── Test 21: Zone anon-only ──────────────────────────────────
    def test_zone_anon_only():
        tracker = make_tracker()
        add_anon_track(tracker, x=10, z=20)
        det = make_det_box(x=10, z=20)
        assert not tracker._in_exclusion_zone(det), \
            "CID=-1 track should NOT create exclusion zone"

    # ── Test 22: Zone stale beacon ───────────────────────────────
    def test_zone_stale_beacon():
        tracker = make_tracker()
        trk = add_identified_track(tracker, x=10, z=20, theta=0,
                                   l=4.5, w=1.8, h=1.5, carla_id=100)
        det = make_det_box(x=10, z=20)

        # Stale anchor: anchoring_age >= anchoring_epoch (40)
        trk.anchoring_age = 40
        assert not tracker._in_exclusion_zone(det), \
            "anchoring_age=40 >= epoch=40 → zone should lapse"

        # Reset anchor
        trk.anchoring_age = 0
        assert tracker._in_exclusion_zone(det), \
            "anchoring_age=0 → zone should be active"

    # ── Test 23: Birth counters ──────────────────────────────────
    def test_birth_counters():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)

        # Two anonymous detections: one in zone, one far away
        dets = [
            make_det_box(x=12, z=20),   # 2m ahead — inside zone
            make_det_box(x=50, z=50),   # far away — outside zone
        ]
        info = np.array([
            [0, 0, -1],  # anonymous
            [0, 0, -1],  # anonymous
        ])
        new_ids = tracker.birth(dets, info, [0, 1])

        assert tracker.birth_attempts_anon == 2, \
            f"Expected 2 anon attempts, got {tracker.birth_attempts_anon}"
        assert tracker.birth_suppressed_by_gate == 1, \
            f"Expected 1 suppressed, got {tracker.birth_suppressed_by_gate}"
        assert tracker.births_anon_after_gate == 1, \
            f"Expected 1 after gate, got {tracker.births_anon_after_gate}"
        # 1 identified + 1 new anon = 2
        assert len(tracker.trackers) == 2, \
            f"Expected 2 trackers, got {len(tracker.trackers)}"

    # ── Test 24: Cull after threshold ────────────────────────────
    def test_cull_after_threshold():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        add_anon_track(tracker, x=12, z=20)  # 2m from identified
        assert len(tracker.trackers) == 2

        # 3 consecutive ticks near → culled (cull_consec_ticks=3)
        tracker._cull_anon_near_identified()
        assert len(tracker.trackers) == 2, "Tick 1: not yet culled"
        tracker._cull_anon_near_identified()
        assert len(tracker.trackers) == 2, "Tick 2: not yet culled"
        tracker._cull_anon_near_identified()
        assert len(tracker.trackers) == 1, "Tick 3: should be culled"
        assert tracker.anon_cull_count == 1

    # ── Test 25: Cull resets on move ─────────────────────────────
    def test_cull_resets_on_move():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        anon = add_anon_track(tracker, x=12, z=20)

        # 2 ticks near
        tracker._cull_anon_near_identified()
        tracker._cull_anon_near_identified()
        assert anon.near_identified_ticks == 2

        # Move the anonymous track far away
        anon.kf.x[0] = 50.0  # x = 50
        tracker._cull_anon_near_identified()
        assert anon.near_identified_ticks == 0, \
            f"Counter should reset on move, got {anon.near_identified_ticks}"
        assert len(tracker.trackers) == 2, "Track should survive"

    # ── Test 26: Cull only anonymous ─────────────────────────────
    def test_cull_only_anon():
        tracker = make_tracker()
        add_identified_track(tracker, x=10, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=100)
        add_identified_track(tracker, x=12, z=20, theta=0,
                             l=4.5, w=1.8, h=1.5, carla_id=200)

        for _ in range(5):
            tracker._cull_anon_near_identified()
        assert len(tracker.trackers) == 2, \
            "Both identified tracks should survive"
        assert tracker.anon_cull_count == 0

    # ── Test 27: Full track cycle (end-to-end) ───────────────────
    def test_full_track_cycle():
        tracker = make_tracker()

        # Frame 0: establish identified track via beacon detection
        dets_f0 = np.array([[1.5, 1.8, 4.5, 10.0, 0.7, 20.0, 0.0, 1.0]])
        info_f0 = np.array([[0, 0, 100]])
        tracker.track({'dets': dets_f0, 'info': info_f0}, frame=0)

        # Frame 1: identified re-detection + ghost 7m ahead + far anon
        # Ghost at x=17 is outside NMS range (7 > max(l,w)=4.5) but
        # inside exclusion zone (7 < dup_x_max=8)
        dets_f1 = np.array([
            [1.5, 1.8, 4.5, 10.0, 0.7, 20.0, 0.0, 1.0],  # matches track
            [1.5, 1.8, 4.5, 17.0, 0.7, 20.0, 0.0, 1.0],  # ghost 7m ahead
            [1.5, 1.8, 4.5, 30.0, 0.7, 20.0, 0.0, 1.0],  # 20m away
        ])
        info_f1 = np.array([
            [1, 0, 100],  # identified — matches existing track
            [1, 0, -1],   # anonymous ghost
            [1, 0, -1],   # anonymous far
        ])
        tracker.track({'dets': dets_f1, 'info': info_f1}, frame=1)

        # Ghost at 7m should be suppressed by gate, far at 30m should be born
        assert tracker.birth_suppressed_by_gate >= 1, \
            f"Ghost 7m ahead should be suppressed (suppressed={tracker.birth_suppressed_by_gate})"
        assert tracker.births_anon_after_gate >= 1, \
            f"Far detection should be born (after_gate={tracker.births_anon_after_gate})"

    return [
        ("test_zone_on_top", test_zone_on_top),
        ("test_zone_along_track", test_zone_along_track),
        ("test_zone_cross_track", test_zone_cross_track),
        ("test_zone_angled_heading", test_zone_angled_heading),
        ("test_zone_size_guard", test_zone_size_guard),
        ("test_zone_anon_only", test_zone_anon_only),
        ("test_zone_stale_beacon", test_zone_stale_beacon),
        ("test_birth_counters", test_birth_counters),
        ("test_cull_after_threshold", test_cull_after_threshold),
        ("test_cull_resets_on_move", test_cull_resets_on_move),
        ("test_cull_only_anon", test_cull_only_anon),
        ("test_full_track_cycle", test_full_track_cycle),
    ]


# ═════════════════════════════════════════════════════════════════
#  MAIN
# ═════════════════════════════════════════════════════════════════

SECTIONS = {
    'mac': ('MAC Model', mac_tests),
    'gps': ('GPS Dropout', gps_tests),
    'birth_gate': ('Birth Gate', birth_gate_tests),
}


def main():
    global _VERBOSE

    ap = argparse.ArgumentParser(
        description="Micro-benchmark validation suite (27 tests, no CARLA)")
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='Print PASS results and full tracebacks')
    ap.add_argument('--section', type=str, default=None,
                    choices=list(SECTIONS.keys()),
                    help='Run only one section')
    args = ap.parse_args()
    _VERBOSE = args.verbose

    sections = ([args.section] if args.section
                else list(SECTIONS.keys()))

    print("Micro-Benchmark Validation Suite")
    print("=" * 50)

    for sec_key in sections:
        sec_name, sec_fn = SECTIONS[sec_key]
        print(f"\n  [{sec_name}]")
        tests = sec_fn()
        for name, fn in tests:
            _run(name, fn)

    print("\n" + "=" * 50)
    total = _PASS + _FAIL
    print(f"  {_PASS}/{total} passed, {_FAIL} failed")

    if _FAIL > 0:
        print("\n  RESULT: FAIL")
        sys.exit(1)
    else:
        print("\n  RESULT: ALL PASS")
        sys.exit(0)


if __name__ == '__main__':
    main()
