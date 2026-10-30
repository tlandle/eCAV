#!/usr/bin/env python3
"""
Recompute sweep metrics from existing simulation_metrics.json files.

Changes from the in-evaluator computation:
  1. Focal-ego only: metrics computed for vehicle index 0 (hero), not aggregated.
  2. S_prog replaced with S_clear: pass unless stuck or collision (not speed-based).
  3. S_op = S_coll & S_ghost & S_fp & S_clear
  4. Per-ego breakdown available for multi-ego analysis.
"""

import json
import math
import os
import sys
import numpy as np
from collections import defaultdict

# Post-hoc labelling knobs. The defaults are the paper's rules; the
# sensitivity sweep (scripts/sensys_analysis/label_sensitivity.py, reviewer
# check A, Oct 29) sets them from the calling script.
#
# A brake's published observation is stale by its age, so matching it to the
# live ground truth at the trigger tick measures the actor's travel during
# the age (7 m at 500 ms, 14 m at 1,000 ms, 20 m at 1,500 ms at 12.5 m/s),
# not the pipeline's error. The paper's rule therefore matches the
# observation to the conflict actor's ground truth at the observation's own
# tick (CONFLICT_MATCH_AT_SOURCE_TICK) within a fixed radius. The growing
# trigger-tick radius below is the fallback for a run without a per-tick log.
CONFLICT_MATCH_AT_SOURCE_TICK = True   # match at the observation's source tick
CONFLICT_SOURCE_MATCH_RADIUS_M = 3.0   # fixed radius of that match
CONFLICT_MATCH_RADIUS_M = 6.0         # fallback: trigger-tick match radius to the conflict actor ...
CONFLICT_MATCH_GROWTH_MPS = 12.5      # ... growing per second of configured delay ...
CONFLICT_MATCH_GROWTH_FROM_MS = 200.0  # ... beyond this delay
COUNT_LOCAL_TRACK_BRAKES = False      # True scores local-track brakes in S_ghost and S_fp
LIVE_HAZARD_REJUDGE = True            # re-judge conflict-actor other_fp brakes on the live state


def load_sweep(base_dir, exclude_ego_counts=None):
    """Load all simulation_metrics.json under base_dir into a list of dicts."""
    exclude_ego_counts = exclude_ego_counts or set()
    runs = []
    for root, dirs, files in os.walk(base_dir):
        if "simulation_metrics.json" not in files:
            continue
        if "evaluation_output" in root:
            continue
        path = os.path.join(root, "simulation_metrics.json")
        try:
            with open(path) as f:
                d = json.load(f)
        except Exception:
            continue
        ego_count = d.get("config_ego_count", 1)
        if ego_count in exclude_ego_counts:
            continue
        d["_path"] = path
        runs.append(d)
    return runs


def focal_ego(d):
    """Return the vehicle dict for the focal ego (lowest actor ID = hero)."""
    vehicles = d.get("vehicles", {})
    if not vehicles:
        return None
    # Hero is the lowest-numbered actor ID (first spawned)
    focal_vid = min(vehicles.keys(), key=lambda k: int(k))
    return vehicles[focal_vid], focal_vid


def _is_local_track(attr):
    """True when the brake was triggered by the CAV's own on-board forecast.

    The planner forecasts its local perception 0.5 s ahead (10 steps) and the
    edge publishes 3 s (60 steps), so the attribution's prediction horizon
    tells the two apart. A brake on a local track never crossed the network:
    it is neither a self-ghost nor a false positive of the shared state, and
    S_ghost and S_fp count shared tracks only (Oct 28, Tyler). Local-track
    brakes are reported in focal_local_episodes."""
    h = attr.get("prediction_horizon_s")
    return h is not None and float(h) < 1.0


def _count_episodes(brake_attributions, gap=2):
    """
    Count brake episodes by GT class.

    An episode = contiguous ticks (gap <= `gap`) with the same track_id
    and same gt_brake_class.

    FP ticks that are concurrent with TP ticks (within `gap`) are filtered
    out: if the ego is already braking for a real hazard, a co-occurring FP
    does not cause additional unwanted behavior.

    Returns (ghost_episodes, fp_episodes, tp_episodes).
    """
    by_class = defaultdict(list)
    for a in brake_attributions:
        cls = a.get("gt_brake_class", "unknown")
        tick = a.get("trigger_tick")
        track = a.get("trigger_track_id")
        if tick is not None:
            by_class[cls].append((tick, track))

    # Build set of TP-active ticks (expanded by gap for episode overlap)
    tp_ticks_raw = set(t for t, _ in by_class.get("true_positive", []))
    tp_ticks_expanded = set()
    for t in tp_ticks_raw:
        for dt in range(-gap, gap + 1):
            tp_ticks_expanded.add(t + dt)

    # Filter FP events: remove ticks concurrent with TP
    fp_filtered = [(t, tr) for t, tr in by_class.get("other_fp", [])
                   if t not in tp_ticks_expanded]

    def count_eps(events):
        if not events:
            return 0
        events.sort()
        eps = 1
        prev_tick, prev_track = events[0]
        for tick, track in events[1:]:
            if tick > prev_tick + gap or track != prev_track:
                eps += 1
            prev_tick, prev_track = tick, track
        return eps

    ghost = count_eps(by_class.get("self_ghost", []))
    fp = count_eps(fp_filtered)
    tp = count_eps(by_class.get("true_positive", []))
    return ghost, fp, tp


def _conflict_actor_id(d):
    """The run's conflict actor id from its per-tick age log (first non-empty
    conflict_actor_id), or None when the run has no log. d is the metrics
    dict; its _run_dir key locates the log."""
    import csv as _csv
    run_dir = d.get("_run_dir") if isinstance(d, dict) else d
    if not run_dir:
        return None
    for p in (os.path.join(run_dir, "per_tick_age.csv"),
              os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")):
        if os.path.exists(p):
            try:
                with open(p) as fh:
                    for r in _csv.DictReader(fh):
                        v = r.get("conflict_actor_id")
                        if v not in (None, "", "nan", "-1"):
                            return int(float(v))
            except Exception:
                return None
    return None



def _conflict_live_states(d):
    """Live state of the run's conflict actor per tick from the per-tick log:
    {tick: {'x','y','vx','vy','speed'}}, velocity by central finite difference
    over the logged ticks (50 ms). Empty when the run has no log."""
    import csv as _csv
    run_dir = d.get("_run_dir") if isinstance(d, dict) else d
    if not run_dir:
        return {}
    pos = {}
    for p in (os.path.join(run_dir, "per_tick_age.csv"),
              os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")):
        if not os.path.exists(p):
            continue
        try:
            with open(p) as fh:
                for r in _csv.DictReader(fh):
                    if r.get("conflict_gt_x") in (None, "", "nan"):
                        continue
                    try:
                        pos.setdefault(int(float(r["tick"])), (float(r["conflict_gt_x"]), float(r["conflict_gt_y"])))
                    except (TypeError, ValueError):
                        continue
        except Exception:
            return {}
        break
    ticks = sorted(pos)
    out = {}
    for i, t in enumerate(ticks):
        a = ticks[max(0, i - 1)]; b = ticks[min(len(ticks) - 1, i + 1)]
        span = (b - a) * 0.05
        vx = (pos[b][0] - pos[a][0]) / span if span > 0 else 0.0
        vy = (pos[b][1] - pos[a][1]) / span if span > 0 else 0.0
        out[t] = {"x": pos[t][0], "y": pos[t][1], "vx": vx, "vy": vy, "speed": (vx * vx + vy * vy) ** 0.5}
    return out


def _conflict_state_at(d, tick, max_gap=3):
    """The conflict actor's live state at tick, or at the nearest logged tick
    within max_gap ticks (the per-tick log has a row only on ticks with a
    consumed object). None when the run has no log or no row near tick."""
    if tick is None:
        return None
    live = _conflict_live_states(d)
    if not live:
        return None
    try:
        t = int(tick)
    except (TypeError, ValueError):
        return None
    for dt in range(0, max_gap + 1):
        for cand in (t - dt, t + dt):
            if cand in live:
                return live[cand]
    return None


def _gt_hazard(actor, ego_xy, ego_vx, ego_vy, speed_thresh=2.0, dca_thresh=8.0, horizon=5.0):
    """The run-time labeller's conflict predicate (edge_manager_base._is_gt_hazard):
    closest approach under relative constant velocity within the horizon."""
    if actor["speed"] < speed_thresh:
        return False, float("inf"), -1.0
    dx = ego_xy[0] - actor["x"]; dy = ego_xy[1] - actor["y"]
    rvx = actor["vx"] - ego_vx; rvy = actor["vy"] - ego_vy
    rv2 = rvx * rvx + rvy * rvy
    if rv2 < 1e-6:
        return False, float("inf"), -1.0
    t_ca = (dx * rvx + dy * rvy) / rv2
    if t_ca < 0 or t_ca > horizon:
        return False, float("inf"), float(t_ca)
    ax = actor["x"] + actor["vx"] * t_ca; ay = actor["y"] + actor["vy"] * t_ca
    ex = ego_xy[0] + ego_vx * t_ca; ey = ego_xy[1] + ego_vy * t_ca
    dca = ((ax - ex) ** 2 + (ay - ey) ** 2) ** 0.5
    return dca < dca_thresh, float(dca), float(t_ca)


def _ego_headings(d):
    """Unit heading of the focal vehicle per tick from the run's per-tick log
    (positions two ticks apart), keyed by tick; empty when the log is absent."""
    out = {}
    rd = d.get("_run_dir")
    p = os.path.join(rd, "runner_stdout.txt") if rd else None
    if not p or not os.path.exists(p):
        return out
    try:
        import re as _re
        pos = {}
        pat = _re.compile(r"\[LOC\] v=(\d+) t=(\d+) gnss=\([^)]*\) gt=\((-?[\d.]+),(-?[\d.]+)\)")
        # The [LOC] lines stop after 100 ticks; the focal vehicle's own beacon
        # rows in the edge's [DET] log (GNSS, every tick it beacons) carry the
        # pose on. With one managed vehicle every beacon is the focal's; with
        # more, the beacon nearest the previous pose is taken.
        bpat = _re.compile(r"\[DET\] tick=(\d+) src=beacon x=(-?[\d.]+) y=(-?[\d.]+) ")
        focal = None
        beacons = {}
        with open(p, errors="ignore") as fh:
            for line in fh:
                m = pat.search(line)
                if m:
                    v, t = int(m.group(1)), int(m.group(2))
                    if focal is None:
                        focal = v
                    if v != focal or t in pos:
                        continue
                    pos[t] = (float(m.group(3)), float(m.group(4)))
                    continue
                b = bpat.search(line)
                if b:
                    beacons.setdefault(int(b.group(1)), []).append((float(b.group(2)), float(b.group(3))))
        for t in sorted(beacons):
            if t in pos:
                continue
            cands = beacons[t]
            prev = pos.get(t - 1, pos.get(t - 2))
            if len(cands) == 1:
                pos[t] = cands[0]
            elif prev is not None:
                pos[t] = min(cands, key=lambda c: (c[0] - prev[0]) ** 2 + (c[1] - prev[1]) ** 2)
        for t in pos:
            a, b = pos.get(t - 2, pos.get(t - 1)), pos.get(t + 2, pos.get(t + 1, pos.get(t)))
            if a is None or b is None:
                continue
            hx, hy = b[0] - a[0], b[1] - a[1]
            n = math.hypot(hx, hy)
            if n > 0.05:
                out[t] = (hx / n, hy / n)
    except Exception:
        return {}
    return out


def compute_run_metrics(d):
    """Compute corrected per-run metrics using focal ego only."""
    result = {}

    # Config
    result["mgr"] = d.get("config_manager_type", "?")
    result["anch"] = "on" if d.get("config_anchoring") else "off"
    result["lat"] = d.get("config_latency_ms", 0)
    result["ego_count"] = d.get("config_ego_count", 1)

    foc = focal_ego(d)
    if foc is None:
        return None
    fv, focal_vid = foc
    result["focal_vid"] = focal_vid

    # --- Safety (focal ego only) ---
    focal_safety = fv.get("safety", {})
    focal_coll = focal_safety.get("collision_count", 0)
    result["focal_collisions"] = focal_coll
    result["focal_stuck"] = focal_safety.get("stuck", False)
    result["focal_offroad"] = focal_safety.get("offroad", False)

    # S_coll: focal ego had no collisions
    result["s_coll"] = 1 if focal_coll == 0 else 0

    # --- Brake classifications (focal ego only, EPISODE-based) ---
    # Post-hoc reclassification: if GT matched the ego actor, it is a
    # self-ghost regardless of distance.  The runtime 3m gate leaked
    # high-latency ego-ghosts into other_fp.
    ba = fv.get("brake_attributions", [])
    ego_actor_id = int(focal_vid)
    reclassified = 0
    # A self-ghost is the ego's own echo: a track in the ego's lane corridor
    # (under 1.75 m lateral to its heading) that trails or sits on it. The
    # nearest-GT-actor rule alone mislabels two things: a parked vehicle
    # beside the lane whose face is nearer the ego than its own centre (the
    # junction-entry truck, 2.1 m lateral), and the echo at high delay, which
    # trails the ego by more than the match radius and matches another actor.
    # The ego heading at the trigger tick comes from the per-tick log when the
    # run has one; otherwise +y (the scenario_3 approach).
    _hdg = _ego_headings(d)
    # The run-time labeller tags a track's provenance 'ego' whenever the
    # nearest actor in the edge's range-limited ground-truth snapshot is a
    # managed CAV; a crossing car first seen beyond that range has no entry,
    # so its track is tagged 'ego' and its brakes are classed self_ghost
    # although the labeller's own full-world match found the car within a
    # couple of metres (contract 88 km/h 0 ms, Oct 17: five of ten brakes).
    # Post hoc, a self_ghost label with a non-ego match inside the match
    # radius is relabelled by the match: the run's conflict actor gives
    # true_positive, any other actor other_fp.
    _conflict_id = _conflict_actor_id(d)
    # The other managed CAVs of a multi-vehicle run cross the junction on
    # their own routes; the run-time labeller tags a track near any managed
    # CAV 'ego' (the same range-limited snapshot defect), so the focal's
    # brakes for a crossing CAV are classed self_ghost while the labeller's
    # full-world match names that CAV (N=4 smoke, Oct 18: eleven brakes on
    # cav3 at 0.1 to 1.8 m). A match to another managed CAV is a brake for a
    # real vehicle: true_positive, as the labeller's own 'cross_ego' class
    # says when the match lies beyond its 3 m gate. At N=1 the managed set is
    # the focal alone, so the N=1 stages are unchanged.
    _managed = set()
    for _k in (d.get("vehicles") or {}).keys():
        try:
            _managed.add(int(_k))
        except (TypeError, ValueError):
            pass
    for attr in ba:
        _raw0 = attr.get("gt_brake_class")
        _mid = attr.get("gt_matched_actor_id")
        _mdist = attr.get("gt_match_dist_m")
        # The run-time labeller's closest-approach test is undefined while
        # the focal stands (gt_dca_m inf, gt_t_ca_s -1), and it then classes
        # a brake other_fp whatever the match. A focal holding at a lane edge
        # for another managed CAV passing within 3 m is braking for a real
        # vehicle (oracle N=8, Oct 18: 7 to 96 such brakes per seed on the
        # westbound follower; the V2X arms' brakes on the same vehicle arrive
        # as raw self_ghost and are relabelled true_positive by the cross-ego
        # rule below, so without this the oracle alone is scored false).
        # Same 3 m radius as the cross-ego rule; the conflict actor is left
        # out so the physics boundary (stale state of the conflict car) is
        # untouched.
        _dca = attr.get("gt_dca_m")
        _stopped = _dca is None or _dca == "inf" or (isinstance(_dca, float) and _dca == float("inf"))
        if (_raw0 == "other_fp" and _stopped and _mid is not None and int(_mid) != ego_actor_id
                and int(_mid) in _managed and _mdist is not None and float(_mdist) < 3.0):
            attr["gt_brake_class"] = "true_positive"; attr["gt_provenance"] = "held_for_cav"
            reclassified += 1
            _raw0 = "true_positive"
        # Match radius: 6 m when the match is the run's conflict actor (its
        # published state carries the pipeline's position error plus up to two
        # edge frames of lag at the crossing: 3.3 to 4 m at 63 km/h and 200 ms,
        # Oct 17), 3 m for any other actor. The conflict actor's stale offset
        # grows with the configured delay (12 m/s x 0.55 s = 7.2 to 7.9 m at
        # 500 ms; j4 SCP 70 contract 500 ms, Oct 26: all ten brakes on the
        # crossing car fell through to other_fp), so above 200 ms the radius
        # grows at 12.5 m/s with the delay beyond 200 ms.
        _is_conf = (_conflict_id is not None and _mid is not None and int(_mid) == int(_conflict_id))
        # Source-tick match: the published observation is compared with the
        # conflict actor's ground truth at the tick it was observed, so the
        # radius needs no delay term. A match within the radius names the
        # conflict actor whatever the run-time labeller matched at the
        # trigger tick.
        if CONFLICT_MATCH_AT_SOURCE_TICK and _conflict_id is not None and _raw0 == "self_ghost":
            _src = _conflict_state_at(d, attr.get("source_tick"))
            _ox, _oy = attr.get("obs_x"), attr.get("obs_y")
            if _src is not None and None not in (_ox, _oy):
                _dsrc = math.hypot(float(_ox) - _src["x"], float(_oy) - _src["y"])
                if _dsrc < CONFLICT_SOURCE_MATCH_RADIUS_M:
                    _is_conf, _mid, _mdist = True, int(_conflict_id), _dsrc
        _is_cav = (_mid is not None and int(_mid) in _managed and int(_mid) != ego_actor_id)
        try:
            _lat_ms = float(d.get("config_latency_ms") or 0.0)
        except (TypeError, ValueError):
            _lat_ms = 0.0
        _conf_radius = CONFLICT_MATCH_RADIUS_M + CONFLICT_MATCH_GROWTH_MPS / 1000.0 * max(
            0.0, _lat_ms - CONFLICT_MATCH_GROWTH_FROM_MS)
        if (_raw0 == "self_ghost" and _mid is not None and int(_mid) != ego_actor_id
                and _mdist is not None and float(_mdist) < (_conf_radius if _is_conf else 3.0)):
            if _is_conf:
                attr["gt_brake_class"] = "true_positive"; attr["gt_provenance"] = "conflict_actor_match"
            elif _is_cav:
                attr["gt_brake_class"] = "true_positive"; attr["gt_provenance"] = "cross_ego_match"
            else:
                attr["gt_brake_class"] = "other_fp"; attr["gt_provenance"] = "non_ego_match"
            reclassified += 1
        ox, oy, ex, ey = attr.get("obs_x"), attr.get("obs_y"), attr.get("ego_x"), attr.get("ego_y")
        lateral = None
        if None not in (ox, oy, ex, ey):
            hx, hy = _hdg.get(attr.get("trigger_tick"), (0.0, 1.0))
            dx, dy = float(ox) - float(ex), float(oy) - float(ey)
            lateral = abs(-dx * hy + dy * hx)
            along = dx * hx + dy * hy
        in_corridor = lateral is not None and lateral < 1.75 and along < 3.0
        raw = attr.get("gt_brake_class")
        looks_like_echo = (attr.get("gt_matched_actor_id") == ego_actor_id or raw == "self_ghost")
        if looks_like_echo and in_corridor and raw != "self_ghost":
            attr["gt_brake_class"] = "self_ghost"; attr["gt_provenance"] = "self"; reclassified += 1
        elif looks_like_echo and not in_corridor and raw == "self_ghost":
            attr["gt_brake_class"] = "other_fp"; attr["gt_provenance"] = "beside_lane"; reclassified += 1

    # The run-time labeller between 2026-10-29 12:43 and the live-state fix
    # (labelfix3) matched the actor at the observation tick and judged the
    # hazard on that stale state against the live ego, so a brake for the
    # crossing car consumed 700 ms late read a closest approach of 10 m
    # (7 m live) and was classed other_fp (reference 70 km/h, 700 and 1,000
    # ms: 10/10 runs). Judge such brakes again on the conflict actor's live
    # state from the per-tick log (position at the trigger tick, velocity by
    # finite difference) with the labeller's own predicate.
    _conf_live = _conflict_live_states(d) if (LIVE_HAZARD_REJUDGE and _conflict_id is not None) else {}
    if _conf_live:
        for attr in ba:
            if attr.get("gt_brake_class") != "other_fp" or attr.get("gt_provenance") != "npc_non_hazard":
                continue
            _mid = attr.get("gt_matched_actor_id")
            if _mid is None or int(_mid) != int(_conflict_id):
                continue
            st = _conf_live.get(attr.get("trigger_tick"))
            ex, ey, ev = attr.get("ego_x"), attr.get("ego_y"), attr.get("ego_speed_mps")
            if st is None or None in (ex, ey, ev):
                continue
            hx, hy = _hdg.get(attr.get("trigger_tick"), (0.0, 1.0))
            ok, dca, t_ca = _gt_hazard(st, (float(ex), float(ey)), float(ev) * hx, float(ev) * hy)
            if ok:
                attr["gt_brake_class"] = "true_positive"; attr["gt_provenance"] = "conflict_live_hazard"
                attr["gt_dca_m"] = dca; attr["gt_t_ca_s"] = t_ca; reclassified += 1

    # Shared tracks only: a brake on the CAV's own local forecast is counted
    # apart (focal_local_*) and enters neither S_ghost nor S_fp.
    local = [a for a in ba if _is_local_track(a) and not COUNT_LOCAL_TRACK_BRAKES]
    shared = [a for a in ba if COUNT_LOCAL_TRACK_BRAKES or not _is_local_track(a)]

    # Recount tick totals after reclassification
    ghost_ticks = sum(1 for a in shared if a.get("gt_brake_class") == "self_ghost")
    fp_ticks = sum(1 for a in shared if a.get("gt_brake_class") == "other_fp")
    tp_ticks = sum(1 for a in shared if a.get("gt_brake_class") == "true_positive")
    result["focal_ghost_ticks"] = ghost_ticks
    result["focal_fp_ticks"] = fp_ticks
    result["focal_tp_ticks"] = tp_ticks
    result["focal_local_ticks"] = len(local)
    result["focal_total_brake_ticks"] = fv.get("total_brake_events", 0)

    # Episode counting: consecutive ticks with same track_id = 1 episode
    # Gap of >2 ticks or different track_id starts a new episode
    ghost_eps, fp_eps, tp_eps = _count_episodes(shared)
    local_ghost_eps, local_fp_eps, local_tp_eps = _count_episodes(local)
    result["focal_local_episodes"] = local_ghost_eps + local_fp_eps
    result["focal_local_tp_episodes"] = local_tp_eps
    result["focal_ghost_episodes"] = ghost_eps
    result["focal_fp_episodes"] = fp_eps
    result["focal_tp_episodes"] = tp_eps
    result["focal_total_episodes"] = ghost_eps + fp_eps + tp_eps

    # S_ghost: focal ego had no self-ghost EPISODES
    result["s_ghost"] = 1 if ghost_eps == 0 else 0

    # S_fp: focal ego had no other-FP EPISODES
    result["s_fp"] = 1 if fp_eps == 0 else 0

    # --- S_clear: task completion (replaces S_prog) ---
    # Pass unless focal ego is stuck or had a collision
    result["s_clear"] = 1 if (not result["focal_stuck"] and focal_coll == 0) else 0

    # --- S_prog: the focal completes the maneuver within the step budget ---
    # Runs on the capped scenarios (config_max_step, or any run that carries
    # the per-tick-log key) exit at the destination before the cap; reaching
    # the cap means the focal never completed. The April runs ended on
    # arrival with no cap, so there the clause is the evaluator's 60 percent
    # of target speed.
    edges_ = list(d.get("edges", {}).values())
    frames_ = edges_[0].get("total_frames") if edges_ else None
    cap = d.get("config_max_step")
    if cap is None and "config_per_tick_age_log" in d:
        cap = 300
    if cap is not None and frames_ is not None:
        result["s_prog"] = 1 if int(frames_) < int(cap) else 0
    else:
        tgt_kmh = d.get("target_speed_kmh")
        avg_mps = fv.get("avg_speed_mps")
        if tgt_kmh is None or avg_mps is None:
            result["s_prog"] = 1
        else:
            result["s_prog"] = 1 if float(avg_mps) >= 0.6 * float(tgt_kmh) / 3.6 else 0

    # --- S_op: operational success ---
    result["s_op"] = (result["s_coll"] & result["s_ghost"] & result["s_fp"]
                      & result["s_clear"] & result["s_prog"])

    # --- Continuous metrics ---
    result["focal_avg_speed_mps"] = fv.get("avg_speed_mps", 0)

    # Edge metrics (scenario-level, not per-ego)
    for eid, edata in d.get("edges", {}).items():
        result["edge_fps"] = edata.get("frames_per_second", 0)
        result["edge_timing_avg_ms"] = edata.get("timing_avg_ms", 0)
        result["ego_uniqueness_violations"] = edata.get("ego_uniqueness_total_violations", 0)
        result["ego_ghost_tracks"] = edata.get("ego_uniqueness_total_ego_ghost_tracks", 0)
        result["detection_precision"] = edata.get("detection_precision", 0)
        result["detection_recall"] = edata.get("detection_recall", 0)
        result["tracking_mota"] = edata.get("tracking_avg_mota", 0)
        result["prediction_ade_1s"] = edata.get("prediction_ade_1s_m", 0)
        result["prediction_fde"] = edata.get("prediction_fde_m", 0)
        break  # Only one edge

    # --- Mixed traffic: outcomes per actor class and duplicate-track rate ---
    # connected = CAVs that are members of the edge; non_connected = CAVs the
    # mixed-traffic knob left out (no beacon, no uplink, no downlink).
    for cls in ("connected", "non_connected"):
        for k in ("n", "collided", "stuck", "ghost", "fp"):
            result[f"{cls}_{k}"] = 0
    for v in d.get("vehicles", {}).values():
        cls = "connected" if v.get("connected", True) else "non_connected"
        vs = v.get("safety", {})
        g, fp, _ = _count_episodes([a for a in v.get("brake_attributions", []) if not _is_local_track(a)])
        result[f"{cls}_n"] += 1
        result[f"{cls}_collided"] += 1 if vs.get("collision_count", 0) > 0 else 0
        result[f"{cls}_stuck"] += 1 if vs.get("stuck", False) else 0
        result[f"{cls}_ghost"] += 1 if g > 0 else 0
        result[f"{cls}_fp"] += 1 if fp > 0 else 0
    for edata in d.get("edges", {}).values():
        dup = edata.get("duplicate_tracks", {})
        for cls in ("connected", "non_connected"):
            result[f"dup_rate_{cls}"] = dup.get(cls, {}).get("duplicate_rate", 0.0)
        break

    # AoI from brake attributions (focal ego)
    aoi_list = []
    for a in fv.get("brake_attributions", []):
        src = a.get("source_tick")
        trg = a.get("trigger_tick")
        if src is not None and trg is not None:
            aoi_ticks = trg - src
            aoi_list.append(aoi_ticks * 50)  # Convert to ms (dt=0.05s)
    if aoi_list:
        result["aoi_mean_ms"] = float(np.mean(aoi_list))
        result["aoi_p95_ms"] = float(np.percentile(aoi_list, 95))
        result["aoi_p99_ms"] = float(np.percentile(aoi_list, 99))
    else:
        result["aoi_mean_ms"] = None
        result["aoi_p95_ms"] = None
        result["aoi_p99_ms"] = None

    # Provenance breakdown (focal ego)
    prov_counts = defaultdict(int)
    for a in fv.get("brake_attributions", []):
        prov = a.get("gt_provenance", "unknown")
        prov_counts[prov] += 1
    result["prov_self"] = prov_counts.get("self", 0)
    result["prov_cross_ego"] = prov_counts.get("cross_ego", 0)
    result["prov_npc_hazard"] = prov_counts.get("npc_hazard", 0)
    result["prov_parked"] = prov_counts.get("parked", 0)
    result["prov_npc_non_hazard"] = prov_counts.get("npc_non_hazard", 0)
    result["prov_unknown"] = prov_counts.get("unknown", 0)

    return result


def print_table(groups, title=""):
    """Print a formatted summary table."""
    if title:
        print(f"\n=== {title} ===")
    header = (f"{'mgr':<12} {'anch':<5} {'lat':>4} {'ego':>3} | "
              f"{'N':>3} {'P(op)':>6} {'P(col)':>6} {'P(gho)':>6} {'P(fp)':>6} {'P(clr)':>6} | "
              f"{'gho_e':>5} {'fp_e':>5} {'tp_e':>5} {'tot_e':>5} | "
              f"{'gho_t':>5} {'fp_t':>5} {'tp_t':>5} | "
              f"{'uniq_v':>6} {'spd':>5}")
    print(header)
    print("-" * len(header))

    for key in sorted(groups.keys()):
        runs = groups[key]
        n = len(runs)
        mgr, anch, lat, ego = key

        def avg(field):
            vals = [r[field] for r in runs if r.get(field) is not None]
            return np.mean(vals) if vals else 0

        print(f"{mgr:<12} {anch:<5} {lat:>4.0f} {ego:>3} | "
              f"{n:>3} "
              f"{avg('s_op'):>6.2f} "
              f"{avg('s_coll'):>6.2f} "
              f"{avg('s_ghost'):>6.2f} "
              f"{avg('s_fp'):>6.2f} "
              f"{avg('s_clear'):>6.2f} | "
              f"{avg('focal_ghost_episodes'):>5.1f} "
              f"{avg('focal_fp_episodes'):>5.1f} "
              f"{avg('focal_tp_episodes'):>5.1f} "
              f"{avg('focal_total_episodes'):>5.1f} | "
              f"{avg('focal_ghost_ticks'):>5.1f} "
              f"{avg('focal_fp_ticks'):>5.1f} "
              f"{avg('focal_tp_ticks'):>5.1f} | "
              f"{avg('ego_uniqueness_violations'):>6.0f} "
              f"{avg('focal_avg_speed_mps'):>5.1f}")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <sweep_dir> [--exclude-ego N,N,...]")
        sys.exit(1)

    base_dir = sys.argv[1]
    exclude = set()
    for i, arg in enumerate(sys.argv):
        if arg == "--exclude-ego" and i + 1 < len(sys.argv):
            exclude = {int(x) for x in sys.argv[i + 1].split(",")}

    print(f"Loading from: {base_dir}")
    print(f"Excluding ego counts: {exclude or 'none'}")

    raw_runs = load_sweep(base_dir, exclude_ego_counts=exclude)
    print(f"Loaded {len(raw_runs)} raw run files")

    # Compute corrected metrics
    runs = []
    for d in raw_runs:
        r = compute_run_metrics(d)
        if r is not None:
            runs.append(r)
    print(f"Computed metrics for {len(runs)} runs")

    # Group by config
    groups = defaultdict(list)
    for r in runs:
        key = (r["mgr"], r["anch"], r["lat"], r["ego_count"])
        groups[key].append(r)

    # Print by ego count
    for ego_n in sorted({k[3] for k in groups}):
        ego_groups = {k: v for k, v in groups.items() if k[3] == ego_n}
        print_table(ego_groups, title=f"{ego_n}-EGO (focal ego only)")

    # First-failure decomposition for 1-ego
    print("\n=== FIRST-FAILURE DECOMPOSITION (1-ego, focal) ===")
    print(f"{'mgr':<12} {'anch':<5} {'lat':>4} | {'N':>3} "
          f"{'OK':>5} {'COLL':>5} {'GHOST':>5} {'FP':>5} {'STUCK':>5}")
    print("-" * 70)
    for key in sorted(groups.keys()):
        mgr, anch, lat, ego = key
        if ego != 1:
            continue
        runs_k = groups[key]
        n = len(runs_k)
        ok = sum(1 for r in runs_k if r["s_op"] == 1)
        coll = sum(1 for r in runs_k if r["s_coll"] == 0)
        # First failure priority: collision > ghost > fp
        ghost_first = sum(1 for r in runs_k
                         if r["s_coll"] == 1 and r["s_ghost"] == 0)
        fp_first = sum(1 for r in runs_k
                      if r["s_coll"] == 1 and r["s_ghost"] == 1 and r["s_fp"] == 0)
        stuck = sum(1 for r in runs_k
                   if r["s_coll"] == 1 and r["s_ghost"] == 1 and r["s_fp"] == 1
                   and r["s_clear"] == 0)
        print(f"{mgr:<12} {anch:<5} {lat:>4.0f} | {n:>3} "
              f"{ok/n:>5.0%} {coll/n:>5.0%} {ghost_first/n:>5.0%} "
              f"{fp_first/n:>5.0%} {stuck/n:>5.0%}")


if __name__ == "__main__":
    main()
