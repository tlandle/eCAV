# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""Offline replay of the late-fusion edge tracker from a run's logs.

Rebuilds every tracker frame the edge fed to AB3DMOT from the ``[DET]``
lines in ``runner_stdout.txt`` (one line per detection, in jitter-buffer
drain order), pushes them through the real ``AB3DMOT`` with the edge's
``mot_cfg`` and the edge's cross-source NMS, and scores the conflict
actor's track against its live ground truth. The ground truth is
trilaterated per tick from ``per_tick_age.csv``: every row at a tick
carries ``conflict_dist_m`` against the same live GT point, so three or
more rows pin it down (residual is zero on the SenSys runs).

Usage::

    python scripts/diag/replay_edge_tracker.py RUN_DIR [RUN_DIR ...]
        [--d-base M] [--v-static MPS] [--gate-dt S] [--nms-thresh M]
        [--max-age N] [--trace] [--ext L W H]

The gate overrides patch the module constants in AB3DMOT_libs/matching.py
so candidate settings can be scored without editing the library; run the
script at two commits for a before/after of a code change. ``--trace``
prints the per-tick replay next to the run's own ``[TRACKS]`` entries for
the conflict actor.

Scores reported per run (window = first tick the conflict actor moves
faster than 1 m/s until its GT x passes ``--x-end``):

* ``max_gap``   longest run of ticks with no published track within the
  per_tick_age match radius (5 m) of the GT
* ``uncovered`` total such ticks
* ``ids``       distinct track ids published for the conflict actor
* ``pub_v_err`` mean |published speed - GT speed|, published speed being
  what the planner consumes (zero while a track coasts)
* ``kf_v_err``  mean |KF state speed - GT speed| (ignores the coasting zero)
* ``zero_frac`` fraction of covered ticks whose published speed is zero
* ``pos_err``   mean ground distance from the published track to the GT
* ``coasting``  the same errors restricted to covered ticks where the
  published track had no measurement that frame (time_since_update > 0),
  so a longer coast is scored by what the extrapolation got wrong
* ``planner-visible`` coverage counted only for tracks the linear predictor
  consumes (present in 10 of the last 30 published frames, as
  linear_predictor_manager requires for anonymous tracks), and the fraction
  of those ticks whose published speed is below 1 m/s, which the predictor
  turns into a stationary prediction
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, '..', '..'))
if _REPO not in sys.path:
    sys.path.insert(0, _REPO)

from easydict import EasyDict as edict                       # noqa: E402
from AB3DMOT_libs.model import AB3DMOT                       # noqa: E402
import AB3DMOT_libs.matching as matching                     # noqa: E402

DT = 0.05
MATCH_R = 5.0           # per_tick_age_logger match_radius_m
HIST_FRAMES = 30        # edge _track_history depth
MIN_HIST_ANON = 10      # linear_predictor_manager._MIN_HISTORY_ANONYMOUS
MIN_SPEED_MPS = 1.0     # linear_predictor_manager._MIN_KF_SPEED_MPS
_DET_RE = re.compile(r'\[DET\] tick=(\d+) src=(\S+) x=(-?[\d.]+) y=(-?[\d.]+) '
                     r'conf=([\d.]+) cid=(-?\d+)')
_TRK_RE = re.compile(r"\((\d+), (-?\d+), '([-\d.]+)', '\((-?[\d.]+),(-?[\d.]+)\)', (\d+)\)")


# ----------------------------------------------------------------------
# Log parsing
# ----------------------------------------------------------------------
def parse_frames(log_path):
    """Return [(tick, [(src, x, y, conf, cid), ...]), ...] in drain order and
    {tick: [(tid, cid, speed, x, y, n), ...]} from the [TRACKS] lines."""
    frames, tracks = [], {}
    cur_tick, cur = None, []
    last_tick = None
    with open(log_path, errors='replace') as fh:
        for line in fh:
            m = _DET_RE.search(line)
            if m:
                t = int(m.group(1))
                if t != cur_tick:
                    if cur_tick is not None:
                        frames.append((cur_tick, cur))
                    cur_tick, cur = t, []
                cur.append((m.group(2), float(m.group(3)), float(m.group(4)),
                            float(m.group(5)), int(m.group(6))))
                last_tick = t
                continue
            if '[TRACKS]' in line and last_tick is not None:
                tracks[last_tick] = [(int(a), int(b), float(c), float(d), float(e), int(f))
                                     for a, b, c, d, e, f in _TRK_RE.findall(line)]
    if cur_tick is not None:
        frames.append((cur_tick, cur))
    return frames, tracks


def gt_from_csv(csv_path):
    """Trilaterate the conflict actor's live GT per tick. Returns
    {tick: (x, y, speed_mps)} with linear fill for ticks that have fewer
    than three rows."""
    byt = {}
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            if r['obs_x'] == '' or r['conflict_dist_m'] == '':
                continue
            byt.setdefault(int(r['tick']), []).append(
                (float(r['obs_x']), float(r['obs_y']), float(r['conflict_dist_m'])))
    gt = {}
    for t, pts in byt.items():
        if len(pts) < 3:
            continue
        x0, y0, d0 = pts[0]
        A = [[2 * (x - x0), 2 * (y - y0)] for (x, y, _) in pts[1:]]
        b = [(x * x - x0 * x0) + (y * y - y0 * y0) - (d * d - d0 * d0) for (x, y, d) in pts[1:]]
        gx, gy = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)[0]
        if max(abs(math.hypot(x - gx, y - gy) - d) for (x, y, d) in pts) < 0.05:
            gt[t] = (float(gx), float(gy))
    ticks = sorted(gt)
    if not ticks:
        return {}
    filled = {}
    for a, b in zip(ticks, ticks[1:]):
        if b - a > 4 or math.hypot(gt[b][0] - gt[a][0], gt[b][1] - gt[a][1]) > 5.0:
            continue  # conflict-actor identity changed or trace gap
        for t in range(a, b):
            f = (t - a) / (b - a)
            filled[t] = (gt[a][0] + f * (gt[b][0] - gt[a][0]),
                         gt[a][1] + f * (gt[b][1] - gt[a][1]))
    filled[ticks[-1]] = gt[ticks[-1]]
    out = {}
    for t in sorted(filled):
        nxt = filled.get(t + 1)
        prv = filled.get(t - 1)
        if nxt is not None and prv is not None:
            v = math.hypot(nxt[0] - prv[0], nxt[1] - prv[1]) / (2 * DT)
        elif nxt is not None:
            v = math.hypot(nxt[0] - filled[t][0], nxt[1] - filled[t][1]) / DT
        elif prv is not None:
            v = math.hypot(filled[t][0] - prv[0], filled[t][1] - prv[1]) / DT
        else:
            v = 0.0
        out[t] = (filled[t][0], filled[t][1], v)
    return out


# ----------------------------------------------------------------------
# Edge-side detection assembly (mirrors _collect_ab3d_detections +
# _cross_source_nms in edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py)
# ----------------------------------------------------------------------
_CAR_H, _CAR_W, _CAR_L = (1.2, 2.0), (1.6, 2.4), (3.5, 5.5)


def _heading(prev_xy, xy):
    if prev_xy is None:
        return 0.0
    dx, dy = xy[0] - prev_xy[0], xy[1] - prev_xy[1]
    if math.hypot(dx, dy) < 0.02:
        return None
    return math.atan2(dy, dx)


def build_dets(frame_dets, tick, ext, beacon_ext, state):
    """Return the AB3DMOT bundle for one frame. ``ext`` = (l, w, h) used for
    every anonymous sensor detection (the log does not carry extents; the
    edge clamps them to car bounds anyway). Beacon rows use ``beacon_ext``."""
    det_rows, info_rows = [], []
    l, w, h = ext
    for (src, x, y, conf, cid) in frame_dets:
        if cid > 0:
            prev = state.get('beacon_xy', {}).get(cid)
            yaw = _heading(prev, (x, y))
            if yaw is None:
                yaw = state.get('beacon_yaw', {}).get(cid, 0.0)
            state.setdefault('beacon_xy', {})[cid] = (x, y)
            state.setdefault('beacon_yaw', {})[cid] = yaw
            bl, bw, bh = beacon_ext
            det_rows.append([bh, bw, bl, x, 0.5, y, yaw, 1.0])
        else:
            hh = min(max(h, _CAR_H[0]), _CAR_H[1])
            ww = min(max(w, _CAR_W[0]), _CAR_W[1])
            ll = min(max(l, _CAR_L[0]), _CAR_L[1])
            det_rows.append([hh, ww, ll, x, 0.5, y, 0.0, conf])
        state['guid'] = state.get('guid', 0) + 1
        info_rows.append([tick, state['guid'], cid])
    if not det_rows:
        return {'dets': np.zeros((0, 8), np.float32), 'info': np.zeros((0, 3), np.int64)}
    return {'dets': np.asarray(det_rows, np.float32), 'info': np.asarray(info_rows, np.int64)}


def cross_source_nms(dets_dict, cdist_thresh):
    """Verbatim copy of the edge's _cross_source_nms (module import pulls carla)."""
    dets, info = dets_dict['dets'], dets_dict['info']
    if len(dets) == 0:
        return dets_dict
    cids = info[:, 2]
    beacon_mask = cids > 0
    anon_mask = ~beacon_mask
    if anon_mask.sum() <= 1:
        return dets_dict
    anon_dets, anon_info = dets[anon_mask], info[anon_mask]
    centers = anon_dets[:, [3, 5]]
    scores = anon_dets[:, 7]
    order = np.argsort(-scores)
    keep, suppressed = [], np.zeros(len(anon_dets), dtype=bool)
    for i in order:
        if suppressed[i]:
            continue
        keep.append(i)
        suppressed |= np.linalg.norm(centers[i] - centers, axis=1) < cdist_thresh
    anon_keep = np.array(keep)
    return {'dets': np.concatenate([dets[beacon_mask], anon_dets[anon_keep]], axis=0),
            'info': np.concatenate([info[beacon_mask], anon_info[anon_keep]], axis=0)}


def make_tracker(max_age, anchoring=True, dup_x_max=None):
    cfg = edict({'vis': False, 'save_path': None, 'use_3d_iou': True, 'thres': 2.0,
                 'output_dir': None, 'min_hits': 3, 'max_age': max_age, 'ego_com': None,
                 'affi_pro': False, 'dataset': 'KITTI', 'det_name': 'pvrcnn',
                 'anchoring': anchoring})
    cfg.assoc_metric = None
    cfg.assoc_threshold = None
    cfg.assoc_algorithm = None
    if dup_x_max is not None:
        cfg.dup_x_max = dup_x_max   # 0 disables the identity exclusion zone and its cull
    return AB3DMOT(cfg, 'Car')


# ----------------------------------------------------------------------
# Replay + scoring
# ----------------------------------------------------------------------
def replay(frames, gt, args):
    tracker = make_tracker(args.max_age, anchoring=not args.no_anchoring,
                           dup_x_max=args.dup_x_max)
    state = {}
    per_tick = {}   # tick -> list of (tid, x, y, pub_speed, kf_speed, tsu, hits, n_hist)
    from collections import deque
    hist = deque(maxlen=HIST_FRAMES)   # published frames, as _track_history on the edge
    for tick, fdets in frames:
        bundle = build_dets(fdets, tick, tuple(args.ext), tuple(args.beacon_ext), state)
        bundle = cross_source_nms(bundle, args.nms_thresh)
        res, _ = tracker.track(bundle, tick)
        rows = res[0] if res and len(res[0]) else np.zeros((0, 16))
        if len(rows):
            hist.append({int(r[7]) for r in rows})
        live = {trk.id: trk for trk in tracker.trackers}
        out = []
        for row in rows:
            tid = int(row[7])
            x, y = float(row[3]), float(row[5])
            pub_v = math.hypot(float(row[10]), float(row[12])) / DT
            trk = live.get(tid)
            kf_v = 0.0
            tsu, hits = -1, -1
            if trk is not None:
                vel = trk.kf.x.reshape(-1)[7:10]
                kf_v = math.hypot(float(vel[0]), float(vel[2])) / DT
                tsu, hits = trk.time_since_update, trk.hits
            n_hist = sum(1 for s in hist if tid in s)
            out.append((tid, x, y, pub_v, kf_v, tsu, hits, n_hist))
        per_tick.setdefault(tick, []).append(out)
    return per_tick


def score(per_tick, gt, args):
    ticks = sorted(t for t in gt if t in per_tick)
    moving = [t for t in ticks if gt[t][2] > 1.0]
    if not moving:
        return None
    t0 = moving[0]
    window = []
    prev = None
    for t in ticks:
        if t < t0:
            continue
        if gt[t][0] < args.x_end:
            break
        if prev is not None and math.hypot(gt[t][0] - gt[prev][0], gt[t][1] - gt[prev][1]) > 5.0:
            break  # conflict-actor identity changed
        window.append(t)
        prev = t
    t1 = window[-1]
    covered, gaps, cur_gap = [], [], 0
    vis_gaps, vis_cur, vis_n, vis_static = [], 0, 0, 0
    vis_gap_start, vis_cur_start = None, None
    ids = set()
    pub_err, kf_err, pos_err, zeros = [], [], [], 0
    coast = {'n': 0, 'pos': [], 'pub_v': [], 'kf_v': []}
    for t in window:
        gx, gy, gv = gt[t]
        best, vis = None, None
        for out in per_tick[t]:
            for (tid, x, y, pub_v, kf_v, tsu, hits, n_hist) in out:
                d = math.hypot(x - gx, y - gy)
                if d > MATCH_R:
                    continue
                if best is None or d < best[0]:
                    best = (d, tid, pub_v, kf_v, tsu)
                if n_hist >= MIN_HIST_ANON and (vis is None or d < vis[0]):
                    vis = (d, pub_v)
        # planner-visible: the linear predictor consumes the track
        if vis is None:
            if vis_cur == 0:
                vis_cur_start = t
            vis_cur += 1
        else:
            if vis_cur:
                vis_gaps.append(vis_cur)
                if vis_cur >= max(vis_gaps):
                    vis_gap_start = vis_cur_start
                vis_cur = 0
            vis_n += 1
            vis_static += vis[1] < MIN_SPEED_MPS
        if best is None:
            cur_gap += 1
            continue
        if cur_gap:
            gaps.append(cur_gap)
            cur_gap = 0
        covered.append(t)
        ids.add(best[1])
        pub_err.append(abs(best[2] - gv))
        kf_err.append(abs(best[3] - gv))
        pos_err.append(best[0])
        zeros += best[2] == 0.0
        if best[4] > 0:
            coast['n'] += 1
            coast['pos'].append(best[0])
            coast['pub_v'].append(abs(best[2] - gv))
            coast['kf_v'].append(abs(best[3] - gv))
    if cur_gap:
        gaps.append(cur_gap)
    if vis_cur:
        vis_gaps.append(vis_cur)
        if vis_cur >= max(vis_gaps):
            vis_gap_start = vis_cur_start

    def _m(v):
        return float(np.mean(v)) if v else float('nan')

    return {
        'window': (t0, t1), 'n': len(window), 'covered': len(covered),
        'uncovered': len(window) - len(covered), 'max_gap': max(gaps) if gaps else 0,
        'ids': len(ids),
        'pub_v_err': _m(pub_err), 'kf_v_err': _m(kf_err),
        'zero_frac': zeros / len(covered) if covered else float('nan'),
        'pos_err': _m(pos_err), 'pos_err_max': float(np.max(pos_err)) if pos_err else float('nan'),
        'coast_n': coast['n'], 'coast_pos_err': _m(coast['pos']),
        'coast_pos_err_max': float(np.max(coast['pos'])) if coast['pos'] else float('nan'),
        'coast_pub_v_err': _m(coast['pub_v']), 'coast_kf_v_err': _m(coast['kf_v']),
        'vis_uncovered': len(window) - vis_n, 'vis_max_gap': max(vis_gaps) if vis_gaps else 0,
        'vis_gap_start': vis_gap_start if vis_gaps else None,
        'vis_gap_gt_x': gt[vis_gap_start][0] if vis_gaps and vis_gap_start in gt else float('nan'),
        'vis_static_frac': vis_static / vis_n if vis_n else float('nan'),
    }


def trace(per_tick, log_tracks, gt, args):
    print(f"{'tick':>4} {'gt_x':>7} {'gt_v':>5} | replay tid x y pub_v kf_v tsu hits | log tid x y v")
    for t in sorted(gt):
        if t not in per_tick:
            continue
        gx, gy, gv = gt[t]
        if gt[t][2] < 1.0 or gx < args.x_end:
            continue
        rep = [r for out in per_tick[t] for r in out if math.hypot(r[1] - gx, r[2] - gy) <= 8.0]
        lg = [r for r in log_tracks.get(t, []) if math.hypot(r[3] - gx, r[4] - gy) <= 8.0]
        rs = ' '.join(f"{tid}:({x:.1f},{y:.1f}) v={pv:.1f}/{kv:.1f} tsu={tsu} h={h} nh={nh}"
                      for (tid, x, y, pv, kv, tsu, h, nh) in rep) or '-'
        ls = ' '.join(f"{tid}:({x:.1f},{y:.1f}) v={v:.1f}" for (tid, _, v, x, y, _) in lg) or '-'
        print(f"{t:4d} {gx:7.2f} {gv:5.1f} | {rs} | {ls}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='*', help='run directories (runner_stdout.txt + evaluation_output/per_tick_age.csv)')
    ap.add_argument('--runs-file', help='text file with one run directory per line')
    ap.add_argument('--d-base', type=float, default=None, help='matching._D_BASE override (m)')
    ap.add_argument('--v-static', type=float, default=None, help='matching._V_STATIC override (m/s)')
    ap.add_argument('--gate-dt', type=float, default=None, help='matching._DT override (s)')
    ap.add_argument('--nms-thresh', type=float, default=5.0,
                    help='edge cross-source NMS radius (m); 3.0 reproduces runs before 2026-10-14')
    ap.add_argument('--max-age', type=int, default=6)
    ap.add_argument('--no-anchoring', action='store_true')
    ap.add_argument('--dup-x-max', type=float, default=None,
                    help='identity exclusion zone along-track half-length (m); 0 disables it (default 8)')
    ap.add_argument('--ext', type=float, nargs=3, default=[4.5, 2.0, 1.5], metavar=('L', 'W', 'H'),
                    help='extents assumed for anonymous sensor detections')
    ap.add_argument('--beacon-ext', type=float, nargs=3, default=[4.6, 1.95, 1.85], metavar=('L', 'W', 'H'))
    ap.add_argument('--x-end', type=float, default=-100.0, help='GT x at which the scoring window ends')
    ap.add_argument('--trace', action='store_true')
    args = ap.parse_args()

    if args.d_base is not None:
        matching._D_BASE = args.d_base
    if args.v_static is not None:
        matching._V_STATIC = args.v_static
    if args.gate_dt is not None:
        matching._DT = args.gate_dt

    runs = list(args.runs)
    if args.runs_file:
        with open(args.runs_file) as fh:
            runs += [ln.strip() for ln in fh if ln.strip()]
    agg = []
    for run in runs:
        log_path = os.path.join(run, 'runner_stdout.txt')
        csv_path = os.path.join(run, 'evaluation_output', 'per_tick_age.csv')
        if not (os.path.isfile(log_path) and os.path.isfile(csv_path)):
            print(f"{run}: skipped (incomplete run, missing log or per_tick_age.csv)")
            continue
        frames, log_tracks = parse_frames(log_path)
        gt = gt_from_csv(csv_path)
        if not gt or not frames:
            print(f"{run}: skipped (no GT trace or no tracker frames)")
            continue
        per_tick = replay(frames, gt, args)
        if args.trace:
            trace(per_tick, log_tracks, gt, args)
        s = score(per_tick, gt, args)
        if s is None:
            print(f"{run}: no moving conflict actor in GT trace")
            continue
        agg.append(s)
        print(f"{os.path.basename(os.path.dirname(run.rstrip('/')))}/{os.path.basename(run.rstrip('/'))}: "
              f"window={s['window']} n={s['n']} uncovered={s['uncovered']} max_gap={s['max_gap']} "
              f"ids={s['ids']} pub_v_err={s['pub_v_err']:.2f} kf_v_err={s['kf_v_err']:.2f} "
              f"zero_frac={s['zero_frac']:.2f} pos_err={s['pos_err']:.2f} (max {s['pos_err_max']:.2f}) "
              f"| coasting: n={s['coast_n']} pos_err={s['coast_pos_err']:.2f} "
              f"(max {s['coast_pos_err_max']:.2f}) pub_v_err={s['coast_pub_v_err']:.2f} "
              f"kf_v_err={s['coast_kf_v_err']:.2f} "
              f"| planner-visible: uncovered={s['vis_uncovered']} max_gap={s['vis_max_gap']} "
              f"gap_start={s['vis_gap_start']} gap_gt_x={s['vis_gap_gt_x']:.1f} "
              f"static_frac={s['vis_static_frac']:.2f}")
    if len(agg) > 1:
        keys = ['uncovered', 'max_gap', 'ids', 'pub_v_err', 'kf_v_err', 'zero_frac', 'pos_err',
                'coast_n', 'coast_pos_err', 'coast_pub_v_err', 'coast_kf_v_err',
                'vis_uncovered', 'vis_max_gap', 'vis_static_frac']
        print('MEAN over %d runs: ' % len(agg) + ' '.join(
            f"{k}={np.nanmean([a[k] for a in agg]):.2f}" for k in keys)
            + f" max_gap_max={max(a['max_gap'] for a in agg)}"
            + f" coast_pos_err_max={np.nanmax([a['coast_pos_err_max'] for a in agg]):.2f}")


if __name__ == '__main__':
    main()
