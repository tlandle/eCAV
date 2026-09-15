#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Corridor (three-locale) landers for the reduced-corridor evaluation.

Post-hoc log lander for the corridor campaign (openscenario_1_corridor_gt).
Parses .log files only; no simulator, GPU, or sim-code changes. STD columns
reuse khonsu_design_extract's [RUNROW] parser (parse_log); per-crossing rows
come from the corridor runner's [CORRIDORCROSS] lines.

Two CSVs (both re-parse the whole logdir on each run, overwrite):

  frozen1l_corridor_crossings.csv (schema row 55) -- one row per crossing:
    tag, arm, seed, npc, crossing_index, src_locale, dst_locale,
    prepare_tick, crossing_tick, first_dst_track_tick, first_use_tick,
    warm, age_at_use_ms, compliant, bytes, eval_tag, machine

  frozen1l_corridor_rows.csv (schema row 53) -- one row per route (run):
    STD (mode/trigger/eps/ct/tx/by/dist_m/time_s/completed/collided) +
    route_success, n_crossings, warm_frac, compliance_frac,
    bytes_per_crossing, tau_ms, complete_m, eval_tag, machine

Definitions:
  route_success   completed the corridor (ego dist_m >= --complete-m) AND no
                  collision (RUNROW collided == no).
  warm_frac       fraction of the route's crossings with warm == YES
                  (dst held the imported track before first use / crossing).
  age_at_use_ms   (first_use_tick - production_tick) * DT_MS, floored at 0.
                  production_tick = crossing_tick for the final-sync arms
                  (warm / edgewarp / edgewarp_full / repl_final / kf_final)
                  when the crossing tick is known, else prepare_tick. This is
                  the age-at-use of the migrated state (AoI at the planner).
  compliant       age_at_use_ms < --tau-ms and the crossing was actually used
                  (first_use_tick >= 0).
  compliance_frac fraction of the route's crossings that are compliant.

Usage:
  python scripts/khonsu_corridor_land.py <logdir> [--outdir DIR]
         [--tau-ms 500] [--complete-m 350] [--dt-ms 50] [--tag TAG]
         [--machine atlas]
"""
from __future__ import annotations

import argparse
import bisect
import csv
import os
import re
import sys

# khonsu_design_extract (STD [RUNROW] parser) lives beside this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import khonsu_design_extract as kde  # noqa: E402
import khonsu_dup_uploader as _dup  # noqa: E402
try:
    import khonsu_ablation_diag as kad    # resolve_collision_partner
except Exception:  # noqa: BLE001 - degrade to blank if the module is absent
    kad = None

# Arms whose migrated state is refreshed at the crossing (final sync). For these
# the age-at-use production origin is the crossing tick, not the prepare tick.
FINAL_SET = {"warm", "edgewarp", "edgewarp_full", "repl_final", "kf_final"}

CROSSING_COLS = [
    "tag", "arm", "seed", "npc", "crossing_index", "src_locale", "dst_locale",
    "prepare_tick", "crossing_tick", "first_dst_track_tick", "first_use_tick",
    "warm", "age_at_use_ms", "compliant", "bytes", "epoch", "eval_tag",
    "machine",
]
# Matches frozengen_corridor_rows.csv column-for-column (the figure reads these
# names/order). The first 30 are the shared flow schema; the last three are the
# corridor's epoch-fencing result, promoted into the main schema per the figure
# owner (already columns in the multi-ego schema, so this keeps the two blocks
# consistent). frozengen_corridor_rows.csv is regenerated to the same 33.
SUMMARY_COLS = [
    "tag", "mode", "trigger", "band", "refresh", "mirror", "look", "rep",
    "eps", "ct", "collided", "dist_m", "time_s", "completed", "tx", "by",
    "eps_raw", "contact_raw", "machine", "oncoming_speed", "trigger_dist",
    "eval_tag", "final_update", "traffic_n", "crossings", "warm_frac",
    "compliance_frac", "bytes_per_crossing", "dual_fuse_ms", "dual_predict_ms",
    "route_success", "stale_owner_consumed", "both_emit_window_ticks",
    "epoch_fence", "source_bound_reads", "conflicting_ticks",
    "collision_partner_resolved",
    "feature_dup_ticks", "radio_dup_ticks", "cosim_live_ticks",
]

DUAL_RE = re.compile(r"DUALROW\] .*?fuse_ms=(?P<fuse>[\d.]+) predict_ms=(?P<pred>[\d.]+)")


def _dual_ms(text):
    """Mean dual-authority fuse/predict ms across DUALROW markers."""
    fu, pr = [], []
    for m in DUAL_RE.finditer(text):
        fu.append(float(m.group("fuse"))); pr.append(float(m.group("pred")))
    f = round(sum(fu) / len(fu), 2) if fu else ""
    p = round(sum(pr) / len(pr), 2) if pr else ""
    return f, p


def _eps_raw_contact(text):
    """Raw collision episodes (dedup >1s) + contact ticks from per-tick warnings."""
    ts = []
    for ln in text.splitlines():
        if "WARNING" in ln and "Collision" in ln and "Eval" not in ln:
            m = re.search(r"(\d{2}):(\d{2}):(\d{2}),(\d{3})", ln)
            if m:
                h, mn, s, ms = map(int, m.groups())
                ts.append(h * 3600 + mn * 60 + s + ms / 1000.0)
    eps, last = 0, None
    for t in ts:
        if last is None or t - last > 1.0:
            eps += 1
        last = t
    return eps, len(ts)


def _launchenv(text, key, default):
    """Read an uppercase env var (e.g. ONCOMING_SPEED=12) from the LAUNCHENV line."""
    m = re.search(rf"\b{key}=([0-9.]+)", text)
    return m.group(1) if m else default


def _locale_ranges(text):
    """x-range per locale parsed from the config JSON in the log (not hardcoded,
    so the split survives a geometry change). Overlap bands are handled by the
    caller checking membership in both src and dst."""
    import json
    ranges = {}
    for m in re.finditer(r'"id":\s*"(locale_\d+)",\s*"polygon":\s*(\[\[.*?\]\])', text):
        try:
            xs = [p[0] for p in json.loads(m.group(2))]
            ranges[m.group(1)] = (min(xs), max(xs))
        except Exception:  # noqa: BLE001
            pass
    return ranges


def _ego_x_map(text):
    """tick -> ego x from EGO-DBG, for reconstructing the consuming CAV's locale."""
    d = {}
    for m in re.finditer(r"EGO-DBG\] tick=(\d+) pos=\(([0-9.-]+),", text):
        d[int(m.group(1))] = float(m.group(2))
    return d

# Tag forms accepted:
#   co_<arm>_n<traffic>_r<rep>   (campaign; arm may carry underscores)
#   co_khonsu_ef0_r<rep>         (EPOCH_FENCE=0 off-arm; no traffic token)
#   corr_<arm>_s<seed> / <arm>_s<seed>  (earlier smoke/synthetic form)
# rep and seed both land in the `seed` column (the per-route replicate index).
TAG_RE = re.compile(
    r"^(?:(?:co|corr)_)?(?P<arm>[a-z][a-z0-9_]*?)"
    r"(?:_n(?P<traffic>\d+))?_(?:r|s)(?P<seed>\d+)$")
CROSS_RE = re.compile(
    r"\[CORRIDORCROSS\] npc=(?P<npc>-?\d+) "
    r"crossing_index=(?P<ci>\d+) src=(?P<src>\S+) dst=(?P<dst>\S+) "
    r"prepare_tick=(?P<pt>-?\d+) crossing_tick=(?P<cx>-?\d+) "
    r"first_dst_track_tick=(?P<fd>-?\d+) first_use_tick=(?P<fu>-?\d+) "
    r"warm=(?P<warm>\S+) bytes=(?P<by>\d+)(?: epoch=(?P<epoch>-?\d+))?")
# [CONSUMEDEPOCH] ego=%s actor=%s tick=%s consumed_epoch=%d fenced=%d
CONSUMED_RE = re.compile(
    r"\[CONSUMEDEPOCH\] ego=(?P<ego>\S+) actor=(?P<actor>-?\d+) "
    r"tick=(?P<tick>-?\d+) consumed_epoch=(?P<ce>-?\d+) fenced=(?P<fenced>\d+)")
# [PUBGATE_SRC] cid=%s tick=%d dropped_after_commit=1
PUBSRC_RE = re.compile(
    r"\[PUBGATE_SRC\] cid=(?P<cid>-?\d+) tick=(?P<tick>-?\d+) "
    r"dropped_after_commit=1")


def _read(path):
    return open(path, encoding="utf-8", errors="ignore").read()


def _mean_frac(flags):
    """Mean of a list of bools as a rounded fraction; '' when empty."""
    if not flags:
        return ""
    return round(sum(1 for f in flags if f) / len(flags), 3)


def _age_and_compliance(mode, prepare, crossing, first_use, dt_ms, tau_ms):
    """(age_at_use_ms, compliant_bool) for one crossing; age '' if never used."""
    if first_use < 0:
        return "", False
    production = crossing if (mode in FINAL_SET and crossing >= 0) else prepare
    if production < 0:
        return "", False
    age_ticks = first_use - production
    if age_ticks < 0:
        age_ticks = 0
    age_ms = round(age_ticks * dt_ms, 1)
    return age_ms, (age_ms < tau_ms)


def run(args):
    crossings = []
    summaries = []
    dt_ms = float(args.dt_ms)
    tau_ms = float(args.tau_ms)
    complete_m = float(args.complete_m)

    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        stem = name[:-4]
        tm = TAG_RE.match(stem)
        if not tm:
            continue
        path = os.path.join(args.logdir, name)
        core = kde.parse_log(path)
        if core is None:
            print(f"skip (no RUNROW, incomplete): {name}", file=sys.stderr)
            continue
        if core == "INVALID":
            print(f"skip (INVALID degenerate startup): {name}", file=sys.stderr)
            continue
        arm = tm.group("arm")
        seed = tm.group("seed")
        traffic = tm.group("traffic") or ""
        mode = core.get("mode", "")
        text = _read(path)
        # collision_partner_resolved: the real CARLA actor the ego physically
        # contacted (geometric nearest non-ego actor at collision onset), blank if
        # no collision or no real actor near. A blank on a collided route flags a
        # phantom-caused stop with no real partner; a real id is a genuine
        # oncoming/blocker impact. Same resolver the headline diag used.
        collision_partner = ""
        if kad is not None:
            try:
                _dk = kad.parse_log(path)
                collision_partner = kad.resolve_collision_partner(
                    _dk, kad.identify_ego_cid(_dk))
            except Exception:  # noqa: BLE001
                collision_partner = ""
        # EPOCH_FENCE off when the arm carries ef0 or the launch env set it 0.
        epoch_fence = "0" if ("ef0" in arm
                              or re.search(r"EPOCH_FENCE=0", text)) else "1"

        warm_flags = []
        comp_flags = []
        n_cross = 0
        committed = {}   # actor carla_id -> [(crossing_tick, epoch_int, src, dst), ...]
        for m in CROSS_RE.finditer(text):
            n_cross += 1
            npc = int(m.group("npc"))
            ci = int(m.group("ci"))
            src = m.group("src")
            dst = m.group("dst")
            pt = int(m.group("pt"))
            cx = int(m.group("cx"))
            fd = int(m.group("fd"))
            fu = int(m.group("fu"))
            warm = m.group("warm")
            by = int(m.group("by"))
            epoch = m.group("epoch")
            epoch = int(epoch) if epoch is not None else ""
            if isinstance(epoch, int):
                committed.setdefault(npc, []).append((cx, epoch, src, dst))
            age_ms, compliant = _age_and_compliance(
                mode, pt, cx, fu, dt_ms, tau_ms)
            warm_flags.append(warm == "YES")
            comp_flags.append(compliant)
            crossings.append({
                "tag": stem, "arm": arm, "seed": seed, "npc": npc,
                "crossing_index": ci, "src_locale": src, "dst_locale": dst,
                "prepare_tick": pt, "crossing_tick": cx,
                "first_dst_track_tick": fd, "first_use_tick": fu,
                "warm": warm, "age_at_use_ms": age_ms,
                "compliant": "YES" if compliant else "no", "bytes": by,
                "epoch": epoch,
                "eval_tag": args.tag, "machine": args.machine,
            })

        # stale_owner_consumed: a [CONSUMEDEPOCH] read under a LOWER epoch than the
        # actor's committed owner epoch, counted ONLY when the consuming CAV is
        # bound to the DESTINATION locale of the transfer (its position is in dst
        # and not in src). A source-bound read (CAV still in the source locale)
        # from the source's own local track is design-permitted (the source serves
        # it until its tracker drops the road user), so it is reported separately
        # as source_bound_reads and excluded from the violation count. The
        # violation count must be 0 under EPOCH_FENCE=1; the ef0 arm is the
        # negative control that drives it non-zero.
        _loc = _locale_ranges(text)
        _egox = _ego_x_map(text)
        _egt = sorted(_egox)

        def _ego_x_at(t):
            i = bisect.bisect_right(_egt, t) - 1
            return _egox[_egt[i]] if i >= 0 else None

        def _inloc(x, loc):
            r = _loc.get(loc)
            return r is not None and r[0] <= x <= r[1]

        stale_owner = 0
        source_bound = 0
        for cm in CONSUMED_RE.finditer(text):
            actor = int(cm.group("actor"))
            ctick = int(cm.group("tick"))
            ce = int(cm.group("ce"))
            evec = [(cx, ep, s, d) for (cx, ep, s, d) in committed.get(actor, [])
                    if 0 <= cx <= ctick]
            if not evec:
                continue
            cur = max(ep for (cx, ep, s, d) in evec)
            if not (ce >= 0 and ce < cur):
                continue
            _cx, _ep, _src, _dst = [t for t in evec if t[1] == cur][-1]
            _x = _ego_x_at(ctick)
            if _x is None:
                continue
            if _inloc(_x, _dst) and not _inloc(_x, _src):
                stale_owner += 1        # destination-bound read: a real violation
            elif _inloc(_x, _src):
                source_bound += 1       # source-bound read: design-permitted
        # both_emit_window_ticks: per actor, ticks in which BOTH the source
        # (still publishing until its [PUBGATE_SRC] drop) and the destination
        # (publishing from its shadow-clear = first crossing_tick) could emit.
        # source_drop - dest_first_publish, floored at 0; reported as the max
        # over actors. With EPOCH_FENCE=1 the source drops at commit so this
        # collapses to ~0; with no drop (ef0) it is the observed consume span.
        pubsrc = {}
        for pm in PUBSRC_RE.finditer(text):
            cid = int(pm.group("cid"))
            pubsrc.setdefault(cid, []).append(int(pm.group("tick")))
        last_consume = {}
        for cm in CONSUMED_RE.finditer(text):
            a = int(cm.group("actor"))
            last_consume[a] = max(last_consume.get(a, -1), int(cm.group("tick")))
        # both_emit_window_ticks is the RAW overlap window (denominator): ticks in
        # which both source and destination could emit. conflicting_ticks is the
        # risk-bearing SUBSET (numerator): ticks within an overlap window in which
        # the consuming CAV is DESTINATION-bound (in dst, not in src) and so could
        # actually receive both the source's prior-epoch stream and the dest's.
        # The paper reports their ratio. A fenced arm has a non-zero raw window but
        # a zero conflicting subset (the CAV is source-bound through the overlap,
        # which stale_owner_consumed=0 independently confirms); the ef0 arm drives
        # the subset non-zero. Do NOT narrow the raw window: it is the denominator.
        _cx_sd = {cx: (s, d) for v in committed.values() for (cx, ep, s, d) in v}
        both_emit = 0
        _max_win = None   # (dp, D, src, dst) of the single largest overlap window
        for actor, evec in committed.items():
            cxs = sorted(cx for (cx, ep, s, d) in evec if cx >= 0)
            if not cxs:
                continue
            windows = []
            drops = pubsrc.get(actor)
            if drops:
                # Pair each source drop with the commit it followed: the latest
                # crossing_tick <= drop. Window = drop - that dest publish.
                for D in drops:
                    dp = max((cx for cx in cxs if cx <= D), default=None)
                    if dp is not None:
                        windows.append((dp, D))
            else:
                # source never fenced (ef0): open window from the last commit's
                # dest publish to the last observed consume of the actor.
                lc = last_consume.get(actor)
                if lc is not None:
                    dp = max((cx for cx in cxs if cx <= lc), default=cxs[0])
                    windows.append((dp, lc))
            for (dp, D) in windows:
                w = max(0, D - dp)
                if w > both_emit:
                    both_emit = w
                    _s_w, _d_w = _cx_sd.get(dp, (None, None))
                    _max_win = (dp, D, _s_w, _d_w)
        # conflicting_ticks is the risk-bearing SUBSET of the raw window and must be
        # <= both_emit, so it is counted within the SAME (largest) overlap window
        # both_emit reports, not summed across every actor's window (which massively
        # over-counts). Ticks in that window where the CAV is destination-bound.
        conflicting = 0
        if _max_win is not None and _max_win[3] is not None:
            _dp, _D, _s_w, _d_w = _max_win
            for t in range(_dp, _D + 1):
                x = _ego_x_at(t)
                if x is not None and _inloc(x, _d_w) and not _inloc(x, _s_w):
                    conflicting += 1

        # Route summary.
        dist_m = core.get("dist_m", "")
        collided = core.get("collided", "")
        try:
            _completed_route = dist_m != "" and float(dist_m) >= complete_m
        except (TypeError, ValueError):
            _completed_route = False
        route_success = "YES" if (_completed_route and collided == "no") else "no"
        try:
            _by_total = int(core.get("by", "0"))
        except (TypeError, ValueError):
            _by_total = 0
        bytes_per_crossing = round(_by_total / n_cross) if n_cross > 0 else ""
        # stale_owner_consumed and both_emit_window_ticks are ONLY defined when an
        # ownership transfer occurs. On a non-migrating arm (cold) there is no
        # second owner and no epoch reads, so 0 would read as "as well fenced as
        # fencing" and the both_emit else-branch emits a spurious open window
        # (e.g. 1298) that a reader takes as the worst dual-emission figure. Emit
        # EMPTY (not applicable) for those arms. Do NOT backfill with zeros.
        # Migration is detected by CAMIGRATED, the actual track-migration event.
        # CONSUMEDEPOCH is NOT the signal: it is logged for every track epoch read
        # including local (non-migrated) tracks, so cold shows thousands of them
        # (15411 in cold_r1) with zero migration. PUBGATE_SRC is likewise absent
        # on cold. CAMIGRATED is 0 on cold and >0 on the migrating arms.
        _migrates = "CAMIGRATED" in text
        # frozengen columns not on RUNROW: raw collisions, dual timings, env.
        eps_raw, contact_raw = _eps_raw_contact(text)
        dual_fuse_ms, dual_predict_ms = _dual_ms(text)
        final_update = "builtin" if mode in FINAL_SET else "none"
        summaries.append({
            "tag": stem, "mode": mode, "trigger": core.get("trigger", ""),
            "band": core.get("band", ""), "refresh": core.get("refresh", ""),
            "mirror": core.get("mirror", ""), "look": core.get("look", ""),
            "rep": seed,
            "eps": core.get("eps", ""), "ct": core.get("ct", ""),
            "collided": collided, "dist_m": dist_m,
            "time_s": core.get("time_s", ""), "completed": core.get("completed", ""),
            "tx": core.get("tx", ""), "by": core.get("by", ""),
            "eps_raw": eps_raw, "contact_raw": contact_raw,
            "machine": args.machine,
            "oncoming_speed": _launchenv(text, "ONCOMING_SPEED", "12"),
            "trigger_dist": _launchenv(text, "TRIGGER_DIST", "300"),
            "eval_tag": args.tag, "final_update": final_update,
            "traffic_n": traffic, "crossings": n_cross,
            "warm_frac": _mean_frac(warm_flags),
            "compliance_frac": _mean_frac(comp_flags),
            "bytes_per_crossing": bytes_per_crossing,
            "dual_fuse_ms": dual_fuse_ms, "dual_predict_ms": dual_predict_ms,
            "route_success": route_success,
            "stale_owner_consumed": stale_owner if _migrates else "",
            "both_emit_window_ticks": both_emit if _migrates else "",
            "epoch_fence": epoch_fence,
            "source_bound_reads": source_bound if _migrates else "",
            "conflicting_ticks": conflicting if _migrates else "",
            "collision_partner_resolved": collision_partner,
            **_dup.counts(text),   # duplicate-uploader recurrence guard
        })

    os.makedirs(args.outdir, exist_ok=True)
    cross_out = os.path.join(args.outdir, "frozen1l_corridor_crossings.csv")
    rows_out = os.path.join(args.outdir, "frozen1l_corridor_rows.csv")
    _write(crossings, CROSSING_COLS, cross_out)
    _write(summaries, SUMMARY_COLS, rows_out)
    if not summaries:
        print("no corridor logs matched (none landed yet)", file=sys.stderr)
    return summaries


def _write(rows, cols, out):
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"{len(rows)} rows -> {out}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir")
    ap.add_argument("--outdir",
                    default="docs/kb/data/relay_eval_2026_08")
    ap.add_argument("--tau-ms", dest="tau_ms", default="500",
                    help="age-at-use compliance threshold in ms (window class)")
    ap.add_argument("--complete-m", dest="complete_m", default="350",
                    help="ego distance for a completed corridor route (m)")
    ap.add_argument("--dt-ms", dest="dt_ms", default="50",
                    help="world tick in ms (fixed_delta_seconds * 1000)")
    ap.add_argument("--tag", default="khonsu-eval-corridor")
    ap.add_argument("--machine", default="atlas")
    args = ap.parse_args()
    run(args)


if __name__ == "__main__":
    main()
