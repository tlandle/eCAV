#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""T25 speed-sweep lander for the NSDI "regions" figure.

POST-HOC log extractor. Emits one row per T25 run log (t25_{mode}_v{spd}_s{N}),
matching the exact 32-column schema of mock_data/frozengen_t25_rows.csv.

The first 23 columns are the STD design-sweep columns produced by
khonsu_design_extract.py (imported and reused verbatim), with three T25
differences: oncoming_speed is parsed per row from the tag v{spd} (not fixed
12); seed and rep both come from the tag _s{N}; machine is the host label
(default "cetus"). completed = dist >= 90 m, collided = episodes > 0, and the
same INVALID rule (no GT INJECT DBG / no EGO-DBG / startup transient) excludes
degenerate logs.

The nine T25 columns reuse the gating-oncoming resolution, sustained-launch and
collision-partner logic from khonsu_ablation_diag.py (imported):

    contact_actor       nearest non-ego actor at collision onset
                        (resolve_collision_partner); blank if no collision.
    crossing_tick       gating oncoming's entry into locale_0 (x >= 240).
                        Migration arms use the HANDOFFROW crossing_tick for the
                        gating npc; cold uses the first GT INJECT DBG frame with
                        the gating oncoming's world x >= 240.
    do_ov_tick          sustained overtake launch (launch_tick, first run of >=4
                        consecutive do_ov=True EGO-DBG samples).
    conflict_tick       first GT INJECT DBG frame with the gating oncoming's
                        world x >= 278 (the truck/overtake conflict x).
    t_avail_s           (conflict_tick - crossing_tick) * 0.05.
    time_to_observe_s   (observe_tick - crossing_tick) * 0.05, where observe_tick
                        is the HANDOFFROW first_use_tick (migration) or the
                        destination's own first-forecast tick of the gating
                        oncoming (cold, own_first_forecast_tick). Blank if the
                        gating oncoming is never observed at the destination.
    frames_at_commit    history-frame depth of the destination's track of the
                        gating oncoming at commit. HANDOFFROW carries no frame
                        field, so migration arms read EXPORT-DBG memo=<n> (the
                        migrated track's memory depth) for the gating cid nearest
                        first_dst_track_tick; cold counts the destination's own
                        forecast frames of the gating cid up to its first own
                        forecast tick. Blank if unavailable.
    motion              always "constant" for the T25 speed sweep.

Frame/tick counter check (documented, done once at build): GT INJECT DBG frame
and EGO-DBG / HANDOFFROW tick are the SAME counter. Cross-check on a migration
arm: HANDOFFROW crossing_tick for the gating npc (257) vs the GT INJECT frame
where that npc first crosses x >= 240 (256) agree within one tick (the crossing
detector fires one tick after the raw position crosses). No conversion applied.

Usage: python3 scripts/khonsu_t25_land.py <logdir> [-o out.csv] [--machine cetus]
"""
from __future__ import annotations

import argparse
import csv
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import khonsu_ablation_diag as kad   # noqa: E402  gating / launch / collision logic
import khonsu_design_extract as kde  # noqa: E402  STD 23-column extractor

SIM_DT = 0.05
DEST_X = 240.0        # locale_0 entry
CONFLICT_X = 278.0    # truck / overtake conflict x

HEADER = [
    "tag", "mode", "trigger", "band", "refresh", "mirror", "look", "rep",
    "eps", "ct", "collided", "dist_m", "time_s", "completed", "tx", "by",
    "eps_raw", "contact_raw", "machine", "oncoming_speed", "trigger_dist",
    "eval_tag", "final_update", "seed", "contact_actor", "crossing_tick",
    "do_ov_tick", "conflict_tick", "t_avail_s", "time_to_observe_s",
    "frames_at_commit", "motion",
    "shift_x_m", "boundary_x", "t_avail_decision_s",   # boundary sweep: b - 250, b, crossing -> CAV decision arrival
]

_TAG = re.compile(r"^t25_(?P<mode>[a-z]+)_v(?P<spd>\d+)_s(?P<seed>\d+)$")
# boundary-moved sweep (atlas_t25_boundary.sh): bnd_<mode>_b<b>_s<seed> (constant motion),
# bndm_<mode>_b<b>_s<seed> (maneuvering). The crossing edge per run is b - 10 (the
# 10 m hysteresis band), read from the [LAUNCHENV] boundary= field.
_BND = re.compile(r"^bndm?_(?P<mode>[a-z]+)_b(?P<b>\d+)_s(?P<seed>\d+)$")
_XSHIFT = re.compile(r"_x(?P<x>-?\d+)_s\d+$")   # landed spawn-shift tags (t25b_<mode>_x<shift>_s<seed>)
_GT = re.compile(
    r"\[GT INJECT DBG\] frame=(\d+) aid=(\d+) type=\S+ "
    r"world=\(([\-0-9.]+),([\-0-9.]+)\)")
_EXPORT = re.compile(r"\[EXPORT-DBG\] cid=(\d+) memo=(\d+)")
_TICK = re.compile(r"tick=(\d+)")


def gt_first_frame_ge_x(text, aid, xthr):
    """First GT INJECT DBG frame where actor aid has world x >= xthr ("" if none)."""
    for m in _GT.finditer(text):
        if int(m.group(2)) == aid and float(m.group(3)) >= xthr:
            return int(m.group(1))
    return ""


def last_conflict_crosser(text, oncoming):
    """Gating-oncoming fallback: the oncoming id that reaches the conflict x last
    (largest first-frame with x >= CONFLICT_X). This is the vehicle the ego waits
    out, used when the launch-time gating resolver finds a cleared gap."""
    firsts = {}
    for aid in oncoming:
        f = gt_first_frame_ge_x(text, aid, CONFLICT_X)
        if f != "":
            firsts[aid] = f
    if firsts:
        return max(firsts, key=firsts.get)
    return max(oncoming) if oncoming else ""


def resolve_gating(d, text, oncoming, lt):
    """Gating oncoming id: launch-time resolver (gating_track) when it names a
    track; otherwise the last oncoming to cross the conflict (late/clear-gap
    launches). Both reduce to the same vehicle in the T25 overtake scenario."""
    gid, _note = kad.gating_track(d, oncoming, lt)
    if gid != "":
        return gid
    return last_conflict_crosser(text, oncoming)


def eps_raw_contact_raw(text):
    """Raw (uncapped) collision episodes and contact ticks from the per-tick
    collision-sensor warnings, deduplicated at >1 s gaps. Copied from
    khonsu_design_extract.main so a row is self-contained."""
    ts = []
    for ln in text.splitlines():
        if "WARNING" in ln and "Collision" in ln and "Eval" not in ln:
            m = re.search(r"(\d{2}):(\d{2}):(\d{2}),(\d{3})", ln)
            if m:
                h, mn, sec, ms = map(int, m.groups())
                ts.append(h * 3600 + mn * 60 + sec + ms / 1000.0)
    eps_raw, last = 0, None
    for t in ts:
        if last is None or t - last > 1.0:
            eps_raw += 1
        last = t
    return eps_raw, len(ts)


def memo_at_commit(text, cid, commit):
    """EXPORT-DBG memo (migrated track memory depth) for cid, taken from the last
    export at or just before commit (context tick from EGO-DBG/EVAL/AGEROW).
    "" if the cid is never exported."""
    cur = None
    best = ""
    for raw in text.splitlines():
        s = kad.strip_ansi(raw)
        if ("[EGO-DBG]" in s) or ("[EVAL] tick=" in s) or ("[AGEROW]" in s):
            m = _TICK.search(s)
            if m:
                cur = int(m.group(1))
        me = _EXPORT.search(s)
        if me and int(me.group(1)) == cid:
            if commit is None or (cur is not None and cur <= commit + 2):
                best = int(me.group(2))
    return best


def build_row(path, machine):
    std = kde.parse_log(path)
    if std is None or std == "INVALID":
        return std  # None -> no RUNROW; "INVALID" -> degenerate; both skipped
    stem = os.path.splitext(os.path.basename(path))[0]
    tm = _TAG.match(stem)
    tb = _BND.match(stem)

    text = open(path, encoding="utf-8", errors="ignore").read()
    d = kad.parse_log(path)

    row = dict(std)
    row["tag"] = stem
    row["machine"] = machine
    row["motion"] = "constant"
    row["oncoming_speed"] = tm.group("spd") if tm else \
        d["launchenv"].get("ONCOMING_SPEED", "")
    row["seed"] = tm.group("seed") if tm else (tb.group("seed") if tb else "")
    row["rep"] = row["seed"]
    row["trigger_dist"] = d["launchenv"].get("TRIGGER_DIST", "300")
    row["eval_tag"] = d.get("eval_tag", "")
    eps_raw, contact_raw = eps_raw_contact_raw(text)
    row["eps_raw"] = eps_raw
    row["contact_raw"] = contact_raw

    mode = row.get("mode", "")
    # final_update mirrors khonsu_design_extract: builtin for warm/edgewarp on
    # every tag past the original freeze-1/1a; commit_full when refresh=full.
    if mode in ("warm", "edgewarp") and row.get("eval_tag") not in (
            "freeze-1", "freeze-1a", "khonsu-eval-freeze-1",
            "khonsu-eval-freeze-1a"):
        row["final_update"] = "builtin"
    elif row.get("refresh") == "full":
        row["final_update"] = "commit_full"
    else:
        row["final_update"] = "none"

    # oracle arm = warm content (MIGRATION_MODE=warm) with the ground-truth
    # crossing (TRIGGER_MODE=oracle) trigger, the T20 upper bound. RUNROW logs
    # mode=warm, so relabel from the tag to keep it a distinct arm in the figure
    # (fig_regions iterates oracle, warm, reactive, kf, cold). final_update is
    # already resolved from the underlying warm above; it migrates the full
    # latent so it reads crossing_tick from HANDOFFROW like warm.
    if (tm and tm.group("mode") == "oracle") or (tb and tb.group("mode") == "oracle"):
        row["mode"] = "oracle"
        mode = "oracle"

    # --- T25-specific ---
    oncoming, _amb = kad.identify_oncoming(d)
    lt = kad.launch_tick(d)
    row["do_ov_tick"] = lt if lt is not None else ""

    gid = resolve_gating(d, text, oncoming, lt)
    row["contact_actor"] = kad.resolve_collision_partner(d, kad.identify_ego_cid(d))

    # Pair the crossing and conflict ticks to the CONTACT actor (the vehicle the
    # ego actually conflicted with), falling back to the gating oncoming when there
    # is no collision. Both ticks must describe ONE actor's crossing-to-conflict
    # span; keying crossing off the gating oncoming while the ego hit a different
    # stream member misplaced the t_avail x axis (visible as the 6 m/s outliers,
    # where the slow stream held several oncoming vehicles in the window at once).
    # gid still drives the migration-depth metrics below (the prepared track).
    _ca = str(row["contact_actor"]).strip()
    pair_id = int(_ca) if _ca.isdigit() else gid

    hf = d["handoff"].get(pair_id, {}) if pair_id != "" else {}
    migrated = mode in ("warm", "reactive", "kf", "oracle") and "crossing_tick" in hf

    # crossing_tick: HANDOFFROW for migration arms, GT x>=240 for cold.
    crossing = ""
    if migrated:
        try:
            crossing = int(hf["crossing_tick"])
        except (TypeError, ValueError):
            crossing = ""
    _bnd = d["launchenv"].get("boundary")
    dest_x = (float(_bnd) - 10.0) if _bnd else DEST_X   # destination edge per run (boundary sweep) else the landed 240
    if crossing == "" and pair_id != "":
        crossing = gt_first_frame_ge_x(text, pair_id, dest_x)
    row["crossing_tick"] = crossing

    # conflict_tick: first GT frame with the CONTACT actor's x >= 278 (same actor
    # as the crossing above).
    conflict = gt_first_frame_ge_x(text, pair_id, CONFLICT_X) if pair_id != "" else ""
    row["conflict_tick"] = conflict

    row["t_avail_s"] = (round((conflict - crossing) * SIM_DT, 2)
                        if crossing != "" and conflict != "" else "")
    # boundary sweep columns (paper side, 2026-09-13): the figure's axis is
    # t_avail_s (oncoming crossing -> conflict, moves only with the boundary);
    # t_avail_decision_s (crossing -> the focal CAV's decision arrival) is emitted
    # beside it and not plotted. shift_x_m = b - 250 for boundary runs; for the
    # landed spawn-shift tags it stays the spawn shift.
    row["boundary_x"] = float(_bnd) if _bnd else 250.0
    _xs = _XSHIFT.search(stem)
    row["shift_x_m"] = (int(_xs.group("x")) if _xs else round(row["boundary_x"] - 250.0, 1))
    row["t_avail_decision_s"] = (round((lt - crossing) * SIM_DT, 2)
                                 if crossing != "" and lt is not None else "")
    _acc = d["launchenv"].get("ONCOMING_ACCEL", "0")
    row["motion"] = "maneuvering" if (_acc not in ("", "0", "0.0")) else "constant"

    # own first-forecast tick of the gating oncoming at the destination.
    dest_fc = kad.dest_forecasts(d, gid) if gid != "" else []
    own_ff = kad.own_first_forecast_tick(dest_fc)

    # time_to_observe_s: HANDOFFROW first_use (migration) or own first forecast
    # (cold), relative to crossing.
    observe = ""
    if migrated and "first_use_tick" in hf:
        try:
            observe = int(hf["first_use_tick"])
        except (TypeError, ValueError):
            observe = ""
    elif not migrated:
        observe = own_ff
    row["time_to_observe_s"] = (round((observe - crossing) * SIM_DT, 2)
                                if observe != "" and crossing != "" else "")

    # frames_at_commit: migration = EXPORT-DBG memo (migrated depth) for the
    # gating cid near first_dst_track_tick; cold = destination own-forecast
    # frames of the gating cid up to its first own forecast.
    if migrated:
        try:
            commit = int(hf.get("first_dst_track_tick"))
        except (TypeError, ValueError):
            commit = None
        row["frames_at_commit"] = memo_at_commit(text, gid, commit) if gid != "" else ""
    else:
        row["frames_at_commit"] = (
            len({r["tick"] for r, _s in dest_fc if r["tick"] <= own_ff})
            if own_ff != "" else "")

    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--machine", default="cetus", help="host label for these rows")
    args = ap.parse_args()

    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        path = os.path.join(args.logdir, name)
        try:
            row = build_row(path, args.machine)
        except Exception as e:  # noqa: BLE001 - one bad log must not abort the sweep
            print(f"ERROR {name}: {type(e).__name__}: {e}", file=sys.stderr)
            continue
        if row is None:
            print(f"no RUNROW: {name}", file=sys.stderr)
            continue
        if row == "INVALID":
            print(f"INVALID degenerate run EXCLUDED: {name}", file=sys.stderr)
            continue
        rows.append(row)

    out = open(args.out, "w", newline="") if args.out else sys.stdout
    w = csv.DictWriter(out, fieldnames=HEADER, extrasaction="ignore")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    if args.out:
        out.close()
        print(f"{len(rows)} rows -> {args.out}")


if __name__ == "__main__":
    main()
