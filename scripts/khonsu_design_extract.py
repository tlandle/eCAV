#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Extract per-run rows from khonsu_design_sweep logs.

Each run log carries a [RUNROW] line (mode/trigger/knobs, deduped collision
episodes, contact ticks, transfer count and bytes) and the ego evaluation
dict (distance_traveled_m, time_alive_s). completed follows the flow-table
convention: the ego reached the route end (>= 90 m) within the scenario
deadline. Binary collided = episodes > 0, per the committee's metric
directive.

Usage: python scripts/khonsu_design_extract.py <logdir> [-o out.csv]
"""
from __future__ import annotations

import argparse
import ast
import csv
import os
import re
import sys

RUNROW = re.compile(
    r"\[RUNROW\] mode=(?P<mode>\S+) trigger=(?P<trigger>\S+) "
    r"band_w=(?P<band>[\d.]+) refresh=(?P<refresh>\S+) "
    r"mirror=(?P<mirror>[\d.]+) lookahead=(?P<look>[\d.]+) "
    r"episodes=(?P<eps>\d+) contact_ticks=(?P<ct>\d+) "
    r"transfers=(?P<tx>\d+) bytes=(?P<by>\d+)")
EGO = re.compile(r"\{'actor_id':.*?\}")
COMPLETE_DIST_M = 90.0


def parse_log(path):
    text = open(path, encoding="utf-8", errors="ignore").read()
    m = RUNROW.search(text)
    if not m:
        return None
    row = {k: m.group(k) for k in (
        "mode", "trigger", "band", "refresh", "mirror", "look",
        "eps", "ct", "tx", "by")}
    dist = time_s = None
    for em in EGO.finditer(text):
        try:
            d = ast.literal_eval(em.group(0))
        except (ValueError, SyntaxError):
            continue
        dist = d.get("distance_traveled_m")
        time_s = d.get("time_alive_s")
    row["dist_m"] = f"{dist:.1f}" if dist is not None else ""
    row["time_s"] = f"{time_s:.1f}" if time_s is not None else ""
    row["completed"] = (
        "YES" if dist is not None and dist >= COMPLETE_DIST_M else "no")
    row["collided"] = "YES" if int(row["eps"]) > 0 else "no"
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logdir")
    ap.add_argument("-o", "--out", default=None)
    args = ap.parse_args()

    rows = []
    for name in sorted(os.listdir(args.logdir)):
        if not name.endswith(".log"):
            continue
        row = parse_log(os.path.join(args.logdir, name))
        if row is None:
            print(f"no RUNROW: {name}", file=sys.stderr)
            continue
        tag = name[:-4]
        row["tag"] = tag
        row["rep"] = tag.rsplit("_r", 1)[-1] if "_r" in tag else ""
        rows.append(row)

    cols = ["tag", "mode", "trigger", "band", "refresh", "mirror", "look",
            "rep", "eps", "ct", "collided", "dist_m", "time_s", "completed",
            "tx", "by"]
    out = open(args.out, "w", newline="") if args.out else sys.stdout
    w = csv.DictWriter(out, fieldnames=cols)
    w.writeheader()
    for r in rows:
        w.writerow(r)
    if args.out:
        out.close()
        print(f"{len(rows)} rows -> {args.out}")


if __name__ == "__main__":
    main()
