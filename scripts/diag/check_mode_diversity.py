#!/usr/bin/env python3
"""Verify-cell gate: MTR mode DIVERSITY (paper side, 2026-09-13).

The one-cell verify before the freeze-1p campaign gated on forward motion and
never on mode diversity; the agent-set isolation (cap=1) trades one for the
other: every MODESROW under isolation carried six identical endpoints at a
uniform 1/64 score. Direction and diversity are different properties. This
gate fails a log unless >= 95 percent of its MODESROW rows have >= 2 distinct
mode endpoints AND non-uniform scores. It sits beside the ISOROW and
perception-off-GT gates before ANY launch on either host.

Usage: check_mode_diversity.py <log> [<log> ...] [--min_frac 0.95]
Exit 0 = pass on every log; 2 = fail (prints the failing rate per log).
"""
import argparse, re, sys
PAT = re.compile(r"\[MODESROW\] locale=(\S+) tick=(\d+) tid=(\d+) cid=(\S+) h=(\S+) (.*)")
MODE = re.compile(r"m\d=\(([-\d.]+),([-\d.]+),([\d.]+)\)")

def diversity(path):
    n = ok = 0
    for line in open(path, errors="ignore"):
        m = PAT.search(line)
        if not m: continue
        ms = MODE.findall(m.group(6))
        if not ms: continue
        n += 1
        pts = {(a, b) for a, b, c in ms}; ps = [float(c) for a, b, c in ms]
        uniform = (max(ps) - min(ps)) < 1e-4
        if len(pts) >= 2 and not uniform: ok += 1
    return n, ok

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("logs", nargs="+"); ap.add_argument("--min_frac", type=float, default=0.95)
    a = ap.parse_args(); fail = False
    for p in a.logs:
        n, ok = diversity(p); frac = ok / n if n else 0.0
        status = "PASS" if (n > 0 and frac >= a.min_frac) else "FAIL"
        if status == "FAIL": fail = True
        print(f"[MODE-DIVERSITY] {status} {p}: rows={n} diverse={ok} frac={frac:.3f} (gate >= {a.min_frac})")
    sys.exit(2 if fail else 0)

if __name__ == "__main__":
    main()
