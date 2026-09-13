#!/usr/bin/env python3
"""Forward-motion gate on [MTR RAW] lines: the best mode's step-20 x displacement
in the centre frame must be positive for moving tracks. Usage: check_forward_motion.py <log>... [--min_frac 0.95] [--min_speed 2.0]"""
import argparse, re, sys
PAT = re.compile(r"\[MTR RAW\] tid=\S+ kf_speed=([\d.]+)m/s .*best\[0,1,2,5,10,20\]=(.*)")
PT = re.compile(r"\(([-+\d.]+),([-+\d.]+)\)")
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("logs", nargs="+"); ap.add_argument("--min_frac", type=float, default=0.95); ap.add_argument("--min_speed", type=float, default=2.0)
    a = ap.parse_args(); fail = False
    for p in a.logs:
        n = ok = 0
        for line in open(p, errors="ignore"):
            m = PAT.search(line)
            if not m or float(m.group(1)) < a.min_speed: continue
            pts = PT.findall(m.group(2))
            if len(pts) < 6: continue
            n += 1; ok += float(pts[5][0]) > 0.0
        frac = ok / n if n else 0.0; status = "PASS" if (n > 0 and frac >= a.min_frac) else "FAIL"; fail |= status == "FAIL"
        print(f"[FORWARD-MOTION] {status} {p}: moving_calls={n} forward={ok} frac={frac:.3f} (gate >= {a.min_frac})")
    sys.exit(2 if fail else 0)
if __name__ == "__main__": main()
