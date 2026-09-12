#!/usr/bin/env python3
"""Move the locale_0/locale_1 boundary in a scenario YAML by rewriting just the
polygon x-coordinates, leaving the rest of the file byte-identical.

The boundary is the shared edge of the two locale polygons. locale_1 (west,
x in [80, B]) hands off to locale_0 (east, x in [B - OVERLAP, 360]); the
overlap is VehicleLocaleTracker's hysteresis band and is preserved.

Sweeping B is the clean T_avail knob: it changes the distance from the
oncoming NPC's locale crossing to the conflict (x ~ 286) without touching
vehicle speeds, spawn phase, road geometry or the conflict itself.
"""
import re
import sys

OVERLAP = 10.0
CONFLICT_X = 286.0   # measured: blocked egos stall at 286-292 (frozen_1n logs)
RSU0_X, RSU1_X = 325.0, 205.0   # each RSU must stay inside the locale it serves

# Moving only the shared edge also resizes both locales: locale_0 spans 200 m at
# b=170 and 95 m at b=275. Fusion cost scales with locale width (S2), so the
# destination's cycle cost varies along the sweep. Measured fusion is ~63 ms
# against the 130 ms allocation, and the implied swing (~55 ms) is ~0.6% of the
# 8.8 s T_avail range, so it is second order. Pass fixed widths to remove it
# anyway: both locales keep their width and the whole partition translates.


def set_boundary(path, b, overlap=OVERLAP, widths=None):
    """widths=None resizes both locales (simple sweep); widths=(w1, w0) keeps both
    widths fixed and translates the partition, which holds fusion cost constant."""
    if not (100.0 < b < CONFLICT_X):
        raise SystemExit(
            'boundary %.1f outside (100, %.1f); past the conflict the NPC would '
            'cross after the interaction it is being prepared for' % (b, CONFLICT_X))
    lo0_hi = lo1_lo = None
    if widths is not None:
        w1, w0 = widths
        lo0_hi, lo1_lo = b - overlap + w0, b - w1
        if lo0_hi < max(CONFLICT_X, RSU0_X):
            raise SystemExit(
                'locale_0 ends at %.1f, west of the conflict/RSU0 at %.1f; widen w0 '
                'or lower the boundary' % (lo0_hi, max(CONFLICT_X, RSU0_X)))
        if not (lo1_lo <= RSU1_X <= b):
            raise SystemExit('RSU1 at %.1f falls outside locale_1 [%.1f, %.1f]'
                             % (RSU1_X, lo1_lo, b))
    src = open(path).read()
    out, n = [], {'locale_0': 0, 'locale_1': 0}
    cur = None
    for line in src.split('\n'):
        m = re.match(r'\s*id:\s*(locale_[01])\s*$', line)
        if m:
            cur = m.group(1)
            out.append(line)
            continue
        p = re.match(r'^(\s*-\s*\[)\s*(-?[\d.]+)(\s*,\s*)(-?[\d.]+)(\s*\]\s*)$', line)
        if p and cur:
            x = float(p.group(2))
            new = None
            if cur == 'locale_0' and abs(x - 240.0) < 60.0 and x < 300.0:
                new = b - overlap
            elif cur == 'locale_0' and x >= 300.0 and lo0_hi is not None:
                new = lo0_hi
            elif cur == 'locale_1' and abs(x - 250.0) < 60.0 and x > 200.0:
                new = b
            elif cur == 'locale_1' and x < 200.0 and lo1_lo is not None:
                new = lo1_lo
            if new is not None:
                n[cur] += 1
                out.append('%s%g%s%s%s' % (p.group(1), new, p.group(3), p.group(4), p.group(5)))
                continue
        if re.match(r'\s*(vehicles|locale|spawn_position):', line):
            if not re.match(r'\s*locale:', line):
                cur = None
        out.append(line)
    want = 4 if widths is not None else 2
    if n['locale_0'] != want or n['locale_1'] != want:
        raise SystemExit('expected 2 edits per locale, got %r; YAML shape changed' % n)
    open(path, 'w').write('\n'.join(out))
    return n


if __name__ == '__main__':
    if len(sys.argv) not in (3, 4):
        raise SystemExit('usage: set_locale_boundary.py <yaml> <boundary_x> [w1,w0]')
    w = None
    if len(sys.argv) == 4:
        w = tuple(float(v) for v in sys.argv[3].split(','))
    print('[boundary] %s -> x=%s widths=%s %r'
          % (sys.argv[1], sys.argv[2], w,
             set_boundary(sys.argv[1], float(sys.argv[2]), widths=w)))
