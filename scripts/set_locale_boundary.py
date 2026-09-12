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


def set_boundary(path, b, overlap=OVERLAP):
    if not (100.0 < b < CONFLICT_X):
        raise SystemExit(
            'boundary %.1f outside (100, %.1f); past the conflict the NPC would '
            'cross after the interaction it is being prepared for' % (b, CONFLICT_X))
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
            elif cur == 'locale_1' and abs(x - 250.0) < 60.0 and x > 200.0:
                new = b
            if new is not None:
                n[cur] += 1
                out.append('%s%g%s%s%s' % (p.group(1), new, p.group(3), p.group(4), p.group(5)))
                continue
        if re.match(r'\s*(vehicles|locale|spawn_position):', line):
            if not re.match(r'\s*locale:', line):
                cur = None
        out.append(line)
    if n['locale_0'] != 2 or n['locale_1'] != 2:
        raise SystemExit('expected 2 edits per locale, got %r; YAML shape changed' % n)
    open(path, 'w').write('\n'.join(out))
    return n


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit('usage: set_locale_boundary.py <yaml> <boundary_x>')
    print('[boundary] %s -> x=%s %r' % (sys.argv[1], sys.argv[2],
                                        set_boundary(sys.argv[1], float(sys.argv[2]))))
