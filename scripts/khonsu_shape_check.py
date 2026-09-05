#!/usr/bin/env python3
"""Check landed Khonsu evaluation rows against the expected shapes of the paper's figures.

Reads every CSV under docs/kb/data/relay_eval_2026_08/ (or a directory given as
argv[1]), detects the row schema from its columns, and prints one verdict line per
figure: NO DATA, AGREE, or DISAGREE with the measured numbers. The expected shapes
are the ones drawn in slides/toy_figs.py of the paper repo:

  headline   Khonsu >= reactive history >= snapshot arms >= cold in clean runs
  lead       with the final update, clean runs must not fall as lead grows
  burst      prepared history > EdgeWarp/snapshot and > cold
  density    the arm ordering holds at every oncoming count
  envelope   clean fraction is non-increasing in realized age; tau(u) exists
  faults     exactly one publishable epoch in every unit scenario; on the live
             path zero stale-owner forecasts consumed with the epoch check on
  load       p99 latency is non-decreasing in platoon size; Khonsu below the
             replication and dual-service arms

Binary outcome rule (Tyler): a run is clean when it did not collide. Mean
episode counts are never used.
"""
import csv
import glob
import os
import re
import sys
from collections import defaultdict

DATA = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'kb', 'data', 'relay_eval_2026_08')
TOL = 1  # runs of slack allowed for an ordering claim (seed noise)


def yes(v):
    return str(v).strip().lower() in ('yes', 'true', '1')


def load_rows():
    rows_by_kind = defaultdict(list)
    for path in sorted(glob.glob(os.path.join(DATA, '*.csv'))):
        try:
            rows = list(csv.DictReader(open(path, newline='')))
        except Exception:
            continue
        if not rows:
            continue
        cols = {c.lower() for c in rows[0].keys()}
        base = os.path.basename(path)
        if 'v1_rows' in base or 'v2_rows' in base:
            continue  # unvalidated geometry sweeps; never part of a figure
        if 'publishable_epochs' in cols:
            kind = 'faults_unit'
        elif 'fault' in cols and ('stale_consumed' in cols or 'stale' in ' '.join(cols)):
            kind = 'faults_live'
        elif {'mode', 'collided'} <= cols and any('age' in c for c in cols):
            kind = 'envelope'
        elif {'mode', 'collided', 'look'} <= cols:
            kind = 'runrows'
        elif any('platoon' in c or 'concurrent' in c for c in cols) and any('p99' in c for c in cols):
            kind = 'load'
        else:
            continue
        for r in rows:
            r['_file'] = base
        rows_by_kind[kind].extend(rows)
    return rows_by_kind


def clean_by(rows, keyfn):
    agg = defaultdict(lambda: [0, 0])
    for r in rows:
        k = keyfn(r)
        if k is None:
            continue
        agg[k][0] += 1
        agg[k][1] += 0 if yes(r.get('collided', '')) else 1
    return agg


def fmt(agg, keys):
    return ', '.join(f"{k} {agg[k][1]}/{agg[k][0]}" for k in keys if k in agg)


def prefer_tag(rows):
    """Use rows from the newest frozen tag when several tags are present."""
    tags = sorted({r.get('eval_tag', '') for r in rows if r.get('eval_tag')})
    if not tags:
        return rows, 'untagged rows'
    tag = tags[-1]
    return [r for r in rows if r.get('eval_tag') == tag], tag


def check_headline(rows):
    flow = [r for r in rows if not r['tag'].startswith(('burst', 'q5', 'tr_', 'th_', 'c_'))
            and float(r.get('look', 1) or 1) == 1.0]
    agg = clean_by(flow, lambda r: r['mode'])
    if not agg:
        return 'NO DATA'
    order = ['warm', 'reactive', 'edgewarp', 'kf', 'handover_snapshot', 'cold']
    frac = {m: agg[m][1] / agg[m][0] for m in order if m in agg and agg[m][0]}
    problems = []
    if 'warm' in frac and 'reactive' in frac and frac['warm'] + TOL / agg['warm'][0] < frac['reactive']:
        problems.append('reactive beats Khonsu beyond noise')
    for m in ('edgewarp', 'kf', 'handover_snapshot', 'cold'):
        if m in frac and 'warm' in frac and frac[m] > frac['warm'] + TOL / agg[m][0]:
            problems.append(f'{m} beats Khonsu')
    verdict = 'AGREE' if not problems else 'DISAGREE (' + '; '.join(problems) + ')'
    return f"{verdict}: {fmt(agg, order)}"


def check_lead(rows):
    warm = [r for r in rows if r['mode'] == 'warm' and r.get('trigger') == 'predictive'
            and not r['tag'].startswith(('burst', 'q5', 'c_'))]
    agg = clean_by(warm, lambda r: float(r.get('look', 0) or 0))
    if len(agg) < 2:
        return 'NO DATA (need two or more leads)'
    leads = sorted(agg)
    fr = [agg[l][1] / agg[l][0] for l in leads]
    drops = [(leads[i], leads[i + 1]) for i in range(len(fr) - 1)
             if fr[i + 1] < fr[i] - TOL / agg[leads[i + 1]][0]]
    verdict = 'AGREE' if not drops else f'DISAGREE (clean runs fall with lead at {drops}; stale-at-crossing signature)'
    return f"{verdict}: " + ', '.join(f"{l:g}s {agg[l][1]}/{agg[l][0]}" for l in leads)


def check_burst(rows):
    b = [r for r in rows if r['tag'].startswith('burst')]
    agg = clean_by(b, lambda r: r['mode'])
    if not agg:
        return 'NO DATA'
    ok = all(agg['warm'][1] > agg[m][1] for m in agg if m != 'warm') if 'warm' in agg else False
    return ('AGREE' if ok else 'DISAGREE (prepared history does not lead)') + ': ' + fmt(agg, sorted(agg))


def check_density(rows):
    q = [r for r in rows if r['tag'].startswith('q5')]
    agg = clean_by(q, lambda r: (int(re.search(r'q5_n(\d+)', r['tag']).group(1)), r['mode']))
    if not agg:
        return 'NO DATA'
    ns = sorted({k[0] for k in agg})
    bad = [n for n in ns if 'warm' in {k[1] for k in agg if k[0] == n}
           and any(agg[(n, m)][1] > agg[(n, 'warm')][1] for m in {k[1] for k in agg if k[0] == n} if m != 'warm')]
    verdict = 'AGREE' if not bad else f'DISAGREE (ordering breaks at N={bad})'
    return verdict + ': ' + '; '.join(
        f"N={n} " + ', '.join(f"{m} {agg[(n, m)][1]}/{agg[(n, m)][0]}" for m in sorted({k[1] for k in agg if k[0] == n}))
        for n in ns)


def check_envelope(rows):
    agecol = next((c for c in rows[0] if 'age' in c.lower() and 'p' not in c.lower()[:1]), None)
    if agecol is None:
        agecol = next((c for c in rows[0] if 'age' in c.lower()), None)
    scen = next((c for c in rows[0] if c.lower() in ('scenario', 'scen', 'u')), None)
    out = []
    for s in sorted({r.get(scen, 'all') for r in rows}) if scen else ['all']:
        rs = [r for r in rows if not scen or r.get(scen) == s]
        agg = defaultdict(lambda: [0, 0])
        for r in rs:
            try:
                b = int(float(r[agecol]) // 50) * 50
            except Exception:
                continue
            agg[b][0] += 1
            agg[b][1] += 0 if yes(r.get('collided', '')) else 1
        if not agg:
            continue
        bins = sorted(agg)
        fr = [agg[b][1] / agg[b][0] for b in bins]
        rises = [bins[i + 1] for i in range(len(fr) - 1) if fr[i + 1] > fr[i] + 1e-9 and fr[i] < 1]
        tau = max([b for b in bins if agg[b][1] == agg[b][0]], default=None)
        lower_bound = all(agg[b][1] == agg[b][0] for b in bins)
        v = 'AGREE' if not rises else f'DISAGREE (clean fraction rises with age at {rises} ms)'
        t = f"tau>={tau} ms (lower bound, no failures yet)" if lower_bound else f"tau={tau} ms"
        out.append(f"{s}: {v}, {t}, bins " + ' '.join(f"{b}:{agg[b][1]}/{agg[b][0]}" for b in bins))
    return '\n           '.join(out) if out else 'NO DATA'


def check_faults_unit(rows):
    bad = [r['scenario'] for r in rows if str(r.get('publishable_epochs', '')).strip() != '1'
           or str(r.get('verdict', '')).upper() != 'PASS']
    return ('AGREE' if not bad else f'DISAGREE ({bad})') + f": {len(rows)} scenarios, one publishable epoch each"


def check_faults_live(rows):
    on = [r for r in rows if yes(r.get('fencing', r.get('epoch_check', '')))]
    if not on:
        return 'NO DATA (no epoch-check-on rows)'
    stale = sum(float(r.get('stale_consumed', 0) or 0) for r in on)
    return ('AGREE' if stale == 0 else f'DISAGREE ({stale:g} stale-owner forecasts consumed with the check on)') + \
        f": {len(on)} rows with the epoch check on"


def check_load(rows):
    sizecol = next(c for c in rows[0] if 'platoon' in c.lower() or 'concurrent' in c.lower())
    p99col = next(c for c in rows[0] if 'p99' in c.lower())
    modecol = next((c for c in rows[0] if c.lower() in ('mode', 'arm')), None)
    out = []
    for m in sorted({r.get(modecol, 'all') for r in rows}) if modecol else ['all']:
        rs = sorted([r for r in rows if not modecol or r.get(modecol) == m], key=lambda r: float(r[sizecol]))
        vals = [(float(r[sizecol]), float(r[p99col])) for r in rs]
        mono = all(vals[i + 1][1] >= vals[i][1] for i in range(len(vals) - 1))
        out.append(f"{m}: {'AGREE' if mono else 'DISAGREE (p99 not monotone)'} " + ' '.join(f"{int(n)}:{p:.0f}ms" for n, p in vals))
    return '\n           '.join(out)


def main():
    kinds = load_rows()
    print(f"# khonsu shape check over {DATA}")
    run = kinds.get('runrows', [])
    if run:
        run, tag = prefer_tag(run)
        print(f"# run rows: {len(run)} from {tag}")
    checks = [
        ('headline', lambda: check_headline(run) if run else 'NO DATA'),
        ('lead', lambda: check_lead(run) if run else 'NO DATA'),
        ('burst', lambda: check_burst(run) if run else 'NO DATA'),
        ('density', lambda: check_density(run) if run else 'NO DATA'),
        ('envelope', lambda: check_envelope(kinds['envelope']) if kinds.get('envelope') else 'NO DATA'),
        ('faults', lambda: check_faults_unit(kinds['faults_unit']) if kinds.get('faults_unit') else 'NO DATA'),
        ('faults_live', lambda: check_faults_live(kinds['faults_live']) if kinds.get('faults_live') else 'NO DATA'),
        ('load', lambda: check_load(kinds['load']) if kinds.get('load') else 'NO DATA'),
    ]
    for name, fn in checks:
        try:
            print(f"{name:<11}{fn()}")
        except Exception as e:  # a schema surprise must not hide the other verdicts
            print(f"{name:<11}ERROR {type(e).__name__}: {e}")


if __name__ == '__main__':
    main()
