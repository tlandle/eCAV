#!/usr/bin/env python3
"""Radio tails of a co-simulated closed-loop run.

Parses the per-tick radio markers an edge writes under NS3_COSIM=1 and
reports, per edge and pooled, the order statistics the validation block
compares between the live ns-3 path and the table path:
  [RADIOROW]     tick edge n_ul ul_live_ms ul_lut_ms delivered_frac ns3_wall_ms n_results
  [RADIOROW-DL]  tick edge dl_live_ms dl_lut_ms
  [RADIOINIT]    edge start_ms shm | edge first_tick_ms n_ue n_results
  [AGEROW]       tick edge realized_age_ms network_age_ms lut_n
A row whose n_results is 0 kept the table's value for that tick (bridge
returned nothing); such rows are counted separately and excluded from the
live quantiles. Output: a text table on stdout and, with --json, one JSON
document with the same numbers.
"""
import argparse
import json
import re
import sys
from collections import defaultdict

_KV = re.compile(r'(\w+)=([-\w.:/]+)')


def _kv(line):
    return {k: v for k, v in _KV.findall(line)}


def _q(xs, p):
    if not xs:
        return None
    s = sorted(xs)
    i = min(len(s) - 1, max(0, int(round(p * (len(s) - 1)))))
    return s[i]


def _stats(xs):
    if not xs:
        return {'n': 0}
    return {'n': len(xs), 'p50': _q(xs, 0.5), 'p95': _q(xs, 0.95), 'p99': _q(xs, 0.99),
            'max': max(xs), 'min': min(xs), 'mean': sum(xs) / len(xs)}


def parse(path):
    per = defaultdict(lambda: {'ul_live': [], 'ul_lut': [], 'ul_lut_all': [], 'dl_live': [], 'dl_lut': [],
                               'wall': [], 'delivered': [], 'n_ul': [], 'empty_ticks': 0, 'ul_ticks': 0,
                               'start_ms': None, 'first_tick_ms': None, 'first_n_ue': None, 'shm': None,
                               'realized_age': [], 'network_age': [], 'first_tick': None, 'last_tick': None,
                               'radioerr': 0})
    with open(path, errors='replace') as fh:
        for line in fh:
            if '[RADIOROW]' in line:
                d = _kv(line.split('[RADIOROW]', 1)[1])
                e = per[d['edge']]
                e['ul_ticks'] += 1
                t = int(d['tick'])
                e['first_tick'] = t if e['first_tick'] is None else e['first_tick']
                e['last_tick'] = t
                e['ul_lut_all'].append(float(d['ul_lut_ms']))
                if int(d.get('n_results', 0)) <= 0:
                    e['empty_ticks'] += 1
                    continue
                e['ul_live'].append(float(d['ul_live_ms']))
                e['ul_lut'].append(float(d['ul_lut_ms']))
                e['wall'].append(float(d['ns3_wall_ms']))
                e['delivered'].append(float(d['delivered_frac']))
                e['n_ul'].append(int(d['n_ul']))
            elif '[RADIOROW-DL]' in line:
                d = _kv(line.split('[RADIOROW-DL]', 1)[1])
                e = per[d['edge']]
                e['dl_live'].append(float(d['dl_live_ms']))
                e['dl_lut'].append(float(d['dl_lut_ms']))
            elif '[RADIOINIT]' in line:
                d = _kv(line.split('[RADIOINIT]', 1)[1])
                e = per[d['edge']]
                if 'start_ms' in d:
                    e['start_ms'] = float(d['start_ms'])
                    e['shm'] = d.get('shm')
                if 'bringup_ms' in d:      # pre-fix marker name
                    e['start_ms'] = float(d['bringup_ms'])
                if 'first_tick_ms' in d:
                    e['first_tick_ms'] = float(d['first_tick_ms'])
                    e['first_n_ue'] = int(d.get('n_ue', -1))
            elif '[RADIOERR]' in line:
                d = _kv(line.split('[RADIOERR]', 1)[1])
                per[d.get('edge', '?')]['radioerr'] += 1
            elif '[AGEROW]' in line:
                d = _kv(line.split('[AGEROW]', 1)[1])
                e = per[d['edge']]
                e['realized_age'].append(float(d['realized_age_ms']))
                e['network_age'].append(float(d['network_age_ms']))
    return per


def summarize(per):
    out = {}
    pooled = defaultdict(list)
    for edge, e in per.items():
        s = {
            'ul_ticks': e['ul_ticks'], 'empty_ticks': e['empty_ticks'], 'radioerr': e['radioerr'],
            'ticks': [e['first_tick'], e['last_tick']],
            'start_ms': e['start_ms'], 'first_tick_ms': e['first_tick_ms'], 'first_n_ue': e['first_n_ue'],
            'shm': e['shm'],
            'n_ul': _stats(e['n_ul']),
            'ul_live_ms': _stats(e['ul_live']), 'ul_lut_ms_same_ticks': _stats(e['ul_lut']),
            'ul_lut_ms_all_ticks': _stats(e['ul_lut_all']),
            'dl_live_ms': _stats(e['dl_live']), 'dl_lut_ms': _stats(e['dl_lut']),
            'ns3_wall_ms': _stats(e['wall']),
            'delivered_frac_mean': (sum(e['delivered']) / len(e['delivered'])) if e['delivered'] else None,
            'realized_age_ms': _stats(e['realized_age']), 'network_age_ms': _stats(e['network_age']),
        }
        out[edge] = s
        for k in ('ul_live', 'ul_lut', 'dl_live', 'dl_lut', 'wall', 'realized_age', 'network_age'):
            pooled[k].extend(e[k])
    out['_pooled'] = {
        'ul_live_ms': _stats(pooled['ul_live']), 'ul_lut_ms_same_ticks': _stats(pooled['ul_lut']),
        'dl_live_ms': _stats(pooled['dl_live']), 'dl_lut_ms': _stats(pooled['dl_lut']),
        'ns3_wall_ms': _stats(pooled['wall']),
        'realized_age_ms': _stats(pooled['realized_age']), 'network_age_ms': _stats(pooled['network_age']),
        'empty_ticks': sum(e['empty_ticks'] for e in per.values()),
        'ul_ticks': sum(e['ul_ticks'] for e in per.values()),
        'radioerr': sum(e['radioerr'] for e in per.values()),
    }
    return out


def _fmt(st):
    if not st or st.get('n', 0) == 0:
        return 'n=0'
    return ('n=%d p50=%.1f p95=%.1f p99=%.1f max=%.1f' % (st['n'], st['p50'], st['p95'], st['p99'], st['max']))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--json', help='write the summary here')
    a = ap.parse_args()
    per = parse(a.log)
    if not per:
        print('no radio markers in', a.log)
        return 1
    s = summarize(per)
    for edge, st in s.items():
        if edge == '_pooled':
            continue
        print('edge %s ticks %s ul_ticks=%d empty=%d radioerr=%d start_ms=%s first_tick_ms=%s (n_ue=%s) shm=%s' % (
            edge, st['ticks'], st['ul_ticks'], st['empty_ticks'], st['radioerr'], st['start_ms'],
            st['first_tick_ms'], st['first_n_ue'], st['shm']))
        print('  n_ul       %s' % _fmt(st['n_ul']))
        print('  UL live    %s' % _fmt(st['ul_live_ms']))
        print('  UL table   %s   (same ticks)' % _fmt(st['ul_lut_ms_same_ticks']))
        print('  DL live    %s' % _fmt(st['dl_live_ms']))
        print('  DL table   %s' % _fmt(st['dl_lut_ms']))
        print('  wall       %s' % _fmt(st['ns3_wall_ms']))
        print('  delivered  %s' % ('%.3f' % st['delivered_frac_mean'] if st['delivered_frac_mean'] is not None else 'n/a'))
        print('  age real.  %s' % _fmt(st['realized_age_ms']))
        print('  age net.   %s' % _fmt(st['network_age_ms']))
    p = s['_pooled']
    print('pooled ul_ticks=%d empty=%d radioerr=%d' % (p['ul_ticks'], p['empty_ticks'], p['radioerr']))
    for k in ('ul_live_ms', 'ul_lut_ms_same_ticks', 'dl_live_ms', 'dl_lut_ms', 'ns3_wall_ms', 'realized_age_ms', 'network_age_ms'):
        print('  %-22s %s' % (k, _fmt(p[k])))
    if a.json:
        with open(a.json, 'w') as fh:
            json.dump(s, fh, indent=1)
        print('wrote', a.json)
    return 0


if __name__ == '__main__':
    sys.exit(main())
