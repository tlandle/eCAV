#!/usr/bin/env python3
"""Weak scaling over locale count at FIXED density (design: docs/agent_plans/
metro_weak_scaling_design.md): from a full-network replay's dump (--dump_pooled),
choose for each k in K a k x k tile window whose mean tracks per locale is closest
to the network mean (within --tol; >= --min_fill of the window's tiles holding a
locale), and report per arm the pooled p95 destination load (fraction of the 70 ms
slack) over the window's locales and over its interior locales (all neighbours
inside), with the window's L, density and degree. The centred window is reported
beside it for the record. Usage:
  weak_scaling_windows.py <dump.npz> <out.csv> --ks 2,4,6,8,10 [--tol 0.15] [--min_fill 0.75] [--trace TAG --row TAG]
"""
import argparse, csv, itertools, json, sys
import numpy as np
SLACK_MS = 70.0

def window_locales(tiles, kx0, ky0, k):
    inside = [g for g, (tx, ty) in enumerate(tiles) if kx0 <= tx <= kx0 + k - 1 and ky0 <= ty <= ky0 + k - 1]
    interior = [g for g, (tx, ty) in enumerate(tiles) if kx0 < tx < kx0 + k - 1 and ky0 < ty < ky0 + k - 1]
    return inside, interior

def p95_over(pooled, locs, ncyc):
    if not locs: return None
    sel = np.concatenate([pooled[g * ncyc:(g + 1) * ncyc] for g in locs]); return float(np.percentile(sel, 95) / SLACK_MS)

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dump"); ap.add_argument("out"); ap.add_argument("--ks", default="2,4,6,8,10")
    ap.add_argument("--tol", type=float, default=0.15); ap.add_argument("--min_fill", type=float, default=0.75); ap.add_argument("--trace", default=""); ap.add_argument("--row", default="")
    a = ap.parse_args(); d = np.load(a.dump); L = int(d["L"]); ncyc = int(d["ncyc"]); dens = d["per_locale_tracks_mean"]; tiles = [tuple(t) for t in d["locale_tile"]]; deg = d["degree_per_locale"]
    arms = [k[len("pooled_"):] for k in d.files if k.startswith("pooled_")]; net_mean = float(dens.mean()); net_deg = float(deg.mean())
    txs = [t[0] for t in tiles]; tys = [t[1] for t in tiles]; cx = (min(txs) + max(txs)) // 2; cy = (min(tys) + max(tys)) // 2
    rows = []
    for k in [int(v) for v in a.ks.split(",")]:
        cands = []
        for kx0, ky0 in itertools.product(range(min(txs), max(txs) - k + 2), range(min(tys), max(tys) - k + 2)):
            inside, interior = window_locales(tiles, kx0, ky0, k)
            if len(inside) < a.min_fill * k * k: continue
            m = float(np.mean(dens[inside])); cands.append((abs(m - net_mean) / max(net_mean, 1e-9), kx0, ky0, inside, interior, m))
        if not cands: print(f"k={k}: no window with fill >= {a.min_fill}", file=sys.stderr); continue
        cands.sort(key=lambda c: c[0]); best = cands[0]
        centred = window_locales(tiles, cx - k // 2, cy - k // 2, k)
        for kind, (kx0, ky0, inside, interior, m) in (("density_matched", best[1:]), ("centred", (cx - k // 2, cy - k // 2, centred[0], centred[1], float(np.mean(dens[centred[0]])) if centred[0] else float("nan")))):
            for arm in arms:
                pooled = d[f"pooled_{arm}"]
                rows.append({"trace": a.trace, "row": a.row, "k": k, "window_kind": kind, "window": f"{kx0},{ky0},{kx0 + k - 1},{ky0 + k - 1}", "L": len(inside), "L_interior": len(interior),
                             "tracks_per_locale": round(m, 2), "network_tracks_per_locale": round(net_mean, 2), "density_dev": round((m - net_mean) / max(net_mean, 1e-9), 3), "within_tol": int(abs(m - net_mean) / max(net_mean, 1e-9) <= a.tol),
                             "degree_used": round(float(np.mean(deg[inside])), 2), "network_degree": round(net_deg, 2), "policy": arm,
                             "dest_load_p95": round(p95_over(pooled, inside, ncyc), 4), "dest_load_p95_interior": (round(p95_over(pooled, interior, ncyc), 4) if interior else "")})
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{len(rows)} rows -> {a.out}")
    for r in rows:
        if r["window_kind"] == "density_matched": print(f"  k{r['k']:<2d} L{r['L']:<3d} int L{r['L_interior']:<3d} d {r['tracks_per_locale']:6.2f} (net {r['network_tracks_per_locale']:6.2f}, dev {r['density_dev']:+.3f}, ok {r['within_tol']}) deg {r['degree_used']:.2f} {r['policy']:12s} p95 {r['dest_load_p95']:.3f} interior {r['dest_load_p95_interior']}")

if __name__ == "__main__":
    main()
