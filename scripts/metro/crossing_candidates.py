#!/usr/bin/env python3
"""Destination ambiguity of the replay's crossings, from the road topology.

The warm arm of metro_replay.py places one prepare by a constant-velocity
projection (first projected point, within lead_s, whose locale differs from
the owner); it carries no destination probability. This script characterises
the choice that projection faces:
  per actual crossing (owner change with a previous owner)
    n_cand_dir    neighbouring locales a road user leaving the last junction
                  before the crossing can enter along traversable roads
                  (SUMO edge directions honoured, chains between junctions
                  walked to the next junction);
    n_cand_undir  the same over all incident roads regardless of direction;
  per warm prepare (same rule and horizon as the engine)
    committed     the road user was already on the road that carries the
                  boundary it then crossed (the turn decision was behind it);
    n_cand_ahead  otherwise, the candidate count at the junction ahead;
  per prepare that never crossed into its destination within the replica TTL
    reason        trace_ended | other_destination | stayed_in_owner.
Outputs a JSON summary and prints it.
"""
import argparse
import json
import math
import os
import sys
from collections import defaultdict, deque, Counter

import numpy as np
from scipy.spatial import cKDTree
import sumolib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pneuma_survey as ps  # noqa: E402
from metro_replay import load_pneuma, load_fcd, CYCLE  # noqa: E402


def _node_neighbours(n):
    out = set()
    for e in n.getIncoming():
        out.add(e.getFromNode().getID())
    for e in n.getOutgoing():
        out.add(e.getToNode().getID())
    out.discard(n.getID())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trace", required=True)
    ap.add_argument("--net", required=True)
    ap.add_argument("--tile", type=int, default=300)
    ap.add_argument("--lead_s", type=float, default=0.99)
    ap.add_argument("--replica_ttl_cycles", type=int, default=25)
    ap.add_argument("--max_rows", type=int, default=None)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    net = sumolib.net.readNet(a.net, withInternal=False)
    P = ps.partition(net)
    tree = cKDTree(np.c_[P["X"], P["Y"]])
    grp = ps.tile_anchors(P["anchors"], a.tile)
    lab_of_pt = np.array([grp.get(l, -1) for l in P["LAB"]])
    anchors = set(P["anchors"])
    nodes = {n.getID(): n for n in net.getNodes()}

    # boundaries (roads whose two junctions sit in different locales)
    b_pts, b_loc, b_key, b_anch = [], [], [], []
    adj = defaultdict(list)   # anchor -> [(road key, other anchor)]
    for key, (ja, jb, xy, ln) in P["roads"].items():
        adj[ja].append((key, jb)); adj[jb].append((key, ja))
        ga, gb = grp.get(ja, -1), grp.get(jb, -1)
        if ga != gb and ga >= 0 and gb >= 0:
            b_pts.append(xy); b_loc.append((ga, gb)); b_key.append(key); b_anch.append((ja, jb))
    b_tree = cKDTree(np.array(b_pts))
    pair_index = defaultdict(list)
    for j, (ga, gb) in enumerate(b_loc):
        pair_index[frozenset((ga, gb))].append(j)

    # traversability of a road from anchor a to anchor b along directed SUMO
    # edges, not passing through a third junction
    trav_cache = {}

    def traversable(src, dst):
        k = (src, dst)
        if k in trav_cache:
            return trav_cache[k]
        seen = {src}; dq = deque([src]); ok = False; steps = 0
        while dq and steps < 5000:
            n = dq.popleft(); steps += 1
            for e in nodes[n].getOutgoing():
                t = e.getToNode().getID()
                if t == dst:
                    ok = True; dq.clear(); break
                if t in anchors or t in seen:
                    continue
                seen.add(t); dq.append(t)
        trav_cache[k] = ok
        return ok

    def candidates(anchor, src_loc):
        d, u = set(), set()
        for key, other in adj.get(anchor, ()):
            g = grp.get(other, -1)
            if g < 0 or g == src_loc:
                continue
            u.add(g)
            if traversable(anchor, other):
                d.add(g)
        return d, u

    # chain walk: SUMO edge -> (anchor at the from side, anchor at the to side)
    chain_cache = {}

    def road_of_edge(e):
        eid = e.getID()
        if eid in chain_cache:
            return chain_cache[eid]

        def walk(start, prev):
            n = start; p = prev; hops = 0
            while n not in anchors and hops < 500:
                nb = _node_neighbours(nodes[n]) - {p}
                if len(nb) != 1:
                    return None
                p, n = n, next(iter(nb)); hops += 1
            return n if n in anchors else None
        fa = walk(e.getFromNode().getID(), e.getToNode().getID())
        ta = walk(e.getToNode().getID(), e.getFromNode().getID())
        res = (fa, ta) if (fa is not None and ta is not None and fa != ta) else None
        chain_cache[eid] = res
        return res

    def road_at(x, y):
        cands = net.getNeighboringEdges(x, y, r=15.0)
        if not cands:
            return None
        e, _d = min(cands, key=lambda t: t[1])
        return road_of_edge(e)

    traces = (load_fcd(a.trace, tree) if a.trace.endswith(".xml")
              else load_pneuma(a.trace, net, tree, max_rows=a.max_rows))
    lead_cycles = int(round(a.lead_s / CYCLE))
    kmin = min(int(t[1][0]) for t in traces)

    cross_rows = []          # per crossing
    prep_rows = []           # per prepare
    never = Counter()        # reasons
    prepared_into = defaultdict(set); prepared_crossed = defaultdict(set)
    unmatched_boundary = 0
    for tid, k5, x5, y5, on, i5 in traces:
        labs = np.where(on, lab_of_pt[i5], -1)
        vx = np.gradient(x5, k5 * CYCLE) if len(k5) > 1 else np.zeros_like(x5)
        vy = np.gradient(y5, k5 * CYCLE) if len(k5) > 1 else np.zeros_like(y5)
        db, ib = b_tree.query(np.c_[x5, y5])
        kk = k5 - kmin
        owner = None
        prepared_dst = None; prep = None
        last_on = None
        preps_open = []      # (k_prep, dst, owner_at_prep, committed, n_cand_ahead)
        for j in range(len(kk)):
            k = int(kk[j]); loc = int(labs[j])
            if loc < 0:
                continue
            last_on = k
            if owner is None or loc != owner:
                prev = owner; owner = loc
                if prev is not None:
                    # the boundary crossed: nearest boundary with this locale pair
                    jb_ = int(ib[j])
                    if set(b_loc[jb_]) != {prev, loc}:
                        cand = pair_index.get(frozenset((prev, loc)), [])
                        if cand:
                            pts = np.array([b_pts[c] for c in cand])
                            jb_ = cand[int(np.argmin(np.hypot(pts[:, 0] - x5[j], pts[:, 1] - y5[j])))]
                        else:
                            jb_ = None
                    if jb_ is None:
                        unmatched_boundary += 1
                    else:
                        ja, jb2 = b_anch[jb_]
                        a_src = ja if grp.get(ja, -1) == prev else jb2
                        cd, cu = candidates(a_src, prev)
                        matched = None
                        for pr in preps_open:
                            if pr[1] == loc:
                                matched = pr
                        cross_rows.append({"tid": str(tid), "k": k, "src": prev, "dst": loc,
                                           "n_cand_dir": len(cd), "n_cand_undir": len(cu),
                                           "dst_in_dir": loc in cd,
                                           "prepared": matched is not None,
                                           "committed_at_prepare": (matched[3] if matched else None),
                                           "n_cand_ahead_at_prepare": (matched[4] if matched else None),
                                           "prep_lead_cycles": ((k - matched[0]) if matched else None)})
                        if tid in prepared_into.get(loc, ()):
                            prepared_crossed[loc].add(tid)
                # resolve open prepares against this crossing
                still = []
                for pr in preps_open:
                    if pr[1] == loc:
                        continue                         # crossed as prepared
                    if k - pr[0] <= a.replica_ttl_cycles:
                        never["other_destination"] += 1   # crossed elsewhere within the TTL
                    else:
                        never["stayed_in_owner"] += 1     # TTL expired before any crossing
                preps_open = still
                prepared_dst = None
            # ---- warm prepare (engine rule)
            if prepared_dst is None:
                exit_loc = None
                for s in range(1, lead_cycles + 1):
                    px, py = x5[j] + vx[j] * s * CYCLE, y5[j] + vy[j] * s * CYCLE
                    d_, i_ = tree.query([px, py])
                    if d_ <= 15.0 and lab_of_pt[i_] >= 0 and lab_of_pt[i_] != owner:
                        exit_loc = int(lab_of_pt[i_]); break
                if exit_loc is not None:
                    prepared_dst = exit_loc
                    prepared_into[exit_loc].add(tid)
                    road = road_at(float(x5[j]), float(y5[j]))
                    committed = None; n_ahead = None
                    if road is not None:
                        ra, rb = road
                        ga, gb = grp.get(ra, -1), grp.get(rb, -1)
                        if ga != gb and {ga, gb} == {owner, exit_loc}:
                            committed = True
                        else:
                            committed = False
                            # junction ahead: the anchor end in the direction of travel
                            pa = P["anchors"][ra]; pb = P["anchors"][rb]
                            da = (pa[0] - x5[j]) * vx[j] + (pa[1] - y5[j]) * vy[j]
                            dbb = (pb[0] - x5[j]) * vx[j] + (pb[1] - y5[j]) * vy[j]
                            ahead = ra if da >= dbb else rb
                            cd, _cu = candidates(ahead, owner)
                            n_ahead = len(cd)
                    prep_rows.append({"tid": str(tid), "k": k, "owner": owner, "dst": exit_loc,
                                      "committed": committed, "n_cand_ahead": n_ahead, "road_known": road is not None})
                    preps_open.append((k, exit_loc, owner, committed, n_ahead))
        # end of track: open prepares never resolved
        for pr in preps_open:
            if last_on is not None and last_on - pr[0] <= a.replica_ttl_cycles:
                never["trace_ended"] += 1
            else:
                never["stayed_in_owner"] += 1

    # ---- summaries
    def dist(vals):
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        c = Counter(vals); n = len(vals)
        return {"n": n, "frac_1": c.get(1, 0) / n, "frac_2": c.get(2, 0) / n, "frac_3plus": sum(v for k_, v in c.items() if k_ >= 3) / n,
                "frac_0": c.get(0, 0) / n, "p50": float(np.percentile(vals, 50)), "p90": float(np.percentile(vals, 90)), "max": max(vals),
                "mean": float(np.mean(vals))}
    n_prep = sum(len(v) for v in prepared_into.values()); n_pc = sum(len(v) for v in prepared_crossed.values())
    committed_vals = [r["committed"] for r in prep_rows if r["committed"] is not None]
    summary = {
        "trace": os.path.basename(a.trace), "net": os.path.basename(a.net), "tile_m": a.tile, "lead_s": a.lead_s,
        "L": len(set(v for v in grp.values() if v >= 0)), "boundaries": len(b_pts), "tracks": len(traces),
        "crossings": len(cross_rows), "crossings_unmatched_boundary": unmatched_boundary,
        "n_cand_dir": dist([r["n_cand_dir"] for r in cross_rows]),
        "n_cand_undir": dist([r["n_cand_undir"] for r in cross_rows]),
        "dst_in_dir_frac": (sum(1 for r in cross_rows if r["dst_in_dir"]) / len(cross_rows)) if cross_rows else None,
        "crossings_prepared_frac": (sum(1 for r in cross_rows if r["prepared"]) / len(cross_rows)) if cross_rows else None,
        "prepares": len(prep_rows),
        "prepare_road_known_frac": (sum(1 for r in prep_rows if r["road_known"]) / len(prep_rows)) if prep_rows else None,
        "prepare_committed_frac": (sum(1 for v in committed_vals if v) / len(committed_vals)) if committed_vals else None,
        "n_cand_ahead_uncommitted": dist([r["n_cand_ahead"] for r in prep_rows if r["committed"] is False]),
        "prep_lead_cycles": dist([r["prep_lead_cycles"] for r in cross_rows if r["prepared"]]),
        "engine_metric": {"prepared_tracks_total": n_prep, "prepared_tracks_crossed": n_pc,
                          "never_crossed_frac": (1.0 - n_pc / n_prep) if n_prep else None},
        "never_crossed_reasons": dict(never),
    }
    with open(a.out, "w") as fh:
        json.dump(summary, fh, indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
