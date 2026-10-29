#!/usr/bin/env python3
"""Check the landed j5 data against the figure shapes the v14 review expects (Tyler, Oct 28: checks on
every landing, no exceptions). Reads the hosts' runs.csv files (envelope_measured.py), the trace replay
CSV and the fault CSVs when present, places every cell at its configured delay, and prints PASS / FAIL
per expectation with the numbers behind it. Writes the same report as Markdown.
    python scripts/sensys_analysis/shape_check.py --runs <runs.csv ...> [--onc20 runs.csv --onc28 runs.csv]
        [--replay figs/trace_replay.csv] [--faults figs/faults_lt.csv figs/faults_scp.csv] --out figs/shape_check.md
"""
import argparse, collections, csv, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from campaign_check import wilson  # noqa: E402

TH = 0.8
LT, SCP, OV = "openscenario_3_edge_late_fusion", "scp_edge_late_fusion", "blind_overtake_edge_late_fusion"
GN = {LT: "left turn", SCP: "straight crossing", OV: "blind overtake"}


def arm_of(r, kv):
    if kv.get("behavior.hazard_response"):
        return "ctrl" if r["mgr"] == "oracle" else None
    if any(k.startswith("assoc_") for k in kv):
        return ("gate+c" if r["anch"] == "on" else "gate") if r["mgr"] == "late_fusion" else None
    m = r["mgr"]
    if m == "oracle": return "reference"
    if m == "late_fusion": return "contract" if r["anch"] == "on" else "naive"
    if "etsi" in m: return "etsi"
    if m == "infra_only": return "i2v"
    if m == "vips": return "vips+c" if r["anch"] == "on" else "vips"
    return None


def load(paths, tag=""):
    cells = collections.defaultdict(lambda: {"k": 0, "n": 0, "cls": collections.Counter()})
    for p in paths:
        if not p or not os.path.exists(p): continue
        for r in csv.DictReader(open(p)):
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("beacon_fault", "beacon_validity")) for k in kv) or "v2x_regime" in r.get("cell", ""): continue
            arm = arm_of(r, kv)
            if arm is None: continue
            c = cells[(r["geom"] + tag, int(r["speed"]), int(r["N"]), arm, int(r["lat"]))]
            c["k"] += int(r["s_op"]); c["n"] += 1
            if r["s_op"] == "0":
                c["cls"]["collision" if r["collided"] == "1" else "self_ghost" if r["self_ghost"] == "1" else "false_brake" if r["false_brake"] == "1" else "late"] += 1
    return cells


def curve(cells, geom, speed, N, arm):
    return sorted((lat, c["k"], c["n"], c["cls"]) for (g, s, n, a, lat), c in cells.items() if g == geom and s == speed and n == N and a == arm)


def pav(pts):
    out = []
    for lat, k, n, _ in pts:
        out.append([lat, k, n])
        while len(out) > 1 and out[-1][1] / out[-1][2] > out[-2][1] / out[-2][2]:
            x1, k1, n1 = out.pop(); x0, k0, n0 = out.pop(); out.append([x0, k0 + k1, n0 + n1])
    return [(x, k / n) for x, k, n in out]


def budget(pts):
    """(largest configured delay with the monotone fit at or above TH, censored?)"""
    if not pts: return None, True
    fit = pav(pts); best = None
    for lat, k, n, _ in pts:
        p = next(v for x, v in reversed(fit) if x <= lat)
        if p >= TH: best = lat
        else: break
    censored = best == pts[-1][0]
    return best, censored


class Report:
    def __init__(self): self.lines = []; self.fails = 0
    def head(self, s): self.lines += ["", f"## {s}"]
    def check(self, ok, what, detail):
        self.lines.append(f"- {'PASS' if ok else 'FAIL'}: {what}. {detail}")
        if not ok: self.fails += 1
    def note(self, s): self.lines.append(f"- note: {s}")


def fmt(pts):
    return " ".join(f"{lat}:{k}/{n}" + ("" if not cls else "{" + ",".join(f"{a}={b}" for a, b in cls.items()) + "}") for lat, k, n, cls in pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True); ap.add_argument("--onc20"); ap.add_argument("--onc28")
    ap.add_argument("--replay"); ap.add_argument("--faults", nargs="*", default=[]); ap.add_argument("--out", required=True)
    ap.add_argument("--speed", type=int, default=70)
    a = ap.parse_args()
    cells = load(a.runs)
    cells.update(load([a.onc20], "@onc20")); cells.update(load([a.onc28], "@onc28"))
    R = Report()

    R.head("Fig. 6 envelopes at %d km/h, N=1 (expected: all arms high at low age; naive falls by self-ghost while the reference and the contract hold; the reference falls at high age; no arm above the reference)" % a.speed)
    for geom in (LT, SCP):
        ref, nv, ct = (curve(cells, geom, a.speed, 1, x) for x in ("reference", "naive", "contract"))
        if not ref: R.check(False, f"{GN[geom]}: reference cells present", "none"); continue
        low = all(pts and pts[0][1] / pts[0][2] >= TH for pts in (ref, nv, ct))
        R.check(low, f"{GN[geom]}: every arm at or above {TH} at the lowest delay", f"ref {fmt(ref[:1])} naive {fmt(nv[:1])} contract {fmt(ct[:1])}")
        refp = {lat: k / n for lat, k, n, _ in ref}; ctp = {lat: k / n for lat, k, n, _ in ct}
        early = [(lat, k, n, cls) for lat, k, n, cls in nv if k / n < TH and refp.get(lat, 0) >= TH and ctp.get(lat, 0) >= TH]
        ghostly = [e for e in early if e[3].get("self_ghost", 0) >= (e[2] - e[1]) / 2]
        R.check(bool(ghostly), f"{GN[geom]}: naive falls below {TH} by self-ghost at a delay where the reference and the contract hold",
                f"naive {fmt(nv)}" + (f"; qualifying delays {[e[0] for e in ghostly]}" if ghostly else "; no such delay"))
        rb, rc = budget(ref)
        R.check(not rc, f"{GN[geom]}: the reference falls below {TH} inside the sweep (physics boundary observed)", f"reference {fmt(ref)}; budget {rb} ms" + (" (censored)" if rc else ""))
        cb, cc = budget(ct)
        R.check(cb is not None and rb is not None and (cc or abs(cb - rb) <= 100), f"{GN[geom]}: contract budget within one step of the reference or censored at the sweep end", f"contract budget {cb}{' (censored)' if cc else ''} vs reference {rb}")
        for arm in ("naive", "contract", "etsi", "i2v"):
            for lat, k, n, cls in curve(cells, geom, a.speed, 1, arm):
                if lat in refp:
                    rk, rn = next((kk, nn) for l, kk, nn, _ in ref if l == lat)
                    if wilson(k, n)[1] > wilson(rk, rn)[2]:
                        R.check(False, f"{GN[geom]}: {arm} above the reference at {lat} ms", f"{arm} {k}/{n} vs reference {rk}/{rn}")

    R.head("Fig. 7a useful-age budget vs ego speed (expected: budgets fall with speed, maneuvers differ, contract within one step of the reference)")
    for geom in (LT, SCP):
        for arm in ("reference", "contract"):
            bs = []
            for sp in (38, 50, 53, 63, 70, 88):
                pts = curve(cells, geom, sp, 1, arm)
                if pts: b, c = budget(pts); bs.append((sp, b, c, pts[-1][0]))
            if not bs: R.check(False, f"{GN[geom]} {arm}: budgets present", "no cells"); continue
            R.note(f"{GN[geom]} {arm}: " + ", ".join(f"{sp}: {b} ms{' censored at ' + str(top) if c else ''}" for sp, b, c, top in bs))
            vals = [(sp, b) for sp, b, c, _ in bs if b is not None]
            rev = [(s0, b0, s1, b1) for (s0, b0), (s1, b1) in zip(vals, vals[1:]) if b1 > b0]
            R.check(not rev, f"{GN[geom]} {arm}: budget non-increasing with speed", f"reversals {rev}" if rev else "monotone")
            cens = [sp for sp, b, c, _ in bs if c]
            R.check(not cens, f"{GN[geom]} {arm}: no censored budget", f"censored at {cens}" if cens else "all boundaries observed")

    R.head("Fig. 7b budget vs oncoming speed on the blind overtake (expected: 12 >= 20 >= 28 m/s for the reference, same for the contract)")
    for arm in ("reference", "contract"):
        bs = []
        for tag, onc in (("", 12), ("@onc20", 20), ("@onc28", 28)):
            pts = curve(cells, OV + tag, 43, 1, arm)
            if pts: b, c = budget(pts); bs.append((onc, b, c))
        if len(bs) < 3: R.check(False, f"overtake {arm}: all three oncoming speeds present", f"have {[o for o, _, _ in bs]}"); continue
        order = all(bs[i][1] is not None and bs[i + 1][1] is not None and bs[i][1] >= bs[i + 1][1] for i in range(2))
        R.check(order, f"overtake {arm}: budgets ordered 12 >= 20 >= 28 m/s", ", ".join(f"{o}: {b}{' censored' if c else ''}" for o, b, c in bs))

    R.head("Fig. 10 early failures (expected at the largest delay where the reference fails in no run: naive and ETSI fail by self-ghost, contract none)")
    for geom, N in ((LT, 4), (SCP, 4), (OV, 1)):
        sp = 43 if geom == OV else a.speed
        ref = curve(cells, geom, sp, N, "reference")
        if not ref: R.check(False, f"{GN[geom]} N={N}: reference present", "none"); continue
        rep = max((lat for lat, k, n, _ in ref if k == n), default=None)
        if rep is None: R.check(False, f"{GN[geom]} N={N}: a delay where the reference fails in no run", fmt(ref)); continue
        row = {arm: next(((k, n, cls) for lat, k, n, cls in curve(cells, geom, sp, N, arm) if lat == rep), None) for arm in ("naive", "etsi", "contract")}
        detail = f"delay {rep} ms: " + ", ".join(f"{arm} {v[1]-v[0]}/{v[1]} failed {dict(v[2])}" if v else f"{arm} missing" for arm, v in row.items())
        ok = all(row[x] and row[x][1] - row[x][0] > 0 and row[x][2].get("self_ghost", 0) > 0 for x in ("naive", "etsi")) and row["contract"] and row["contract"][1] == row["contract"][0]
        R.check(ok, f"{GN[geom]} N={N}: naive and ETSI fail by self-ghost, contract does not", detail)

    R.head("Fig. 11 trace replay (expected: observed success inside the envelope's interval at the replayed age)")
    if a.replay and os.path.exists(a.replay):
        rows = list(csv.DictReader(open(a.replay)))
        bad = [f"{r['arm']} / {r['segment']}: observed {r['observed']} predicted {r['predicted']}" for r in rows if r["inside_interval"] != "1"]
        R.check(not bad, "every replay cell inside its interval", "; ".join(bad) if bad else f"{len(rows)} cells inside")
    else:
        R.check(False, "trace replay CSV present", "missing")

    R.head("Fig. 12 association baselines at the left turn (expected: residual failures for geometric, ETSI, Mahalanobis and VIPS; none of the self-ghost kind with the contract; anomalies need explanation)")
    for arm in ("gate", "gate+c", "vips", "vips+c", "etsi", "naive", "contract"):
        pts = curve(cells, LT, a.speed, 1, arm)
        if not pts: R.note(f"{arm}: no cells"); continue
        ps = [(lat, k / n) for lat, k, n, _ in pts]
        rises = [(l0, round(p0, 2), l1, round(p1, 2)) for (l0, p0), (l1, p1) in zip(ps, ps[1:]) if p1 - p0 > 0.3]
        R.note(f"{arm}: {fmt(pts)}" + (f"; nonmonotonic rises {rises}" if rises else ""))
    ctg = sum(cls.get("self_ghost", 0) for _, _, _, cls in curve(cells, LT, a.speed, 1, "contract"))
    R.check(ctg == 0, "contract has no self-ghost failure at the left turn", f"{ctg} self-ghost runs")

    R.head("Fig. 13 beacon faults (expected: contract at or above naive and ETSI under every fault; severity worsens outcomes)")
    for p in a.faults:
        if not os.path.exists(p): R.check(False, f"{p} present", "missing"); continue
        rows = list(csv.DictReader(open(p))); name = os.path.basename(p)
        by = collections.defaultdict(dict)
        for r in rows: by[(r["fault"], float(r["value"]))][r["arm"]] = (int(r["k"]), int(r["n"]))
        worse = [f"{f} {v}: contract {d['contract']} naive {d.get('naive')} etsi {d.get('etsi')}" for (f, v), d in sorted(by.items()) if "contract" in d and any(d.get(x, (0, 1))[0] > d["contract"][0] for x in ("naive", "etsi"))]
        R.check(not worse, f"{name}: contract at or above naive and ETSI in every fault cell", "; ".join(worse) if worse else "holds in every cell")
        for arm in ("contract", "contract+valid", "naive", "etsi"):
            for fault in sorted({f for f, _ in by}):
                seq = [(v, d[arm][0] / d[arm][1]) for (f, v), d in sorted(by.items()) if f == fault and arm in d]
                rises = [(v0, v1) for (v0, p0), (v1, p1) in zip(seq, seq[1:]) if p1 - p0 > 0.3]
                if rises: R.note(f"{name} {arm} {fault}: success rises with severity at {rises}")

    R.head("Fig. 16 controller sensitivity (expected: the second controller moves the 0.8 boundary)")
    for geom in (LT, SCP):
        for sp in (a.speed, 88):
            rb, rc = budget(curve(cells, geom, sp, 1, "reference")); cb, cc = budget(curve(cells, geom, sp, 1, "ctrl"))
            if cb is None: R.note(f"{GN[geom]} {sp}: no controller cells"); continue
            R.check(not cc and rb is not None and cb != rb, f"{GN[geom]} {sp} km/h: controller boundary observed and different from the reference", f"reference {rb}{' censored' if rc else ''}, second controller {cb}{' censored' if cc else ''}")

    R.head("Fig. 17 failure taxonomy (expected: before the reference boundary naive fails only by self-ghost and the contract not at all; beyond it, collisions and late completion)")
    for geom in (LT, SCP):
        ref = curve(cells, geom, a.speed, 1, "reference"); rb, _ = budget(ref)
        if rb is None: continue
        for arm in ("naive", "contract"):
            before = [(lat, cls) for lat, k, n, cls in curve(cells, geom, a.speed, 1, arm) if lat <= rb and cls]
            other = [(lat, dict(cls)) for lat, cls in before if set(cls) - {"self_ghost"}]
            if arm == "naive": R.check(not other, f"{GN[geom]}: naive failures before the reference boundary ({rb} ms) are self-ghosts only", f"other classes {other}" if other else "self-ghost only" if before else "no failures")
            else: R.check(not before, f"{GN[geom]}: contract has no failure before the reference boundary ({rb} ms)", f"{[(l, dict(c)) for l, c in before]}" if before else "none")

    text = "# Shape check against the v14 expectations\n" + "\n".join(R.lines) + f"\n\n{R.fails} expectation(s) failing.\n"
    open(a.out, "w").write(text); print(text)


if __name__ == "__main__":
    main()
