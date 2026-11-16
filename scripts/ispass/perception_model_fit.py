#!/usr/bin/env python
"""Fit the perception error models to a detection trace of the full sensor pipeline.

    python scripts/ispass/perception_model_fit.py <stamp> [<stamp> ...] --out paper_ispass/perception_model.json
                                                  [--curves paper_ispass/perception_curves.json]

Input: the [PERC-TRACE] lines (PERC_TRACE=1, perception_manager._perception_trace) in the
runner_stdout.txt of every run under experiment_results/*/<stamp>*. One line per simulator vehicle
within the LiDAR range per detection call: distance, unblocked rays of five (vis), detected or not
(a detection within 3 m of the centre), radial and tangential position error. actor=-1 lines are
detections that match no vehicle.

Output (--out), read by ecav/core/sensing/perception/perception_error_model.py:
  calibrated table   detection probability by distance bin and visibility bin, separately for the
                     first call in view, after a detection, and after a miss; radial bias, radial
                     and tangential spread by distance bin; lag-1 correlation of the radial error
  independent        the marginal miss probability and position spread of the same trace, for the
                     model with independent misses and white noise
A cell with fewer than MIN samples takes the value of its distance bin, then of the whole trace.

--curves: the curves of the trace itself: recall by distance, recall by visibility, the lengths of
the runs of consecutive misses in view, the calls from first view to first detection, unmatched
detections per call.
"""
import argparse, collections, glob, json, math, os, re, sys
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PAT = re.compile(r"\[PERC-TRACE\] call=(\d+) ego=(\S+) actor=(-?\d+) d=([\d.]+) vis=(\d) det=(\d) er=(\S+) et=(\S+) v=(\S+)")
DIST = [0, 10, 20, 30, 40, 50]; VIS = [2, 4, 5]; MIN = 30


def dbin(d): return min(int(d // 10), len(DIST) - 2)
def vbin(v): return next(i for i, hi in enumerate(VIS) if v <= hi)


def load(stamps):
    """Per run: {(ego, actor): [(call, d, vis, det, er, et)]} and the unmatched detections per call."""
    runs = []
    for st in stamps:
        for f in sorted(glob.glob(os.path.join(ROOT, "experiment_results", "*", st + "*", "**", "runner_stdout.txt"), recursive=True)):
            seq = collections.defaultdict(list); fp = collections.Counter(); calls = set()
            for line in open(f, errors="replace"):
                m = PAT.search(line)
                if not m: continue
                call, ego, actor, d, vis, det = int(m.group(1)), m.group(2), int(m.group(3)), float(m.group(4)), int(m.group(5)), int(m.group(6))
                calls.add((ego, call))
                if actor < 0: fp[(ego, call)] += 1; continue
                er = float(m.group(7)); et = float(m.group(8))
                seq[(ego, actor)].append((call, d, vis, det, er, et))
            if seq: runs.append((f, seq, fp, len(calls)))
    return runs


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stamps", nargs="+"); ap.add_argument("--out", required=True); ap.add_argument("--curves")
    a = ap.parse_args(); runs = load(a.stamps)
    if not runs: sys.exit("no [PERC-TRACE] lines under the given stamps")
    nd, nv = len(DIST) - 1, len(VIS)
    cnt = {k: np.zeros((nd, nv, 2)) for k in ("first", "dd", "dm")}       # [..., 0] trials, [..., 1] detections
    er_by = [[] for _ in range(nd)]; et_by = [[] for _ in range(nd)]; pairs = []
    gaps = []; first_delay = []; rec_d = np.zeros((nd, 2)); rec_v = np.zeros((6, 2)); n_calls = 0; n_fp = 0
    for f, seq, fp, calls in runs:
        n_calls += calls; n_fp += sum(fp.values())
        for key, rows in seq.items():
            rows.sort(); prev = None; gap = 0; since_view = None
            for call, d, vis, det, er, et in rows:
                rec_v[vis] += (1, det)
                if vis == 0:            # in range, not in line of sight: outside the model
                    prev = None; continue
                i, j = dbin(d), vbin(vis); rec_d[i] += (1, det)
                cont = prev is not None and prev[0] == call - 1
                if not cont:
                    if gap: gaps.append(gap)
                    gap = 0; since_view = 0; cnt["first"][i, j] += (1, det)
                else:
                    cnt["dd" if prev[1] else "dm"][i, j] += (1, det)
                if det:
                    if gap: gaps.append(gap)
                    gap = 0
                    if since_view is not None: first_delay.append(since_view); since_view = None
                    er_by[i].append(er); et_by[i].append(et)
                    if cont and prev[1]: pairs.append((prev[2], er))
                else:
                    gap += 1
                    if since_view is not None: since_view += 1
                prev = (call, det, er)
            if gap: gaps.append(gap)

    def table(c):
        tot = c.sum(axis=(0, 1)); glob_p = tot[1] / max(tot[0], 1)
        out = []
        for i in range(nd):
            row_t = c[i].sum(axis=0); row_p = row_t[1] / row_t[0] if row_t[0] >= MIN else glob_p
            out.append([float(c[i, j, 1] / c[i, j, 0]) if c[i, j, 0] >= MIN else float(row_p) for j in range(nv)])
        return out
    all_r = np.array([x for b in er_by for x in b]); all_t = np.array([x for b in et_by for x in b])
    stat = lambda b, fn, fb: float(fn(np.array(b))) if len(b) >= MIN else float(fb)
    rho = float(np.corrcoef(np.array(pairs).T)[0, 1]) if len(pairs) >= MIN else 0.0
    in_view = rec_d.sum(axis=0)
    model = dict(source=a.stamps, runs=len(runs), samples_in_view=int(in_view[0]),
                 dist_edges=DIST, vis_bins=VIS,
                 p_first=table(cnt["first"]), p_det_given_det=table(cnt["dd"]), p_det_given_miss=table(cnt["dm"]),
                 radial_bias_m=[stat(b, np.mean, all_r.mean()) for b in er_by],
                 radial_sigma_m=[stat(b, np.std, all_r.std()) for b in er_by],
                 tangential_sigma_m=[stat(b, np.std, all_t.std()) for b in et_by],
                 error_correlation=max(0.0, min(0.99, rho)),
                 independent=dict(p_miss=float(1 - in_view[1] / in_view[0]),
                                  pos_sigma_m=float(math.sqrt((np.mean(all_r ** 2) + np.mean(all_t ** 2)) / 2))))
    json.dump(model, open(a.out, "w"), indent=1); print("wrote", a.out)
    print("runs %d, samples in view %d, recall in view %.3f, independent p_miss %.3f sigma %.2f m, error correlation %.2f" % (
        len(runs), in_view[0], in_view[1] / in_view[0], model["independent"]["p_miss"], model["independent"]["pos_sigma_m"], rho))
    print("recall by distance bin:", [round(float(r[1] / r[0]), 3) if r[0] else None for r in rec_d], "samples", [int(r[0]) for r in rec_d])
    print("recall by unblocked rays 0..5:", [round(float(r[1] / r[0]), 3) if r[0] else None for r in rec_v], "samples", [int(r[0]) for r in rec_v])
    print("P(detect | detected before) by distance:", [round(float(np.mean(r)), 3) for r in model["p_det_given_det"]])
    print("P(detect | missed before) by distance:  ", [round(float(np.mean(r)), 3) for r in model["p_det_given_miss"]])
    g = np.array(gaps) if gaps else np.array([0]); fd = np.array(first_delay) if first_delay else np.array([0])
    print("miss runs: n %d, median %.0f, p90 %.0f, max %d calls; calls from first view to first detection: median %.0f, p90 %.0f" % (
        len(gaps), np.median(g), np.percentile(g, 90), g.max(), np.median(fd), np.percentile(fd, 90)))
    print("unmatched detections per call: %.3f" % (n_fp / max(n_calls, 1)))
    if a.curves:
        json.dump(dict(dist_edges=DIST, recall_by_distance=[[int(r[0]), int(r[1])] for r in rec_d],
                       recall_by_rays=[[int(r[0]), int(r[1])] for r in rec_v], miss_runs=[int(x) for x in gaps],
                       first_detection_delay=[int(x) for x in first_delay], unmatched_per_call=n_fp / max(n_calls, 1),
                       calls=n_calls), open(a.curves, "w"))
        print("wrote", a.curves)


if __name__ == "__main__":
    main()
