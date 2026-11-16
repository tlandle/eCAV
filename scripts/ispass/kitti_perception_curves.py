#!/usr/bin/env python
"""Run the platform's camera and LiDAR object pipeline on real recordings (KITTI tracking benchmark).

    python scripts/ispass/kitti_perception_curves.py --root /media/atlas/HDD/Datasets/KITTI_tracking \
           --out paper_ispass/perception_curves_kitti.json [--seqs 0 1 2] [--max-frames N]

Per frame, the steps of ecav/core/sensing/perception (perception_manager.activate_mode and
o3d_lidar_libs.o3d_camera_lidar_fusion_from_tracker) are applied to the recorded image of camera 2 and the recorded
Velodyne sweep, with the recording's calibration in place of the simulator's sensor transforms:
  1. YOLOv5m (the repository's local hub model, the platform's weights) on the image
  2. the 2D SORT tracker with the platform's settings (max_age 5, min_hits 1, IoU 0.3), every class
  3. LiDAR points (within RANGE_M of the sensor) projected into the image; for every 2D track the points inside its
     box, the platform's outlier filter (integer mode of |x| and |y|, plus or minus 3 m), at least 2 points, the mean
     of the points moved 1 m away from the sensor
  4. the platform's deduplication (single linkage at NMS_M, the first detection of a cluster is kept)
A labelled Car, Van or Truck within RANGE_M counts as detected when a detection lies within 3 m of its centre, the rule
of the simulator trace (perception_manager._perception_trace). Output: the curves of
scripts/ispass/perception_model_fit.py --curves, with KITTI's occlusion level (0 fully visible, 1 partly, 2 largely
occluded) in place of the unblocked rays, and times in frames of 0.1 s.

Differences from the simulated vehicle that the comparison has to state: one forward camera of 1242 x 375 pixels and
about 81 degrees against four cameras of 640 x 480; 10 Hz against 20 Hz; a Velodyne HDL-64E (+2 to -24.8 degrees)
against a simulated 64-channel LiDAR (+10 to -30 degrees); only objects inside the camera image are labelled.
"""
import argparse, collections, glob, json, os, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, ROOT)
RANGE_M = 50.0; NMS_M = 5.0; MATCH_M = 3.0; DIST = [0, 10, 20, 30, 40, 50]; TYPES = ("Car", "Van", "Truck")


def read_calib(path):
    c = {}
    for line in open(path):
        k, *v = line.replace(":", " ").split()
        c[k] = np.array([float(x) for x in v])
    P2 = c["P2"].reshape(3, 4)
    R = np.eye(4); R[:3, :3] = c["R_rect"].reshape(3, 3)
    T = np.eye(4); T[:3, :4] = c["Tr_velo_cam"].reshape(3, 4)
    return P2, R, T


def read_labels(path):
    by = collections.defaultdict(list)
    for line in open(path):
        p = line.split()
        if p[2] not in TYPES: continue
        by[int(p[0])].append(dict(track=int(p[1]), trunc=float(p[3]), occ=int(p[4]), h=float(p[10]),
                                  cam=np.array([float(p[13]), float(p[14]), float(p[15])])))
    return by


def fuse(tracks, pts, uvz):
    """Detections (x, y in the LiDAR frame) from 2D tracks and LiDAR points; the platform's fusion."""
    from scipy.stats import mode
    out = []
    for t in tracks:
        x1, y1, x2, y2 = (int(v) for v in t[:4])
        inside = (uvz[:, 0] > x1) & (uvz[:, 0] < x2) & (uvz[:, 1] > y1) & (uvz[:, 1] < y2) & (uvz[:, 2] > 0.0)
        sel = pts[inside][:, :3]
        if sel.shape[0] == 0: continue
        xm = mode(np.array(np.abs(sel[:, 0]), dtype=np.int64), axis=0, keepdims=False).mode
        ym = mode(np.array(np.abs(sel[:, 1]), dtype=np.int64), axis=0, keepdims=False).mode
        xm = xm if np.isscalar(xm) else np.asarray(xm).item(); ym = ym if np.isscalar(ym) else np.asarray(ym).item()
        ok = (np.abs(sel[:, 0]) > xm - 3) & (np.abs(sel[:, 0]) < xm + 3) & (np.abs(sel[:, 1]) > ym - 3) & (np.abs(sel[:, 1]) < ym + 3)
        sel = sel[ok]
        if sel.shape[0] < 2: continue
        c = sel.mean(axis=0); r = float(np.hypot(c[0], c[1]))
        if r > 1e-3: c = c + np.array([c[0] / r, c[1] / r, 0.0])
        out.append((float(c[0]), float(c[1])))
    # deduplication: single linkage at NMS_M, keep the first of each cluster
    n = len(out); cid = list(range(n))
    def find(i):
        while cid[i] != i: cid[i] = cid[cid[i]]; i = cid[i]
        return i
    for i in range(n):
        for j in range(i + 1, n):
            if np.hypot(out[i][0] - out[j][0], out[i][1] - out[j][1]) < NMS_M:
                a, b = find(i), find(j)
                if a != b: cid[b] = a
    seen, keep = set(), []
    for i in range(n):
        r = find(i)
        if r not in seen: seen.add(r); keep.append(out[i])
    return keep


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--root", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--seqs", type=int, nargs="*"); ap.add_argument("--max-frames", type=int, default=None)
    a = ap.parse_args()
    import torch, cv2
    from ecav.core.sensing.tracking.sort_tracker import MultiObjectSORTTracker
    os.chdir(ROOT)
    model = torch.hub.load("yolov5/", model="yolov5m", source="local"); model.eval()
    tr = os.path.join(a.root, "training")
    seqs = a.seqs if a.seqs else sorted(int(os.path.basename(p)) for p in glob.glob(os.path.join(tr, "image_02", "*")))
    rec_d = np.zeros((len(DIST) - 1, 2)); rec_o = np.zeros((3, 2)); gaps = []; first_delay = []; n_frames = 0; n_fp = 0; n_det = 0
    err_r = []; per_seq = {}
    for s in seqs:
        P2, R, T = read_calib(os.path.join(tr, "calib", "%04d.txt" % s)); labels = read_labels(os.path.join(tr, "label_02", "%04d.txt" % s))
        cam_to_velo = np.linalg.inv(R @ T); proj = P2 @ R @ T
        frames = sorted(glob.glob(os.path.join(tr, "image_02", "%04d" % s, "*.png")))
        if a.max_frames: frames = frames[:a.max_frames]
        tracker = MultiObjectSORTTracker(max_age=5, min_matching_iou=0.3)
        state = {}      # track id -> [last frame in view, current miss run, frames since first view or None]
        s_tot = s_det = 0
        for fp in frames:
            k = int(os.path.splitext(os.path.basename(fp))[0])
            vp = os.path.join(tr, "velodyne", "%04d" % s, "%06d.bin" % k)
            if not os.path.exists(vp): continue
            img = cv2.cvtColor(cv2.imread(fp), cv2.COLOR_BGR2RGB)
            pts = np.fromfile(vp, dtype=np.float32).reshape(-1, 4)
            pts = pts[np.linalg.norm(pts[:, :3], axis=1) <= RANGE_M]
            h = np.c_[pts[:, :3], np.ones(len(pts))] @ proj.T
            uvz = np.c_[h[:, 0] / np.where(h[:, 2] != 0, h[:, 2], 1e-9), h[:, 1] / np.where(h[:, 2] != 0, h[:, 2], 1e-9), h[:, 2]]
            with torch.no_grad(): res = model([img])
            tracks = tracker.track(res.xyxy[0])
            dets = fuse(np.asarray(tracks), pts, uvz); n_frames += 1; n_det += len(dets)
            free = set(range(len(dets))); gts = []
            for lab in labels.get(k, []):
                c = lab["cam"].copy(); c[1] -= lab["h"] / 2.0          # box centre, not its bottom
                v = cam_to_velo @ np.r_[c, 1.0]; d = float(np.hypot(v[0], v[1]))
                if d < RANGE_M: gts.append((d, lab, v[:2]))
            for d, lab, xy in sorted(gts, key=lambda g: g[0]):
                best, bd = None, MATCH_M
                for i in free:
                    dd = float(np.hypot(dets[i][0] - xy[0], dets[i][1] - xy[1]))
                    if dd < bd: best, bd = i, dd
                det = int(best is not None)
                if det:
                    free.discard(best); u = xy / (np.linalg.norm(xy) or 1.0); e = np.array(dets[best]) - xy; err_r.append(float(e @ u))
                rec_d[min(int(d // 10), len(DIST) - 2)] += (1, det); s_tot += 1; s_det += det
                if lab["occ"] in (0, 1, 2): rec_o[lab["occ"]] += (1, det)
                st = state.get(lab["track"])
                if st is None or st[0] != k - 1:
                    if st is not None and st[1]: gaps.append(st[1])
                    st = [k, 0, 0]
                st[0] = k
                if det:
                    if st[1]: gaps.append(st[1])
                    st[1] = 0
                    if st[2] is not None: first_delay.append(st[2]); st[2] = None
                else:
                    st[1] += 1
                    if st[2] is not None: st[2] += 1
                state[lab["track"]] = st
            n_fp += len(free)
        for st in state.values():
            if st[1]: gaps.append(st[1])
        per_seq[s] = [s_det, s_tot]
        print("sequence %04d: %d frames, recall %.3f (%d of %d)" % (s, len(frames), s_det / max(s_tot, 1), s_det, s_tot), flush=True)
    tot = rec_d.sum(axis=0)
    print("frames %d, labelled vehicles within %.0f m %d, recall %.3f, detections per frame %.2f, unmatched per frame %.2f" % (
        n_frames, RANGE_M, tot[0], tot[1] / max(tot[0], 1), n_det / max(n_frames, 1), n_fp / max(n_frames, 1)))
    print("recall by distance bin:", [round(float(r[1] / r[0]), 3) if r[0] else None for r in rec_d], "samples", [int(r[0]) for r in rec_d])
    print("recall by occlusion level 0, 1, 2:", [round(float(r[1] / r[0]), 3) if r[0] else None for r in rec_o], "samples", [int(r[0]) for r in rec_o])
    g = np.array(gaps) if gaps else np.array([0]); fd = np.array(first_delay) if first_delay else np.array([0])
    print("miss runs: n %d, median %.0f, p90 %.0f, max %d frames of 0.1 s; frames from first view to first detection: median %.0f, p90 %.0f" % (
        len(gaps), np.median(g), np.percentile(g, 90), g.max(), np.median(fd), np.percentile(fd, 90)))
    print("radial error of matched detections: mean %.2f m, sd %.2f m" % (np.mean(err_r), np.std(err_r)))
    json.dump(dict(source="KITTI tracking, training sequences %s" % seqs, frame_s=0.1, range_m=RANGE_M, dist_edges=DIST,
                   recall_by_distance=[[int(r[0]), int(r[1])] for r in rec_d], recall_by_occlusion=[[int(r[0]), int(r[1])] for r in rec_o],
                   miss_runs=[int(x) for x in gaps], first_detection_delay=[int(x) for x in first_delay],
                   unmatched_per_call=n_fp / max(n_frames, 1), calls=n_frames, per_sequence=per_seq,
                   radial_error_mean_m=float(np.mean(err_r)), radial_error_sd_m=float(np.std(err_r))), open(a.out, "w"))
    print("wrote", a.out)


if __name__ == "__main__":
    main()
