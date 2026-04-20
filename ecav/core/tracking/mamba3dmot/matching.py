import cv2
import numpy as np
import scipy
import lap
from scipy.spatial.distance import cdist
import math
import time

try:
    from cython_bbox import bbox_overlaps as bbox_ious
except ImportError:
    bbox_ious = None  # 3D mode doesn't need 2D IoU

chi2inv95 = {
    1: 3.8415,
    2: 5.9915,
    3: 7.8147,
    4: 9.4877,
    5: 11.070,
    6: 12.592,
    7: 14.067,
    8: 15.507,
    9: 16.919}


def merge_matches(m1, m2, shape):
    O,P,Q = shape
    m1 = np.asarray(m1)
    m2 = np.asarray(m2)

    M1 = scipy.sparse.coo_matrix((np.ones(len(m1)), (m1[:, 0], m1[:, 1])), shape=(O, P))
    M2 = scipy.sparse.coo_matrix((np.ones(len(m2)), (m2[:, 0], m2[:, 1])), shape=(P, Q))

    mask = M1*M2
    match = mask.nonzero()
    match = list(zip(match[0], match[1]))
    unmatched_O = tuple(set(range(O)) - set([i for i, j in match]))
    unmatched_Q = tuple(set(range(Q)) - set([j for i, j in match]))

    return match, unmatched_O, unmatched_Q


def _indices_to_matches(cost_matrix, indices, thresh):
    matched_cost = cost_matrix[tuple(zip(*indices))]
    matched_mask = (matched_cost <= thresh)

    matches = indices[matched_mask]
    unmatched_a = tuple(set(range(cost_matrix.shape[0])) - set(matches[:, 0]))
    unmatched_b = tuple(set(range(cost_matrix.shape[1])) - set(matches[:, 1]))

    return matches, unmatched_a, unmatched_b


def linear_assignment(cost_matrix, thresh):
    if cost_matrix.size == 0:
        return np.empty((0, 2), dtype=int), tuple(range(cost_matrix.shape[0])), tuple(range(cost_matrix.shape[1]))
    matches, unmatched_a, unmatched_b = [], [], []
    cost, x, y = lap.lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)
    for ix, mx in enumerate(x):
        if mx >= 0:
            matches.append([ix, mx])
    unmatched_a = np.where(x < 0)[0]
    unmatched_b = np.where(y < 0)[0]
    matches = np.asarray(matches)
    return matches, unmatched_a, unmatched_b


def ious(atlbrs, btlbrs):
    """
    Compute cost based on IoU
    :type atlbrs: list[tlbr] | np.ndarray
    :type atlbrs: list[tlbr] | np.ndarray

    :rtype ious np.ndarray
    """
    ious = np.zeros((len(atlbrs), len(btlbrs)), dtype=np.float64)
    if ious.size == 0:
        return ious

    ious = bbox_ious(
        np.ascontiguousarray(atlbrs, dtype=np.float64),
        np.ascontiguousarray(btlbrs, dtype=np.float64)
    )

    return ious


def iou_distance(atracks, btracks):
    """
    Compute cost based on IoU
    :type atracks: list[STrack]
    :type btracks: list[STrack]

    :rtype cost_matrix np.ndarray
    """

    if (len(atracks)>0 and isinstance(atracks[0], np.ndarray)) or (len(btracks) > 0 and isinstance(btracks[0], np.ndarray)):
        atlbrs = atracks
        btlbrs = btracks
    else:
        atlbrs = [track.tlbr for track in atracks]
        btlbrs = [track.tlbr for track in btracks]
    _ious = ious(atlbrs, btlbrs)
    cost_matrix = 1 - _ious

    return cost_matrix

def _extract_bev_box(track):
    """Extract [x, y, l, w, yaw] from a tracklet, using predicted position if available."""
    if hasattr(track, 'predicted_last_bbox') and track.predicted_last_bbox is not None:
        s = track.predicted_last_bbox
    elif hasattr(track, '_bbox_3d'):
        s = track._bbox_3d
    else:
        s = track
    return s[0], s[1], s[3], s[4], s[6]  # x, y, l, w, yaw


def _rotated_rect_corners(x, y, l, w, yaw):
    """Compute 4 BEV corners of a rotated rectangle."""
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)
    hl, hw = l / 2, w / 2
    return np.array([
        [x + hl * cos_a - hw * sin_a, y + hl * sin_a + hw * cos_a],
        [x - hl * cos_a - hw * sin_a, y - hl * sin_a + hw * cos_a],
        [x - hl * cos_a + hw * sin_a, y - hl * sin_a - hw * cos_a],
        [x + hl * cos_a + hw * sin_a, y + hl * sin_a - hw * cos_a],
    ])


def iou_distance_3d(atracks, btracks):
    """
    Compute BEV rotated IoU cost matrix between 3D tracklets.

    Uses center distance as a coarse gate (20m), then computes rotated
    BEV IoU only for close pairs. Returns 1 - IoU as cost.

    :type atracks: list[MambaTracklet3D]
    :type btracks: list[MambaTracklet3D]
    :rtype cost_matrix np.ndarray
    """
    from shapely.geometry import Polygon

    COARSE_GATE = 20.0  # skip IoU for pairs beyond this distance

    na, nb = len(atracks), len(btracks)
    if na == 0 or nb == 0:
        return np.empty((na, nb), dtype=np.float32)

    # Extract BEV boxes
    a_boxes = [_extract_bev_box(t) for t in atracks]
    b_boxes = [_extract_bev_box(t) for t in btracks]

    # Coarse gate: center distances
    a_centers = np.array([[b[0], b[1]] for b in a_boxes])
    b_centers = np.array([[b[0], b[1]] for b in b_boxes])
    center_dists = cdist(a_centers, b_centers, metric='euclidean')

    # Precompute Shapely polygons
    a_polys = [None] * na
    b_polys = [None] * nb

    cost_matrix = np.ones((na, nb), dtype=np.float32)  # default: no overlap

    for i in range(na):
        for j in range(nb):
            if center_dists[i, j] > COARSE_GATE:
                continue

            # Lazy polygon construction
            if a_polys[i] is None:
                a_polys[i] = Polygon(_rotated_rect_corners(*a_boxes[i]))
            if b_polys[j] is None:
                b_polys[j] = Polygon(_rotated_rect_corners(*b_boxes[j]))

            pa, pb = a_polys[i], b_polys[j]
            if not pa.is_valid or not pb.is_valid:
                continue

            inter = pa.intersection(pb).area
            union = pa.area + pb.area - inter
            if union > 0:
                cost_matrix[i, j] = 1.0 - inter / union

    return cost_matrix


def center_distance_3d(atracks, btracks):
    """
    Compute 3D center distance cost matrix.
    For 3D tracks, uses Euclidean distance between bbox centers.
    Normalized to [0, 1] range using a max distance threshold.

    :type atracks: list[MambaTracklet3D]
    :type btracks: list[MambaTracklet3D] or list[np.ndarray]
    :rtype cost_matrix np.ndarray
    """
    MAX_DIST = 20.0  # meters, pairs beyond this get cost=1.0

    if len(atracks) == 0 or len(btracks) == 0:
        return np.empty((len(atracks), len(btracks)), dtype=np.float32)

    # Extract centers: for 3D boxes, center is [x, y, z]
    if hasattr(atracks[0], '_bbox_3d'):
        a_centers = np.array([t._bbox_3d[:3] for t in atracks])
    elif hasattr(atracks[0], 'predicted_last_bbox') and atracks[0].predicted_last_bbox is not None:
        a_centers = np.array([t.predicted_last_bbox[:3] if t.predicted_last_bbox is not None
                              else t._bbox_3d[:3] for t in atracks])
    else:
        a_centers = np.array([t[:3] if isinstance(t, np.ndarray) else [0, 0, 0] for t in atracks])

    if hasattr(btracks[0], '_bbox_3d'):
        b_centers = np.array([t._bbox_3d[:3] for t in btracks])
    else:
        b_centers = np.array([t[:3] if isinstance(t, np.ndarray) else [0, 0, 0] for t in btracks])

    # Vectorized distance computation
    dists = cdist(a_centers, b_centers, metric='euclidean')
    cost_matrix = np.clip(dists / MAX_DIST, 0, 1)

    return cost_matrix.astype(np.float32)


def v_iou_distance(atracks, btracks):
    """
    Compute cost based on IoU
    :type atracks: list[STrack]
    :type btracks: list[STrack]

    :rtype cost_matrix np.ndarray
    """

    if (len(atracks)>0 and isinstance(atracks[0], np.ndarray)) or (len(btracks) > 0 and isinstance(btracks[0], np.ndarray)):
        atlbrs = atracks
        btlbrs = btracks
    else:
        atlbrs = [track.tlwh_to_tlbr(track.pred_bbox) for track in atracks]
        btlbrs = [track.tlwh_to_tlbr(track.pred_bbox) for track in btracks]
    _ious = ious(atlbrs, btlbrs)
    cost_matrix = 1 - _ious

    return cost_matrix

def embedding_distance(tracks, detections, metric='cosine'):
    """
    :param tracks: list[STrack]
    :param detections: list[BaseTrack]
    :param metric:
    :return: cost_matrix np.ndarray
    """

    cost_matrix = np.zeros((len(tracks), len(detections)), dtype=np.float64)
    if cost_matrix.size == 0:
        return cost_matrix
    det_features = np.asarray([track.curr_feat for track in detections], dtype=np.float64)
    #for i, track in enumerate(tracks):
        #cost_matrix[i, :] = np.maximum(0.0, cdist(track.smooth_feat.reshape(1,-1), det_features, metric))
    track_features = np.asarray([track.smooth_feat for track in tracks], dtype=np.float64)
    cost_matrix = np.maximum(0.0, cdist(track_features, det_features, metric))  # Nomalized features
    return cost_matrix


def gate_cost_matrix(kf, cost_matrix, tracks, detections, only_position=False):
    if cost_matrix.size == 0:
        return cost_matrix
    gating_dim = 2 if only_position else 4
    gating_threshold = chi2inv95[gating_dim]
    measurements = np.asarray([det.to_xyah() for det in detections])
    for row, track in enumerate(tracks):
        gating_distance = kf.gating_distance(
            track.mean, track.covariance, measurements, only_position)
        cost_matrix[row, gating_distance > gating_threshold] = np.inf
    return cost_matrix


def fuse_motion(kf, cost_matrix, tracks, detections, only_position=False, lambda_=0.98):
    if cost_matrix.size == 0:
        return cost_matrix
    gating_dim = 2 if only_position else 4
    gating_threshold = chi2inv95[gating_dim]
    measurements = np.asarray([det.to_xyah() for det in detections])
    for row, track in enumerate(tracks):
        gating_distance = kf.gating_distance(
            track.mean, track.covariance, measurements, only_position, metric='maha')
        cost_matrix[row, gating_distance > gating_threshold] = np.inf
        cost_matrix[row] = lambda_ * cost_matrix[row] + (1 - lambda_) * gating_distance
    return cost_matrix


def fuse_iou(cost_matrix, tracks, detections):
    if cost_matrix.size == 0:
        return cost_matrix
    reid_sim = 1 - cost_matrix
    iou_dist = iou_distance(tracks, detections)
    iou_sim = 1 - iou_dist
    fuse_sim = reid_sim * (1 + iou_sim) / 2
    det_scores = np.array([det.score for det in detections])
    det_scores = np.expand_dims(det_scores, axis=0).repeat(cost_matrix.shape[0], axis=0)
    #fuse_sim = fuse_sim * (1 + det_scores) / 2
    fuse_cost = 1 - fuse_sim
    return fuse_cost


def fuse_score(cost_matrix, detections):
    if cost_matrix.size == 0:
        return cost_matrix
    iou_sim = 1 - cost_matrix
    det_scores = np.array([det.score for det in detections])
    det_scores = np.expand_dims(det_scores, axis=0).repeat(cost_matrix.shape[0], axis=0)
    fuse_sim = iou_sim * det_scores
    fuse_cost = 1 - fuse_sim
    return fuse_cost


def greedy_assignment_iou(dist, thresh):
        matched_indices = []
        if dist.shape[1] == 0:
            return np.array(matched_indices, np.int32).reshape(-1, 2)
        for i in range(dist.shape[0]):
            j = dist[i].argmin()
            if dist[i][j] < thresh:
                dist[:, j] = 1.
                matched_indices.append([j, i])
        return np.array(matched_indices, np.int32).reshape(-1, 2)
    
def greedy_assignment(dists, threshs):
    matches = greedy_assignment_iou(dists.T, threshs)
    u_det = [d for d in range(dists.shape[1]) if not (d in matches[:, 1])]
    u_track = [d for d in range(dists.shape[0]) if not (d in matches[:, 0])]
    return matches, u_track,  u_det

def fuse_score_matrix(cost_matrix, detections, tracks):
    if cost_matrix.size == 0:
        return cost_matrix
    iou_sim = 1 - cost_matrix
    
    det_scores = np.array([det.score for det in detections])
    det_scores = np.expand_dims(det_scores, axis=0).repeat(cost_matrix.shape[0], axis=0)
    trk_scores = np.array([trk.score for trk in tracks])
    trk_scores = np.expand_dims(trk_scores, axis=1).repeat(cost_matrix.shape[1], axis=1)
    mid_scores = (det_scores + trk_scores) / 2
    fuse_sim = iou_sim * mid_scores
    fuse_cost = 1 - fuse_sim
    
    return fuse_cost

def BIoU_distance(atracks, btracks, sigma = 0.4):
    """
    Compute cost based on IoU
    :type atracks: list[STrack]
    :type btracks: list[STrack]

    :rtype cost_matrix np.ndarray
    """
    atlbrs, btlbrs = [], []
    for trk in atracks:
        x1,y1,w,h = trk.tlwh
        delta_h, delta_w = h * sigma, w * sigma
        x1_ = x1 - delta_w
        y1_ = y1 - delta_h
        x2_ = x1 + w + delta_w
        y2_ = y1 + h + delta_h
        bbox_new = np.array([x1_, y1_, x2_, y2_], dtype=np.float64)
        atlbrs.append(bbox_new)
        
    for trk in btracks:
        x1,y1,w,h = trk.tlwh
        delta_h, delta_w = h * sigma, w * sigma
        x1_ = x1 - delta_w
        y1_ = y1 - delta_h
        x2_ = x1 + w + delta_w
        y2_ = y1 + h + delta_h
        bbox_new = np.array([x1_, y1_, x2_, y2_], dtype=np.float64)
        btlbrs.append(bbox_new)

    _ious = ious(atlbrs, btlbrs)
    cost_matrix = 1 - _ious

    return cost_matrix