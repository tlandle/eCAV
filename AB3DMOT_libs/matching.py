import logging
import numpy as np
from scipy.optimize import linear_sum_assignment
from AB3DMOT_libs.dist_metrics import iou, dist3d, dist_ground, m_distance

logger = logging.getLogger(__name__)

SIZE_RATIO_TH = 1.30       # >30 % length mismatch ⇒ never match
COST_MAX      = 1e3        # large positive cost (for −distance metrics)
FRAME_IDX = 0          # simulation step or frame number
GUID      = 1          # globally-unique ID supplied by beacon / vehicle
CID       = 2          # carla_id (server-side vehicle actor id), –1 if unknown

# ── Kinematic innovation gate ──────────────────────────────────────────
# Prevents implausible associations (e.g. camera-handoff 2m jumps on
# stationary tracks) by bounding the allowed innovation as a function
# of the track's current ground-plane speed.
#
# Gate:  d  <  max(trk_speed * DT + D_BASE, D_MIN)
#
# Only applied to mature tracks (hits >= KINEMATIC_MIN_HITS) where the
# KF velocity estimate is reliable.  For young tracks the large P[7:]
# uncertainty makes the velocity unreliable, so we rely on the distance
# threshold alone.
#
# trk_speed arrives as the KF state velocity, which is per tracker frame
# (one frame per source tick), so it is converted to m/s with _DT before
# the comparison. Until 2026-10-14 the per-frame value was compared
# directly against the m/s constants: every track below 10 m/s counted
# as stationary and got the 1 m gate, and the speed term contributed
# at most 0.03 m, so a 13 m/s crossing vehicle was gated at 1.0-1.5 m.
#
# D_BASE covers the disagreement between object-level detections of the
# same vehicle from different viewpoints. Roadside camera-lidar fusion
# places a box at the nearest-surface centroid, so two RSUs that see a
# crossing car end-on report its front and rear faces: 3.3 m apart on
# average, 4.5 m worst case, in the SenSys scenario_3 logs
# (docs/kb/wiki/current_state.md, 2026-10-14). The old 1 m allowance
# rejected the measurement whenever the surviving detection switched
# source, so the track coasted and died and the vehicle was re-born as
# a new id. 3 m is the smallest allowance that keeps a 13 m/s crossing
# vehicle on one track through an RSU handover in the offline replay
# (scripts/diag/replay_edge_tracker.py). The association threshold
# (4 m for dist_2d) still bounds the innovation above this.
_DT                  = 0.05   # tick duration (seconds)
_D_BASE              = 3.0    # cross-view measurement allowance (m)
_D_MIN               = 1.5    # minimum gate size (m)
_KINEMATIC_MIN_HITS  = 3      # min tracker updates before gate activates
_V_STATIC            = 0.5    # below this = "stationary" track (m/s)
_D_STATIC            = 1.0    # max innovation for stationary tracks (m)

def compute_affinity(dets, trks, metric, trk_inv_inn_matrices=None,
                     anchoring=True, anchoring_epoch=40):
	# compute affinity matrix

	aff_matrix = np.zeros((len(dets), len(trks)), dtype=np.float32)
	for d, det in enumerate(dets):
		for t, trk in enumerate(trks):

			# choose to use different distance metrics
			if 'iou' in metric:    	  dist_now = iou(det, trk, metric)
			elif metric == 'm_dis':   dist_now = -m_distance(det, trk, trk_inv_inn_matrices[t])
			elif metric == 'euler':   dist_now = -m_distance(det, trk, None)
			elif metric == 'dist_2d': dist_now = -dist_ground(det, trk)
			elif metric == 'dist_3d': dist_now = -dist3d(det, trk)
			else: assert False, 'error'

			#  ─── identity-aware association ──────────────────────────────────────
			det_cid = det.info[CID] if hasattr(det, "info") else -1
			trk_cid = trk.carla_id

			if det_cid != -1 and trk_cid != -1:
			    if det_cid != trk_cid:
			        dist_now = COST_MAX      # forbid: beacon ID disagrees with track ID
			    elif anchoring:
			        # Only force the identity match while the track's
			        # anchoring epoch has not expired.  After anchoring_epoch
			        # ticks without a beacon-matched update the constraint
			        # lapses and we fall through to geometry-only scoring.
			        trk_age = getattr(trk, "anchoring_age", 0)
			        if trk_age < anchoring_epoch:
			            dist_now = -0.01     # force match: beacon ID agrees & epoch valid
			        # else: keep geometry-only dist_now
			#  ────────────────────────────────────────────────────────────────────
			aff_matrix[d, t] = dist_now

	return aff_matrix

def greedy_matching(cost_matrix):
    # association in the greedy manner
    # refer to https://github.com/eddyhkchiu/mahalanobis_3d_multi_object_tracking/blob/master/main.py

    num_dets, num_trks = cost_matrix.shape[0], cost_matrix.shape[1]

    # sort all costs and then convert to 2D
    distance_1d = cost_matrix.reshape(-1)
    index_1d = np.argsort(distance_1d)
    index_2d = np.stack([index_1d // num_trks, index_1d % num_trks], axis=1)

    # assign matches one by one given the sorting, but first come first serves
    det_matches_to_trk = [-1] * num_dets
    trk_matches_to_det = [-1] * num_trks
    matched_indices = []
    for sort_i in range(index_2d.shape[0]):
        det_id = int(index_2d[sort_i][0])
        trk_id = int(index_2d[sort_i][1])

        # if both id has not been matched yet
        if trk_matches_to_det[trk_id] == -1 and det_matches_to_trk[det_id] == -1:
            trk_matches_to_det[trk_id] = det_id
            det_matches_to_trk[det_id] = trk_id
            matched_indices.append([det_id, trk_id])

    return np.asarray(matched_indices)

def data_association(dets, trks, metric, threshold, algm='greedy', \
	trk_innovation_matrix=None, hypothesis=1, anchoring=True, anchoring_epoch=40):
	"""
	Assigns detections to tracked object

	dets:  a list of Box3D object
	trks:  a list of Box3D object

	Returns 3 lists of matches, unmatched_dets and unmatched_trks, and total cost, and affinity matrix
	"""

	# if there is no item in either row/col, skip the association and return all as unmatched
	aff_matrix = np.zeros((len(dets), len(trks)), dtype=np.float32)
	if len(trks) == 0:
		return np.empty((0, 2), dtype=int), np.arange(len(dets)), [], 0, aff_matrix
	if len(dets) == 0:
		return np.empty((0, 2), dtype=int), [], np.arange(len(trks)), 0, aff_matrix

	# prepare inverse innovation matrix for m_dis
	if metric == 'm_dis':
		assert trk_innovation_matrix is not None, 'error'
		trk_inv_inn_matrices = [np.linalg.inv(m) for m in trk_innovation_matrix]
	else:
		trk_inv_inn_matrices = None

	# compute affinity matrix
	aff_matrix = compute_affinity(dets, trks, metric, trk_inv_inn_matrices,
	                              anchoring=anchoring, anchoring_epoch=anchoring_epoch)

	# association based on the affinity matrix
	if hypothesis == 1:
		if algm == 'hungar':
			row_ind, col_ind = linear_sum_assignment(-aff_matrix)      	# hougarian algorithm
			matched_indices = np.stack((row_ind, col_ind), axis=1)
		elif algm == 'greedy':
			matched_indices = greedy_matching(-aff_matrix) 				# greedy matching
		else: assert False, 'error'
	else:
		cost_list, hun_list = best_k_matching(-aff_matrix, hypothesis)

	# compute total cost
	cost = 0
	for row_index in range(matched_indices.shape[0]):
		cost -= aff_matrix[matched_indices[row_index, 0], matched_indices[row_index, 1]]

	# save for unmatched objects
	unmatched_dets = []
	for d, det in enumerate(dets):
		if (d not in matched_indices[:, 0]): unmatched_dets.append(d)
	unmatched_trks = []
	for t, trk in enumerate(trks):
		if (t not in matched_indices[:, 1]): unmatched_trks.append(t)

	# filter out matches with low affinity + kinematic innovation gate
	matches = []
	for m in matched_indices:
		if (aff_matrix[m[0], m[1]] < threshold):
			unmatched_dets.append(m[0])
			unmatched_trks.append(m[1])
			continue

		# ── Kinematic innovation gate ──────────────────────────────
		# For mature tracks, reject associations that would require
		# an implausible velocity jump.  The max allowed innovation
		# scales with the track's current speed: fast tracks get a
		# wider gate, stationary tracks get a tighter one.
		#
		# This kills the "parked car track hijacked by camera-handoff
		# detection" pathology where a 2m position jump on a
		# stationary track implies 40 m/s velocity (800 m/s² accel).
		trk_speed = getattr(trks[m[1]], 'kf_speed_ground', None)
		trk_hits  = getattr(trks[m[1]], 'hits', 0)
		if trk_speed is not None:
			trk_speed = trk_speed / _DT       # KF velocity is per frame; gate is in m/s

		if (trk_speed is not None and trk_hits >= _KINEMATIC_MIN_HITS
				and metric in ('dist_3d', 'dist_2d', 'm_dis', 'euler')):
			if metric in ('m_dis', 'euler') and aff_matrix[m[0], m[1]] < -0.01:
				# Mahalanobis affinity is dimensionless; gate on the
				# ground-plane distance in metres like the dist_* metrics.
				# A beacon-forced identity match keeps its -0.01 affinity
				# under every metric and so passes the gate, as for dist_*.
				d = dist_ground(dets[m[0]], trks[m[1]])
			else:
				d = -aff_matrix[m[0], m[1]]       # positive distance
			max_innov = max(trk_speed * _DT + _D_BASE, _D_MIN)

			if d > max_innov:
				logger.debug(
					"[KINEMATIC GATE] rejected match det=%d trk=%d "
					"d=%.2fm > max_innov=%.2fm (trk_speed=%.1f m/s, "
					"hits=%d, cid=%s)",
					m[0], m[1], d, max_innov, trk_speed, trk_hits,
					getattr(trks[m[1]], 'carla_id', '?'))
				unmatched_dets.append(m[0])
				unmatched_trks.append(m[1])
				continue

			# Extra: stationary tracks get a tighter absolute gate
			if trk_speed < _V_STATIC and d > _D_STATIC:
				logger.debug(
					"[STATIC GATE] rejected match det=%d trk=%d "
					"d=%.2fm > D_STATIC=%.2fm (trk_speed=%.2f m/s)",
					m[0], m[1], d, _D_STATIC, trk_speed)
				unmatched_dets.append(m[0])
				unmatched_trks.append(m[1])
				continue
		# ──────────────────────────────────────────────────────────

		matches.append(m.reshape(1, 2))

	if len(matches) == 0:
		matches = np.empty((0, 2),dtype=int)
	else: matches = np.concatenate(matches, axis=0)

	return matches, np.array(unmatched_dets), np.array(unmatched_trks), cost, aff_matrix
