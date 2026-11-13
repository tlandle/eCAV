# Author: Xinshuo Weng
# email: xinshuo.weng@gmail.com

import numpy as np, os, copy, math
from AB3DMOT_libs.box import Box3D
from AB3DMOT_libs.matching import data_association
from AB3DMOT_libs.nms import nms
from AB3DMOT_libs.kalman_filter import KF
from AB3DMOT_libs.vis import vis_obj
#from xinshuo_miscellaneous import print_log
#from xinshuo_io import mkdir_if_missing

np.set_printoptions(suppress=True, precision=3)

FRAME_IDX = 0          # simulation step or frame number
GUID      = 1          # globally-unique ID supplied by beacon / vehicle
CID       = 2          # carla_id (server-side vehicle actor id), –1 if unknown

# A Baseline of 3D Multi-Object Tracking
class AB3DMOT(object):			  	
	def __init__(self, cfg, cat, calib=None, oxts=None, img_dir=None, vis_dir=None, hw=None, log=None, ID_init=0):

		# vis and log purposes
		self.img_dir = img_dir
		self.vis_dir = vis_dir
		self.vis = cfg.vis
		self.hw = hw
		self.log = log

		# counter
		self.trackers = []
		self.frame_count = 0
		# Time index of the current call, in source ticks. It advances by the number of source
		# ticks between consecutive calls (the stride), so it equals frame_count for a tracker
		# fed every tick. The rules below that the comments state in time (half a second of
		# history before a velocity is published, the velocity window, the rest hold) are
		# counted in this index, and the filter predicts over the stride, so they hold for a
		# tracker that is fed every third or fourth tick (service occupancy). min_hits and
		# max_age stay in calls: a track needs that many observations at any frame rate
		# (min_hits 3 and max_age 3 are two and three strides: 100 and 150 ms at one tick per
		# call, about 350 and 525 ms at 3.5 ticks per call).
		# The time index is used when cfg.time_rules is set (the intermediate-fusion manager,
		# whose frames arrive in order at a stride of one tick, or three to four under service
		# occupancy). Without it the index is the call count, as before: the object-level
		# managers ingest frames in arrival order, which under measured compute is not always
		# source-tick order.
		self.time_rules = bool(getattr(cfg, 'time_rules', False))
		self._tidx = 0
		self._stride = 1
		self._prev_tick = None
		self._call_tidx = []
		self.ID_count = [ID_init]
		self.id_now_output = []

		# config
		self.cat = cat
		self.ego_com = cfg.ego_com 			# ego motion compensation
		self.calib = calib
		self.oxts = oxts
		self.affi_process = cfg.affi_pro	# post-processing affinity
		self.anchoring = getattr(cfg, 'anchoring', True)
		self.anchoring_epoch = getattr(cfg, 'anchoring_epoch', 40)  # ticks before forced-match expires (default 40 = 2s at 0.05s/tick)
		self.get_param(cfg, cat)

		# Allow config-level overrides (take precedence over get_param defaults)
		if hasattr(cfg, 'max_age'):
			self.max_age = cfg.max_age
		if hasattr(cfg, 'min_hits'):
			self.min_hits = cfg.min_hits
		# Association gate override (SenSys reviewer A6 baseline sweep). The
		# metric is one of dist_2d, dist_3d, m_dis (Mahalanobis with the KF
		# innovation covariance), euler, iou_3d, giou_3d; the threshold is in
		# the metric's native units (metres for dist_*, Mahalanobis units for
		# m_dis, IoU for iou_*). Unset keys leave the get_param defaults.
		if (getattr(cfg, 'assoc_metric', None) or getattr(cfg, 'assoc_algorithm', None)
				or getattr(cfg, 'assoc_threshold', None) is not None):
			self._apply_assoc_override(cfg)

		# Published-velocity options. The defaults are the junction behaviour
		# (slower measured box edge over half a second, see _edge_velocity).
		# vel_from_edges False publishes the slope of the track's own output
		# positions; vel_window sets the slope window in frames. On a
		# multi-lane road at highway speed the edges of a lidar box seen from
		# behind move by 1 to 2 m between frames and sensors: the slower-edge
		# rule read a 17 m/s car at 11 to 23 m/s and its lateral velocity at
		# zero or at 1.5 m/s of noise (highway merge, 2026-11-11).
		_vfe = getattr(cfg, 'vel_from_edges', True)
		self.vel_from_edges = str(_vfe).strip().lower() not in ('0', 'false', 'no', 'off')
		_vw = getattr(cfg, 'vel_window', None)
		if _vw not in (None, ''):
			self.VEL_WINDOW = int(float(_vw))
			self.VEL_WINDOW_LONG = 2 * self.VEL_WINDOW

		# Identity exclusion zone: anisotropic heading-aligned gate
		# prevents birth of anonymous tracks that are depth-error
		# duplicates of beacon-identified participants.
		#   x_max: along-track (longitudinal) — covers 5-8m depth error
		#   y_max: cross-track (lateral)      — tight to spare adjacent lanes
		self._dup_x_max = getattr(cfg, 'dup_x_max', 8.0)
		# The stale echo of a moving identified vehicle lies BEHIND it along
		# its heading (it is that vehicle's own earlier position), never
		# ahead: the zone reaches dup_x_max back and only dup_x_ahead forward,
		# so a vehicle crossing in front of a stopped CAV is not swallowed.
		# A duplicate also moves with the vehicle it duplicates: the cull
		# needs the anonymous track's speed within dup_v_max of the
		# identified track's (both from the KF state, m/tick over dt).
		self._dup_x_ahead = getattr(cfg, 'dup_x_ahead', 2.5)
		self._dup_v_max = getattr(cfg, 'dup_v_max', 2.0)
		self._dup_dt = getattr(cfg, 'dt', 0.05)
		self._dup_y_max = getattr(cfg, 'dup_y_max', 2.0)
		self._dup_size_ratio = getattr(cfg, 'dup_size_ratio', 2.5)

		# Instrumentation counters
		self.birth_attempts_anon = 0
		self.birth_suppressed_by_gate = 0
		self.births_anon_after_gate = 0
		self.anon_cull_count = 0
		# Anonymous-hold log: frames an anonymous track lived before a beacon
		# bound it to an identity (detection arrived before its beacon).
		self.anon_hold_ticks = []

		# Post-birth cull: consecutive ticks an anon track is near
		# a beacon-identified track before deletion
		self._cull_consec_ticks = getattr(cfg, 'cull_consec_ticks', 3)

		# debug
		# self.debug_id = 2
		self.debug_id = None

	def _apply_assoc_override(self, cfg):
		metric = getattr(cfg, 'assoc_metric', None) or self.metric
		algm = getattr(cfg, 'assoc_algorithm', None) or self.algm
		thres = getattr(cfg, 'assoc_threshold', None)
		dist_like = metric in ['dist_3d', 'dist_2d', 'm_dis', 'euler']
		if thres is None:
			thres = abs(self.thres) if dist_like else self.thres
		if dist_like: thres = -abs(float(thres))
		self.algm, self.metric, self.thres = algm, metric, thres
		if dist_like:                              self.max_sim, self.min_sim = 0.0, -100.
		elif self.metric in ['iou_2d', 'iou_3d']:   self.max_sim, self.min_sim = 1.0, 0.0
		elif self.metric in ['giou_2d', 'giou_3d']: self.max_sim, self.min_sim = 1.0, -1.0

	def get_param(self, cfg, cat):
		# get parameters for each dataset

		if cfg.dataset == 'KITTI':
			if cfg.det_name == 'pvrcnn':				# tuned for PV-RCNN detections
				# Car: BEV center / kinematic association (dist_2d). Object-level
				# cooperative detections from roadside camera-lidar fusion have
				# accurate centers but unstable extents/yaw (axis-aligned box fit
				# to near-face LiDAR points), so a shape-overlap gate (giou_3d)
				# treats viewpoint-induced extent error as association evidence
				# and fragments fast cross-traffic tracks. Center distance plus
				# the kinematic innovation gate matches the reliable part of the
				# measurement. thres=4 -> 4 m gate after sign flip below.
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'hungar', 'dist_2d', 4, 3, 2
				elif cat == 'Pedestrian': 	algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.4, 1, 4 		
				elif cat == 'Cyclist': 		algm, metric, thres, min_hits, max_age = 'hungar', 'dist_3d', 2, 3, 4
				else: assert False, 'error'
			elif cfg.det_name == 'pointrcnn':			# tuned for PointRCNN detections
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'hungar', 'giou_3d', -0.2, 3, 2
				elif cat == 'Pedestrian': 	algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.4, 1, 4 		
				elif cat == 'Cyclist': 		algm, metric, thres, min_hits, max_age = 'hungar', 'dist_3d', 2, 3, 4
				else: assert False, 'error'
			elif cfg.det_name == 'deprecated':			
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'hungar', 'dist_3d', 6, 3, 2
				elif cat == 'Pedestrian': 	algm, metric, thres, min_hits, max_age = 'hungar', 'dist_3d', 1, 3, 2		
				elif cat == 'Cyclist': 		algm, metric, thres, min_hits, max_age = 'hungar', 'dist_3d', 6, 3, 2
				else: assert False, 'error'
			else: assert False, 'error'
		elif cfg.dataset == 'nuScenes':
			if cfg.det_name == 'centerpoint':		# tuned for CenterPoint detections
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.4, 1, 2
				elif cat == 'Pedestrian': 	algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.5, 1, 2
				elif cat == 'Truck': 		algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.4, 1, 2
				elif cat == 'Trailer': 		algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.3, 3, 2
				elif cat == 'Bus': 			algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.4, 1, 2
				elif cat == 'Motorcycle':	algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.7, 3, 2
				elif cat == 'Bicycle': 		algm, metric, thres, min_hits, max_age = 'greedy', 'dist_3d',    6, 3, 2
				else: assert False, 'error'
			elif cfg.det_name == 'megvii':			# tuned for Megvii detections
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.5, 1, 2
				elif cat == 'Pedestrian': 	algm, metric, thres, min_hits, max_age = 'greedy', 'dist_3d',    2, 1, 2
				elif cat == 'Truck': 		algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.2, 1, 2
				elif cat == 'Trailer': 		algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.2, 3, 2
				elif cat == 'Bus': 			algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.2, 1, 2
				elif cat == 'Motorcycle':	algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.8, 3, 2
				elif cat == 'Bicycle': 		algm, metric, thres, min_hits, max_age = 'greedy', 'giou_3d', -0.6, 3, 2
				else: assert False, 'error'
			elif cfg.det_name == 'deprecated':		
				if cat == 'Car': 			metric, thres, min_hits, max_age = 'dist', 10, 3, 2
				elif cat == 'Pedestrian': 	metric, thres, min_hits, max_age = 'dist',  6, 3, 2	
				elif cat == 'Bicycle': 		metric, thres, min_hits, max_age = 'dist',  6, 3, 2
				elif cat == 'Motorcycle':	metric, thres, min_hits, max_age = 'dist', 10, 3, 2
				elif cat == 'Bus': 			metric, thres, min_hits, max_age = 'dist', 10, 3, 2
				elif cat == 'Trailer': 		metric, thres, min_hits, max_age = 'dist', 10, 3, 2
				elif cat == 'Truck': 		metric, thres, min_hits, max_age = 'dist', 10, 3, 2
				else: assert False, 'error'
			else: assert False, 'error'
		else: assert False, 'no such dataset'

		# add negative due to it is the cost
		if metric in ['dist_3d', 'dist_2d', 'm_dis']: thres *= -1	
		self.algm, self.metric, self.thres, self.max_age, self.min_hits = \
			algm, metric, thres, max_age, min_hits

		# define max/min values for the output affinity matrix
		if self.metric in ['dist_3d', 'dist_2d', 'm_dis']: self.max_sim, self.min_sim = 0.0, -100.
		elif self.metric in ['iou_2d', 'iou_3d']:   	   self.max_sim, self.min_sim = 1.0, 0.0
		elif self.metric in ['giou_2d', 'giou_3d']: 	   self.max_sim, self.min_sim = 1.0, -1.0

	def process_dets(self, dets, info):
		# convert each detection into the class Box3D 
		# inputs: 
		# 	dets - a numpy array of detections in the format [[h,w,l,x,y,z,theta],...]

		dets_new = []
		for k, det in enumerate(dets):
			det_tmp = Box3D.array2bbox_raw(det)
			det_tmp.carla_id = int(info[k, CID])    # ← add fields
			det_tmp.guid     = int(info[k, GUID])
			dets_new.append(det_tmp)

		return dets_new

	def within_range(self, theta):
		# make sure the orientation is within a proper range

		if theta >= np.pi: theta -= np.pi * 2    # make the theta still in the range
		if theta < -np.pi: theta += np.pi * 2

		return theta

	def orientation_correction(self, theta_pre, theta_obs):
		# update orientation in propagated tracks and detected boxes so that they are within 90 degree
		
		# make the theta still in the range
		theta_pre = self.within_range(theta_pre)
		theta_obs = self.within_range(theta_obs)

		# if the angle of two theta is not acute angle, then make it acute
		if abs(theta_obs - theta_pre) > np.pi / 2.0 and abs(theta_obs - theta_pre) < np.pi * 3 / 2.0:     
			theta_pre += np.pi       
			theta_pre = self.within_range(theta_pre)

		# now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
		if abs(theta_obs - theta_pre) >= np.pi * 3 / 2.0:
			if theta_obs > 0: theta_pre += np.pi * 2
			else: theta_pre -= np.pi * 2

		return theta_pre, theta_obs

	def ego_motion_compensation(self, frame, trks):
		# inverse ego motion compensation, move trks from the last frame of coordinate to the current frame for matching
		
		from AB3DMOT_libs.kitti_oxts import get_ego_traj, egomotion_compensation_ID
		assert len(self.trackers) == len(trks), 'error'
		ego_xyz_imu, ego_rot_imu, left, right = get_ego_traj(self.oxts, frame, 1, 1, only_fut=True, inverse=True) 
		for index in range(len(self.trackers)):
			trk_tmp = trks[index]
			xyz = np.array([trk_tmp.x, trk_tmp.y, trk_tmp.z]).reshape((1, -1))
			compensated = egomotion_compensation_ID(xyz, self.calib, ego_rot_imu, ego_xyz_imu, left, right)
			trk_tmp.x, trk_tmp.y, trk_tmp.z = compensated[0]

			# update compensated state in the Kalman filter
			try:
				self.trackers[index].kf.x[:3] = copy.copy(compensated).reshape((-1))
			except:
				self.trackers[index].kf.x[:3] = copy.copy(compensated).reshape((-1, 1))

		return trks

	def visualization(self, img, dets, trks, calib, hw, save_path, height_threshold=0):
		# visualize to verify if the ego motion compensation is done correctly
		# ideally, the ego-motion compensated tracks should overlap closely with detections
		import cv2 
		from PIL import Image
		from AB3DMOT_libs.vis import draw_box3d_image
		from xinshuo_visualization import random_colors

		dets, trks = copy.copy(dets), copy.copy(trks)
		img = np.array(Image.open(img))
		max_color = 20
		colors = random_colors(max_color)       # Generate random colors

		# visualize all detections as yellow boxes
		for det_tmp in dets: 
			img = vis_obj(det_tmp, img, calib, hw, (255, 255, 0))				# yellow for detection
		
		# visualize color-specific tracks
		count = 0
		ID_list = [tmp.id for tmp in self.trackers]
		for trk_tmp in trks: 
			ID_tmp = ID_list[count]
			color_float = colors[int(ID_tmp) % max_color]
			color_int = tuple([int(tmp * 255) for tmp in color_float])
			str_vis = '%d, %f' % (ID_tmp, trk_tmp.o)
			img = vis_obj(trk_tmp, img, calib, hw, color_int, str_vis)		# blue for tracklets
			count += 1
		
		img = Image.fromarray(img)
		img = img.resize((hw['image'][1], hw['image'][0]))
		img.save(save_path)

	def prediction(self):
		# get predicted locations from existing tracks
		# Batched Kalman predict across all trackers.  F and Q are identical
		# across trackers (constant-velocity model in AB3DMOT), so we stack
		# states and covariances and apply the predict step in one BLAS call.

		trks = []
		n = len(self.trackers)
		if n == 0:
			return trks

		F = self.trackers[0].kf.F
		Q = self.trackers[0].kf.Q
		if self._stride != 1:
			# The state velocity is in m per source tick: predict over the ticks since the last
			# call. The process noise is the noise of that many one-tick predicts,
			# Q_s = sum_{k<s} F^k Q (F^k)^T, so one predict over s ticks leaves the same state and
			# covariance as s one-tick predicts (tests/test_tracker_predict_interval.py).
			F1 = F
			Fk = np.eye(F1.shape[0])
			Qs = np.zeros_like(Q, dtype=np.float64)
			for _ in range(int(self._stride)):
				Qs += Fk @ Q @ Fk.T
				Fk = F1 @ Fk
			F, Q = Fk, Qs
		FT = F.T

		# Preallocate and fill via direct indexing (faster than np.stack
		# for small N where Python overhead dominates)
		X = np.empty((n, 10, 1), dtype=np.float64)
		P = np.empty((n, 10, 10), dtype=np.float64)
		for i in range(n):
			kf = self.trackers[i].kf
			X[i] = kf.x
			P[i] = kf.P

		# np.matmul with broadcasting is BLAS-accelerated
		X_new = F @ X                                           # [N, 10, 1]
		P_new = F @ P @ FT + Q                                  # [N, 10, 10]

		for t_idx in range(n):
			kf_tmp = self.trackers[t_idx]
			if kf_tmp.id == self.debug_id:
				print('\n before prediction')
				print(kf_tmp.kf.x.reshape((-1)))
				print('\n current velocity')
				print(kf_tmp.get_velocity())
			kf_tmp.kf.x = X_new[t_idx]
			kf_tmp.kf.P = P_new[t_idx]
			if kf_tmp.id == self.debug_id:
				print('After prediction')
				print(kf_tmp.kf.x.reshape((-1)))
			kf_tmp.kf.x[3] = self.within_range(kf_tmp.kf.x[3])

			# update statistics
			kf_tmp.time_since_update += 1
			kf_tmp.anchoring_age += 1  			# tick the anchoring epoch counter
			trk_tmp = kf_tmp.kf.x.reshape((-1))[:7]

			trk_box = Box3D.array2bbox(trk_tmp)

			# attach meta so compute_affinity / data_association can read it
			trk_box.carla_id     = getattr(kf_tmp, "carla_id", -1)
			trk_box.guid         = getattr(kf_tmp, "guid",     -1)
			trk_box.anchoring_age = kf_tmp.anchoring_age
			trk_box.hits         = kf_tmp.hits

			# Ground-plane speed for kinematic gating in matching.
			# KF state: [x, y, z, theta, l, w, h, dx, dy, dz]
			# KITTI x,z = ground plane (CARLA x,y).  Skip dy (height).
			trk_vel = kf_tmp.kf.x.reshape((-1))[7:10]
			trk_box.kf_speed_ground = float(
				(trk_vel[0]**2 + trk_vel[2]**2)**0.5)
			# measured-edge history: a track with fewer than ten matched frames
			# has no robust velocity yet and gets the young-track gate
			_eh = getattr(kf_tmp, 'edge_hist', None)
			trk_box.edges_in_use = _eh is not None
			trk_box.n_edge_frames = len(_eh) if _eh is not None else 0
			if _eh is not None and trk_box.n_edge_frames < 10 and self._mature(_eh):
				trk_box.n_edge_frames = 10    # half a second of matched frames at a longer stride

			trks.append(trk_box)
			#trks.append(Box3D.array2bbox(trk_tmp))

		return trks

	def update(self, matched, unmatched_trks, dets, info, edges=None):
		# update matched trackers with assigned detections
		# Vectorized Kalman update across all matched trackers.
		# H and R are identical across trackers; stack x, P, and z and
		# compute innovation, gain, and covariance in batched BLAS calls.

		dets = copy.copy(dets)
		unmatched_set = set(unmatched_trks) if not isinstance(unmatched_trks, set) else unmatched_trks

		# Build the list of matched (tracker_idx, detection_idx)
		matched_pairs = []
		for t in range(len(self.trackers)):
			if t in unmatched_set:
				continue
			d = matched[np.where(matched[:, 1] == t)[0], 0]
			assert len(d) == 1, 'error'
			matched_pairs.append((t, int(d[0])))

		M = len(matched_pairs)
		if M == 0:
			return

		H = self.trackers[matched_pairs[0][0]].kf.H
		R = self.trackers[matched_pairs[0][0]].kf.R
		HT = H.T

		# Stack matched state / covariance / measurement
		X = np.empty((M, 10, 1), dtype=np.float64)
		P = np.empty((M, 10, 10), dtype=np.float64)
		Z = np.empty((M, 7, 1), dtype=np.float64)
		bbox3d_store = [None] * M
		for j, (t, d_idx) in enumerate(matched_pairs):
			trk = self.trackers[t]
			X[j] = trk.kf.x
			P[j] = trk.kf.P
			bbox3d = Box3D.bbox2array(dets[d_idx])[:7]
			theta_pre, theta_obs = self.orientation_correction(float(X[j, 3, 0]), float(bbox3d[3]))
			X[j, 3, 0] = theta_pre
			bbox3d[3] = theta_obs
			Z[j] = bbox3d.reshape(7, 1)
			bbox3d_store[j] = bbox3d

		# Batched Kalman update
		HX = H @ X                                # (M, 7, 1)
		Y = Z - HX                                # innovation
		PHT = P @ HT                              # (M, 10, 7)
		S = H @ PHT + R                           # (M, 7, 7)
		Sinv = np.linalg.inv(S)                   # (M, 7, 7)
		K = PHT @ Sinv                            # (M, 10, 7)
		X_new = X + K @ Y                         # (M, 10, 1)
		I10 = np.eye(10)
		I_KH = I10 - K @ H                        # (M, 10, 10)
		# Joseph form: P = (I-KH)P(I-KH)^T + KRK^T
		# The simple form P = (I-KH)P is only numerically stable for the
		# exact optimal gain; with floating-point K the missing KRK^T term
		# causes P to collapse toward zero after one update, driving K→0
		# and making the filter ignore subsequent measurements.
		KT = K.transpose(0, 2, 1)                 # (M, 7, 10)
		I_KHT = I_KH.transpose(0, 2, 1)          # (M, 10, 10)
		P_new = I_KH @ P @ I_KHT + K @ R @ KT    # (M, 10, 10)

		# Write back and apply per-tracker side effects
		for j, (t, d_idx) in enumerate(matched_pairs):
			trk = self.trackers[t]
			trk.kf.x = X_new[j]
			trk.kf.P = P_new[j]
			trk.time_since_update = 0
			trk.hits += 1
			self._note_edges(trk, edges, d_idx)
			if getattr(self, '_cur_frame_tick', None) is not None:
				trk.last_obs_tick = self._cur_frame_tick

			if trk.id == self.debug_id:
				print('After ego-compoensation')
				print(trk.kf.x.reshape((-1)))
				print('matched measurement')
				print(bbox3d_store[j].reshape((-1)))

			cid_in = int(info[d_idx, CID])
			if cid_in != -1:
				if trk.carla_id != -1 and cid_in == trk.carla_id:
					trk.anchoring_age = 0
				if trk.carla_id == -1:
					already = any((o is not trk) and (o.carla_id == cid_in)
								  for o in self.trackers)
					if not already:
						trk.carla_id = cid_in
						trk.anchoring_age = 0
						if getattr(trk, 'anon_birth_frame', -1) >= 0:
							self.anon_hold_ticks.append(self._tidx - trk.anon_birth_frame)
							trk.anon_birth_frame = -1

			if trk.id == self.debug_id:
				print('after matching')
				print(trk.kf.x.reshape((-1)))
				print('\n current velocity')
				print(trk.get_velocity())

			trk.kf.x[3] = self.within_range(trk.kf.x[3])
			trk.info = info[d_idx, :]

	def _in_exclusion_zone(self, det_box):
		"""Check if a CID=-1 detection falls in the anisotropic heading-
		aligned exclusion zone of any beacon-identified track.

		The zone is an oriented rectangle in the identified track's heading
		frame: wide along-track (covers depth error) and tight cross-track
		(spares adjacent-lane vehicles).

		Returns True if the detection should be suppressed.
		"""
		if self._dup_x_max <= 0:
			return False
		det_l = getattr(det_box, 'l', 4.0)
		det_w = getattr(det_box, 'w', 2.0)
		for trk in self.trackers:
			if trk.carla_id == -1:
				continue
			if trk.anchoring_age >= self.anchoring_epoch:
				continue
			# Size guard: skip if detection is a very different size
			# (avoids merging a pedestrian into a vehicle track)
			trk_l = float(trk.kf.x[4])  # KF state: [x,y,z,theta,l,w,h,...]
			trk_w = float(trk.kf.x[5])
			if trk_l > 0 and det_l > 0:
				lr = max(det_l, trk_l) / max(min(det_l, trk_l), 0.1)
				if lr > self._dup_size_ratio:
					continue
			if trk_w > 0 and det_w > 0:
				wr = max(det_w, trk_w) / max(min(det_w, trk_w), 0.1)
				if wr > self._dup_size_ratio:
					continue
			# Displacement on ground plane (KITTI x,z = CARLA x,y)
			dx_world = det_box.x - float(trk.kf.x[0])
			dz_world = det_box.z - float(trk.kf.x[2])
			# Rotate into track heading frame
			theta = float(trk.kf.x[3])
			cos_t, sin_t = math.cos(theta), math.sin(theta)
			dx_along =  dx_world * cos_t + dz_world * sin_t
			dy_cross = -dx_world * sin_t + dz_world * cos_t
			if -self._dup_x_max <= dx_along <= self._dup_x_ahead and abs(dy_cross) <= self._dup_y_max:
				return True
		return False

	def birth(self, dets, info, unmatched_dets, edges=None):
		# create and initialise new trackers for unmatched detections

		new_id_list = list()
		for i in unmatched_dets:
			cid = int(info[i, CID])

			# Identity exclusion zone: suppress anonymous detections
			# that are depth-error duplicates of beacon-identified tracks.
			if cid == -1:
				self.birth_attempts_anon += 1
				if self._in_exclusion_zone(dets[i]):
					self.birth_suppressed_by_gate += 1
					continue

			trk = KF(Box3D.bbox2array(dets[i])[:7], info[i, :], self.ID_count[0])
			trk.carla_id = cid
			trk.guid     = int(info[i, GUID])
			trk.near_identified_ticks = 0  # for post-birth cull
			trk.anon_birth_frame = self._tidx if cid == -1 else -1
			self._note_edges(trk, edges, i)
			if getattr(self, '_cur_frame_tick', None) is not None:
				trk.last_obs_tick = self._cur_frame_tick   # born from this frame's detection
			self.trackers.append(trk)
			new_id_list.append(trk.id)

			if cid == -1:
				self.births_anon_after_gate += 1

			self.ID_count[0] += 1

		return new_id_list

	def _cull_anon_near_identified(self):
		"""Delete anonymous tracks that persist near beacon-identified
		tracks for too many consecutive ticks.

		Uses the same anisotropic heading-aligned gate as the birth
		exclusion zone. This catches ghosts that were born during a
		beacon loss burst and survived past the birth gate.
		"""
		if self._dup_x_max <= 0:
			return

		identified = [t for t in self.trackers if t.carla_id != -1
		              and t.anchoring_age < self.anchoring_epoch]
		if not identified:
			return

		to_remove = []
		for trk in self.trackers:
			if trk.carla_id != -1:
				continue
			near = False
			trk_x = float(trk.kf.x[0])
			trk_z = float(trk.kf.x[2])
			trk_l = float(trk.kf.x[4])
			trk_w = float(trk.kf.x[5])
			for it in identified:
				# Size guard
				it_l = float(it.kf.x[4])
				it_w = float(it.kf.x[5])
				if it_l > 0 and trk_l > 0:
					lr = max(trk_l, it_l) / max(min(trk_l, it_l), 0.1)
					if lr > self._dup_size_ratio:
						continue
				if it_w > 0 and trk_w > 0:
					wr = max(trk_w, it_w) / max(min(trk_w, it_w), 0.1)
					if wr > self._dup_size_ratio:
						continue
				dx_w = trk_x - float(it.kf.x[0])
				dz_w = trk_z - float(it.kf.x[2])
				theta = float(it.kf.x[3])
				cos_t, sin_t = math.cos(theta), math.sin(theta)
				dx_along =  dx_w * cos_t + dz_w * sin_t
				dy_cross = -dx_w * sin_t + dz_w * cos_t
				if not (-self._dup_x_max <= dx_along <= self._dup_x_ahead and abs(dy_cross) <= self._dup_y_max):
					continue
				# kinematic consistency: a duplicate moves with its original
				v_trk = math.hypot(float(trk.kf.x[7]), float(trk.kf.x[9])) / self._dup_dt
				v_it = math.hypot(float(it.kf.x[7]), float(it.kf.x[9])) / self._dup_dt
				if abs(v_trk - v_it) > self._dup_v_max:
					continue
				near = True
				break

			if near:
				trk.near_identified_ticks = getattr(trk, 'near_identified_ticks', 0) + 1
				if trk.near_identified_ticks >= self._cull_consec_ticks:
					to_remove.append(trk)
			else:
				trk.near_identified_ticks = 0

		for trk in to_remove:
			self.trackers.remove(trk)
			self.anon_cull_count += 1

	def _note_edges(self, trk, edges, idx):
		"""Keep the measured box edges (world xmin, xmax, ymin, ymax) of the
		detection a track was born from or matched to, with the frame, for the
		published velocity. Bundles without 'edges' leave the track on the
		position-displacement estimate."""
		if edges is None or idx >= len(edges):
			return
		h = getattr(trk, 'edge_hist', None)
		if h is None:
			h = trk.edge_hist = []
		e = tuple(float(v) for v in edges[idx][:4])
		if h:
			# A hop of the whole box beyond what the track's velocity explains
			# (2 m over the expected motion) is a change of object or of view,
			# not motion: the velocity estimate restarts from this frame.
			f0, e0 = h[-1]
			df = max(1, self._tidx - f0)
			v = trk.kf.x.reshape(-1)[7:10]
			expected = math.hypot(float(v[0]), float(v[2])) * df
			moved = math.hypot((e[0] + e[1] - e0[0] - e0[1]) / 2.0, (e[2] + e[3] - e0[2] - e0[3]) / 2.0)
			if moved > expected + 2.0:
				del h[:]
		h.append((self._tidx, e))
		if len(h) > 21:
			del h[0]

	# Published-velocity window in frames (50 ms each). The slope is fitted over the history
	# entries of the last VEL_WINDOW frames by frame number, so dropped detections do not stretch
	# the window: the former 21-entry displacement spanned 1.5 s under a 30% drop rate and read a
	# car accelerating through 12 m/s at 9.3 m/s (j4 left turn, 70 km/h, 500 ms, 2026-10-26). The
	# long window serves only when the short one holds too few entries.
	VEL_WINDOW = 10
	VEL_WINDOW_LONG = 20
	VEL_MIN_SAMPLES = 4

	@staticmethod
	def _window_slopes(hist, f1, window, min_samples):
		"""Least-squares slope per column, in units per frame, over the entries of hist (a list of
		(frame, values)) whose frame lies within the last `window` frames of f1. None with fewer
		than min_samples entries or a span under two frames."""
		pts = [(f, v) for f, v in hist if f >= f1 - window]
		if len(pts) < min_samples or pts[-1][0] - pts[0][0] < 2:
			return None
		fr = np.array([p[0] for p in pts], dtype=float)
		vals = np.array([p[1] for p in pts], dtype=float)
		fr -= fr.mean()
		den = float((fr * fr).sum())
		if den <= 0.0:
			return None
		return ((vals - vals.mean(axis=0)) * fr[:, None]).sum(axis=0) / den

	def _mean_stride(self):
		"""Source ticks per call over the calls of the last half second (1.0 for a tracker fed every tick)."""
		k = [t for t in self._call_tidx if t > self._tidx - self.VEL_WINDOW]
		if len(k) < 2:
			return float(self._stride)
		return (k[-1] - k[0]) / (len(k) - 1)

	def _samples(self, n):
		"""The number of calls that n one-tick frames correspond to at the current stride, at least two."""
		return max(2, int(math.ceil(n / self._mean_stride())))

	def _mature(self, hist, from_birth=True):
		"""True when hist (a list of (time index, ...)) holds half a second of history.

		For a tracker fed every tick the rule is ten entries, as before: ten consecutive entries
		span nine ticks. For a tracker fed every s ticks the rule is the same time: the entries
		that fit in the span, never fewer than two (no velocity from one sample), over a span of
		nine ticks. The measured-edge history (from_birth) starts at the track's first detection
		and uses that span as is. The position history starts when the track is first published,
		min_hits - 1 calls after its first detection, which costs (min_hits - 1) * (s - 1) more
		ticks than at one tick per call. That wait is observation time too, so it is taken off
		the span (down to one stride), and a launched vehicle is published as moving after the
		same time since its first detection at any stride, to within one stride."""
		ms = self._mean_stride()
		span = float(self.VEL_WINDOW - 1)
		if not from_birth:
			span = max(ms, span - (self.min_hits - 1) * (ms - 1.0))
		need = max(2, int(math.ceil((span + 1.0) / ms)))
		return len(hist) >= need and hist[-1][0] - hist[0][0] >= span

	def _history_slopes(self, hist):
		"""Slopes (units per source tick) over the last VEL_WINDOW ticks of hist, else over
		VEL_WINDOW_LONG, else None."""
		f1 = hist[-1][0]
		s = self._window_slopes(hist, f1, self.VEL_WINDOW, self._samples(self.VEL_MIN_SAMPLES))
		if s is None:
			s = self._window_slopes(hist, f1, self.VEL_WINDOW_LONG, self._samples(3))
		return s

	def _edge_velocity(self, eh, vel):
		"""Velocity (m per frame, [vx, vy, vz] in the state's axes) from the
		slower-moving edge of the measured box per ground axis. A rigid vehicle
		moves both edges of its box; a change of what is visible moves one: a
		parked truck seen from behind by a CAV closing on it grows from its rear
		face along its side, so the mean of its points slid north at 4 to 7 m/s
		while its rear edge stood still (2026-10-15, scenario_3 column), and a
		camera box cut by the image edge slides the same way alongside. Edges
		moving in opposite directions give zero, edges moving together within
		the slope noise are averaged. Each edge's motion is the
		least-squares slope over the last half second of matched frames, so an
		accelerating vehicle is read at its recent speed and a face that slid
		and then stopped is read at rest; under 2 m/s the track is published at
		rest, as for the position estimate."""
		def slower(a, b):
			# Two edges that both move faster than the rest threshold and agree
			# within the slope noise (2 m/s at the half-second window) average, so
			# a rigid vehicle is not read at the lower of two noisy estimates. Any
			# other pair gives the slower, so a parked vehicle whose box drifts
			# with the viewpoint stays at rest.
			if a * b <= 0.0:
				return 0.0
			v_rest = 2.0 * max(getattr(self, 'dt', 0.05), 1e-3)
			if abs(a) >= v_rest and abs(b) >= v_rest and abs(a - b) <= v_rest:
				return 0.5 * (a + b)
			return a if abs(a) < abs(b) else b
		s = self._history_slopes(eh)
		if s is None:
			return np.zeros(3)
		vx = slower(float(s[0]), float(s[1]))
		vz = slower(float(s[2]), float(s[3]))
		if math.hypot(vx, vz) / max(getattr(self, 'dt', 0.05), 1e-3) < 2.0:
			vx, vz = 0.0, 0.0
		return np.array([vx, float(vel[1]) if len(vel) > 1 else 0.0, vz])

	def output(self):
		# output exiting tracks that have been stably associated, i.e., >= min_hits
		# and also delete tracks that have appeared for a long time, i.e., >= max_age

		num_trks = len(self.trackers)
		results = []
		for trk in reversed(self.trackers):
			# change format from [x,y,z,theta,l,w,h] to [h,w,l,x,y,z,theta]
			d = Box3D.array2bbox(trk.kf.x[:7].reshape((7, )))     # bbox location self
			d = Box3D.bbox2array_raw(d)
			# Box3D.__init__ defaults self.s = 0.0 (not None), so bbox2array_raw
			# returns the 8-element form [h,w,l,x,y,z,ry,score]. The downstream
			# row layout below assumes 7 elements; drop the trailing score so
			# col 7 of the concatenated row is trk.id, not score=0. Without this
			# every track output gets track_id=0 (collapses all tracks into one
			# trajectory bucket in _ab3d_history_to_trajs and build_trajectories).
			if d.shape[0] == 8:
				d = d[:7]

			if ((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <= self.min_hits)):
				vel = trk.get_velocity().flatten()
				# Published velocity is the track's displacement over its last
				# 0.5 s (finite difference of the output positions), not the
				# filter's instantaneous velocity state. The state absorbs the
				# jumps between the faces of one object seen from different
				# sensors (a stationary truck's state read up to 16 m/s on
				# 2026-10-14); the displacement over ten ticks does not, and a
				# coasting track's positions advance by the state velocity so the
				# two agree there. The state stays as is for prediction and gating.
				_hist = getattr(trk, 'pos_hist', None)
				if _hist is None:
					_hist = trk.pos_hist = []
				_hist.append((self._tidx, float(trk.kf.x[0]), float(trk.kf.x[2])))
				if len(_hist) > max(21, self.VEL_WINDOW_LONG + 1):
					del _hist[0]
				_eh = getattr(trk, 'edge_hist', None) if self.vel_from_edges else None
				if not self._mature(_hist, from_birth=False):
					# under half a second of history the displacement is
					# dominated by the measured centroid settling (a track
					# born from one sensor's face of a parked vehicle read
					# 5 to 8 m/s on its first frames, 2026-10-14): at rest
					vel = np.zeros(3)
				elif _eh is not None:
					# measured box edges in use: the slower edge decides after ten
					# matched frames (see _edge_velocity); before that the track is
					# published at rest, never from the position displacement, which
					# a young track dragged along a column by a passing sensor's
					# wandering reports read as 16 m/s (2026-10-15)
					vel = self._edge_velocity(_eh, vel) if self._mature(_eh) else np.zeros(3)
				else:
					# Least-squares slope of the output position over the last
					# half second of frames (_history_slopes): a measured face
					# that slid along a parked vehicle and then stopped at its
					# end (the passing CAV's camera box reaching the front
					# corner) reads as rest once it has stood for half a second,
					# and an accelerating vehicle reads at its recent speed
					# instead of its average over the whole history.
					_s = self._history_slopes([(f, (x, z)) for f, x, z in _hist])
					if _s is None:
						vel = np.zeros(3)
					else:
						vel = np.array([float(_s[0]), float(vel[1]) if len(vel) > 1 else 0.0, float(_s[1])])
					# Slower than 2 m/s over the last second is published at
					# rest: a parked vehicle whose measured centroid wanders
					# between its visible faces (up to 1.9 m across a second)
					# must not be extrapolated into the road, and a vehicle
					# that is really moving that slowly covers under 2.5 m in
					# the planner's horizon. The threshold is the one the
					# predictor already applied at 1 m/s (_MIN_KF_SPEED_MPS).
					if math.hypot(vel[0], vel[2]) / max(getattr(self, 'dt', 0.05), 1e-3) < 2.0:
						vel = np.zeros(3)
				# A track published moving keeps its last published velocity while
				# the rest rules read zero for under half a second: a change of the
				# visible box extent at a sensor handover moves the two edges in
				# opposite directions for a few frames and read a crossing car at
				# rest (j6s straight crossing 63 km/h, 2026-10-30); the zero went
				# into the state below, the track froze, lost its association and
				# the planner held a stationary forecast for 650 ms. Rest needs the
				# same half second of evidence that motion needs.
				if self._mature(_hist, from_birth=False):
					_dt_s = max(getattr(self, 'dt', 0.05), 1e-3)
					_prev = getattr(trk, 'pub_vel', None)
					_rest_now = math.hypot(vel[0], vel[2]) / _dt_s < 2.0
					if (_rest_now and _prev is not None
							and math.hypot(_prev[0], _prev[2]) / _dt_s >= 2.0):
						trk.rest_frames = getattr(trk, 'rest_frames', 0) + self._stride
						if trk.rest_frames < 10:
							vel = np.array(_prev, dtype=float)
						else:
							trk.pub_vel = np.array(vel, dtype=float)
					else:
						trk.rest_frames = 0
						trk.pub_vel = np.array(vel, dtype=float)
				# After half a second the filter's ground-plane velocity state
				# follows the same estimate: the state velocity is what the
				# coasting prediction and the velocity-aware association gate
				# use, and a state that absorbed one jump between sensor faces
				# walked a track along a column of parked trucks at 14 to 21
				# m/s (2026-10-14) because each coast re-associated to the next
				# vehicle. Under half a second the filter keeps its own
				# estimate (a 13 m/s target would otherwise be lost before the
				# displacement is available). Position and size untouched.
				if self._mature(_hist, from_birth=False):
					trk.kf.x[7] = float(vel[0])
					trk.kf.x[9] = float(vel[2])
				elif trk.time_since_update > 0:
					# A young track that has lost its detection holds its
					# position instead of coasting on a velocity it has had
					# no time to establish: coasting on a face jump walked a
					# newborn track onto the neighbouring parked vehicle and
					# the displacement then read that hop as 16 to 19 m/s.
					trk.kf.x[7] = 0.0
					trk.kf.x[9] = 0.0
				# A coasting track (predict-only, no fresh measurement) keeps
				# the velocity the filter converged to while it was matched:
				# a crossing vehicle briefly lost at a camera handover must
				# be published moving, not as a stationary obstacle (the
				# planner braked for such phantoms, 2026-10-14). Only a track
				# that never reached min_hits has an uninitialised velocity;
				# that one is published at rest so the downstream
				# _MIN_KF_SPEED_MPS gate treats it as stationary.
				if trk.time_since_update > 0 and trk.hits < self.min_hits:
					vel = np.zeros(3)
				out_row = np.concatenate([
					d,                         # 0..6  [h,w,l,x,y,z,theta]
					[trk.id],                  # 7     track id
					[trk.carla_id],            # 8     CARLA actor id
					[trk.guid],                # 9     sender GUID
					vel[:3],                   # 10..12 KF velocity [dx,dy,dz] (m/tick)
					trk.info                   # 13…   frame, guid, carla_id, …
				]).reshape(1, -1)
				results.append(out_row)		# append the output row
				#results.append(np.concatenate((d, [trk.id], trk.info)).reshape(1, -1)) 		
			num_trks -= 1

			# deadth, remove dead tracklet
			if (trk.time_since_update >= self.max_age):
				self.trackers.pop(num_trks)

		return results

	def process_affi(self, affi, matched, unmatched_dets, new_id_list):

		# post-processing affinity matrix, convert from affinity between raw detection and past total tracklets
		# to affinity between past "active" tracklets and current active output tracklets, so that we can know 
		# how certain the results of matching is. The approach is to find the correspondes of ID for each row and
		# each column, map to the actual ID in the output trks, then purmute/expand the original affinity matrix
		
		###### determine the ID for each past track
		trk_id = self.id_past 			# ID in the trks for matching

		###### determine the ID for each current detection
		det_id = [-1 for _ in range(affi.shape[0])]		# initialization

		# assign ID to each detection if it is matched to a track
		for match_tmp in matched:		
			det_id[match_tmp[0]] = trk_id[match_tmp[1]]

		# assign the new birth ID to each unmatched detection
		count = 0
		assert len(unmatched_dets) == len(new_id_list), 'error'
		for unmatch_tmp in unmatched_dets:
			det_id[unmatch_tmp] = new_id_list[count] 	# new_id_list is in the same order as unmatched_dets
			count += 1
		assert not (-1 in det_id), 'error, still have invalid ID in the detection list'

		############################ update the affinity matrix based on the ID matching
		
		# transpose so that now row is past trks, col is current dets	
		affi = affi.transpose() 			

		###### compute the permutation for rows (past tracklets), possible to delete but not add new rows
		permute_row = list()
		for output_id_tmp in self.id_past_output:
			index = trk_id.index(output_id_tmp)
			permute_row.append(index)
		affi = affi[permute_row, :]	
		assert affi.shape[0] == len(self.id_past_output), 'error'

		###### compute the permutation for columns (current tracklets), possible to delete and add new rows
		# addition can be because some tracklets propagated from previous frames with no detection matched
		# so they are not contained in the original detection columns of affinity matrix, deletion can happen
		# because some detections are not matched

		max_index = affi.shape[1]
		permute_col = list()
		to_fill_col, to_fill_id = list(), list() 		# append new columns at the end, also remember the ID for the added ones
		for output_id_tmp in self.id_now_output:
			try:
				index = det_id.index(output_id_tmp)
			except:		# some output ID does not exist in the detections but rather predicted by KF
				index = max_index
				max_index += 1
				to_fill_col.append(index); to_fill_id.append(output_id_tmp)
			permute_col.append(index)

		# expand the affinity matrix with newly added columns
		append = np.zeros((affi.shape[0], max_index - affi.shape[1]))
		append.fill(self.min_sim)
		affi = np.concatenate([affi, append], axis=1)

		# find out the correct permutation for the newly added columns of ID
		for count in range(len(to_fill_col)):
			fill_col = to_fill_col[count]
			fill_id = to_fill_id[count]
			row_index = self.id_past_output.index(fill_id)

			# construct one hot vector because it is proapgated from previous tracks, so 100% matching
			affi[row_index, fill_col] = self.max_sim		
		affi = affi[:, permute_col]

		return affi

	def track(self, dets_all, frame):
		"""
		Params:
		  	dets_all: dict
				dets - a numpy array of detections in the format [[h,w,l,x,y,z,theta],...]
				info: a array of other info for each det
			frame:    str, frame number, used to query ego pose
		Requires: this method must be called once for each frame even with empty detections.
		Returns the a similar array, where the last column is the object ID.

		NOTE: The number of objects returned may differ from the number of detections provided.
		"""
		_prev = os.environ.get('AB3D_SHADOW_PREV')
		if _prev and not getattr(self, '_shadow_busy', False):
			# differential check against an earlier model.py (AB3DMOT_libs/shadow_check.py)
			from AB3DMOT_libs.shadow_check import shadow_track
			return shadow_track(self, dets_all, frame, _prev)
		dets, info = dets_all['dets'], dets_all['info']         # dets: N x 7, float numpy array
		edges = dets_all.get('edges') if hasattr(dets_all, 'get') else None   # measured box edges per det
		if self.debug_id: print('\nframe is %s' % frame)
		# tick of the frame being ingested; matched tracks record it as the
		# time of their last real observation (observation lineage)
		try:
			self._cur_frame_tick = int(frame)
		except (TypeError, ValueError):
			self._cur_frame_tick = None
	
		# logging
		self.frame_count += 1
		_t = self._cur_frame_tick
		self._stride = 1
		if self.time_rules and _t is not None:
			if self._prev_tick is not None and _t > self._prev_tick:
				self._stride = _t - self._prev_tick
			self._prev_tick = _t if self._prev_tick is None else max(self._prev_tick, _t)
		self._tidx += self._stride
		self._call_tidx.append(self._tidx)
		if len(self._call_tidx) > 32:
			del self._call_tidx[0]

		# recall the last frames of outputs for computing ID correspondences during affinity processing
		self.id_past_output = copy.copy(self.id_now_output)
		self.id_past = [trk.id for trk in self.trackers]

		# process detection format
		dets = self.process_dets(dets, info)		# convert to Box3D objects

		# NMS: suppress duplicate detections (e.g. sensor echo of a beacon)
		if len(dets) > 0:
			inst_types = [self.cat] * len(dets)
			keep_idx, _ = nms(dets, inst_types)
			dets = [dets[i] for i in keep_idx]
			info = info[keep_idx]
			if edges is not None:
				edges = np.asarray(edges)[keep_idx]

		# tracks propagation based on velocity
		trks = self.prediction()

		# ego motion compensation, adapt to the current frame of camera coordinate
		if (frame > 0) and (self.ego_com) and (self.oxts is not None):
			trks = self.ego_motion_compensation(frame, trks)

		# visualization
		#if self.vis and (self.vis_dir is not None):
	#		img = os.path.join(self.img_dir, f'{frame:06d}.png')
	#		save_path = os.path.join(self.vis_dir, f'{frame:06d}.jpg'); mkdir_if_missing(save_path)
	#		self.visualization(img, dets, trks, self.calib, self.hw, save_path)

		# matching
		trk_innovation_matrix = None
		if self.metric == 'm_dis':
			trk_innovation_matrix = [trk.compute_innovation_matrix() for trk in self.trackers]
		matched, unmatched_dets, unmatched_trks, cost, affi = \
			data_association(dets, trks, self.metric, self.thres, self.algm, trk_innovation_matrix,
			                 anchoring=self.anchoring, anchoring_epoch=self.anchoring_epoch)
		# print_log('detections are', log=self.log, display=False)
		# print_log(dets, log=self.log, display=False)
		# print_log('tracklets are', log=self.log, display=False)
		# print_log(trks, log=self.log, display=False)
		# print_log('matched indexes are', log=self.log, display=False)
		# print_log(matched, log=self.log, display=False)
		# print_log('raw affinity matrix is', log=self.log, display=False)
		# print_log(affi, log=self.log, display=False)

		# update trks with matched detection measurement
		self.update(matched, unmatched_trks, dets, info, edges)

		# create and initialise new trackers for unmatched detections
		new_id_list = self.birth(dets, info, unmatched_dets, edges)

		# Post-birth cull: delete anonymous tracks that have persisted
		# near a beacon-identified track for too many consecutive ticks.
		# Handles ghosts born during beacon loss bursts.
		if self._cull_consec_ticks > 0:
			self._cull_anon_near_identified()

		# output existing valid tracks
		results = self.output()
		if len(results) > 0: results = [np.concatenate(results)]		# h,w,l,x,y,z,theta, ID, other info, confidence
		else:            	 results = [np.empty((0, 16))]
		self.id_now_output = results[0][:, 7].tolist()					# only the active tracks that are outputed

		# post-processing affinity to convert to the affinity between resulting tracklets
		if self.affi_process:
			affi = self.process_affi(affi, matched, unmatched_dets, new_id_list)
			# print_log('processed affinity matrix is', log=self.log, display=False)
			# print_log(affi, log=self.log, display=False)

		return results, affi
