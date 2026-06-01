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

		# Identity exclusion zone: anisotropic heading-aligned gate
		# prevents birth of anonymous tracks that are depth-error
		# duplicates of beacon-identified participants.
		#   x_max: along-track (longitudinal) — covers 5-8m depth error
		#   y_max: cross-track (lateral)      — tight to spare adjacent lanes
		self._dup_x_max = getattr(cfg, 'dup_x_max', 8.0)
		self._dup_y_max = getattr(cfg, 'dup_y_max', 2.0)
		self._dup_size_ratio = getattr(cfg, 'dup_size_ratio', 2.5)

		# Instrumentation counters
		self.birth_attempts_anon = 0
		self.birth_suppressed_by_gate = 0
		self.births_anon_after_gate = 0
		self.anon_cull_count = 0

		# Post-birth cull: consecutive ticks an anon track is near
		# a beacon-identified track before deletion
		self._cull_consec_ticks = getattr(cfg, 'cull_consec_ticks', 3)

		# debug
		# self.debug_id = 2
		self.debug_id = None

	def get_param(self, cfg, cat):
		# get parameters for each dataset

		if cfg.dataset == 'KITTI':
			if cfg.det_name == 'pvrcnn':				# tuned for PV-RCNN detections
				if cat == 'Car': 			algm, metric, thres, min_hits, max_age = 'hungar', 'giou_3d', -0.2, 3, 2
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
		FT = F.T
		Q = self.trackers[0].kf.Q

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

			trks.append(trk_box)
			#trks.append(Box3D.array2bbox(trk_tmp))

		return trks

	def update(self, matched, unmatched_trks, dets, info):
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
			if abs(dx_along) <= self._dup_x_max and abs(dy_cross) <= self._dup_y_max:
				return True
		return False

	def birth(self, dets, info, unmatched_dets):
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
				if abs(dx_along) <= self._dup_x_max and abs(dy_cross) <= self._dup_y_max:
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
				# Zero out velocity for coasting tracks (predict-only, no
				# fresh measurement).  Without update(), velocity is frozen
				# at whatever the KF estimated during initialization — often
				# a phantom artifact from LiDAR noise.  Setting it to zero
				# lets the downstream _MIN_KF_SPEED_MPS gate filter them.
				if trk.time_since_update > 0:
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
		dets, info = dets_all['dets'], dets_all['info']         # dets: N x 7, float numpy array
		if self.debug_id: print('\nframe is %s' % frame)
	
		# logging
		self.frame_count += 1

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
		self.update(matched, unmatched_trks, dets, info)

		# create and initialise new trackers for unmatched detections
		new_id_list = self.birth(dets, info, unmatched_dets)

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
