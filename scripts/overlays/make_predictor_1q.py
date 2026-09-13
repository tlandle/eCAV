"""Build mtr_edge_predictor_1q.py from the campaign overlay mtr_edge_predictor_1m.py.
PREPARED, NOT APPLIED: the live overlay stays _1m until Tyler picks the fix shape.
Shape (1): (a) pad an isolated single-track MTR call with one far static dummy
(batch-size-1 mode collapse); (b) sigma-free selection of the consumed modes
(two smallest |v_m - v_kf|) and Gaussian certainty (sigma 3 m/s) for the mode
stash the trigger reads; PADROW/RANKROW provenance lines once per run."""
import sys
SP="/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad"
s=open(f"{SP}/mtr_edge_predictor_1m.py").read()
def rep(old, new, n=1, tag=""):
    global s
    assert s.count(old)==n, f"{tag}: {s.count(old)} != {n}"
    s=s.replace(old,new)

# (a) padded isolated calls in the all/gate/cap branches
rep("""                for _tid, _ot in selected.items():
                    mtr_preds.extend(
                        self._run_mtr({_tid: _ot}, source_tick, publish_tick))""",
    """                for _tid, _ot in selected.items():
                    mtr_preds.extend(
                        self._run_mtr_padded({_tid: _ot}, source_tick, publish_tick))""", tag="all")
rep("""                mtr_preds = self._run_mtr({_ft: selected[_ft]},
                                          source_tick, publish_tick)""",
    """                mtr_preds = self._run_mtr_padded({_ft: selected[_ft]},
                                                 source_tick, publish_tick)""", tag="gate")
rep("""                for _t in _isot:
                    mtr_preds.extend(
                        self._run_mtr({_t: selected[_t]}, source_tick, publish_tick))""",
    """                for _t in _isot:
                    mtr_preds.extend(
                        self._run_mtr_padded({_t: selected[_t]}, source_tick, publish_tick))""", tag="cap")

# the padded call itself, inserted before _run_mtr
rep("""    def _run_mtr(self, tracks: Dict[int, ObstacleTrajectory],
                 source_tick, publish_tick) -> List[ObstaclePrediction]:""",
    """    _PAD_TID = -777          # freeze-1q: reserved id of the padding dummy
    _PAD_BEHIND_M = 150.0    # behind the track along its heading (not ahead)
    _PAD_LATERAL_M = 3.5     # one lane over (not the track's lane)

    def _run_mtr_padded(self, tracks: Dict[int, ObstacleTrajectory],
                        source_tick, publish_tick) -> List[ObstaclePrediction]:
        \"\"\"freeze-1q: an ISOLATED single-track call pads the batch with one far,
        static dummy agent so the decoder's intention queries have a second key.
        At batch size 1 every query cross-attends to one key, the 64 queries
        receive identical context and the model returns one trajectory
        replicated with uniform scores (measured: every freeze-1p MODESROW;
        scripts/diag/mtr_batch1_collapse_test.py). The dummy sits behind the
        track along its heading, one lane over, zero velocity, constant position
        over the whole history, valid mask; its output is dropped and its cache
        and mode-stash entries removed. Batched calls (n >= 2) are untouched.\"\"\"
        if len(tracks) != 1:
            return self._run_mtr(tracks, source_tick, publish_tick)
        (tid, ot), = tracks.items()
        tf0 = ot.trajectory[0]
        h = math.radians(float(tf0.rotation.yaw))
        if len(ot.trajectory) >= 2:
            _dx = tf0.location.x - ot.trajectory[1].location.x
            _dy = tf0.location.y - ot.trajectory[1].location.y
            if math.hypot(_dx, _dy) > 0.05:
                h = math.atan2(_dy, _dx)
        px = tf0.location.x - self._PAD_BEHIND_M * math.cos(h) - self._PAD_LATERAL_M * math.sin(h)
        py = tf0.location.y - self._PAD_BEHIND_M * math.sin(h) + self._PAD_LATERAL_M * math.cos(h)
        from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
        _dobs = ObstacleVehicle(corners=np.zeros((8, 3)), o3d_bbx=None,
                                track_id=self._PAD_TID, tick_id=0)
        _dobs.kf_speed_mps = 0.0; _dobs.kf_vx = 0.0; _dobs.kf_vy = 0.0; _dobs.carla_id = -1
        _dtraj = deque(maxlen=max(len(ot.trajectory), 2))
        for _ in range(len(ot.trajectory)):
            _dtraj.append(Transform(location=Location(x=px, y=py, z=float(tf0.location.z)),
                                    rotation=Rotation(yaw=math.degrees(h))))
        _dot = ObstacleTrajectory(_dobs, _dtraj)
        if not getattr(self, '_pad_logged', False):
            logger.info("[PADROW] behind_m=%.1f lateral_m=%.1f speed_mps=0 frames=%d "
                        "type=static_dummy tid=%d mask=valid rule=batch_pad_isolated",
                        self._PAD_BEHIND_M, self._PAD_LATERAL_M, len(_dtraj), self._PAD_TID)
            self._pad_logged = True
        preds = self._run_mtr({tid: ot, self._PAD_TID: _dot}, source_tick, publish_tick)
        if hasattr(self, 'last_mtr_modes'):
            self.last_mtr_modes.pop(self._PAD_TID, None)
        self._cache.pop(self._PAD_TID, None)
        return [p for p in preds if p.obstacle_trajectory is not _dot]

    def _run_mtr(self, tracks: Dict[int, ObstacleTrajectory],
                 source_tick, publish_tick) -> List[ObstaclePrediction]:""", tag="padded def")

# (b) selection + certainty inside _run_mtr's per-track loop
rep("""        predictions = []
        n_modes = pred_trajs.shape[1]
        for i, (tid, ot) in enumerate(zip(track_ids, otrajs)):
            best = pred_trajs[i, best_mode[i], :, :2]  # (T, 2) in center frame""",
    """        predictions = []
        n_modes = pred_trajs.shape[1]
        # freeze-1q mode selection / certainty (paper side, 2026-09-13):
        # with a second key the scoring head puts a stationary or backward mode
        # on top while a correct forward mode sits at rank 1-5 (measured). For a
        # moving track (v_kf > _RANK_VMIN) the CONSUMED modes are the two with
        # the smallest |v_m - v_kf|, v_m = the mode's 5 s endpoint progress along
        # the heading / horizon (sigma-free). The CERTAINTY the trigger sums is
        # w_m = s_m exp(-(v_m - v_kf)^2 / 2 sigma^2), sigma = _RANK_SIGMA,
        # normalised over the modes. One assumption, stated: the tracker's
        # velocity is a better speed prior over the next seconds than the score
        # head, for a track that is already moving.
        _horizon_s = float(pred_trajs.shape[2]) * float(self._dt)
        if not getattr(self, '_rank_logged', False):
            logger.info("[RANKROW] selection=argmin_abs_v_top2 certainty=gauss sigma_mps=%.1f "
                        "vmin_mps=%.1f horizon_s=%.1f", self._RANK_SIGMA, self._RANK_VMIN, _horizon_s)
            self._rank_logged = True
        for i, (tid, ot) in enumerate(zip(track_ids, otrajs)):
            _vkf = float(getattr(ot.obstacle, 'kf_speed_mps', 0.0) or 0.0)
            _vm = pred_trajs[i, :, -1, 0] / max(_horizon_s, 1e-6)          # progress along heading, m/s
            if _vkf > self._RANK_VMIN and int(tid) != self._PAD_TID:
                _order = list(np.argsort(np.abs(_vm - _vkf)))
                _w = pred_scores[i] * np.exp(-((_vm - _vkf) ** 2) / (2.0 * self._RANK_SIGMA ** 2))
            else:
                _order = list(np.argsort(-pred_scores[i]))
                _w = pred_scores[i].astype(np.float64)
            _w = _w / _w.sum() if float(_w.sum()) > 1e-12 else np.full(len(_w), 1.0 / len(_w))
            best_mode[i] = int(_order[0])
            best = pred_trajs[i, best_mode[i], :, :2]  # (T, 2) in center frame""", tag="selection")
# the stash carries the reweighted certainty; the emitted prediction's probability is the best mode's w
rep("""            _modes = []
            _sc = pred_scores[i]
            _sc_sum = float(_sc.sum()) if float(_sc.sum()) > 1e-6 else 1.0
            for _m in range(pred_trajs.shape[1]):
                _end_l = pred_trajs[i, _m, -1, :2]
                _wx = cx + float(_end_l[0]) * cos_h - float(_end_l[1]) * sin_h
                _wy = cy + float(_end_l[0]) * sin_h + float(_end_l[1]) * cos_h
                _modes.append((_wx, _wy, float(_sc[_m]) / _sc_sum))
            self.last_mtr_modes[int(tid)] = _modes""",
    """            _modes = []
            for _m in range(pred_trajs.shape[1]):
                _end_l = pred_trajs[i, _m, -1, :2]
                _wx = cx + float(_end_l[0]) * cos_h - float(_end_l[1]) * sin_h
                _wy = cy + float(_end_l[0]) * sin_h + float(_end_l[1]) * cos_h
                _modes.append((_wx, _wy, float(_w[_m])))      # freeze-1q: certainty = reweighted w
            self.last_mtr_modes[int(tid)] = _modes""", tag="stash")
rep("""            all_modes_world = [_to_world(pred_trajs[i, m, :, :2])
                               for m in range(n_modes)]
            mode_scores = [float(pred_scores[i, m]) for m in range(n_modes)]""",
    """            # freeze-1q: modes ordered by the selection rule (index 0 = consumed
            # best, 1 = second) and scored by the reweighted certainty, so a
            # consumer that takes the first two or sorts by score sees the same pair.
            all_modes_world = [_to_world(pred_trajs[i, m, :, :2]) for m in _order]
            mode_scores = [float(_w[m]) for m in _order]""", tag="all modes")
rep("""                ot, cur_tf, probability=float(pred_scores[i, best_mode[i]]),""",
    """                ot, cur_tf, probability=float(_w[best_mode[i]]),""", tag="probability")
# class constants for the rule
rep("""    _PAD_TID = -777          # freeze-1q: reserved id of the padding dummy""",
    """    _RANK_SIGMA = 3.0        # m/s, certainty reweighting (stated, not fitted)
    _RANK_VMIN = 2.0         # m/s, below this the raw score order is kept (stationary track)
    _PAD_TID = -777          # freeze-1q: reserved id of the padding dummy""", tag="consts")
open(f"{SP}/mtr_edge_predictor_1q.py","w").write(s)
print("written mtr_edge_predictor_1q.py (prepared, not applied)")
