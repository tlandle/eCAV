import re, sys
p="/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad/branching/runner_branch.py"
s=open(p).read()
def rep(old, new, n=1, tag=""):
    global s
    assert s.count(old)==n, f"anchor {tag!r}: found {s.count(old)} expected {n}"
    s=s.replace(old,new)
# (a) scenario name from the file name; branching knobs
rep("SCENARIO_NAME = 'openscenario_1_flow_gt'\n",
    "SCENARIO_NAME = os.path.splitext(os.path.basename(__file__))[0]  # branching: copied per site\n", tag="name")
rep("MIRROR_PERIOD_S = float(os.environ.get('MIRROR_PERIOD_S', 0.0))\n",
'''MIRROR_PERIOD_S = float(os.environ.get('MIRROR_PERIOD_S', 0.0))
# ── Branching-junction harness (NSDI experiment 1). The source locale has k
# candidate destinations (yaml locale.neighbors; default: every other locale, which
# reproduces the two-locale runner exactly). Each arm decides a FIRE SET at its
# trigger tick; prepares go to every locale in the set; the transferred record is
# identical across arms (payload hash in HANDOFFROW).
BRANCH_TAKEN = os.environ.get('BRANCH_TAKEN', '')            # locale the scripted actor takes (oracle + scoring)
REPLICATE_ALL = os.environ.get('REPLICATE_ALL', '0') == '1'   # fire set = all candidates (brute-force arm)
MTR_TOP1 = os.environ.get('MTR_TOP1', '0') == '1'             # mtr: argmax MODE's branch only; same tick and L
CONF_LOG = os.environ.get('CONF_LOG', '1') == '1'             # [CONFROW] per tick in a branching source
CONFLICT_R_M = 3.0                                            # actor within this of the branch conflict point = conflict tick
''', tag="knobs")
# (b) bookkeeping
rep("    npc_mirror_last_tick = {}  # carla_id -> tick of the last mirror resend\n",
'''    npc_mirror_last_tick = {}  # carla_id -> tick of the last mirror resend
    # branching: per-(npc, destination) bookkeeping; t19_first_dst / t19_first_use
    # are keyed (nid, locale_id) below.
    npc_prepare_ticks = {}     # (nid, dst_lid) -> first prepare tick
    npc_prepare_hash = {}      # (nid, dst_lid) -> payload hash of that prepare
    npc_conflict_tick = {}     # nid -> tick within CONFLICT_R_M of the entered branch's conflict point
    npc_entered = {}           # nid -> (tick, locale_id) first candidate locale containing the actor
    npc_trigger_tick = {}      # nid -> tick the arm's trigger fired
    npc_cv_pick = {}           # nid -> constant-velocity projected branch at the trigger tick
''', tag="dicts")
# (c) neighbors / conflict points after the router
rep('''        link = InterLocaleLink(edge_list[0].latency_model)
''','''        link = InterLocaleLink(edge_list[0].latency_model)
        # branching: candidate destinations per source locale + per-branch conflict points
        neighbors_by_locale, conflict_by_locale = {}, {}
        for ec in edge_cfgs:
            if 'locale' not in ec:
                continue
            _lid = str(ec['locale']['id'])
            _nb = ec['locale'].get('neighbors')
            neighbors_by_locale[_lid] = [str(n) for n in _nb] if _nb else \\
                [l for l in locale_by_id if l != _lid]
            _cp = ec['locale'].get('conflict_point')
            if _cp is not None:
                conflict_by_locale[_lid] = (float(_cp[0]), float(_cp[1]))
        K_BRANCH = max((len(v) for v in neighbors_by_locale.values()), default=1)
        logger.info("[BRANCHCFG] taken=%s k=%d replicate_all=%d top1=%d theta=%.2f candidates=%s",
                    BRANCH_TAKEN, K_BRANCH, int(REPLICATE_ALL), int(MTR_TOP1), MTR_THETA,
                    {k: v for k, v in neighbors_by_locale.items()})
''', tag="router")
# (d) hero binding: all heroes, XML order (= actor id order), once every actor exists
rep('''        while ego_vehicle is None or num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            for vehicle in vehicles:
                if vehicle.attributes['role_name'] == 'hero' and ego_vehicle is None:
                    print("Ego vehicle found")
                    ego_vehicle = vehicle
            num_actors = len(vehicles) + len(walkers)
        print(f'Found all {num_actors} actors')
''','''        while num_actors < scenario_params.scenario_runner.num_actors:
            print("Waiting for the actors")
            time.sleep(2)
            vehicles = world.get_actors().filter('vehicle.*')
            walkers = world.get_actors().filter('walker.*')
            num_actors = len(vehicles) + len(walkers)
        # branching: k heroes (one recipient CAV per branch). Scenario runner spawns
        # the XML ego list in order, so actor-id order is XML order; sim_api binds
        # hero[i] to edge_list[0]['vehicles'][i] via member_vehicles.
        _heroes = sorted([v for v in vehicles if v.attributes['role_name'] == 'hero'],
                         key=lambda v: v.id)
        assert _heroes, "no hero vehicle found"
        ego_vehicle = _heroes[0] if len(_heroes) == 1 else _heroes
        for _h in _heroes:
            _hl = _h.get_transform().location
            logger.info("[HEROROW] id=%d x=%.2f y=%.2f yaw=%.1f", _h.id, _hl.x, _hl.y,
                        _h.get_transform().rotation.yaw)
        print(f'Found all {num_actors} actors; heroes {[h.id for h in _heroes]}')
''', tag="heroes")
rep("        spectator = ego_vehicle.get_world().get_spectator()\n",
    "        spectator = world.get_spectator()\n", tag="spectator")
# (e) T19 firsts -> per destination; crossing into ANY candidate; conflict tick; first use per locale
old_t19 = s[s.index("            # T19 firsts (cheap polls, only until each first is recorded)"):s.index("            # Per-tick snapshot upload (parity with Scenario A; keeps the store warm).")]
new_t19 = '''            # T19 firsts, branching form: first destination track per CANDIDATE
            # locale, crossing = first candidate that contains the actor (commit
            # there), conflict tick, first use per locale (the recipient CAV of
            # that locale consuming a forecast for the actor).
            for _nid3 in list(npc_ids):
                _src3 = npc_locale.get(_nid3)
                _cands3 = neighbors_by_locale.get(_src3, []) if _src3 else []
                for _dl3 in _cands3:
                    if (_nid3, _dl3) in t19_first_dst:
                        continue
                    _dst3 = edge_by_locale.get(_dl3)
                    if _dst3 is not None and any(
                            _c == _nid3 for _c in
                            getattr(_dst3, 'track_to_carla', {}).values()):
                        t19_first_dst[(_nid3, _dl3)] = step
                _a3 = world.get_actor(_nid3)
                if _a3 is None:
                    continue
                _l3 = _a3.get_transform().location
                if _nid3 not in npc_entered:
                    for _dl3 in _cands3:
                        if locale_by_id[_dl3].contains((_l3.x, _l3.y)):
                            npc_entered[_nid3] = (step, _dl3)
                            t19_crossing[_nid3] = step
                            # COMMIT (paper 3.5): the actor entered a destination
                            # locale -> its imported track (if any) becomes
                            # publishable there. Arm-independent.
                            _de3 = edge_by_locale.get(_dl3)
                            if _de3 is not None and hasattr(_de3, '_shadow_obstacles'):
                                _de3._shadow_obstacles[int(_nid3)] = False
                            logger.info("[ENTERROW] tick=%d npc=%d entered=%s taken=%s",
                                        step, _nid3, _dl3, BRANCH_TAKEN)
                            break
                if _nid3 in npc_entered and _nid3 not in npc_conflict_tick:
                    _cp3 = conflict_by_locale.get(npc_entered[_nid3][1])
                    if _cp3 is not None and ((_l3.x - _cp3[0]) ** 2 +
                                             (_l3.y - _cp3[1]) ** 2) ** 0.5 <= CONFLICT_R_M:
                        npc_conflict_tick[_nid3] = step
                for _lid3, _e3 in edge_by_locale.items():
                    if (_nid3, _lid3) in t19_first_use:
                        continue
                    _hit3 = False
                    for _vm3 in _e3.vehicle_manager_list:
                        for _p3 in getattr(_vm3.agent, 'generated_predictions', []):
                            _o3 = _p3.obstacle_trajectory.obstacle
                            if getattr(_o3, 'carla_id', -1) == _nid3:
                                _hit3 = True
                                break
                        if _hit3:
                            break
                    if _hit3:
                        t19_first_use[(_nid3, _lid3)] = step

'''
s = s.replace(old_t19, new_t19)
# (f) advance-warning unpack
rep("                        htick, dst_lid = npc_handoff_done[nid]\n",
    "                        htick, _src_h, _dsts_h = npc_handoff_done[nid]\n                        dst_lid = BRANCH_TAKEN if BRANCH_TAKEN in _dsts_h else (_dsts_h[0] if _dsts_h else None)\n", tag="advwarn")
# (g) resend block: entered locale, final sync only where prepared
rep('''                        _htick, _dst_lid = npc_handoff_done[nid]
                        _src_lid2 = next(
                            (l for l in locale_by_id if l != _dst_lid), None)
                        _se = edge_by_locale.get(_src_lid2)
                        _de = edge_by_locale.get(_dst_lid)
                        _crossed = locale_by_id[_dst_lid].contains(nxy)
''','''                        _htick, _src_lid2, _dsts2 = npc_handoff_done[nid]
                        _ent2 = npc_entered.get(nid)
                        # final sync goes to the locale the actor ENTERED, and only
                        # if that locale was prepared (a wrong-destination prepare
                        # has nothing to update; the miss shows in READYROW).
                        _dst_lid = _ent2[1] if _ent2 is not None else (_dsts2[0] if _dsts2 else None)
                        _se = edge_by_locale.get(_src_lid2)
                        _de = edge_by_locale.get(_dst_lid) if _dst_lid else None
                        _crossed = _ent2 is not None and _dst_lid in _dsts2
''', tag="resend")
# (h) trigger block: from dst_lid resolution to the end of the prepare/log
i0 = s.index("                dst_lid = next((l for l in locale_by_id if l != src_lid), None)\n")
i1 = s.index('''                        "no track on %s yet",
                        step, nid, nxy[0], nxy[1], src_lid)
''') + len('''                        "no track on %s yet",
                        step, nid, nxy[0], nxy[1], src_lid)
''')
new_trig = '''                cands = neighbors_by_locale.get(src_lid) or \\
                    [l for l in locale_by_id if l != src_lid]
                if not cands:
                    continue
                nvel = actor.get_velocity()
                if step % 5 == 0:
                    logger.info("[NPCROW] tick=%d npc=%d x=%.2f y=%.2f v=%.2f src=%s",
                                step, nid, nxy[0], nxy[1],
                                (nvel.x ** 2 + nvel.y ** 2) ** 0.5, src_lid)
                # Lead L, computed exactly as the two-locale computed trigger
                # (transfer EMA + fold-in + margin, capped); shared by mtr/top1.
                _xfer_s = getattr(run_scenario, '_xfer_ema_s', 0.003)
                _fold_s = 3 * 0.2
                _lead_c = min(2.5, max(_fold_s + 0.35, _xfer_s + _fold_s + 0.35))

                def _cv_traj(_lead):
                    _ns = int(_lead / world_dt) + 1
                    _ta = np.arange(_ns, dtype=np.float64) * world_dt
                    return np.column_stack([nloc.x + nvel.x * _ta,
                                            nloc.y + nvel.y * _ta])

                def _cv_branch():
                    """Candidate the constant-velocity projection enters first
                    (nearest candidate to the 6 s point if it enters none)."""
                    _t = _cv_traj(6.0)
                    for _p in _t:
                        for _c in cands:
                            if locale_by_id[_c].contains(_p):
                                return _c, 'contains'
                    _far = _t[-1]
                    return min(cands, key=lambda _c: locale_by_id[_c].signed_distance(_far)), 'nearest'

                def _mtr_probs():
                    """Per-candidate destination probability from the SOURCE edge's
                    predictor mode stash (the same object the trigger reads):
                    mode probabilities summed by the candidate containing the
                    mode's world-frame endpoint. Also the top mode's branch."""
                    _se_m = edge_by_locale.get(src_lid)
                    _pred = getattr(_se_m, 'predictor', None) if _se_m is not None else None
                    _mm = getattr(_pred, 'last_mtr_modes', {}) if _pred is not None else {}
                    _tid = next((t for t, c in getattr(_se_m, 'track_to_carla', {}).items()
                                 if c == nid), None) if _se_m is not None else None
                    _modes = _mm.get(int(_tid)) if _tid is not None else None
                    if not _modes:
                        return None, None, 0
                    _pc = {_c: 0.0 for _c in cands}
                    for (wx, wy, p) in _modes:
                        for _c in cands:
                            if locale_by_id[_c].contains((wx, wy)):
                                _pc[_c] += p
                                break
                    _top = max(_modes, key=lambda m: m[2])
                    _topc = next((_c for _c in cands
                                  if locale_by_id[_c].contains((_top[0], _top[1]))), None)
                    return _pc, _topc, len(_modes)

                # CONFROW every tick while the actor is in a branching source, from
                # the same predictor instance every arm's trigger reads.
                if len(cands) >= 2 and CONF_LOG:
                    _pc, _topc, _nm = _mtr_probs()
                    _cvb, _cvr = _cv_branch()
                    if _pc is not None:
                        _am = max(_pc, key=_pc.get)
                        logger.info(
                            "[CONFROW] tick=%d npc=%d src=%s taken=%s n_modes=%d argmax=%s "
                            "p_argmax=%.3f top_mode_branch=%s cv_branch=%s %s",
                            step, nid, src_lid, BRANCH_TAKEN, _nm, _am, _pc[_am], _topc, _cvb,
                            " ".join("p[%s]=%.3f" % (_c, _pc[_c]) for _c in cands))
                    else:
                        logger.info("[CONFROW] tick=%d npc=%d src=%s taken=%s n_modes=0 cv_branch=%s",
                                    step, nid, src_lid, BRANCH_TAKEN, _cvb)

                # ── Trigger tick + fire set per arm. Only the DECISION varies. ──
                fire, _rule = None, ''
                if TRIGGER_MODE == 'band' and MIGRATION_MODE not in (
                        "reactive", "handover_snapshot"):
                    _cvb, _ = _cv_branch()
                    if locale_by_id[_cvb].signed_distance(nxy) > BAND_W_M:
                        continue
                    fire, _rule = [_cvb], 'band'
                elif TRIGGER_MODE == 'mtr':
                    # Khonsu: trigger tick = L gate AND some candidate at/above
                    # theta; fire set = every candidate at/above theta. MTR_TOP1
                    # shares the SAME tick and L and fires only the top mode's
                    # branch (destination selection differs, timing never).
                    _pc, _topc, _nm = _mtr_probs()
                    if _pc is None or max(_pc.values()) < MTR_THETA:
                        continue
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _cv_traj(_lead_c), _lead_c, world_dt):
                        continue
                    if MTR_TOP1:
                        fire = [_topc] if _topc is not None else [max(_pc, key=_pc.get)]
                        _rule = 'top1'
                    else:
                        fire = [_c for _c in cands if _pc[_c] >= MTR_THETA]
                        _rule = 'theta'
                elif TRIGGER_MODE == 'oracle':
                    # Knows the taken branch; timing = L before the GT-velocity
                    # projected exit, as the two-locale oracle.
                    _L = 3 * 0.2 + 0.35
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _cv_traj(_L), _L, world_dt):
                        continue
                    fire = [BRANCH_TAKEN] if BRANCH_TAKEN in cands else [_cv_branch()[0]]
                    _rule = 'oracle'
                elif MIGRATION_MODE in ("reactive", "handover_snapshot"):
                    # At-crossing: fire when the actor is inside a candidate; the
                    # destination is the cell being entered (EdgeWarp radio trigger).
                    _ent = [_c for _c in cands if locale_by_id[_c].contains(nxy)]
                    if not _ent:
                        continue
                    fire, _rule = [_ent[0]], 'crossing'
                else:
                    if TRIGGER_MODE == 'computed':
                        _lead = _lead_c
                        logger.info(
                            "[LEADROW] npc=%d lead=%.3f xfer_ema=%.3f "
                            "fold=%.3f margin=0.350 tick=%d",
                            nid, _lead, _xfer_s, _fold_s, step)
                    else:
                        _lead = OBSTACLE_HANDOFF_LOOKAHEAD_S
                    if not locale_by_id[src_lid].predicted_to_exit_within(
                            _cv_traj(_lead), _lead, world_dt):
                        continue
                    _cvb, _cvr = _cv_branch()
                    fire, _rule = [_cvb], 'cv_' + _cvr
                if REPLICATE_ALL:
                    fire, _rule = list(cands), _rule + '+replicate_all'
                src_edge = edge_by_locale.get(src_lid)
                if src_edge is None or not fire:
                    continue
                npc_trigger_tick.setdefault(nid, step)
                npc_cv_pick.setdefault(nid, _cv_branch()[0])
                _done_any = False
                for _dl in fire:
                    dst_edge = edge_by_locale.get(_dl)
                    if dst_edge is None or dst_edge is src_edge:
                        continue
                    cost = None if MIGRATION_MODE == "cold" \\
                        else daemon.transfer_obstacle_state(
                            nid, src_edge, dst_edge, link, step, position=nxy)
                    if cost is None:
                        continue
                    _done_any = True
                    scenario_manager.record_handoff_cost(cost)
                    _prev = getattr(run_scenario, '_xfer_ema_s', None)
                    _cur_s = cost.total_ms / 1000.0
                    run_scenario._xfer_ema_s = _cur_s if _prev is None \\
                        else 0.3 * _cur_s + 0.7 * _prev
                    npc_prepare_ticks.setdefault((nid, _dl), step)
                    npc_prepare_hash.setdefault((nid, _dl), getattr(cost, 'payload_hash', ''))
                    logger.info(
                        "[SCENB] PREDICTIVE OBSTACLE HANDOFF tick=%d "
                        "carla_id=%d npc=(%.1f,%.1f) %s->%s bytes=%d "
                        "total_ms=%.3f rule=%s hash=%s",
                        step, nid, nxy[0], nxy[1], src_lid, _dl,
                        cost.payload_bytes, cost.total_ms, _rule,
                        getattr(cost, 'payload_hash', ''))
                if _done_any or MIGRATION_MODE == "cold":
                    npc_handoff_done[nid] = (
                        step, src_lid, [d for d in fire if (nid, d) in npc_prepare_ticks])
                    logger.info("[FIREROW] tick=%d npc=%d src=%s rule=%s fire=%s prepared=%s taken=%s",
                                step, nid, src_lid, _rule, ",".join(fire),
                                ",".join(npc_handoff_done[nid][2]) or "none", BRANCH_TAKEN)
                elif step % 10 == 0:
                    logger.warning(
                        "[SCENB-DBG] predictive tick=%d npc=%d (%.1f,%.1f) — "
                        "no track on %s yet",
                        step, nid, nxy[0], nxy[1], src_lid)
'''
s = s[:i0] + new_trig + s[i1:]
# the old block computed nvel after dst_lid; make sure no stale 'nvel = actor.get_velocity()' remains before the trigger comment
assert s.count("                nvel = actor.get_velocity()\n") == 1, "nvel occurrences"
# (i) ego = the taken branch's CAV
rep('''            ego_cav = None
            ego_vm = None
            for edge in edge_list:
                if edge.vehicle_manager_list:
                    ego_vm = edge.vehicle_manager_list[0]
                    ego_cav = ego_vm.vehicle
                    break
''','''            ego_cav = None
            ego_vm = None
            _tk_edge = edge_by_locale.get(BRANCH_TAKEN) if BRANCH_TAKEN else None
            if _tk_edge is not None and _tk_edge.vehicle_manager_list:
                ego_vm = _tk_edge.vehicle_manager_list[0]   # recipient CAV of the taken branch
                ego_cav = ego_vm.vehicle
            for edge in edge_list:
                if ego_cav is not None:
                    break
                if edge.vehicle_manager_list:
                    ego_vm = edge.vehicle_manager_list[0]
                    ego_cav = ego_vm.vehicle
''', tag="ego")
# (j) end rows
old_rows = s[s.index("            for _nid4, (_ht4, _dl4) in npc_handoff_done.items():"):s.index("            import os as _os2\n")]
new_rows = '''            for (_nid4, _dl4), _pt4 in sorted(npc_prepare_ticks.items()):
                _fd = t19_first_dst.get((_nid4, _dl4), -1)
                _fu = t19_first_use.get((_nid4, _dl4), -1)
                _cx = t19_crossing.get(_nid4, -1)
                _warm = (_fd >= 0 and (_fu < 0 or _fd <= _fu)
                         and (_cx < 0 or _fd <= _cx))
                logger.info(
                    "[HANDOFFROW] npc=%d prepare_tick=%d crossing_tick=%d "
                    "first_dst_track_tick=%d first_use_tick=%d "
                    "warm_before_first_use=%s dst=%s hash=%s",
                    _nid4, _pt4, _cx, _fd, _fu,
                    "YES" if _warm else "no", _dl4,
                    npc_prepare_hash.get((_nid4, _dl4), ''))
            # READYROW / PLACEROW per actor: readiness at the TAKEN branch,
            # decomposed into wrong_destination and right_destination_too_late.
            for _nid5 in sorted(npc_ids):
                _ent5 = npc_entered.get(_nid5)
                _entl = _ent5[1] if _ent5 else ''
                _tk = BRANCH_TAKEN or _entl
                _prep = sorted(d for (n, d) in npc_prepare_ticks if n == _nid5)
                _fd = t19_first_dst.get((_nid5, _tk), -1)
                _fu = t19_first_use.get((_nid5, _tk), -1)
                _cx = t19_crossing.get(_nid5, -1)
                _cf = npc_conflict_tick.get(_nid5, -1)
                _pt = npc_prepare_ticks.get((_nid5, _tk), -1)
                _ready = (_tk in _prep) and _fd >= 0 and (_fu < 0 or _fd <= _fu) \\
                    and (_cx < 0 or _fd <= _cx)
                _wrong = bool(_prep) and _tk not in _prep
                _late = (_tk in _prep) and not _ready
                _lead_s = ((_cx - _pt) * world_dt) if (_pt >= 0 and _cx >= 0) else float('nan')
                _tavail_s = ((_cf - _fd) * world_dt) if (_fd >= 0 and _cf >= 0) else float('nan')
                logger.info(
                    "[READYROW] npc=%d taken=%s entered=%s route_ok=%s k=%d prepared=%s "
                    "n_prepared=%d prepare_tick=%d trigger_tick=%d crossing_tick=%d "
                    "conflict_tick=%d first_dst_track_tick=%d first_use_tick=%d ready=%s "
                    "wrong_destination=%s right_destination_too_late=%s lead_s=%.3f "
                    "t_avail_s=%.3f cv_pick=%s",
                    _nid5, _tk, _entl or 'none',
                    "YES" if (_entl == _tk) else ("no_entry" if not _entl else "no"),
                    K_BRANCH, ",".join(_prep) or "none", len(_prep), _pt,
                    npc_trigger_tick.get(_nid5, -1), _cx, _cf, _fd, _fu,
                    "YES" if _ready else "no", "YES" if _wrong else "no",
                    "YES" if _late else "no", _lead_s, _tavail_s,
                    npc_cv_pick.get(_nid5, ''))
                logger.info("[PLACEROW] npc=%d k=%d prepared=%s count=%d run_bytes=%d",
                            _nid5, K_BRANCH, ",".join(_prep) or "none", len(_prep), _tbytes)
'''
s = s.replace(old_rows, new_rows)
rep('''                "transfers=%d bytes=%d fault=%s fencing=%s dbl_ticks=%d",
                MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
                MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S,
                _eps, _contact, len(transfer_costs), _tbytes,
                _os2.environ.get('FAULT_MODE', 'none') or 'none',
                _os2.environ.get('FENCING', 'on'), _dblsum)''',
'''                "transfers=%d bytes=%d fault=%s fencing=%s dbl_ticks=%d "
                "k=%d taken=%s replicate_all=%d top1=%d theta=%.2f",
                MIGRATION_MODE, TRIGGER_MODE, BAND_W_M, COMMIT_REFRESH,
                MIRROR_PERIOD_S, OBSTACLE_HANDOFF_LOOKAHEAD_S,
                _eps, _contact, len(transfer_costs), _tbytes,
                _os2.environ.get('FAULT_MODE', 'none') or 'none',
                _os2.environ.get('FENCING', 'on'), _dblsum,
                K_BRANCH, BRANCH_TAKEN, int(REPLICATE_ALL), int(MTR_TOP1), MTR_THETA)''', tag="runrow")
open(p,"w").write(s); print("runner_branch.py patched")
