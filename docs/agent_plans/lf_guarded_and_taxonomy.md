# LF-guarded arm + brake taxonomy wiring

Status: Change 1 (stale-track suppression) APPLIED 2026-05-31 and py_compile-clean
(see VERIFIED IMPLEMENTATION section, Edits A/B/C). Default OFF. Change 2 (taxonomy
column) still pending. Smoke test next.

Target file (unless noted):
`ecav/core/application/edge/edge_manager/edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py`

## Why

From the validation sweep (experiment_results/openscenario_3_edge_late_fusion_smoke/20260531_154919):

- AoI knob verified: measured `delta_use_ticks` scales 1/3/5/7/10 for injected
  latency 0/100/200/300/450 ms (50 ms/tick + 1-tick inherent AoI).
- SBA removes self-ghosting: off-arm lat0 = 17 self_ghost episodes, on-arm = 0 in
  all cells.
- The physics boundary is MASKED. SBA-on has 0 collisions through 450 ms, and
  P[S_op] (0.90/0.40/0.70/1.00/0.90) tracks `other_fp` brake count, not AoI.
- The masking FP is the stale Tesla echo (external_stale class), proven directly:
  every other_fp brake sits at the conflict point (-84,127); matched-actor speed
  is 13.4 m/s = the cross-traffic Tesla; it matches a parked/non-hazard actor
  because the Tesla already cleared. Count rises 5 -> 42 as AoI grows (echo coasts
  longer). This is NOT ego self-ghosting and NOT a tracker crash; it is a coasting
  AB3DMOT track that keeps being published after its source detection stopped.

So the claim "SBA makes LF track Oracle until the physics boundary" is unsupported
until the coasting-echo FP is removed (LF-guarded) or separated (taxonomy).

## Change 1 (operational, the actual unblocker): stale-track collision suppression

Goal: a coasting track (no fresh detection for >= N ticks) is withheld from the
published collision-relevant predictions. Config-gated so LF-basic vs LF-guarded is
one toggle, orthogonal to SBA. This is the standard LF guardrail the professor
asked for, not a new method.

Mechanism is single-file and does NOT change the AB3DMOT row format (so no other
edge manager is affected). AB3DMOT keeps per-track state on
`self.mot_tracker.trackers` (each a KalmanBoxTracker with `.time_since_update`,
`.hits`, `.id`); the published track id is `trk.id + 1` (model.py:197). A track
matched this tick has `time_since_update == 0`, so an actively-detected real
obstacle is never suppressed (satisfies prof safety-critical check #4).

### 1a. __init__ config read

ACCESSOR UNVERIFIED. On 2026-05-31 the read/display layer was fabricating file
content (it echoed this spec's own lines back as if they were in the source). The
config-accessor text it showed (`ec = config['edge'] ...`, `self.min_hits = int(ec.get...)`)
is FALSE: byte-level `in`-checks on the real file returned
`ec = config['edge']` = False and `self.min_hits = int(ec.get` = False. So do NOT
trust any quoted source lines in this doc for the LF manager. RE-DERIVE the real
config accessor with clean tooling before editing.

Reliably-true facts (verified by byte-count checks on the real file, not content reads):
- `self.cfg` does NOT exist (count 0).
- `stale_track_suppression` does NOT exist yet (count 0) — 1a must be ADDED.
- `self.mot_tracker` exists (True).
- `self._predict_one(` is called exactly 3 times.
- AB3DMOT (`ecav/core/tracking/ab3dmot/model.py`) keeps `time_since_update`, `hits`,
  `id` per `KalmanBoxTracker`; published tid = `trk.id + 1` (standard AB3DMOT; high
  confidence but re-confirm).

To add the config (after the real min_hits/max_age reads, whatever their accessor is):

```python
        # LF-guarded: withhold coasting (stale) tracks from the published
        # collision-relevant prediction set. Off => LF-basic, on => LF-guarded.
        # Orthogonal to SBA/anchoring. Use the SAME config accessor the file uses
        # for min_hits/max_age (re-derive it; it is NOT self.cfg).
        self.stale_track_suppression = bool(<accessor>.get('stale_track_suppression', False))
        self.stale_track_n = int(<accessor>.get('stale_track_n', 2))
```

### 1b. Publish boundary gate

CORRECTED: `_predict_one` has THREE call sites (def at ~603; calls at ~634, ~642,
~869), so the publish path is not one loop. ~642 is
`self._predict_one(traj, horizon=self.pred_horizon)` (main predictions list); ~634
and ~869 are other paths (likely per-ego / SBA). With clean reads, identify which
loop(s) build the per-ego published set and apply the skip there. The most likely
single point is the ~642 loop over `self.tracked_trajectories.items()` that appends
to `predictions`.

Compute the stale id set once, before that loop:

```python
            stale_ids = set()
            if self.stale_track_suppression and self.mot_tracker is not None:
                stale_ids = {trk.id + 1 for trk in self.mot_tracker.trackers
                             if trk.time_since_update >= self.stale_track_n}
```

Inside the loop, skip stale tracks (first statement of the loop body):

```python
                if tid in stale_ids:
                    continue
```

If both the ~642 and the per-ego ~869 loops feed published output, apply the same
`if tid in stale_ids: continue` guard in each. Confirm the loop variable is `tid`
(the rebuild loop at ~1052 uses `tid, rows`; the provenance loop at ~1107 uses
`_tid, _traj` so it will not collide).

Rationale for fully withholding (vs flag-only): for a baseline guardrail, an
unconfirmed coasting track should not reach the vehicle planner at all. SBA stays a
separate toggle so the four curves are clean.

### 1c. Config / YAML

Add to the late-fusion edge config(s) under the edge entry, default false:

```yaml
    stale_track_suppression: false   # LF-basic
    stale_track_n: 2
```

LF-guarded runs flip `stale_track_suppression: true`.

## Change 2 (diagnostic, optional): make the brake taxonomy column populate

`gt_provenance_class` is empty in all cells because provenance is only appended when
`_gt_snap = self._gt_snapshots.get(self._latest_source_tick)` is non-empty
(LF ~line 1100), and `_latest_source_tick` (set ~552) is likely not a key of
`_gt_snapshots` (stored by `frame_idx` ~423). The id spaces DO match
(`obstacle.track_id == tid == _track_provenance key`), so once the snapshot lookup
resolves, `_label_brake_attributions_gt` (base ~504-516) will classify correctly.

Robust fix (decouples labeling from deque liveness): when a brake attribution is
recorded, snapshot the triggering track's provenance history onto the attr.
- behavior_agent.py:789-790 builds the attr with
  `'trigger_track_id': pred.obstacle_trajectory.obstacle.track_id`. Add
  `'provenance_hist': list(getattr(pred.obstacle_trajectory.obstacle, 'provenance_hist', []))`
  IF we also stamp the deque onto the published obstacle.
- Simpler: fix the snapshot key. Change LF ~line 1100 to use the same snapshot the
  rest of the pipeline uses (e.g. `self._gt_snapshots.get(tick) or
  self._gt_snapshots.get(self._latest_source_tick)`, mirroring the working pattern
  at ~728-729), and verify `_gt_snap` is non-empty with a one-run debug log.

Note: Change 2 is NOT required for the paper's diagnosis. The external_stale class
is already established from the raw data (obs location + matched speed). Change 2
only automates per-run labeling.

## Verification (smoke, 2 runs)

After Change 1, with `stale_track_suppression: true`:
1. lat0: ego must NOT crawl. Confirm avg_speed_mps recovers vs LF-basic and the
   conflict-point other_fp brakes drop toward 0.
2. lat450: confirm whether a collision now appears (boundary reached) or S_op stays
   clean. At the observed brake distances (12-17 m), analytic tau_max ~= 0.04-0.42 s,
   so the cliff should appear in/just past this range once the echo FP is gone.

Then run the four curves at the bounded latency set:
Oracle / LF-basic / LF-guarded / LF-guarded+SBA, plus 1-RSU I2V and 2-RSU I2V.

## Open item from the sweep

Off-arm cells lat 100/200/300/450 (40 runs) were killed mid-sweep and still need
re-running; on-arm (50) and off/lat0 survived in run 20260531_154919.

## Multi-source framing (paper)

Self-ghosting is a multi-source edge-publication identity failure, not a vehicle-
uplink artifact. Axis is single-source vs multi-source edge publication, not
V2X2V vs I2V. Baseline matrix: 1-RSU I2V (clean, hits boundary), 2-RSU I2V (tests
whether multi-source infra alone creates duplicate ego-consistent tracks),
Vehicle+RSU LF, LF+SBA / I2V+SBA, Oracle. If LF+SBA still shows non-ego FPs, the
claim becomes: SBA removes ego-identity failures; an unoptimized LF stack can still
fail from non-ego merges; the envelope exposes both regimes.

## VERIFIED IMPLEMENTATION (2026-05-31, reliable reads, cross-checked vs byte tests)

This supersedes the uncertain anchors above. All facts below were confirmed both by
file content AND by `substring in open(file).read()` byte checks (the read layer was
fabricating content earlier this session, so only byte-cross-checked facts are trusted).

File: edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py

Verified:
- Tracker object is `self.tracker` (line 272: `self.tracker = AB3DMOT(self.mot_cfg, ...)`),
  imported from `AB3DMOT_libs.model` (line 26). NOT `self.mot_tracker`.
- Per-track coast state: `self.tracker.trackers` is a list of KalmanBoxTracker, each
  with `.time_since_update` and `.id`. Published track id = `trk.id + 1`
  (AB3DMOT_libs/model.py:181-197 output loop). `ab3dmot_wrapper.py` does NOT expose
  this; the lib object does.
- Config is read from the local `cfg` param via `cfg.get(...)` in __init__
  (lines 229/243/265/323). There is NO `self.cfg`.
- There is NO `_predict_one` method (byte count 0). Predictions come from
  `self.predictor` / `self.lin_pred` (LinearPredictorManager) and live in the `preds`
  list before the publish boundary (~line 656).
- The suppression mechanism is INDEX-based per ego:
  `ego_suppress_sets = {}  # {vehicle_id: set of pred indices}` (line 675), built
  inside `if self.anchoring:` (line 683), and applied at lines 935-947:
  `suppress_set = ego_suppress_sets.get(vm.vehicle.id, set())` then
  `ego_preds = [p for i,p in enumerate(preds) if i not in suppress_set]`.
- `preds[i].obstacle_trajectory.obstacle.track_id` is the tid; `.carla_id`,
  `.location`, `.kf_speed_mps` also available.

### Edit A — __init__ config (add near line 265, beside `self.anchoring = cfg.get(...)`)

```python
        self.stale_track_suppression = bool(cfg.get('stale_track_suppression', False))
        self.stale_track_n = int(cfg.get('stale_track_n', 2))
```

### Edit B — stale-id set, anchoring-INDEPENDENT, computed once before the per-ego
publish loop (i.e. before/around line 675, OUTSIDE the `if self.anchoring:` block so
LF-guarded works without SBA):

```python
        stale_pred_idx = set()
        if self.stale_track_suppression:
            stale_tids = {trk.id + 1 for trk in self.tracker.trackers
                          if trk.time_since_update >= self.stale_track_n}
            stale_pred_idx = {
                i for i, p in enumerate(preds)
                if getattr(p.obstacle_trajectory.obstacle, 'track_id', None)
                in stale_tids}
```

### Edit C — fold stale_pred_idx into every per-ego suppress_set at line ~936:

Replace:
```python
                    suppress_set = ego_suppress_sets.get(vm.vehicle.id, set())
```
with:
```python
                    suppress_set = ego_suppress_sets.get(vm.vehicle.id, set()) | stale_pred_idx
```

This withholds coasting tracks from every ego regardless of anchoring, so:
- LF-basic: stale_track_suppression=false -> stale_pred_idx empty -> unchanged.
- LF-guarded: stale_track_suppression=true -> coasting echoes withheld, SBA off.
- LF-guarded+SBA: both on -> ego_suppress_sets (SBA) UNION stale_pred_idx.

Safety (prof check #4): an actively-detected real obstacle has time_since_update==0,
so it is never in stale_tids -> never withheld. Only coasting tracks (no fresh
detection for >= N ticks) are withheld.

### Verify after editing
```
python -c "import py_compile,sys; py_compile.compile('<file>',doraise=True)"
grep -c stale_track_suppression <file>   # expect 3 (init bool, init n? -> 2 + gate) 
```
Then smoke 2 runs (lat0, lat450) with stale_track_suppression true: ego must not
crawl at lat0; check whether a collision appears at lat450.

## NEXT TASK (queued, user request 2026-05-31): SSM / Mamba3DMOT tracker
Re-run the same scenario_3 / 40 km/h conflict with the Mamba3DMOT (SSM) tracker and
check whether the coasting external-stale echo (and any ego self-echo) appears there
too. Tests whether the masking FP is AB3DMOT-specific or general to object-level
tracking. The migration harness already has Mamba3DMOT tracker state transfer
(commit 3bd09544), so the tracker is runnable. Compare other_fp brake count + ego
crawl at the conflict point vs the AB3DMOT LF-basic numbers (5/42/15/0/2).
