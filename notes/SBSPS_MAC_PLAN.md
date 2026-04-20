# SB-SPS MAC Model Implementation (v4 — mandatory seed, explicit reset)

**Repo:** `/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox`
**Branch:** `conductor`

## Context

Current `should_drop()` (line 267 of `latency_model.py`) is i.i.d./memoryless. C-V2X PC5 Mode 4 uses SB-SPS where resource collisions persist across consecutive packets until a reselection counter expires → **burst losses**. The burst structure drives tail AoI.

**Goal:** Replace stateless `should_drop()` with a per-sender SB-SPS state machine for PC5 vehicle→RSU uplink. MAC controls *delivery/drop*; existing `LatencyModel` controls *delay shape* for delivered packets. Validate standalone model against ns-3 offline.

---

## Architecture

```
                    ┌──────────────┐
  sender_ids ──────>│  SbSpsMac    │──── delivered_set ──┐
  (per tick)        │ (tick-cached) │                     │
                    └──────────────┘                     v
                                              LatencyModel.stamp()
                                              JitterBuffer.push()
```

**Key invariants:**
1. **One MAC per channel per run** — all edge managers in `edge_list` share the same `SbSpsMac` instance via module-level registry
2. **Tick-idempotent** — second call same tick returns cached result, tolerates subset queries
3. **PC5 access hop only** — SB-SPS governs `L_access` (vehicle → roadside access node); `L_backhaul` (access node → MEC/edge) uses existing LatencyModel
4. **Deterministic** — seeded `random.Random(seed)`, no global RNG. Seed is **mandatory** (derived from `--seed` in test_runner if not in YAML)

**Process model:** Each test_runner sweep run is a separate subprocess (`subprocess.Popen` at `test_runner.py:462`). Manager types are swept as separate runs, NOT concurrent managers. Within a run, `edge_list` typically has 1 entry, but the shared registry handles multiple.

---

## Prerequisite: Remove VIPS Infrastructure-Only (dead code)

`VIPSTemporalEdge` is the proper VIPS implementation. The infra-only `VIPSEdge` is unused dead code — not imported in `__init__.py`, not in the registry.

**Delete these 3 files:**
- `ecav/core/application/edge/edge_manager/edge_manager_vips_ab3dmot_linear_predictor.py`
- `ecav/scenario_testing/openscenario_3_edge_vips.py`
- `ecav/scenario_testing/config_yaml/openscenario_3_edge_vips.yaml`

No other files reference these. The `__init__.py` registry already maps "VIPS"→`VIPSTemporalEdge`.

---

## File 1: `ecav/core/application/edge/latency/mac_model.py` (NEW ~250 lines)

### MACMetrics
```python
class MACMetrics:
    """PRR + burst tracking with histogram storage (bounded)."""
    def __init__(self):
        self.attempted = 0
        self.delivered = 0
        self.collision_drops = 0
        self.fading_drops = 0
        self._burst_histogram: Dict[int, int] = {}  # {length: count}
        self._current_burst: Dict[int, int] = {}    # {sender_id: run_length}
        self._finalized = False

    def record(self, sender_id: int, delivered: bool, drop_cause: str = None):
        self.attempted += 1
        if delivered:
            self.delivered += 1
            # Close any in-progress burst for this sender
            if sender_id in self._current_burst:
                bl = self._current_burst.pop(sender_id)
                capped = min(bl, 200)
                self._burst_histogram[capped] = self._burst_histogram.get(capped, 0) + 1
        else:
            if drop_cause == 'collision':
                self.collision_drops += 1
            elif drop_cause == 'fading':
                self.fading_drops += 1
            self._current_burst[sender_id] = self._current_burst.get(sender_id, 0) + 1

    def finalize(self):
        """Flush in-progress bursts at end-of-run. Idempotent."""
        if self._finalized:
            return
        for sid, bl in self._current_burst.items():
            capped = min(bl, 200)
            self._burst_histogram[capped] = self._burst_histogram.get(capped, 0) + 1
        self._current_burst.clear()
        self._finalized = True

    def prr(self) -> float:
        return self.delivered / max(self.attempted, 1)

    def _weighted_stats(self):
        """Compute mean/p95/p99 from histogram without materializing list."""
        if not self._burst_histogram:
            return 0.0, 0.0, 0.0
        total = sum(self._burst_histogram.values())
        mean = sum(l * c for l, c in self._burst_histogram.items()) / max(total, 1)
        # Scan sorted keys for percentiles
        sorted_lens = sorted(self._burst_histogram.keys())
        cumul = 0
        p95 = p99 = sorted_lens[-1]
        for l in sorted_lens:
            cumul += self._burst_histogram[l]
            if cumul >= total * 0.95 and p95 == sorted_lens[-1]:
                p95 = l
            if cumul >= total * 0.99:
                p99 = l
                break
        return float(mean), float(p95), float(p99)

    def get_summary(self) -> Dict[str, Any]:
        self.finalize()
        mean, p95, p99 = self._weighted_stats()
        return {
            'prr': self.prr(),
            'total_attempted': self.attempted,
            'total_delivered': self.delivered,
            'collision_drops': self.collision_drops,
            'fading_drops': self.fading_drops,
            'burst_histogram': self._burst_histogram,
            'burst_length_mean': mean,
            'burst_length_p95': p95,
            'burst_length_p99': p99,
            'mac_effective_hz_under_20hz_offered': self.prr() * 20,  # effective delivered rate under 20 Hz offered load
            'mac_burstiness_factor': p95 / max(mean, 0.01),
        }
```

### MACModel ABC
```python
class MACModel(ABC):
    @abstractmethod
    def attempt_tick(self, tick: int, sender_ids: List[int]) -> Dict[int, bool]: ...

    @property
    @abstractmethod
    def metrics(self) -> MACMetrics: ...
```

### LegacyMacAdapter
```python
class LegacyMacAdapter(MACModel):
    """Wraps latency_model.should_drop() — exact old behavior.
    Used when no `mac:` YAML block is present."""

    def __init__(self, latency_model: LatencyModel):
        self._lm = latency_model
        self._metrics = MACMetrics()

    def attempt_tick(self, tick: int, sender_ids: List[int]) -> Dict[int, bool]:
        result = {}
        for sid in sender_ids:
            delivered = not self._lm.should_drop()
            result[sid] = delivered
            self._metrics.record(sid, delivered)
        return result

    @property
    def metrics(self): return self._metrics
```

### NullMac
```python
class NullMac(MACModel):
    """Explicit no-loss: all delivered. For `mac.type: null`."""
    def __init__(self):
        self._metrics = MACMetrics()

    def attempt_tick(self, tick, sender_ids):
        for sid in sender_ids:
            self._metrics.record(sid, True)
        return {sid: True for sid in sender_ids}

    @property
    def metrics(self): return self._metrics
```

### SbSpsMac
```python
class SbSpsMac(MACModel):
    """C-V2X PC5 Mode 4 SB-SPS resource scheduling.

    Tick-idempotent: caches result per tick. Subset queries of a
    previously-computed tick return slices without recomputation.
    New senders in same tick raise RuntimeError (call with full set).

    Parameters (3GPP TS 36.321):
        M=20, RC_min=5, RC_max=15, p_keep=0.4, p_loss_base=0.02
    """
    def __init__(self, M=20, RC_min=5, RC_max=15, p_keep=0.4,
                 p_loss_base=0.02, seed: int = 0):
        self._M, self._RC_min, self._RC_max = M, RC_min, RC_max
        self._p_keep, self._p_loss_base = p_keep, p_loss_base
        self._rng = random.Random(seed)  # seed is mandatory (int)
        self._metrics = MACMetrics()
        self._state: Dict[int, Dict] = {}
        self._last_tick: int | None = None
        self._last_result: Dict[int, bool] = {}

    def attempt_tick(self, tick: int, sender_ids: List[int]) -> Dict[int, bool]:
        if self._last_tick == tick:
            # Subset query: return cached results
            missing = [s for s in sender_ids if s not in self._last_result]
            if missing:
                raise RuntimeError(
                    f"SbSpsMac: tick {tick} already computed but new senders "
                    f"{missing} appeared. Call once with full sender set.")
            return {s: self._last_result[s] for s in sender_ids}

        # --- Fresh tick ---
        # 1. Init new senders
        for sid in sender_ids:
            if sid not in self._state:
                self._state[sid] = {
                    'resource': self._rng.randint(0, self._M - 1),
                    'rc': self._rng.randint(self._RC_min, self._RC_max),
                }

        # 2. Resource → senders mapping
        res_users: Dict[int, List[int]] = defaultdict(list)
        for sid in sender_ids:
            res_users[self._state[sid]['resource']].append(sid)

        # 3. Collisions + fading
        result = {}
        for sid in sender_ids:
            r = self._state[sid]['resource']
            collided = len(res_users[r]) > 1
            fading = self._rng.random() < self._p_loss_base
            delivered = not collided and not fading
            result[sid] = delivered
            cause = 'collision' if collided else ('fading' if fading else None)
            self._metrics.record(sid, delivered, drop_cause=cause)

        # 4. Reselection
        for sid in sender_ids:
            st = self._state[sid]
            st['rc'] -= 1
            if st['rc'] <= 0:
                if self._rng.random() >= self._p_keep:
                    st['resource'] = self._rng.randint(0, self._M - 1)
                st['rc'] = self._rng.randint(self._RC_min, self._RC_max)

        self._last_tick = tick
        self._last_result = result
        return result

    @property
    def metrics(self): return self._metrics
```

### Shared Registry + Factory
```python
_GLOBAL_MACS: Dict[str, MACModel] = {}

def reset_global_macs():
    """Call at start of each run to prevent state leakage."""
    _GLOBAL_MACS.clear()

def create_mac_model(cfg: Dict, latency_model: LatencyModel = None,
                     seed: int = 0, channel_id: str = "pc5_uplink") -> MACModel:
    """Factory: one shared instance per (channel_id, mac_config) per run.

    - No `mac:` key → LegacyMacAdapter (wraps should_drop(), exact old behavior)
    - mac.type: null → NullMac (explicit no-loss)
    - mac.type: sbsps → SbSpsMac (shared across all edge managers)
    """
    mac_cfg = cfg.get("mac", None)
    if mac_cfg is None:
        # No mac block → legacy adapter. NOT shared (each manager
        # wraps its own latency_model.should_drop() call).
        return LegacyMacAdapter(latency_model)

    # Build registry key from config content
    mac_sig = json.dumps(mac_cfg, sort_keys=True)
    key = f"{channel_id}:{seed}:{mac_sig}"

    if key in _GLOBAL_MACS:
        return _GLOBAL_MACS[key]

    mac_type = (mac_cfg.get("type", "null") or "null").lower()

    if mac_type in ("null", "none"):
        model = NullMac()
    elif mac_type == "sbsps":
        model = SbSpsMac(
            M=int(mac_cfg.get("M", 20)),
            RC_min=int(mac_cfg.get("RC_min", 5)),
            RC_max=int(mac_cfg.get("RC_max", 15)),
            p_keep=float(mac_cfg.get("p_keep", 0.4)),
            p_loss_base=float(mac_cfg.get("p_loss_base", 0.02)),
            seed=seed,
        )
    else:
        raise ValueError(f"Unknown MAC type: '{mac_type}'")

    _GLOBAL_MACS[key] = model
    return model
```

---

## File 2: `ecav/core/application/edge/latency/__init__.py`

Add exports:
```python
from .mac_model import (MACModel, LegacyMacAdapter, NullMac, SbSpsMac,
                         create_mac_model, reset_global_macs, MACMetrics)
```

---

## File 3: `edge_manager_base.py` (line ~68)

After `self.latency_model = create_latency_model(cfg, world_dt)`:
```python
from ecav.core.application.edge.latency import create_mac_model, reset_global_macs
reset_global_macs()  # idempotent — prevents cross-test contamination
_mac_seed = cfg.get("mac", {}).get("seed", 0)
self.mac_model = create_mac_model(cfg, latency_model=self.latency_model, seed=_mac_seed)
```

**Seed propagation in test_runner:** `patch_yaml()` always writes `mac.seed` from `SEED` (the `--seed` CLI arg, default 0). This ensures reproducibility even if the YAML doesn't specify a seed.

With the shared registry, if two edge managers have the same `mac:` config, they get the same `SbSpsMac` instance → same collision process, tick-idempotent cache returns same results.

---

## Files 4-6: Edge Manager Integration

Replace `should_drop()` loop in Late Fusion, VIPS Temporal, Oracle. All use the same pattern — **no isinstance checks**, polymorphism handles it.

### Late Fusion (`edge_manager_prediction_late_fusion_ab3dmot_linear_predictor.py`, lines 276-283)

**Current:**
```python
beacons = {}
for vm in self.vehicle_manager_list:
    if self.latency_model.should_drop():
        continue
    self._dict_extend(objects, vm.agent.objects)
    beacons[vm.vehicle.id] = (vm.vehicle.get_location(),
                               vm.vehicle.bounding_box.extent)
```

**New:**
```python
beacons = {}
vehicle_ids = [vm.vehicle.id for vm in self.vehicle_manager_list]
mac_delivery = self.mac_model.attempt_tick(frame_idx, vehicle_ids)

for vm in self.vehicle_manager_list:
    vid = vm.vehicle.id
    if not mac_delivery.get(vid, True):
        continue
    self._dict_extend(objects, vm.agent.objects)
    beacons[vid] = (vm.vehicle.get_location(),
                    vm.vehicle.bounding_box.extent)
```

### VIPS Temporal (lines 367-374) — same pattern
### Oracle (lines 298-304) — same pattern (only beacons, no objects)

---

## File 7: MAC Metrics in `evaluate()`

In each edge manager's `evaluate()`, add:
```python
self.mac_model.metrics.finalize()  # flush in-progress bursts (idempotent)
metrics['mac'] = self.mac_model.metrics.get_summary()
```

Since shared MAC, `finalize()` is idempotent (tracks boolean). Multiple managers calling `evaluate()` won't double-flush.

---

## File 8: `test_runner.py`

### New CLI args:
```python
ap.add_argument("--mac-type", type=str, default=None,
                choices=["null", "sbsps"],
                help="MAC model type")
ap.add_argument("--mac-M", nargs="*", type=int, default=[],
                help="MAC resource slot counts to sweep (e.g., 10 20 50)")
ap.add_argument("--mac-p-keep", nargs="*", type=float, default=[],
                help="MAC p_keep values to sweep (e.g., 0.0 0.4 0.8)")
```

### Sweep dimension lists:
```python
MAC_M_LIST = args.mac_M or [None]
MAC_PK_LIST = args.mac_p_keep or [None]
MAC_TYPE = args.mac_type  # single value, not sweep
```

### `patch_yaml()` extension (scalar per run, not list):
```python
def patch_yaml(..., mac_type=None, mac_M=None, mac_p_keep=None, seed=None):
    ...
    if mac_type is not None:
        mac_block = edge.setdefault("mac", {})
        mac_block["type"] = mac_type
        mac_block["seed"] = seed or 0  # always write seed for reproducibility
    if mac_M is not None:
        mac_block = edge.setdefault("mac", {})
        mac_block["M"] = mac_M        # scalar int
    if mac_p_keep is not None:
        mac_block = edge.setdefault("mac", {})
        mac_block["p_keep"] = mac_p_keep  # scalar float
```

### Directory naming:
```python
mac_str = f"mac_{mac_type}" if mac_type is not None else ""
macM_str = f"macM_{mac_M}" if mac_M is not None else ""
macpk_str = f"macpk_{mac_p_keep}" if mac_p_keep is not None else ""
```

### `simulation_metrics.json`:
```python
if mac_type is not None: metrics["config_mac_type"] = mac_type
if mac_M is not None: metrics["config_mac_M"] = mac_M
if mac_p_keep is not None: metrics["config_mac_p_keep"] = mac_p_keep
```

### Sweep loop:
Add `MAC_M_LIST` and `MAC_PK_LIST` to `itertools.product(...)` and `TOTAL_RUNS`.

---

## File 9: `paper1_real_data_plots.py`

### `load_sweep()` additions:
```python
mac = edge.get('mac', {})
run['mac_prr'] = mac.get('prr', None)
run['mac_burst_histogram'] = mac.get('burst_histogram', {})
run['mac_collision_drops'] = mac.get('collision_drops', 0)
run['mac_fading_drops'] = mac.get('fading_drops', 0)
run['mac_burst_mean'] = mac.get('burst_length_mean', None)
run['mac_burst_p95'] = mac.get('burst_length_p95', None)
run['mac_burstiness_factor'] = mac.get('mac_burstiness_factor', None)
run['config_mac_type'] = d.get('config_mac_type', 'null')
run['config_mac_M'] = d.get('config_mac_M', None)
run['config_mac_p_keep'] = d.get('config_mac_p_keep', None)
```

### New figure: `eval_fig_mac_characterization()`
Three-panel:
- **(a)** PRR vs N_ego — lines per M value + dashed ns-3 and analytical overlay
- **(b)** Burst-length CDF per N_ego (from histogram) + geometric(same mean) baseline overlay
- **(c)** Burstiness factor (p95/mean) vs N — proves losses are bursty, not memoryless

---

## NS-3 Validation (Offline — main paper, not appendix-dependent)

Use [Eckermann's ns-3_c-v2x](https://github.com/FabianEckermann/ns-3_c-v2x) + analytical formula from [Todisco et al. 2023](https://arxiv.org/html/2309.16680).

1. Clone + build ns-3_c-v2x separately
2. Run PRR sweeps: N = {2, 4, 8, 16, 32}, M variations
3. `scripts/validate_mac_vs_ns3.py` — compares standalone SB-SPS vs ns-3 CSVs vs analytical formula
4. **Main paper overlay** (not appendix): `eval_fig_mac_characterization` panel (a) includes one dashed ns-3 curve + one dashed analytical curve alongside our model. Sufficient to neutralize "unphysical MAC" criticism. Extra ns-3 config details go in appendix if needed.

---

## Files Summary

| File | Change |
|------|--------|
| VIPS infra-only (3 files) | **DELETE** — dead code, not in registry |
| `latency/mac_model.py` | **NEW** — SbSpsMac, LegacyMacAdapter, NullMac, MACMetrics, registry, factory |
| `latency/__init__.py` | Add MAC exports |
| `edge_manager_base.py` | Add `self.mac_model = create_mac_model(cfg, latency_model, seed)` |
| Late fusion edge manager | Replace `should_drop()` with `mac_model.attempt_tick(frame_idx, ids)` |
| VIPS temporal edge manager | Same replacement |
| Oracle edge manager | Same replacement |
| `test_runner.py` | `--mac-type`, `--mac-M`, `--mac-p-keep` + sweep dims |
| `paper1_real_data_plots.py` | MAC metrics extraction + `eval_fig_mac_characterization()` |
| `scripts/validate_mac_vs_ns3.py` | **NEW** — offline validation |

---

## Key Design Decisions

1. **Shared MAC via registry** — `_GLOBAL_MACS` keyed by `(channel_id, seed, mac_config_json)`. All edge managers in `edge_list` get same `SbSpsMac` instance → same PRR, same bursts, consistent delivery. Each run is a subprocess so `_GLOBAL_MACS` starts empty, but `reset_global_macs()` is called explicitly in `__init__` for robustness against future in-process test runs.

2. **Tick cache keyed by tick only** — `_last_tick` and `_last_result` dict. Subset queries return slices; new senders on cached tick raise `RuntimeError`. Prevents silent double-advance.

3. **LegacyMacAdapter NOT shared** — each manager wraps its own `latency_model.should_drop()`. This is correct because `should_drop()` is stateless (i.i.d.). The legacy path needs no coordination. **Invariant:** legacy adapter fairness depends on "one manager per run" — if co-running multiple managers in one process, legacy i.i.d. drops will differ between them. Document in code.

4. **p_keep = 0.4 default** — produces realistic burst persistence. RC in [5,15] at 20 Hz = 250-750ms retention, 40% keep → extended collision bursts.

5. **Seeded RNG** — `self._rng = random.Random(seed)` per MAC instance. Seed is mandatory (default 0), propagated from test_runner `--seed` into YAML `mac.seed`.

6. **Burst histogram** — `{length: count}` dict, capped at 200. Bounded storage. `finalize()` is idempotent.

7. **Split drop causes** — `collision_drops` vs `fading_drops`. Proves burst structure comes from MAC collisions.

8. **`M` definition for paper** — "effective number of orthogonal sidelink resources per 50ms update quantum available to BSM-like messages."

9. **AoI coupling** — MAC drops do NOT create raw AoI holdover because RSU detections flow independently via backhaul. Instead, MAC burst drops degrade **prediction quality**: missing beacons → no anchoring → ego self-ghosting; burst drops → tracks age out after `max_age` → predictions disappear. This is the more interesting coupling for the paper — MAC bursts affect track continuity and anchoring reliability.

---

## YAML Config Example

```yaml
edge_list:
  - manager_type: late_fusion
    latency: 0.2
    jitter_std: 0.05
    latency_distribution: normal
    mac:
      type: sbsps
      M: 20
      RC_min: 5
      RC_max: 15
      p_keep: 0.4
      p_loss_base: 0.02
      seed: 42
```

---

## Verification

### Unit Tests
1. `SbSpsMac(M=2).attempt_tick(0, [1,2,3])` → some dropped (pigeonhole)
2. **Idempotence:** call `attempt_tick(0, [1,2,3])` twice → same result, `metrics.attempted` only counted once
3. **Subset tolerance:** `attempt_tick(0, [1,2,3])` then `attempt_tick(0, [1])` → returns `{1: same_value}`
4. **New sender on cached tick:** `attempt_tick(0, [1,2])` then `attempt_tick(0, [1,2,99])` → raises `RuntimeError`
5. **No `mac:` in cfg** → `LegacyMacAdapter` → identical to old `should_drop()`
6. **Burst persistence:** N=4, M=10, 10000 ticks → `burst_histogram` has keys > 1
7. **Shared registry:** two `create_mac_model()` calls with same config → same object (`is`)
8. **Monotonicity:** PRR(N=2) > PRR(N=8) > PRR(N=16) for fixed M=20
9. **Collision dominance:** at N=16/M=20, `collision_drops >> fading_drops`

### Integration
10. ```bash
    python test_runner.py -t openscenario_3_edge_late_fusion \
        --manager-types late_fusion --anchoring on \
        --latencies 200 --jitter-std 50 \
        --mac-type sbsps --ego-counts 1 2 4 8 \
        --repetitions 1
    ```
11. Check `simulation_metrics.json` has `mac.prr`, `mac.burst_histogram`, `mac.collision_drops`
12. `eval_fig_mac_characterization.png`: PRR ↓ with N, burst CDF heavier than geometric

### NS-3 Validation (Separate)
13. Build ns-3 + C-V2X module, run PRR sweeps
14. `python scripts/validate_mac_vs_ns3.py` — confirm < 3% PRR error vs both ns-3 and analytical

### Sanity Checks (Must Pass Before Full Sweeps)
1. **Shared-channel consistency:** If edge_list has 2+ managers, all report identical `mac_prr` and `burst_histogram`
2. **Monotonicity:** PRR decreases with N for fixed M; burst p95 increases with N
3. **Idempotence:** calling `attempt_tick(tick, ids)` twice does not change `attempted` counters
4. **Collision dominance:** `collision_drops` dominate `fading_drops` at high N
