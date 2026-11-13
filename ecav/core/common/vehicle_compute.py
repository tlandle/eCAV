# -*- coding: utf-8 -*-
"""Measured-time execution of a vehicle's perceive, plan and control step.

One rule for every component, the rule the edge service follows in
ecav/core/application/edge/service_occupancy.py:

  admission   an invocation starts at a step boundary, and only when the
              component is free in simulated time.
  occupancy   the invocation occupies the component for its duration d,
              t_free = t_start + d.
  latest      inputs that arrive while the component is busy are superseded.
              The next invocation reads the newest input.
  visibility  the output is visible to its consumer at the first step boundary
              at or after t_start + d. A duration of zero is the same step.
  elapsed     the component is told how many steps passed since its previous
              invocation, so its internal dynamics run on simulated time.

The vehicle pipeline (perceive, plan, control) is one component: the three
stages run back to back in one invocation, as the vehicle manager executes
them, and the control command is the output. While the pipeline is busy the
vehicle holds the command it applied last.

Behavior options (test_runner --behavior-kv, so they are in the run directory
name), all optional. Without vehicle_compute the vehicle step runs as before,
at zero cost inside the tick, and nothing in this module is on the path.

  vehicle_compute=zero|measured|trace|<ms>
      zero      today's behaviour (default).
      measured  wall time of each stage, measured in the run.
      trace     durations replayed from a recorded trace.
      <ms>      a constant duration of the whole pipeline, in milliseconds.
  vehicle_compute_scale=<x>
      factor on every duration (a vehicle computer slower or faster than the
      simulation host). vehicle_perceive_scale, vehicle_plan_scale and
      vehicle_control_scale set a stage's own factor.
  vehicle_compute_trace=<path>
      trace file. Default: the environment variable ECAV_TIMING_TRACE with
      ".veh<index>" appended. With vehicle_compute=measured the run records the
      trace, with vehicle_compute=trace the run replays it.
"""
import json
import logging
import math
import os
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

_EPS = 1e-9
STAGES = ("perceive", "plan", "control")


def steps_for(duration_ms: float, dt_ms: float) -> int:
    """Whole steps from an invocation's start to the first step boundary at or
    after its completion. Zero duration is zero steps."""
    if duration_ms <= _EPS:
        return 0
    return int(math.ceil(duration_ms / dt_ms - _EPS))


class StepOccupancy:
    """Occupancy and output visibility of one component at step granularity."""

    def __init__(self, dt_ms: float, name: str = ""):
        self.dt_ms = float(dt_ms)
        self.name = name
        self.t_free_ms = 0.0
        self.last_start: Optional[int] = None
        self._queue: List[Tuple[int, int, Any]] = []   # (apply_tick, start_tick, output)
        self.invoked = 0
        self.superseded = 0
        self.superseded_since_start = 0

    def free(self, tick: int) -> bool:
        return self.t_free_ms <= tick * self.dt_ms + _EPS

    def elapsed_steps(self, tick: int) -> int:
        """Steps since the previous invocation started (1 on the first one)."""
        return 1 if self.last_start is None else max(1, int(tick) - self.last_start)

    def start(self, tick: int, duration_ms: float, output: Any) -> int:
        """Record an invocation that started at this step. Returns the step at
        which its output is visible."""
        d = max(0.0, float(duration_ms))
        self.t_free_ms = tick * self.dt_ms + d
        self.last_start = int(tick)
        self.invoked += 1
        self.superseded_since_start = 0
        apply_tick = int(tick) + steps_for(d, self.dt_ms)
        self._queue.append((apply_tick, int(tick), output))
        return apply_tick

    def skip(self, tick: int) -> None:
        """The component is busy at this step: the input of this step is superseded."""
        self.superseded += 1
        self.superseded_since_start += 1

    def visible(self, tick: int) -> Optional[Tuple[int, int, Any]]:
        """The newest output whose visibility step has come, removed from the
        queue with every older one. None when nothing new is visible."""
        due = [q for q in self._queue if q[0] <= tick]
        if not due:
            return None
        self._queue = [q for q in self._queue if q[0] > tick]
        return due[-1]

    def metrics(self) -> Dict[str, Any]:
        n = self.invoked + self.superseded
        return {"invoked": int(self.invoked), "superseded": int(self.superseded),
                "superseded_fraction": (float(self.superseded) / n) if n else 0.0}


class DurationSource:
    """Durations of the pipeline's stages: zero, measured, replayed or constant, scaled."""

    def __init__(self, cfg, vehicle_index=None):
        get = cfg.get if hasattr(cfg, "get") else (lambda k, d=None: d)
        raw = str(get("vehicle_compute", "zero")).strip().lower()
        self.constant_ms: Optional[float] = None
        if raw in ("zero", "off", "false", "none", "0", ""):
            self.mode = "zero"
        elif raw in ("measured", "live", "on", "true"):
            self.mode = "measured"
        elif raw in ("trace", "replay"):
            self.mode = "trace"
        else:
            try:
                self.constant_ms = float(raw)
            except ValueError:
                raise ValueError("vehicle_compute=%r: expected zero, measured, trace or a duration in ms" % raw)
            self.mode = "constant" if self.constant_ms > 0.0 else "zero"
        base = float(get("vehicle_compute_scale", 1.0))
        self.scale = {s: float(get("vehicle_%s_scale" % s, base)) for s in STAGES}
        path = get("vehicle_compute_trace", None)
        if not path and os.environ.get("ECAV_TIMING_TRACE"):
            path = "%s.veh%s" % (os.environ["ECAV_TIMING_TRACE"],
                                 "" if vehicle_index is None else vehicle_index)
        self.trace_path = str(path) if path else None
        self.records: Dict[int, Dict[str, float]] = {}
        self._warned = False
        if self.mode == "trace":
            if self.trace_path and os.path.isfile(self.trace_path):
                with open(self.trace_path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line:
                            r = json.loads(line)
                            self.records[int(r["tick"])] = r
                logger.info("vehicle compute: %d invocations replayed from %s",
                            len(self.records), self.trace_path)
            else:
                logger.warning("vehicle compute trace %s not found; durations fall back to measured",
                               self.trace_path)
        elif self.mode == "measured" and self.trace_path:
            os.makedirs(os.path.dirname(os.path.abspath(self.trace_path)), exist_ok=True)
            open(self.trace_path, "w", encoding="utf-8").close()

    @property
    def enabled(self) -> bool:
        return self.mode != "zero"

    def resolve(self, tick: int, measured: Dict[str, float]) -> Dict[str, float]:
        """Charged duration per stage (ms) of the invocation that started at
        this step. measured holds the wall time of each stage in this run."""
        if self.mode == "constant":
            # the constant is the duration of the whole pipeline, charged to the planning stage
            return {"perceive": 0.0, "plan": float(self.constant_ms) * self.scale["plan"], "control": 0.0}
        raw = measured
        if self.mode == "trace":
            r = self.records.get(int(tick))
            if r is not None:
                raw = {s: float(r.get("%s_ms" % s, 0.0)) for s in STAGES}
            elif not self._warned:
                self._warned = True
                logger.warning("vehicle compute trace has no invocation at tick %d; measured durations "
                               "are used wherever the trace has none", tick)
        elif self.mode == "measured" and self.trace_path:
            with open(self.trace_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({"tick": int(tick),
                                    **{"%s_ms" % s: float(measured.get(s, 0.0)) for s in STAGES}}) + "\n")
        return {s: float(raw.get(s, 0.0)) * self.scale[s] for s in STAGES}


class VehicleCompute:
    """Per-vehicle pipeline occupancy. The vehicle manager calls note_perceive()
    from update_info() and step() from run_step()."""

    def __init__(self, cfg, dt_s: float, vehicle_index=None):
        self.source = DurationSource(cfg, vehicle_index)
        self.enabled = self.source.enabled
        self.dt_s = float(dt_s)
        self.dt_ms = self.dt_s * 1000.0
        self.vehicle_index = vehicle_index
        self.pipeline = StepOccupancy(self.dt_ms, "vehicle")
        # cooperative planning: the planner runs on the edge and the controller on the vehicle,
        # two components with their own occupancy
        self.edge_planner = StepOccupancy(self.dt_ms, "edge_planner")
        self.controller = StepOccupancy(self.dt_ms, "controller")
        self._perceive_ms = 0.0
        self._tick = -1
        self._held: Any = None
        self._held_start: Optional[int] = None
        self.rows: List[Dict[str, Any]] = []
        self.log_path: Optional[str] = None
        prefix = os.environ.get("ECAV_VEHICLE_COMPUTE_LOG")
        if self.enabled and prefix:
            self.set_log_path("%s.veh%s.jsonl" % (prefix, "" if vehicle_index is None else vehicle_index))
        if self.enabled:
            logger.info("vehicle compute: mode=%s scale=%s dt=%.0f ms (vehicle %s)",
                        self.source.mode if self.source.constant_ms is None
                        else "%g ms" % self.source.constant_ms,
                        self.source.scale, self.dt_ms, vehicle_index)

    # ------------------------------------------------------------------
    def set_log_path(self, path: str) -> None:
        self.log_path = path
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            for row in self.rows:
                f.write(json.dumps(row) + "\n")

    def _log(self, row: Dict[str, Any]) -> None:
        self.rows.append(row)
        if len(self.rows) > 20000:
            del self.rows[0]
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(row) + "\n")

    def tick_of(self, step_id) -> int:
        """The step index: the caller's step id, else a private count of calls."""
        self._tick = int(step_id) if step_id is not None else self._tick + 1
        return self._tick

    def note_perceive(self, wall_ms: float) -> None:
        """Wall time of localization and perception at this step."""
        self._perceive_ms = float(wall_ms)

    def _applied(self, out, tick: int, row: Dict[str, Any], idle_command: Any) -> Any:
        if out is not None:
            _, self._held_start, self._held = out
        if self._held_start is not None:
            row.update(applied_start_tick=self._held_start,
                       command_age_ms=round((tick - self._held_start) * self.dt_ms, 3))
        self._log(row)
        return self._held if self._held_start is not None else idle_command

    # ------------------------------------------------------------------
    def step(self, step_id, plan_fn: Callable[[int], Any],
             control_fn: Callable[[Any, float], Any], idle_command: Any = None) -> Any:
        """One step of the vehicle pipeline. Returns the command to apply at this step.

        plan_fn(elapsed_steps) runs the planner and returns its target.
        control_fn(target, elapsed_s) runs the controller and returns a command.
        Both are called only at a step where the pipeline is free. Until the
        first command is visible the vehicle applies idle_command."""
        tick = self.tick_of(step_id)
        row: Dict[str, Any] = {"tick": tick, "invoked": 0}
        if self.pipeline.free(tick):
            elapsed = self.pipeline.elapsed_steps(tick)
            superseded = self.pipeline.superseded_since_start
            t0 = time.perf_counter()
            target = plan_fn(elapsed)
            t1 = time.perf_counter()
            command = control_fn(target, elapsed * self.dt_s)
            t2 = time.perf_counter()
            d = self.source.resolve(tick, {"perceive": self._perceive_ms,
                                           "plan": (t1 - t0) * 1000.0,
                                           "control": (t2 - t1) * 1000.0})
            total = d["perceive"] + d["plan"] + d["control"]
            apply_tick = self.pipeline.start(tick, total, command)
            row.update(invoked=1, start_tick=tick, perceive_ms=round(d["perceive"], 3),
                       plan_ms=round(d["plan"], 3), control_ms=round(d["control"], 3),
                       duration_ms=round(total, 3), apply_tick=apply_tick, superseded=superseded)
        else:
            self.pipeline.skip(tick)
        return self._applied(self.pipeline.visible(tick), tick, row, idle_command)

    # ------------------------------------------------------------------
    #  Cooperative planning: planner on the edge, controller on the vehicle
    # ------------------------------------------------------------------
    def edge_plan(self, tick: int, plan_fn: Callable[[int], Any]) -> Optional[Tuple[Any, int]]:
        """Run the edge planner at this step if it is free. Returns (plan,
        steps until the plan leaves the edge), or None when the planner is busy."""
        if not self.edge_planner.free(tick):
            self.edge_planner.skip(tick)
            return None
        elapsed = self.edge_planner.elapsed_steps(tick)
        superseded = self.edge_planner.superseded_since_start
        t0 = time.perf_counter()
        plan = plan_fn(elapsed)
        d = self.source.resolve(tick, {"perceive": 0.0, "plan": (time.perf_counter() - t0) * 1000.0,
                                       "control": 0.0})["plan"]
        ready = self.edge_planner.start(tick, d, None)
        self.edge_planner.visible(ready)          # the caller queues the plan with its downlink
        self._log({"tick": int(tick), "invoked": 1, "component": "edge_planner", "start_tick": int(tick),
                   "plan_ms": round(d, 3), "duration_ms": round(d, 3), "apply_tick": ready,
                   "superseded": superseded})
        return plan, ready - int(tick)

    def control(self, tick: int, control_fn: Callable[[float], Any], idle_command: Any = None) -> Any:
        """Run the vehicle's controller at this step if it is free. Returns the
        command to apply at this step (the held one while a newer one is in progress).
        A constant vehicle_compute is charged to the planner, so the controller costs nothing there."""
        row: Dict[str, Any] = {"tick": int(tick), "invoked": 0, "component": "controller"}
        if self.controller.free(tick):
            elapsed = self.controller.elapsed_steps(tick)
            superseded = self.controller.superseded_since_start
            t0 = time.perf_counter()
            command = control_fn(elapsed * self.dt_s)
            d = self.source.resolve(tick, {"perceive": 0.0, "plan": 0.0,
                                           "control": (time.perf_counter() - t0) * 1000.0})["control"]
            apply_tick = self.controller.start(tick, d, command)
            row.update(invoked=1, start_tick=int(tick), control_ms=round(d, 3),
                       duration_ms=round(d, 3), apply_tick=apply_tick, superseded=superseded)
        else:
            self.controller.skip(tick)
        return self._applied(self.controller.visible(tick), int(tick), row, idle_command)

    def metrics(self) -> Dict[str, Any]:
        inv = [r for r in self.rows if r.get("invoked") and "component" not in r]
        ages = sorted(r["command_age_ms"] for r in self.rows
                      if "command_age_ms" in r and r.get("component") != "edge_planner")
        m: Dict[str, Any] = {"mode": self.source.mode, "constant_ms": self.source.constant_ms,
                             "scale": dict(self.source.scale)}
        m.update(self.pipeline.metrics())
        if inv:
            for s in STAGES + ("duration",):
                v = sorted(r["%s_ms" % s] for r in inv)
                m["%s_ms_mean" % s] = sum(v) / len(v)
                m["%s_ms_p95" % s] = v[min(len(v) - 1, int(0.95 * len(v)))]
        if ages:
            m["command_age_ms_mean"] = sum(ages) / len(ages)
            m["command_age_ms_p95"] = ages[min(len(ages) - 1, int(0.95 * len(ages)))]
        return m
