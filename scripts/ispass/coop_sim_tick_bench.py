#!/usr/bin/env python3
"""Wall-clock time per simulation tick of a scenario-runner based cooperative driving platform.

Wraps the platform's own entry script without editing its files: the scenario manager's
_tick_scenario is timed from one entry to the next (one full loop period: agent, sharing,
scenario tree and the CARLA tick), the run is stopped after --warmup + --ticks ticks, and a
JSON summary with the fields of opencda_scale_bench.py is written.

Run from the root of the platform's checkout, against a CARLA server already listening:
    python coop_sim_tick_bench.py --platform v2xverse --n 4 --out r.json -- \
        simulation/leaderboard/leaderboard/leaderboard_evaluator_parameter.py --ego-num=4 ...
    python coop_sim_tick_bench.py --platform autocast --n 32 --out r.json -- \
        scenario_runner.py --route ... --sharing --bgtraffic 32 ...
--n is the requested count (V2Xverse: communicating driving vehicles, each with one roadside
unit; AutoCastSim: background vehicles). The number of vehicles that actually share is logged
per tick (AutoCastSim attaches a collaborator only to vehicles within 50 m of the ego).
"""
import argparse
import importlib
import json
import os
import runpy
import subprocess
import sys
import time

import numpy as np
import psutil

MANAGERS = {"v2xverse": "leaderboard.scenarios.scenario_manager", "autocast": "srunner.scenariomanager.scenario_manager"}


def carla_rss_gb():
    tot = 0
    for p in psutil.process_iter(["name", "memory_info"]):
        try:
            if p.info["name"] and p.info["name"].startswith("CarlaUE4-Linux"):
                tot += p.info["memory_info"].rss
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return tot / 2 ** 30


def gpu_mem_mb():
    try:
        return int(subprocess.check_output(
            ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits"], text=True).split()[0])
    except Exception:
        return -1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--platform", required=True, choices=sorted(MANAGERS))
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--ticks", type=int, default=300)
    ap.add_argument("--warmup", type=int, default=40)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tag", default="")
    ap.add_argument("entry", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    entry = [x for x in a.entry if x != "--"]
    if not entry:
        sys.exit("no entry script")
    sys.path.insert(0, os.getcwd())
    for _n, _t in (("float", float), ("int", int), ("bool", bool), ("object", object)):   # NumPy >= 1.24 with older sources
        if _n not in np.__dict__:
            setattr(np, _n, _t)
    res = {"platform": a.platform, "n_requested": a.n, "tag": a.tag, "completed": False, "host": os.uname().nodename,
           "ticks_requested": a.ticks, "warmup": a.warmup, "entry": entry}
    st = {"last": None, "wall": [], "sharing": [], "count": 0, "rss": 0.0, "carla": 0.0, "gpu": -1, "t0": time.time(), "done": False}
    proc = psutil.Process()

    def write(final):
        w = st["wall"]
        if w:
            q = lambda p: round(float(np.percentile(w, p)), 1)
            res.update(ticks=len(w), tick_ms=dict(mean=round(float(np.mean(w)), 1), p50=q(50), p95=q(95), p99=q(99)),
                       sim_s_per_wall_s=round(50.0 / float(np.mean(w)), 3),
                       sharing_vehicles=dict(mean=round(float(np.mean(st["sharing"])), 2), max=int(max(st["sharing"]))),
                       peak_client_rss_gb=round(st["rss"], 2), peak_carla_rss_gb=round(st["carla"], 2), peak_gpu_mb=st["gpu"])
        res["completed"] = bool(final and len(w) >= a.ticks)
        res["wall_total_s"] = round(time.time() - st["t0"], 1)
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        json.dump(res, open(a.out, "w"), indent=1)

    # V2Xverse's agent builds its voxel generator with the spconv 1.x class (spconv.utils.VoxelGeneratorV2). With
    # spconv 2.x installed, the class is provided here over Point2VoxelCPU3d, the same substitution the platform's
    # own opencood preprocessor makes for spconv 2.x. The platform's files are not edited.
    try:
        import spconv.utils as _su
        if not hasattr(_su, "VoxelGeneratorV2") and hasattr(_su, "Point2VoxelCPU3d"):
            from cumm import tensorview as _tv

            class VoxelGeneratorV2:
                def __init__(self, voxel_size, point_cloud_range, max_num_points, max_voxels, **_):
                    self._g = _su.Point2VoxelCPU3d(vsize_xyz=voxel_size, coors_range_xyz=point_cloud_range,
                                                   max_num_points_per_voxel=max_num_points, num_point_features=4,
                                                   max_num_voxels=max_voxels)

                def generate(self, points):
                    v, c, n = self._g.point_to_voxel(_tv.from_numpy(np.ascontiguousarray(points)))
                    return {"voxels": v.numpy(), "coordinates": c.numpy(), "num_points_per_voxel": n.numpy()}

            _su.VoxelGeneratorV2 = VoxelGeneratorV2
            res["compat"] = "spconv 2.x: VoxelGeneratorV2 provided over Point2VoxelCPU3d"
    except ImportError:
        pass
    mod = importlib.import_module(MANAGERS[a.platform])
    cls = mod.ScenarioManager
    orig = cls._tick_scenario

    def sharing_count(mgr):
        if a.platform == "autocast":
            return len(getattr(getattr(mgr, "_agent", None), "_collaborator_dict", {}) or {})
        return int(getattr(mgr, "ego_vehicles_num", a.n))

    def timed(self, timestamp):
        now = time.perf_counter()
        if st["last"] is not None and getattr(self, "_running", True):
            st["count"] += 1
            if st["count"] > a.warmup and len(st["wall"]) < a.ticks:
                st["wall"].append((now - st["last"]) * 1e3)
                st["sharing"].append(sharing_count(self))
            if st["count"] % 20 == 0:
                tree = [proc] + proc.children(recursive=True)
                st["rss"] = max(st["rss"], sum(p.memory_info().rss for p in tree if p.is_running()) / 2 ** 30)
                st["carla"] = max(st["carla"], carla_rss_gb()); st["gpu"] = max(st["gpu"], gpu_mem_mb())
                print(f"bench tick {st['count']} period {(now - st['last']) * 1e3:.0f} ms sharing {sharing_count(self)}", flush=True)
            if len(st["wall"]) >= a.ticks and not st["done"]:
                st["done"] = True
                write(True)
                self._running = False          # the platform's own loop ends and cleans up
        st["last"] = now
        return orig(self, timestamp)

    cls._tick_scenario = timed
    sys.argv = entry
    try:
        runpy.run_path(entry[0], run_name="__main__")
    except SystemExit:
        pass
    except BaseException as e:
        res["error"] = f"{type(e).__name__}: {e}"[:500]
        import traceback; traceback.print_exc()
    finally:
        if not st["done"]:
            res["ended_early"] = "the scenario ended before the requested ticks"
            write(False)
        print(json.dumps(res))
        sys.stdout.flush(); sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
