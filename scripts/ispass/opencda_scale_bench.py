#!/usr/bin/env python3
"""Scale benchmark of upstream OpenCDA (ucla-mobility/OpenCDA) on the eCAV scale workload.

Runs N OpenCDA vehicle managers ('single' application: localization, planner, controller,
optionally camera and LiDAR perception with OpenCDA's own detector) in OpenCDA's own
single-process loop on Town06, at the spawn slots and destinations of
scripts/ispass/gen_scale_town06.py, and records the wall-clock time of every simulation tick.

Run from the root of an OpenCDA checkout, against a CARLA server already listening:
    python opencda_scale_bench.py --gen ~/scale_wt/scripts/ispass/gen_scale_town06.py \
        -n 32 --layout dense --ticks 300 --out results/opencda_dense_n32.json
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys
import time

import numpy as np
import psutil
from omegaconf import OmegaConf

sys.path.insert(0, os.getcwd())
# Compatibility with NumPy >= 1.24 (OpenCDA pins 1.20): restore the removed scalar aliases
# that OpenCDA's source uses. OpenCDA's files are not modified.
for _n, _t in (("float", float), ("int", int), ("bool", bool), ("object", object)):
    if _n not in np.__dict__:
        setattr(np, _n, _t)


def load_slots(gen_path, n, layout):
    spec = importlib.util.spec_from_file_location("gen_scale_town06", gen_path)
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    xodr = gen.find_xodr(None)
    for spacing, min_route in ((12.0, 150.0), (10.0, 120.0), (9.0, 100.0)):   # the generator's own fallbacks
        sl = gen.slots(xodr, spacing, 250.0, 80.0, [350.0, 145.0], min_route)
        if len(sl) >= n:
            break
    else:
        sys.exit(f"only {len(sl)} slots")
    if layout == "spread":
        order = sorted(sl, key=lambda p: (p["road"], p["s"], p["lane"]))
        sl = [order[int(i * len(order) / n)] for i in range(n)]
    else:
        sl = sl[:n]
    return sl, spacing


def carla_rss_gb():
    tot = 0
    for p in psutil.process_iter(["exe", "memory_info"]):
        try:
            if p.info["exe"] and p.info["exe"].endswith("CarlaUE4-Linux-Shipping"):
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
    ap.add_argument("--gen", required=True, help="path to eCAV's gen_scale_town06.py")
    ap.add_argument("-n", type=int, required=True)
    ap.add_argument("--layout", choices=["dense", "spread"], default="dense")
    ap.add_argument("--perception", choices=["on", "off"], default="off")
    ap.add_argument("--map-manager", choices=["on", "off"], default="on", help="OpenCDA default is on")
    ap.add_argument("--ticks", type=int, default=300)
    ap.add_argument("--skip", type=int, default=5)
    ap.add_argument("--speed-kmh", type=int, default=40)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--version", default="0.9.12", help="OpenCDA blueprint naming switch; 0.9.12 names hold on 0.9.15")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    percep = a.perception == "on"

    import carla
    import opencda.scenario_testing.utils.sim_api as sim_api
    from opencda.core.common.cav_world import CavWorld
    from opencda.scenario_testing.utils.yaml_utils import add_current_time
    from opencda.version import __version__ as opencda_version

    if percep:
        # Headless host: OpenCDA opens one Open3D window per vehicle whenever perception is active,
        # also with LiDAR visualization off. The window is not used then, so its creation is skipped.
        import opencda.core.sensing.perception.perception_manager as _pm
        _pm.o3d_visualizer_init = lambda actor_id: None

    sl, spacing = load_slots(a.gen, a.n, a.layout)
    params = OmegaConf.load("opencda/scenario_testing/config_yaml/default.yaml")
    over = {
        "world": {"client_port": a.port, "fixed_delta_seconds": 0.05},
        "vehicle_base": {
            "sensing": {"perception": {"activate": percep, "camera": {"visualize": 0}, "lidar": {"visualize": False}},
                        "localization": {"activate": True, "debug_helper": {"show_animation": False}}},
            "map_manager": {"visualize": False, "activate": a.map_manager == "on"},
            "safety_manager": {"print_message": False},
            "behavior": {"max_speed": a.speed_kmh, "overtake_allowed": False,
                         "local_planner": {"debug": False, "debug_trajectory": False}},
        },
        "scenario": {"single_cav_list": [
            {"name": f"cav{k + 1}", "spawn_position": [p["x"], p["y"], p["z"], 0, p["yaw"], 0], "destination": p["dest"]}
            for k, p in enumerate(sl)]},
    }
    if percep:
        # the sensor suite of eCAV's scale scenario (scale_town06_base.yaml): four cameras and one
        # 64-channel LiDAR per vehicle. OpenCDA does not set the camera resolution, so its cameras
        # render at CARLA's default 800x600 (eCAV: 640x480).
        over["vehicle_base"]["sensing"]["perception"]["camera"].update(
            num=4, positions=[[1.5, 0.0, 1.7, 0], [0.0, -1.5, 1.7, -90], [0.0, 1.5, 1.7, 90], [-1.5, 0.0, 1.7, 180]])
        over["vehicle_base"]["sensing"]["perception"]["lidar"].update(
            channels=64, range=50, points_per_second=1150000, rotation_frequency=20, upper_fov=10.0, lower_fov=-30.0)
    params = add_current_time(OmegaConf.merge(params, OmegaConf.create(over)))

    res = dict(system="OpenCDA", opencda_version=opencda_version, n=a.n, layout=a.layout, perception=a.perception,
               map_manager=a.map_manager, spacing=spacing, ticks_requested=a.ticks, skip=a.skip, completed=False)
    t_start = time.time()
    cavs, sm = [], None
    proc = psutil.Process()
    try:
        cav_world = CavWorld(percep)
        sm = sim_api.ScenarioManager(params, percep, a.version, town="Town06", cav_world=cav_world)
        sm.client.set_timeout(600.0)
        cavs = sm.create_vehicle_manager(application=["single"])
        res["spawned"] = len(cavs)
        res["setup_s"] = round(time.time() - t_start, 1)
        wall, world, veh = [], [], []
        planner_exc, first_tb = 0, ""
        peak_rss = peak_carla = peak_gpu = 0
        for t in range(a.ticks + a.skip):
            t0 = time.perf_counter()
            sm.tick()
            t1 = time.perf_counter()
            for c in cavs:
                c.update_info()
                try:
                    control = c.run_step()
                except IndexError:
                    # OpenCDA's local planner indexes an empty waypoint or history buffer for a
                    # vehicle held in a queue. The vehicle brakes for this tick and stays in the
                    # run, so the load is kept; the count and the first traceback are reported.
                    if not planner_exc:
                        import traceback; first_tb = traceback.format_exc()
                    planner_exc += 1
                    control = carla.VehicleControl(brake=1.0)
                c.vehicle.apply_control(control)
            t2 = time.perf_counter()
            if t >= a.skip:
                wall.append((t2 - t0) * 1e3); world.append((t1 - t0) * 1e3); veh.append((t2 - t1) * 1e3)
            if t % 20 == 0:
                peak_rss = max(peak_rss, proc.memory_info().rss / 2 ** 30)
                peak_carla = max(peak_carla, carla_rss_gb())
                peak_gpu = max(peak_gpu, gpu_mem_mb())
                print(f"tick {t} wall {(t2 - t0) * 1e3:.0f} ms", flush=True)
        q = lambda x, p: round(float(np.percentile(x, p)), 1)
        res.update(completed=True, ticks=len(wall),
                   tick_ms=dict(mean=round(float(np.mean(wall)), 1), p50=q(wall, 50), p95=q(wall, 95), p99=q(wall, 99)),
                   world_ms=dict(mean=round(float(np.mean(world)), 1), p50=q(world, 50)),
                   vehicles_ms=dict(mean=round(float(np.mean(veh)), 1), p50=q(veh, 50)),
                   planner_exceptions=planner_exc, planner_exception_first=first_tb[-600:],
                   sim_s_per_wall_s=round(50.0 / float(np.mean(wall)), 3),
                   peak_client_rss_gb=round(peak_rss, 2), peak_carla_rss_gb=round(peak_carla, 2), peak_gpu_mb=peak_gpu,
                   speeds_mps_end=round(float(np.mean([np.hypot(c.vehicle.get_velocity().x, c.vehicle.get_velocity().y)
                                                        for c in cavs])), 2))
    except BaseException as e:   # record the failure and still write the file
        res["error"] = f"{type(e).__name__}: {e}"[:500]
        res["spawned"] = len(cavs)
        import traceback; traceback.print_exc()
    finally:
        res["wall_total_s"] = round(time.time() - t_start, 1)
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        json.dump(res, open(a.out, "w"), indent=1)
        print(json.dumps(res))
        try:
            for c in cavs:
                c.destroy()
            if sm is not None:
                sm.close()
        except Exception:
            pass
        sys.stdout.flush(); sys.stderr.flush()
        os._exit(0)   # OpenCDA's sensor callbacks outlive their actors at interpreter teardown


if __name__ == "__main__":
    main()
