
#!/usr/bin/env python3
"""
test_runner.py
---------------
Sweeps edge-latency, ego-vehicle max-speed, packet loss, baselines, and anchoring.
Automatically restarts CARLA whenever it's not running and force-kills /
restarts the server after every 5 completed runs to avoid drift.

Example:
    python test_runner.py -t openscenario_3_edge --latencies 0 100 200 --speeds 70 --packet-loss 0 5 10 --repetitions 3

    # Baseline + anchoring sweep for MobiCom experiments:
    python test_runner.py -t openscenario_3_edge \
        --baselines late_fusion vips \
        --anchoring both \
        --latencies 0 100 200 300 \
        --speeds 70 --repetitions 3
"""

import argparse, copy, json, shutil, subprocess, sys, time, yaml, socket
from pathlib import Path
from datetime import datetime

# ───────────────────────── CLI ────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--scenario", required=True, help="YAML without .yaml (base scenario name)")
ap.add_argument("--latencies", nargs="*", type=float, default=[],
                help="edge latency values (ms)")
ap.add_argument("--speeds",    nargs="*", type=int,   default=[],
                help="ego max_speed values (km/h)")
ap.add_argument("--packet-loss", nargs="*", type=float, default=[],
                help="List of packet loss percentages to apply to both uplink and downlink (e.g., 0 5 10)")
ap.add_argument("--baselines", nargs="*", type=str, default=[],
                help="Edge manager baselines to sweep (e.g., late_fusion vips worldfusion). "
                     "Each must have a matching YAML: <scenario>_<baseline>.yaml")
ap.add_argument("--anchoring", type=str, default=None, choices=["on", "off", "both"],
                help="Anchoring protocol: on, off, or both (runs with and without)")
ap.add_argument("--repetitions", type=int, default=1)
args = ap.parse_args()

CARLA_BIN = "CarlaUE4-Linux-Shipping"      # running binary name
LAT_LIST  = args.latencies   or [None]    # keep original if empty
SPD_LIST  = args.speeds      or [None]
LOSS_LIST = args.packet_loss or [None]
BASE_LIST = args.baselines   or [None]    # None = use scenario as-is

# Anchoring sweep list
if args.anchoring == "both":
    ANCHOR_LIST = [True, False]
elif args.anchoring == "on":
    ANCHOR_LIST = [True]
elif args.anchoring == "off":
    ANCHOR_LIST = [False]
else:
    ANCHOR_LIST = [None]  # don't touch anchoring config

TOTAL_RUNS = len(BASE_LIST) * len(ANCHOR_LIST) * len(LAT_LIST) * len(SPD_LIST) * len(LOSS_LIST) * args.repetitions

# ───────────────────────── paths ──────────────────────────────────────────
ROOT        = Path(__file__).resolve().parent
CFG_DIR     = ROOT / "opencda/scenario_testing/config_yaml"

STAMP       = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT    = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE    = ROOT / "opencda/scenario_testing/evaluation_outputs"
LOG_DIR     = ROOT / "logs"
LOG_DIR.mkdir(exist_ok=True)
LIVE_LOG    = LOG_DIR / f"run_{STAMP}.log"
LATEST_LOG  = LOG_DIR / "latest.log"
# Symlink logs/latest.log → current run log for easy tailing
LATEST_LOG.unlink(missing_ok=True)
LATEST_LOG.symlink_to(LIVE_LOG.name)

print(f"\n  Results under: {EXP_ROOT}", flush=True)
print(f"  Live log:      {LIVE_LOG}", flush=True)
print(f"  Tail with:     tail -f {LATEST_LOG}", flush=True)
print(f"  Total runs: {TOTAL_RUNS}\n", flush=True)

# ───────────────── CARLA watchdog helpers ────────────────────────────────
CARLA_SH   = Path.home() / "carla-0.9.15/CarlaUE4.sh"
CARLA_PORT = 2000

def oncoming_speed_for(ego_kmh: int) -> float:
    """Compute Lincoln speed (km/h) so it arrives at the intersection
    simultaneously with the ego vehicle.

    Geometry (scenario_3):
        ego runway  = 48.0 m  (Y=80 → Y=128)
        Lincoln run = 43.0 m  (X=-35 → X=-78)
        ego accel   ~ 3 m/s²

    Derived by solving t_ego = sqrt(2*d_ego/a) when ego hasn't reached
    max_speed, or t_ego = v/a + (d - v²/2a)/v otherwise, then
    v_lincoln = 43 / t_ego.
    """
    import math
    a = 3.0        # m/s²  typical CARLA accel
    d_ego = 48.0   # m     ego runway
    d_lin = 43.0   # m     Lincoln distance to intersection

    v_max = ego_kmh / 3.6  # m/s
    d_to_max = 0.5 * v_max**2 / a

    if d_ego <= d_to_max:
        t_ego = math.sqrt(2 * d_ego / a)
    else:
        t_accel = v_max / a
        t_cruise = (d_ego - d_to_max) / v_max
        t_ego = t_accel + t_cruise

    return (d_lin / t_ego) * 3.6  # km/h

def carla_running() -> bool:
    """True if server proc exists AND port answers."""
    try:
        ok_proc = subprocess.run(["pgrep", "-f", CARLA_BIN], stdout=subprocess.DEVNULL).returncode == 0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            ok_port = s.connect_ex(("127.0.0.1", CARLA_PORT)) == 0
        return ok_proc and ok_port
    except Exception:
        return False

def start_carla():
    print("    (re)starting CARLA ...", flush=True)
    subprocess.Popen([str(CARLA_SH)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 30:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", CARLA_PORT)) == 0:
                print("      CARLA ready", flush=True)
                return
        time.sleep(1)
    raise RuntimeError("CARLA failed to open port 2000 within 30 s")

def kill_carla():
    print("    stopping CARLA ...", flush=True)
    subprocess.run(["pkill", "-f", CARLA_BIN], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 15:
        subprocess.run(["pkill", "-f", "CarlaUE4-Linux-Shipping"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", CARLA_PORT)) != 0:
                print("      CARLA stopped", flush=True)
                return
        time.sleep(1)
    print("      (timeout - continuing anyway)", flush=True)

# ───────────────────── YAML helpers ──────────────────────────────────────
def get_cfg_file(baseline):
    """Get the YAML config file path for a given baseline."""
    if baseline is None:
        return CFG_DIR / f"{args.scenario}.yaml"
    return CFG_DIR / f"{args.scenario}_{baseline}.yaml"

def load_original_cfg(baseline):
    """Load and return the original YAML config for a baseline."""
    cfg_file = get_cfg_file(baseline)
    if not cfg_file.exists():
        raise FileNotFoundError(f"Config not found: {cfg_file}")
    with cfg_file.open() as f:
        return yaml.safe_load(f)

def patch_yaml(d, latency_ms, speed_kmh, packet_loss, anchoring):
    """Updates the YAML dictionary with the current sweep parameters."""
    edge_present = bool(d.get("scenario", {}).get("edge_list", []))
    if edge_present:
        for edge in d["scenario"]["edge_list"]:
            if latency_ms is not None:
                print(f"    setting edge latency to {latency_ms/1000.0} s")
                edge["latency"] = latency_ms / 1000.0

            # Set both uplink and downlink loss to the same value
            if packet_loss is not None:
                print(f"    setting packet loss to {packet_loss}%")
                edge["uplink_packet_loss_pct"] = packet_loss
                edge["downlink_packet_loss_pct"] = packet_loss

            # Set anchoring flag
            if anchoring is not None:
                print(f"    setting anchoring to {anchoring}")
                edge["anchoring"] = anchoring

    if speed_kmh is not None:
        if "edge_list" in d.get("scenario", {}):
            beh = (d["scenario"]["edge_list"][0].setdefault("vehicles", [{}])[0].setdefault("behavior", {}))
        else:
            beh = (d["scenario"]["single_cav_list"][0].setdefault("behavior", {}))
        beh["max_speed"] = int(speed_kmh)

        oc_speed = oncoming_speed_for(speed_kmh)
        d.setdefault("scenario_runner", {})["openscenarioparams"] = [
            f"ego_vehicle_max_speed={int(speed_kmh)}",
            f"oncoming_vehicle_speed={int(oc_speed)}"
        ]

# ───────────────────── single run helper ─────────────────────────────────
def run_once(baseline, anchoring, lat_ms, spd_kmh, loss, rep):
    base_str = f"baseline_{baseline}" if baseline is not None else "baseline_default"
    anchor_str = f"anchoring_{'on' if anchoring else 'off'}" if anchoring is not None else "anchoring_default"
    lat_str = f"lat_{int(lat_ms)}" if lat_ms is not None else "lat_orig"
    spd_str = f"spd_{spd_kmh}" if spd_kmh is not None else "spd_orig"
    loss_str = f"loss_{int(loss)}" if loss is not None else "loss_orig"

    run_dir = EXP_ROOT / f"speed_{spd_str}" / base_str / anchor_str / f"{lat_str}_{loss_str}" / f"run_{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)

    eval_dir = OUT_BASE / STAMP / f"{base_str}_{anchor_str}_{spd_str}_{lat_str}_{loss_str}_rep_{rep}"
    eval_dir.mkdir(parents=True, exist_ok=True)

    # Use the baseline-specific scenario name
    scenario_name = f"{args.scenario}_{baseline}" if baseline is not None else args.scenario
    cmd = [sys.executable, "opencda.py", "-t", scenario_name, "--apply_ml", "--output_dir", str(eval_dir)]

    run_log = run_dir / "runner_stdout.txt"
    with run_log.open("w") as log_f:
        log_f.write(f"CMD: {' '.join(cmd)}\n\n")
        log_f.flush()
        # Stream stdout+stderr to both the per-run log and the shared live log
        with LIVE_LOG.open("a") as live_f:
            header = f"\n{'='*60}\n  {base_str} {anchor_str} {lat_str} {loss_str} rep={rep}\n  CMD: {' '.join(cmd)}\n{'='*60}\n"
            live_f.write(header)
            live_f.flush()
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding="utf-8", errors="replace")
            for line in proc.stdout:
                log_f.write(line)
                live_f.write(line)
                live_f.flush()
            proc.wait()
        log_f.write(f"\nRET: {proc.returncode}\n")

    try:
        shutil.copytree(eval_dir, run_dir / "evaluation_output", dirs_exist_ok=True)
        with (eval_dir / "simulation_metrics.json").open() as f:
            metrics = json.load(f)
    except Exception as e:
        metrics = {"success_rate": 0.0, "error": str(e)}

    # Add config parameters to metrics file for easier analysis
    if baseline is not None: metrics["config_baseline"] = baseline
    if anchoring is not None: metrics["config_anchoring"] = anchoring
    if lat_ms is not None: metrics["config_latency_ms"] = lat_ms
    if spd_kmh is not None: metrics["config_speed_kmh"] = spd_kmh
    if loss is not None: metrics["config_packet_loss_pct"] = loss
    (run_dir / "simulation_metrics.json").write_text(json.dumps(metrics, indent=2))

# ───────────────────── backup all relevant YAMLs ─────────────────────────
# Backup each baseline's YAML so we can patch and restore
yaml_backups = {}  # baseline -> (cfg_file, bak_file, original_cfg)
for baseline in BASE_LIST:
    cfg_file = get_cfg_file(baseline)
    bak_file = cfg_file.with_suffix(".yaml.bak")
    if cfg_file.exists():
        shutil.copy(cfg_file, bak_file)
        with bak_file.open() as f:
            original = yaml.safe_load(f)
        yaml_backups[baseline] = (cfg_file, bak_file, original)
    else:
        print(f"WARNING: Config file not found: {cfg_file}", flush=True)

# ───────────────────── sweep loop ────────────────────────────────────────
try:
    run_idx = 0
    for baseline in BASE_LIST:
        if baseline not in yaml_backups:
            print(f"  Skipping baseline '{baseline}' (no config found)", flush=True)
            continue

        cfg_file, _, original_cfg = yaml_backups[baseline]

        for anchor in ANCHOR_LIST:
            for lat in LAT_LIST:
                for spd in SPD_LIST:
                    for loss in LOSS_LIST:
                        for rep in range(1, args.repetitions + 1):
                            run_idx += 1
                            anchor_label = "default" if anchor is None else ("on" if anchor else "off")
                            base_label = baseline if baseline else "default"
                            print(f"-> Run {run_idx}/{TOTAL_RUNS}: baseline={base_label} anchoring={anchor_label} lat={lat} spd={spd} loss={loss} rep={rep}", flush=True)

                            if not carla_running():
                                start_carla()

                            patched = copy.deepcopy(original_cfg)
                            patch_yaml(patched, lat, spd, loss, anchor)
                            with cfg_file.open("w") as f:
                                yaml.safe_dump(patched, f, sort_keys=False)

                            run_once(baseline, anchor, lat, spd, loss, rep)

                            if run_idx < TOTAL_RUNS and run_idx % 5 == 0:
                                kill_carla()
                                start_carla()
finally:
    # Restore all backed-up YAMLs
    for baseline, (cfg_file, bak_file, _) in yaml_backups.items():
        if bak_file.exists():
            bak_file.replace(cfg_file)
    print("\nYAML(s) restored. Sweep completed.", flush=True)
