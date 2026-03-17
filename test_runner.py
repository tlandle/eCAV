
#!/usr/bin/env python3
"""
test_runner.py
---------------
Sweeps edge-latency, ego-vehicle max-speed, packet loss, baselines, anchoring,
GPS noise, and GPS dropout.
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

    # GPS noise + dropout sweep:
    python test_runner.py -t openscenario_3_edge \
        --gps-noise 0 1e-6 3e-6 1e-5 \
        --gps-dropout 0 5 10 25 \
        --speeds 70 --repetitions 3
"""

import argparse, copy, itertools, json, shutil, subprocess, sys, time, yaml, socket
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
ap.add_argument("--gps-noise", nargs="*", type=float, default=[],
                help="GPS noise stddev values applied to both noise_lat_stddev and noise_lon_stddev "
                     "(e.g., 0 1e-6 3e-6 1e-5)")
ap.add_argument("--gps-dropout", nargs="*", type=float, default=[],
                help="GPS dropout rate percentages 0-100, i.e. probability of dropping a GNSS fix "
                     "(e.g., 0 5 10 25)")
ap.add_argument("--compute-budget", nargs="*", type=str, default=[],
                help="Edge compute budget values in ms (e.g., 50 100 200 None). "
                     "Use 'None' for unlimited budget.")
ap.add_argument("--jitter-std", nargs="*", type=float, default=[],
                help="jitter std values (ms) to sweep")
ap.add_argument("--latency-distribution", type=str, default=None,
                choices=["fixed", "normal", "lognormal", "hybrid"],
                help="latency distribution model")
ap.add_argument("--manager-types", nargs="*", type=str, default=[],
                help="Edge manager types to sweep (e.g., late_fusion oracle vips_temporal). "
                     "Overrides manager_type in YAML for each edge in edge_list.")
ap.add_argument("--nms-threshold", nargs="*", type=float, default=[],
                help="Cross-camera NMS distance threshold (m) to sweep (e.g., 2 3 5 7)")
ap.add_argument("--ego-counts", nargs="*", type=int, default=[],
                help="Ego counts to sweep (e.g., 1 2 4 8 16). "
                     "Selects matching multi-ego YAML: <scenario>_<N>ego.yaml")
ap.add_argument("--timestamp-noise", nargs="*", type=float, default=[],
                help="Timestamp noise std (ms) to sweep for clock skew modeling (e.g., 0 10 25 50)")
ap.add_argument("--self-id-radius", nargs="*", type=float, default=[],
                help="Self-ID radius (m) to sweep for spatial ego identification (e.g., 2 3 5 7 10)")
ap.add_argument("--see-v2x-trace", type=str, default=None,
                help="Path to SEE-V2X merged_latency.csv (overrides env/repo-relative)")
ap.add_argument("--see-v2x-filter", nargs="*", type=str, default=[],
                help="SEE-V2X trace filter to sweep (e.g., intersection=3 intersection=4 "
                     "regime=L regime=M regime=H). Each filter is a separate run.")
ap.add_argument("--backhaul-mu", type=float, default=None,
                help="Backhaul log-normal mu parameter (default: 2.9957)")
ap.add_argument("--backhaul-sigma", type=float, default=None,
                help="Backhaul log-normal sigma parameter (default: 0.6556)")
ap.add_argument("--disable-backhaul", action="store_true", default=False,
                help="Disable backhaul component in hybrid model (radio-only)")
ap.add_argument("--mac-type", type=str, default=None,
                choices=["null", "sbsps"],
                help="MAC model type (null=no-loss, sbsps=C-V2X PC5 Mode 4)")
ap.add_argument("--mac-M", nargs="*", type=int, default=[],
                help="MAC resource slot counts to sweep (e.g., 10 20 50)")
ap.add_argument("--mac-p-keep", nargs="*", type=float, default=[],
                help="MAC p_keep values to sweep (e.g., 0.0 0.4 0.8)")
ap.add_argument("--seed", type=int, default=None,
                help="Random seed for reproducibility")
ap.add_argument("--repetitions", type=int, default=1)
args = ap.parse_args()

CARLA_BIN = "CarlaUE4-Linux-Shipping"      # running binary name
LAT_LIST  = args.latencies   or [None]    # keep original if empty
SPD_LIST  = args.speeds      or [None]
LOSS_LIST = args.packet_loss or [None]
BASE_LIST = args.baselines   or [None]    # None = use scenario as-is
GPS_NOISE_LIST   = args.gps_noise   or [None]
GPS_DROPOUT_LIST = args.gps_dropout or [None]
JITTER_LIST = args.jitter_std or [None]   # keep original if empty
MGR_TYPE_LIST = args.manager_types or [None]  # None = use YAML as-is
LAT_DIST = args.latency_distribution      # single value, not a sweep
NMS_THRESH_LIST = args.nms_threshold or [None]
EGO_COUNT_LIST = args.ego_counts or [None]
TS_NOISE_LIST = args.timestamp_noise or [None]
SELF_ID_RADIUS_LIST = args.self_id_radius or [None]
SEE_V2X_TRACE = args.see_v2x_trace          # single value, not a sweep
SEE_V2X_FILTER_LIST = args.see_v2x_filter or [None]
BACKHAUL_MU = args.backhaul_mu               # single value
BACKHAUL_SIGMA = args.backhaul_sigma         # single value
DISABLE_BACKHAUL = args.disable_backhaul     # single value
MAC_TYPE = args.mac_type                      # single value, not sweep
MAC_M_LIST = args.mac_M or [None]
MAC_PK_LIST = args.mac_p_keep or [None]
SEED = args.seed                              # single value

# Parse compute-budget: accept floats or literal "None"
def _parse_budget(val):
    if val is None or val.lower() == "none":
        return None
    return float(val)

BUDGET_LIST = [_parse_budget(v) for v in args.compute_budget] if args.compute_budget else [None]

# Anchoring sweep list
if args.anchoring == "both":
    ANCHOR_LIST = [True, False]
elif args.anchoring == "on":
    ANCHOR_LIST = [True]
elif args.anchoring == "off":
    ANCHOR_LIST = [False]
else:
    ANCHOR_LIST = [None]  # don't touch anchoring config

TOTAL_RUNS = (len(BASE_LIST) * len(MGR_TYPE_LIST) * len(ANCHOR_LIST) * len(LAT_LIST)
              * len(SPD_LIST) * len(LOSS_LIST) * len(GPS_NOISE_LIST) * len(GPS_DROPOUT_LIST)
              * len(BUDGET_LIST) * len(JITTER_LIST) * len(NMS_THRESH_LIST)
              * len(EGO_COUNT_LIST) * len(TS_NOISE_LIST) * len(SELF_ID_RADIUS_LIST)
              * len(SEE_V2X_FILTER_LIST) * len(MAC_M_LIST) * len(MAC_PK_LIST)
              * args.repetitions)

# ───────────────────────── paths ──────────────────────────────────────────
ROOT        = Path(__file__).resolve().parent
CFG_DIR     = ROOT / "ecav/scenario_testing/config_yaml"

STAMP       = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT    = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE    = ROOT / "ecav/scenario_testing/evaluation_outputs"
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
    """Return Lincoln target speed in m/s (WaypointFollower uses m/s).

    The Lincoln spawns at X=-35 and drives west to X=-84.8 (ego lane
    center, 49.8 m).  With WaypointFollower accel ~5 m/s² the Lincoln
    needs ~18 m to reach 13.4 m/s (48 km/h).
    """
    return 13.4

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
def get_cfg_file(baseline, ego_count=None):
    """Get the YAML config file path for a given baseline and ego count."""
    if baseline is None:
        name = args.scenario
    else:
        name = f"{args.scenario}_{baseline}"
    if ego_count is not None and ego_count > 1:
        name = f"{name}_{ego_count}ego"
    return CFG_DIR / f"{name}.yaml"

def load_original_cfg(baseline):
    """Load and return the original YAML config for a baseline."""
    cfg_file = get_cfg_file(baseline)
    if not cfg_file.exists():
        raise FileNotFoundError(f"Config not found: {cfg_file}")
    with cfg_file.open() as f:
        return yaml.safe_load(f)

def _patch_vehicle_localization(vehicle, gps_noise, gps_dropout):
    """Apply GPS noise and dropout settings to a single vehicle dict."""
    if gps_noise is not None:
        loc = vehicle.setdefault("sensing", {}).setdefault("localization", {})
        gnss = loc.setdefault("gnss", {})
        print(f"    setting GPS noise_lat/lon_stddev to {gps_noise}")
        gnss["noise_lat_stddev"] = gps_noise
        gnss["noise_lon_stddev"] = gps_noise
    if gps_dropout is not None:
        loc = vehicle.setdefault("sensing", {}).setdefault("localization", {})
        gnss = loc.setdefault("gnss", {})
        print(f"    setting GPS dropout rate to {gps_dropout}%")
        gnss["gnss_dropout_pct"] = gps_dropout


_SENTINEL = object()  # default: "not sweeping compute budget"

def patch_yaml(d, latency_ms, speed_kmh, packet_loss, anchoring,
               gps_noise=None, gps_dropout=None, compute_budget=_SENTINEL,
               jitter_std_ms=None, latency_distribution=None,
               manager_type=None, nms_threshold=None, timestamp_noise_ms=None,
               self_id_radius=None,
               see_v2x_trace=None, see_v2x_filter=None,
               backhaul_mu=None, backhaul_sigma=None,
               disable_backhaul=False,
               mac_type=None, mac_M=None, mac_p_keep=None, seed=None):
    """Updates the YAML dictionary with the current sweep parameters.

    *compute_budget*: float (ms) or None (unlimited).  The default
    sentinel ``_SENTINEL`` means "not part of this sweep, leave config
    as-is".
    *jitter_std_ms*: float (ms) or None.  Sets edge jitter_std in seconds.
    *latency_distribution*: str or None.  Sets edge latency_distribution model.
    *manager_type*: str or None.  Overrides edge manager_type for every edge.
    *nms_threshold*: float (m) or None.  Sets cross_camera_nms_threshold.
    *timestamp_noise_ms*: float (ms) or None.  Sets timestamp_noise_ms.
    *self_id_radius*: float (m) or None.  Sets self_id_radius.
    *see_v2x_trace*: str or None.  Explicit path to SEE-V2X merged_latency.csv.
    *see_v2x_filter*: str or None.  Filter expression for SEE-V2X trace.
    *backhaul_mu*: float or None.  Backhaul log-normal mu parameter.
    *backhaul_sigma*: float or None.  Backhaul log-normal sigma parameter.
    *disable_backhaul*: bool.  If True, skip backhaul component.
    """
    edge_present = bool(d.get("scenario", {}).get("edge_list", []))
    if edge_present:
        for edge in d["scenario"]["edge_list"]:
            # Override edge manager type
            if manager_type is not None:
                print(f"    setting manager_type to {manager_type}")
                edge["manager_type"] = manager_type
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

            # Set compute budget (None = unlimited, float = ms cap)
            if compute_budget is not _SENTINEL:
                if compute_budget is not None:
                    print(f"    setting compute_budget_ms to {compute_budget}")
                    edge["compute_budget_ms"] = compute_budget
                else:
                    print("    setting compute_budget_ms to None (unlimited)")
                    edge.pop("compute_budget_ms", None)

            # Set jitter std
            if jitter_std_ms is not None:
                print(f"    setting jitter_std to {jitter_std_ms/1000.0} s")
                edge["jitter_std"] = jitter_std_ms / 1000.0

            # Set latency distribution model
            if latency_distribution is not None:
                print(f"    setting latency_distribution to {latency_distribution}")
                edge["latency_distribution"] = latency_distribution

            # Set timestamp noise (clock skew)
            if timestamp_noise_ms is not None:
                print(f"    setting timestamp_noise_ms to {timestamp_noise_ms}")
                edge["timestamp_noise_ms"] = timestamp_noise_ms

            # Set self-ID radius
            if self_id_radius is not None:
                print(f"    setting self_id_radius to {self_id_radius}")
                edge["self_id_radius"] = self_id_radius

            # SEE-V2X trace configuration
            if see_v2x_trace is not None:
                print(f"    setting see_v2x_trace to {see_v2x_trace}")
                edge["see_v2x_trace"] = see_v2x_trace
            if see_v2x_filter is not None:
                print(f"    setting see_v2x_filter to {see_v2x_filter}")
                edge["see_v2x_filter"] = see_v2x_filter
            if backhaul_mu is not None:
                print(f"    setting backhaul_mu to {backhaul_mu}")
                edge["backhaul_mu"] = backhaul_mu
            if backhaul_sigma is not None:
                print(f"    setting backhaul_sigma to {backhaul_sigma}")
                edge["backhaul_sigma"] = backhaul_sigma
            if disable_backhaul:
                print("    disabling backhaul")
                edge["disable_backhaul"] = True

            # MAC model configuration
            if mac_type is not None:
                mac_block = edge.setdefault("mac", {})
                mac_block["type"] = mac_type
                mac_block["seed"] = seed or 0
                print(f"    setting mac.type to {mac_type}, mac.seed to {seed or 0}")
            if mac_M is not None:
                mac_block = edge.setdefault("mac", {})
                mac_block["M"] = mac_M
                print(f"    setting mac.M to {mac_M}")
            if mac_p_keep is not None:
                mac_block = edge.setdefault("mac", {})
                mac_block["p_keep"] = mac_p_keep
                print(f"    setting mac.p_keep to {mac_p_keep}")

            # Set cross-camera NMS threshold for each vehicle's perception
            if nms_threshold is not None:
                print(f"    setting cross_camera_nms_threshold to {nms_threshold}")
                for veh in edge.get("vehicles", []):
                    perc = veh.setdefault("sensing", {}).setdefault("perception", {})
                    perc["cross_camera_nms_threshold"] = nms_threshold

            # Patch GPS noise / dropout for every vehicle in the edge
            for veh in edge.get("vehicles", []):
                _patch_vehicle_localization(veh, gps_noise, gps_dropout)

    # Scale ScenarioRunner watchdog timeout by vehicle count.
    # With many vehicles the CARLA tick time grows (each has 4 cameras to
    # render), and the ScenarioRunner subprocess may receive tick updates
    # with large gaps.  Use a generous timeout to avoid watchdog fires
    # during the spawning phase.  The main process has its own MAX_STEP
    # guard so this timeout is just a safety net.
    if edge_present:
        n_vehicles = sum(len(e.get("vehicles", [])) for e in d["scenario"]["edge_list"])
        if n_vehicles > 4:
            sr_timeout = max(300, n_vehicles * 20)
            d.setdefault("scenario_runner", {})["timeout"] = sr_timeout

    # Also handle single_cav_list (non-edge scenarios)
    if gps_noise is not None or gps_dropout is not None:
        for veh in d.get("scenario", {}).get("single_cav_list", []):
            _patch_vehicle_localization(veh, gps_noise, gps_dropout)

    if speed_kmh is not None:
        if "edge_list" in d.get("scenario", {}):
            # Patch only the focal ego (vehicles[0]) — other egos keep default
            # speed. oncoming_speed_for() computes Lincoln timing for the focal
            # ego's approach only.
            beh = (d["scenario"]["edge_list"][0].setdefault("vehicles", [{}])[0]
                   .setdefault("behavior", {}))
        else:
            beh = (d["scenario"]["single_cav_list"][0].setdefault("behavior", {}))
        beh["max_speed"] = int(speed_kmh)

        oc_speed = oncoming_speed_for(speed_kmh)
        d.setdefault("scenario_runner", {})["openscenarioparams"] = [
            f"ego_vehicle_max_speed={int(speed_kmh)}",
            f"oncoming_vehicle_speed={oc_speed}"
        ]

# ───────────────────── single run helper ─────────────────────────────────
def run_once(baseline, anchoring, lat_ms, spd_kmh, loss, gps_noise, gps_dropout,
             compute_budget, jitter_std, mgr_type, rep,
             nms_threshold=None, ego_count=None, timestamp_noise=None,
             self_id_radius=None, see_v2x_filter=None,
             mac_type=None, mac_M=None, mac_p_keep=None):
    base_str = f"baseline_{baseline}" if baseline is not None else "baseline_default"
    anchor_str = f"anchoring_{'on' if anchoring else 'off'}" if anchoring is not None else "anchoring_default"
    lat_str = f"lat_{int(lat_ms)}" if lat_ms is not None else "lat_orig"
    spd_str = f"spd_{spd_kmh}" if spd_kmh is not None else "spd_orig"
    loss_str = f"loss_{int(loss)}" if loss is not None else "loss_orig"
    gnoise_str = f"gnoise_{gps_noise}" if gps_noise is not None else "gnoise_orig"
    gdrop_str = f"gdrop_{int(gps_dropout)}" if gps_dropout is not None else "gdrop_orig"
    budget_str = f"budget_{int(compute_budget)}" if compute_budget is not None else "budget_unlim"
    jitter_str = f"jitter_{jitter_std}" if jitter_std is not None else "jitter_orig"
    mgr_str = f"mgr_{mgr_type}" if mgr_type is not None else "mgr_default"
    nms_str = f"nms_{nms_threshold}" if nms_threshold is not None else ""
    ego_str = f"ego_{ego_count}" if ego_count is not None else ""
    tsnoise_str = f"tsnoise_{timestamp_noise}" if timestamp_noise is not None else ""
    sidr_str = f"sidr_{self_id_radius}" if self_id_radius is not None else ""
    v2x_str = f"v2x_{see_v2x_filter.replace('=','')}" if see_v2x_filter is not None else ""
    mac_str = f"mac_{mac_type}" if mac_type is not None else ""
    macM_str = f"macM_{mac_M}" if mac_M is not None else ""
    macpk_str = f"macpk_{mac_p_keep}" if mac_p_keep is not None else ""

    # Build config component string (filter out empty)
    config_parts = [lat_str, loss_str, gnoise_str, gdrop_str, budget_str, jitter_str]
    for extra in [nms_str, ego_str, tsnoise_str, sidr_str, v2x_str,
                  mac_str, macM_str, macpk_str]:
        if extra:
            config_parts.append(extra)
    config_dir = "_".join(config_parts)

    run_dir = EXP_ROOT / f"speed_{spd_str}" / base_str / mgr_str / anchor_str / config_dir / f"run_{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)

    eval_dir = OUT_BASE / STAMP / f"{base_str}_{mgr_str}_{anchor_str}_{spd_str}_{config_dir}_rep_{rep}"
    eval_dir.mkdir(parents=True, exist_ok=True)

    # Use the baseline-specific scenario name, with ego count suffix
    scenario_name = f"{args.scenario}_{baseline}" if baseline is not None else args.scenario
    if ego_count is not None and ego_count > 1:
        scenario_name = f"{scenario_name}_{ego_count}ego"
    cmd = [sys.executable, "ecav.py", "-t", scenario_name, "--apply_ml", "--output_dir", str(eval_dir)]

    run_log = run_dir / "runner_stdout.txt"
    with run_log.open("w") as log_f:
        log_f.write(f"CMD: {' '.join(cmd)}\n\n")
        log_f.flush()
        # Stream stdout+stderr to both the per-run log and the shared live log
        with LIVE_LOG.open("a") as live_f:
            header = f"\n{'='*60}\n  {base_str} {mgr_str} {anchor_str} {lat_str} {loss_str} rep={rep}\n  CMD: {' '.join(cmd)}\n{'='*60}\n"
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
    if gps_noise is not None: metrics["config_gps_noise_stddev"] = gps_noise
    if gps_dropout is not None: metrics["config_gps_dropout_pct"] = gps_dropout
    metrics["config_compute_budget_ms"] = compute_budget  # None = unlimited
    if mgr_type is not None: metrics["config_manager_type"] = mgr_type
    if jitter_std is not None: metrics["config_jitter_std_ms"] = jitter_std
    if LAT_DIST is not None: metrics["config_latency_distribution"] = LAT_DIST
    if nms_threshold is not None: metrics["config_nms_threshold"] = nms_threshold
    if ego_count is not None: metrics["config_ego_count"] = ego_count
    if timestamp_noise is not None: metrics["config_timestamp_noise_ms"] = timestamp_noise
    if self_id_radius is not None: metrics["config_self_id_radius"] = self_id_radius
    if see_v2x_filter is not None: metrics["config_see_v2x_filter"] = see_v2x_filter
    if SEE_V2X_TRACE is not None: metrics["config_see_v2x_trace"] = SEE_V2X_TRACE
    if BACKHAUL_MU is not None: metrics["config_backhaul_mu"] = BACKHAUL_MU
    if BACKHAUL_SIGMA is not None: metrics["config_backhaul_sigma"] = BACKHAUL_SIGMA
    if DISABLE_BACKHAUL: metrics["config_disable_backhaul"] = True
    if SEED is not None: metrics["config_seed"] = SEED
    if mac_type is not None: metrics["config_mac_type"] = mac_type
    if mac_M is not None: metrics["config_mac_M"] = mac_M
    if mac_p_keep is not None: metrics["config_mac_p_keep"] = mac_p_keep
    (run_dir / "simulation_metrics.json").write_text(json.dumps(metrics, indent=2))

# ───────────────────── backup all relevant YAMLs ─────────────────────────
# Backup each baseline's YAML (and multi-ego variants) so we can patch and restore.
# Key: (baseline, ego_count) tuple → (cfg_file, bak_file, original_cfg)
yaml_backups = {}
for baseline in BASE_LIST:
    for ego_count in EGO_COUNT_LIST:
        cfg_file = get_cfg_file(baseline, ego_count)
        bak_file = cfg_file.with_suffix(".yaml.bak")
        if cfg_file.exists():
            shutil.copy(cfg_file, bak_file)
            with bak_file.open() as f:
                original = yaml.safe_load(f)
            yaml_backups[(baseline, ego_count)] = (cfg_file, bak_file, original)
        else:
            # Only warn if this combo will actually be used
            if ego_count is not None and ego_count > 1:
                print(f"WARNING: Multi-ego config not found: {cfg_file}", flush=True)
            elif ego_count is None or ego_count == 1:
                print(f"WARNING: Config file not found: {cfg_file}", flush=True)

# ───────────────────── sweep loop ────────────────────────────────────────
_SWEEP_DIMS = list(itertools.product(
    BASE_LIST, MGR_TYPE_LIST, ANCHOR_LIST, LAT_LIST, SPD_LIST,
    LOSS_LIST, GPS_NOISE_LIST, GPS_DROPOUT_LIST, BUDGET_LIST,
    JITTER_LIST, NMS_THRESH_LIST, EGO_COUNT_LIST, TS_NOISE_LIST,
    SELF_ID_RADIUS_LIST, SEE_V2X_FILTER_LIST,
    MAC_M_LIST, MAC_PK_LIST,
    range(1, args.repetitions + 1)))

# Set seed if specified
if SEED is not None:
    import random as _rnd
    _rnd.seed(SEED)
    print(f"  Random seed set to {SEED}", flush=True)

try:
    for run_idx, (baseline, mgr_type, anchor, lat, spd, loss, gnoise,
                  gdrop, cbudget, jitter, nms_thresh, ego_count,
                  ts_noise, sidr, v2x_filter,
                  mac_m, mac_pk, rep) in enumerate(_SWEEP_DIMS, 1):
        # Look up the correct YAML: ego_count>1 uses ego-specific YAML,
        # ego_count=1 or None uses base YAML.  get_cfg_file() handles this.
        backup_key = (baseline, ego_count)
        if backup_key not in yaml_backups:
            # Fall back: ego_count=1 might be stored as (baseline, None) or vice versa
            fallback_key = (baseline, None)
            if fallback_key not in yaml_backups:
                print(f"  Skipping baseline '{baseline}' ego={ego_count} (no config found)", flush=True)
                continue
            backup_key = fallback_key

        cfg_file, _, original_cfg = yaml_backups[backup_key]

        anchor_label = "default" if anchor is None else ("on" if anchor else "off")
        base_label = baseline if baseline else "default"
        mgr_label = mgr_type if mgr_type is not None else "default"
        v2x_label = v2x_filter if v2x_filter is not None else "none"
        mac_m_label = mac_m if mac_m is not None else "default"
        mac_pk_label = mac_pk if mac_pk is not None else "default"
        print(f"-> Run {run_idx}/{TOTAL_RUNS}: baseline={base_label} mgr={mgr_label} "
              f"anchoring={anchor_label} "
              f"lat={lat} spd={spd} loss={loss} gps_noise={gnoise} gps_dropout={gdrop} "
              f"budget={cbudget} jitter={jitter} nms={nms_thresh} ego={ego_count} "
              f"ts_noise={ts_noise} sidr={sidr} v2x={v2x_label} "
              f"mac_M={mac_m_label} mac_pk={mac_pk_label} rep={rep}", flush=True)

        if not carla_running():
            start_carla()

        patched = copy.deepcopy(original_cfg)
        patch_yaml(patched, lat, spd, loss, anchor,
                   gps_noise=gnoise, gps_dropout=gdrop,
                   compute_budget=cbudget,
                   jitter_std_ms=jitter,
                   latency_distribution=LAT_DIST,
                   manager_type=mgr_type,
                   nms_threshold=nms_thresh,
                   timestamp_noise_ms=ts_noise,
                   self_id_radius=sidr,
                   see_v2x_trace=SEE_V2X_TRACE,
                   see_v2x_filter=v2x_filter,
                   backhaul_mu=BACKHAUL_MU,
                   backhaul_sigma=BACKHAUL_SIGMA,
                   disable_backhaul=DISABLE_BACKHAUL,
                   mac_type=MAC_TYPE,
                   mac_M=mac_m,
                   mac_p_keep=mac_pk,
                   seed=SEED)
        with cfg_file.open("w") as f:
            yaml.safe_dump(patched, f, sort_keys=False)

        run_once(baseline, anchor, lat, spd, loss, gnoise, gdrop,
                 cbudget, jitter, mgr_type, rep,
                 nms_threshold=nms_thresh, ego_count=ego_count,
                 timestamp_noise=ts_noise, self_id_radius=sidr,
                 see_v2x_filter=v2x_filter,
                 mac_type=MAC_TYPE, mac_M=mac_m, mac_p_keep=mac_pk)

        if run_idx < TOTAL_RUNS and run_idx % 5 == 0:
            kill_carla()
            start_carla()
finally:
    # Restore all backed-up YAMLs (base and multi-ego variants)
    for (baseline, ego_count), (cfg_file, bak_file, _) in yaml_backups.items():
        if bak_file.exists():
            bak_file.replace(cfg_file)
    print("\nYAML(s) restored. Sweep completed.", flush=True)
