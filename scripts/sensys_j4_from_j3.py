#!/usr/bin/env python3
"""Emit the j4 campaign (planner age re-indexing + 2.5 m margin, develop b979d020) as test_runner
commands that replay every j3 cell with its recorded configuration.

    python scripts/sensys_j4_from_j3.py --host atlas  > tmp/j4_atlas_cmds.sh
    python scripts/sensys_j4_from_j3.py --host cetus  > tmp/j4_cetus_cmds.sh

Each j3 cell's run_1/simulation_metrics.json records config_* (manager, anchoring, latency, speed,
ego count, edge and behavior kv, scenario env, self-id radius). Cells that share everything but the
latency are merged into one command. Seeds: every N=1 cell gets 10 (the exact-state reference too,
which j3 ran at 3), N>1 cells keep their j3 count (5). The overtake stamps and the cetus mirror are
covered; the trace replay cells are emitted from the campaign script's definition instead.
"""
import argparse, collections, glob, json, os, re, sys

ATLAS = [("openscenario_3_edge_late_fusion", "experiment_results/openscenario_3_edge_late_fusion/sensys_nov27_j3"),
         ("scp_edge_late_fusion", "experiment_results/scp_edge_late_fusion/sensys_nov27_j3"),
         ("blind_overtake_edge_late_fusion", "experiment_results/blind_overtake_edge_late_fusion/sensys_nov27_ov4"),
         ("blind_overtake_edge_late_fusion", "experiment_results/blind_overtake_edge_late_fusion/sensys_nov27_ov4_onc20"),
         ("blind_overtake_edge_late_fusion", "experiment_results/blind_overtake_edge_late_fusion/sensys_nov27_ov4_onc28")]
CETUS = [("openscenario_3_edge_late_fusion", "experiment_results/_cetus_mirror/experiment_results/openscenario_3_edge_late_fusion/sensys_nov27_j3"),
         ("scp_edge_late_fusion", "experiment_results/_cetus_mirror/experiment_results/scp_edge_late_fusion/sensys_nov27_j3")]


def kv_str(d):
    out = []
    for k, v in sorted(d.items()):
        if isinstance(v, bool):
            v = "true" if v else "false"
        out.append(f"{k}={v}")
    return " ".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", choices=["atlas", "cetus"], required=True)
    ap.add_argument("--stamp", default="sensys_nov27_j4")
    ap.add_argument("--port", default="2000")
    ap.add_argument("--reps-n1", type=int, default=10)
    a = ap.parse_args()
    roots = ATLAS if a.host == "atlas" else CETUS
    cells = collections.OrderedDict()
    for geom, root in roots:
        for mf in sorted(glob.glob(root + "/speed_spd_*/baseline_default/mgr_*/anchoring_*/*/run_1/simulation_metrics.json")):
            cell_dir = os.path.dirname(os.path.dirname(mf))
            if "v2x_regime" in cell_dir:
                continue  # trace replay cells: emitted below from the stage definition
            n_runs = len(glob.glob(cell_dir + "/run_*"))
            d = json.load(open(mf))
            ego = int(d.get("config_ego_count") or 1)
            key = (geom, d["config_manager_type"], bool(d["config_anchoring"]), int(d["config_speed_kmh"]), ego,
                   kv_str(d.get("config_edge_kv") or {}), kv_str(d.get("config_behavior_kv") or {}),
                   kv_str(d.get("config_scenario_env") or {}), float(d.get("config_self_id_radius") or 1.0),
                   int(d.get("config_max_step") or 0))
            lat = int(round(float(d["config_latency_ms"])))
            reps = a.reps_n1 if ego == 1 else n_runs
            cells.setdefault(key, {"lats": set(), "reps": reps})
            cells[key]["lats"].add(lat)
            cells[key]["reps"] = max(cells[key]["reps"], reps if ego == 1 else n_runs)
    total = 0
    print("#!/usr/bin/env bash")
    print(f"# j4 campaign for {a.host}: {len(cells)} commands generated from the j3 cells by scripts/sensys_j4_from_j3.py")
    print('set -uo pipefail; cd "$(dirname "$0")/.."; PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"')
    print('export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"')
    print(f'STAMP="${{STAMP:-{a.stamp}}}"; PORT="${{CARLA_PORT:-{a.port}}}"')
    for key, v in cells.items():
        geom, mgr, anch, spd, ego, ekv, bkv, senv, sidr, max_step = key
        lats = " ".join(str(x) for x in sorted(v["lats"]))
        cmd = [f'"$PY" test_runner.py -t {geom} --manager-types {mgr} --anchoring {"on" if anch else "off"}',
               f"--latencies {lats} --speeds {spd} --ego-counts {ego} --repetitions {v['reps']}",
               "--latency-distribution hybrid --per-tick-age-log --self-id-radius %g" % sidr,
               '--stamp "$STAMP" --carla-port "$PORT"']
        # The scenario env is not part of the run directory, so cells that differ only in the
        # oncoming speed collide under one stamp and --stamp resume skips the later ones as done
        # (j5 blind overtake, 2026-10-28). The 20 and 28 m/s cells get their own stamp suffix, as
        # the j3 ov4_onc20 and ov4_onc28 roots had.
        m_onc = re.search(r"ONCOMING_SPEED=(\S+)", senv or "")
        if m_onc and m_onc.group(1) != "12":
            cmd[3] = '--stamp "${STAMP}_onc%s" --carla-port "$PORT"' % m_onc.group(1)
        if ekv:
            cmd.append(f"--edge-kv {ekv}")
        if bkv:
            cmd.append(f"--behavior-kv {bkv}")
        if senv:
            cmd.append(f"--scenario-env {senv}")
        print(" ".join(cmd))
        total += len(v["lats"]) * v["reps"]
    if a.host == "atlas":
        # trace replay stage (scripts/sensys_campaign.sh 2b): left turn 70 km/h, in-order SEE-V2X replay
        tta = "38:5.25,50:5.0,53:5.0,63:5.5,70:7.0,88:9.0"
        hide = None
        for key in cells:
            if key[0] == "openscenario_3_edge_late_fusion" and "HIDE_ENV_OBJECTS" in key[7]:
                hide = re.search(r"HIDE_ENV_OBJECTS=(\S+)", key[7]).group(1); break
        senv = f"CROSS_TRIGGER_TTA={tta}" + (f" HIDE_ENV_OBJECTS={hide}" if hide else "")
        for mgr, anch in (("late_fusion", "both"), ("oracle", "on")):
            print(f'"$PY" test_runner.py -t openscenario_3_edge_late_fusion --manager-types {mgr} --anchoring {anch} --latencies 0 --speeds 70 --ego-counts 1 '
                  f'--see-v2x-filter regime=L regime=M regime=H --see-v2x-replay sequential --repetitions 10 '
                  f'--latency-distribution hybrid --per-tick-age-log --self-id-radius 1.0 --stamp "$STAMP" --carla-port "$PORT" '
                  f'--edge-kv prediction_steps=60 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 --scenario-env {senv}')
            total += 30 * (2 if anch == "both" else 1)
    print(f'"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" > logs/campaign_check_${{STAMP}}_{a.host}.txt 2>&1 || true')
    print(f'echo "$(date +%T) j4 {a.host} done"')
    print(f"# total runs: {total}", file=sys.stderr)


if __name__ == "__main__":
    main()
