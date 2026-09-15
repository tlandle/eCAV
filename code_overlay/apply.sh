#!/bin/bash
# Apply the code overlay (this directory) onto the frozen base, or restore it.
# The overlay files are byte-identical to what the campaign/probe scripts copy at
# runtime; this is the same cp-then-git-checkout step, packaged so reproduction is
# one command instead of copying files by hand.
#
#   code_overlay/apply.sh apply    [repo_root]   # overlay -> base paths
#   code_overlay/apply.sh restore  [repo_root]   # git checkout the base paths
#
# The frozen base is commit 71c9f37e (freeze-1h). Run `apply` on a checkout at
# that base, run the scenario, then `restore`. scenario_1_1l.py is NOT included
# yet (the maneuvering endogenous-onset geometry is being finalized); it is added
# to the MAP below when it lands.
set -u
OVL="$(cd "$(dirname "$0")" && pwd)"
ROOT="${2:-$(git -C "$OVL" rev-parse --show-toplevel)}"
BASE=71c9f37e   # frozen base (freeze-1h); restore checks out from here explicitly
MAP=(
  "edge_manager_merged_1l_epoch.py|ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py"
  "ns3_bridge_wired.py|ecav/core/networking/ns3_cosim/ns3_bridge.py"
  "factories_1l.py|ecav/core/application/edge/migration/factories.py"
  "mtr_edge_predictor_1m.py|ecav/core/prediction/mtr_edge_predictor.py"
  "behavior_agent_1m.py|ecav/core/plan/behavior_agent.py"
  "pluggable_base_wired.py|ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py"
  "runner_wired.py|ecav/scenario_testing/openscenario_1_flow_gt.py"
  "wrapper_wired.py|ecav/core/tracking/mamba3dmot/wrapper.py"
  "tracker_wired.py|ecav/core/tracking/mamba3dmot/tracker.py"
  "tracklet_1l.py|ecav/core/tracking/mamba3dmot/tracklet.py"
  "payload_wired.py|ecav/core/application/edge/migration/payload.py"
  "daemon_1l.py|ecav/core/application/edge/migration/daemon.py"
  "scenario_1_1l.py|ecav/scenario_testing/scenarios/scenario_1.py"
)
cd "$ROOT" || { echo "bad repo root: $ROOT"; exit 1; }
case "${1:-}" in
  apply)
    # Refuse to overlay on top of live edits: a later `restore` would discard
    # them. Check every mapped base path (working tree + index) up front.
    dirty=()
    for m in "${MAP[@]}"; do b="${m##*|}"
      [ -f "$b" ] || continue
      if ! git diff --quiet -- "$b" || ! git diff --cached --quiet -- "$b"; then dirty+=("$b"); fi
    done
    if [ "${#dirty[@]}" -gt 0 ]; then
      echo "REFUSING: uncommitted changes to mapped base path(s); commit/stash them first:"
      for d in "${dirty[@]}"; do echo "  $d"; done
      exit 1
    fi
    for m in "${MAP[@]}"; do o="${m%%|*}"; b="${m##*|}"
      [ -f "$OVL/$o" ] || { echo "MISSING overlay $o"; exit 1; }
      [ -f "$b" ] || { echo "MISSING base $b"; exit 1; }
      cp "$OVL/$o" "$b" && echo "applied $o -> $b"
    done ;;
  restore)
    # Restore the frozen base explicitly, not whatever the working tree sits on.
    for m in "${MAP[@]}"; do b="${m##*|}"
      git checkout -q "$BASE" -- "$b" 2>/dev/null && echo "restored $b -> $BASE" || echo "could not restore $b"
    done ;;
  *) echo "usage: $(basename "$0") apply|restore [repo_root]"; exit 1 ;;
esac
