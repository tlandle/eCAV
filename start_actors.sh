#!/bin/bash

# Usage: ./start_actors.sh [scenario_name] [--auto|-y]
#   scenario_name : if given, skips the scenario prompt. Counts (egos, rsus,
#                   edges, fusion type) are derived from config_yaml/<name>.yaml.
#   --auto / -y   : skip all runtime prompts and use fusion-type-aware defaults.
#                   Override via env vars: USE_ML, USE_LITSERVE, USE_YOLO_GRPC,
#                   USE_EDGE_ONLY, REBUILD, START_CARLA, HEADLESS, VERBOSE.
#
# Environment knobs (apply in both interactive and auto mode):
#   CONDA_ENV  : conda env for host-side processes (default: opencda)
#   CONDA_ROOT : conda install root              (default: $HOME/anaconda3)
#   CARLA_ROOT : CARLA install dir               (default: autodetect)

CONDA_ENV="${CONDA_ENV:-ecav310}"
CONDA_ROOT="${CONDA_ROOT:-$HOME/anaconda3}"

auto_mode=0
positional=()
for arg in "$@"; do
    case "$arg" in
        --auto|-y) auto_mode=1 ;;
        *)         positional+=("$arg") ;;
    esac
done

# Scenario name: CLI arg or prompt
if [[ -n "${positional[0]:-}" ]]; then
    scenario_name="${positional[0]}"
    echo "Scenario: $scenario_name (from CLI)"
else
    read -p "Enter scenario name (e.g., openscenario_3_edge): " scenario_name
fi

if [[ -z "$scenario_name" ]]; then
    echo "Error: Scenario name cannot be empty"
    exit 1
fi

# Derive ego/RSU/edge counts and fusion type from the scenario YAML.
SCENARIO_YAML="ecav/scenario_testing/config_yaml/${scenario_name}.yaml"
if [[ ! -f "$SCENARIO_YAML" ]]; then
    echo "Error: $SCENARIO_YAML not found"
    exit 1
fi

read num_ego num_rsu num_edges fusion_type < <(python3 - "$SCENARIO_YAML" <<'PY'
import sys, yaml
with open(sys.argv[1]) as f:
    d = yaml.safe_load(f) or {}
sc = d.get("scenario", {}) or {}
edge_list = sc.get("edge_list", []) or []
single_list = sc.get("single_cav_list", []) or []
num_edges = len(edge_list)
num_rsu = sum(len((e or {}).get("rsus", []) or []) for e in edge_list)
num_ego = sum(len((e or {}).get("vehicles", []) or []) for e in edge_list) + len(single_list)
mgr_types = {(e or {}).get("manager_type", "") for e in edge_list}
if "worldfusion" in mgr_types:
    fusion = "worldfusion"
elif "late_fusion" in mgr_types:
    fusion = "late_fusion"
else:
    fusion = "none"
print(num_ego, num_rsu, num_edges, fusion)
PY
)
echo "Derived from $SCENARIO_YAML: ego=$num_ego rsu=$num_rsu edges=$num_edges fusion=$fusion_type"

if ! [[ "$num_ego" =~ ^[0-9]+$ ]] || [[ "$num_ego" -lt 1 ]]; then
    echo "Error: derived num_ego=$num_ego invalid (need >=1). Check scenario YAML."
    exit 1
fi

# ML / gRPC defaults derived from fusion type:
#   worldfusion  -> ML=Y, WF-gRPC=Y, YOLO-gRPC=n
#   late_fusion  -> ML=Y, WF-gRPC=n, YOLO-gRPC=Y
#   none         -> ML=n
case "$fusion_type" in
    worldfusion) default_ml=Y; default_wf=Y; default_yolo=n ;;
    late_fusion) default_ml=Y; default_wf=n; default_yolo=Y ;;
    *)           default_ml=n; default_wf=n; default_yolo=n ;;
esac

if (( auto_mode )); then
    use_ml="${USE_ML:-$default_ml}"
    use_litserve="${USE_LITSERVE:-$default_wf}"
    use_yolo_grpc="${USE_YOLO_GRPC:-$default_yolo}"
    use_sequential="${USE_SEQUENTIAL:-n}"
    use_edge_only="${USE_EDGE_ONLY:-n}"
    rebuild="${REBUILD:-n}"
    use_verbose="${VERBOSE:-n}"
    echo "Auto mode: ML=$use_ml WF-gRPC=$use_litserve YOLO-gRPC=$use_yolo_grpc sequential=$use_sequential edge_only=$use_edge_only rebuild=$rebuild verbose=$use_verbose"
else
    use_ml="$default_ml"
    if [[ "$fusion_type" == "worldfusion" ]]; then
        echo "WorldFusion scenario — ML is required."
        read -p "Use WorldFusion gRPC server for distributed inference (Y/n)? " use_litserve
        use_litserve="${use_litserve:-$default_wf}"
        use_yolo_grpc="$default_yolo"
    elif [[ "$fusion_type" == "late_fusion" ]]; then
        echo "Late Fusion scenario — ML is required."
        read -p "Use YOLO gRPC server for distributed inference (Y/n)? " use_yolo_grpc
        use_yolo_grpc="${use_yolo_grpc:-$default_yolo}"
        use_litserve="$default_wf"
    else
        read -p "Use ML (Y/n)? " use_ml
        use_litserve="$default_wf"
        use_yolo_grpc="$default_yolo"
    fi
    read -p "Run mode — (s)equential, (e)dge-only, (d)istributed [d]: " _run_mode
    _run_mode="${_run_mode:-d}"
    case "${_run_mode,,}" in
        s|seq|sequential) use_sequential=Y; use_edge_only=n ;;
        e|eo|edge|edge-only) use_sequential=n; use_edge_only=Y ;;
        *) use_sequential=n; use_edge_only=n ;;
    esac
    rebuild="${REBUILD:-n}"
    if [[ "$use_sequential" != "Y" && "$use_sequential" != "y" ]]; then
        read -p "Rebuild containers (Y/n)? " rebuild
    fi
    read -p "Enable verbose/debug logging (y/N)? " use_verbose
fi

# Check and clean up any existing containers
running_containers=$(docker ps -a -q | wc -l)
if (( running_containers > 0 )); then
    echo "WARNING: Found existing containers. Cleaning up..."
    echo "Stopping and removing all old containers..."
    docker stop $(docker ps -a -q) 2>/dev/null
    docker rm $(docker ps -a -q) 2>/dev/null
fi

# Rebuild container if requested
if [[ "$rebuild" = "Y" || "$rebuild" = "y" ]]; then
    echo "Rebuilding container image..."
    docker build --network=host -f Dockerfile -t ecav-python310:latest .
fi

# Handle Carla - kill local instance if running, then optionally start fresh
echo ""
echo "Checking for local Carla process..."
if pgrep -f "CarlaUE4" > /dev/null; then
    echo "Found local Carla process running. Killing it for a fresh start..."
    pkill -9 -f "CarlaUE4" 2>/dev/null
    sleep 2
    echo "✓ Local Carla process killed"
fi

# Ask if user wants to start Carla locally (or use remote)
echo ""
if (( auto_mode )); then
    start_carla="${START_CARLA:-Y}"
    echo "Start Carla locally: $start_carla (auto)"
else
    read -p "Start Carla locally? (Y/n - select 'n' if using remote Carla): " start_carla
fi

# Resolve CARLA install root. Prefer $CARLA_ROOT env var, then known locations.
if [[ -z "$CARLA_ROOT" ]]; then
    for cand in "$HOME/carla-0.9.15" "/opt/carla-simulator" "$HOME/carla"; do
        if [[ -x "$cand/CarlaUE4.sh" ]]; then
            CARLA_ROOT="$cand"
            break
        fi
    done
fi
# Host path mounted into containers as /opt/carla-simulator/PythonAPI.
CARLA_PYTHONAPI="${CARLA_ROOT:-/opt/carla-simulator}/PythonAPI"

if [[ "$start_carla" = "Y" || "$start_carla" = "y" ]]; then
    if [[ -z "$CARLA_ROOT" ]]; then
        echo "ERROR: CARLA install not found. Set CARLA_ROOT or install at ~/carla-0.9.15."
        exit 1
    fi
    if (( auto_mode )); then
        headless="${HEADLESS:-Y}"
        echo "Headless: $headless (auto)"
    else
        read -p "Run Carla in headless mode (no display)? (Y/n): " headless
    fi

    echo ""
    echo "Starting Carla from $CARLA_ROOT..."
    if [[ "$headless" = "Y" || "$headless" = "y" ]]; then
        echo "  Mode: Headless (RenderOffScreen)"
        ( cd "$CARLA_ROOT" && ./CarlaUE4.sh -RenderOffScreen & )
    else
        echo "  Mode: With display"
        ( cd "$CARLA_ROOT" && ./CarlaUE4.sh & )
    fi

    echo ""
    echo "Waiting 10 seconds for Carla to initialize..."
    sleep 10

    # Verify Carla started successfully
    if ! pgrep -f "CarlaUE4" > /dev/null; then
        echo "ERROR: Failed to start Carla!"
        echo "Please check the Carla installation at $CARLA_ROOT/"
        exit 1
    fi
    CARLA_PID=$(pgrep -f "CarlaUE4" | head -1)
    echo "  Carla PID: $CARLA_PID"
    echo "✓ Carla started successfully"
else
    echo ""
    echo "Using remote Carla instance (not starting locally)"
    echo "Make sure your remote Carla server is running and accessible."
fi
echo ""

# Start WorldFusion gRPC server if requested
WF_GRPC_PID=""
if [[ "$use_litserve" = "Y" || "$use_litserve" = "y" ]]; then
    if [[ "$use_ml" != "Y" && "$use_ml" != "y" ]]; then
        echo "ERROR: WorldFusion gRPC server requires ML to be enabled. Please answer 'Y' to 'Use ML'."
        exit 1
    fi
    echo "Starting WorldFusion gRPC inference server (port 18002)..."
    WF_GRPC_LOG=$(mktemp /tmp/worldfusion_grpc.XXXXXX.log)
    bash -c "source $CONDA_ROOT/etc/profile.d/conda.sh && conda activate $CONDA_ENV && PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True python ecav/ml_manager/worldfusion_grpc_server.py > '$WF_GRPC_LOG' 2>&1" &
    WF_GRPC_PID=$!
    echo "  WorldFusion gRPC PID: $WF_GRPC_PID"
    echo "  Log file: $WF_GRPC_LOG"
    echo "  Waiting for WorldFusion gRPC server to be ready on port 18002..."

    timeout=90
    elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if conda run -n "$CONDA_ENV" python -c "import grpc; c=grpc.insecure_channel('localhost:18002'); grpc.channel_ready_future(c).result(timeout=1)" 2>/dev/null; then
            echo "  ✓ WorldFusion gRPC server is ready"
            break
        fi
        sleep 2
        ((elapsed+=2))
        echo -n "."
    done
    echo ""

    if [[ $elapsed -ge $timeout ]]; then
        echo "ERROR: WorldFusion gRPC server did not become ready within ${timeout}s."
        echo "Check logs: tail -f $WF_GRPC_LOG"
        exit 1
    fi
fi

# Start YOLO gRPC server if requested
YOLO_GRPC_PID=""
if [[ "$use_yolo_grpc" = "Y" || "$use_yolo_grpc" = "y" ]]; then
    if [[ "$use_ml" != "Y" && "$use_ml" != "y" ]]; then
        echo "ERROR: YOLO gRPC server requires ML to be enabled. Please answer 'Y' to 'Use ML'."
        exit 1
    fi
    echo "Starting YOLO gRPC inference server (port 18001)..."
    YOLO_GRPC_LOG=$(mktemp /tmp/yolo_grpc.XXXXXX.log)
    bash -c "source $CONDA_ROOT/etc/profile.d/conda.sh && conda activate $CONDA_ENV && PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True python ecav/ml_manager/yolo_grpc_server.py > '$YOLO_GRPC_LOG' 2>&1" &
    YOLO_GRPC_PID=$!
    echo "  YOLO gRPC PID: $YOLO_GRPC_PID"
    echo "  Log file: $YOLO_GRPC_LOG"
    echo "  Waiting for YOLO gRPC server to be ready on port 18001..."

    timeout=90
    elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if conda run -n "$CONDA_ENV" python -c "import grpc; c=grpc.insecure_channel('localhost:18001'); grpc.channel_ready_future(c).result(timeout=1)" 2>/dev/null; then
            echo "  ✓ YOLO gRPC server is ready"
            break
        fi
        sleep 2
        ((elapsed+=2))
        echo -n "."
    done
    echo ""

    if [[ $elapsed -ge $timeout ]]; then
        echo "ERROR: YOLO gRPC server did not become ready within ${timeout}s."
        echo "Check logs: tail -f $YOLO_GRPC_LOG"
        exit 1
    fi
fi

# Determine GPU settings
gpu_flag=""
if [[ "$use_ml" = "Y" || "$use_ml" = "y" ]]; then
    # Check if nvidia-smi is available
    if ! command -v nvidia-smi &> /dev/null; then
        echo "ERROR: ML requested but nvidia-smi not found."
        echo "GPU support is required for ML. Please install NVIDIA drivers."
        exit 1
    fi

    # Check if GPUs are detected
    num_gpus=$(nvidia-smi -L 2>/dev/null | wc -l)
    if [[ $num_gpus -eq 0 ]]; then
        echo "ERROR: ML requested but no GPUs detected."
        echo "GPU support is required for ML."
        exit 1
    fi

    # Check if nvidia runtime is available in Docker
    if ! docker info 2>/dev/null | grep -q "nvidia"; then
        echo "ERROR: ML requested but NVIDIA Docker runtime not configured."
        echo "GPU support is required for ML."
        echo ""
        echo "To enable GPU support, install nvidia-container-toolkit:"
        echo "  https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
        echo ""
        echo "Or run without ML by answering 'n' when prompted for ML usage."
        exit 1
    fi

    echo "This machine has $num_gpus GPU core(s)"
    echo "Using NVIDIA Docker runtime"
    gpu_flag="--runtime=nvidia --gpus all"
fi

# Build ML flag for ecav.py
ml_flag=""
if [[ "$use_ml" = "Y" || "$use_ml" = "y" ]]; then
    ml_flag="--apply_ml"
fi

litserve_flag=""
wf_grpc_env=""
if [[ "$use_litserve" = "Y" || "$use_litserve" = "y" ]]; then
    litserve_flag="-l"
    wf_grpc_env="-e WF_GRPC_ENDPOINT=localhost:18002"
fi

verbose_flag=""
if [[ "$use_verbose" = "Y" || "$use_verbose" = "y" ]]; then
    verbose_flag="--verbose"
fi

# Determine run mode flag for ecav.py
if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
    mode_flag=""           # sequential: ScenarioRunner subprocess, no Docker actors
elif [[ "$use_edge_only" = "Y" || "$use_edge_only" = "y" ]]; then
    mode_flag="-eo"
else
    mode_flag="-d"
fi

echo ""
echo "=========================================="
echo "Starting eCAV Distributed Scenario"
echo "=========================================="
echo "Scenario: $scenario_name"
echo "Mode: $(if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then echo "sequential (no -d)"; elif [[ "$use_edge_only" = "Y" || "$use_edge_only" = "y" ]]; then echo "edge-only (-eo)"; else echo "fully-distributed (-d)"; fi)"
echo "Ego vehicles: $num_ego"
echo "RSUs: $num_rsu"
echo "Edges: $num_edges"
echo "ML enabled: $use_ml"
echo "WorldFusion gRPC: $use_litserve"
echo "YOLO gRPC: $use_yolo_grpc"
echo "Verbose: $use_verbose"
echo "=========================================="
echo ""

# Poll docker logs for a pattern, with timeout and crash detection.
# Usage: wait_for_container_log <container> <pattern> [timeout_seconds]
wait_for_container_log() {
    local container=$1
    local pattern=$2
    local timeout=${3:-90}
    local elapsed=0
    echo -n "  Waiting for container $container to be ready..."
    while [[ $elapsed -lt $timeout ]]; do
        if docker logs "$container" 2>&1 | grep -q "$pattern"; then
            echo " ✓"
            return 0
        fi
        local state
        state=$(docker inspect -f '{{.State.Status}}' "$container" 2>/dev/null)
        if [[ "$state" == "exited" || "$state" == "dead" ]]; then
            echo ""
            echo "ERROR: Container $container stopped unexpectedly (status: $state)"
            docker logs --tail 20 "$container" 2>&1
            return 1
        fi
        sleep 1
        ((elapsed++))
        echo -n "."
    done
    echo ""
    echo "ERROR: Timeout (${timeout}s) waiting for '$pattern' in $container"
    docker logs --tail 20 "$container" 2>&1
    return 1
}

# Create temporary log file for ecav base process
ECAV_LOG=$(mktemp /tmp/ecav_base.XXXXXX.log)
echo "eCAV log file: $ECAV_LOG"
echo ""

# Start base eCAV process (on host, not in container)
echo "Starting base eCAV process on host..."
echo "  Log file: $ECAV_LOG"

# Start base process in background using conda environment
# Source conda.sh directly to enable conda commands
bash -c "source $CONDA_ROOT/etc/profile.d/conda.sh && conda activate $CONDA_ENV && python -u ecav.py -t '$scenario_name' -v 0.9.15 $mode_flag $ml_flag $litserve_flag $verbose_flag > '$ECAV_LOG' 2>&1" &
ECAV_PID=$!

echo "  ✓ Base process started (PID: $ECAV_PID)"
if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
    # Sequential mode: no gRPC handshake — process owns its own ScenarioRunner subprocess.
    # Give it a moment to start then proceed straight to monitoring.
    echo "  Sequential mode — no readiness gate; monitoring for completion."
    sleep 3
elif [[ "$use_edge_only" = "Y" || "$use_edge_only" = "y" ]]; then
    echo "  Monitoring log file for 'EdgeRegistrationServer' message..."
    ready_pattern="EdgeRegistrationServer"
    ready_msg="EdgeRegistrationServer ready"
else
    echo "  Monitoring log file for 'pushed scenario start' message..."
    ready_pattern="pushed scenario start"
    ready_msg="Scenario initialization complete"
fi

if [[ "$use_sequential" != "Y" && "$use_sequential" != "y" ]]; then
    timeout=60
    elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if grep -qi "$ready_pattern" "$ECAV_LOG" 2>/dev/null; then
            echo "  ✓ $ready_msg!"
            break
        fi

        sleep 2
        ((elapsed+=2))
        echo -n "."
    done

    echo ""

    if [[ $elapsed -ge $timeout ]]; then
        echo "ERROR: Timeout waiting for '$ready_pattern' message."
        echo "Check logs: tail -f $ECAV_LOG"
        echo ""

        if pgrep -f "CarlaUE4" > /dev/null; then
            echo "Stopping Carla processes..."
            pkill -f "CarlaUE4"
            sleep 2
            if pgrep -f "CarlaUE4" > /dev/null; then
                pkill -9 -f "CarlaUE4"
                sleep 1
            fi
            if ! pgrep -f "CarlaUE4" > /dev/null; then
                echo "✓ Carla stopped"
            else
                echo "⚠ Warning: Some Carla processes may still be running"
            fi
        else
            echo "Carla is not running."
        fi
        exit 1
    fi
fi

echo ""

# Start edge containers (if any, and not sequential — sequential uses ScenarioRunner subprocess)
if [[ $num_edges -gt 0 && "$use_sequential" != "Y" && "$use_sequential" != "y" ]]; then
    echo "Starting $num_edges edge container(s)..."

    if [[ "$use_edge_only" = "Y" || "$use_edge_only" = "y" ]]; then
        # Edge-only mode: edges register with ecav.py's EdgeRegistrationServer on port 50055.
        # Use port base 50060 to avoid collision with the registration server (50055) and
        # the C++ orchestrator port (50051).
        EDGE_BASE_PORT=50060
        for ((e=0; e<$num_edges; e++))
        do
            container_name="edge_$e"
            edge_port=$((EDGE_BASE_PORT + e))
            echo "  Starting $container_name (port: $edge_port, connects to registration server on 50055)..."

            docker run $gpu_flag -d \
                --network=host \
                --name="$container_name" \
                -e "HOSTNAME=$container_name" \
                -e IS_DOCKER=1 \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$CARLA_PYTHONAPI":/opt/carla-simulator/PythonAPI:ro \
                -e DISPLAY=$DISPLAY \
                -e TERM \
                ecav-python310:latest \
                python3.10 -u ecav/ecav2/edge_process.py \
                    --orchestrator_ip localhost \
                    --orchestrator_port 50055 \
                    -P $edge_port

            echo "  ✓ $container_name started"
            wait_for_container_log "$container_name" "edge-only ready" 120 || exit 1
        done

        echo ""
        echo "  Waiting for ecav.py to connect to all edge fusion servers..."
        timeout=60
        elapsed=0
        while [[ $elapsed -lt $timeout ]]; do
            if grep -q "\[EDGE-ONLY\]" "$ECAV_LOG" 2>/dev/null; then
                echo "  ✓ All edges connected — scenario starting"
                break
            fi
            sleep 2
            ((elapsed+=2))
            echo -n "."
        done
        echo ""
        if [[ $elapsed -ge $timeout ]]; then
            echo "ERROR: Timeout waiting for '[EDGE-ONLY]' in ecav.py log."
            echo "Check logs: tail -f $ECAV_LOG"
            exit 1
        fi
    else
        EDGE_BASE_PORT=50054
        for ((e=0; e<$num_edges; e++))
        do
            container_name="edge_$e"
            edge_port=$((EDGE_BASE_PORT + e))
            echo "  Starting $container_name (edge index: $e, port: $edge_port)..."

            docker run $gpu_flag -d \
                --network=host \
                --name="$container_name" \
                -e "HOSTNAME=$container_name" \
                -e IS_DOCKER=1 \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -v "$CARLA_PYTHONAPI":/opt/carla-simulator/PythonAPI:ro \
                -e DISPLAY=$DISPLAY \
                -e TERM \
                ecav-python310:latest \
                python3.10 -u ecav/ecav2/edge_process.py -e $e -P $edge_port

            echo "  ✓ $container_name started"
            wait_for_container_log "$container_name" "registered successfully" 90 || exit 1
        done
    fi
    echo ""
fi

if [[ "$use_edge_only" != "Y" && "$use_edge_only" != "y" && "$use_sequential" != "Y" && "$use_sequential" != "y" ]]; then
# Start ego vehicle containers
echo "Starting $num_ego ego vehicle container(s)..."
for ((i=0; i<$num_ego; i++))
do
    container_name="ego_vehicle_$i"
    echo "  Starting $container_name (actor index: $i)..."

    docker run $gpu_flag -d \
        --network=host \
        --name="$container_name" \
        -e "HOSTNAME=$container_name" \
        -e IS_DOCKER=1 \
        $wf_grpc_env \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$CARLA_PYTHONAPI":/opt/carla-simulator/PythonAPI:ro \
        -e DISPLAY=$DISPLAY \
        -e TERM \
        -e QT_X11_NO_MITSHM=1 \
        ecav-python310:latest \
        python3.10 -u ecav.py $ml_flag $litserve_flag $verbose_flag -v 0.9.15 -d -i $i -T $((8000 + i))

    echo "  ✓ $container_name started"
    wait_for_container_log "$container_name" "Registered with" 90 || exit 1
done

# Start RSU containers
if [[ $num_rsu -gt 0 ]]; then
    echo ""
    echo "Starting $num_rsu RSU container(s)..."
    for ((i=0; i<$num_rsu; i++))
    do
        container_name="rsu_$i"
        echo "  Starting $container_name (actor index: $i)..."

        docker run $gpu_flag -d \
            --network=host \
            --name="$container_name" \
            -e "HOSTNAME=$container_name" \
            -e IS_DOCKER=1 \
            $wf_grpc_env \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v "$CARLA_PYTHONAPI":/opt/carla-simulator/PythonAPI:ro \
            -e DISPLAY=$DISPLAY \
            ecav-python310:latest \
            python3.10 -u ecav/ecav2/ecloud_actor_client.py $ml_flag $litserve_flag $verbose_flag -v 0.9.15 -i $i

        echo "  ✓ $container_name started"
        wait_for_container_log "$container_name" "Registered with" 90 || exit 1
    done
fi

# Start non-ego vehicles container
# Non-ego vehicles always use index -1
echo ""
echo "Starting non-ego vehicles container..."
container_name="non_ego_vehicles"
echo "  Starting $container_name (actor index: -1)..."

docker run $gpu_flag -d \
    --network=host \
    --name="$container_name" \
    -e "HOSTNAME=$container_name" \
    -e IS_DOCKER=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$CARLA_PYTHONAPI":/opt/carla-simulator/PythonAPI:ro \
    -e DISPLAY=$DISPLAY \
    ecav-python310:latest \
    python3.10 -u ecav.py $ml_flag $litserve_flag $verbose_flag -v 0.9.15 -d -i -1 -T 8100

echo "  ✓ $container_name started"

fi  # end: not edge-only and not sequential

echo ""
echo "=========================================="
if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
    echo "Sequential scenario running (no Docker containers)"
else
    echo "All containers started successfully!"
fi
echo "=========================================="
echo ""
echo "Container summary:"
if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
    echo "  - (sequential mode: all actors managed by ScenarioRunner subprocess — no Docker containers)"
else
    if [[ $num_edges -gt 0 ]]; then
        if [[ "$use_edge_only" = "Y" || "$use_edge_only" = "y" ]]; then
            echo "  - edge_0 to edge_$((num_edges-1)): Edge fusion servers (ports 50060-$((50060+num_edges-1)))"
            echo "  - (vehicles/RSUs run in base process — no separate containers)"
        else
            echo "  - edge_0 to edge_$((num_edges-1)): Edge servers (ports 50054-$((50054+num_edges-1)))"
        fi
    fi
    if [[ "$use_edge_only" != "Y" && "$use_edge_only" != "y" ]]; then
        echo "  - ego_vehicle_0 to ego_vehicle_$((num_ego-1)): Ego vehicles (indices 0-$((num_ego-1)))"
        if [[ $num_rsu -gt 0 ]]; then
            echo "  - rsu_0 to rsu_$((num_rsu-1)): RSUs (indices 0-$((num_rsu-1)))"
        fi
        echo "  - non_ego_vehicles: Non-ego vehicle controller (index -1)"
    fi
    echo ""
    docker container ls
fi
echo ""

# Monitor for scenario completion or errors
echo "=========================================="
echo "Monitoring scenario execution..."
echo "=========================================="
echo ""
echo "Watching ecav_base logs for completion or errors..."
if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
    echo "Sequential mode: monitoring for base process exit (PID $ECAV_PID)..."
else
    echo "Waiting for 'pushed END' message (press Ctrl+C to stop monitoring)..."
fi
echo ""

# Monitor loop for completion
monitor_timeout=300  # 5 minutes
monitor_elapsed=0
scenario_completed=false

# Track reported errors to avoid duplicate notifications
REPORTED_ERRORS_FILE=$(mktemp /tmp/reported_errors.XXXXXX)
> "$REPORTED_ERRORS_FILE"

while [[ $monitor_elapsed -lt $monitor_timeout ]]; do
    # Check for completion
    if [[ "$use_sequential" = "Y" || "$use_sequential" = "y" ]]; then
        # Sequential: done when base process exits
        if ! kill -0 "$ECAV_PID" 2>/dev/null; then
            echo ""
            echo "✓ Scenario completed (base process exited)"
            scenario_completed=true
            break
        fi
    else
        if grep -qi "pushed END" "$ECAV_LOG"; then
            echo ""
            echo "✓ Scenario completed successfully!"
            scenario_completed=true
            break
        fi
    fi

    # Check for errors (common Python error patterns) - only report new ones
    grep -E "Traceback|Error:|Exception:|WARNING|COLLISION|CRITICAL|FATAL" "$ECAV_LOG" 2>/dev/null > /tmp/ecav_errors_all.txt
    if [[ -s /tmp/ecav_errors_all.txt ]]; then
        # Find errors that haven't been reported yet
        new_errors=()
        while IFS= read -r error_line; do
            # Use md5sum of line as unique identifier to handle special characters
            error_hash=$(echo "$error_line" | md5sum | cut -d' ' -f1)
            if ! grep -q "^$error_hash$" "$REPORTED_ERRORS_FILE" 2>/dev/null; then
                new_errors+=("$error_line")
                echo "$error_hash" >> "$REPORTED_ERRORS_FILE"
            fi
        done < /tmp/ecav_errors_all.txt

        # Only display if there are new errors
        if [[ ${#new_errors[@]} -gt 0 ]]; then
            echo ""
            echo "⚠ New errors detected in ecav_base logs:"
            echo "----------------------------------------"
            for err in "${new_errors[@]}"; do
                echo "$err"
            done
            echo "----------------------------------------"
            echo ""
        fi
    fi

    # Check container health every 30 seconds (skip in sequential mode — no containers)
    if [[ "$use_sequential" != "Y" && "$use_sequential" != "y" ]] \
       && (( monitor_elapsed % 30 == 0 )) && (( monitor_elapsed > 0 )); then
        failed_containers=()

        # Check edges
        for ((e=0; e<$num_edges; e++)); do
            container_name="edge_$e"
            status=$(docker inspect -f '{{.State.Status}}' "$container_name" 2>/dev/null)
            if [[ "$status" != "running" ]]; then
                failed_containers+=("$container_name ($status)")
            fi
        done

        if [[ "$use_edge_only" != "Y" && "$use_edge_only" != "y" ]]; then
            # Check ego vehicles
            for ((i=0; i<$num_ego; i++)); do
                container_name="ego_vehicle_$i"
                status=$(docker inspect -f '{{.State.Status}}' "$container_name" 2>/dev/null)
                if [[ "$status" != "running" ]]; then
                    failed_containers+=("$container_name ($status)")
                fi
            done

            # Check RSUs
            for ((i=0; i<$num_rsu; i++)); do
                container_name="rsu_$i"
                status=$(docker inspect -f '{{.State.Status}}' "$container_name" 2>/dev/null)
                if [[ "$status" != "running" ]]; then
                    failed_containers+=("$container_name ($status)")
                fi
            done

            # Check non-ego vehicles
            status=$(docker inspect -f '{{.State.Status}}' "non_ego_vehicles" 2>/dev/null)
            if [[ "$status" != "running" ]]; then
                failed_containers+=("non_ego_vehicles ($status)")
            fi
        fi

        # Report any failed containers and show their error logs
        if [[ ${#failed_containers[@]} -gt 0 ]]; then
            echo ""
            echo "⚠ Container health check - failed containers detected:"
            for container_info in "${failed_containers[@]}"; do
                # Extract container name (remove status in parentheses)
                container_name=$(echo "$container_info" | sed 's/ (.*//')
                echo ""
                echo "  ╔══════════════════════════════════════════════════════════"
                echo "  ║ FAILED: $container_info"
                echo "  ╠══════════════════════════════════════════════════════════"
                echo "  ║ Last 20 lines of logs:"
                echo "  ╟──────────────────────────────────────────────────────────"
                # Get last 20 lines and indent them
                docker logs --tail 20 "$container_name" 2>&1 | while IFS= read -r line; do
                    echo "  ║ $line"
                done
                echo "  ╟──────────────────────────────────────────────────────────"
                # Try to find specific error patterns
                error_line=$(docker logs "$container_name" 2>&1 | grep -E "Error|Exception|Traceback|FATAL" | tail -1)
                if [[ -n "$error_line" ]]; then
                    echo "  ║ Error summary: $error_line"
                fi
                echo "  ╚══════════════════════════════════════════════════════════"
            done
            echo ""
        fi
    fi

    sleep 5
    ((monitor_elapsed+=5))
    echo -n "."
done

echo ""
echo ""

# Clean up temporary error tracking file
rm -f "$REPORTED_ERRORS_FILE" /tmp/ecav_errors_all.txt

# Stop WorldFusion gRPC server if it was started
if [[ -n "$WF_GRPC_PID" ]]; then
    echo ""
    echo "=========================================="
    echo "Stopping WorldFusion gRPC Server"
    echo "=========================================="
    if kill -0 "$WF_GRPC_PID" 2>/dev/null; then
        echo "Stopping WorldFusion gRPC server (PID: $WF_GRPC_PID)..."
        kill "$WF_GRPC_PID"
        sleep 2
        if kill -0 "$WF_GRPC_PID" 2>/dev/null; then
            kill -9 "$WF_GRPC_PID"
            sleep 1
        fi
        echo "✓ WorldFusion gRPC server stopped"
    else
        echo "WorldFusion gRPC server process already exited."
    fi
    pkill -f "worldfusion_grpc_server" 2>/dev/null || true
    pkill -f "litserve_models" 2>/dev/null || true
fi

# Stop YOLO gRPC server if it was started
if [[ -n "$YOLO_GRPC_PID" ]]; then
    echo ""
    echo "=========================================="
    echo "Stopping YOLO gRPC Server"
    echo "=========================================="
    if kill -0 "$YOLO_GRPC_PID" 2>/dev/null; then
        echo "Stopping YOLO gRPC server (PID: $YOLO_GRPC_PID)..."
        kill "$YOLO_GRPC_PID"
        sleep 2
        if kill -0 "$YOLO_GRPC_PID" 2>/dev/null; then
            kill -9 "$YOLO_GRPC_PID"
            sleep 1
        fi
        echo "✓ YOLO gRPC server stopped"
    else
        echo "YOLO gRPC server process already exited."
    fi
    pkill -f "yolo_grpc_server" 2>/dev/null || true
fi

if [[ "$scenario_completed" == false ]]; then
    echo "⚠ Monitoring timeout reached (5 minutes) or scenario still running."
    echo ""
fi

echo "=========================================="
echo "Session Summary"
echo "=========================================="
echo ""
echo "eCAV base log file: $ECAV_LOG"
echo ""
echo "To view logs for a specific container, use:"
echo "  docker logs -f <container_name>"
echo ""
echo "To check for errors in the base process:"
echo "  grep -E 'Error|Exception|Traceback' $ECAV_LOG"
echo ""
echo "To stop all containers, run:"
echo "  ./stop_actors.sh"
echo ""
