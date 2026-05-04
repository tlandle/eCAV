#!/bin/bash

# Prompt for scenario configuration
read -p "Enter scenario name (e.g., openscenario_3_edge): " scenario_name
read -p "Enter number of ego vehicles: " num_ego
read -p "Enter number of RSUs: " num_rsu
read -p "Enter number of edges (0 if edge-less scenario): " num_edges
read -p "Use ML (Y/n)? " use_ml
read -p "Use WorldFusion gRPC server for distributed ML inference (Y/n)? " use_litserve
read -p "Use YOLO gRPC server for distributed YOLO inference / late fusion (Y/n)? " use_yolo_grpc
read -p "Rebuild containers (Y/n)? " rebuild

# Validate inputs
if [[ -z "$scenario_name" ]]; then
    echo "Error: Scenario name cannot be empty"
    exit 1
fi

if ! [[ "$num_ego" =~ ^[0-9]+$ ]] || [[ "$num_ego" -lt 1 ]]; then
    echo "Error: Number of ego vehicles must be a positive integer"
    exit 1
fi

if ! [[ "$num_rsu" =~ ^[0-9]+$ ]] || [[ "$num_rsu" -lt 0 ]]; then
    echo "Error: Number of RSUs must be a non-negative integer"
    exit 1
fi

if ! [[ "$num_edges" =~ ^[0-9]+$ ]] || [[ "$num_edges" -lt 0 ]]; then
    echo "Error: Number of edges must be a non-negative integer"
    exit 1
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
read -p "Start Carla locally? (Y/n - select 'n' if using remote Carla): " start_carla

if [[ "$start_carla" = "Y" || "$start_carla" = "y" ]]; then
    read -p "Run Carla in headless mode (no display)? (Y/n): " headless

    echo ""
    echo "Starting Carla..."
    if [[ "$headless" = "Y" || "$headless" = "y" ]]; then
        echo "  Mode: Headless (RenderOffScreen)"
        cd /opt/carla-simulator && ./CarlaUE4.sh -RenderOffScreen &
    else
        echo "  Mode: With display"
        cd /opt/carla-simulator && ./CarlaUE4.sh &
    fi

    CARLA_PID=$!
    echo "  Carla PID: $CARLA_PID"
    echo ""
    echo "Waiting 10 seconds for Carla to initialize..."
    sleep 10

    # Verify Carla started successfully
    if ! pgrep -f "CarlaUE4" > /dev/null; then
        echo "ERROR: Failed to start Carla!"
        echo "Please check the Carla installation at /opt/carla-simulator/"
        exit 1
    fi
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
    _CONDA_ROOT="/home/jordan/anaconda3"
    WF_GRPC_LOG=$(mktemp /tmp/worldfusion_grpc.XXXXXX.log)
    bash -c "source $_CONDA_ROOT/etc/profile.d/conda.sh && conda activate opencda && PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True python ecav/ml_manager/worldfusion_grpc_server.py > '$WF_GRPC_LOG' 2>&1" &
    WF_GRPC_PID=$!
    echo "  WorldFusion gRPC PID: $WF_GRPC_PID"
    echo "  Log file: $WF_GRPC_LOG"
    echo "  Waiting for WorldFusion gRPC server to be ready on port 18002..."

    timeout=90
    elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if conda run -n opencda python -c "import grpc; c=grpc.insecure_channel('localhost:18002'); grpc.channel_ready_future(c).result(timeout=1)" 2>/dev/null; then
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
    _CONDA_ROOT="/home/jordan/anaconda3"
    YOLO_GRPC_LOG=$(mktemp /tmp/yolo_grpc.XXXXXX.log)
    bash -c "source $_CONDA_ROOT/etc/profile.d/conda.sh && conda activate opencda && PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True python ecav/ml_manager/yolo_grpc_server.py > '$YOLO_GRPC_LOG' 2>&1" &
    YOLO_GRPC_PID=$!
    echo "  YOLO gRPC PID: $YOLO_GRPC_PID"
    echo "  Log file: $YOLO_GRPC_LOG"
    echo "  Waiting for YOLO gRPC server to be ready on port 18001..."

    timeout=90
    elapsed=0
    while [[ $elapsed -lt $timeout ]]; do
        if conda run -n opencda python -c "import grpc; c=grpc.insecure_channel('localhost:18001'); grpc.channel_ready_future(c).result(timeout=1)" 2>/dev/null; then
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

echo ""
echo "=========================================="
echo "Starting eCAV Distributed Scenario"
echo "=========================================="
echo "Scenario: $scenario_name"
echo "Ego vehicles: $num_ego"
echo "RSUs: $num_rsu"
echo "Edges: $num_edges"
echo "ML enabled: $use_ml"
echo "WorldFusion gRPC: $use_litserve"
echo "YOLO gRPC: $use_yolo_grpc"
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
_CONDA_ROOT="/home/jordan/anaconda3"
bash -c "source $_CONDA_ROOT/etc/profile.d/conda.sh && conda activate opencda && python -u ecav.py -t '$scenario_name' -v 0.9.15 -d $ml_flag $litserve_flag > '$ECAV_LOG' 2>&1" &
ECAV_PID=$!

echo "  ✓ Base process started (PID: $ECAV_PID)"
echo "  Monitoring log file for 'pushed scenario start' message..."

# Monitor the log file until we see "pushed scenario start"
timeout=60  # 60 second timeout
elapsed=0
while [[ $elapsed -lt $timeout ]]; do
    if grep -qi "pushed scenario start" "$ECAV_LOG" 2>/dev/null; then
        echo "  ✓ Scenario initialization complete!"
        break
    fi

    sleep 2
    ((elapsed+=2))
    echo -n "."
done

echo ""

if [[ $elapsed -ge $timeout ]]; then
    echo "ERROR: Timeout waiting for 'pushed scenario start' message."
    echo "Check logs: tail -f $ECAV_LOG"
    echo ""
    
    # Check if Carla is running
    if pgrep -f "CarlaUE4" > /dev/null; then
        echo "Stopping Carla processes..."
        pkill -f "CarlaUE4"
        sleep 2

        # Force kill if still running
        if pgrep -f "CarlaUE4" > /dev/null; then
            echo "Force killing Carla processes..."
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

echo ""

container_id=0

# Start edge containers (if any)
if [[ $num_edges -gt 0 ]]; then
    echo "Starting $num_edges edge container(s)..."
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
            -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
            -e DISPLAY=$DISPLAY \
            -e TERM \
            ecav-python310:latest \
            python3.10 -u ecav/ecav2/edge_process.py -e $e -P $edge_port

        echo "  ✓ $container_name started"
        wait_for_container_log "$container_name" "registered successfully" 90 || exit 1
    done
    echo ""
fi

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
        -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
        -e DISPLAY=$DISPLAY \
        -e TERM \
        -e QT_X11_NO_MITSHM=1 \
        ecav-python310:latest \
        python3.10 -u ecav.py $ml_flag $litserve_flag -v 0.9.15 -d -i $i -T $((8000 + i))

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
            -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
            -e DISPLAY=$DISPLAY \
            ecav-python310:latest \
            python3.10 -u ecav/ecav2/ecloud_actor_client.py $ml_flag $litserve_flag -v 0.9.15 -i $i

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
    -v /opt/carla-simulator/PythonAPI:/opt/carla-simulator/PythonAPI:ro \
    -e DISPLAY=$DISPLAY \
    ecav-python310:latest \
    python3.10 -u ecav.py $ml_flag $litserve_flag -v 0.9.15 -d -i -1 -T 8100

echo "  ✓ $container_name started"

echo ""
echo "=========================================="
echo "All containers started successfully!"
echo "=========================================="
echo ""
echo "Container summary:"
if [[ $num_edges -gt 0 ]]; then
    echo "  - edge_0 to edge_$((num_edges-1)): Edge servers (ports 50054-$((50054+num_edges-1)))"
fi
echo "  - ego_vehicle_0 to ego_vehicle_$((num_ego-1)): Ego vehicles (indices 0-$((num_ego-1)))"
if [[ $num_rsu -gt 0 ]]; then
    echo "  - rsu_0 to rsu_$((num_rsu-1)): RSUs (indices 0-$((num_rsu-1)))"
fi
echo "  - non_ego_vehicles: Non-ego vehicle controller (index -1)"
echo ""
docker container ls
echo ""

# Monitor for scenario completion or errors
echo "=========================================="
echo "Monitoring scenario execution..."
echo "=========================================="
echo ""
echo "Watching ecav_base logs for completion or errors..."
echo "Waiting for 'pushed END' message (press Ctrl+C to stop monitoring)..."
echo ""

# Monitor loop for completion
monitor_timeout=300  # 5 minutes
monitor_elapsed=0
scenario_completed=false

# Track reported errors to avoid duplicate notifications
REPORTED_ERRORS_FILE=$(mktemp /tmp/reported_errors.XXXXXX)
> "$REPORTED_ERRORS_FILE"

while [[ $monitor_elapsed -lt $monitor_timeout ]]; do
    # Check for completion (log file is already being written to by base process)
    if grep -qi "pushed END" "$ECAV_LOG"; then
        echo ""
        echo "✓ Scenario completed successfully!"
        scenario_completed=true
        break
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

    # Check container health every 30 seconds
    if (( monitor_elapsed % 30 == 0 )) && (( monitor_elapsed > 0 )); then
        failed_containers=()

        # Check edges
        for ((e=0; e<$num_edges; e++)); do
            container_name="edge_$e"
            status=$(docker inspect -f '{{.State.Status}}' "$container_name" 2>/dev/null)
            if [[ "$status" != "running" ]]; then
                failed_containers+=("$container_name ($status)")
            fi
        done

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
