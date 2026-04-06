#!/bin/bash

echo "=========================================="
echo "Stopping eCAV Actor Containers"
echo "=========================================="
echo ""

# Get list of running containers
running_containers=$(docker ps -q)

if [[ -z "$running_containers" ]]; then
    echo "No running containers found."
else
    echo "Stopping all running containers..."
    docker stop $(docker ps -q)
    echo "✓ All containers stopped"
fi

echo ""

# Get list of all containers (including stopped)
all_containers=$(docker ps -a -q)

if [[ -z "$all_containers" ]]; then
    echo "No containers to remove."
else
    echo "Removing all containers..."
    docker rm $(docker ps -a -q)
    echo "✓ All containers removed"
fi

echo ""
echo "=========================================="
echo "Stopping Base eCAV Process"
echo "=========================================="
echo ""

# Check if base eCAV process is running (on host, not in container)
if pgrep -f "python ecav.py.*-d" > /dev/null; then
    echo "Stopping base eCAV process..."
    pkill -f "python ecav.py.*-d"
    sleep 2

    # Force kill if still running
    if pgrep -f "python ecav.py.*-d" > /dev/null; then
        echo "Force killing base eCAV process..."
        pkill -9 -f "python ecav.py.*-d"
        sleep 1
    fi

    if ! pgrep -f "python ecav.py.*-d" > /dev/null; then
        echo "✓ Base eCAV process stopped"
    else
        echo "⚠ Warning: Some eCAV processes may still be running"
    fi
else
    echo "Base eCAV process is not running."
fi

echo ""
echo "=========================================="
echo "Stopping WorldFusion gRPC / LitServe Server"
echo "=========================================="
echo ""

if pgrep -f "worldfusion_grpc_server" > /dev/null; then
    echo "Stopping WorldFusion gRPC server..."
    pkill -f "worldfusion_grpc_server"
    sleep 2
    if pgrep -f "worldfusion_grpc_server" > /dev/null; then
        pkill -9 -f "worldfusion_grpc_server"
        sleep 1
    fi
    if ! pgrep -f "worldfusion_grpc_server" > /dev/null; then
        echo "✓ WorldFusion gRPC server stopped"
    else
        echo "⚠ Warning: WorldFusion gRPC server may still be running"
    fi
else
    echo "WorldFusion gRPC server is not running."
fi

if pgrep -f "yolo_grpc_server" > /dev/null; then
    echo "Stopping YOLO gRPC server..."
    pkill -f "yolo_grpc_server"
    sleep 2
    if pgrep -f "yolo_grpc_server" > /dev/null; then
        pkill -9 -f "yolo_grpc_server"
        sleep 1
    fi
    if ! pgrep -f "yolo_grpc_server" > /dev/null; then
        echo "✓ YOLO gRPC server stopped"
    else
        echo "⚠ Warning: YOLO gRPC server may still be running"
    fi
fi

if pgrep -f "litserve_models" > /dev/null; then
    echo "Stopping LitServe inference server..."
    pkill -f "litserve_models"
    sleep 2
    if pgrep -f "litserve_models" > /dev/null; then
        pkill -9 -f "litserve_models"
        sleep 1
    fi
    if ! pgrep -f "litserve_models" > /dev/null; then
        echo "✓ LitServe stopped"
    else
        echo "⚠ Warning: LitServe processes may still be running"
    fi
fi

echo ""
echo "=========================================="
echo "Stopping Carla"
echo "=========================================="
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

echo ""
echo "=========================================="
echo "Killing Any Remaining Python Processes"
echo "=========================================="
echo ""

# sometimes we just get python as a process still running. but this is ALWAYS a lingering scenario
if pgrep -f "python .*-d" > /dev/null; then
    echo "Stopping python process..."
    pkill -f "python .*-d"
    sleep 2

    # Force kill if still running
    if pgrep -f "python .*-d" > /dev/null; then
        echo "Force killing python process..."
        pkill -9 -f "python .*-d"
        sleep 1
    fi

    if ! pgrep -f "python .*-d" > /dev/null; then
        echo "✓ All python processes stopped"
    else
        echo "⚠ Warning: Some python processes may still be running"
    fi
else
    echo "No python processes are running."
fi


echo ""
echo "=========================================="
echo "Current status:"
echo "=========================================="
echo ""
echo "Docker containers:"
docker container ls -a
echo ""
echo "Base eCAV processes:"
if pgrep -f "python ecav.py.*-d" > /dev/null; then
    pgrep -af "python ecav.py.*-d"
else
    echo "  No base eCAV processes running"
fi
echo ""
echo "Carla processes:"
if pgrep -f "CarlaUE4" > /dev/null; then
    pgrep -af "CarlaUE4"
else
    echo "  No Carla processes running"
fi
echo ""
echo "WorldFusion / YOLO gRPC / LitServe processes:"
if pgrep -f "worldfusion_grpc_server\|yolo_grpc_server\|litserve_models" > /dev/null; then
    pgrep -af "worldfusion_grpc_server\|yolo_grpc_server\|litserve_models"
else
    echo "  No WorldFusion / YOLO gRPC / LitServe processes running"
fi
echo ""
