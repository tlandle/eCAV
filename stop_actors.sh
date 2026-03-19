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
echo "Stopping Base OpenCDA Process"
echo "=========================================="
echo ""

# Check if base OpenCDA process is running (on host, not in container)
if pgrep -f "python opencda.py.*-d" > /dev/null; then
    echo "Stopping base OpenCDA process..."
    pkill -f "python opencda.py.*-d"
    sleep 2

    # Force kill if still running
    if pgrep -f "python opencda.py.*-d" > /dev/null; then
        echo "Force killing base OpenCDA process..."
        pkill -9 -f "python opencda.py.*-d"
        sleep 1
    fi

    if ! pgrep -f "python opencda.py.*-d" > /dev/null; then
        echo "✓ Base OpenCDA process stopped"
    else
        echo "⚠ Warning: Some OpenCDA processes may still be running"
    fi
else
    echo "Base OpenCDA process is not running."
fi

echo ""
echo "=========================================="
echo "Stopping LitServe Server"
echo "=========================================="
echo ""

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
else
    echo "LitServe is not running."
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
echo "Current status:"
echo "=========================================="
echo ""
echo "Docker containers:"
docker container ls -a
echo ""
echo "Base OpenCDA processes:"
if pgrep -f "python opencda.py.*-d" > /dev/null; then
    pgrep -af "python opencda.py.*-d"
else
    echo "  No base OpenCDA processes running"
fi
echo ""
echo "Carla processes:"
if pgrep -f "CarlaUE4" > /dev/null; then
    pgrep -af "CarlaUE4"
else
    echo "  No Carla processes running"
fi
echo ""
echo "LitServe processes:"
if pgrep -f "litserve_models" > /dev/null; then
    pgrep -af "litserve_models"
else
    echo "  No LitServe processes running"
fi
echo ""
