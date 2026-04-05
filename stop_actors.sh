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
echo "LitServe processes:"
if pgrep -f "litserve_models" > /dev/null; then
    pgrep -af "litserve_models"
else
    echo "  No LitServe processes running"
fi
echo ""
