#!/usr/bin/env bash
set -e
export PYTHONNOUSERSITE=1

RUN_DIR="$1"
if [ -z "${RUN_DIR}" ]; then
    TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
    RUN_DIR="$(pwd)/data/${TIMESTAMP}"
fi

source /opt/ros/humble/setup.bash
[ -f "ws/install/setup.bash" ] && source ws/install/setup.bash
[ -d ".venv" ] && source .venv/bin/activate

echo "Starting Exploration & Mapping into: ${RUN_DIR}"
ros2 launch roomwatch_bringup explore.launch.py run_dir:="${RUN_DIR}"
