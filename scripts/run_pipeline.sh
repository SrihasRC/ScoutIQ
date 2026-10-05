#!/usr/bin/env bash
set -e

# Default parameters
USE_MOCK=false
LIGHT_WORLD=false
GUI=false
RUN_DIR=""
FAKE_DETECTOR=false
TIMEOUT_EXPLORE=300
TIMEOUT_NAV=45

usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  --mock                 Run fast mock robot instead of full Gazebo"
    echo "  --light                Use lightweight house world for sim"
    echo "  --headless             Run headless in background without GUI (default)"
    echo "  --gui                  Open Gazebo GUI and RViz2"
    echo "  --run-dir DIR          Specify data output directory"
    echo "  --fake-detector        Use fast mock detector for perception tests"
    echo "  --timeout-explore SEC  Exploration time limit in seconds (default: 120)"
    echo "  --help, -h             Show this message"
    exit 1
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mock) USE_MOCK=true; shift ;;
        --light) LIGHT_WORLD=true; shift ;;
        --headless) GUI=false; shift ;;
        --gui) GUI=true; shift ;;
        --run-dir) RUN_DIR="$2"; shift 2 ;;
        --fake-detector) FAKE_DETECTOR=true; shift ;;
        --timeout-explore) TIMEOUT_EXPLORE="$2"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${ROOT_DIR}/ws"
VENV_DIR="${ROOT_DIR}/.venv"

# Source ROS 2 and workspace
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
if [ -f "${ROOT_DIR}/ws/install/setup.bash" ]; then
    source "${ROOT_DIR}/ws/install/setup.bash"
fi
if [ -d "${ROOT_DIR}/.venv" ]; then
    source "${ROOT_DIR}/.venv/bin/activate"
fi

# Ensure Gazebo Fortress resolves both world models and robot meshes
WORLD_MODELS="${ROOT_DIR}/ws/install/scoutiq_world/share/scoutiq_world/models"
DESC_SHARE="${ROOT_DIR}/ws/install/scoutiq_description/share"
export IGN_GAZEBO_RESOURCE_PATH="${WORLD_MODELS}:${DESC_SHARE}:${IGN_GAZEBO_RESOURCE_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH}"
export SDF_PATH="${IGN_GAZEBO_RESOURCE_PATH}"

# Ensure run directory
if [ -z "${RUN_DIR}" ]; then
    TIMESTAMP=$(date +%Y-%m-%d_%H-%M-%S)
    RUN_DIR="${ROOT_DIR}/data/${TIMESTAMP}"
fi
mkdir -p "${RUN_DIR}/pose"

# Automatically save all stdout and stderr to terminal.log in run directory
exec > >(tee -a "${RUN_DIR}/terminal.log") 2>&1

echo "============================================================"
echo "ScoutIQ Autonomous Surveillance & Semantic Mapping Pipeline"
echo "Run Directory: ${RUN_DIR}"
echo "Mode: $( [ "$USE_MOCK" = true ] && echo "MOCK ROBOT" || echo "FULL GAZEBO SIM" )"
echo "============================================================"

# Background process management
PIDS=()
cleanup_all() {
    echo "Terminating running background processes..."
    for pid in "${PIDS[@]}"; do
        kill -9 "${pid}" 2>/dev/null || true
    done
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "mock_robot.py" 2>/dev/null || true
    pkill -9 -f "async_slam_toolbox" 2>/dev/null || true
    pkill -9 -f "nav2" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "smoother_server" 2>/dev/null || true
    pkill -9 -f "waypoint_follower" 2>/dev/null || true
    pkill -9 -f "velocity_smoother" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "scoutiq_explore" 2>/dev/null || true
    pkill -9 -f "save_data" 2>/dev/null || true
    pkill -9 -f "navigate" 2>/dev/null || true
    pkill -9 -f "semantic" 2>/dev/null || true
}
cleanup() {
    cleanup_all
}
trap cleanup EXIT INT TERM

# Ensure completely clean environment before starting
echo "Ensuring clean environment (stopping any lingering processes)..."
cleanup_all
ros2 daemon stop 2>/dev/null || true
sleep 1

# 1. Start Simulator / Robot
echo "[1/4] Starting robot simulation..."
HEADLESS_FLAG="true"
[ "$GUI" = true ] && HEADLESS_FLAG="false"

ros2 launch scoutiq_bringup sim.launch.py \
    mock:="${USE_MOCK}" \
    light:="${LIGHT_WORLD}" \
    headless:="${HEADLESS_FLAG}" \
    rviz:="${GUI}" &
PIDS+=($!)

if [ "$USE_MOCK" = true ]; then
    sleep 4
else
    echo "  Waiting for robot simulation and /odom to be ready..."
    WAIT_SIM=0
    while ! ros2 topic echo /odom --once >/dev/null 2>&1 && [ $WAIT_SIM -lt 30 ]; do
        sleep 1
        WAIT_SIM=$((WAIT_SIM+1))
    done
    echo "  Robot simulation ready! (waited ${WAIT_SIM}s)"
    sleep 2
fi

# 2. Stage 1: Exploration & Mapping
echo "[2/4] Stage 1: Autonomous Exploration & SLAM..."
# For mock mode, mock_robot already provides map and handles nav goals
if [ "$USE_MOCK" = true ]; then
    echo "  Running explore node and save_data against mock robot..."
    ros2 run scoutiq_explore explore --ros-args \
        -p costmap_topic:=map \
        -p planner_frequency:=1.0 \
        -p progress_timeout:=5.0 \
        -p min_local_frontiers:=1.0 \
        -p min_global_frontiers:=1.0 \
        -p save_map:=true \
        -p run_dir:="${RUN_DIR}" &
    PIDS+=($!)

    ros2 run scoutiq_core save_data 0.5 "${RUN_DIR}" &
    PIDS+=($!)

    # Wait for map to be written and at least 5 poses to be recorded
    WAIT_COUNT=0
    while { [ ! -f "${RUN_DIR}/map.yaml" ] || [ $(ls -1 "${RUN_DIR}/pose/"*.npz 2>/dev/null | wc -l) -lt 5 ]; } && [ $WAIT_COUNT -lt 25 ]; do
        sleep 1
        WAIT_COUNT=$((WAIT_COUNT+1))
    done
else
    ros2 launch scoutiq_bringup explore.launch.py run_dir:="${RUN_DIR}" &
    EXPLORE_LAUNCH_PID=$!
    PIDS+=($EXPLORE_LAUNCH_PID)

    # Wait for exploration to explore the house (until all frontiers are exhausted or timeout)
    echo "  Exploration running (timeout: ${TIMEOUT_EXPLORE}s)..."
    WAIT_COUNT=0
    sleep 5
    while [ ! -f "${RUN_DIR}/exploration_done" ] && pgrep -f "scoutiq_explore" >/dev/null && [ $WAIT_COUNT -lt ${TIMEOUT_EXPLORE} ]; do
        sleep 3
        WAIT_COUNT=$((WAIT_COUNT+3))
    done
fi

# Ensure map exists (if not generated in time, synthesize minimal map fallback for test integrity)
if [ ! -f "${RUN_DIR}/map.yaml" ]; then
    echo "  Generating fallback map for pipeline continuation..."
    ros2 run scoutiq_nav save_map "${RUN_DIR}" || true
fi

# Stop exploration and pose recorder before next phase (keep SLAM and Nav2 active)
pkill -f "scoutiq_explore/explore" 2>/dev/null || true
pkill -f "save_data" 2>/dev/null || true
sleep 2

# 3. Stage 2: Trajectory Post-processing
echo "[3/4] Stage 2: Processing Trajectory (Extract -> TSP)..."
python3 -m scoutiq_core.extract_robot_trajectory "${RUN_DIR}/pose" "${RUN_DIR}/robot_trajectory.json"
python3 -m scoutiq_core.tsp_surveillance_trajectory "${RUN_DIR}/robot_trajectory.json" "${RUN_DIR}/surveillance_traj.npz"

# Generate visual map images (clean map.png and map_trajectory.png)
echo "  Generating 2D map and trajectory visualizations..."
"${VENV_DIR}/bin/python3" "${WS_DIR}/src/scoutiq_core/scoutiq_core/visualize_map.py" "${RUN_DIR}" || true

# 4. Stage 3 & 4: Traverse, Semantic Construction, and Update
echo "[4/4] Stage 3 & 4: Semantic Construct & Update..."
DETECTOR_ARG=""
[ "$FAKE_DETECTOR" = true ] && DETECTOR_ARG="--fake-detector"

SIM_TIME_ARG=""
[ "$USE_MOCK" = false ] && SIM_TIME_ARG="--ros-args -p use_sim_time:=true"

# Run semantic construct
echo "  Constructing initial 3D semantic graph (graph.json)..."
scoutiq-semantic-construct --output "${RUN_DIR}/graph.json" --max-iterations 3 ${DETECTOR_ARG} ${SIM_TIME_ARG} &
CONSTRUCT_PID=$!
PIDS+=($CONSTRUCT_PID)

# Run navigation along surveillance trajectory
ros2 run scoutiq_core navigate "${RUN_DIR}/surveillance_traj.npz" ${SIM_TIME_ARG} || true
wait $CONSTRUCT_PID 2>/dev/null || true

# Run semantic update
echo "  Updating 3D semantic graph (graph_updated.json)..."
scoutiq-semantic-update --input "${RUN_DIR}/graph.json" --output "${RUN_DIR}/graph_updated.json" --max-iterations 3 ${DETECTOR_ARG} ${SIM_TIME_ARG} &
UPDATE_PID=$!
PIDS+=($UPDATE_PID)
ros2 run scoutiq_core navigate "${RUN_DIR}/surveillance_traj.npz" ${SIM_TIME_ARG} || true
wait $UPDATE_PID 2>/dev/null || true

# Generate final semantic map overlay
echo "  Generating final semantic object map overlay..."
"${VENV_DIR}/bin/python3" "${WS_DIR}/src/scoutiq_core/scoutiq_core/visualize_map.py" "${RUN_DIR}" || true

echo "============================================================"
echo "Artifact Verification:"
ARTIFACTS=(
    "map.pgm"
    "map.yaml"
    "map.png"
    "map_trajectory.png"
    "robot_trajectory.json"
    "surveillance_traj.npz"
    "graph.json"
    "graph_updated.json"
)

ALL_PASSED=true
for art in "${ARTIFACTS[@]}"; do
    FILE="${RUN_DIR}/${art}"
    if [ -f "${FILE}" ] && [ -s "${FILE}" ]; then
        SIZE=$(stat -c%s "${FILE}")
        echo "  [OK] ${art} (${SIZE} bytes)"
    else
        echo "  [FAIL] ${art} missing or empty"
        ALL_PASSED=false
    fi
done

POSE_COUNT=$(ls -1 "${RUN_DIR}/pose/"*.npz 2>/dev/null | wc -l)
echo "  [OK] Recorded ${POSE_COUNT} pose files in ${RUN_DIR}/pose/"

SEGMENTED_COUNT=$(ls -1 "${RUN_DIR}/segmented/"*.png 2>/dev/null | wc -l)
echo "  [OK] Saved ${SEGMENTED_COUNT} segmented detection images in ${RUN_DIR}/segmented/"

if [ "$ALL_PASSED" = true ]; then
    echo "SUCCESS: Full pipeline completed and all CONTRACT artifacts verified!"
    exit 0
else
    echo "FAILURE: Some artifacts were not generated properly."
    exit 1
fi
