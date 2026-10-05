# ScoutIQ

Autonomous Surveillance & Persistent Open-Vocabulary 3D Semantic Mapping for Indoor Environments.

Rebuilt entirely from legacy ROS 1 / Docker into **native ROS 2 Humble** and **Ignition Gazebo 6 (Fortress)** with **CPU-only ML inference** (GroundingDINO + MobileSAM) and virtual environment isolation via `.venv`.

---

## 1. System Overview

```
[ Ignition Fortress / Mock ] <--- (ros_gz_bridge) ---> [ ROS 2 Humble Stack ]
        |                                                     |
  Fetch Robot + World                               +-- SLAM & Nav2 (scoutiq_nav)
  - DiffDrive (/cmd_vel, /odom)                     +-- Frontier Explore (scoutiq_explore)
  - 2D LiDAR (/scan)                                +-- Trajectory / Poses (scoutiq_core)
  - RGB-D Carmine (/head_camera)                    +-- CPU Perception (scoutiq_perception)
  - Arm/Head Controllers (scoutiq_gz)               +-- Orchestration (scoutiq_bringup)
```

### Operational Pipeline Stages:
1. **Autonomous Exploration & Mapping**: Dynamic search window frontier exploration explores the unknown environment, builds a 2D occupancy grid (`map.pgm`, `map.yaml`) using `slam_toolbox`, and records robot poses.
2. **Surveillance Trajectory Planning**: Extracts the robot traversal path and solves the Traveling Salesperson Problem (TSP) to generate an optimal surveillance trajectory (`surveillance_traj.npz`).
3. **Semantic Map Construction**: Robot traverses the surveillance route under AMCL localization. Synchronized RGB-D frames are processed by GroundingDINO (bounding boxes) and MobileSAM (segmentation masks), back-projected to 3D map coordinates, and stored in an open-vocabulary semantic graph (`graph.json`).
4. **Persistent Semantic Map Update**: Robot revisits the surveillance route in a subsequent pass, detects moved, removed, or added objects, and writes the updated scene graph (`graph_updated.json`).

---

## 2. Directory Layout

```
scoutiq/ (symlinked from roomwatch/)
├── ws/src/
│   ├── scoutiq_bringup/      # Master launch files, pipeline scripts, RViz configs
│   ├── scoutiq_world/        # AWS small house worlds (full & light) in SDF 1.8
│   ├── scoutiq_description/  # Fetch URDF, meshes, and Fortress plugin xacros
│   ├── scoutiq_gz/           # Spawner, ros_gz_bridge configuration, tuck_arm & set_head
│   ├── scoutiq_nav/          # Nav2 params (DWB local planner) & async slam_toolbox
│   ├── scoutiq_explore/      # C++ Dynamic window frontier exploration node
│   └── scoutiq_core/         # Trajectory extraction, TSP solver, Nav2 action client
├── perception/               # CPU-only GroundingDINO + MobileSAM (scoutiq_perception)
├── scripts/                  # Setup, environment check, weight download, execution runners
├── tests/                    # Mock robot, unit tests, integration tests, E2E suite
├── docs/                     # CONTRACT.md, WP_PROMPTS.md, STATUS.md
└── data/                     # Run artifacts (map, poses, trajectories, semantic graphs)
```

---

## 3. Quickstart

### Environment Setup
```bash
# 1. Install system apt packages (ROS 2 Humble, Ignition Fortress, build tools)
bash scripts/setup_system.sh

# 2. Create Python virtual environment and install CPU dependencies via uv
bash scripts/setup_venv.sh

# 3. Download pretrained weights (GroundingDINO, MobileSAM, BERT cache)
bash scripts/download_weights.sh

# 4. Verify system environment
bash scripts/check_env.sh
```

### Build Workspace
```bash
source /opt/ros/humble/setup.bash
cd ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

---

## 4. Running the Pipeline

### Automated End-to-End Pipeline
You can run the entire pipeline from exploration through semantic updates with one script:

```bash
# Fast headless execution using mock robot (great for CI & low-power testing):
./scripts/run_pipeline.sh --mock --fake-detector

# Full simulation in Ignition Fortress (headless):
./scripts/run_pipeline.sh --light

# Full simulation with Gazebo GUI and RViz2:
./scripts/run_pipeline.sh --gui
```

### Step-by-Step Manual Execution

#### Step 1: Start Simulation
```bash
# Terminal 1: Launch simulation world and robot
ros2 launch scoutiq_bringup sim.launch.py light:=true headless:=false rviz:=true
```

#### Step 2: Autonomous Exploration & Mapping
```bash
# Terminal 2: Launch SLAM, Nav2, frontier explorer, and pose recorder
RUN_DIR="$(pwd)/data/$(date +%Y-%m-%d_%H-%M-%S)"
ros2 launch scoutiq_bringup explore.launch.py run_dir:="${RUN_DIR}"
```
*When exploration completes, the map is saved to `${RUN_DIR}/map.pgm` and `map.yaml`.*

#### Step 3: Trajectory Planning
```bash
# Terminal 3: Extract trajectory and compute TSP surveillance path
python3 -m scoutiq_core.extract_robot_trajectory "${RUN_DIR}/pose" "${RUN_DIR}/robot_trajectory.json"
python3 -m scoutiq_core.tsp_surveillance_trajectory "${RUN_DIR}/robot_trajectory.json" "${RUN_DIR}/surveillance_traj.npz"
```

#### Step 4: Semantic Map Construction
```bash
# Terminal 2: Run surveillance traverse & build 3D semantic graph
ros2 launch scoutiq_bringup traverse.launch.py run_dir:="${RUN_DIR}"
```
*Saves initial semantic graph to `${RUN_DIR}/graph.json`.*

#### Step 5: Semantic Map Update
```bash
# Terminal 2: Run second traverse & update semantic graph
ros2 launch scoutiq_bringup update.launch.py run_dir:="${RUN_DIR}"
```
*Saves updated semantic graph to `${RUN_DIR}/graph_updated.json`.*

---

## 5. Testing & Verification

ScoutIQ includes complete test suites for every subsystem:

```bash
# Source environment
source /opt/ros/humble/setup.bash
source ws/install/setup.bash
source .venv/bin/activate

# 1. Robot & Sensor Verification (Ignition Fortress)
python3 tests/test_wp2_robot.py

# 2. Joint Control & Tuck/Head Verification
python3 tests/test_wp3_joints.py

# 3. Core Trajectory & Parity Tests (20/20 tests)
pytest tests/core

# 4. Perception Pipeline Tests (11/11 tests)
pytest perception/tests

# 5. Dynamic Window Frontier Exploration Tests (13/13 tests)
colcon test --packages-select scoutiq_explore && colcon test-result --all

# 6. Complete End-to-End Pipeline Integration Test
python3 tests/e2e/test_pipeline.py
```

---

## 6. Output Artifacts

Every completed run produces a standardized bundle under `data/<run_dir>/` strictly adhering to `docs/CONTRACT.md`:

| Artifact | Format | Description |
|---|---|---|
| `map.pgm` | PGM image | 2D Occupancy grid generated by `slam_toolbox` |
| `map.yaml` | YAML | Map metadata (resolution 0.05m, origin, thresholds) |
| `pose/%06d_pose.npz` | NumPy archive | Synchronized camera (`RT_camera`) and robot (`RT_base`) 4x4 poses |
| `robot_trajectory.json` | JSON | NetworkX node-link structure containing historical robot trajectory |
| `surveillance_traj.npz` | NumPy archive | Optimal TSP ordered waypoints for surveillance traversal (`traj` key) |
| `graph.json` | JSON | 3D object semantic graph (categories, 3D poses, robot viewpoints) |
| `graph_updated.json` | JSON | Updated 3D object semantic graph capturing environment changes |
