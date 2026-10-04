# AGENTS.md: rules for every coding agent on roomwatch

roomwatch = native **ROS 2 Humble + Gazebo Fortress (Ignition Gazebo 6)** persistent semantic mapping for a mobile manipulator
(Fetch-style robot). No Docker, no ROS 1.

## Read first
1. `docs/CONTRACT.md`: topics, frames, joints, actions, file formats. **Do not change it**; ask the main agent.
2. `.agents/skills/ros2/SKILL.md`: ROS 2 best practices (QoS, launch, params, colcon).
3. `docs/STATUS.md`: update only your own row.

## Environment
- Source ROS then venv: `source /opt/ros/humble/setup.bash && source .venv/bin/activate`
- Python 3.10, **numpy 1.26.4 (<2)**, **CPU-only torch**. Do not install CUDA packages.
- Python deps: `uv pip install ...` into `.venv` only. System deps via apt are installed by the user.
- Build: `cd ws && colcon build --symlink-install --packages-select <pkg>`; `source ws/install/setup.bash`.
- Reference ROS 1 code lives at `../` (original project). **Read-only.** Never copy `rospy`/`actionlib`/`tf` calls.

## Rules
- Work **only** in your work package's owned paths, in your own git worktree/branch (`scripts/new_worktree.sh`).
- Never edit: `AGENTS.md`, `docs/CONTRACT.md`, root `scripts/`, `.gitignore` (main agent only).
- Never merge into `main`; the main agent merges after the gate (build, tests, integration).
- Unique `export ROS_DOMAIN_ID=<N>` and `export IGN_PARTITION=<wp-name>`; run Gazebo headless (`ign gazebo -s -r --headless-rendering`) unless told otherwise.
- **Simulator is Ignition Fortress, NOT Harmonic.** Apt `ros_gz_bridge` is linked to Fortress; Harmonic cannot talk to it. Use `ign gazebo`, message types `ignition.msgs.*`, SDF plugins `ignition-gazebo-*-system` / `ignition::gazebo::systems::*`, SDF version 1.8. Working example: `docs/spike/`.
- No absolute `/home/...` paths in code; use `ament_index_python`, params, or env vars. Use `use_sim_time`.
- Sensors use `SensorDataQoS`; maps use transient-local QoS.
- Commit small and often: `<wp>: <message>`. Done = builds, tests pass, demo command works, STATUS row updated with evidence.

## Work packages
WP1 world, WP2 robot+sensors, WP3 joints, WP4 nav, WP5 explore, WP6 core scripts, WP7 perception, WP8 bringup/docs.
