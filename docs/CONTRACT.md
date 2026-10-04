# roomwatch interface contract (DRAFT v0, frozen at tag `v0-scaffold`)

Only the main agent edits this file. WPs conform to it; request changes via the main agent.
Derived from the original ROS 1 system. Items marked **TBD(spike)** are fixed after the Phase 0 simulator spike.

## Simulator
Ignition Gazebo 6 (Fortress) + apt `ros_gz_*` (bridge types `ignition.msgs.*`). World is 3D; SLAM/Nav2 use a 2D lidar occupancy grid; semantic graph nodes are 3D-derived map poses. Verified spike: `docs/spike/`.

## Frames (TF tree)
`map → odom → base_link → {laser_link, torso_lift_link → … → gripper_link, head_pan_link → head_tilt_link → head_camera_link → head_camera_rgb_optical_frame, head_camera_depth_optical_frame}`
- `map→odom`: slam_toolbox (mapping) or nav2_amcl (localization). `odom→base_link`: Ignition DiffDrive.
- All nodes `use_sim_time:=true` in simulation.

## Topics
| Topic | Type | Publisher | QoS |
|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | gz bridge | default |
| `/cmd_vel` | geometry_msgs/Twist | nav2 / teleop | reliable |
| `/odom` | nav_msgs/Odometry | ign DiffDrive (bridge) | reliable |
| `/scan` | sensor_msgs/LaserScan (frame `laser_link`) | ign gpu_lidar (bridge) | SensorData |
| `/head_camera/rgb/image_raw` | sensor_msgs/Image (rgb8/bgr8) | ign rgbd_camera (bridge) | SensorData |
| `/head_camera/rgb/camera_info` | sensor_msgs/CameraInfo | bridge | SensorData |
| `/head_camera/depth_registered/image_raw` | sensor_msgs/Image (**32FC1, metres**) | bridge | SensorData |
| `/joint_states` | sensor_msgs/JointState | ign JointStatePublisher (bridge) | reliable |
| `/map` | nav_msgs/OccupancyGrid | slam_toolbox / map_server | transient_local |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | initial-pose pub | reliable |
| `/yes_no` | std_msgs/Int32 (pause=1) | user | reliable |
| `/lidar_pc` | sensor_msgs/PointCloud2 | perception listener | SensorData |
| `/seg_image` | sensor_msgs/Image | perception | SensorData |
| `/graph_nodes` | visualization_msgs/MarkerArray | perception | reliable |
| `/visualization_marker_array` | visualization_msgs/MarkerArray | semmap_core publish_traj | reliable |

## Actions
- `navigate_to_pose` (nav2_msgs/NavigateToPose), replaces `move_base`. Goal frame `map`.
- Head/arm/torso: **TBD(spike)**, either `/head_controller/follow_joint_trajectory` (ros2_control) or per-joint position topics `/<joint>/cmd_pos` (Float64) via ign JointPositionController (ign_ros2_control is an apt option).

## Joints (from Fetch)
`torso_lift_joint, head_pan_joint, head_tilt_joint, shoulder_pan_joint, shoulder_lift_joint, upperarm_roll_joint, elbow_flex_joint, forearm_roll_joint, wrist_flex_joint, wrist_roll_joint, l_gripper_finger_joint, r_gripper_finger_joint, l_wheel_joint, r_wheel_joint`
- Tucked-arm pose and head pose: copy the numbers from the original `tuck_arm.py` / `set_head.py`.

## Files / formats (must match the originals byte-for-structure)
- Map: `map.pgm` + `map.yaml` (`image`, `resolution 0.05`, `origin`, thresholds). Saved to `data/<run>/map.*` (not `$HOME`).
- Pose recording: `data/<run>/pose/%06d_pose.npz` with arrays `RT_camera`, `RT_base` (4×4).
- `robot_trajectory.json`, `surveillance_traj.npz` (key `traj`, N×2+), `graph.json`, `graph_updated.json` (networkx node-link; node attrs include `category`, `pose`).
- Run directory layout: `data/<YYYY-mm-dd_HH-MM-SS>/` with `map.pgm`, `map.yaml`, `pose/`, `robot_trajectory.json`, `surveillance_traj.npz`, `graph.json`, `graph_updated.json`.

## Packages (colcon, under ws/src)
`roomwatch_bringup, roomwatch_world, roomwatch_description, roomwatch_gz, roomwatch_nav, roomwatch_explore, roomwatch_core`
Perception lives in `perception/` (venv, not colcon) and exposes console entry points
`rw-semantic-construct` / `rw-semantic-update`.

## Environment per agent
`ROS_DOMAIN_ID=<WP number>`, `IGN_PARTITION=<wp-name>`, headless sim by default.
