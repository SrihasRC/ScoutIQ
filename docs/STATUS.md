# Status board (each agent edits only its own row)

| WP | Branch | Owner | State | Evidence |
|---|---|---|---|---|
| WP0 scaffold | main | main agent | done | Base setup, uv venv, Ignition Fortress spike, mock_robot, v0-scaffold |
| WP1 world | wp1-world | WP1 agent | done | Ported small_house.sdf (87 models, RTF 1.0) & small_house_light.sdf (34 models, RTF 1.0) to Ignition Fortress SDF 1.8 with physics, sensors/ogre2, scene-broadcaster, user-commands; colcon build passes; world.launch.py tested headless |
| WP2 robot+sensors | wp2-robot | WP2 agent | done | colcon build clean; robot.launch.py spawns fetch in Fortress; /clock, /cmd_vel (drives >0.7m), /odom, /scan (360 samples, 10Hz, laser_link), /head_camera/rgb/image_raw (640x480, rgb8), /head_camera/depth_registered/image_raw (640x480, 32FC1 m), /head_camera/rgb/camera_info, /joint_states (15 joints), TF (odom->base_link->laser_link/cam); tests/test_wp2_robot.py passes |
| WP3 joints | wp3-joints | WP3 agent | done | Added JointPositionController plugins for 10 joints (torso, arm, head); bridged Float64->Double; CLI tools tuck_arm & set_head implemented and verified on Fortress Fetch; all joints reached target within tolerance (err < 0.04 rad/m); tests/test_wp3_joints.py passed |
| WP4 nav | wp4-nav | wp4-agent | done | colcon build clean; 5 pytest tests pass; mapping.launch.py (slam_toolbox async + Nav2) & localize.launch.py active against mock_robot; pub_initial_pose & save_map verified |
| WP5 explore | wp5-explore | WP5 agent | done | builds (colcon); 8/8 unit tests pass; test_mock_robot_explore.py pass on ROS_DOMAIN_ID=15; test_growing_map_explore.py pass; map.pgm/yaml saved; docs/explore.md |
| WP6 core | wp6-core | wp6 agent | done | colcon build OK; 20/20 parity & mock tests passed in tests/core; mock_robot integration verified (domain 16) |
| WP7 perception | wp7-perception | wp7-agent | done | Ported robokit to standalone roomwatch-perception; CPU-only GroundingDINO+MobileSAM; CLIs rw-semantic-construct & rw-semantic-update verified against mock_robot (ROS_DOMAIN_ID=17); 11/11 pytest passed; CPU demo 5.56s/frame. |
| WP8 bringup+docs | wp8-bringup | WP8 agent | done | Created roomwatch_bringup package with sim.launch.py, explore.launch.py, traverse.launch.py, update.launch.py, roomwatch.rviz; runner scripts run_pipeline.sh, run_explore.sh, run_traverse.sh, run_update.sh; tests/e2e/test_pipeline.py passes 100% with all 7 CONTRACT artifacts verified; updated comprehensive README.md |
