# Status board (each agent edits only its own row)

| WP | Branch | Owner | State | Evidence |
|---|---|---|---|---|
| WP0 scaffold | main | main agent | done | Base setup, uv venv, Ignition Fortress spike, mock_robot, v0-scaffold |
| WP1 world | wp1-world | WP1 agent | done | Ported small_house.sdf (87 models, RTF 1.0) & small_house_light.sdf (34 models, RTF 1.0) to Ignition Fortress SDF 1.8 with physics, sensors/ogre2, scene-broadcaster, user-commands; colcon build passes; world.launch.py tested headless |
| WP2 robot+sensors | wp2-robot | | in progress | |
| WP3 joints | wp3-joints | | todo | |
| WP4 nav | wp4-nav | wp4-agent | done | colcon build clean; 5 pytest tests pass; mapping.launch.py (slam_toolbox async + Nav2) & localize.launch.py active against mock_robot; pub_initial_pose & save_map verified |
| WP5 explore | wp5-explore | | in progress | |
| WP6 core | wp6-core | wp6 agent | done | colcon build OK; 20/20 parity & mock tests passed in tests/core; mock_robot integration verified (domain 16) |
| WP7 perception | wp7-perception | wp7-agent | done | Ported robokit to standalone roomwatch-perception; CPU-only GroundingDINO+MobileSAM; CLIs rw-semantic-construct & rw-semantic-update verified against mock_robot (ROS_DOMAIN_ID=17); 11/11 pytest passed; CPU demo 5.56s/frame. |
| WP8 bringup+docs | wp8-bringup | | todo | |
