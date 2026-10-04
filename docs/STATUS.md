# Status board (each agent edits only its own row)

| WP | Branch | Owner | State | Evidence |
|---|---|---|---|---|
| WP0 scaffold | main | main agent | in progress | |
| WP1 world | wp1-world | | todo | |
| WP2 robot+sensors | wp2-robot | WP2 agent | done | colcon build clean; robot.launch.py spawns fetch in Fortress; /clock, /cmd_vel (drives >0.7m), /odom, /scan (360 samples, 10Hz, laser_link), /head_camera/rgb/image_raw (640x480, rgb8), /head_camera/depth_registered/image_raw (640x480, 32FC1 m), /head_camera/rgb/camera_info, /joint_states (15 joints), TF (odom->base_link->laser_link/cam); tests/test_wp2_robot.py passes |
| WP3 joints | wp3-joints | | todo | |
| WP4 nav | wp4-nav | | todo | |
| WP5 explore | wp5-explore | | todo | |
| WP6 core | wp6-core | | todo | |
| WP7 perception | wp7-perception | | todo | |
| WP8 bringup+docs | wp8-bringup | | todo | |
