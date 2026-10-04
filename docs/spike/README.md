# Phase 0 spike result (2026-10-04)
- Gazebo Harmonic (`gz sim` 8) + apt `ros_gz_bridge` = NO DATA (bridge links ignition-transport11 = Fortress).
- Gazebo Fortress (`ign gazebo` 6.18) + apt `ros_gz_bridge`: OK. 10 s: clock 9991, scan 100, rgb 100 (rgb8 320x240), depth 99 (32FC1 metres).
- Software rendering works headless (`--headless-rendering`), no GPU needed.
- Reproduce: `ign gazebo -s -r --headless-rendering docs/spike/fortress_sensors.sdf`, then run
  `ros2 run ros_gz_bridge parameter_bridge '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan' ...`,
  then `/usr/bin/python3 docs/spike/check_sensors.py`.
- Use `ros2 topic ... --no-daemon` if the ros2 daemon is stale; never `pkill -f "ign gazebo"` from a shell whose command line contains that text.
