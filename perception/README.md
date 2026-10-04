# RoomWatch Perception Pipeline

Standalone Python package for CPU-based 3D semantic mapping in ROS 2 Humble.

## Models
- **GroundingDINO**: Zero-shot open-vocabulary object detector (CPU inference, deformable attention fallback).
- **MobileSAM**: Lightweight segment-anything model (TinyViT, CPU inference).

## CLI Entry Points
- `rw-semantic-construct`: Subscribes to camera stream, performs zero-shot detection & segmentation, and builds `graph.json`.
- `rw-semantic-update`: Reads existing `graph.json`, tracks FOV changes, adds newly discovered objects, removes missing objects, and writes `graph_updated.json`.
- `python -m roomwatch_perception.demo <image> [prompt]`: Runs standalone CPU benchmark and outputs timing report.

## ROS 2 Topics
- Subscribes:
  - `/head_camera/rgb/image_raw` (`sensor_msgs/msg/Image`, QoS SensorData)
  - `/head_camera/depth_registered/image_raw` (`sensor_msgs/msg/Image`, 32FC1 metres, QoS SensorData)
  - `/head_camera/rgb/camera_info` (`sensor_msgs/msg/CameraInfo`, QoS SensorData)
  - `/yes_no` (`std_msgs/msg/Int32`, pause trigger for update node)
- Publishes:
  - `/seg_image` (`sensor_msgs/msg/Image`, QoS SensorData)
  - `/graph_nodes` (`visualization_msgs/msg/MarkerArray`, QoS Reliable)
