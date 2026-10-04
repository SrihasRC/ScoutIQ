#!/usr/bin/env bash
# One-time system packages for roomwatch (Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress).
# Run with:  sudo bash scripts/setup_system.sh   (or just run it; it uses sudo itself)
set -euo pipefail

SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"

$SUDO apt-get update

# Core ROS 2 / build tooling
$SUDO apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  python3-pip git build-essential cmake

# Navigation + SLAM + localization
$SUDO apt-get install -y \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-teleop-twist-keyboard

# Robot model / TF / visualization
$SUDO apt-get install -y \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-tf-transformations ros-humble-tf2-tools \
  ros-humble-rviz2 ros-humble-vision-msgs

# Control (needed for joint controllers; harmless if unused)
$SUDO apt-get install -y \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-controller-manager ros-humble-joint-trajectory-controller

# Gazebo Fortress <-> ROS (apt ros_gz for Humble is built against Fortress, not Harmonic)
$SUDO apt-get install -y ignition-fortress
$SUDO apt-get install -y \
  ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image \
  ros-humble-cv-bridge python3-opencv python3-numpy python3-scipy python3-networkx \
  python3-shapely python3-matplotlib python3-yaml python3-lxml python3-transforms3d

# Optional: ros2_control for Ignition Fortress (ignore failure)
$SUDO apt-get install -y ros-humble-ign-ros2-control || \
  echo "[info] ros-humble-ign-ros2-control unavailable -> will use native ign joint controllers"

echo "[ok] system packages installed. Next: bash scripts/setup_venv.sh"
