#!/usr/bin/env python3
"""Launch dynamic window frontier exploration with map saving enabled (alias for explore.launch.py)."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('roomwatch_explore'),
                'launch',
                'explore.launch.py'
            ])
        )
    )
    return LaunchDescription([explore_launch])
