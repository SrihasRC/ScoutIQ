#!/usr/bin/env python3
"""
Autonomous exploration stage launch file.
Launches slam_toolbox, Nav2 navigation stack, frontier explorer, and pose recorder.
Saves map.pgm, map.yaml, and pose files to data/<run_dir>/.
"""

import datetime
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_launch(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('roomwatch_nav')
    pkg_explore = get_package_share_directory('roomwatch_explore')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_dir_str = LaunchConfiguration('run_dir').perform(context)
    record_pose = LaunchConfiguration('record_pose')

    if not run_dir_str:
        timestamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        run_dir_str = os.path.join(os.getcwd(), 'data', timestamp)

    os.makedirs(run_dir_str, exist_ok=True)
    os.makedirs(os.path.join(run_dir_str, 'pose'), exist_ok=True)

    # 1. SLAM + Nav2 Mapping
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'nav2': 'true',
        }.items(),
    )

    # 2. Frontier Exploration
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_explore, 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'save_map': 'true',
            'run_dir': run_dir_str,
        }.items(),
    )

    # 3. Pose Recorder (roomwatch_core save_data)
    save_data_node = Node(
        package='roomwatch_core',
        executable='save_data',
        name='save_data_recorder',
        arguments=['1.0', run_dir_str],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(record_pose),
    )

    return [
        LogInfo(msg=f"Exploration saving artifacts to: {run_dir_str}"),
        mapping_launch,
        explore_launch,
        save_data_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim clock'),
        DeclareLaunchArgument('run_dir', default_value='', description='Run directory path for artifacts'),
        DeclareLaunchArgument('record_pose', default_value='true', description='Record pose NPZs'),
        OpaqueFunction(function=setup_launch),
    ])
