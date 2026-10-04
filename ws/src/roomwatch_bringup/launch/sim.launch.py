#!/usr/bin/env python3
"""
Simulation bringup launch file for roomwatch.
Spawns the Fetch robot in AWS small house (or light variant), tucks the arm,
or optionally starts the mock robot for fast headless unit testing.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('roomwatch_bringup')
    pkg_world = get_package_share_directory('roomwatch_world')
    pkg_gz = get_package_share_directory('roomwatch_gz')

    mock_arg = DeclareLaunchArgument(
        'mock',
        default_value='false',
        description='If true, runs mock robot instead of Ignition Gazebo',
    )
    light_arg = DeclareLaunchArgument(
        'light',
        default_value='false',
        description='If true, uses lightweight house world (fewer CPU objects)',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Custom SDF world path (empty = default house)',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Ignition Gazebo headless (--headless-rendering)',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz2 visualization',
    )
    tuck_arg = DeclareLaunchArgument(
        'tuck',
        default_value='true',
        description='Automatically tuck robot arm after spawning',
    )
    set_head_arg = DeclareLaunchArgument(
        'set_head',
        default_value='true',
        description='Automatically tilt head down after spawning',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    mock = LaunchConfiguration('mock')
    light = LaunchConfiguration('light')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    tuck = LaunchConfiguration('tuck')
    set_head = LaunchConfiguration('set_head')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resolve world path
    world_file = PythonExpression([
        "('", world, "' if '", world, "' != '' else os.path.join('", pkg_world, "', 'worlds', 'small_house_light.sdf' if '", light, "' == 'true' else 'small_house.sdf'))"
    ])

    # 1. Real Gazebo Simulation
    robot_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'headless': headless,
            'start_gazebo': 'true',
            'use_sim_time': use_sim_time,
            'x': '0.0',
            'y': '0.0',
            'z': '0.05',
            'yaw': '0.0',
        }.items(),
        condition=UnlessCondition(mock),
    )

    # Arm tucking (after robot spawns)
    tuck_arm_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'roomwatch_gz', 'tuck_arm', '--timeout', '15.0', '--tolerance', '0.15'],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mock, "' == 'false' and '", tuck, "' == 'true'"])
        ),
    )
    delayed_tuck = TimerAction(period=6.0, actions=[tuck_arm_cmd])

    # Head setting
    set_head_cmd = ExecuteProcess(
        cmd=['ros2', 'run', 'roomwatch_gz', 'set_head', '--tilt', '0.394', '--pan', '0.0', '--timeout', '10.0'],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", mock, "' == 'false' and '", set_head, "' == 'true'"])
        ),
    )
    delayed_head = TimerAction(period=8.0, actions=[set_head_cmd])

    # 2. Mock robot (for fast testing without GPU/sim overhead)
    # Find repository root from pkg_bringup
    # pkg_bringup is typically <ws>/install/roomwatch_bringup/share/roomwatch_bringup
    repo_root = os.path.abspath(os.path.join(pkg_bringup, '..', '..', '..', '..'))
    mock_robot_script = os.path.join(repo_root, 'tests', 'mock_robot', 'mock_robot.py')

    mock_robot_node = ExecuteProcess(
        cmd=['python3', mock_robot_script],
        output='screen',
        condition=IfCondition(mock),
    )

    # 3. RViz2
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'roomwatch.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        mock_arg,
        light_arg,
        world_arg,
        headless_arg,
        rviz_arg,
        tuck_arg,
        set_head_arg,
        use_sim_time_arg,
        robot_sim_launch,
        delayed_tuck,
        delayed_head,
        mock_robot_node,
        rviz_node,
    ])
