#!/usr/bin/env python3
"""
Traverse & semantic map update launch file.
Localizes robot with AMCL/Nav2, executes surveillance trajectory,
and updates existing semantic graph (graph_updated.json) with changes.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_launch(context, *args, **kwargs):
    pkg_nav = get_package_share_directory('scoutiq_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_dir_str = LaunchConfiguration('run_dir').perform(context)
    fake_detector = LaunchConfiguration('fake_detector').perform(context).lower() == 'true'
    text_prompt = LaunchConfiguration('text_prompt').perform(context)
    navigate_robot = LaunchConfiguration('navigate').perform(context).lower() == 'true'

    if not run_dir_str:
        run_dir_str = os.path.join(os.getcwd(), 'data')

    map_yaml = os.path.join(run_dir_str, 'map.yaml')
    traj_npz = os.path.join(run_dir_str, 'surveillance_traj.npz')
    input_graph = os.path.join(run_dir_str, 'graph.json')
    output_graph = os.path.join(run_dir_str, 'graph_updated.json')

    # 1. Nav2 Localization (AMCL + map_server + navigation)
    localize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'localize.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'autostart': 'true',
        }.items(),
    )

    # 2. Initial pose publisher (sets initial pose to 0,0,0)
    pub_initial_pose_node = Node(
        package='scoutiq_nav',
        executable='pub_initial_pose',
        name='pub_initial_pose',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    delayed_init_pose = TimerAction(period=3.0, actions=[pub_initial_pose_node])

    # 3. Perception semantic update CLI (rw-semantic-update)
    update_cmd = [
        'rw-semantic-update',
        '--input', input_graph,
        '--output', output_graph,
        '--text-prompt', text_prompt,
    ]
    if fake_detector:
        update_cmd.append('--fake-detector')

    update_proc = ExecuteProcess(
        cmd=update_cmd,
        output='screen',
    )
    delayed_update = TimerAction(period=4.0, actions=[update_proc])

    # 4. Trajectory publisher for RViz
    pub_traj_node = Node(
        package='scoutiq_core',
        executable='publish_traj',
        name='publish_traj',
        arguments=[traj_npz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 5. Trajectory navigation runner
    navigate_node = Node(
        package='scoutiq_core',
        executable='navigate',
        name='navigate_trajectory',
        arguments=[traj_npz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    delayed_navigate = TimerAction(period=6.0, actions=[navigate_node])

    actions = [
        LogInfo(msg=f"Update run directory: {run_dir_str}"),
        localize_launch,
        delayed_init_pose,
        delayed_update,
        pub_traj_node,
    ]
    if navigate_robot:
        actions.append(delayed_navigate)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim clock'),
        DeclareLaunchArgument('run_dir', default_value='', description='Run directory path with map.yaml, surveillance_traj.npz & graph.json'),
        DeclareLaunchArgument('fake_detector', default_value='false', description='Use fast fake perception detector'),
        DeclareLaunchArgument('text_prompt', default_value='table . door . chair .', description='Perception detection prompt'),
        DeclareLaunchArgument('navigate', default_value='true', description='Execute trajectory navigation'),
        OpaqueFunction(function=setup_launch),
    ])
