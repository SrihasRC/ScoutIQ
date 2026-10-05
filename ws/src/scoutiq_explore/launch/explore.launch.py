#!/usr/bin/env python3
"""Launch dynamic window frontier exploration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link', description='Base frame of the robot'),
        DeclareLaunchArgument('costmap_topic', default_value='map', description='Topic name for occupancy grid/costmap'),
        DeclareLaunchArgument('costmap_updates_topic', default_value='map_updates', description='Topic name for partial updates'),
        DeclareLaunchArgument('visualize', default_value='true', description='Publish visualization markers'),
        DeclareLaunchArgument('planner_frequency', default_value='0.35', description='Planning rate in Hz'),
        DeclareLaunchArgument('progress_timeout', default_value='12.0', description='Timeout for progress tracking in seconds'),
        DeclareLaunchArgument('potential_scale', default_value='5.0', description='Weight on distance in cost function'),
        DeclareLaunchArgument('orientation_scale', default_value='0.0', description='Weight on orientation'),
        DeclareLaunchArgument('gain_scale', default_value='1.0', description='Weight on frontier size in cost function'),
        DeclareLaunchArgument('transform_tolerance', default_value='0.3', description='TF transform tolerance in seconds'),
        DeclareLaunchArgument('min_frontier_size', default_value='0.5', description='Minimum frontier size in meters'),
        DeclareLaunchArgument('local_frontier_filter_radius', default_value='3.0', description='Local search window radius in meters'),
        DeclareLaunchArgument('min_local_frontiers', default_value='2.0', description='Threshold count of local frontiers before expanding search'),
        DeclareLaunchArgument('min_global_frontiers', default_value='1.0', description='Minimum global frontiers threshold to terminate exploration'),
        DeclareLaunchArgument('global_frontier_filter_radius', default_value='30.0', description='Global search window radius in meters'),
        DeclareLaunchArgument('min_frontier_spacing', default_value='0.8', description='Minimum spacing between frontiers in meters'),
        DeclareLaunchArgument('save_map', default_value='true', description='Save map on exploration completion'),
        DeclareLaunchArgument('run_dir', default_value='', description='Run directory to save map into (e.g. data/<run>)'),
        DeclareLaunchArgument('data_dir', default_value='data', description='Base data directory if run_dir is not set'),
    ]

    explore_node = Node(
        package='scoutiq_explore',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'costmap_topic': LaunchConfiguration('costmap_topic'),
            'costmap_updates_topic': LaunchConfiguration('costmap_updates_topic'),
            'visualize': LaunchConfiguration('visualize'),
            'planner_frequency': LaunchConfiguration('planner_frequency'),
            'progress_timeout': LaunchConfiguration('progress_timeout'),
            'potential_scale': LaunchConfiguration('potential_scale'),
            'orientation_scale': LaunchConfiguration('orientation_scale'),
            'gain_scale': LaunchConfiguration('gain_scale'),
            'transform_tolerance': LaunchConfiguration('transform_tolerance'),
            'min_frontier_size': LaunchConfiguration('min_frontier_size'),
            'local_frontier_filter_radius': LaunchConfiguration('local_frontier_filter_radius'),
            'min_local_frontiers': LaunchConfiguration('min_local_frontiers'),
            'min_global_frontiers': LaunchConfiguration('min_global_frontiers'),
            'global_frontier_filter_radius': LaunchConfiguration('global_frontier_filter_radius'),
            'min_frontier_spacing': LaunchConfiguration('min_frontier_spacing'),
            'save_map': LaunchConfiguration('save_map'),
            'run_dir': LaunchConfiguration('run_dir'),
            'data_dir': LaunchConfiguration('data_dir'),
        }],
    )

    return LaunchDescription(declared_arguments + [explore_node])
