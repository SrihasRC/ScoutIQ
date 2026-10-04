import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('roomwatch_nav')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    nav2 = LaunchConfiguration('nav2')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start up the nav2 stack'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn crashed nodes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level'
    )

    declare_nav2_cmd = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Whether to launch Nav2 navigation stack alongside localization'
    )

    declare_publish_initial_pose_cmd = DeclareLaunchArgument(
        'publish_initial_pose',
        default_value='false',
        description='Whether to publish initial pose (0,0,0) after startup'
    )

    # 1. Start Nav2 localization (map_server + amcl + lifecycle_manager_localization)
    start_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items()
    )

    # 2. Start Nav2 navigation stack (controller, planner, recoveries, bt_navigator)
    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items(),
        condition=IfCondition(nav2)
    )

    # 3. Optional initial pose publisher
    start_initial_pose_cmd = Node(
        package='roomwatch_nav',
        executable='pub_initial_pose',
        name='pub_initial_pose',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(publish_initial_pose),
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_nav2_cmd)
    ld.add_action(declare_publish_initial_pose_cmd)
    ld.add_action(start_localization_cmd)
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_initial_pose_cmd)

    return ld
