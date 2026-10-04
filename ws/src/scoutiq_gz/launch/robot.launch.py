import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_scoutiq_gz = FindPackageShare('scoutiq_gz')
    pkg_scoutiq_desc = FindPackageShare('scoutiq_description')

    default_world_path = PathJoinSubstitution([
        pkg_scoutiq_gz, 'worlds', 'empty.sdf'
    ])
    bridge_config_path = PathJoinSubstitution([
        pkg_scoutiq_gz, 'config', 'ros_gz_bridge.yaml'
    ])
    xacro_file_path = PathJoinSubstitution([
        pkg_scoutiq_desc, 'urdf', 'fetch.urdf.xacro'
    ])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to SDF world file or world name',
    )
    start_gazebo_arg = DeclareLaunchArgument(
        'start_gazebo',
        default_value='true',
        description='Whether to start Ignition Gazebo simulation process',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run Ignition Gazebo headless (--headless-rendering)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn x')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn y')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Spawn z')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw')

    world = LaunchConfiguration('world')
    start_gazebo = LaunchConfiguration('start_gazebo')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    pkg_scoutiq_world = FindPackageShare('scoutiq_world')
    world_models_path = PathJoinSubstitution([pkg_scoutiq_world, 'models'])

    # Ensure Ignition Fortress resolves model://scoutiq_description/meshes/... and model://aws_...
    # Parent of scoutiq_description package share is <prefix>/share
    parent_share_path = PathJoinSubstitution([pkg_scoutiq_desc, '..'])
    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=parent_share_path,
    )
    set_ign_world_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=world_models_path,
    )
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=parent_share_path,
    )
    set_gz_world_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=world_models_path,
    )

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file_path,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # Ignition Gazebo headless process
    ign_gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', '--headless-rendering', world],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", start_gazebo, "' == 'true' and '", headless, "' == 'true'"])
        ),
    )

    # Ignition Gazebo GUI process
    ign_gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", start_gazebo, "' == 'true' and '", headless, "' == 'false'"])
        ),
    )

    # Spawn entity using ros_gz_sim create
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_fetch',
        output='screen',
        arguments=[
            '-name', 'fetch',
            '-topic', 'robot_description',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
    )
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn_robot],
    )

    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        world_arg,
        start_gazebo_arg,
        headless_arg,
        use_sim_time_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        set_ign_resource_path,
        set_ign_world_path,
        set_gz_resource_path,
        set_gz_world_path,
        robot_state_publisher,
        ign_gazebo_headless,
        ign_gazebo_gui,
        delayed_spawn,
        ros_gz_bridge,
    ])
