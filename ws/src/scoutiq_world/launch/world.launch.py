import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def launch_world(context, *args, **kwargs):
    light_str = LaunchConfiguration('light').perform(context).strip().lower()
    headless_str = LaunchConfiguration('headless').perform(context).strip().lower()

    is_light = light_str in ('true', '1', 'yes')
    is_headless = headless_str in ('true', '1', 'yes')

    try:
        pkg_share = get_package_share_directory('scoutiq_world')
    except Exception:
        pkg_share = get_package_share_directory('scoutiq_world')
    world_filename = 'small_house_light.sdf' if is_light else 'small_house.sdf'
    world_path = os.path.join(pkg_share, 'worlds', world_filename)
    models_path = os.path.join(pkg_share, 'models')

    curr_ign_res = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    curr_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    curr_sdf_path = os.environ.get('SDF_PATH', '')

    ign_resource_path = f"{models_path}:{curr_ign_res}" if curr_ign_res else models_path
    gz_resource_path = f"{models_path}:{curr_gz_res}" if curr_gz_res else models_path
    sdf_path = f"{models_path}:{curr_sdf_path}" if curr_sdf_path else models_path

    env = {
        'IGN_GAZEBO_RESOURCE_PATH': ign_resource_path,
        'GZ_SIM_RESOURCE_PATH': gz_resource_path,
        'SDF_PATH': sdf_path,
    }

    cmd = ['ign', 'gazebo']
    if is_headless:
        cmd.extend(['-s', '-r', '--headless-rendering'])
    else:
        cmd.extend(['-r'])
    cmd.append(world_path)

    return [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_resource_path),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='SDF_PATH', value=sdf_path),
        ExecuteProcess(
            cmd=cmd,
            output='screen',
            additional_env=env
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'light',
            default_value='false',
            description='Load lightweight world optimized for CPU rendering (true/false)'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo in headless mode without GUI (true/false)'
        ),
        OpaqueFunction(function=launch_world),
    ])
