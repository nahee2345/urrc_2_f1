"""Competition entry: connect to the operator's world/car; never spawn either."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def setup(context):
    share=Path(get_package_share_directory('team_01_race'))
    mode=LaunchConfiguration('race_start_mode').perform(context)
    if mode not in ('camera','signal'):
        raise ValueError('race_start_mode must be camera or signal')
    names=('gz_partition','use_sim_time','world_name','clock_topic','bridge_clock','race_start_topic',
           'speed_topic','steering_topic','lidar_topic','imu_topic','speed_profile_file',
           'validation_speed_ceiling_mps','camera_mount_config')
    arguments={name:LaunchConfiguration(name).perform(context) for name in names}
    arguments.update(bridge_sensors='true',require_official_start='true' if mode=='signal' else 'false')
    # Both modes still require all five red lamps to be seen and then go dark.
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/race.launch.py')),
                                    launch_arguments=arguments.items())]


def generate_launch_description():
    share=Path(get_package_share_directory('team_01_race'))
    defaults=dict(gz_partition=EnvironmentVariable('GZ_PARTITION',default_value=''),use_sim_time='true',world_name='',clock_topic='/team_01/clock',bridge_clock='true',
        race_start_mode='camera',race_start_topic='/race/start/team_01',speed_topic='/team_01/speed',
        steering_topic='/team_01/steering',lidar_topic='/team_01/scan',imu_topic='/team_01/imu',
        speed_profile_file=str(share/'config/speed_profile_standard.yaml'),validation_speed_ceiling_mps='12.0',
        camera_mount_config=str(share/'config/camera_mount.yaml'))
    return LaunchDescription([*[DeclareLaunchArgument(name,default_value=value) for name,value in defaults.items()],
                              OpaqueFunction(function=setup)])
