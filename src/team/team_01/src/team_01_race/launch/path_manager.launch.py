from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('team_01_race'))
    defaults = dict(clock_topic='/team_01/clock', primary_steering_source='lidar',
        path_source_mode='auto', use_sim_time='true', lidar_topic='/team_01/scan',
        camera_mount_config='', speed_profile_file=str(share/'config/speed_profile_standard.yaml'))
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='team_01_race', executable='path_manager_node', output={'both':'log'},
            remappings=[('/clock', LaunchConfiguration('clock_topic'))],
            parameters=[str(share/'config/path_manager.yaml'), LaunchConfiguration('speed_profile_file'),
                {key: LaunchConfiguration(key) for key in
                 ('primary_steering_source', 'use_sim_time', 'lidar_topic', 'path_source_mode')}]),
    ])
