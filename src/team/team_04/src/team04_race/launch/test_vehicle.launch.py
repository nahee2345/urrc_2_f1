"""Spawn only into an ALREADY running world. No world/clock bridge here."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p=Path(get_package_share_directory('team04_race'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='team04_test'),
        DeclareLaunchArgument('model_file', default_value=str(p/'models/team04_car/model.sdf')),
        DeclareLaunchArgument('x', default_value='0'), DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.04'), DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('race_start_mode', default_value='signal'),
        DeclareLaunchArgument('driving_mode', default_value='manual'),
        DeclareLaunchArgument('autonomy_max_speed', default_value='0.3'),
        Node(package='ros_gz_sim',executable='create',name='spawn_04',output='screen',
             arguments=['-world',LaunchConfiguration('world'),'-file',LaunchConfiguration('model_file'),
                        '-name','team04','-x',LaunchConfiguration('x'),'-y',LaunchConfiguration('y'),
                        '-z',LaunchConfiguration('z'),'-Y',LaunchConfiguration('yaw')]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'launch/race.launch.py')),
                                 launch_arguments={k:LaunchConfiguration(k) for k in ('race_start_mode','driving_mode','autonomy_max_speed')}.items())])
