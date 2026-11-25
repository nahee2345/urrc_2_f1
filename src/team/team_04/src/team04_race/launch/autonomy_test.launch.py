"""Spawn a single development vehicle into an existing world; no world or clock."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share=Path(get_package_share_directory('team04_race'))
    values=dict(world='monza',race_start_mode='signal',autonomy_max_speed='3.0')
    # Caller supplies the official grid pose; no track coordinates are embedded.
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in values.items()],
        *[DeclareLaunchArgument(k) for k in ('x','y','z','yaw')],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/test_vehicle.launch.py')),
            launch_arguments={**{k:LaunchConfiguration(k) for k in [*values,'x','y','z','yaw']},'driving_mode':'autonomous'}.items())])
