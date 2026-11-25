"""Official entry: LiDAR/Pure Pursuit steering, RGB-D speed, RGB start gate."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    share = Path(get_package_share_directory('team_01_race'))/'launch'
    defaults = {'gz_partition':EnvironmentVariable('GZ_PARTITION',default_value=''),'speed_profile_file':str(share.parent/'config/speed_profile_standard.yaml'),'clock_topic':'/team_01/clock','world_name':'','validation_speed_ceiling_mps':'12.0','primary_steering_source':'lidar','path_source_mode':'auto','camera_mount_config':str(share.parent/'config/camera_mount.yaml'),'use_sim_time': 'true', 'bridge_clock': 'true', 'bridge_sensors': 'true',
                'race_start_topic': '/race/start/team_01', 'require_official_start': 'true',
                'camera_rgb_far_debug':'false','camera_process_fps': '20.0', 'camera_debug_fps': '5.0',
                'speed_topic': '/team_01/speed', 'steering_topic': '/team_01/steering', 'imu_topic': '/team_01/imu', 'lidar_topic': '/team_01/scan'}
    actions = [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
    for role, arguments in [('path_manager',['use_sim_time','lidar_topic','path_source_mode']),('sensors', ['gz_partition','world_name','use_sim_time','bridge_clock','lidar_topic','imu_topic']),
                            ('depth', ['use_sim_time']), ('camera', ['use_sim_time','camera_rgb_far_debug','camera_process_fps','camera_debug_fps']), ('lidar', ['use_sim_time','lidar_topic']),
                            ('decision', ['validation_speed_ceiling_mps','use_sim_time','race_start_topic','require_official_start','imu_topic']),
                            ('control', ['use_sim_time','speed_topic','steering_topic','imu_topic'])]:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/(role+'.launch.py'))),
                       launch_arguments={key: LaunchConfiguration(key) for key in arguments+['clock_topic','camera_mount_config','primary_steering_source','speed_profile_file']}.items(),
                       **({'condition': IfCondition(LaunchConfiguration('bridge_sensors'))} if role == 'sensors' else {})))
    return LaunchDescription(actions)
