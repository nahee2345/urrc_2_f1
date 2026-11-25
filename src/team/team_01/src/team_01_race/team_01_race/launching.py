from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def role_launch(role):
    config = str(Path(get_package_share_directory('team_01_race'))/'config/racing.yaml')
    executable, name = {
        'depth': ('depth_perception_node', 'team_01_depth_perception'),
        'camera': ('camera_perception_node', 'team_01_camera_perception'),
        'lidar': ('lidar_obstacle_node', 'team_01_lidar_obstacle'),
        'decision': ('decision_node', 'team_01_decision'),
        'control': ('control_adapter_node', 'team_01_control_adapter'),
    }[role]
    defaults = {'speed_profile_file':str(Path(get_package_share_directory('team_01_race'))/'config/speed_profile_standard.yaml'),'clock_topic':'/team_01/clock','primary_steering_source':'lidar','use_sim_time': 'true','camera_mount_config':str(Path(get_package_share_directory('team_01_race'))/'config/camera_mount.yaml')}
    parameters = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    if role == 'camera':
        defaults.update(camera_topic='/team_01/camera/image_raw', camera_info_topic='/team_01/camera/camera_info', camera_process_fps='20.0', camera_debug_fps='5.0',camera_rgb_far_debug='false')
    elif role == 'lidar':
        defaults.update(lidar_topic='/team_01/scan')
    elif role == 'decision':
        defaults.update(validation_speed_ceiling_mps='12.0',race_start_topic='/race/start/team_01', require_official_start='true', imu_topic='/team_01/imu')
    elif role == 'control':
        defaults.update(speed_topic='/team_01/speed', steering_topic='/team_01/steering', imu_topic='/team_01/imu')
    for key in defaults:
        if key not in ('use_sim_time','clock_topic','speed_profile_file'):
            parameter = {'camera_process_fps':'camera.process_fps','camera_debug_fps':'camera.debug_fps','camera_rgb_far_debug':'camera.rgb_far_debug_enabled'}.get(key,key)
            parameters[parameter] = LaunchConfiguration(key)
    parameter_files = [config,str(Path(get_package_share_directory('team_01_race'))/'config/path_manager.yaml'),str(Path(get_package_share_directory('team_01_race'))/'config/corner_braking_preview.yaml')]
    parameter_files.append(str(Path(get_package_share_directory('team_01_race'))/'config/depth_corner.yaml'))
    if role in ('decision','control','depth'):
        parameter_files.append(str(Path(get_package_share_directory('team_01_race'))/'config/corner_control.yaml'))
    parameter_files.append(str(Path(get_package_share_directory('team_01_race'))/'config/corner_path_recovery.yaml'))
    if role == 'control':
        parameter_files.append(str(Path(get_package_share_directory('team_01_race'))/'config/steering_guard.yaml'))
    parameter_files.append(LaunchConfiguration('speed_profile_file'))
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        # Detailed diagnostics must not block control on a slow/stalled terminal.
        # Keep ROS debug topics and on-disk logs; launch still reports exits.
        Node(package='team_01_race', executable=executable, name=name, output={'both':'log'},
             parameters=[*parameter_files, parameters],remappings=[('/clock',LaunchConfiguration('clock_topic'))]),
    ])
