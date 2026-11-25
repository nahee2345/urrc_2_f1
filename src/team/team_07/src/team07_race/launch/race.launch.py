"""Operator launch for original TEAM07 algorithm (no algorithm edits)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    options = {
        'race_start_mode': 'camera',
        'race_start_topic': '/race/start/team_07',
        'speed_topic': '/team07/hybrid/speed',
        'steering_topic': '/team07/hybrid/steering',
        'lidar_topic': '/team07/hybrid/scan',
        'use_sim_time': 'true',
    }
    bridge_topics = [
        '/team07/hybrid/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team07/hybrid/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/team07/hybrid/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team07/hybrid/lidar3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        '/team07/hybrid/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/team07/hybrid/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k,v in options.items()],
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='team07_sensor_bridge', arguments=bridge_topics, output='screen'),
        Node(package='team07_race', executable='hybrid_racer',
             name='team07_hybrid_racer', output='screen',
             parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'race_start_mode': LaunchConfiguration('race_start_mode'),
                 'race_start_topic': LaunchConfiguration('race_start_topic'),
                 'speed_topic': LaunchConfiguration('speed_topic'),
                 'steering_topic': LaunchConfiguration('steering_topic'),
                 'start_gate': True,
                 'require_start_authorization': True,
                 'autonomy_mode': 'race',
                 'cruise_speed': 0.3,
                 'left_speed_limit': 0.3,
                 'path_mode': 'center',
             }]),
    ])
