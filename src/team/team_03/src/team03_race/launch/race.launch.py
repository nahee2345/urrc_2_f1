from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge_args = [
        '/team03/scan_03@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team03/imu_03@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/team03/odom_03@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/team03/camera/image_03@sensor_msgs/msg/Image[gz.msgs.Image',
    ]
    return LaunchDescription([
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='team03_bridge_03', arguments=bridge_args, output='screen'),
        Node(package='team03_race', executable='race_driver.py',
             name='team03_race_driver_03', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='team03_race', executable='vehicle_adapter.py',
             name='vehicle_adapter_03', parameters=[{'use_sim_time': True}], output='screen'),
    ])
