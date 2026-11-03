"""Race launcher: NO world / spawn / global clock; operator owns these."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share=Path(get_package_share_directory('team01_race'))
    names={
        'race_start_mode':'camera',
        'race_start_topic':'/race/start/team01',
        'speed_topic':'/team01/speed',
        'steering_topic':'/team01/steering',
        'lidar_topic':'/team01/front_scan',
        'use_sim_time':'true',
    }
    args=[DeclareLaunchArgument(n,default_value=v) for n,v in names.items()]
    # Real ROS->GZ bridge must use ] for direction; prevent feedback loops.
    bridge_args=[
        '/team01/front_depth@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/road_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team01/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team01/gz_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/team01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ]
    args += [Node(package='ros_gz_bridge',executable='parameter_bridge',
                  name='team01_bridge',arguments=bridge_args,output='screen')]
    args += [Node(package='team01_race',executable='team01_control',name='team01_control',
                  parameters=[str(share/'config/parameters.yaml'),
                              {n:LaunchConfiguration(n) for n in names}],output='screen')]
    args += [Node(package='team01_race',executable='team01_adapter',name='team01_adapter',
                  parameters=[{n:LaunchConfiguration(n) for n in
                               ('race_start_topic','speed_topic','steering_topic','use_sim_time')}],
                  output='screen')]
    return LaunchDescription(args)
