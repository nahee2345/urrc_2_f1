from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    args={
        'race_start_topic':'/race/start/team_05',
        'race_start_mode':'signal',
        'speed_topic':'/team_05/speed',
        'steering_topic':'/team_05/steering',
        'lidar_topic':'/team_05/scan',
        'use_sim_time':'true',
        'max_speed':'0.85'
    }

    params={k:LaunchConfiguration(k) for k in args}
    params['use_sim_time']=ParameterValue(
        LaunchConfiguration('use_sim_time'),value_type=bool)
    params['max_speed']=ParameterValue(
        LaunchConfiguration('max_speed'),value_type=float)

    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v)
          for k,v in args.items()],

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/team_05/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/team_05/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/team_05/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/team_05/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
            ],
            output='screen'
        ),

        Node(
            package='team05_race',
            executable='controller',
            parameters=[params],
            output='screen'
        )
    ])
