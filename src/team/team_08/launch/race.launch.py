"""TEAM08: signal gate + Gazebo-native coordinate replay (no Ackermann adapter)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared = {
        "race_start_mode": "signal",
        "race_start_topic": "/race/start/team_08",
        "speed_topic": "/team08/speed_08",
        "steering_topic": "/team08/steering_08",
        "lidar_topic": "/team08/front_scan_08",
        "use_sim_time": "true",
    }
    args=[DeclareLaunchArgument(k, default_value=v) for k,v in declared.items()]
    bridge=[
        "/team08/front_scan_08@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team08/rear_scan_08@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team08/imu_08@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/team08/front_camera_08/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/team08/replay_go_08@std_msgs/msg/Bool]gz.msgs.Boolean",
        "/team08/speed_08@std_msgs/msg/Float32[gz.msgs.Float",
        "/team08/steering_08@std_msgs/msg/Float32[gz.msgs.Float",
    ]
    args.append(Node(package="ros_gz_bridge",executable="parameter_bridge",
                     name="team08_bridge_08",arguments=bridge,output="screen"))
    args.append(Node(package="team08_race",executable="team08_control",
                     name="team08_control_08",output="screen",
                     parameters=[{
                         "race_start_topic":LaunchConfiguration("race_start_topic"),
                         "use_sim_time":LaunchConfiguration("use_sim_time")
                     }]))
    return LaunchDescription(args)
