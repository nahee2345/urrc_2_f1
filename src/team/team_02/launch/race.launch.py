"""TEAM02: signal gate + Gazebo-native coordinate replay (no Ackermann adapter)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    declared = {
        "race_start_mode": "signal",
        "race_start_topic": "/race/start/team_02",
        "speed_topic": "/team02/speed_02",
        "steering_topic": "/team02/steering_02",
        "lidar_topic": "/team02/front_scan_02",
        "use_sim_time": "true",
    }
    args=[DeclareLaunchArgument(k, default_value=v) for k,v in declared.items()]
    bridge=[
        "/team02/front_scan_02@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team02/rear_scan_02@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team02/imu_02@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/team02/front_camera_02/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/team02/replay_go_02@std_msgs/msg/Bool]gz.msgs.Boolean",
        "/team02/speed_02@std_msgs/msg/Float32[gz.msgs.Float",
        "/team02/steering_02@std_msgs/msg/Float32[gz.msgs.Float",
    ]
    args.append(Node(package="ros_gz_bridge",executable="parameter_bridge",
                     name="team02_bridge_02",arguments=bridge,output="screen"))
    args.append(Node(package="team02_race",executable="team02_control",
                     name="team02_control_02",output="screen",
                     parameters=[{
                         "race_start_topic":LaunchConfiguration("race_start_topic"),
                         "use_sim_time":LaunchConfiguration("use_sim_time")
                     }]))
    return LaunchDescription(args)
