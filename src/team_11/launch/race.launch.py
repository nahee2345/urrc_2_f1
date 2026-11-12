"""TEAM11 launch. Organizer owns the world, spawn, global /clock and timing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared={
        "race_start_mode":"signal",
        "race_start_topic":"/race/start/team_11",
        "speed_topic":"/team11/speed_11",
        "steering_topic":"/team11/steering_11",
        "lidar_topic":"/team11/front_scan_11",
        "use_sim_time":"true",
    }
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in declared.items()]
    # All private cross-package bridge topics end with _11. External
    # /race/start/team_11 and /clock are organizer-owned, not renamed.
    bridge=[
        "/team11/pose_11@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/team11/odom_11@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/team11/start_image_11@sensor_msgs/msg/Image[gz.msgs.Image",
        "/team11/front_scan_11@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team11/rear_scan_11@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team11/imu_11@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/team11/cmd_vel_11@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]
    args.append(Node(package="ros_gz_bridge",executable="parameter_bridge",
        name="team11_bridge_11",arguments=bridge,output="screen"))
    shared={
        "race_start_topic":LaunchConfiguration("race_start_topic"),
        "speed_topic":LaunchConfiguration("speed_topic"),
        "steering_topic":LaunchConfiguration("steering_topic"),
        "use_sim_time":LaunchConfiguration("use_sim_time"),
    }
    args.append(Node(package="team11_race",executable="team11_control",
        name="team11_control_11",output="screen",
        parameters=[dict(shared,race_start_mode=LaunchConfiguration("race_start_mode"))]))
    args.append(Node(package="team11_race",executable="team11_adapter",
        name="team11_adapter_11",output="screen",parameters=[shared]))
    return LaunchDescription(args)
