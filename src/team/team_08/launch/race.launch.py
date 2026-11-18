"""TEAM08 launch. Organizer owns the world, spawn, global /clock and timing."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared={
        "race_start_mode":"camera",
        "race_start_topic":"/race/start/team_08",
        "speed_topic":"/team08/speed_08",
        "steering_topic":"/team08/steering_08",
        "lidar_topic":"/team08/front_scan_08",
        "use_sim_time":"true",
    }
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in declared.items()]
    # All private cross-package bridge topics end with _08. External
    # /race/start/team_08 and /clock are organizer-owned, not renamed.
    bridge=[
        "/team08/pose_08@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/team08/odom_08@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/team08/start_image_08@sensor_msgs/msg/Image[gz.msgs.Image",
        "/team08/front_scan_08@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team08/rear_scan_08@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team08/imu_08@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/team08/cmd_vel_08@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]
    args.append(Node(package="ros_gz_bridge",executable="parameter_bridge",
        name="team08_bridge_08",arguments=bridge,output="screen"))
    shared={
        "race_start_topic":LaunchConfiguration("race_start_topic"),
        "speed_topic":LaunchConfiguration("speed_topic"),
        "steering_topic":LaunchConfiguration("steering_topic"),
        "use_sim_time":LaunchConfiguration("use_sim_time"),
    }
    args.append(Node(package="team08_race",executable="team08_control",
        name="team08_control_08",output="screen",
        parameters=[dict(shared,race_start_mode="camera")]))
    args.append(Node(package="team08_race",executable="team08_adapter",
        name="team08_adapter_08",output="screen",parameters=[shared]))
    return LaunchDescription(args)
