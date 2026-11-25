"""Participant-only launch: bridges, navigation, adapter, and safety gates."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    race_start_topic = LaunchConfiguration("race_start_topic")
    speed_topic = LaunchConfiguration("speed_topic")
    steering_topic = LaunchConfiguration("steering_topic")
    lidar_topic = LaunchConfiguration("lidar_topic")
    opponent_lidar_topic = LaunchConfiguration("opponent_lidar_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    share = get_package_share_directory("team10_race")
    config = os.path.join(share, "config", "navigation.yaml")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="vehicle_bridge_10",
        arguments=[
            "/team_10/scan_gz_10@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/team_10/opponent_scan_gz_10@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/team_10/rear_opponent_scan_gz_10@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/model/team_10/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        remappings=[
            ("/team_10/scan_gz_10", lidar_topic),
            ("/team_10/opponent_scan_gz_10", opponent_lidar_topic),
            ("/team_10/rear_opponent_scan_gz_10", "/team_10/rear_opponent_scan_10"),
            ("/model/team_10/cmd_vel", "/team_10/gz_cmd_vel_10"),
        ],
        output="screen",
    )

    navigation = Node(
        package="team10_race",
        executable="navigation",
        name="navigation_10",
        output="screen",
        parameters=[
            config,
            {
                "race_start_topic": race_start_topic,
                "speed_topic": speed_topic,
                "steering_topic": steering_topic,
                "lidar_topic": lidar_topic,
                "opponent_lidar_topic": opponent_lidar_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    adapter = Node(
        package="team10_race",
        executable="race_adapter",
        name="race_adapter_10",
        output="screen",
        parameters=[
            config,
            {
                "race_start_topic": race_start_topic,
                "speed_topic": speed_topic,
                "steering_topic": steering_topic,
                "actuator_topic": "/team_10/gz_cmd_vel_10",
                "use_sim_time": use_sim_time,
            },
        ],
    )

    image_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="image_bridge_10",
        parameters=[{"config_file": os.path.join(share, "config", "image_bridge.yaml")}], output="screen",
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge_10",
        parameters=[{"config_file": os.path.join(share, "config", "clock_bridge.yaml")}], output="screen",
    )
    camera = Node(
        package="team10_race", executable="start_camera", name="start_camera_10",
        parameters=[{"race_start_topic": race_start_topic, "use_sim_time": use_sim_time}], output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument("race_start_topic", default_value="/race/start/team_10"),
        DeclareLaunchArgument("speed_topic", default_value="/team_10/speed_10"),
        DeclareLaunchArgument("steering_topic", default_value="/team_10/steering_10"),
        DeclareLaunchArgument("lidar_topic", default_value="/team_10/scan_10"),
        DeclareLaunchArgument("opponent_lidar_topic", default_value="/team_10/opponent_scan_10"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("race_start_mode", default_value="camera", choices=["camera"], description="Camera-confirmed five-red lights-out is mandatory"),
        clock_bridge,
        bridge,
        image_bridge,
        camera,
        navigation,
        adapter,
    ])
