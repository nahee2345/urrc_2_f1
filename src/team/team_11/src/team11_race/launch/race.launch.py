from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    race_start_mode = LaunchConfiguration("race_start_mode")
    race_start = LaunchConfiguration("race_start_topic")
    speed = LaunchConfiguration("speed_topic")
    steering = LaunchConfiguration("steering_topic")
    lidar = LaunchConfiguration("lidar_topic")
    camera = LaunchConfiguration("camera_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "race_start_mode",
            default_value="signal",
        ),
        DeclareLaunchArgument(
            "race_start_topic",
            default_value="/race/start/team_11",
        ),
        DeclareLaunchArgument(
            "speed_topic",
            default_value="/team_11/speed",
        ),
        DeclareLaunchArgument(
            "steering_topic",
            default_value="/team_11/steering",
        ),
        DeclareLaunchArgument(
            "lidar_topic",
            default_value="/team_11/scan",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/team_11/camera/image",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
    ]

    driver = Node(
        package="team11_race",
        executable="race_driver",
        parameters=[
            {
                "race_start_mode": race_start_mode,
                "race_start_topic": "/team_11/internal/race_start",
                "race_start_transient_local": True,
                "speed_topic": speed,
                "steering_topic": steering,
                "lidar_topic": lidar,
                "camera_topic": camera,
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }
        ],
        output="screen",
    )

    adapter = Node(
        package="team11_race",
        executable="vehicle_adapter",
        additional_env={
            "FASTRTPS_DEFAULT_PROFILES_FILE": str(
                Path(get_package_share_directory("team11_race"))
                / "config"
                / "fastdds_adapter.xml"
            ),
        },
        parameters=[
            {
                "race_start_topic": race_start,
                "race_start_relay_topic": "/team_11/internal/race_start",
                "speed_topic": speed,
                "steering_topic": steering,
                "use_sim_time": ParameterValue(
                    use_sim_time,
                    value_type=bool,
                ),
            }
        ],
        output="screen",
    )

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="team11_sensor_bridge",
        arguments=[
            [
                lidar,
                "@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            ],
            [
                camera,
                "@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            "/team_11/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            *arguments,
            sensor_bridge,
            driver,
            adapter,
        ]
    )