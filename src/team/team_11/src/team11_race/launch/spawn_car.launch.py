from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = Path(get_package_share_directory("team11_race"))
    config = yaml.safe_load((package / "config/spawn.yaml").read_text())

    world = config["world"]
    pose = config["spawn"]

    description = xacro.process_file(
        str(package / "urdf/car.urdf.xacro"),
        mappings={
            "with_lidar": str(config["with_lidar"]).lower(),
            "with_camera": str(config["with_camera"]).lower(),
        },
    ).toxml()

    state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="team_11",
        name="state_publisher",
        parameters=[
            {
                "robot_description": description,
                "use_sim_time": True,
                "frame_prefix": "team_11/",
            }
        ],
        remappings=[
            ("/tf", "/team_11/tf"),
            ("/tf_static", "/team_11/tf_static"),
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            str(world),
            "-name",
            "team11_car",
            "-topic",
            "/team_11/robot_description",
            "-allow_renaming",
            "false",
            "-x",
            str(pose["x"]),
            "-y",
            str(pose["y"]),
            "-z",
            str(pose["z"]),
            "-Y",
            str(pose["yaw"]),
        ],
        output="screen",
    )

    topics = [
        "/team_11/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/team_11/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/team_11/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        "/team_11/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        config["clock_topic"] + "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ]

    if config["with_lidar"]:
        topics.append(
            "/team_11/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        )

    if config["with_camera"]:
        topics.append(
            "/team_11/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"
        )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="team11_bridge",
        arguments=topics,
        remappings=[(config["clock_topic"], "/clock")],
        output="screen",
    )

    adapter = Node(
        package="team11_race",
        executable="vehicle_adapter",
        parameters=[{"require_race_start": False}],
        output="screen",
    )

    return LaunchDescription([state, bridge, spawn, adapter])