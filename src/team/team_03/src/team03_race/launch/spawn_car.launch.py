from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package = Path(get_package_share_directory("team03_race"))
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
        namespace="team03",
        name="state_publisher_03",
        parameters=[{
            "robot_description": description,
            "use_sim_time": True,
            "frame_prefix": "team03/",
        }],
        remappings=[
            ("/tf", "/team03/tf_03"),
            ("/tf_static", "/team03/tf_static_03"),
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", str(world),
            "-name", "team03_car",
            "-topic", "/team03/robot_description_03",
            "-allow_renaming", "false",
            "-x", str(pose["x"]),
            "-y", str(pose["y"]),
            "-z", str(pose["z"]),
            "-Y", str(pose["yaw"]),
        ],
        output="screen",
    )

    topics = [
        "/team03/cmd_vel_03@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/team03/odom_03@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/team03/tf_03@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        config["clock_topic"] + "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ]
    if config["with_lidar"]:
        topics.append("/team03/scan_03@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan")
    if config["with_camera"]:
        topics.append("/team03/camera/image_03@sensor_msgs/msg/Image[gz.msgs.Image")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="team03_bridge_03",
        arguments=topics,
        remappings=[(config["clock_topic"], "/clock")],
        output="screen",
    )

    adapter = Node(
        package="team03_race",
        executable="vehicle_adapter.py",
        output="screen",
    )

    return LaunchDescription([state, spawn, bridge, adapter])
    