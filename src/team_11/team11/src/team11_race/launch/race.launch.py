"""Official organizer-owned world/spawn: launch ONLY TEAM11 participants."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names={
        "race_start_mode":"signal",
        "race_start_topic":"/race/start/team11",
        "speed_topic":"/team11/speed",
        "steering_topic":"/team11/steering",
        "lidar_topic":"/team11/front_scan",
        "use_sim_time":"true",
    }
    launch_args=[DeclareLaunchArgument(k,default_value=v)
                 for k,v in names.items()]
    sensor_bridges=[
        "/team11/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/team11/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image",
        "/team11/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    ]
    launch_args.extend([
        Node(package="ros_gz_bridge",executable="parameter_bridge",
             name="team11_bridge",arguments=sensor_bridges,output="screen"),
        Node(package="team11_race",executable="team11_drive",
             name="team11_drive",output="screen",
             parameters=[{key:LaunchConfiguration(key)
                          for key in names}]),
        Node(package="team11_race",executable="team11_adapter",
             name="team11_adapter",output="screen",
             parameters=[{key:LaunchConfiguration(key)
                          for key in ("race_start_topic","speed_topic",
                                      "steering_topic","use_sim_time")}]),
    ])
    # No Gazebo world launch, vehicle spawn, /clock bridge or race topic writes.
    return LaunchDescription(launch_args)
