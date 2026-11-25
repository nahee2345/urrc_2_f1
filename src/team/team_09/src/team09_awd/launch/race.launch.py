"""Controller/bridge only; the operator spawns the vehicle and owns the clock."""
import json
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from team09_awd.config import load, SETTINGS, topic, frame
from team09_awd.model_builder import fingerprint
from team09_awd.kinematics import quaternion


def multiply(a,b):
    x,y,z,w=a;X,Y,Z,W=b
    return (w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z)


def build(context):
    share=Path(get_package_share_directory("team09_awd"))
    config=Path(LaunchConfiguration("config_file").perform(context));load(config)
    generated=json.loads((share/"config/generated.json").read_text())
    if generated["config_sha256"]!=fingerprint():
        raise RuntimeError("Configuration and submitted SDF differ. Run configure_team.py, then sync/prepare again.")
    for key,suffix in (("speed_topic","speed"),("steering_topic","steering"),("lidar_topic","lidar/points")):
        given=LaunchConfiguration(key).perform(context)
        if given and given!=topic(suffix):raise RuntimeError(key+" differs from race_entry.json/model config")
    sim=ParameterValue(LaunchConfiguration("use_sim_time"),value_type=bool)
    if LaunchConfiguration("use_sim_time").perform(context).lower()!="true":
        raise RuntimeError("This Gazebo-only package requires use_sim_time:=true")
    common={"use_sim_time":sim,"config_file":str(config)}
    actuator=Node(package="team09_awd",executable="actuator",output="screen",parameters=[common,{
        "race_start_topic":LaunchConfiguration("race_start_topic"),"race_start_mode":LaunchConfiguration("race_start_mode")}])
    racer=Node(package="team09_awd",executable="racer",output="screen",parameters=[common])
    bridge=Node(package="ros_gz_bridge",executable="parameter_bridge",name=SETTINGS["entry_id"]+"_bridge",output="screen",
                parameters=[{"use_sim_time":sim,"config_file":str(share/"config/bridge.yaml")}])
    actions=[bridge,actuator,racer]
    for key,child in (("lidar_pose","lidar"),("camera_pose","camera_optical"),("start_camera_pose","start_camera_optical")):
        pose=SETTINGS["sensors"][key];q=quaternion(*pose[3:])
        if child.endswith("optical"):q=multiply(q,quaternion(-math.pi/2,0,-math.pi/2))
        args=["--x",str(pose[0]),"--y",str(pose[1]),"--z",str(pose[2]),"--qx",str(q[0]),"--qy",str(q[1]),"--qz",str(q[2]),"--qw",str(q[3]),"--frame-id",frame("base_link"),"--child-frame-id",frame(child)]
        actions.append(Node(package="tf2_ros",executable="static_transform_publisher",name=SETTINGS["entry_id"]+"_"+child+"_tf",arguments=args,parameters=[{"use_sim_time":sim}]))
    # Any essential process failure ends the launch; the host must retire it.
    for node in (bridge,actuator,racer):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown(reason="Essential team process exited"))])))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory("team09_awd"))
    cfg=json.loads((share/"config/vehicle.json").read_text())
    return LaunchDescription([
        DeclareLaunchArgument("config_file",default_value=str(share/"config/vehicle.json")),
        DeclareLaunchArgument("use_sim_time",default_value="true"),
        DeclareLaunchArgument("race_start_topic",default_value="/race/start/"+cfg["entry_id"]),
        DeclareLaunchArgument("race_start_mode",default_value="camera"),
        DeclareLaunchArgument("speed_topic",default_value=""),DeclareLaunchArgument("steering_topic",default_value=""),
        DeclareLaunchArgument("lidar_topic",default_value=""),OpaqueFunction(function=build)])
