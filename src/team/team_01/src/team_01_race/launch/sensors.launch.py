from pathlib import Path
import re
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def transport_partition(context):
    # Keep the operator's environment unless an explicit connection is selected.
    selected=LaunchConfiguration('gz_partition').perform(context)
    return [SetEnvironmentVariable('GZ_PARTITION',selected)] if selected else []


def clock_bridge(context):
    if LaunchConfiguration('bridge_clock').perform(context).lower() not in ('true','1'):
        return []
    world=LaunchConfiguration('world_name').perform(context)
    if not world:
        result=subprocess.run(['gz','service','-l'],env=context.environment,
                              capture_output=True,text=True,timeout=8)
        worlds=sorted(set(re.findall(r'^/world/([^/]+)/scene/info$',result.stdout,re.MULTILINE)))
        if result.returncode or len(worlds)!=1:
            partition=context.environment.get('GZ_PARTITION') or '(Gazebo 기본값)'
            raise RuntimeError('Gazebo 월드를 하나로 선택할 수 없습니다: '+str(worlds)+
                '; GZ_PARTITION='+partition+
                '. 맵 실행기의 --partition과 센서의 gz_partition 값을 맞추세요. 여러 월드가 있으면 world_name도 지정하세요.')
        world=worlds[0]
    source='/world/'+world+'/clock'
    target=LaunchConfiguration('clock_topic').perform(context)
    return [Node(package='ros_gz_bridge',executable='parameter_bridge',name='team_01_clock_bridge',
                 arguments=[source+'@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                 remappings=[(source,target),('/clock',target)],output={'both':'log'})]


def generate_launch_description():
    share = Path(get_package_share_directory('team_01_race'))
    sim = LaunchConfiguration('use_sim_time')
    clock = LaunchConfiguration('clock_topic')
    return LaunchDescription([
        DeclareLaunchArgument('gz_partition',default_value=EnvironmentVariable('GZ_PARTITION',default_value='')),
        DeclareLaunchArgument('clock_topic', default_value='/team_01/clock'),
        DeclareLaunchArgument('world_name', default_value=''),
        DeclareLaunchArgument('camera_mount_config',default_value=str(share/'config/camera_mount.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('bridge_clock', default_value='true'),
        DeclareLaunchArgument('imu_topic', default_value='/team_01/imu'),
        DeclareLaunchArgument('lidar_topic', default_value='/team_01/scan'),
        OpaqueFunction(function=transport_partition),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='team_01_sensor_bridge',
             output={'both':'log'}, parameters=[{'config_file': str(share/'config/sensors_bridge.yaml'), 'use_sim_time': sim}],
             remappings=[('/clock',clock),('/team_01/scan', LaunchConfiguration('lidar_topic')), ('/team_01/imu', LaunchConfiguration('imu_topic'))]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='team_01_actuator_bridge',
             output={'both':'log'}, parameters=[{'config_file': str(share/'config/bridge.yaml'), 'use_sim_time': sim}],remappings=[('/clock',clock)]),
        OpaqueFunction(function=clock_bridge),
        Node(package='team_01_race', executable='sensor_tf_node', name='team_01_sensor_tf',
             parameters=[{'use_sim_time': sim,'camera_mount_config':LaunchConfiguration('camera_mount_config')}],
             remappings=[('/clock',clock)],output={'both':'log'}),
    ])
