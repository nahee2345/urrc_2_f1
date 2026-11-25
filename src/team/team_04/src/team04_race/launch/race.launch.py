from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share = Path(get_package_share_directory('team04_race'))
    defaults = dict(race_start_mode='signal', race_start_topic='/race/start/team04',
                    speed_topic='/team04/speed', steering_topic='/team04/steering',
                    lidar_topic='/team04/scan', use_sim_time='true', driving_mode='manual', autonomy_max_speed='4.5',autonomy_path_mode='racing_line',autonomy_traffic_mode='solo_racing',autonomy_config=str(share/'config/autonomy.yaml'))
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    control = {k: LaunchConfiguration(k) for k in defaults if k not in ('lidar_topic', 'use_sim_time', 'driving_mode', 'autonomy_max_speed', 'autonomy_config', 'autonomy_path_mode', 'autonomy_traffic_mode')}
    nodes = [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge_04',
             parameters=[{'config_file': str(share/'config/bridge.yaml')}, sim]),
        Node(package='team04_race', executable='control_04', name='control_04',
             parameters=[str(share/'config/vehicle.yaml'), sim, control, {'exclusive_command_source': ParameterValue(PythonExpression(["'",LaunchConfiguration('driving_mode'),"' == 'autonomous'"]),value_type=bool)}], output='screen'),
        Node(package='team04_race', executable='scan_04', name='scan_04',
             parameters=[str(share/'config/sensors.yaml'), sim, {'lidar_topic': LaunchConfiguration('lidar_topic')}], output='screen'),
    ]
    for child, xyz, rpy in [('lidar_link',(0,0,.13),(0,0,0)),
                             ('imu_link',(.01,0,.06),(0,0,0)),
                             ('camera_link',(.21,0,.12),(0,0,0)),
                             ('camera_optical_frame',(0,0,0),(-1.57079632679,0,-1.57079632679))]:
        parent = 'team04/camera_link' if child == 'camera_optical_frame' else 'team04/base_link'
        opts = ['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                '--roll',str(rpy[0]),'--pitch',str(rpy[1]),'--yaw',str(rpy[2]),
                '--frame-id',parent,'--child-frame-id','team04/'+child]
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='tf_04_'+child, arguments=opts, parameters=[sim]))
    def autonomous(context):
        mode=LaunchConfiguration('driving_mode').perform(context)
        if mode not in ('manual','autonomous'):
            raise ValueError('driving_mode must be manual or autonomous')
        if mode=='manual':return []
        return [Node(package='team04_race',executable='autonomy_04',name='autonomy_04',output='screen',
                     parameters=[LaunchConfiguration('autonomy_config'),sim,
                                 {k:LaunchConfiguration(k) for k in ('speed_topic','steering_topic','race_start_topic')},
                                 {'path_mode':LaunchConfiguration('autonomy_path_mode'),'traffic_mode':LaunchConfiguration('autonomy_traffic_mode')},
                                 {'max_speed':ParameterValue(LaunchConfiguration('autonomy_max_speed'),value_type=float)}])]
    # Enforce a single speed/steering publisher in autonomous mode as an additive safety gate.
    return LaunchDescription(args+nodes+[OpaqueFunction(function=autonomous)])
