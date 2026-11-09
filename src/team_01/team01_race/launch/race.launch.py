"""Race launcher: NO world / spawn / global clock; operator owns these."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import os
import time


def generate_launch_description():
    share=Path(get_package_share_directory('team01_race'))
    # Enable automatically when RTAB-Map is installed; missing dependency
    # must not break the existing low-speed camera/LiDAR controller.
    try:
        get_package_prefix('rtabmap_slam')
        get_package_prefix('rtabmap_odom')
        slam_default='true'
    except Exception:
        slam_default='false'
    names={
        'race_start_mode':'camera',
        'race_start_topic':'/race/start/team01',
        'speed_topic':'/team01/speed',
        'steering_topic':'/team01/steering',
        'lidar_topic':'/team01/front_scan',
        'use_sim_time':'true',
        'enable_slam':slam_default,
    }
    args=[DeclareLaunchArgument(n,default_value=v) for n,v in names.items()]
    # Real ROS->GZ bridge must use ] for direction; prevent feedback loops.
    bridge_args=[
        '/team01/front_depth@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/front_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/road_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
        '/team01/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team01/rear_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/team01/gz_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/team01/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/team01/steering_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/team01/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
    ]
    args += [Node(package='ros_gz_bridge',executable='parameter_bridge',
                  name='team01_bridge',arguments=bridge_args,output='screen')]
    args += [Node(package='team01_race',executable='team01_control',name='team01_control',
                  parameters=[str(share/'config/parameters.yaml'),
                              {n:LaunchConfiguration(n) for n in names}],output='screen')]
    args += [Node(package='team01_race',executable='team01_adapter',name='team01_adapter',
                  parameters=[{n:LaunchConfiguration(n) for n in
                               ('race_start_topic','speed_topic','steering_topic','use_sim_time')}],
                  output='screen')]
    state=Path(os.environ.get('XDG_STATE_HOME',str(Path.home()/'.local/state')))
    database_dir=state/'urrc_team01'/'slam'
    database_dir.mkdir(parents=True,exist_ok=True)
    fresh_db=str(database_dir/('run_'+str(time.time_ns())+'.db'))
    slam_enabled=IfCondition(LaunchConfiguration('enable_slam'))
    args += [Node(package='team01_race',executable='team01_scan_frame',
                  name='team01_scan_frame',output='screen',condition=slam_enabled,
                  parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]),
             Node(package='team01_race',executable='team01_scan_fusion',
                  name='team01_scan_fusion',output='screen',condition=slam_enabled,
                  parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]),
             Node(package='team01_race',executable='team01_rgbd_relay',
                  name='team01_rgbd_relay',output='screen',condition=slam_enabled,
                  parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]),
             Node(package='rtabmap_odom',executable='rgbd_odometry',
                  name='team01_rgbd_odometry',output='screen',condition=slam_enabled,
                  parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),
                               'frame_id':'base_link','odom_frame_id':'team01_odom',
                               'publish_tf':True,'Reg/Force3DoF':'true',
                               'Odom/Strategy':'0','approx_sync':True,
                               'qos':2}],
                  remappings=[('rgb/image','/team01/slam/rgb'),
                              ('depth/image','/team01/slam/depth'),
                              ('rgb/camera_info','/team01/slam/camera_info'),
                              ('odom','/team01/slam_odom')]),
             Node(package='rtabmap_slam',executable='rtabmap',
                  name='team01_rtabmap',output='screen',condition=slam_enabled,
                  parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),
                               'frame_id':'base_link','map_frame_id':'team01_map',
                               'odom_frame_id':'team01_odom',
                               'subscribe_rgb':True,'subscribe_depth':True,
                               'subscribe_scan':True,'subscribe_rgbd':False,
                               'approx_sync':True,'qos':2,
                               'database_path':fresh_db,
                               'Reg/Strategy':'2',
                               'Reg/Force3DoF':'true',
                               'RGBD/NeighborLinkRefining':'true',
                               'Grid/FromDepth':'true',
                               'Grid/CellSize':'0.05',
                               'RGBD/LinearUpdate':'0.08',
                               'RGBD/AngularUpdate':'0.04'}],
                  remappings=[('rgb/image','/team01/slam/rgb'),
                              ('depth/image','/team01/slam/depth'),
                              ('rgb/camera_info','/team01/slam/camera_info'),
                              ('scan','/team01/slam/scan'),
                              ('odom','/team01/slam_odom')])]
    return LaunchDescription(args)
