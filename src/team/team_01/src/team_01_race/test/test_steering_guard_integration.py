"""Installed Control: Camera severity authority, yaw independence, validity/STOP."""
import os
os.environ['ROS_DOMAIN_ID']='103'
import json,math,time,unittest
from pathlib import Path
import numpy as np
import pytest,launch,launch_ros.actions,launch_testing.actions
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import Header,Bool,Float32,String
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from team_01_race.ros_support import path_message
from team_01_race.corner import SEVERITIES


@pytest.mark.launch_test
def generate_test_description():
    root=Path(__file__).resolve().parents[1]
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='team_01_race',executable='control_adapter_node',name='team_01_control_adapter',
            parameters=[str(root/'config/racing.yaml'),str(root/'config/corner_control.yaml'),str(root/'config/steering_guard.yaml'),
                        {'primary_steering_source':'auto','use_sim_time':False}],output='screen'),launch_testing.actions.ReadyToTest()])


class TestGuardPipeline(unittest.TestCase):
    def test_camera_authority_yaw_independence_jumps_and_forward_stop(self):
        rclpy.init();n=rclpy.create_node('camera_severity_control_test')
        try:
            paths={k:n.create_publisher(PathMsg,'/team_01/camera/'+k,10) for k in ('center_path','left_boundary','right_boundary')}
            selected=n.create_publisher(PathMsg,'/team_01/selected_path',10);meta=n.create_publisher(DiagnosticArray,'/team_01/camera/path_status',10)
            imu=n.create_publisher(Imu,'/team_01/imu',qos_profile_sensor_data)
            enabled=n.create_publisher(Bool,'/team_01/internal/drive_enabled',10);speed=n.create_publisher(Float32,'/team_01/internal/requested_speed',10)
            policy=n.create_publisher(String,'/team_01/internal/corner_policy',10)
            speeds=[];steers=[];statuses=[];raw=[]
            n.create_subscription(Float32,'/team_01/speed',lambda m:speeds.append(m.data),10)
            n.create_subscription(Float32,'/team_01/steering',lambda m:steers.append(m.data),10)
            n.create_subscription(Float32,'/team_01/debug/desired_steering',lambda m:raw.append(m.data),10)
            n.create_subscription(String,'/team_01/debug/steering_guard_status',lambda m:statuses.append(json.loads(m.data)),10)
            x=np.arange(.1,5.01,.1)
            rng=np.random.default_rng(1010)
            def spin(duration,level=0,k=0.,offset=0.,requested=7.,drive=True,bad=False,confidence=1.,publish_policy=True,noise=False):
                end=time.monotonic()+duration
                while time.monotonic()<end:
                    stamp=n.get_clock().now().to_msg();header=Header(stamp=stamp,frame_id='team_01/base_link')
                    yy=.5*k*np.minimum(x,2.)**2+2.*k*np.maximum(0.,x-2.)
                    center=np.column_stack((x,yy));chosen=center+[0.,offset]
                    if noise:chosen[:,1]+=rng.uniform(-.005,.005,len(chosen))
                    if bad:chosen[:,0]=-x
                    for key,data in [('center_path',center),('left_boundary',center+[0.,1.2]),('right_boundary',center-[0.,1.2])]:paths[key].publish(path_message(data,header))
                    d=DiagnosticArray();d.header=header;d.status=[DiagnosticStatus(name='camera_path',values=[KeyValue(key='confidence',value=str(confidence))])];meta.publish(d)
                    if publish_policy:policy.publish(String(data=json.dumps(dict(corner_severity=SEVERITIES[level],camera_source_stamp=stamp.sec+stamp.nanosec*1e-9,camera_geometry_confidence=confidence,frame_id='team_01/base_link'))))
                    selected.publish(path_message(chosen,header));enabled.publish(Bool(data=drive));speed.publish(Float32(data=requested))
                    # Wild yaw/rate must have no effect on primary Control.
                    m=Imu();m.header=Header(stamp=stamp,frame_id='team_01/imu_link');m.orientation.z=1.;m.angular_velocity.z=20.;imu.publish(m)
                    end_step=time.monotonic()+.04
                    while time.monotonic()<end_step:rclpy.spin_once(n,timeout_sec=.001)
            deadline=time.monotonic()+8
            while time.monotonic()<deadline and selected.get_subscription_count()<1:rclpy.spin_once(n,timeout_sec=.02)
            spin(8.2)
            self.assertGreater(speeds[-1],6.9)
            before=len(speeds)
            spin(2.,noise=True)
            self.assertGreater(min(speeds[before:]),6.85)  # No false stop/deceleration for 5 mm sample noise.
            self.assertFalse(any(t=='sensor_msgs/msg/Imu' for _,types in n.get_subscriber_names_and_types_by_node('team_01_control_adapter','/') for t in types))
            spin(.045,offset=.65);spin(.4)
            self.assertTrue(any(s['state']=='PATH_HOLD' for s in statuses))
            # Decelerate before testing steering authority: the retained
            # 3 m/s² physical moving-speed clamp is intentionally still active.
            spin(3.,requested=1.5)
            for level,limit in ((1,14.),(2,20.),(3,25.)):
                for sign in (1.,-1.):
                    start=len(steers);spin(.8,level,k=sign*.8,offset=sign*.7,requested=1.5)
                    self.assertTrue(any(s['state_steering_limit_deg']==limit for s in statuses[-4:]))
                    self.assertLessEqual(max(abs(v) for v in steers[start:]),math.radians(limit)+1e-5)
                    self.assertGreater(abs(steers[-1]),math.radians(min(limit,22.)-1.))
                    self.assertLess(steers[-1]*sign,0.)  # Official RIGHT-positive.
            self.assertGreater(max(map(abs,raw)),math.radians(22.))
            spin(.7,level=3,k=.35,confidence=.2,requested=1.5)
            self.assertLessEqual(abs(steers[-1]),math.radians(8.)+1e-5)
            spin(.3,level=3,k=.35,requested=1.5,bad=True)
            self.assertEqual(speeds[-1],0.)
            spin(.3,drive=False)
            self.assertEqual(speeds[-1],0.)
            self.assertTrue(all(abs(v)<=.5235991 for v in steers))
        finally:n.destroy_node();rclpy.shutdown()
