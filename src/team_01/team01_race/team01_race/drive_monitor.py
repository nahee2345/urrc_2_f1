"""Passive controller-to-adapter-to-odometry health diagnostics."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32


def classify_drive(cmd, adapter, speed, joint, odom_age):
    if cmd is None:return 'NO_CONTROLLER_COMMAND'
    if adapter is None:return 'NO_ADAPTER_DIAGNOSTICS'
    if not adapter.get('send_enabled',False):
        return 'ADAPTER_STOP_'+str(adapter.get('stop_reason','UNKNOWN'))
    if speed is None or odom_age>1.:return 'ODOMETRY_MISSING_OR_STALE'
    if cmd>.4 and adapter.get('sent_linear_mps',0.)>.4 and speed<.04:
        return 'COMMAND_SENT_NO_MEASURED_MOTION'
    if joint is None:return 'STEERING_JOINT_UNKNOWN'
    return 'CHAIN_OBSERVED'


class DriveMonitor(Node):
    def __init__(self):
        super().__init__('team01_drive_monitor')
        self.declare_parameter('use_sim_time',True)
        self.cmd=None;self.adapter=None;self.speed=None
        self.joint=None;self.odom_at=0.
        self.create_subscription(Float32,'/team01/speed',self.on_cmd,10)
        self.create_subscription(String,'/team01/debug/adapter',self.on_adapter,10)
        self.create_subscription(Odometry,'/team01/gz_odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(JointState,'/team01/steering_joint_states',
                                 self.on_joint,qos_profile_sensor_data)
        self.pub=self.create_publisher(String,'/team01/debug/drive_chain',10)
        self.create_timer(.2,self.tick)

    def on_cmd(self,msg):
        self.cmd=float(msg.data) if math.isfinite(float(msg.data)) else None

    def on_adapter(self,msg):
        try:self.adapter=json.loads(msg.data)
        except (ValueError,TypeError):self.adapter=None

    def on_odom(self,msg):
        value=float(msg.twist.twist.linear.x)
        if math.isfinite(value):
            self.speed=abs(value);self.odom_at=time.monotonic()

    def on_joint(self,msg):
        targets=('front_left_steer_joint','front_right_steer_joint')
        names=list(msg.name)
        if not all(t in names for t in targets):return
        try:
            angles=[float(msg.position[names.index(t)]) for t in targets]
            if all(math.isfinite(a) for a in angles):
                self.joint=sum(angles)/2.
        except (ValueError,IndexError,TypeError):pass

    def tick(self):
        age=time.monotonic()-self.odom_at if self.odom_at else float('inf')
        payload={'status':classify_drive(self.cmd,self.adapter,self.speed,self.joint,age),
                 'cmd_speed_mps':self.cmd,'adapter':self.adapter,
                 'odom_speed_mps':self.speed if age<=1. else None,
                 'steering_joint_rad':self.joint,
                 'odom_wall_age_s':round(age,3) if math.isfinite(age) else None,
                 'diagnostic_only':True}
        self.pub.publish(String(data=json.dumps(payload,separators=(',',':'))))


def main():
    rclpy.init()
    node=DriveMonitor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
