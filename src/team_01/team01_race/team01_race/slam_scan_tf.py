"""Attach the actual bridged Gazebo LiDAR frame to the vehicle TF tree."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class ScanFrameBridge(Node):
    def __init__(self):
        super().__init__('team01_scan_frame')
        self.sent=set()
        self.tf=StaticTransformBroadcaster(self)
        self.create_subscription(LaserScan,'/team01/front_scan',self.on_scan,
                                 qos_profile_sensor_data)

    def on_scan(self,msg):
        frame=str(msg.header.frame_id).lstrip('/')
        if not frame or frame=='base_link' or frame in self.sent:return
        t=TransformStamped()
        t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id='base_link'
        t.child_frame_id=frame
        t.transform.translation.x=.187
        t.transform.translation.z=.109
        t.transform.rotation.w=1.
        self.tf.sendTransform(t)
        self.sent.add(frame)
        self.get_logger().info('Attached Gazebo LiDAR frame: '+frame)


def main():
    rclpy.init()
    n=ScanFrameBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
