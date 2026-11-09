"""Time-coherent front/rear LiDAR merger for visual-LiDAR SLAM only."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from team01_race.scan_fusion import fused_ranges


class ScanFusion(Node):
    def __init__(self):
        super().__init__('team01_scan_fusion')
        self.front=None;self.rear=None
        self.last_stamp=None
        self.pub=self.create_publisher(LaserScan,'/team01/slam/scan',
                                       qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/team01/front_scan',self.on_front,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan,'/team01/rear_scan',self.on_rear,
                                 qos_profile_sensor_data)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def on_front(self,m):
        self.front=m;self.emit()

    def on_rear(self,m):
        self.rear=m;self.emit()

    def emit(self):
        if self.front is None or self.rear is None:return
        a=self.stamp(self.front);b=self.stamp(self.rear)
        # At 5m/s a 100ms mismatch would shift obstacles by ~0.5m.
        if a<=0 or b<=0 or abs(a-b)>.025:return
        stamp=max(a,b)
        if self.last_stamp is not None and stamp-self.last_stamp<.045:return
        self.last_stamp=stamp
        def pack(m):
            return dict(ranges=m.ranges,angle_min=m.angle_min,
                        angle_increment=m.angle_increment,
                        range_min=m.range_min,range_max=m.range_max)
        data=fused_ranges(pack(self.front),pack(self.rear))
        msg=LaserScan()
        msg.header.stamp=(self.front if a>=b else self.rear).header.stamp
        msg.header.frame_id='base_link'
        msg.angle_min=-math.pi;msg.angle_max=math.pi
        msg.angle_increment=2.*math.pi/(len(data)-1)
        msg.range_min=.1;msg.range_max=16.
        msg.scan_time=.05
        msg.ranges=data.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node=ScanFusion()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
