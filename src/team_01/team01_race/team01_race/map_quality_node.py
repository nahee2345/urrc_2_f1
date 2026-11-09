"""Read RTAB-Map loop closure and RGB-D Odometry uncertainty."""
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from team01_race.map_quality import MapQuality


class MapQualityMixin:
    def init_map_quality(self):
        self.map_quality=MapQuality()
        self.create_subscription(Odometry,'/team01/slam_odom',
                                 self.on_slam_odom,qos_profile_sensor_data)
        try:
            from rtabmap_msgs.msg import Info
            self.create_subscription(Info,'/team01_rtabmap/info',
                                     self.on_rtab_info,10)
        except ImportError:
            self.get_logger().warning('RTAB Info unavailable; map guidance disabled')

    @staticmethod
    def stamp(m):
        return float(m.header.stamp.sec)+float(m.header.stamp.nanosec)*1e-9

    def on_slam_odom(self,msg):
        pos=msg.pose.pose.position
        self.map_quality.odom(self.stamp(msg),msg.pose.covariance,pos.x,pos.y)

    def on_rtab_info(self,msg):
        self.map_quality.info(self.stamp(msg),msg.loop_closure_id,
                              msg.proximity_detection_id)
