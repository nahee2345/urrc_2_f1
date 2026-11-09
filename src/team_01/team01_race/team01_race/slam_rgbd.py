"""Synchronized calibrated RGB-D image relay for visual odometry and SLAM."""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from team01_race.slam_alignment import wide_rgb_to_depth, camera_focal, paired_stamp


class RGBDRelay(Node):
    def __init__(self):
        super().__init__('team01_rgbd_relay')
        self.rgb=None
        self.depth=None
        self.last_pair=None
        self.frame='team01_optical'
        self.pub_rgb=self.create_publisher(Image,'/team01/slam/rgb',qos_profile_sensor_data)
        self.pub_depth=self.create_publisher(Image,'/team01/slam/depth',qos_profile_sensor_data)
        self.pub_info=self.create_publisher(CameraInfo,'/team01/slam/camera_info',10)
        self.create_subscription(Image,'/team01/road_rgb',self.on_rgb,qos_profile_sensor_data)
        self.create_subscription(Image,'/team01/front_depth',self.on_depth,qos_profile_sensor_data)
        self.tf=StaticTransformBroadcaster(self)
        self._static_transforms()

    def _static_transforms(self):
        def make(parent,child,x,y,z,roll,pitch,yaw):
            t=TransformStamped()
            t.header.stamp=self.get_clock().now().to_msg()
            t.header.frame_id=parent
            t.child_frame_id=child
            t.transform.translation.x=x;t.transform.translation.y=y;t.transform.translation.z=z
            cr,sr=math.cos(roll/2),math.sin(roll/2)
            cp,sp=math.cos(pitch/2),math.sin(pitch/2)
            cy,sy=math.cos(yaw/2),math.sin(yaw/2)
            t.transform.rotation.w=cr*cp*cy+sr*sp*sy
            t.transform.rotation.x=sr*cp*cy-cr*sp*sy
            t.transform.rotation.y=cr*sp*cy+sr*cp*sy
            t.transform.rotation.z=cr*cp*sy-sr*sp*cy
            return t
        self.tf.sendTransform([
            make('base_link','team01_camera_link',.214,0.,.112,0.,.191986,0.),
            make('team01_camera_link',self.frame,0.,0.,0.,-math.pi/2,0.,-math.pi/2)])

    @staticmethod
    def timestamp(m):
        return float(m.header.stamp.sec)+1e-9*float(m.header.stamp.nanosec)

    def on_rgb(self,m):
        self.rgb=m;self.publish_pair()

    def on_depth(self,m):
        self.depth=m;self.publish_pair()

    def publish_pair(self):
        if self.rgb is None or self.depth is None:return
        r,d=self.rgb,self.depth
        if not r.width or not d.width or not d.height:return
        if not paired_stamp(self.timestamp(r),self.timestamp(d)):return
        key=(r.header.stamp.sec,r.header.stamp.nanosec,
             d.header.stamp.sec,d.header.stamp.nanosec)
        if key==self.last_pair:return
        if r.encoding.lower() not in ('rgb8','bgr8') or d.encoding.lower()!='32fc1':
            self.get_logger().warn('RGB-D unsupported encodings: '+r.encoding+' '+d.encoding)
            return
        try:
            img=np.ndarray((r.height,r.width,3),dtype=np.uint8,
                           buffer=r.data,strides=(r.step,3,1))
            aligned=wide_rgb_to_depth(img,d.width,d.height)
        except (ValueError,TypeError) as exc:
            self.get_logger().warn('RGB-D projection rejected: '+str(exc))
            return
        self.last_pair=key
        stamp=r.header.stamp
        out=Image()
        out.header.stamp=stamp;out.header.frame_id=self.frame
        out.width=d.width;out.height=d.height
        out.encoding=r.encoding;out.is_bigendian=r.is_bigendian
        out.step=d.width*3;out.data=aligned.tobytes()
        dep=Image()
        dep.header.stamp=stamp;dep.header.frame_id=self.frame
        dep.width=d.width;dep.height=d.height
        dep.encoding=d.encoding;dep.is_bigendian=d.is_bigendian
        dep.step=d.step;dep.data=d.data
        info=CameraInfo()
        info.header=out.header
        info.width=d.width;info.height=d.height
        focal=camera_focal(d.width,1.6057)
        cx=(d.width-1.)/2.;cy=(d.height-1.)/2.
        info.distortion_model='plumb_bob';info.d=[0.]*5
        info.k=[focal,0.,cx,0.,focal,cy,0.,0.,1.]
        info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p=[focal,0.,cx,0.,0.,focal,cy,0.,0.,0.,1.,0.]
        self.pub_info.publish(info)
        self.pub_depth.publish(dep)
        self.pub_rgb.publish(out)


def main():
    rclpy.init()
    n=RGBDRelay()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
