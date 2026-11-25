"""Real Gazebo RGB-D Image+CameraInfo to metric wall preview; no steering."""
import json,math,time
import xml.etree.ElementTree as ET
from dataclasses import fields
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from .ros_support import ConfigNode,SENSOR_QOS,stamp_seconds
from .depth_geometry import DepthConfig,depth_array,project_depth,center_geometry,analyze_center,wall_preview
from .corner import CornerConfig

class DepthNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_depth_perception')
        self.c=DepthConfig()
        for f in fields(self.c):setattr(self.c,f.name,self.p('depth_corner.'+f.name,getattr(self.c,f.name)))
        self.c.validate()
        g=self.geometry();self.pose=[g['camera_pose'][i]-g['chassis_pose'][i] for i in range(3)]+g['camera_pose'][3:]
        self.ground_z=-g['chassis_pose'][2]-g['wheel_radius_m']
        share=Path(get_package_share_directory('team_01_race'))
        sensor=ET.parse(self.p('vehicle_model_file',str(share/'models/team_01_car/model.sdf'))).find("model/link[@name='camera_link']/sensor")
        if sensor.get('type')!='rgbd_camera':raise ValueError('real RGB-D sensor required')
        self.near=float(sensor.findtext('camera/clip/near'));self.far=float(sensor.findtext('camera/clip/far'))
        self.corner=CornerConfig()
        for f in fields(self.corner):setattr(self.corner,f.name,self.p('corner_control.'+f.name,getattr(self.corner,f.name)))
        self.info=None;self.last_stamp=-math.inf;self.last=-math.inf
        self.pub=self.create_publisher(String,'/team_01/depth/corner_geometry',1)
        self.create_subscription(CameraInfo,self.p('camera_info_topic','/team_01/camera/camera_info'),self.on_info,SENSOR_QOS)
        self.create_subscription(Image,self.p('depth_topic','/team_01/camera/depth_image'),self.on_depth,SENSOR_QOS)
    def on_info(self,msg):
        # CameraInfo is static calibration, not a synchronized measurement.
        # Delayed/reordered/invalid metadata cannot overwrite a valid newer K.
        if (msg.width<=0 or msg.height<=0 or msg.header.frame_id!='team_01/camera_optical_frame' or
            not all(math.isfinite(v) for v in msg.k) or min(msg.k[0],msg.k[4])<=0):return
        if self.info is not None and stamp_seconds(msg.header.stamp)<=stamp_seconds(self.info.header.stamp):return
        self.info=msg
    def on_depth(self,msg):
        stamp=stamp_seconds(msg.header.stamp);ros_now=self.get_clock().now().nanoseconds*1e-9
        if stamp<=self.last_stamp or not -.05<=ros_now-stamp<=self.c.timeout_sec:return
        self.last_stamp=stamp;self.last=stamp;started=time.monotonic()
        try:
            info=self.info
            if info is None or any(abs(v)>1e-9 for v in info.d) or info.width!=msg.width or info.height!=msg.height or info.header.frame_id!=msg.header.frame_id or msg.header.frame_id!='team_01/camera_optical_frame':raise ValueError('unaligned/missing CameraInfo')
            points=project_depth(depth_array(msg),info.k,self.pose,self.near,self.far,
                z_bounds=(self.ground_z+self.c.min_wall_height_m,self.ground_z+self.c.max_wall_height_m),
                x_bounds=(.5,self.c.horizon_m))
            center,counts=center_geometry(points,self.ground_z,self.c)
            result=analyze_center(center,counts,self.c,self.corner.curvature_enters(),self.corner.heading_enters())
            wall=wall_preview(points,self.ground_z,self.c,self.corner.curvature_enters(),self.corner.heading_enters())
            if wall is not None and (not result.get('depth_corner_candidate') or wall['distance']<result['distance']):result=wall
        except (ValueError,TypeError) as e:result=dict(depth_valid=False,reason=str(e))
        result.update(stamp=stamp,frame_id='team_01/base_link',processing_wall_sec=time.monotonic()-started)
        self.pub.publish(String(data=json.dumps(result,allow_nan=False)))
