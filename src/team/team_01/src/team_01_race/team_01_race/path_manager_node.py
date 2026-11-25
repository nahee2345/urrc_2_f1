import json,math
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields,replace
import numpy as np
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String,Bool
from tf2_ros import Buffer,TransformListener,TransformException
from .ros_support import ConfigNode,SENSOR_QOS,stamp_seconds,path_message
from .camera_bundle import decode_bundle,encode_bundle,encode_lidar_frame
from .lidar_node import LidarObstacleNode
from .lidar_filter import scan_points
from .path_manager import SourceSelector,PathManagerConfig,geometry_valid
from .lidar_track import LidarTrack,LidarTrackConfig
from .source_debug import source_snapshot

class PathManagerNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_path_manager')
        primary=self.p('primary_steering_source','lidar')
        if primary not in ('lidar','auto'):raise ValueError('primary_steering_source must be lidar or auto')
        self.primary_lidar=primary=='lidar'
        self.mode=self.p('path_source_mode','auto')
        if self.mode not in ('auto','camera','lidar'):raise ValueError('invalid path source mode')
        cfg=PathManagerConfig();tc=LidarTrackConfig()
        for f in fields(cfg):setattr(cfg,f.name,self.p('path_manager.'+f.name,getattr(cfg,f.name)))
        for f in fields(tc):setattr(tc,f.name,self.p('lidar_track.'+f.name,getattr(tc,f.name)))
        self.publish_lidar_far=self.p('path_manager.publish_lidar_far_preview',not self.primary_lidar)
        self.far_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='lidar_preview');self.far_future=None;self.far_stopping=False
        self.far_track=LidarTrack(replace(tc,horizon_m=25.,single_wall_enabled=False));self.last_far=-math.inf
        self.far_pub=self.create_publisher(String,'/team_01/lidar/braking_preview',1)
        self.selector=SourceSelector(cfg);self.track=LidarTrack(tc);self.last_debug=-math.inf
        self.wall_labels_pub=self.create_publisher(String,'/team_01/lidar/wall_classification',10)
        self.track_debug=self.create_publisher(String,'/team_01/debug/lidar_track',10)
        self.last_chosen_path=None
        self.camera=self.lidar=None;self.camera_header=self.lidar_header=None
        self.camera_ready=None
        self.create_subscription(Bool,'/team_01/debug/camera_ready',self.on_camera_ready,1)
        self.camera_blocked=False;self.camera_reacquire=0;self.previous_camera_path=None
        self.last_camera_stamp=-math.inf;self.rejected_frames=0;self.last_rejected_stamp=-math.inf
        self.create_subscription(String,'/team_01/internal/path_acceptance',self.on_acceptance,10)
        self.light_pub=self.create_publisher(String,'/team_01/path_manager/traffic_light',10)
        self.last_output=-math.inf;self.light='UNKNOWN';self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pub=self.create_publisher(String,'/team_01/path_manager/geometry_bundle',10)
        self.source_status_pub=self.create_publisher(String,'/team_01/internal/source_status',10)
        self.debug=self.create_publisher(String,'/team_01/debug/path_manager',10)
        self.lidar_pub=self.create_publisher(Path,'/team_01/lidar/center_path',10)
        self.lidar_geometry_pub=self.create_publisher(String,'/team_01/lidar/geometry_bundle',10)
        self.lidar_frame_pub=self.create_publisher(String,'/team_01/lidar/perception_frame',1)
        self.hold_context=None
        self.create_subscription(String,'/team_01/internal/corner_path_context',self.on_hold,10)
        self.create_subscription(String,'/team_01/camera/geometry_bundle',self.on_camera,1)
        self.create_subscription(LaserScan,self.p('lidar_topic','/team_01/scan'),self.on_scan,SENSOR_QOS)
    def on_hold(self,msg):
        try:self.hold_context=json.loads(msg.data)
        except (ValueError,TypeError):self.hold_context=None

    def on_camera_ready(self,msg):
        self.camera_ready=msg.data
        self.publish_selected()

    def on_acceptance(self,msg):
        try:
            d=json.loads(msg.data)
            if self.mode=='auto' and d['source']=='CAMERA_PATH' and d['accepted'] is False and self.now_ros()-float(d['stamp'])<=.25:
                stamp=float(d['stamp'])
                if stamp>self.last_rejected_stamp:
                    self.rejected_frames+=1;self.last_rejected_stamp=stamp
                if d.get('reason')!='trajectory_jump' or self.rejected_frames>=3:
                    self.camera_blocked=True;self.camera_reacquire=0
            elif d.get('source')=='CAMERA_PATH' and d.get('accepted'):
                self.rejected_frames=0
        except (ValueError,KeyError,TypeError):return

    def now_ros(self):return self.get_clock().now().nanoseconds*1e-9
    def on_camera(self,msg):
        try:
            parts=decode_bundle(msg.data,self.now_ros());d=json.loads(msg.data)
            stamp=stamp_seconds(parts['center_path'].header.stamp)
            if stamp<=self.last_camera_stamp:return
            self.last_camera_stamp=stamp
            self.light_pub.publish(String(data=d['traffic_light']))
            self.camera=dict(path=parts['center_path'].points,left=parts['left_boundary'].points,right=parts['right_boundary'].points,
                confidence=d['confidence'],observed=d['observed'],light=d['traffic_light'],reason=d['reason'],path_source='CAMERA_PATH',track_width=d.get('track_width'),track_width_age=d.get('track_width_age'))
            path=self.camera['path'];previous=self.previous_camera_path
            from .navigation import valid_path
            stable=False
            if valid_path(path) and self.camera['confidence']>=.75 and previous is not None:
                lo=max(path[0,0],previous[0,0]);hi=min(path[-1,0],previous[-1,0]);x=np.arange(lo,hi,.1)
                stable=len(x)>=3 and np.max(abs(np.interp(x,path[:,0],path[:,1])-np.interp(x,previous[:,0],previous[:,1])))<=.2
            self.previous_camera_path=path.copy() if valid_path(path) else None
            if self.camera_blocked:
                self.camera_reacquire=self.camera_reacquire+1 if stable else 0
                if self.camera_reacquire>=3:self.camera_blocked=False
            self.camera_header=parts['center_path'].header;self.light=d['traffic_light'];self.publish_selected()
        except (ValueError,TypeError,KeyError):return
    def on_scan(self,msg):
        if not -.05<=self.now_ros()-stamp_seconds(msg.header.stamp)<=.25:return
        try:
            rot,trans=LidarObstacleNode.transform(self,msg)
            valid,points,indices=scan_points(list(msg.ranges),msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,rot,trans,5.,20)
            now=self.now_seconds()
            self.track.c.enabled=self.get_parameter('lidar_track.enabled').value
            ref=self.camera['path'] if not self.primary_lidar and geometry_valid(self.camera) and self.camera_header is not None and self.now_ros()-stamp_seconds(self.camera_header.stamp)<=.25 else None
            if not self.primary_lidar and self.camera is not None and self.camera.get('track_width') is not None:
                self.track.note_width(float(self.camera['track_width']),float(self.camera.get('track_width_age') or 0)+max(0.,self.now_ros()-stamp_seconds(self.camera_header.stamp)),now)
            capture_time=now-max(0.,self.now_ros()-stamp_seconds(msg.header.stamp)) if self.primary_lidar else now
            self.lidar=self.track.update(points,indices,stamp_seconds(msg.header.stamp),capture_time,ref) if valid else None;self.lidar_header=msg.header
            if self.primary_lidar:
                current=self.lidar if geometry_valid(self.lidar) else dict(path=None,left=None,right=None,
                    confidence=0.,observed=(False,False),reason='no_current_geometry')
                frame=dict(current,light=self.light,path_source='LIDAR_PATH' if geometry_valid(self.lidar) else 'NO_PATH')
                self.lidar_frame_pub.publish(String(data=encode_lidar_frame(frame,msg.header,points,indices,
                    self.track.wall_indices if geometry_valid(self.lidar) else [],valid)))
            if geometry_valid(self.lidar):
                corridor=dict(self.lidar,light=self.light,path_source='LIDAR_PATH')
                self.lidar_geometry_pub.publish(String(data=encode_bundle(corridor,msg.header)))
            self.wall_labels_pub.publish(String(data=json.dumps(dict(stamp=stamp_seconds(msg.header.stamp),wall_indices=self.track.wall_indices if geometry_valid(self.lidar) else [],confidence=self.lidar['confidence'] if self.lidar is not None else 0.))))
            self.track_debug.publish(String(data=json.dumps(self.track.debug)))
            if now-self.last_debug>=.5:
                self.get_logger().info('[LIDAR_TRACK] '+json.dumps(self.track.debug));self.last_debug=now
            self.lidar_pub.publish(path_message(self.lidar['path'] if self.lidar else None,msg.header))
            self.publish_selected()
            if self.publish_lidar_far and now-self.last_far>=.2 and (self.far_future is None or self.far_future.done()):
                self.last_far=now
                far_valid,far_points,far_indices=scan_points(list(msg.ranges),msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,rot,trans,25.,20)
                if far_valid:self.far_future=self.far_executor.submit(self.publish_far,far_points,far_indices,stamp_seconds(msg.header.stamp),now,ref)
        except (ValueError,TransformException):return
    def publish_far(self,points,indices,stamp,now,reference):
        try:
            far=self.far_track.update(points,indices,stamp,now,reference)
            if far is not None and all(far['observed']) and not self.far_stopping:
                from .far_geometry import analyze_preview
                geometry=analyze_preview(far['path'],far['left'],far['right'],far['confidence'],'LIDAR_PREVIEW')
                self.far_pub.publish(String(data=json.dumps(dict(stamp=stamp,frame_id='team_01/base_link',
                    confidence=far['confidence'],path=far['path'].tolist(),left=far['left'].tolist(),right=far['right'].tolist(),geometry=geometry))))
        except (ValueError,RuntimeError):pass

    def destroy_node(self):
        self.far_stopping=True;self.far_executor.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

    def publish_selected(self):
        self.mode=self.get_parameter('path_source_mode').value
        if self.mode not in ('auto','camera','lidar'):return
        def fresh(d,h):return d if h is not None and -.05<=self.now_ros()-stamp_seconds(h.stamp)<=.25 else None
        camera=fresh(self.camera,self.camera_header) if not self.camera_blocked and self.camera_ready is not False else None;lidar=fresh(self.lidar,self.lidar_header)
        keys=dict(camera=stamp_seconds(self.camera_header.stamp) if self.camera_header else None,lidar=stamp_seconds(self.lidar_header.stamp) if self.lidar_header else None)
        if self.primary_lidar:
            chosen=lidar if geometry_valid(lidar) else None
            source='LIDAR_PATH' if chosen is not None else 'NO_PATH'
            self.selector.source=source
            self.selector.reason='lidar_primary' if chosen is not None else 'lidar_unavailable'
        else:
            chosen,source=self.selector.select(camera if self.mode!='lidar' else None,lidar if self.mode!='camera' else None,keys,self.now_ros())
        if source=='FUSED_PATH':header=min((self.camera_header,self.lidar_header),key=lambda h:stamp_seconds(h.stamp))
        elif source=='NO_PATH':header=max((h for h in (self.camera_header,self.lidar_header) if h is not None),key=lambda h:stamp_seconds(h.stamp),default=None)
        else:header=self.lidar_header if source=='LIDAR_PATH' else self.camera_header
        if self.primary_lidar:header=self.lidar_header
        if header is None:return
        stamp=stamp_seconds(header.stamp)
        if stamp<=self.last_output:return
        if chosen is None:chosen=dict(path=None,left=None,right=None,confidence=0.,observed=(False,False),reason='no_current_geometry')
        chosen=dict(chosen,light=self.light,path_source=source)
        self.pub.publish(String(data=encode_bundle(chosen,header)));self.last_output=stamp
        hold=self.hold_context or {}
        previous=bool(hold.get('holding') and 0<=self.now_ros()-float(hold.get('source_stamp',0))<=.5)
        selected_source=('PREVIOUS_LIDAR_PATH' if self.primary_lidar else 'PREVIOUS_PATH') if chosen.get('path') is None and previous else source
        diagnostic=dict(camera_valid=geometry_valid(camera),camera_confidence=camera['confidence'] if camera else 0.,lidar_path_valid=geometry_valid(lidar),lidar_confidence=lidar['confidence'] if lidar else 0.,previous_valid=previous,selected_source=selected_source,switch_reason=self.selector.reason,selected_path_points=len(chosen['path']) if chosen.get('path') is not None else 0)
        self.get_logger().info('[PATH_MANAGER] '+json.dumps(diagnostic),throttle_duration_sec=.5)
        status=source_snapshot(camera,lidar,keys,self.now_ros(),self.selector,previous)
        if chosen.get('path') is not None and self.last_chosen_path is not None:
            current=chosen['path'];old=self.last_chosen_path
            lo=max(current[0,0],old[0,0]);hi=min(current[-1,0],old[-1,0]);x=np.arange(lo,hi,.1)
            if len(x)>=3:status['trajectory_difference']=float(np.max(abs(np.interp(x,current[:,0],current[:,1])-np.interp(x,old[:,0],old[:,1]))))
        if chosen.get('path') is not None:self.last_chosen_path=chosen['path'].copy()
        self.source_status_pub.publish(String(data=json.dumps(status,allow_nan=False)))
        self.debug.publish(String(data=json.dumps(dict(**diagnostic,stamp=stamp,source=source,camera_blocked=self.camera_blocked,camera_reacquire=self.camera_reacquire,confidence=chosen['confidence'],forward_length=float(chosen['path'][-1,0]) if chosen['path'] is not None else 0.))))
