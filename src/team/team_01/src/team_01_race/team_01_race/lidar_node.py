from dataclasses import fields
from .camera_bundle import decode_bundle,decode_lidar_frame,encode_lidar_observation
import json
import math
import time
import numpy as np
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from diagnostic_msgs.msg import DiagnosticArray
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from std_msgs.msg import Bool, String
from .ros_support import SENSOR_QOS, ConfigNode, path_array, stamp_seconds
from .navigation import ObstacleTracker
from .corner_recovery import RecoveryConfig,RecoveryReceiver
from .lidar_filter import CameraWallFilter, WallFilterConfig, scan_points


class LidarObstacleNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_lidar_obstacle')
        primary=self.p('primary_steering_source','lidar')
        if primary not in ('lidar','auto'):raise ValueError('primary_steering_source must be lidar or auto')
        self.primary_lidar=primary=='lidar'
        g = self.geometry()
        config = WallFilterConfig()
        for field in fields(config):
            setattr(config, field.name, self.p('lidar_obstacle_filter.'+field.name, getattr(config,field.name)))
        self.filter = CameraWallFilter(g['vehicle_width_m'],
                                      self.p('obstacle.vehicle_lateral_margin_m', .15), config)
        rc=RecoveryConfig()
        for field in fields(rc):setattr(rc,field.name,self.p('corner_path_recovery.'+field.name,getattr(rc,field.name)))
        rc.corner_path_hold_speed_mps=min(rc.corner_path_hold_speed_mps,self.p('path_manager.previous_path_max_speed_mps',1.2))
        self.recovery_receiver=RecoveryReceiver(rc,config.camera_timeout_sec)
        self.create_subscription(Bool,'/team_01/debug/camera_ready',lambda m:self.recovery_receiver.health(m.data,self.now_seconds()),1)
        self.create_subscription(String,'/team_01/internal/corner_path_context',self.on_recovery_context,10)
        self.horizon = self.p('obstacle.detection_distance_m', 5.0)
        self.min_beams = self.p('perception.min_valid_scan_beams', 20)
        self.options = dict(cluster_gap=self.p('perception.cluster_gap_m', .14),
                            min_width=self.p('obstacle.min_vehicle_width_m', .12),
                            max_width=self.p('obstacle.max_vehicle_width_m', .80),
                            min_depth=self.p('obstacle.min_vehicle_depth_m', .10),
                            max_depth=self.p('obstacle.max_vehicle_depth_m', 1.20))
        self.tracker = ObstacleTracker(self.p('obstacle.confirm_frames', 3),
                                       self.p('obstacle.match_distance_m', .8),
                                       self.p('obstacle.track_timeout_sec', .3),
                                       release_frames=self.p('obstacle.release_frames', 3))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera_packets = {}
        self.last_camera_stamp = self.last_scan_stamp = -math.inf
        self.pub = self.create_publisher(Detection3DArray, '/team_01/perception/obstacles', 1)
        self.sync_pub=self.create_publisher(String,'/team_01/lidar/synchronized_observation',1)
        self.current_frame=None
        self.scan_ready_pub=self.create_publisher(Bool,'/team_01/debug/lidar_scan_ready',10)
        self.valid_pub = self.create_publisher(Bool, '/team_01/debug/lidar_valid', 10)
        self.debug_pub = self.create_publisher(String, '/team_01/debug/lidar_classification', 10)
        self.last_log = -math.inf
        self.wall_labels={}
        self.lidar_corridors={};self.pending_scans={};self.last_observation_stamp=-math.inf
        if self.primary_lidar:
            self.create_subscription(String,'/team_01/lidar/perception_frame',self.on_lidar_frame,1)
        else:
            self.create_subscription(String,'/team_01/lidar/geometry_bundle',self.on_lidar_corridor,10)
            self.create_subscription(String,'/team_01/lidar/wall_classification',self.on_wall_labels,10)
        self.last_debug_state = None;self.geometry_source='CAMERA_PATH'
        if not self.primary_lidar and self.p('camera_geometry.prefer_bundle',False):
            self.create_subscription(String,self.p('camera_geometry.bundle_topic','/team_01/camera/geometry_bundle'),self.on_camera_bundle,1)
        elif not self.primary_lidar:
            for name in ('center_path','left_boundary','right_boundary'):
                self.create_subscription(Path, '/team_01/camera/'+name, lambda m,name=name:self.part(name,m), 1)
            self.create_subscription(DiagnosticArray, '/team_01/camera/path_status', lambda m:self.part('status',m), 1)
        self.create_subscription(Path, '/team_01/selected_path', self.on_selected, 1)
        if not self.primary_lidar:
            self.create_subscription(LaserScan, self.p('lidar_topic', '/team_01/scan'), self.on_scan, SENSOR_QOS)
        self.get_logger().info(f"Camera-wall LiDAR filter: vehicle_width={g['vehicle_width_m']:.4f} "
            f"wall_margin={config.wall_reject_margin_m:.3f} path_half_width={self.filter.path_half_width:.4f} "
            f"confirm={self.tracker.confirm_frames} release={self.tracker.release_frames}")

    def on_camera_bundle(self,msg):
        try:
            if self.primary_lidar and json.loads(msg.data).get('path_source') not in ('LIDAR_PATH','NO_PATH'):return
            parts=decode_bundle(msg.data,self.get_clock().now().nanoseconds*1e-9,self.filter.c.camera_timeout_sec)
            for name,part in parts.items():self.part(name,part)
        except (ValueError,TypeError,KeyError,IndexError,OverflowError):return

    def on_lidar_frame(self,msg):
        """Never join independently queued path/labels with a different scan."""
        try:
            from types import SimpleNamespace
            parts,points,indices,walls,valid=decode_lidar_frame(msg.data,self.get_clock().now().nanoseconds*1e-9)
            header=parts['center_path'].header;stamp=stamp_seconds(header.stamp)
            if stamp<=self.last_scan_stamp:return
            self.last_scan_stamp=stamp
            self.current_frame=msg.data
            self.scan_ready_pub.publish(Bool(data=valid));self.recovery_receiver.lidar_health(valid,self.now_seconds())
            self.lidar_corridors={stamp:parts};self.wall_labels={stamp:walls}
            if not valid:
                self.valid_pub.publish(Bool(data=False))
                self.sync_pub.publish(String(data=encode_lidar_observation(msg.data,[],False)))
                return
            self.classify_scan(SimpleNamespace(header=header),points,indices)
        except (ValueError,TypeError,KeyError,IndexError,OverflowError):
            self.valid_pub.publish(Bool(data=False))
        finally:self.current_frame=None

    def age(self, header):
        return self.get_clock().now().nanoseconds*1e-9-stamp_seconds(header.stamp)

    def part(self, name, msg):
        stamp = stamp_seconds(msg.header.stamp)
        if stamp <= self.last_camera_stamp or not -.05 <= self.age(msg.header) <= self.filter.c.camera_timeout_sec:
            return
        packet = self.camera_packets.setdefault(stamp,{})
        packet[name] = msg
        if len(self.camera_packets) > 8:
            del self.camera_packets[min(self.camera_packets)]
        required = ('center_path','left_boundary','right_boundary','status')
        if not all(k in packet for k in required):
            return
        self.last_camera_stamp = stamp
        if any(packet[k].header.frame_id != 'team_01/base_link' for k in required[:3]):
            self.filter.latest_invalid = True
            return
        status = packet['status'].status
        values = {v.key:v.value for v in status[0].values} if status else {}
        try:
            confidence = float(values.get('confidence','0'))
            observed = tuple(bool(int(values.get(k,'0'))) for k in ('left_observed','right_observed'))
        except ValueError:
            self.filter.latest_invalid = True
            return
        self.geometry_source=values.get('path_source','CAMERA_PATH')
        self.filter.update_camera(*(path_array(packet[k]) for k in required[:3]), confidence, observed,
                                  stamp, self.now_seconds()-max(0.,self.age(msg.header)))

    def on_selected(self, msg):
        if msg.header.frame_id == 'team_01/base_link' and -.05 <= self.age(msg.header) <= self.filter.c.camera_timeout_sec:
            self.filter.update_selected(path_array(msg), stamp_seconds(msg.header.stamp),
                                        self.now_seconds()-max(0.,self.age(msg.header)))

    def transform(self, msg):
        if msg.header.frame_id == 'team_01/base_link':
            return np.eye(3), np.zeros(3)
        tf = self.buffer.lookup_transform('team_01/base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        q = tf.rotation
        values = np.array([q.x,q.y,q.z,q.w], dtype=float)
        norm = np.linalg.norm(values)
        if not np.isfinite(values).all() or norm < .9 or norm > 1.1:
            raise ValueError('invalid LiDAR TF rotation')
        x,y,z,w = values/norm
        rotation = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                             [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                             [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        translation = np.array([tf.translation.x,tf.translation.y,tf.translation.z])
        if not np.isfinite(translation).all():
            raise ValueError('invalid LiDAR TF translation')
        return rotation, translation

    def debug(self, data, now, force=False):
        if now-self.last_log >= 1.0 or force or data['state'] != self.last_debug_state:
            self.debug_pub.publish(String(data=json.dumps(data, separators=(',',':'))))
            counts = ' '.join(f'{key}={data.get(key,0)}' for key in
                ('raw_points','wall_rejected_points','corridor_points','clusters','wall_clusters',
                 'obstacle_candidates','confirmed_obstacles','path_blocking_obstacles'))
            self.get_logger().info(f"[LIDAR_CLASSIFY] {counts} state={data['state']}")
            self.last_log = now
            self.last_debug_state = data['state']

    def on_recovery_context(self,msg):
        try:
            data=json.loads(msg.data)
            if self.primary_lidar and data.get('holding') and data.get('source_sensor')!='LIDAR_PATH':
                self.recovery_receiver.data=None;return
            self.recovery_receiver.update(data,self.get_clock().now().nanoseconds*1e-9,self.now_seconds())
        except (ValueError,TypeError):return

    def on_wall_labels(self,msg):
        try:
            d=json.loads(msg.data);stamp=float(d['stamp']);ids=d['wall_indices']
            if not math.isfinite(stamp) or not all(type(i) is int and 0<=i<100000 for i in ids):return
            self.wall_labels[stamp]=set(ids)
            if len(self.wall_labels)>8:del self.wall_labels[min(self.wall_labels)]
        except (ValueError,KeyError,TypeError):return

    def on_lidar_corridor(self,msg):
        try:
            if self.primary_lidar and json.loads(msg.data).get('path_source') not in ('LIDAR_PATH','NO_PATH'):return
            parts=decode_bundle(msg.data,self.get_clock().now().nanoseconds*1e-9,self.filter.c.camera_timeout_sec)
            status=parts['status'];values={v.key:v.value for v in status.status[0].values}
            if values.get('path_source')!='LIDAR_PATH':return
            stamp=stamp_seconds(status.header.stamp)
            self.lidar_corridors[stamp]=parts
            if len(self.lidar_corridors)>8:del self.lidar_corridors[min(self.lidar_corridors)]
            pending=self.pending_scans.get(stamp)
            if pending is not None:self.classify_scan(*pending)
        except (ValueError,KeyError,TypeError,IndexError):return

    def on_scan(self, msg):
        now, stamp = self.now_seconds(), stamp_seconds(msg.header.stamp)
        if stamp <= self.last_scan_stamp or not -.05 <= self.age(msg.header) <= .25:
            self.valid_pub.publish(Bool(data=False))
            return
        self.last_scan_stamp = stamp
        try:
            rotation, translation = self.transform(msg)
        except (TransformException, ValueError) as error:
            self.valid_pub.publish(Bool(data=False))
            self.debug(dict(state='LIDAR_TF_UNAVAILABLE',error=str(error)),now)
            return
        valid, points, indices = scan_points(list(msg.ranges),msg.angle_min,msg.angle_increment,
                                            msg.range_min,msg.range_max,rotation,translation,self.horizon,self.min_beams)
        self.scan_ready_pub.publish(Bool(data=valid));self.recovery_receiver.lidar_health(valid,now)
        if not valid:
            self.valid_pub.publish(Bool(data=False))
            self.debug(dict(state='INVALID_SCAN'),now)
            return
        self.pending_scans[stamp]=(msg,points,indices)
        if len(self.pending_scans)>8:del self.pending_scans[min(self.pending_scans)]
        self.classify_scan(msg,points,indices)

    def classify_scan(self,msg,points,indices):
        now,stamp=self.now_seconds(),stamp_seconds(msg.header.stamp)
        if stamp<=self.last_observation_stamp or not -.05<=self.age(msg.header)<=.25:return
        parts=self.lidar_corridors.get(stamp)
        if parts is not None:
            values={v.key:v.value for v in parts['status'].status[0].values}
            self.filter.update_lidar_corridor(*(path_array(parts[k]) for k in ('center_path','left_boundary','right_boundary')),
                float(values['confidence']),tuple(bool(int(values[k])) for k in ('left_observed','right_observed')),
                stamp,now-max(0.,self.age(msg.header)))
        self.filter.recovery_context=self.recovery_receiver.wall_context(self.get_clock().now().nanoseconds*1e-9,now)
        self.filter.shared_wall_indices=self.wall_labels.get(stamp,set())
        ready, candidates, debug = self.filter.classify(points,indices,stamp,now,**self.options)
        debug['geometry_source']=self.geometry_source
        if debug['state']=='LIDAR_CORRIDOR':debug['geometry_source']='LIDAR_PATH'
        self.valid_pub.publish(Bool(data=ready))
        if not ready:
            self.debug(debug,now)
            if self.current_frame is not None:
                self.sync_pub.publish(String(data=encode_lidar_observation(self.current_frame,[],False)))
            return  # Never renew clear-space freshness with unusable geometry.
        # Closing velocity must use acquisition intervals, not callback jitter.
        obstacles = self.tracker.update(candidates, stamp if self.primary_lidar else now)
        obstacles = [o for o in obstacles if self.filter.tracked_relevant(o,stamp,now)]
        array = Detection3DArray()
        array.header = msg.header
        array.header.frame_id = 'team_01/base_link'
        for obstacle in obstacles:
            detection = Detection3D()
            detection.header = array.header
            detection.id = obstacle.id
            detection.bbox.center.position.x = obstacle.x
            detection.bbox.center.position.y = obstacle.y
            detection.bbox.center.orientation.w = 1.0
            detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = obstacle.depth, obstacle.width, .15
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = 'path_blocking_obstacle'
            result.hypothesis.score = obstacle.confidence
            result.pose.pose = detection.bbox.center
            detection.results = [result]
            array.detections.append(detection)
        self.pub.publish(array)
        if self.current_frame is not None:
            self.sync_pub.publish(String(data=encode_lidar_observation(self.current_frame,obstacles,True)))
        self.last_observation_stamp=stamp
        self.pending_scans={key:value for key,value in self.pending_scans.items() if key>stamp}
        debug['confirmed_obstacles'] = debug['path_blocking_obstacles'] = len(obstacles)
        debug['tracks'] = [dict(id=o.id,x=o.x,y=o.y,width=o.width,depth=o.depth,length=o.depth,min_x=o.min_x,max_x=o.max_x,min_y=o.min_y,max_y=o.max_y,closest_distance=o.closest_distance,closing_speed=o.closing_speed) for o in obstacles]
        self.debug(debug,now)
