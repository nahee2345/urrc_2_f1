from dataclasses import fields, replace
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
from multiprocessing import get_context
import json
import time
import cv2
import numpy as np
from .image_codec import to_bgr, from_bgr
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Path
from std_msgs.msg import String, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection3DArray
from .ros_support import SENSOR_QOS, ConfigNode, path_message, path_array
from .vision import CameraVision, VisionConfig
from .camera_health import CameraHealth
from .camera_bundle import encode_bundle
from rclpy.clock import Clock, ClockType


class CameraNode(ConfigNode):
    def __init__(self):
        super().__init__('team_01_camera_perception')
        cv2.setNumThreads(max(1, self.p('camera.opencv_threads', 2)))
        geometry = self.geometry()
        config = VisionConfig()
        cp = geometry['camera_pose']
        if cp is None:
            raise ValueError('vehicle SDF has no camera_link')
        config.camera_x, config.camera_y = cp[0]-geometry['chassis_pose'][0],cp[1]-geometry['chassis_pose'][1]
        config.camera_roll, config.camera_pitch, config.camera_yaw = cp[3:]
        config.camera_height = geometry['camera_height_m']
        for field in fields(config):
            if field.name in ('camera_x','camera_y','camera_height','camera_roll','camera_pitch','camera_yaw'):continue
            setattr(config, field.name, self.p('vision.'+field.name, getattr(config, field.name)))
        config.enable_single_wall_fallback=self.p('corner_path_recovery.enable_single_wall_fallback',True)
        self.vision = CameraVision(config)
        depth_mode=self.p('depth_corner.enabled',True)
        far_debug=self.p('camera.rgb_far_debug_enabled',False)
        # Depth owns speed. Keep RGB signal/near semantic processing, while
        # the expensive legacy 25m worker is explicitly requested for debug.
        self.preview_enabled=self.p('corner_braking_preview.enabled',True) and (not depth_mode or far_debug)
        self.preview_horizon=self.p('corner_braking_preview.horizon_m',25.)
        self.preview_half_width=self.p('corner_braking_preview.bev_half_width_m',4.5)
        self.preview_roi_top=self.p('corner_braking_preview.road_roi_top',.38)
        self.preview_fps=self.p('corner_braking_preview.process_fps',10.0)
        if not 5<=self.preview_fps<=30:
            raise ValueError('far Camera process_fps must be in [5,30]')
        self.last_preview=-1e9
        if not 5<=self.preview_horizon<=25 or not config.bev_half_width<=self.preview_half_width<=6 or not 0<=self.preview_roi_top<=config.road_roi_top:
            raise ValueError('invalid far Camera ROI')
        self.preview_vision=CameraVision(replace(config,horizon=self.preview_horizon,bev_half_width=self.preview_half_width,road_roi_top=self.preview_roi_top,bev_resolution=max(config.bev_resolution,.05),far_local_boundary_fit=True)) if self.preview_enabled else None
        self.preview_executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix='camera_preview') if self.preview_enabled else None
        self.preview_process=self.p('camera.preview_worker_process',True)
        self.preview_process_pool=ProcessPoolExecutor(max_workers=1,mp_context=get_context('spawn')) if self.preview_enabled and self.preview_process else None
        self.preview_future=None;self.preview_stopping=False
        self.far_bev_pub=self.create_publisher(Image,'/team_01/camera/far_bev_image',SENSOR_QOS)
        self.preview_pub=self.create_publisher(String,'/team_01/camera/braking_preview',10)
        self.bundle_pub=self.create_publisher(String,'/team_01/camera/geometry_bundle',10)
        self.info = None
        self.last_process = self.last_debug = self.last_log = -1e9
        self.max_fps = self.p('camera.process_fps', 20.0)
        self.debug_fps = self.p('camera.debug_fps', 5.0)
        self.calibration_pub=self.create_publisher(String,'/team_01/debug/camera_calibration',1)
        self.lidar_path=None;self.lidar_path_time=-1e9
        self.selected = None
        self.target = None
        self.obstacles = []
        self.mode = 'WAIT_LIGHT'
        self.left_pub = self.create_publisher(Path, '/team_01/camera/left_boundary', 10)
        self.right_pub = self.create_publisher(Path, '/team_01/camera/right_boundary', 10)
        self.path_pub = self.create_publisher(Path, '/team_01/camera/center_path', 10)
        self.meta_pub = self.create_publisher(DiagnosticArray, '/team_01/camera/path_status', 10)
        self.light_pub = self.create_publisher(String, '/team_01/traffic_light_state', 10)
        self.conf_pub = self.create_publisher(Float32, '/team_01/camera/path_confidence', 10)
        self.image_age_pub=self.create_publisher(Float32,'/team_01/debug/camera_image_age',10)
        self.ready_pub = self.create_publisher(Bool, '/team_01/debug/camera_ready', 10)
        self.path_ready_pub = self.create_publisher(Bool, '/team_01/debug/path_ready', 10)
        self.health = CameraHealth()
        self.debug_pub = self.create_publisher(Image, '/team_01/camera/debug_image', SENSOR_QOS)
        self.bev_pub = self.create_publisher(Image, '/team_01/camera/bev_image', SENSOR_QOS)
        self.create_subscription(CameraInfo, self.p('camera_info_topic', '/team_01/camera/camera_info'), self.on_info, SENSOR_QOS)
        self.create_subscription(Image, self.p('camera_topic', '/team_01/camera/image_raw'), self.on_image, SENSOR_QOS)
        self.create_subscription(Path,'/team_01/lidar/center_path',self.on_lidar_path,1)
        self.create_subscription(Path, '/team_01/selected_path', self.on_selected, 1)
        self.create_subscription(PointStamped, '/team_01/debug/lookahead_target', self.on_target, 1)
        self.create_subscription(String, '/team_01/debug/mode', self.on_mode, 1)
        self.create_subscription(Detection3DArray, '/team_01/perception/obstacles', self.on_obstacles, 1)
        # Wall-clock watchdog must still report false when simulation pauses.
        self.create_timer(.1, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_image = -1e9
        self.last_received_image_stamp=None

    def on_info(self, msg):
        if msg.width > 0 and msg.height > 0 and msg.k[0] > 0 and msg.k[4] > 0:
            self.info = msg

    def on_lidar_path(self,msg):
        self.lidar_path=path_array(msg);self.lidar_path_time=self.now_seconds()

    def on_selected(self, msg):
        self.selected = path_array(msg)

    def on_target(self, msg):
        self.target = [msg.point.x, msg.point.y]

    def on_mode(self, msg):
        self.mode = msg.data

    def on_obstacles(self, msg):
        self.obstacles = [(d.bbox.center.position.x, d.bbox.center.position.y) for d in msg.detections]

    def watchdog(self):
        self.image_age_pub.publish(Float32(data=float(max(0.,self.now_seconds()-self.last_image))))
        self.ready_pub.publish(Bool(data=self.health.ready(self.now_seconds())))
        if self.now_seconds()-self.last_process > .25:
            self.path_ready_pub.publish(Bool(data=False))

    def publish_preview(self,image,k,distortion,now,source_stamp):
        started=time.monotonic()
        try:
            if self.preview_process_pool is not None:
                from .camera_preview_worker import compute_preview
                preview,geometry,compute_sec=self.preview_process_pool.submit(compute_preview,image,k,distortion,now,self.preview_vision.c).result()
            else:
                from .far_geometry import analyze_preview
                preview=self.preview_vision.process(image,k,distortion,now)
                pixels=None
                if len(preview.get('left_samples',[]))>=5 and len(preview.get('right_samples',[]))>=5:
                    pixels=tuple(self.preview_vision.project(samples,k) for samples in (preview['left_samples'],preview['right_samples']))
                geometry=analyze_preview(preview['path'],preview.get('left_samples'),preview.get('right_samples'),preview['confidence'],source_pixels=pixels)
                compute_sec=time.monotonic()-started
            path=preview['path']
            if not self.preview_stopping:
                self.preview_pub.publish(String(data=json.dumps(dict(stamp=source_stamp,
                    frame_id='team_01/base_link',processing_wall_sec=time.monotonic()-started,compute_wall_sec=compute_sec,source_publish_age=self.get_clock().now().nanoseconds*1e-9-source_stamp,confidence=preview['confidence'],path=path.tolist() if path is not None else [],
                    left=preview['left'].tolist() if preview.get('left') is not None else [],
                    right=preview['right'].tolist() if preview.get('right') is not None else [],
                    left_samples=preview.get('left_samples',[]),right_samples=preview.get('right_samples',[]),
                    near_path=self.vision.previous_path.tolist() if self.vision.previous_path is not None else [],
                    geometry=geometry))))
                if preview.get('bev') is not None:self.far_bev_pub.publish(from_bgr(preview['bev']))
        except (ValueError,RuntimeError,cv2.error) as error:
            self.get_logger().warning('far Camera worker rejected frame: '+str(error),throttle_duration_sec=1.)

    def destroy_node(self):
        self.preview_stopping=True
        if self.preview_executor is not None:self.preview_executor.shutdown(wait=True,cancel_futures=True)
        if self.preview_process_pool is not None:self.preview_process_pool.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()

    def on_image(self, msg):
        wall_started=time.monotonic()
        now = self.now_seconds()
        image_stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        age=self.get_clock().now().nanoseconds*1e-9-image_stamp*1e-9
        if (self.last_received_image_stamp is not None and image_stamp<=self.last_received_image_stamp) or not -.05<=age<=.25:
            return
        self.last_received_image_stamp=image_stamp
        self.last_image = now
        if now-self.last_process < .95/max(self.max_fps, 1.0):
            return
        self.last_process = now
        try:
            image = to_bgr(msg)
            stamp = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            self.health.accept(now, stamp)
            if self.info is None:
                self.path_ready_pub.publish(Bool(data=False))
                return
            k = np.asarray(self.info.k, dtype=float).reshape(3, 3).copy()
            k[0] *= image.shape[1]/self.info.width
            k[1] *= image.shape[0]/self.info.height
            if (self.preview_vision is not None and now-self.last_preview>=.95/self.preview_fps and
                    (self.preview_future is None or self.preview_future.done())):
                self.last_preview=now
                # Start from this observation immediately; do not add near BEV
                # latency to the far worker's source age. Keep one in flight.
                self.preview_future=self.preview_executor.submit(self.publish_preview,image,k,np.asarray(self.info.d).copy(),now,
                    msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
            result = self.vision.process(image, k, np.asarray(self.info.d), now)
        except (ValueError, RuntimeError, cv2.error) as error:
            self.get_logger().error(f'camera frame rejected: {error}', throttle_duration_sec=1.0)
            self.health.invalidate()
            self.ready_pub.publish(Bool(data=False))
            self.path_ready_pub.publish(Bool(data=False))
            return
        header = msg.header
        try:self.bundle_pub.publish(String(data=encode_bundle(result,header)))
        except (ValueError,TypeError):
            invalid=dict(result,path=None,left=None,right=None,confidence=0.,reason='invalid_camera_geometry')
            self.bundle_pub.publish(String(data=encode_bundle(invalid,header)))
        self.left_pub.publish(path_message(result['left'], header))
        self.right_pub.publish(path_message(result['right'], header))
        self.light_pub.publish(String(data=result['light']))
        light=self.vision.light
        self.get_logger().info('[TRAFFIC_LIGHT] '+json.dumps(dict(state=light.state,pending=light.pending,confirm_count=light.count,red_row_count=light.full_red_count,armed_lamps=len(light.armed_boxes),released=light.released,reason=light.reason)),throttle_duration_sec=1.)
        self.conf_pub.publish(Float32(data=float(result['confidence'])))
        meta = DiagnosticArray()
        meta.header = header
        status = DiagnosticStatus(name='camera_path', level=DiagnosticStatus.OK if result['path'] is not None else DiagnosticStatus.WARN,
                                  message=result['light'])
        status.values = [KeyValue(key='confidence', value=str(result['confidence'])),
                         KeyValue(key='left_observed', value=str(int(result['observed'][0]))),
                         KeyValue(key='right_observed', value=str(int(result['observed'][1]))),
                         KeyValue(key='traffic_light', value=result['light']),
                         KeyValue(key='path_reason', value=result['reason'])]
        if result.get('track_width') is not None:
            status.values.extend([KeyValue(key='track_width',value=str(result['track_width'])),KeyValue(key='track_width_age',value=str(result['track_width_age']))])
        meta.status = [status]
        self.meta_pub.publish(meta)
        self.path_pub.publish(path_message(result['path'], header))
        self.path_ready_pub.publish(Bool(data=result['path'] is not None))
        if (now-self.last_debug >= 1.0/max(1.0, self.debug_fps) and
                (self.debug_pub.get_subscription_count() or self.bev_pub.get_subscription_count())):
            debug = result['image']
            # Project the vehicle centreline independently of the optical axis.
            for points,color in [(np.column_stack((np.linspace(.5,12,60),np.zeros(60))),(255,255,0)),(self.lidar_path if now-self.lidar_path_time<=.25 else None,(255,0,0))]:
                if points is not None and len(points):
                    uv=self.vision.project(points,k)
                    uv=uv[np.isfinite(uv).all(axis=1)]
                    if len(uv):cv2.polylines(debug,[np.clip(uv,-10000,10000).astype(np.int32)],False,color,2)
            cv2.drawMarker(debug,(int(k[0,2]),int(k[1,2])),(0,255,255),cv2.MARKER_CROSS,16,2)
            cv2.putText(debug,'vehicle:cyan optical:yellow lidar:blue selected:magenta',(8,65),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
            if self.selected is not None and len(self.selected):
                visible = self.selected[self.selected[:,0] > self.vision.c.camera_x+.05]
                pixels = self.vision.project(visible, k)
                if np.isfinite(pixels).all():
                    cv2.polylines(debug, [np.clip(pixels, -10000, 10000).astype(np.int32)], False, (255, 0, 255), 2)
            for point, color in [(self.target, (0,255,255)), *[(p, (0,0,255)) for p in self.obstacles]]:
                if point is not None and point[0] > self.vision.c.camera_x:
                    uv = self.vision.project(np.asarray([point]), k)[0]
                    if np.isfinite(uv).all() and np.max(np.abs(uv)) < 10000:
                        cv2.circle(debug, tuple(uv.astype(int)), 5, color, 2)
            cv2.putText(debug, self.mode, (10, 45), cv2.FONT_HERSHEY_SIMPLEX, .55, (255,255,0), 2)
            for publisher, pixels in [(self.debug_pub, debug), (self.bev_pub, result['bev'])]:
                output = from_bgr(pixels)
                output.header = msg.header
                publisher.publish(output)
            self.last_debug = now
        if now-self.last_log > 1.0:
            c=self.vision.c;w=int(round(2*c.bev_half_width/c.bev_resolution));h=int(round((c.horizon-c.bev_near)/c.bev_resolution))
            ground=np.array([[c.bev_near,c.bev_half_width],[c.bev_near,-c.bev_half_width],[c.horizon,-c.bev_half_width],[c.horizon,c.bev_half_width]])
            calibration=dict(frame='team_01/base_link',optical_frame='team_01/camera_optical_frame',camera_ground_height_m=c.camera_height,
                camera_x_m=c.camera_x,camera_y_m=c.camera_y,roll_deg=float(np.degrees(c.camera_roll)),pitch_deg=float(np.degrees(c.camera_pitch)),yaw_deg=float(np.degrees(c.camera_yaw)),
                intrinsics=k.tolist(),distortion=list(self.info.d),image_size=[image.shape[1],image.shape[0]],bev_source_pixels=self.vision.project(ground,k).tolist(),bev_ground_points=ground.tolist(),
                bev_destination_pixels=[[0,h-1],[w-1,h-1],[w-1,0],[0,0]],bev_metres_per_pixel=[2*c.bev_half_width/(w-1),(c.horizon-c.bev_near)/(h-1)],vehicle_origin_bev=[(w-1)/2,c.horizon/(c.horizon-c.bev_near)*(h-1)])
            self.calibration_pub.publish(String(data=json.dumps(calibration)))
            self.get_logger().info(f"[PERCEPTION] source_age={self.get_clock().now().nanoseconds*1e-9-(header.stamp.sec+header.stamp.nanosec*1e-9):.4f} processing_sec={time.monotonic()-wall_started:.4f} LIGHT={result['light']} LEFT={result['observed'][0]} RIGHT={result['observed'][1]} PATH_CONF={result['confidence']:.2f} reason={result['reason']}")
            self.last_log = now
