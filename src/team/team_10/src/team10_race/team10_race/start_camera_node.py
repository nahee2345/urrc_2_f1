"""Leased camera gate: ready is permission to observe, never permission to drive."""
import json
import time
import os
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool,String
from .start_vision import FiveRedDetector

class StartCamera(Node):
    def __init__(self):
        super().__init__('start_camera_10')
        cv2.setNumThreads(1)
        self.declare_parameter('race_start_topic','/race/start/team_10')
        self.ready=False;self.detector=FiveRedDetector();self.last_frame=None;self.last_stamp=None
        self.last_rejection=None
        self.allowed=False;self.reason='NO_CAMERA';self.frames=0;self.last_log='';self.log_time=0
        audit=os.environ.get('HANLA_START_AUDIT_DIR')
        self.audit_dir=Path(audit) if audit else None
        self.audit_reference=None
        self.audit_frame_count=0
        if self.audit_dir:self.audit_dir.mkdir(parents=True,exist_ok=True)
        self.pub=self.create_publisher(Bool,'/team_10/camera_gate_10',10)
        self.status=self.create_publisher(String,'/team_10/start_status_10',10)
        self.create_subscription(Bool,self.get_parameter('race_start_topic').value,self.on_ready,10)
        self.create_subscription(Image,'/team_10/start_camera_image_10',self.on_image,QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE))
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def invalidate(self,reason):
        if self.allowed or self.reason != reason:
            self.get_logger().warning(f'camera_invalidate reason={reason} previous={self.reason} detail={self.last_rejection}')
        # A baseline is comparison data only. An outage always clears all red/off
        # confirmations, so dark reconnect frames can never reopen the gate.
        history = self.detector.history
        preserve = reason != 'NOT_PREPARED' and not self.detector.released
        self.allowed=False;self.reason=reason;self.detector.reset()
        if preserve:self.detector.history=history
        self.pub.publish(Bool(data=False))
    def on_ready(self,msg):
        self.ready=bool(msg.data)
        self.get_logger().info(f'prepare_permission={self.ready}')
        if not self.ready:self.invalidate('NOT_PREPARED')
    def on_image(self,msg):
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        receipt_wall=time.time()
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if age<-.10 or age>.25 or (self.last_stamp is not None and stamp<=self.last_stamp):
            self.last_rejection=dict(incoming_stamp=stamp, sim_age=age, accepted_stamp=self.last_stamp, wall_since_accepted=None if self.last_frame is None else now-self.last_frame)
            self.invalidate('STALE_FRAME');return
        try:
            if msg.encoding not in ('rgb8','bgr8') or msg.step<msg.width*3:raise ValueError('encoding')
            rgb=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            if msg.encoding=='bgr8':rgb=cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
            if np.std(rgb)<4:raise ValueError('blank frame')
        except (ValueError,TypeError):self.invalidate('INVALID_OR_COVERED_FRAME');return
        self.last_stamp=stamp;self.last_frame=now;self.frames+=1
        if self.ready:self.allowed,self.reason=self.detector.update(rgb)
        else:self.reason='NOT_PREPARED'
        processed=time.monotonic()
        if self.audit_dir and self.ready and self.audit_frame_count<1200:
            self.audit_frame_count+=1
            filename=f'frame_{self.frames:05d}.png'
            cv2.imwrite(str(self.audit_dir/filename),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),[cv2.IMWRITE_PNG_COMPRESSION,1])
            if self.detector.reference is not None and self.audit_reference!=id(self.detector.reference):
                self.audit_reference=id(self.detector.reference)
                self.audit_reference_file=f'reference_{self.frames:05d}.png'
                cv2.imwrite(str(self.audit_dir/self.audit_reference_file),cv2.cvtColor(self.detector.reference,cv2.COLOR_RGB2BGR))
            row=dict(receipt_wall=receipt_wall,stamp=stamp,clock_at_receipt=stamp+age,
                processing_s=processed-now,reason=self.reason,allowed=self.allowed,
                red_count=self.detector.red_count,all_red_count=self.detector.all_red_count,
                off_count=self.detector.off_count,
                boxes=None if self.detector.boxes is None else [[int(v) for v in b] for b in self.detector.boxes],
                context=None if self.detector.context is None else [int(v) for v in self.detector.context],
                reference=getattr(self,'audit_reference_file',None),image=filename)
            with (self.audit_dir/'frames.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
    def tick(self):
        now=time.monotonic()
        age=None if self.last_stamp is None else self.get_clock().now().nanoseconds*1e-9-self.last_stamp
        if not self.ready:
            self.allowed=False;self.reason='NOT_PREPARED'
        elif self.last_frame is None or now-self.last_frame>.35:
            self.last_rejection=dict(watchdog=True, sim_age=age, accepted_stamp=self.last_stamp, wall_since_accepted=None if self.last_frame is None else now-self.last_frame)
            self.invalidate('CAMERA_TIMEOUT')
        elif age is not None and (age>.25 or age<-.10):
            self.last_rejection=dict(watchdog=True, sim_age=age, accepted_stamp=self.last_stamp, wall_since_accepted=now-self.last_frame)
            self.invalidate('STALE_FRAME')
        self.pub.publish(Bool(data=bool(self.ready and self.allowed)))
        data=dict(ready=self.ready,allowed=self.allowed,reason=self.reason,frames=self.frames,
                  red_count=self.detector.red_count,stamp=self.last_stamp,sim_age=age,last_rejection=self.last_rejection,
                  wall_age=None if self.last_frame is None else now-self.last_frame)
        self.status.publish(String(data=json.dumps(data)))
        if self.reason!=self.last_log or now-self.log_time>2:
            self.get_logger().info(json.dumps(data));self.last_log=self.reason;self.log_time=now

def main(args=None):
    rclpy.init(args=args);node=StartCamera()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():node.pub.publish(Bool(data=False))
        node.destroy_node();rclpy.try_shutdown()
