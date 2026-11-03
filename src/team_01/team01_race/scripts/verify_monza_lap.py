#!/usr/bin/env python3
"""Independent Gazebo odometry QA, never publishes steering or speed."""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from team01_race.lap_validation import LapVerifier,load_reference

class Validator(Node):
    def __init__(self,reference,report_path,max_seconds,motion_start_file=None):
        super().__init__('team01_independent_lap_qa')
        self.verifier=LapVerifier(*load_reference(reference))
        self.report_path=Path(report_path)
        self.max_seconds=max_seconds
        self.motion_start_file=Path(motion_start_file) if motion_start_file else None
        self.since=time.monotonic()
        self.last_at=None
        self.last_log=0.
        self.done=False
        self.create_subscription(Odometry,'/team01/gz_odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(1.,self.heartbeat)
        self.get_logger().info('[LAP QA] Independent reference validation STARTED')

    def on_odom(self,msg):
        if self.done:return
        point=msg.pose.pose.position
        status=self.verifier.observe(point.x,point.y)
        self.last_at=time.monotonic()
        if status not in ('WAITING_FOR_ODOM','TRACKING'):
            self.done=True
            self.write_report()
            self.get_logger().info('[LAP QA] '+status+' '+str(self.report_path))

    def write_report(self):
        self.report_path.parent.mkdir(parents=True,exist_ok=True)
        data=self.verifier.report()
        data['elapsed_wall_s']=round(time.monotonic()-self.since,2)
        tmp=self.report_path.with_suffix('.json.tmp')
        tmp.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        tmp.replace(self.report_path)

    def heartbeat(self):
        if self.done:return
        now=time.monotonic()
        if self.last_at is None and now-self.since>30:
            self.verifier.status='NO_ODOM';self.done=True
        elif self.last_at is not None and now-self.last_at>8:
            self.verifier.status='STALE_ODOM';self.done=True
        elif (self.motion_start_file is not None and self.motion_start_file.is_file()
              and now-self.motion_start_file.stat().st_mtime > 80.0
              and self.verifier.travel_m < 2.0):
            # A high commanded speed with nearly zero Gazebo motion is a
            # drivetrain/initial collision problem, not a corner-planner issue.
            self.verifier.status='DRIVE_STALL';self.done=True
        elif now-self.since>self.max_seconds:
            self.verifier.status='LAP_TIMEOUT';self.done=True
        if self.done:self.write_report();return
        if now-self.last_log>8:
            r=self.verifier.report()
            self.get_logger().info('[LAP QA] '+f"progress={r['progress_m']:.1f}/{r['reference_length_m']:.1f}m, "
                                   f"coverage={r['coverage_bins_of_120']}/120, offroad={r['offroad_frames']}")
            self.last_log=now
            self.write_report()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--reference',required=True)
    p.add_argument('--report',required=True)
    p.add_argument('--max-seconds',type=float,default=1800.)
    p.add_argument('--motion-start-file')
    args=p.parse_args()
    rclpy.init()
    node=Validator(args.reference,args.report,args.max_seconds,args.motion_start_file)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=1.)
    except KeyboardInterrupt:
        node.verifier.status='INTERRUPTED'
        node.write_report()
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
