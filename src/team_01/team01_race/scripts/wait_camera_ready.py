#!/usr/bin/env python3
"""Persistent ROS subscriber for verified pre-light camera calibration."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CameraReadyProbe(Node):
    def __init__(self):
        super().__init__('team01_wait_camera_ready')
        self.consecutive=0
        self.last=None
        self.samples=0
        self.create_subscription(Bool,'/team01/ready_for_lights',
                                 self.on_ready,10)
        self.create_subscription(String,'/team01/debug/start',
                                 self.on_diag,10)

    def on_ready(self,msg):
        self.samples+=1
        self.consecutive=self.consecutive+1 if msg.data else 0

    def on_diag(self,msg):
        try:self.last=json.loads(msg.data)
        except (ValueError,TypeError):pass


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--timeout',type=float,default=60.)
    parser.add_argument('--out',default='')
    opts=parser.parse_args()
    rclpy.init()
    node=CameraReadyProbe()
    deadline=time.monotonic()+max(1.,opts.timeout)
    status='START_CALIBRATION_FAILED'
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.25)
            if node.consecutive>=3:
                status='CAMERA_START_BASELINE_READY'
                break
        if status=='CAMERA_START_BASELINE_READY':
            diagnosis='READY'
        elif node.samples==0 and node.last is None:
            diagnosis='NO_CONTROLLER_READINESS_OR_DIAGNOSTIC_MESSAGES'
        elif node.samples==0:
            diagnosis='NO_READY_FLAG_MESSAGES'
        else:
            diagnosis='CAMERA_BASELINE_NOT_YET_CONFIRMED'
        summary={'status':status,'diagnosis':diagnosis,
                 'ready_samples':node.samples,
                 'consecutive_ready':node.consecutive,
                 'last_start_status':node.last}
        print(json.dumps(summary,ensure_ascii=False),flush=True)
        if opts.out:
            from pathlib import Path
            p=Path(opts.out)
            p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',
                         encoding='utf-8')
        return 0 if status=='CAMERA_START_BASELINE_READY' else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    raise SystemExit(main())
