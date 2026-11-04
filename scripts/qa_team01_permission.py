#!/usr/bin/env python3
"""Independent QA: publish gate permission until BOTH vehicle nodes confirm."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,String

class PermissionQA(Node):
    def __init__(self):
        super().__init__('team01_qa_permission')
        self.controller=False
        self.adapter=False
        self.controller_samples=0
        self.adapter_samples=0
        self.sent=0
        self.pub=self.create_publisher(Bool,'/race/start/team01',10)
        self.create_subscription(String,'/team01/debug/start',self.on_control,10)
        self.create_subscription(String,'/team01/debug/adapter',self.on_adapter,10)
        self.create_timer(.20,self.send)
    def on_control(self,msg):
        try:
            self.controller=bool(json.loads(msg.data).get('permission'))
            self.controller_samples+=1
        except (ValueError,TypeError):pass
    def on_adapter(self,msg):
        try:
            self.adapter=bool(json.loads(msg.data).get('allowed'))
            self.adapter_samples+=1
        except (ValueError,TypeError):pass
    def send(self):
        if self.pub.get_subscription_count()>=2:
            self.pub.publish(Bool(data=True))
            self.sent+=1

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--timeout',type=float,default=35.)
    p.add_argument('--out',required=True)
    args=p.parse_args()
    rclpy.init()
    node=PermissionQA()
    start=time.monotonic()
    stable=0
    try:
        while rclpy.ok() and time.monotonic()-start<args.timeout:
            rclpy.spin_once(node,timeout_sec=.15)
            stable=(stable+1 if node.controller and node.adapter else 0)
            if stable>=3 and node.sent>=2:break
    finally:
        success=(stable>=3 and node.sent>=2)
        data={'status':'PASS' if success else 'FAIL',
              'controller_permission':node.controller,
              'adapter_allowed':node.adapter,
              'controller_samples':node.controller_samples,
              'adapter_samples':node.adapter_samples,
              'published_permission_count':node.sent,
              'elapsed_s':round(time.monotonic()-start,3)}
        output=Path(args.out)
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',
                          encoding='utf-8')
        print(json.dumps(data,ensure_ascii=False),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if not success:raise SystemExit(2)
if __name__=='__main__':main()
