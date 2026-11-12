"""Publish controlled manual commands for stage-5 testing only."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--sequence",choices=("straight","left","right","reverse"),default="straight")
    p.add_argument("--seconds",type=float,default=2.)
    args=p.parse_args()
    if not .1<=args.seconds<=4:p.error("seconds must be 0.1..4")
    speed,steer={"straight":(.7,0.),"left":(.5,-math.radians(10)),
        "right":(.5,math.radians(10)),"reverse":(-.3,0.)}[args.sequence]
    rclpy.init()
    n=Node("team11_stage5_manual")
    speedpub=n.create_publisher(Float32,"/speed_11",10)
    steerpub=n.create_publisher(Float32,"/steering_11",10)
    time.sleep(.5)
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            speedpub.publish(Float32(data=float(speed)))
            steerpub.publish(Float32(data=float(steer)))
            time.sleep(.05)
    finally:
        for _ in range(8):
            speedpub.publish(Float32(data=0.))
            steerpub.publish(Float32(data=0.))
            time.sleep(.05)
        n.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":main()
