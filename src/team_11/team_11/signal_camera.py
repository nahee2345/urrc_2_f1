"""Camera-only red-light start detector; emits Bool on /start_allowed_11.

Official competition mode also requires race/start/<id> true: this stage7
local trial only grants start after its own red-lamps-on then off detection.
"""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from .signal_core import StartGate
from .red_lamps import count_red_lamps

class SignalNode(Node):
    def __init__(self):
        super().__init__("team11_signal_camera")
        self.gate=StartGate()
        self.last_frame=-1.e10
        self.last_log=-1.e10
        self.publisher=self.create_publisher(Bool,"/start_allowed_11",10)
        self.create_subscription(Image,"/camera_11",self.on_frame,10)
        self.create_timer(.1,self.tick)
        self.get_logger().info("WAIT: require 4+ red lamps THEN all-off; initial dark is NOT start")
    def on_frame(self,msg):
        now=time.monotonic()
        self.last_frame=now
        try:
            lamps=count_red_lamps(msg.data,msg.width,msg.height,msg.step,msg.encoding)
            valid=True
        except (ValueError,TypeError) as exc:
            lamps=0
            valid=False
            if now-self.last_log>2.:
                self.get_logger().warning("Invalid camera image: "+str(exc))
        before=self.gate.started
        if self.gate.observe(lamps,now,valid) and not before:
            self.get_logger().info("START: 4+ red lamps observed, now ALL OFF")
        if now-self.last_log>1.:
            self.get_logger().info("camera red_lamps=%d armed=%s started=%s" % (
                lamps,self.gate.armed_at is not None,self.gate.started))
            self.last_log=now
    def tick(self):
        # Even after a start, stop if camera stream dies.
        fresh=time.monotonic()-self.last_frame<=.65
        self.publisher.publish(Bool(data=self.gate.started and fresh))

def main(args=None):
    rclpy.init(args=args)
    n=SignalNode()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
