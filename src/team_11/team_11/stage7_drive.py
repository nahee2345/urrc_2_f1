"""Stage7 automatically sends a 1s forward pulse ONLY after camera gate true."""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

class Driver(Node):
    def __init__(self):
        super().__init__("team11_signal_test_drive")
        self.started_at=None
        self.complete=False
        self.last_allowed=-1.e10
        self.speed=self.create_publisher(Float32,"/speed_11",10)
        self.steer=self.create_publisher(Float32,"/steering_11",10)
        self.create_subscription(Bool,"/start_allowed_11",self.on_gate,10)
        self.create_timer(.05,self.tick)
        self.get_logger().info("WAIT_SIGNAL: No forward command until camera detected lights-out")
    def on_gate(self,msg):
        if msg.data:
            self.last_allowed=time.monotonic()
            if self.started_at is None and not self.complete:
                self.started_at=self.last_allowed
                self.get_logger().info("START: Driving forward for 1 second")
        else:
            self.last_allowed=-1.e10
    def tick(self):
        now=time.monotonic()
        good=now-self.last_allowed <.25
        running=(not self.complete and self.started_at is not None and
                 now-self.started_at<=1 and good)
        if self.started_at is not None and now-self.started_at>1 and not self.complete:
            self.complete=True
            self.get_logger().info("FINISHED: One-second pulse complete; speed held at zero")
        if self.started_at is not None and not good and not self.complete:
            self.complete=True
            self.get_logger().warning("STOP: Camera start permission lost")
        self.speed.publish(Float32(data=.65 if running else 0.0))
        self.steer.publish(Float32(data=0.0))

def main(args=None):
    rclpy.init(args=args)
    n=Driver()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
