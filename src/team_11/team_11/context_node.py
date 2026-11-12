"""ROS2 pre-race configuration diagnostic, distinct from driving controller.

Emits metadata only; does not subscribe to world pose/timing or other teams.
No vehicle commands are produced. In formal race runner, when grid number is
not provided, stage6 rejects ambiguous automatic inference.
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from .race_context import configure

class ContextNode(Node):
    def __init__(self):
        super().__init__("team11_context_node")
        self.declare_parameter("track","")
        self.declare_parameter("grid",0)
        track=self.get_parameter("track").value
        grid=int(self.get_parameter("grid").value)
        self.context=configure(track,grid)
        self.context_publisher=self.create_publisher(String,"/race_context_11",10)
        self.lane_publisher=self.create_publisher(Int32,"/initial_lane_11",10)
        self.create_timer(1.0,self.publish_context)
        self.publish_context()
    def publish_context(self):
        self.context_publisher.publish(String(data=json.dumps({
            "track":self.context.track,
            "grid":self.context.grid,
            "initial_lane":self.context.initial_lane,
            "coordinate_data_for_driving":False,
        },sort_keys=True)))
        self.lane_publisher.publish(Int32(data=self.context.initial_lane))

def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=ContextNode()
        rclpy.spin(node)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
if __name__=="__main__":main()
