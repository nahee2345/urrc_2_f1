"""Independent ready AND leased camera permission at both driving stages."""
import time
from rclpy.clock import Clock, ClockType
from std_msgs.msg import Bool

class StartInterlock:
    def __init__(self, node, ready_topic, callback, group):
        self.ready = False
        self.camera = False
        self.received = None
        self.active = False
        self.callback = callback
        self.node = node
        node.create_subscription(Bool, ready_topic, self.on_ready, 10, callback_group=group)
        node.create_subscription(Bool, '/team_10/camera_gate_10', self.on_camera, 10, callback_group=group)
        # Wall timer: a paused /clock cannot keep old permission alive.
        node.create_timer(.025, self.check, clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=group)
    def on_ready(self, msg):
        self.ready = bool(msg.data)
        self.node.get_logger().info(f'prepare_permission={self.ready}')
        if not self.ready:
            self.camera = False
            self.received = None
        self.check()
    def on_camera(self, msg):
        self.camera = bool(msg.data)
        self.received = time.monotonic()
        self.check()
    def check(self):
        active = (self.ready and self.camera and self.received is not None
                  and time.monotonic()-self.received < .25)
        if active != self.active:
            self.active = active
            self.node.get_logger().info(f'start_interlock active={active} ready={self.ready} camera={self.camera}')
            self.callback(Bool(data=active))
