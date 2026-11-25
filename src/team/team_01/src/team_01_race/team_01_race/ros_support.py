import math
import copy
import time
from .runtime_clock import RuntimeClock
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from .geometry import vehicle_geometry


SENSOR_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


class ConfigNode(Node):
    def p(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def now_seconds(self):
        if self.get_parameter('use_sim_time').value:
            if not hasattr(self,'_runtime_clock'):self._runtime_clock=RuntimeClock()
            return self._runtime_clock.update(self.get_clock().now().nanoseconds*1e-9,time.monotonic())
        return time.monotonic()

    def simulation_clock_healthy(self,timeout=.25):
        if not self.get_parameter('use_sim_time').value:return True
        self.now_seconds()
        return self._runtime_clock.alive(time.monotonic(),timeout)

    def geometry(self):
        default = str(Path(get_package_share_directory('team_01_race'))/'models/team_01_car/model.sdf')
        from .camera_mount import load_mount
        mount_file=self.p('camera_mount_config',str(Path(get_package_share_directory('team_01_race'))/'config/camera_mount.yaml'))
        return vehicle_geometry(self.p('vehicle_model_file', default),load_mount(mount_file))


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def path_message(points, header):
    msg = PathMsg()
    msg.header = copy.deepcopy(header)
    msg.header.frame_id = 'team_01/base_link'
    if points is None:
        return msg
    for i, (x, y) in enumerate(points):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        if len(points) > 1:
            direction = points[min(i+1, len(points)-1)]-points[max(0, i-1)]
            yaw = math.atan2(float(direction[1]), float(direction[0]))
        else:
            yaw = 0.0
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        msg.poses.append(pose)
    return msg


def path_array(msg):
    if hasattr(msg,'points'):return np.asarray(msg.points,dtype=float).reshape((-1,2))
    return np.asarray([(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=float).reshape((-1, 2))


def run(node_type):
    rclpy.init()
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
