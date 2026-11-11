"""TEAM11 competition controller.

The organizer owns Gazebo world, grid selection, model spawn and /clock.
Driving depends ONLY on this vehicle's front LaserScan, never on world pose,
stored track coordinates, opponent topics, IMU or a timing/scoreboard topic.
RGB frames are processed ONLY to authorize the official start.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import Bool,Float32,String

from .geometry import ThreeLanePlanner
from .signal_vision import SignalGate,decode_rgb


class Team11Controller(Node):
    def __init__(self):
        super().__init__("team11_drive")
        self.declare_parameter("race_start_mode","signal")
        self.declare_parameter("race_start_topic","/race/start/team11")
        self.declare_parameter("speed_topic","/team11/speed")
        self.declare_parameter("steering_topic","/team11/steering")
        self.declare_parameter("lidar_topic","/team11/front_scan")
        self.declare_parameter("use_sim_time",True)
        self.mode=str(self.get_parameter("race_start_mode").value)
        if self.mode not in ("signal","camera"):
            raise ValueError("Invalid official race_start_mode: "+self.mode)
        self.allowed=False
        self.authorized=False
        self.have_scan=False
        self.last_scan_at=0.
        self.last_tick=time.monotonic()
        self.scan=None
        self.gate=SignalGate()
        self.planner=ThreeLanePlanner()
        self.speed_pub=self.create_publisher(
            Float32,self.get_parameter("speed_topic").value,10)
        self.steer_pub=self.create_publisher(
            Float32,self.get_parameter("steering_topic").value,10)
        self.authorized_pub=self.create_publisher(
            Bool,"/team11/start_authorized",10)
        self.status_pub=self.create_publisher(
            String,"/team11/status",10)
        self.create_subscription(
            Bool,self.get_parameter("race_start_topic").value,
            self.on_start,10)
        self.create_subscription(
            LaserScan,self.get_parameter("lidar_topic").value,
            self.on_scan,qos_profile_sensor_data)
        self.create_subscription(
            Image,"/team11/front_rgb",self.on_rgb,
            qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
        self.get_logger().info(
            "TEAM11 official entry: no world/spawn/map/clock; "
            "LiDAR driving, RGB start only, start_mode="+self.mode)

    def on_start(self,msg):
        old=self.allowed
        self.allowed=bool(msg.data)
        if not self.allowed:
            self.authorized=False
            # A false arriving after true is the organizer stop condition.
            if old:
                self.planner.reset()
                self.gate.reset()
                self.have_scan=False
        elif not old:
            self.get_logger().info("Operator true received; camera="+
                                   str(self.gate.green))

    def on_scan(self,msg):
        self.scan=msg
        self.have_scan=True
        self.last_scan_at=time.monotonic()

    def on_rgb(self,msg):
        if self.mode!="camera" or self.gate.green:
            return
        rgb=decode_rgb(msg)
        if rgb is None:
            return
        was_ready=self.gate.green
        self.gate.observe(rgb)
        if not was_ready and self.gate.green:
            self.get_logger().info(
                "Actual onboard camera recognized five red lamps -> all off")

    def tick(self):
        now=time.monotonic()
        dt=max(.001,min(.20,now-self.last_tick))
        self.last_tick=now
        self.authorized=(self.allowed and
                         (self.mode=="signal" or self.gate.green))
        scan_fresh=self.have_scan and now-self.last_scan_at<=.35
        speed=0.
        steering=0.
        state="GATE" if not self.authorized else "STALE_SCAN"
        lane=0
        ahead=0.
        if self.authorized and scan_fresh:
            plan=self.planner.update(
                self.scan.ranges,self.scan.angle_min,
                self.scan.angle_increment,self.scan.range_min,
                self.scan.range_max,dt)
            speed=float(plan.speed)
            steering=float(plan.steer)
            state=plan.state
            lane=plan.lane+1
            ahead=plan.ahead_m
            if not math.isfinite(speed) or not math.isfinite(steering):
                speed=0.
                steering=0.
                state="NONFINITE_STOP"
        else:
            # Never keep a stale target speed across signal loss.
            self.planner.last_speed=0.
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steer_pub.publish(Float32(data=float(steering)))
        self.authorized_pub.publish(
            Bool(data=bool(self.authorized and scan_fresh)))
        # Keep status short enough for independent QA to monitor.
        if not hasattr(self,"next_debug") or now>=self.next_debug:
            self.next_debug=now+1.
            self.status_pub.publish(String(data=json.dumps({
                "status":state,"lane":lane,"speed_target":round(speed,3),
                "steer":round(steering,4),
                "front_m":round(ahead,3) if math.isfinite(ahead) else -1,
                "signal_mode":self.mode,"five_seen":self.gate.seen_five,
                "all_off":self.gate.green,"operator_allowed":self.allowed,
                "scan_fresh":scan_fresh,
            },separators=(",",":"))))


def main():
    rclpy.init()
    node=Team11Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.speed_pub.publish(Float32(data=0.))
        node.authorized_pub.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
