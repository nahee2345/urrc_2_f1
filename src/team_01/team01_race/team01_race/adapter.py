"""Hard independent race gate + command timeout: Float32 -> Gazebo Twist."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
from team01_race.command_freshness import fresh_command, command_age


class Adapter(Node):
    def __init__(self):
        super().__init__('team01_adapter')
        for k,v in [('race_start_topic','/race/start/team01'),
                    ('speed_topic','/team01/speed'),('steering_topic','/team01/steering'),
                    ('command_topic','/team01/cmd_vel'),('use_sim_time',True)]:
            if not self.has_parameter(k):self.declare_parameter(k,v)
        self.allowed=False
        self.race_allowed=False
        self.fallback_allowed=False
        self.controller_authorized=False
        self.controller_auth_at=0.
        self.speed=0.
        self.steer=0.
        self.ts=0.
        self.tt=0.
        self.ts_sim=0.
        self.tt_sim=0.
        self.command_skew_limit_s=.12
        self.create_subscription(Bool,self.get_parameter('race_start_topic').value,
                                 self.on_gate,10)
        self.create_subscription(Bool,'/team01/start_fallback',self.on_fallback,10)
        self.create_subscription(Bool,'/team01/start_authorized',
                                 self.on_controller_auth,10)
        self.create_subscription(Float32,self.get_parameter('speed_topic').value,
                                 self.on_speed,10)
        self.create_subscription(Float32,self.get_parameter('steering_topic').value,
                                 self.on_steer,10)
        self.pub=self.create_publisher(Twist,self.get_parameter('command_topic').value,10)
        self.pub_debug=self.create_publisher(String,'/team01/debug/adapter',10)
        self.create_timer(.04,self.tick)

    def on_gate(self,m):
        self.race_allowed=bool(m.data)
        self.allowed=self.race_allowed or self.fallback_allowed
        if not self.allowed:
            self.speed=self.steer=0.
            self.pub.publish(Twist())

    def on_fallback(self,m):
        self.fallback_allowed=bool(m.data)
        self.allowed=self.race_allowed or self.fallback_allowed
        if not self.allowed:
            self.speed=self.steer=0.
            self.pub.publish(Twist())

    def on_controller_auth(self,m):
        self.controller_authorized=bool(m.data)
        self.controller_auth_at=time.monotonic()
        if not self.controller_authorized:
            self.pub.publish(Twist())

    def on_speed(self,m):
        self.speed=max(0.,min(12.,float(m.data))) if math.isfinite(m.data) else 0.
        self.ts=time.monotonic()
        self.ts_sim=self.get_clock().now().nanoseconds*1e-9

    def on_steer(self,m):
        # Race rule steering: +RIGHT, -LEFT. Max centre steer deliberately
        # below 35 degrees because Ackermann INNER tyre can be sharper.
        self.steer=max(-.52,min(.52,float(m.data))) if math.isfinite(m.data) else 0.
        self.tt=time.monotonic()
        self.tt_sim=self.get_clock().now().nanoseconds*1e-9

    def tick(self):
        now=time.monotonic()
        sim_now=self.get_clock().now().nanoseconds*1e-9
        speed_fresh=fresh_command(sim_now,self.ts_sim,now,self.ts)
        steer_fresh=fresh_command(sim_now,self.tt_sim,now,self.tt)
        # Pair velocity and steering from the same control generation window.
        skew=abs(self.ts_sim-self.tt_sim) if sim_now>0 else abs(self.ts-self.tt)
        paired=(self.ts>0 and self.tt>0 and skew<=self.command_skew_limit_s)
        controller_fresh=(self.controller_authorized and
                          self.controller_auth_at>0 and
                          0.<=now-self.controller_auth_at<=.25)
        safe=(self.allowed and controller_fresh and speed_fresh and
              steer_fresh and paired)
        speed_sim_age,speed_wall_age=command_age(sim_now,self.ts_sim,now,self.ts)
        steer_sim_age,steer_wall_age=command_age(sim_now,self.tt_sim,now,self.tt)
        m=Twist()
        if safe:
            m.linear.x=self.speed
            m.angular.z=-self.speed*math.tan(self.steer)/.34
        self.pub.publish(m)
        # ROS command-side evidence: distinguishes camera gate from missing
        # adapter permission, command timeout and Gazebo drivetrain issues.
        self.pub_debug.publish(String(data=json.dumps({
            'allowed':self.allowed,'race_allowed':self.race_allowed,
            'controller_authorized':controller_fresh,
            'fallback_allowed':self.fallback_allowed,
            'fresh_speed':speed_fresh,
            'fresh_steer':steer_fresh,'paired_commands':paired,
            'command_skew_s':round(skew,4),'send_enabled':safe,
            'watchdog_clock':'simulation' if sim_now>0 else 'wall_fallback',
            'speed_sim_age_s':speed_sim_age,
            'steer_sim_age_s':steer_sim_age,
            'speed_wall_age_s':speed_wall_age,
            'steer_wall_age_s':steer_wall_age,
            'stop_reason':('gate' if not self.allowed else
                           'controller_start_not_authorized' if not controller_fresh else
                           'stale_speed' if not speed_fresh else
                           'stale_steering' if not steer_fresh else
                           'command_pair_skew' if not paired else 'none'),
            'received_speed_mps':round(self.speed,3),
            'received_steer_rad':round(self.steer,4),
            'sent_linear_mps':round(float(m.linear.x),3),
            'sent_yaw_rad_s':round(float(m.angular.z),4),
        },separators=(',',':'))))


def main():
    rclpy.init()
    n=Adapter()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.pub.publish(Twist())
        n.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
