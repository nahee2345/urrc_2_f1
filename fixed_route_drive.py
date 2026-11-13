#!/usr/bin/env python3
"""ROS 2/Gazebo physical test of ONE saved coordinate path from grid 1.

Uses only model odometry and a pre-generated CSV. No lane changes, tactics,
live road detection, or sensor-based path generation.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

BASE=Path(__file__).resolve().parent
WHEELBASE=.36
MAX_STEER=math.pi/6
MAX_YAW_RATE=1.5
SPEED=2.0
MAX_WALL_SECS=1200

def clamp(v,lo,hi):return min(hi,max(lo,v))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class LaneTest(Node):
    def __init__(self,args):
        super().__init__("team11_fixed_route_test")
        self.args=args
        self.route=[{k:float(v) for k,v in r.items()} for r in
                    csv.DictReader((BASE/"routes"/args.track/f"lane_{args.lane}.csv").open())]
        self.meta=json.loads((BASE/"routes"/args.track/"manifest.json").read_text())
        official=json.loads((BASE/"src"/"urrc_track_gazebo"/"tracks"/"processed"/f"{args.track}.json").read_text())
        starting=[p for p in official["grid"] if int(p["position"])==1]
        if len(starting)!=1:raise ValueError("official grid 1 missing")
        p=starting[0]
        self.gx,self.gy,self.gyaw=float(p["x"]),float(p["y"]),float(p["yaw"])
        self.total=self.meta["length_m"]
        self.pub=self.create_publisher(Twist,"/model/team_11/cmd_vel_11",10)
        self.create_subscription(Odometry,"/model/team_11/odometry_11",self.on_odom,10)
        self.create_timer(.05,self.tick)
        self.latest=None
        self.last_odom=time.monotonic()
        self.ready=time.monotonic()
        self.start=None
        self.previous=None
        self.index=None
        self.progress=0.
        self.previous_steer=0.
        self.command_speed=0.
        self.last_tick=time.monotonic()
        self.last_motion=time.monotonic()
        self.result=None
        self.records=[]
        self.log_path=BASE/"log"/"fixed_routes"/f"{args.track}_lane{args.lane}.csv"
        self.log_path.parent.mkdir(parents=True,exist_ok=True)
        self.get_logger().info(f"FIXED ROUTE: {args.track} / lane {args.lane} / grid 1; waiting for lights out")

    def on_odom(self,msg):
        self.latest=msg
        self.last_odom=time.monotonic()

    def stop(self,reason):
        if self.result is None:
            self.result=reason
            self.get_logger().info("ROUTE RESULT: "+reason)
        self.pub.publish(Twist())

    def tick(self):
        if self.result:
            self.pub.publish(Twist())
            return
        now=time.monotonic()
        dt=clamp(now-self.last_tick,.01,.15)
        self.last_tick=now
        if now-self.ready>MAX_WALL_SECS:
            self.stop("FAIL_TIMEOUT")
            return
        if not Path(self.args.start_file).is_file():
            self.pub.publish(Twist())
            return
        if not self.start:
            self.start=now
            self.last_motion=now
        if self.latest is None or now-self.last_odom>.6:
            if now-self.start>8:
                self.stop("FAIL_ODOM_TIMEOUT")
            else:
                self.pub.publish(Twist())
            return
        pose=self.latest.pose.pose
        x_local,y_local=pose.position.x,pose.position.y
        c,s=math.cos(self.gyaw),math.sin(self.gyaw)
        x=self.gx+c*x_local-s*y_local
        y=self.gy+s*x_local+c*y_local
        yaw=wrap(self.gyaw+angle(pose.orientation))
        if not all(map(math.isfinite,(x,y,yaw))):
            self.stop("FAIL_INVALID_POSE");return
        # Use adjacent forward indices after acquisition; never jump to a
        # different piece of track when its geometry crosses nearby.
        n=len(self.route)
        indices=(range(n) if self.index is None else
                 ((self.index+j)%n for j in range(-35,71)))
        nearest=min(indices,key=lambda i:
            (self.route[i]["x_m"]-x)**2+(self.route[i]["y_m"]-y)**2)
        target=self.route[nearest]
        error=math.hypot(target["x_m"]-x,target["y_m"]-y)
        # Road-width criterion is intentionally not inferred from fixed
        # offset alone. Compare against road centre where available later.
        if error>1.5:
            self.stop("FAIL_ROUTE_LOST");return
        if self.index is not None:
            forward=(nearest-self.index)%n
            if forward>n//2:forward-=n
            if forward>0:
                prev_s=self.route[self.index]["s_m"]
                next_s=self.route[nearest]["s_m"]
                ds=(next_s-prev_s)%self.total
                if ds>2.0:
                    self.stop("FAIL_POSE_JUMP");return
                self.progress+=ds
                self.last_motion=now
        self.index=nearest
        if now-self.last_motion>10 and now-self.start>12:
            self.stop("FAIL_STALLED");return

        self.command_speed=min(SPEED,self.command_speed+1.0*dt)
        lookahead=clamp(1.0+.25*self.command_speed,1.0,2.0)
        i=nearest
        d=0.
        for _ in range(n):
            j=(i+1)%n
            d+=math.hypot(self.route[j]["x_m"]-self.route[i]["x_m"],
                          self.route[j]["y_m"]-self.route[i]["y_m"])
            i=j
            if d>=lookahead:break
        goal=self.route[i]
        dx,dy=goal["x_m"]-x,goal["y_m"]-y
        alpha=wrap(math.atan2(dy,dx)-yaw)
        L=max(.35,math.hypot(dx,dy))
        desired=math.atan2(2*WHEELBASE*math.sin(alpha),L)
        steer=clamp(desired,-MAX_STEER,MAX_STEER)
        steer=clamp(steer,self.previous_steer-1.57*dt,
                    self.previous_steer+1.57*dt)
        yaw_steer_limit=math.atan(MAX_YAW_RATE*WHEELBASE/max(.25,self.command_speed))
        steer=clamp(steer,-min(MAX_STEER,yaw_steer_limit),min(MAX_STEER,yaw_steer_limit))
        self.previous_steer=steer
        output=Twist()
        output.linear.x=self.command_speed
        output.angular.z=self.command_speed*math.tan(steer)/WHEELBASE
        self.pub.publish(output)
        velocity=self.latest.twist.twist.linear
        observed=math.hypot(velocity.x,velocity.y)
        self.records.append((round(now-self.start,3),round(self.progress,3),
                             round(x,5),round(y,5),round(observed,3),
                             round(error,3),round(math.degrees(steer),3),
                             nearest,round(goal["x_m"],5),round(goal["y_m"],5)))
        if len(self.records)%100==0:
            self.get_logger().info(
                f"lane {self.args.lane}: {self.progress:.1f}/{self.total:.1f}m, "
                f"error={error:.2f}m speed={observed:.2f}m/s steer={math.degrees(steer):.1f}deg")
        # Must travel almost all the actual track before lap crossing can PASS.
        if self.progress>=.98*self.total and self.progress<=1.06*self.total:
            if self.progress>=self.total-.8:
                self.stop("PASS_LAP")
        if self.progress>1.06*self.total:self.stop("FAIL_LAP_COUNT")

    def save(self):
        with self.log_path.open("w",newline="") as stream:
            writer=csv.writer(stream)
            writer.writerow(("elapsed_s","progress_m","world_x","world_y",
                             "measured_speed_ms","path_error_m","steering_deg",
                             "nearest_index","target_x","target_y"))
            writer.writerows(self.records)
        info={"track":self.args.track,"lane":self.args.lane,"grid":1,
              "result":self.result or "FAIL_INTERRUPTED",
              "progress_m":round(self.progress,2),"total_m":self.total,
              "max_path_error_m":max((row[5] for row in self.records),default=None),
              "csv":str(self.log_path)}
        print(json.dumps(info,ensure_ascii=False,indent=2),flush=True)
        self.log_path.with_suffix(".json").write_text(
            json.dumps(info,ensure_ascii=False,indent=2)+"\n")
        return info

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--track",required=True)
    parser.add_argument("--lane",required=True,type=int,choices=(1,2,3))
    parser.add_argument("--start-file",required=True)
    args=parser.parse_args()
    rclpy.init()
    node=LaneTest(args)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        node.stop("FAIL_INTERRUPTED")
    finally:
        for _ in range(4):
            node.pub.publish(Twist())
            rclpy.spin_once(node,timeout_sec=.02)
        info=node.save()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if info["result"]=="PASS_LAP" else 1)

if __name__=="__main__":main()
