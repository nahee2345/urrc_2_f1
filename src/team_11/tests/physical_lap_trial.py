#!/usr/bin/env python3
"""Gazebo-only single-vehicle lap assessment; NO signal/camera/start bypass in race.

Intended exclusively for a separately launched practice Gazebo world.
ROS odometry is obtained from the actual Gazebo Ackermann model via ros_gz_bridge.
A PASS requires physical odometry progress, no track escape, and a full lap.
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

from strategy import Planner, Track, Vehicle
from team_11.stage5_core import convert
from team_11.practice_frame import local_to_world, wrap_angle, read_spawn

class LapTrial(Node):
    def __init__(self, args):
        super().__init__("team11_physical_lap_trial")
        self.args=args
        self.track=Track(Path(args.routes)/args.track)
        self.spawn=read_spawn(Path(args.official),args.track,args.grid)
        self.get_logger().info("FRAME odometry=model_local; spawn=(%.3f,%.3f,%.3f)" % self.spawn)
        self.planner=Planner(self.track,spawn_lane=1 if args.grid%2 else 3,wheelbase=.36)
        self.pub=self.create_publisher(Twist,"/model/team_11/cmd_vel_11",10)
        self.sub=self.create_subscription(
            Odometry,"/model/team_11/odometry_11",self.on_odom,10)
        self.timer=self.create_timer(.05,self.tick)
        self.odom=None
        self.last_odom_wall=0.0
        self.first_index=None
        self.previous_index=None
        self.distance=0.0
        self.records=[]
        self.max_speed=0.0
        self.reason=None
        self.started_wall=time.monotonic()
        self.last_progress=self.started_wall
        self.last_info=self.started_wall
        self.seen_odom=False
        self.lap_min=self.track.length*.90
        self.output=Path(args.output)
        self.output.parent.mkdir(parents=True,exist_ok=True)

    def on_odom(self,msg):
        self.odom=msg
        self.last_odom_wall=time.monotonic()
        self.seen_odom=True

    @staticmethod
    def yaw(q):
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def stop(self,reason):
        if self.reason is None:
            self.reason=reason
            self.get_logger().info("TRIAL RESULT "+reason)
        zero=Twist()
        self.pub.publish(zero)

    def tick(self):
        if self.reason is not None:
            self.pub.publish(Twist())
            return
        wall=time.monotonic()
        if wall-self.started_wall>self.args.timeout:
            self.stop("FAIL_TIMEOUT")
            return
        if not self.seen_odom:
            if wall-self.started_wall>20:
                self.stop("FAIL_ODOM_MISSING")
            return
        if wall-self.last_odom_wall>.5:
            self.stop("FAIL_ODOM_STALE")
            return
        msg=self.odom
        pos=msg.pose.pose.position
        if not all(math.isfinite(v) for v in (pos.x,pos.y,pos.z)):
            self.stop("FAIL_BAD_POSE")
            return
        x,y=local_to_world(pos.x,pos.y,*self.spawn)
        idx=self.track.nearest(x,y,self.previous_index)
        centre=self.track.lanes[2][idx]
        offset=math.hypot(x-centre["x_m"],y-centre["y_m"])
        allowed=centre["road_width_m"]*.5 +.30
        if offset>allowed:
            self.stop("FAIL_OFF_TRACK")
            return
        if self.first_index is None:
            self.first_index=idx
            self.previous_index=idx
            self.last_progress=wall
        else:
            forward=self.track.along(self.previous_index,idx)
            # A short backward movement is not one entire completed lap.
            if forward>self.track.length*.5:
                forward=-(self.track.length-forward)
            if abs(forward)>5.0:
                self.stop("FAIL_POSE_JUMP")
                return
            if forward>.01:
                self.distance+=forward
                self.last_progress=wall
            self.previous_index=idx
        yaw=wrap_angle(self.spawn[2]+self.yaw(msg.pose.pose.orientation))
        vx=msg.twist.twist.linear.x
        vy=msg.twist.twist.linear.y
        speed=math.hypot(vx,vy)
        self.max_speed=max(speed,self.max_speed)
        if wall-self.last_progress>12 and wall-self.started_wall>15:
            self.stop("FAIL_STALLED")
            return
        # Physical odometry and control decisions are logged independently.
        # This bypass is ONLY in the separate test executable, not production gate.
        ego=Vehicle(x,y,yaw,speed,wall,"ego")
        command=self.planner.step(ego,[],wall,enabled=True)
        if command.mode=="STOP":
            self.stop("FAIL_CONTROLLER_"+command.reason.replace(" ","_"))
            return
        # Planner steer is left-positive. stage5_core expects right-positive.
        linear,angular=convert(command.speed,-command.steer,True,True,True)
        output=Twist()
        output.linear.x=linear
        output.angular.z=angular
        self.pub.publish(output)
        curvature=self.track.curvature[idx]
        self.records.append((round(wall-self.started_wall,3),
            round(self.distance,3),round(x,4),round(y,4),
            round(speed,3),round(command.speed,3),round(math.degrees(command.steer),3),
            self.planner.target,command.mode,round(curvature,5),round(offset,3)))
        if wall-self.last_info>5:
            self.get_logger().info(
                "PROGRESS %.1fm/%.1fm speed %.2f/%.2f cmd target lane %d" %
                (self.distance,self.track.length,speed,command.speed,self.planner.target))
            self.last_info=wall
        index_gap=min((idx-self.first_index)%self.track.n,
                      (self.first_index-idx)%self.track.n)
        if self.distance>=self.lap_min and index_gap<=6:
            self.stop("PASS_LAP")
        elif self.distance>=self.track.length*1.1:
            self.stop("FAIL_LAP_CROSS_NOT_DETECTED")

    def write_result(self):
        csvpath=self.output.with_suffix(".csv")
        with csvpath.open("w",newline="",encoding="utf-8") as fh:
            writer=csv.writer(fh)
            writer.writerow(("wall_elapsed_s","forward_progress_m","world_x","world_y",
                "actual_speed_ms","command_speed_ms","steer_deg","target_lane",
                "mode","local_curvature","center_offset_m"))
            writer.writerows(self.records)
        corner=[r[4] for r in self.records if r[9]>=.08]
        long_straight=[r[4] for r in self.records if r[9]<.02]
        result={
            "track":self.args.track,"grid":self.args.grid,
            "spawn_world_x_y_yaw":list(self.spawn),
            "odometry_frame_conversion":"model-local odometry transformed using official grid pose",
            "result":self.reason or "FAIL_UNKNOWN",
            "lap_length_m":round(self.track.length,2),
            "progress_m":round(self.distance,2),
            "max_actual_speed_ms":round(self.max_speed,2),
            "corner_actual_speed_min_ms":round(min(corner),2) if corner else None,
            "corner_actual_speed_max_ms":round(max(corner),2) if corner else None,
            "straight_actual_speed_max_ms":round(max(long_straight),2) if long_straight else None,
            "samples":len(self.records),"measurements_csv":str(csvpath),
            "note":"Physical Gazebo odometry; signal lights disabled for practice only"}
        self.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",
                               encoding="utf-8")
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        return result

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--track",default="monza")
    p.add_argument("--grid",type=int,default=1)
    p.add_argument("--routes",required=True)
    p.add_argument("--official",required=True)
    p.add_argument("--timeout",type=float,default=180)
    p.add_argument("--output",required=True)
    args=p.parse_args()
    rclpy.init()
    node=LapTrial(args)
    try:
        while rclpy.ok() and node.reason is None:
            rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        node.stop("FAIL_INTERRUPTED")
    finally:
        # Publish a few stop commands before releasing the vehicle publisher.
        for _ in range(5):
            node.pub.publish(Twist())
            rclpy.spin_once(node,timeout_sec=.03)
        result=node.write_result()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result["result"]=="PASS_LAP" else 1)

if __name__=="__main__":
    main()
