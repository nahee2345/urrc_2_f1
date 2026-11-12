"""TEAM11 independent replay controller: no world spawning, no map edits."""
from __future__ import annotations
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage

from team11_race.planner import Planner, RouteMap
from team11_race.trackdetect import (TRACKS, discover_world, map_assets,
                                      matching_grid, match_map_from_spawn)


def share_directory():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("team11_race"))
    except (ImportError,LookupError):
        return Path(__file__).resolve().parents[1]


def quaternion_yaw(q):
    return math.atan2(2.*(q.w*q.z+q.x*q.y),
                      1.-2.*(q.y*q.y+q.z*q.z))


def count_red_lamps(image: Image) -> int:
    """Conservative five-column red sampling of the upward-facing start camera.

    Counts red lamp columns, not road pixels. Invalid images return -1.
    Camera geometry may need real Gazebo calibration on the competition PC.
    """
    w,h,step=int(image.width),int(image.height),int(image.step)
    encoding=str(image.encoding).lower()
    if w<200 or h<100 or encoding not in ("rgb8","bgr8") or step<w*3:
        return -1
    buf=image.data
    if len(buf)<step*h:
        return -1
    red_indices=(0,1,2) if encoding=="rgb8" else (2,1,0)
    counts=[0]*5
    start=int(w*.22);end=int(w*.78)
    for y in range(int(h*.18),int(h*.67),5):
        row=y*step
        for x in range(start,end,4):
            i=row+3*x
            r=int(buf[i+red_indices[0]])
            g=int(buf[i+red_indices[1]])
            b=int(buf[i+red_indices[2]])
            if r>=135 and r>g*1.7 and r>b*1.7 and r-g>65:
                idx=min(4,int((x-start)*5/max(1,end-start)))
                counts[idx]+=1
    return sum(v>=7 for v in counts)


class Team11Control(Node):
    def __init__(self):
        super().__init__("team11_controller_11")
        for k,v in (
            ("race_start_mode","signal"),
            ("race_start_topic","/race/start/team_11"),
            ("speed_topic","/team11/speed_11"),
            ("steering_topic","/team11/steering_11"),
            ("max_speed_mps",10.0)):
            self.declare_parameter(k,v)
        self.mode=str(self.get_parameter("race_start_mode").value)
        if self.mode not in ("signal","camera"):
            raise ValueError("Invalid race_start_mode")
        self.allowed=False
        self.camera_go=False
        self.red_frames=0
        self.off_frames=0
        self.saw_red=False
        self.pose=None
        self.pose_wall=0.
        self.last_tick=0.
        self.last_discover=0.
        self.track=None
        self.grid=None
        self.planner=None
        self.announced=False
        self.last_report=0.
        self.share=share_directory()
        self.speed_pub=self.create_publisher(
            Float32,str(self.get_parameter("speed_topic").value),10)
        self.steer_pub=self.create_publisher(
            Float32,str(self.get_parameter("steering_topic").value),10)
        self.create_subscription(Bool,str(self.get_parameter("race_start_topic").value),
                                 self.gate_callback,10)
        self.create_subscription(TFMessage,"/team11/pose_11",
                                 self.pose_callback,10)
        self.create_subscription(Image,"/team11/start_image_11",
                                 self.camera_callback,10)
        self.create_timer(.05,self.tick)
        self.get_logger().info("TEAM11 initialized; awaiting own pose, world and race gate")

    def publish(self,speed,steering):
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steer_pub.publish(Float32(data=float(steering)))

    def gate_callback(self,msg):
        self.allowed=bool(msg.data)
        if not self.allowed:
            self.publish(0.,0.)
            self.get_logger().info("TEAM11 external race gate FALSE; drive stopped")
        else:
            self.get_logger().info("TEAM11 external race gate TRUE")

    def pose_callback(self,msg):
        # Own PosePublisher broadcasts the model and its base_link.
        for tf in msg.transforms:
            child=tf.child_frame_id.lower()
            tail=child.replace("::","/").split("/")[-1]
            if tail not in ("team_11","base_link"):
                continue
            if tail=="base_link" and "team_11" not in child:
                continue
            t=tf.transform.translation
            yaw=quaternion_yaw(tf.transform.rotation)
            values=(float(t.x),float(t.y),float(t.z),yaw)
            if all(math.isfinite(v) for v in values):
                self.pose=values
                self.pose_wall=time.monotonic()
                return

    def camera_callback(self,msg):
        if self.camera_go:
            return
        active=count_red_lamps(msg)
        if active<0:
            return
        if active>=4:
            self.red_frames+=1
            self.off_frames=0
            if self.red_frames>=2:self.saw_red=True
        elif active==0 and self.saw_red:
            self.off_frames+=1
            if self.off_frames>=4:
                self.camera_go=True
                self.get_logger().info("TEAM11 four/five red lights then all off")
        else:
            self.off_frames=0
            if active<4:self.red_frames=0

    def prepare_route(self,now):
        if self.pose is None or self.planner is not None:
            return
        if now-self.last_discover<1.0:
            return
        self.last_discover=now
        try:
            name=discover_world()
            if name is None:
                result=match_map_from_spawn(self.share,self.pose)
                if result is None:return
                name,slot=result
            else:
                slot=matching_grid(map_assets(self.share,name),self.pose)
            path=map_assets(self.share,name)
            route=RouteMap(path)
            s,_,_=route.nearest(*self.pose[:2])
            lane=min(range(3),key=lambda k:
                     math.hypot(route.sample(k,s)[0]-self.pose[0],
                                route.sample(k,s)[1]-self.pose[1]))
            self.planner=Planner(
                route,lane,max_speed=float(self.get_parameter("max_speed_mps").value))
            self.track=name
            self.grid=slot
            self.get_logger().info(f"TEAM11 map={name} grid={slot} initial_route={lane+1}")
        except (OSError,ValueError,KeyError) as exc:
            if now-self.last_report>3.:
                self.get_logger().warning(f"TEAM11 map identification pending: {exc}")
                self.last_report=now

    def tick(self):
        now=time.monotonic()
        if self.pose is None or now-self.pose_wall>1.:
            self.publish(0.,0.)
            return
        if self.planner is None:
            self.prepare_route(now)
        if not self.allowed or (self.mode=="camera" and not self.camera_go):
            self.publish(0.,0.)
            self.last_tick=0.
            return
        if self.planner is None:
            self.publish(0.,0.)
            return
        dt=.05 if self.last_tick<=0 else min(.20,now-self.last_tick)
        self.last_tick=now
        speed,steer,status=self.planner.step(self.pose,dt)
        if status not in ("DRIVE","BAD_DT"):
            self.publish(0.,0.)
            if not self.announced:
                self.get_logger().error(f"TEAM11 SAFE STOP reason={status}")
                self.announced=True
            return
        self.publish(speed,steer)
        if now-self.last_report>1.0:
            self.get_logger().info(
                f"map={self.track} route={self.planner.target+1} "
                f"speed={speed:.2f} steering_right_deg={math.degrees(steer):.1f} "
                f"physical_distance={self.planner.travel:.1f} "
                f"laps={self.planner.laps}")
            self.last_report=now


def main():
    rclpy.init()
    node=Team11Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.,0.)
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
