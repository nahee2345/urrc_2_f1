"""TEAM11 independent replay controller: no world spawning, no map edits."""
from __future__ import annotations
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from team11_race.planner import Planner, RouteMap
from team11_race.perception import LaserTracker
from team11_race.strategy import RaceDirector, Decision
from team11_race.start_policy import choose_start
from team11_race.camera_start import LightTransition, red_evidence
from team11_race.pose_decode import (
    extract_vehicle_pose, pose_frame_examples, yaw_from_quaternion
)
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


class Team11Control(Node):
    def __init__(self):
        super().__init__("team11_controller_11")
        for k,v in (
            ("race_start_mode","signal"),
            ("race_start_topic","/race/start/team_11"),
            ("speed_topic","/team11/speed_11"),
            ("steering_topic","/team11/steering_11"),
            ("max_speed_mps",15.0)):
            self.declare_parameter(k,v)
        self.mode=str(self.get_parameter("race_start_mode").value)
        if self.mode not in ("signal","camera"):
            raise ValueError("Invalid race_start_mode")
        self.allowed=False
        self.camera_go=False
        self.camera_gate=LightTransition()
        self.camera_frames=0
        self.gate_true_at=0.
        self.start_source=""
        self.last_strategy_reason=""
        self.perception=LaserTracker()
        self.director=RaceDirector()
        self.camera_processed=0
        self.pose_frames=0
        self.pose_unrecognized=0
        self.last_tf_wall=0.
        self.odom_frames=0
        self.odom_world_valid=False
        self.odom_rejected=0
        self.last_odom_probe=0.
        self.pose_source=""
        self.pose=None
        self.pose_wall=0.
        self.last_tick=0.
        self.last_discover=0.
        self.track=None
        self.grid=None
        self.planner=None
        self.announced=False
        self.camera_announced=False
        self.last_report=0.
        self.last_health_report=0.
        self.started_at=time.monotonic()
        self.share=share_directory()
        self.speed_pub=self.create_publisher(
            Float32,str(self.get_parameter("speed_topic").value),10)
        self.steer_pub=self.create_publisher(
            Float32,str(self.get_parameter("steering_topic").value),10)
        self.create_subscription(Bool,str(self.get_parameter("race_start_topic").value),
                                 self.gate_callback,10)
        self.create_subscription(TFMessage,"/team11/pose_11",
                                 self.pose_callback,10)
        # Independent Ackermann odometry, accepted as map/world coordinates
        # only after matching a unique bundled start grid.
        self.create_subscription(Odometry,"/team11/odom_11",
                                 self.odom_callback,10)
        self.create_subscription(Image,"/team11/start_image_11",
                                 self.camera_callback,10)
        self.create_subscription(LaserScan,"/team11/front_scan_11",
            lambda msg:self.scan_callback(msg,"front"),10)
        self.create_subscription(LaserScan,"/team11/rear_scan_11",
            lambda msg:self.scan_callback(msg,"rear"),10)
        self.create_timer(.05,self.tick)
        self.get_logger().info("TEAM11 initialized; awaiting own pose, world and race gate")

    def publish(self,speed,steering):
        self.speed_pub.publish(Float32(data=float(speed)))
        self.steer_pub.publish(Float32(data=float(steering)))

    def gate_callback(self,msg):
        previously=self.allowed
        self.allowed=bool(msg.data)
        if self.allowed and not previously:
            self.gate_true_at=time.monotonic()
        if not self.allowed:
            self.publish(0.,0.)
            self.get_logger().info("TEAM11 external race gate FALSE; drive stopped")
        else:
            self.get_logger().info(
                "TEAM11 external race gate TRUE; camera_frames=%s "
                "red_seen=%s all_off=%s pose_frames=%s own_pose=%s map=%s" %
                (self.camera_frames,self.camera_gate.saw_red,self.camera_go,
                 self.pose_frames,self.pose is not None,self.track))

    def pose_callback(self,msg):
        # Gazebo PosePublisher -> private /team11/pose_11. The Pose_V->TF
        # converter may name frames as team_11, team_11::base_link or
        # base_link. Prefer the model world pose over a relative link pose.
        self.pose_frames+=1
        result=extract_vehicle_pose(msg)
        if result is None:
            self.pose_unrecognized+=1
            if self.pose_unrecognized in (1,40,200):
                self.get_logger().warning(
                    "TEAM11 own pose TF frames unrecognized: %s" %
                    (pose_frame_examples(msg),))
            return
        x,y,z,yaw,frame=result
        if self.pose is None:
            self.get_logger().info(
                "TEAM11 own pose ready: frame=%s xyz=(%.3f,%.3f,%.3f) yaw=%.3f" %
                (frame,x,y,z,yaw))
        self.pose=(x,y,z,yaw)
        self.pose_source=frame
        self.pose_wall=time.monotonic()
        self.last_tf_wall=self.pose_wall

    def odom_callback(self,msg):
        self.odom_frames+=1
        now=time.monotonic()
        # Prefer the native own-model world pose whenever it is fresh.
        if now-self.last_tf_wall<0.5:
            return
        p=msg.pose.pose.position
        q=msg.pose.pose.orientation
        values=(float(p.x),float(p.y),float(p.z),
                float(yaw_from_quaternion(q)))
        if not all(math.isfinite(v) for v in values):
            return
        if not self.odom_world_valid:
            if now-self.last_odom_probe<1.0:
                return
            self.last_odom_probe=now
            # An Ackermann odom stream may be local to the wheel origin.
            # Never treat local (0,0,0) as the global world pose.
            if match_map_from_spawn(self.share,values) is None:
                self.odom_rejected+=1
                if self.odom_rejected in (1,5,20):
                    self.get_logger().warning(
                        "TEAM11 odom not world-aligned; ignoring frame=%s "
                        "xyz=(%.3f,%.3f,%.3f)" %
                        (msg.header.frame_id,*values[:3]))
                return
            self.odom_world_valid=True
            self.get_logger().info(
                "TEAM11 independent odom aligned with official start grid; "
                "using as own pose fallback")
        self.pose=values
        self.pose_wall=now
        self.pose_source="self_odom_world_aligned"

    def scan_callback(self,msg,side):
        if self.pose is None:
            return
        scan_now=time.monotonic()
        # Gazebo sensors run at 30Hz each; process at most 15Hz per side
        # to prevent LiDAR callbacks from starving pose and control timers.
        last=self.perception.last_front if side=="front" else self.perception.last_rear
        if scan_now-last<.06:
            return
        try:
            self.perception.update(
                msg.ranges,float(msg.angle_min),float(msg.angle_increment),
                float(msg.range_min),float(msg.range_max),side,self.pose,
                self.planner.speed if self.planner is not None else 0.,
                scan_now)
        except (TypeError,ValueError,OverflowError) as exc:
            self.get_logger().warning(
                "TEAM11 %s LiDAR invalid: %s" % (side,str(exc)))

    def camera_callback(self,msg):
        self.camera_frames+=1
        # 20Hz RGB source -> 10Hz high-resolution detection. Four dark
        # frames still require real successive camera updates.
        if self.camera_frames%2:
            return
        self.camera_processed+=1
        amount=red_evidence(msg.data,int(msg.width),int(msg.height),
                            int(msg.step),str(msg.encoding))
        previous_red=self.camera_gate.saw_red
        self.camera_go=self.camera_gate.update(amount)
        if not previous_red and self.camera_gate.saw_red:
            self.get_logger().info(
                "TEAM11 camera red confirmed: evidence=%d peak=%d" %
                (amount,self.camera_gate.peak))
        if self.camera_go and not self.camera_announced:
            self.get_logger().info(
                "TEAM11 camera LIGHTS OUT confirmed (red->dark stable); "
                "race gate also required")
            self.camera_announced=True

    def report_wait(self,now,reason):
        if now-self.last_health_report<3.0:
            return
        self.last_health_report=now
        self.get_logger().warning(
            "TEAM11 WAIT %s gate=%s camera_mode=%s camera_frames=%d "
            "camera_processed=%d red_seen=%s evidence=%d peak=%d camera_out=%s "
            "pose_messages=%d unmatched_pose=%d odom_frames=%d "
            "odom_world_aligned=%s pose_source=%s pose_fresh=%s "
            "track=%s grid=%s lidar_frames=%d nearby=%d "
            "adjacent_moving=%s start_source=%s strategy=%s" %
            (reason,self.allowed,self.mode,self.camera_frames,
             self.camera_processed,self.camera_gate.saw_red,self.camera_gate.last_red,
             self.camera_gate.peak,self.camera_go,self.pose_frames,
             self.pose_unrecognized,self.odom_frames,
             self.odom_world_valid,self.pose_source,
             self.pose is not None and now-self.pose_wall<=1.,
             self.track,self.grid,self.perception.frames,
             len(self.perception.visible(now)),
             self.perception.neighbor_motion(self.pose,now)
                if self.pose is not None else False,
             self.start_source,self.last_strategy_reason))

    def prepare_route(self,now):
        if self.pose is None or self.planner is not None:
            return
        if now-self.last_discover<1.0:
            return
        self.last_discover=now
        try:
            # Spawn pose uniquely identifies all six official start grids.
            # Match it first: the external 'gz topic -l' subprocess may block
            # the single-threaded rclpy executor and starve camera/pose input.
            result=match_map_from_spawn(self.share,self.pose)
            if result is not None:
                name,slot=result
            else:
                name=discover_world()
                if name is None:
                    self.report_wait(now,"MAP_NOT_IDENTIFIED_FROM_SPAWN")
                    return
                slot=matching_grid(map_assets(self.share,name),self.pose)
            path=map_assets(self.share,name)
            route=RouteMap(path)
            s,_,_=route.nearest(*self.pose[:2])
            lane=min(range(3),key=lambda k:
                     math.hypot(route.sample(k,s)[0]-self.pose[0],
                                route.sample(k,s)[1]-self.pose[1]))
            self.planner=Planner(
                route,lane,max_speed=float(self.get_parameter("max_speed_mps").value),
                external_strategy=True)
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
            self.report_wait(now,"NO_OWN_POSE")
            return
        if self.planner is None:
            self.prepare_route(now)
        permission=choose_start(
            self.mode,self.allowed,self.camera_gate.started,
            self.perception.neighbor_motion(self.pose,now),
            self.perception.neighbor_present(self.pose,now),
            now-self.gate_true_at if self.gate_true_at else 0.)
        if permission is None:
            self.publish(0.,0.)
            self.last_tick=0.
            if self.allowed:
                self.report_wait(now,"CAMERA_OR_NEIGHBOR_NOT_READY")
            return
        if not self.start_source:
            self.start_source=permission
            self.get_logger().info("TEAM11 START source=%s" % permission)
        self.camera_go=True
        if self.planner is None:
            self.publish(0.,0.)
            self.report_wait(now,"MAP_OR_ROUTE_NOT_READY")
            return
        dt=.05 if self.last_tick<=0 else min(.20,now-self.last_tick)
        self.last_tick=now
        decision=Decision(None,float("inf"),"ROUTE_INITIALIZING")
        if self.planner.last_s is not None:
            decision=self.director.choose(
                self.planner,self.pose,self.perception.visible(now),now)
            if decision.lane is not None:
                moved=self.planner.request_lane(decision.lane,decision.reason)
                if moved:
                    self.get_logger().info(
                        "TEAM11 STRATEGY %s lane=%d -> %d threat=%d ttc=%.2f" %
                        (decision.reason,self.planner.lane+1,
                         self.planner.target+1,decision.target_id,decision.ttc))
        self.last_strategy_reason=decision.reason
        speed,steer,status=self.planner.step(
            self.pose,dt,traffic_speed_cap=decision.speed_cap)
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
                f"laps={self.planner.laps} "
                f"strategy={self.last_strategy_reason} "
                f"start={self.start_source} targets={len(self.perception.visible(now))}")
            self.last_report=now


def main():
    rclpy.init()
    node=Team11Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # SIGINT may already have invalidated the ROS context; never publish
        # into a shutdown context (the v11 log's secondary exception).
        if rclpy.ok():
            try:
                node.publish(0.,0.)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=="__main__":
    main()
