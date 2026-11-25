#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
export GZ_PARTITION="urrc_practice_$(id -u)"

TEAM="$HOME/urrc_2/src/team/team_05"
export TEAM

cd "$TEAM"

pkill -INT -f '[p]ython3.*monaco_practice.py' || true
pkill -INT -f '[p]ython3.*monaco_drive.py' || true
pkill -INT -f '[p]ython3.*monaco_center_drive.py' || true
pkill -INT -f '[r]os2 launch team05_race' || true
pkill -TERM -f 'parameter_bridge.*[/]model/team_05/cmd_vel' || true

sleep 2

gz service -l | grep -Fx '/world/monaco/create'

# Create a separate practice model with actual pose feedback.
python3 - <<'PY'
import xml.etree.ElementTree as ET
from pathlib import Path
import os

team = Path(os.environ["TEAM"])
src = team / "src/team05_race/models/car/model.sdf"
tree = ET.parse(src)
model = tree.getroot().find("model")

for p in list(model.findall("plugin")):
    if "PosePublisher" in p.get("name", ""):
        model.remove(p)

plugin = ET.SubElement(model, "plugin", {
    "filename": "gz-sim-pose-publisher-system",
    "name": "gz::sim::systems::PosePublisher"
})

for k,v in {
    "publish_model_pose":"true",
    "publish_link_pose":"false",
    "publish_sensor_pose":"false",
    "publish_collision_pose":"false",
    "publish_visual_pose":"false",
    "publish_nested_model_pose":"false",
    "use_pose_vector_msg":"true",
    "update_frequency":"20"
}.items():
    ET.SubElement(plugin,k).text=v

ET.indent(tree)
tree.write(team/"practice_model.sdf",
           encoding="utf-8",
           xml_declaration=True)

print("POSE MODEL READY")
PY

cat > "$TEAM/monaco_center_drive.py" <<'PY'
import csv
import math
import time
import json
import numpy as np
import rclpy

from pathlib import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

TRACK = Path.home() / (
    "urrc_2/src/urrc_track_gazebo/tracks/processed"
)

MAX_SPEED = 1.50
WHEELBASE = 0.28
REAR = WHEELBASE/2
KMAX = 1.75
D = np.arange(.05,1.41,.05)

def clamp(v,a,b):
    return max(a,min(b,v))

def yaw(q):
    return math.atan2(
        2*(q.w*q.z+q.x*q.y),
        1-2*(q.y*q.y+q.z*q.z))

def load_track():
    with open(TRACK/"monaco_centerline.csv") as f:
        reader = csv.DictReader(f)
        keys = {k.strip().lower():k
                for k in reader.fieldnames}
        x = next(keys[k] for k in
                 ["x","x_m","local_x","local_x_m"]
                 if k in keys)
        y = next(keys[k] for k in
                 ["y","y_m","local_y","local_y_m"]
                 if k in keys)
        p = np.array([[float(r[x]),float(r[y])]
                      for r in reader])

    keep = np.r_[True,
        np.linalg.norm(np.diff(p,axis=0),axis=1)>1e-5]
    p = p[keep]

    if np.linalg.norm(p[0]-p[-1])<.01:
        p=p[:-1]

    return p

class Driver(Node):
    def __init__(self):
        super().__init__("team05_center_driver")

        self.track=load_track()
        self.n=len(self.track)

        grid=json.loads(
            (TRACK/"monaco.json").read_text()
        )["grid"][0]

        spawn=np.array([grid["x"],grid["y"]])
        self.spawn=spawn

        i=int(np.argmin(np.linalg.norm(
            self.track-spawn,axis=1)))

        tangent=self.track[(i+3)%self.n]-self.track[i]
        heading=np.array([
            math.cos(grid["yaw"]),
            math.sin(grid["yaw"])])

        if np.dot(tangent,heading)<0:
            self.track=self.track[::-1].copy()

        self.vec=np.roll(self.track,-1,axis=0)-self.track
        self.seg=np.linalg.norm(self.vec,axis=1)
        self.s=np.r_[0.,np.cumsum(self.seg)]
        self.length=self.s[-1]

        self.pos=None
        self.heading=0.
        self.points=None
        self.pose_time=0.
        self.scan_time=0.
        self.last_s=None
        self.progress=0.
        self.speed=0.
        self.prev_k=0.
        self.last_report=0.
        self.finished=False
        self.verified=False

        self.pub=self.create_publisher(
            Twist,"/model/team_05/cmd_vel",10)

        self.create_subscription(
            TFMessage,"/model/team_05/pose",
            self.pose,qos_profile_sensor_data)

        self.create_subscription(
            LaserScan,"/team_05/scan",
            self.scan,qos_profile_sensor_data)

        self.create_timer(.1,self.drive)

        print("CENTERLINE READY",
              round(self.length,1),"m",flush=True)

    def pose(self,msg):
        poses=msg.transforms
        selected=[p for p in poses
                  if p.child_frame_id in
                  ("team_05","/team_05")]

        if selected:
            t=selected[0].transform
        elif len(poses)==1:
            t=poses[0].transform
        else:
            return

        x=t.translation.x
        y=t.translation.y
        h=yaw(t.rotation)

        if not all(math.isfinite(v)
                   for v in [x,y,h]):
            return

        self.pos=np.array([x,y])
        self.heading=h
        self.pose_time=time.monotonic()

    def scan(self,msg):
        r=np.asarray(msg.ranges,dtype=float)

        if len(r)<30:
            self.points=None
            return

        a=msg.angle_min+np.arange(len(r))*msg.angle_increment

        valid=(np.isfinite(r)
               &(r>=msg.range_min)
               &(r<=msg.range_max))

        if np.mean(valid|np.isposinf(r))<.8:
            self.points=None
            return

        p=np.column_stack((
            r[valid]*np.cos(a[valid]),
            r[valid]*np.sin(a[valid])))

        # Ignore only the actual vehicle footprint.
        own=((np.abs(p[:,0])<=.18)
             &(np.abs(p[:,1])<=.108))

        self.points=p[~own]
        self.scan_time=time.monotonic()

    def project(self,p):
        d=p-self.track
        u=np.clip(
            np.sum(d*self.vec,axis=1)
            /np.maximum(self.seg**2,1e-10),0,1)

        q=self.track+self.vec*u[:,None]
        error=np.linalg.norm(p-q,axis=1)
        i=int(np.argmin(error))

        return (
            float(self.s[i]+u[i]*self.seg[i]),
            float(error[i]),i
        )

    def point_at(self,s):
        s=s%self.length
        i=min(self.n-1,
              np.searchsorted(self.s,s,side="right")-1)

        u=(s-self.s[i])/max(self.seg[i],1e-9)
        return self.track[i]+u*self.vec[i]

    def send(self,v,k):
        m=Twist()
        m.linear.x=float(v)
        m.angular.z=float(v*k)
        self.pub.publish(m)

        self.speed=float(v)
        self.prev_k=float(k)

    def path(self,k):
        a=k*D

        if abs(k)<1e-7:
            x=D.copy()
            y=np.zeros_like(D)
        else:
            x=np.sin(a)/k
            y=(1-np.cos(a))/k

        x=x-REAR+REAR*np.cos(a)
        y=y+REAR*np.sin(a)

        return x,y,a

    def clearance(self,x,y,a):
        p=self.points

        if len(p)==0:
            return 1.4

        dx=p[:,0][None,:]-x[:,None]
        dy=p[:,1][None,:]-y[:,None]

        px=np.cos(a[:,None])*dx+np.sin(a[:,None])*dy
        py=-np.sin(a[:,None])*dx+np.cos(a[:,None])*dy

        hit=((px>-.20)&(px<.21)&(np.abs(py)<.128))
        idx=np.flatnonzero(np.any(hit,axis=1))

        return float(max(0,D[idx[0]]-.05)) if len(idx) else 1.4

    def report(self,state,extra=""):
        now=time.monotonic()
        if now-self.last_report>2:
            print(
                state,
                "progress=%.2f%%"%(100*self.progress/self.length),
                "speed=%.2f"%self.speed,
                extra,flush=True)

            self.last_report=now

    def drive(self):
        now=time.monotonic()

        if self.finished:
            self.send(0,0)
            return

        if (self.pos is None or self.points is None
                or now-self.pose_time>.8
                or now-self.scan_time>.8):
            self.send(0,0)
            self.report("WAIT POSE / LIDAR")
            return

        if not self.verified:
            if np.linalg.norm(self.pos-self.spawn)>.8:
                self.send(0,0)
                self.report("POSE COORDINATE ERROR",
                            str(self.pos))
                return
            self.verified=True

        s,cte,i=self.project(self.pos)

        if self.last_s is not None:
            ds=(s-self.last_s+self.length/2)%self.length-self.length/2
            if abs(ds)<1.5:
                self.progress=max(0,self.progress+ds)

        self.last_s=s

        if self.progress>=self.length:
            self.finished=True
            self.send(0,0)
            print("MONACO LAP COMPLETE",
                  flush=True)
            return

        c=math.cos(self.heading)
        sn=math.sin(self.heading)

        rear=self.pos-REAR*np.array([c,sn])

        look=clamp(.85+.4*self.speed,.85,1.15)
        target=self.point_at(s+look)-rear

        tx=c*target[0]+sn*target[1]
        ty=-sn*target[0]+c*target[1]

        if tx<.05:
            self.send(0,0)
            self.report("HEADING ERROR")
            return

        k0=clamp(
            2*ty/max(tx*tx+ty*ty,.05),
            -KMAX,KMAX)

        best=None

        for k in np.unique(np.clip(
                k0+np.array([0,-.2,.2,-.4,.4,
                             -.7,.7,-1.,1.]),
                -KMAX,KMAX)):

            x,y,a=self.path(k)
            free=self.clearance(x,y,a)

            # Keep the predicted path near the centerline.
            j=int(np.argmin(np.abs(D-.75)))

            future=self.pos+np.array([
                c*x[j]-sn*y[j],
                sn*x[j]+c*y[j]])

            _,future_cte,_=self.project(future)

            score=(
                3.5*min(free,1.0)
                -2.0*future_cte
                -1.2*abs(k-k0)
                -.15*abs(k-self.prev_k)
            )

            if free<.20:
                continue

            if best is None or score>best[0]:
                best=(score,float(k),free)

        if best is None:
            self.send(0,0)
            self.report("OBSTACLE STOP",
                        "cte=%.3f"%cte)
            return

        _,k,free=best

        v=min(MAX_SPEED,
              math.sqrt(1.20/max(abs(k),.1)),
              math.sqrt(1.4*max(0,free-.15)),
              self.speed+.08)

        if cte>.45:
            v=min(v,.30)

        self.send(v,k)

        self.report(
            "DRIVING",
            "cte=%.3f steer=%.3f free=%.2f"
            %(cte,k,free))

def main():
    rclpy.init()
    node=Driver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.send(0,0)
            node.destroy_node()
            rclpy.shutdown()

if __name__=="__main__":
    main()
PY

python3 -m py_compile "$TEAM/monaco_center_drive.py"
gz sdf -k "$TEAM/practice_model.sdf"

echo "===== RESET TEAM05 ====="

if gz topic -l | grep -Fxq '/model/team_05/odometry'; then
    gz service \
      -s /world/monaco/remove \
      --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean \
      --timeout 5000 \
      --req 'name: "team_05", type: MODEL'
    sleep 2
fi

ros2 run ros_gz_sim create \
  -world monaco \
  -name team_05 \
  -file "$TEAM/practice_model.sdf" \
  -x 0.454020 \
  -y -2.007622 \
  -z 2.101087 \
  -Y 1.554473

sleep 3

echo "===== POSE CHECK ====="
gz topic -l | grep -Fx '/model/team_05/pose'

ros2 run ros_gz_bridge parameter_bridge \
  '/model/team_05/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/model/team_05/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V' \
  '/team_05/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/team_05/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image' \
  > "$HOME/team05_center_bridge.log" 2>&1 &

BRIDGE=$!

cleanup() {
    gz topic -t /model/team_05/cmd_vel \
      -m gz.msgs.Twist \
      -p 'linear: {x: 0}, angular: {z: 0}' \
      >/dev/null 2>&1 || true
    kill "$BRIDGE" 2>/dev/null || true
}

trap cleanup EXIT

sleep 3

python3 -u "$TEAM/monaco_race_plus.py"
